#include <mujoco/mujoco.h>

#include "robot_control/EmbeddedController.hpp"
#include "robot_control/GeneratedGeometry.hpp"
#include "robot_control/ImuStateEstimator.hpp"
#include "robot_hal/MotorFrameCodec.hpp"
#include "robot_hal/MotorMapping.hpp"
#include "robot_control/WholeBodyModel.hpp"
#include "SimActuatorBus.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <cerrno>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

namespace {

using robot_control::ControlCommand;
using robot_control::EmbeddedController;
using robot_control::ImuStateEstimator;
using robot_control::JointArray;
using robot_control::JointId;
using robot_control::MotorCommand;
using robot_control::RobotState;
using robot_control::WholeBodyModel;
namespace motor_frame = robot_control::motor_frame;
namespace motor_mapping = robot_control::motor_mapping;
namespace motor_transport = robot_control::motor_transport;

constexpr std::array<int, 6> kJointQpos = {7, 8, 9, 10, 11, 12};
constexpr std::array<int, 6> kJointQvel = {6, 7, 8, 9, 10, 11};
constexpr std::array<int, 6> kJointSensor = {2, 3, 4, 5, 6, 7};
constexpr std::array<int, 6> kVelocitySensor = {8, 9, 10, 11, 12, 13};
constexpr float kNativeDriveCommandLimitMps = 1.00F;
// Leg impedance is deliberately ANISOTROPIC. The wheel balancer's reduced
// model is a RIGID inverted pendulum, so fore/aft leg compliance breaks the
// very model it balances with - which is why the reference firmware this core
// descends from raised its leg_kp rather than lowering it. But the two leg
// axes are nearly decoupled: the hip is almost pure fore/aft
// (dCOMx/dhip = 0.093 m/rad vs dh/dhip = 0.007, a 13x ratio) while the knee
// carries the height axis (dh/dknee = -0.090). So the hip stays stiff to keep
// the pendulum model honest, and the knee alone is softened to give real
// vertical suspension. Compliance where the ground is; rigidity where the
// balancer looks.
// Task-space impedance, in physical units. The leg behaves as a virtual
// spring whose stiffness differs per AXIS rather than per joint: stiff
// fore/aft so the balancer's rigid-pendulum model stays honest, soft
// vertically so the leg is a real suspension.
//
// Per-joint gains cannot express this. The leg Jacobian is not diagonal -
// the knee carries 61% of the hip's fore/aft authority - so softening the
// knee alone leaks compliance into fore/aft, the COM moves, the balancer
// chases it and the robot drifts. The correct object is the stiffness
// MATRIX K_joint = J^T K_task J, whose off-diagonal term is what keeps the
// two axes independent.
//
// A GDS68 only applies a SCALAR Kp/Kd per joint, so the diagonal goes in the
// MIT frame's Kp fields and the off-diagonal is evaluated host-side and sent
// as T_ff. The cost is that the cross term uses the delayed encoder sample
// rather than the driver's fresh local one; it is a correction on top of the
// diagonal, so the lag costs far less than leaving the coupling out.
constexpr float kLegTaskStiffnessForeAftNPerM = 4618.0F;
// The wheel contact reaction is part of the supported leg load.  800 N/m
// left a 4--5 mm model/contact residual with no restoring authority, so the
// knee slowly collapsed even though the gravity term had the right sign.
constexpr float kLegTaskStiffnessVerticalNPerM = 3000.0F;
// Keep the dance inside the already tested posture envelope.  The runner
// chooses only the next target; EmbeddedController owns each height path.
// Keep differential-wheel demand below the plant's low-torque capture margin.
// The station gate can still reduce this to zero on live capture energy; the
// nominal rate must not consume the wheel authority needed to catch pitch.
constexpr float kDanceYawRateRadS = 0.18F;
constexpr float kDanceHighHeightM = 0.200F;
constexpr float kDanceLowHeightM = 0.155F;
constexpr double kDanceHalfPeriodS = 12.0;
constexpr double kDanceYawRampS = 1.2;
constexpr float kDanceYawSlewRateRadS2 = 0.5F;
// The HEIGHT endpoint flag means that the C2 leg path reached its target;
// it does not mean the body has finished exchanging pitch momentum with the
// wheels.  Require the same sensor-derived station dwell used by the core
// before handing differential-wheel authority back to the presentation task.
constexpr double kDancePostHeightQuietS = 0.50;
// The low endpoint is close to the leg/ground torque envelope. Reserve all
// differential-wheel authority there for pitch capture; the high posture
// retains the full presentation yaw and the height transaction remains live.
constexpr float kDanceLowPostureYawScale = 0.0F;
// The next leg move is released only after the previous path has retired and
// the station has had one bounded settling interval. This is a handoff guard,
// not a phase clock: slow capture naturally stretches the dance.
constexpr double kDanceHeightDwellS = 4.0;
constexpr double kDanceLowHeightDwellS = 1.0;
// The native plant has no identified leg/ground torque margin for indefinite
// repeated height cycles. Complete one low->high presentation cycle, then
// keep the safer high posture while continuing the yaw performance.
constexpr unsigned kDanceLowHeightCycles = 1U;
// A finite yaw budget prevents a presentation turn from consuming the wheel
// capture margin indefinitely.  1.8 rad remains above the regression's
// 1.5-rad presentation requirement; after that the skill holds station.
constexpr float kDanceYawBudgetRad = 1.7F;

const motor_mapping::LegCanBinding* legBindingForNode(std::uint8_t node_id) {
  for (const auto& binding : motor_mapping::kVerifiedLegCanBindings) {
    if (binding.node_id == node_id) return &binding;
  }
  return nullptr;
}

struct Options {
  std::string model;
  double rate_hz = 3000.0;
  double sensor_rate_hz = 500.0;
  double run_for_s = 0.0;
  bool realtime = false;
  bool gds68_feedback = true;
  std::uint16_t command_port = 0;
  unsigned seed = 6814U;
};

double asDouble(const char* text, const char* option) {
  char* end = nullptr;
  const double value = std::strtod(text, &end);
  if (end == text || *end != '\0' || !std::isfinite(value)) {
    throw std::invalid_argument(std::string("invalid ") + option);
  }
  return value;
}

Options parseOptions(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    auto value = [&](const char* name) -> const char* {
      if (i + 1 >= argc) throw std::invalid_argument(std::string("missing ") + name);
      return argv[++i];
    };
    if (arg == "--model") options.model = value("--model");
    else if (arg == "--rate") options.rate_hz = asDouble(value("--rate"), "--rate");
    else if (arg == "--sensor-rate") options.sensor_rate_hz = asDouble(value("--sensor-rate"), "--sensor-rate");
    else if (arg == "--run-for") options.run_for_s = asDouble(value("--run-for"), "--run-for");
    else if (arg == "--seed") options.seed = static_cast<unsigned>(std::stoul(value("--seed")));
    else if (arg == "--no-gds68") options.gds68_feedback = false;
    else if (arg == "--command-port") {
      const auto port = std::stoul(value("--command-port"));
      if (port > 65535U) throw std::invalid_argument("invalid --command-port");
      options.command_port = static_cast<std::uint16_t>(port);
    }
    else if (arg == "--realtime") options.realtime = true;
    else if (arg == "--help") {
      std::cout << "tap_mujoco_native --model FILE [--rate 3000|4000] "
                    "[--sensor-rate 500] [--command-port PORT] [--realtime] [--run-for SEC]\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown option: " + arg);
    }
  }
  if (options.model.empty()) throw std::invalid_argument("--model is required");
  if (!(options.rate_hz >= 1000.0 && options.rate_hz <= 4000.0))
    throw std::invalid_argument("--rate must be in [1000,4000] Hz");
  if (!(options.sensor_rate_hz > 0.0 && options.sensor_rate_hz <= options.rate_hz))
    throw std::invalid_argument("--sensor-rate must be positive and <= --rate");
  if (options.run_for_s < 0.0) throw std::invalid_argument("--run-for must be non-negative");
  return options;
}

struct ImuSample {
  double ready_s = 0.0;
  double stamp_s = 0.0;
  std::array<float, 3> accel{};
  std::array<float, 3> gyro{};
};

struct EncoderSample {
  double ready_s = 0.0;
  double stamp_s = 0.0;
  JointArray position{};
  JointArray velocity{};
  JointArray effort{};
  bool effort_valid = false;
};

class NativeRunner {
 public:
  NativeRunner(const Options& options) : options_(options), rng_(options.seed) {
    char error[1024] = {};
    model_ = mj_loadXML(options_.model.c_str(), nullptr, error, sizeof(error));
    if (model_ == nullptr) throw std::runtime_error(std::string("MuJoCo load failed: ") + error);
    model_->opt.timestep = 1.0 / options_.rate_hz;
    data_ = mj_makeData(model_);
    if (data_ == nullptr) throw std::runtime_error("MuJoCo data allocation failed");
    initializeState();
    configureController();
    configureSensors();
    RobotState initial;
    initial.dt_s = static_cast<float>(model_->opt.timestep);
    for (std::size_t i = 0; i < 6; ++i) {
      initial.joints.position_rad[i] = static_cast<float>(data_->qpos[kJointQpos[i]]);
      initial.joints.velocity_rad_s[i] = static_cast<float>(data_->qvel[kJointQvel[i]]);
    }
    initial.imu.accel_magnitude_m_s2 = 9.81F;
    controller_.reset(initial);
    setNonBlockingStdin();
    openCommandSocket();
    std::cout << "READY|backend=mujoco_native|physics_hz=" << options_.rate_hz
              << "|control_hz=" << options_.rate_hz
              << "|sensor_hz=" << options_.sensor_rate_hz
              << "|ground_truth_to_controller=0"
              << "|gds68_feedback=" << (options_.gds68_feedback ? 1 : 0)
              << "|imu_startup_qualification=1"
              << "|scenario=neutral"
              << std::endl;
  }

  ~NativeRunner() {
    if (command_socket_ >= 0) ::close(command_socket_);
    if (data_) mj_deleteData(data_);
    if (model_) mj_deleteModel(model_);
  }

  int run() {
    const auto wall_start = std::chrono::steady_clock::now();
    while (!quit_) {
      step();
      if (options_.run_for_s > 0.0 && sim_time_s_ >= options_.run_for_s) break;
      if (options_.realtime) {
        const auto deadline = wall_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(sim_time_s_));
        std::this_thread::sleep_until(deadline);
      }
    }
    std::cout << "EXIT|sim_time_s=" << sim_time_s_ << std::endl;
    return 0;
  }

 private:
  static float clampFinite(float value, float lo, float hi) {
    return std::isfinite(value) ? std::clamp(value, lo, hi) : 0.0F;
  }

  static float axleForwardOffset(float pitch, float hip, float knee) {
    constexpr float ax = -0.01410935F;
    constexpr float az = -0.10476742F;
    constexpr float bx = 0.04663076F;
    constexpr float bz = -0.10000000F;
    const float first_x = std::cos(hip) * ax + std::sin(hip) * az;
    const float first_z = -std::sin(hip) * ax + std::cos(hip) * az;
    const float second = hip + knee +
        robot_control::geometry::knee_zero_rebase_rad;
    const float second_x = std::cos(second) * bx + std::sin(second) * bz;
    const float second_z = -std::sin(second) * bx + std::cos(second) * bz;
    const float px = first_x + second_x;
    const float pz = first_z + second_z;
    return std::cos(pitch) * px + std::sin(pitch) * pz;
  }

  static float axleForwardOffsetRate(float pitch, float hip, float knee,
                                     float pitch_rate, float hip_rate,
                                     float knee_rate) {
    constexpr float ax = -0.01410935F;
    constexpr float az = -0.10476742F;
    constexpr float bx = 0.04663076F;
    constexpr float bz = -0.10000000F;
    const float first_x = std::cos(hip) * ax + std::sin(hip) * az;
    const float first_z = -std::sin(hip) * ax + std::cos(hip) * az;
    const float second = hip + knee +
        robot_control::geometry::knee_zero_rebase_rad;
    const float second_x = std::cos(second) * bx + std::sin(second) * bz;
    const float second_z = -std::sin(second) * bx + std::cos(second) * bz;
    const float px = first_x + second_x;
    const float pz = first_z + second_z;
    const float dpx_dhip = first_z + second_z;
    const float dpz_dhip = -first_x - second_x;
    const float dpx_dknee = second_z;
    const float dpz_dknee = -second_x;
    const float c = std::cos(pitch);
    const float s = std::sin(pitch);
    return pitch_rate * (-s * px + c * pz) +
        hip_rate * (c * dpx_dhip + s * dpz_dhip) +
        knee_rate * (c * dpx_dknee + s * dpz_dknee);
  }

  void updateSensorWheelOdometry() {
    if (imu_sequence_ == odometry_imu_sequence_ &&
        encoder_sequence_ == odometry_encoder_sequence_) {
      return;
    }
    const float dt = state_.imu.sample_dt_s > 0.0F &&
            std::isfinite(state_.imu.sample_dt_s)
        ? std::clamp(state_.imu.sample_dt_s, 1.0e-4F, 0.05F)
        : static_cast<float>(model_->opt.timestep);
    if (!odometry_seeded_) {
      odometry_previous_pitch_ = state_.imu.pitch_rad;
      odometry_previous_joints_ = state_.joints.position_rad;
      odometry_imu_sequence_ = imu_sequence_;
      odometry_encoder_sequence_ = encoder_sequence_;
      odometry_seeded_ = true;
      return;
    }

    const float pitch = state_.imu.pitch_rad;
    const float pitch_delta = pitch - odometry_previous_pitch_;
    const std::array<std::size_t, 2> hips = {0, 3};
    const std::array<std::size_t, 2> knees = {1, 4};
    const std::array<std::size_t, 2> wheels = {2, 5};
    for (std::size_t side = 0; side < 2; ++side) {
      const std::size_t hip = hips[side];
      const std::size_t knee = knees[side];
      const std::size_t wheel = wheels[side];
      const float delta_angle = pitch_delta +
          (state_.joints.position_rad[hip] - odometry_previous_joints_[hip]) +
          (state_.joints.position_rad[knee] - odometry_previous_joints_[knee]) +
          (state_.joints.position_rad[wheel] -
           odometry_previous_joints_[wheel]);
      const float current_offset = axleForwardOffset(
          pitch, state_.joints.position_rad[hip],
          state_.joints.position_rad[knee]);
      const float previous_offset = axleForwardOffset(
          odometry_previous_pitch_, odometry_previous_joints_[hip],
          odometry_previous_joints_[knee]);
      const float displacement = robot_control::geometry::wheel_radius_m * delta_angle -
          (current_offset - previous_offset);
      odometry_position_m_[side] += displacement;
      const float raw_velocity = robot_control::geometry::wheel_radius_m *
          (state_.imu.pitch_rate_rad_s +
           state_.joints.velocity_rad_s[hip] +
           state_.joints.velocity_rad_s[knee] +
           state_.joints.velocity_rad_s[wheel]) -
          axleForwardOffsetRate(
              pitch, state_.joints.position_rad[hip],
              state_.joints.position_rad[knee], state_.imu.pitch_rate_rad_s,
              state_.joints.velocity_rad_s[hip],
              state_.joints.velocity_rad_s[knee]);
      // Position odometry remains the station anchor.  Derive its velocity
      // from the six encoder rates and the MPU gyro instead of differentiating
      // the 0.0061-rad wheel position packets, which injects a fake speed into
      // the fast MPC at every quantisation step.
      constexpr float kWheelVelocityFilterTauS = 0.008F;
      const float alpha = std::clamp(
          dt / (kWheelVelocityFilterTauS + dt), 0.0F, 1.0F);
      if (!odometry_velocity_initialized_) {
        odometry_velocity_filtered_m_s_[side] = raw_velocity * alpha;
      } else {
        odometry_velocity_filtered_m_s_[side] += alpha *
            (raw_velocity - odometry_velocity_filtered_m_s_[side]);
      }
      odometry_velocity_m_s_[side] = odometry_velocity_filtered_m_s_[side];
    }
    odometry_previous_pitch_ = pitch;
    odometry_previous_joints_ = state_.joints.position_rad;
    odometry_velocity_initialized_ = true;
    odometry_imu_sequence_ = imu_sequence_;
    odometry_encoder_sequence_ = encoder_sequence_;
    state_.wheel.left_position_m = odometry_position_m_[0];
    state_.wheel.right_position_m = odometry_position_m_[1];
    state_.wheel.left_velocity_m_s = odometry_velocity_m_s_[0];
    state_.wheel.right_velocity_m_s = odometry_velocity_m_s_[1];
    state_.wheel.position_m = 0.5F *
        (state_.wheel.left_position_m + state_.wheel.right_position_m);
    state_.wheel.velocity_m_s = 0.5F *
        (state_.wheel.left_velocity_m_s + state_.wheel.right_velocity_m_s);
  }

  void initializeState() {
    std::fill(data_->qpos, data_->qpos + model_->nq, 0.0);
    data_->qpos[2] = robot_control::geometry::neutral_spawn_z_m;
    // The CAD assembly zero is the public natural-pose reference.  Disabled
    // lifecycle handling below pauses dynamics until ENABLE, so this exact
    // reference does not fall while the operator/IMU qualification is idle.
    data_->qpos[3] = 1.0;
    data_->qpos[7] = robot_control::geometry::neutral_hip_rad;
    data_->qpos[8] = robot_control::geometry::neutral_knee_rad;
    data_->qpos[10] = robot_control::geometry::neutral_hip_rad;
    data_->qpos[11] = robot_control::geometry::neutral_knee_rad;
    std::fill(data_->qvel, data_->qvel + model_->nv, 0.0);
    // Spawn on the supported natural ENABLE reference. The robot first
    // acquires a valid MPU6500 tilt seed before the operator enables balance.
    mj_forward(model_, data_);
  }

  RobotState sensorResetState() const {
    RobotState state;
    state.dt_s = static_cast<float>(model_->opt.timestep);
    for (std::size_t i = 0; i < 6; ++i) {
      state.joints.position_rad[i] =
          static_cast<float>(data_->qpos[kJointQpos[i]]);
      state.joints.velocity_rad_s[i] =
          static_cast<float>(data_->qvel[kJointQvel[i]]);
    }
    state.imu.accel_magnitude_m_s2 = 9.81F;
    return state;
  }

  void resetSensorPipeline() {
    imu_queue_.clear();
    encoder_queue_.clear();
    next_imu_s_ = sim_time_s_;
    next_encoder_s_ = sim_time_s_;
    next_telemetry_s_ = sim_time_s_;
    last_imu_stamp_s_ = 0.0;
    last_encoder_stamp_s_ = 0.0;
    imu_sequence_ = 0;
    encoder_sequence_ = 0;
    odometry_imu_sequence_ = 0;
    odometry_encoder_sequence_ = 0;
    odometry_seeded_ = false;
    odometry_position_m_.fill(0.0F);
    odometry_velocity_m_s_.fill(0.0F);
    odometry_velocity_filtered_m_s_.fill(0.0F);
    odometry_velocity_initialized_ = false;
    odometry_previous_joints_.fill(0.0F);
    odometry_previous_pitch_ = 0.0F;
    body_accel_x_m_s2_ = 0.0F;
    accel_tilt_x_m_s2_ = 0.0F;
    body_accel_compensation_valid_ = false;
    accel_trust_ = 0.0F;
    raw_accel_.fill(0.0F);
    feedback_.fill(0.0F);
    leg_feedback_position_.fill(0.0F);
    leg_feedback_velocity_.fill(0.0F);
    wheel_feedback_position_previous_.fill(0.0F);
    wheel_feedback_position_unwrapped_.fill(0.0F);
    wheel_feedback_position_seeded_.fill(false);
    feedback_clock_.fill(0.0F);
    imu_bias_accel_.fill(0.0F);
    imu_bias_gyro_.fill(0.0F);
    for (float& value : imu_bias_accel_) value = normal(0.08F);
    for (float& value : imu_bias_gyro_) value = normal(0.008F);
  }

  void resetCommandPipeline() {
    command_ = ControlCommand{};
    command_.target_joint_position_rad = {
        robot_control::geometry::neutral_hip_rad,
        robot_control::geometry::neutral_knee_rad, 0.0F,
        robot_control::geometry::neutral_hip_rad,
        robot_control::geometry::neutral_knee_rad, 0.0F};
    enable_request_ = false;
    disable_request_ = false;
    estop_ = false;
    enabled_ = false;
    leg_command_.fill(0.0F);
    leg_state_.fill(0.0F);
    // RESET drops every bus/driver state along with the command latches
    // above - a real re-power would too, and a stale CLOSED_LOOP/torque
    // target surviving a sim reset would be a false positive nothing else
    // catches.
    motor_transport_.reset();
    dance_active_ = false;
    dance_started_s_ = sim_time_s_;
    dance_direction_ = 1.0F;
    dance_height_target_m_ = kDanceHighHeightM;
    dance_low_height_cycles_ = 0U;
    dance_yaw_budget_rad_ = 0.0F;
    dance_yaw_command_rad_s_ = 0.0F;
    dance_height_request_active_ = false;
    height_request_rejected_ = false;
    dance_last_height_done_s_ = sim_time_s_;
    last_command_s_ = -1.0e9;
    height_request_started_ = false;
    boot_hold_until_s_ = sim_time_s_ + boot_hold_duration_s_;
    disable_hold_until_s_ = sim_time_s_;
  }

  void resetSimulation() {
    // Keep the runner's sensor/control clock monotonic. MuJoCo data time may
    // reset, but estimator timestamps must not jump backwards on RESET.
    mj_resetData(model_, data_);
    initializeState();
    state_ = sensorResetState();
    controller_.reset(state_);
    imu_estimator_.reset();
    resetSensorPipeline();
    resetCommandPipeline();
    seen_ballistic_ = false;
    seen_touchdown_ = false;
    imu_seeded_ = false;
    imu_tilt_valid_ = false;
  }

  void configureController() {
    // The firmware uses a verbatim copy of this profile:
    // robot_control::tunedControllerConfig() (core TunedConfig.cpp). Change
    // both together until this function calls it directly.
    EmbeddedController::Config cfg;
    auto& lqr = cfg.lqr;
    lqr.wheel_radius_m = robot_control::geometry::wheel_radius_m;
    lqr.half_wheel_base_m = 0.5F * robot_control::geometry::axle_spacing_m;
    lqr.left_wheel_sign = 1.0F;
    // The speed actuator is torque-limited at 0.13 N.m. Keep the operator
    // envelope wider than the validated cruise range so the feedback loop can
    // spend speed to catch pitch without violating the physical cap.
    // Stay below the native Moving Speed frame envelope while leaving enough
    // translational authority to catch a pitch transient after the wheel has
    // already crossed the quiet station speed.
    lqr.max_ground_velocity_m_s = 1.50F;
    cfg.wheel_max_torque_nm = 0.13F;
    const float wheel_torque_accel_m_s2 = 2.0F * cfg.wheel_max_torque_nm /
        (lqr.wheel_radius_m * robot_control::geometry::total_mass_kg);
    lqr.max_ground_accel_m_s2 = std::min(1.40F, wheel_torque_accel_m_s2);
    lqr.wheel_ground_mu = 0.8F;
    lqr.wheel_velocity_reference_from_measured = true;
    lqr.target_velocity_slew_rate_m_s2 = 0.30F;
    lqr.target_velocity_jerk_m_s3 = 1.0F;
    // Releasing DRIVE uses a faster but still torque-feasible profile. The
    // 0.65 factor leaves pitch-capture margin below the two-wheel torque cap;
    // both speed and acceleration remain continuous at the command edge.
    const float brake_capacity_m_s2 = std::min(
        wheel_torque_accel_m_s2, lqr.wheel_ground_mu * lqr.gravity_m_s2);
    lqr.stop_velocity_slew_rate_m_s2 = 0.65F * brake_capacity_m_s2;
    lqr.stop_velocity_jerk_m_s3 = 4.0F;
    lqr.target_yaw_slew_rate_rad_s2 = 1.0F;
    lqr.yaw_rate_feedback_kp = 1.0F;
    // Fail safe before the supported floor-side posture observed in the
    // native model (~0.69 rad).
    lqr.fall_pitch_rad = 0.55F;
    // Let the leg-side gimbal absorb the small supported pitch error. The
    // wheel LQR remains the fast catch controller for larger transients.
    lqr.wheel_pitch_feedback_scale = 0.70F;
    lqr.leg_length_m = 0.234F;
    lqr.pendulum_length_per_com_height = 1.077F;
    lqr.min_pendulum_length_m = 0.10F;
    lqr.max_pendulum_length_m = 0.27F;
    // LQR is solved for the fresh 500 Hz sensor interval, not the 3 kHz
    // actuator/cache loop. Gains are rebuilt only when the live COM height
    // changes enough to warrant a new reduced-order model.
    lqr.sampling_period_s = 0.002F;
    // Pitch capture is the primary safety objective.  A former MPC-shaped
    // velocity weight made the LQR brake odometry before it arrested pitch;
    // station return remains an adaptive outer target, not the fast state.
    lqr.position_cost_scale = 100.0F;
    lqr.pitch_cost_scale = 400.0F;
    lqr.velocity_cost_scale = 300.0F;
    lqr.pitch_rate_cost_scale = 40.0F;
    lqr.input_cost_scale = 10.0F;
    lqr.station_position_cost_scale = 200.0F;
    lqr.station_pitch_cost_scale = 400.0F;
    lqr.station_velocity_cost_scale = 300.0F;
    lqr.station_pitch_rate_cost_scale = 40.0F;
    lqr.station_input_cost_scale = 10.0F;
    lqr.station_gain_pitch_start_rad = 0.12F;
    lqr.station_gain_pitch_quiet_rad = 0.05F;
    lqr.station_gain_rate_start_rad_s = 0.60F;
    lqr.station_gain_rate_quiet_rad_s = 0.20F;
    lqr.station_gain_entry_dwell_s = 0.10F;
    lqr.station_gain_transition_s = 0.20F;
    lqr.station_pitch_return_gain_rad_per_m_s = 0.0F;
    lqr.station_pitch_return_max_rad = 0.0F;
    lqr.riccati_iterations = 80U;
    // A zero command is the natural ENABLE balance hold. Keep a local
    // odometry anchor in Balance so small IMU/encoder bias does not look like
    // continuous wheel slip. EmbeddedController releases this anchor whenever
    // a nonzero Drive/Yaw command or a pose transition is active.
    // Height transactions own a local support point. Re-anchor only at their
    // supported endpoint instead of pulling a long dance back to the initial
    // odometry sample after every shape change.
    lqr.position_anchor_hold = true;
    // Do not bleed the integrated wheel reference during a posture move: the
    // 0.13 N.m wheel servo needs the retained catch command to arrest pitch.
    // Position/velocity station feedback still owns the return target.
    lqr.station_velocity_decay_gain_hz = 0.0F;
    // The estimator's corrected rate remains exposed and calibrated, but the
    // current CAD plant profile keeps raw rate in the fast MPC until the
    // corrected-rate adoption trial clears all posture/stop envelopes.
    lqr.use_bias_corrected_pitch_rate = false;
    // The station return is an outer position loop around the same LQR.  A
    // 15 Hz return gain drove the bounded target straight into +/-0.10 m/s
    // on encoder quantisation, then reversed it every few hundred ms.  Keep
    // the return inside the measured wheel damping envelope so the body does
    // not trade a few millimetres of odometry for visible pitch chatter.
    lqr.station_hold_gain_hz = 5.0F;
    lqr.station_hold_yaw_return = true;
    lqr.station_hold_max_v = 0.06F;
    lqr.station_posture_hold_max_v = 0.25F;
    // Keep the integrated station command inside the posture envelope. The
    // live capture schedule still decides when this ceiling is approached;
    // it is distinct from the wider internal MPC catch acceleration limit.
    lqr.station_hold_command_max_v = 0.10F;
    // The station command starts quiet, then expands with the measured capture
    // distance in LqrBalancer. Do not cap a real posture catch at the quiet
    // 0.25 m/s envelope; that leaves the wheel without authority exactly when
    // pitch/velocity evidence says it must brake.
    lqr.station_hold_dynamic_max_v = lqr.station_posture_hold_max_v;
    // Once pitch/rate have been quiet, station return may move the LQR speed
    // reference back toward the local anchor.  Keep that outer reference
    // slower than the wheel pitch-capture loop so the two objectives cannot
    // fight during the first transient.
    lqr.station_hold_command_slew_rate_m_s2 = 0.10F;
    // station_target is fed into the LQR's own velocity_target, so this term
    // does not act as an independent outer damper: it scales the velocity
    // state the DARE already weights. At 5.0 it made x2 = 6*v + 15*pos_err,
    // i.e. ~6x the designed velocity feedback and ~48x K0's position gain,
    // which sat on the 0.25 m/s station clamp and limit-cycled. Keep this
    // small and let position/velocity regulation come from the costs below,
    // where the Riccati solve co-designs it against the pitch gains.
    lqr.station_hold_velocity_damping_gain = 1.00F;
    // Fade the slow station return to 25% inside a 100 mm capture envelope.
    // The fast pitch-capture MPC remains untouched, so this removes quiet
    // encoder chatter without sacrificing recovery authority.
    lqr.station_adaptive_full_capture_distance_m = 0.10F;
    lqr.station_quiet_gain_scale = 0.25F;
    // Capture-point scheduling supplies the slow, height-adaptive position
    // term; the separate damping term remains the high-frequency damper.
    // Keep the capture-point path available to callers, but do not enable it
    // in the native profile until it clears the closed-loop envelope; the
    // plant's provisional friction/torque model currently amplifies it.
    lqr.station_capture_gain = 0.0F;
    // Do not add a second velocity servo around the pitch LQR.  The first
    // closed-loop trial showed that this term uses the stale integrated
    // command, not the measured wheel state, and can reinforce a saturated
    // correction.  Keep it disabled until an actuator-aware reference
    // governor is proven in simulation.
    lqr.station_hold_velocity_servo_gain_hz = 0.0F;
    // Posture transitions must remain in place. The leg motion changes the
    // encoder-derived wheel odometry and COM geometry at the same time; if the
    // station anchor is released, the balance loop can turn that transient
    // into metres of unintended rolling. EmbeddedController re-anchors at the
    // actual endpoint, so this does not pull the robot back after a pose.
    lqr.hold_station_during_pose = true;
    // Station anchoring is adaptive: this is the allowable residual capture
    // distance, not a speed gate.  The core recomputes the required distance
    // from the live pendulum length and sensor state before resetting MPC.
    cfg.station_capture_margin_m = 0.025F;
    cfg.station_capture_filter_period_s = 0.08F;
    cfg.station_settle_dwell_s = 0.35F;
    cfg.posture_settle_dwell_s = 0.25F;
    // A posture path starts only after the wheel capture transient has mostly
    // decayed; otherwise the leg motion and the retained wheel velocity add
    // instead of sharing the same stationary anchor.
    cfg.posture_settle_wheel_velocity_m_s = 0.03F;
    cfg.height_pending_timeout_s = 2.0F;
    cfg.posture_reanchor_settle_s = 0.50F;
    // Riccati iterations are fixed and bounded; no online optimizer or
    // horizon storage remains in the native balance path.
    // Keep the provisional leg actuator envelope at the independently checked
    // 3 N.m value.  A larger clamp hides posture/geometry faults and has no
    // motor-spec evidence behind it.
    cfg.leg_max_torque_nm = 3.0F;
    cfg.leg_command_max_rate_rad_s = 12.0F;
    cfg.posture_abort_blend_s = 0.35F;
    cfg.hip_min_rad = robot_control::geometry::native_hip_min_rad;
    cfg.hip_max_rad = robot_control::geometry::native_hip_max_rad;
    cfg.knee_min_rad = robot_control::geometry::native_knee_min_rad;
    cfg.knee_max_rad = robot_control::geometry::native_knee_max_rad;
    // Height changes are task-space trajectories; the duration scales with
    // joint travel, while the balance loop catches the changing CoM.  Keep the
    // same measured rate in both directions so the GUI does not make one half
    // of the slider appear stuck.
    cfg.pose_change_duration_s = 2.5F;
    cfg.pose_max_joint_velocity_rad_s = 0.18F;
    cfg.pose_max_duration_s = 16.0F;
    cfg.height_min_duration_s = 5.0F;
    cfg.height_max_rate_m_s = 0.012F;
    cfg.height_lowering_rate_m_s = 0.012F;
    cfg.height_max_duration_s = 16.0F;
    cfg.height_capture_slowdown_min_scale = 0.02F;
    // Allow the final extension near the straight-leg branch to finish while
    // LQR is still inside its recoverable pitch envelope.  Pausing at 50 mm
    // froze the path around 0.237 m and then rejected a valid 0.245 m target.
    cfg.height_capture_pause_distance_m = 0.080F;
    cfg.height_capture_abort_s = 8.00F;
    // The fake driver has a steady-state load sag near the straight-leg and
    // natural branches; settle within 25 mm after the quiet-rate dwell.
    cfg.height_reached_tolerance_m = 0.025F;
    // LQR owns measured body pitch through the wheels. WBC owns only the
    // body-frame COM line through the leg Jacobian.
    cfg.whole_body_compensation = false;
    cfg.whole_body_pitch_gain = 1.0F;
    cfg.height_whole_body_compensation = true;
    cfg.height_whole_body_pitch_gain = cfg.whole_body_pitch_gain;
    // The legs are the fast sagittal body suspension. Its dimensionless gains
    // are scheduled from the live COM height inside the stance controller;
    // the wheel MPC remains the capture loop for translation.
    cfg.whole_body_stance.enabled = true;
    // Leg COM trim stays allowed through a fast rolling transient and the
    // bounded pre-fall capture window; the independent fall detector
    // still owns the final stop.
    cfg.whole_body_stance.max_pitch_rate_for_trim_rad_s = 10.0F;
    cfg.whole_body_stance.gravity_m_s2 = cfg.lqr.gravity_m_s2;
    cfg.whole_body_stance.com_position_gain = 3.00F;
    cfg.whole_body_stance.com_velocity_damping_s = 0.02F;
    // Give the legs a slow COM-line position task. The wheel LQR remains the
    // fast body-pitch capture loop; no IMU-pitch-to-leg path exists.
    cfg.whole_body_stance.max_pitch_for_trim_rad = 0.35F;
    cfg.whole_body_stance.roll_position_gain = 0.25F;
    cfg.whole_body_stance.height_position_weight = 1.0F;
    cfg.whole_body_stance.axle_position_weight = 0.10F;
    cfg.whole_body_stance.symmetric_task_damping = 1.0e-4F;
    cfg.whole_body_stance.correction_filter_hz = 4.0F;
    // Let the bounded correction_ set-point remove FK/model COM bias only in
    // settled stationary Balance; HEIGHT owns the leg target while active.
    cfg.whole_body_stance.com_integrator_gain_hz = 2.0F;
    cfg.whole_body_stance.max_correction_rad = 0.12F;
    // Leg COM motion is intentionally slower than the wheel capture loop;
    // otherwise the wheel spends its authority chasing the moving axle.
    cfg.whole_body_stance.max_correction_rate_rad_s = 0.30F;
    cfg.whole_body_stance.com_deadband_m = 0.0003F;
    cfg.whole_body_stance.body_pitch_task_enabled = true;
    cfg.whole_body_stance.body_pitch_task_gain = 0.80F;
    cfg.whole_body_stance.body_pitch_task_weight = 2.0F;
    cfg.whole_body_stance.body_pitch_task_max_axle_error_m = 0.020F;
    // Let the legs supply the accelerating lean geometrically. 1.0 = full
    // h*a/g compensation; the envelope below caps it at ~0.83 m/s2 worth of
    // COM travel, past which the wheels own the rest as before.
    // Legs own the outer speed loop; wheels own balance only. tau is the
    // speed-loop time constant the body shift alone would produce.
    cfg.whole_body_stance.speed_loop_tau_s = 0.55F;
    cfg.whole_body_stance.speed_loop_integral_hz = 0.0F;
    cfg.whole_body_stance.speed_loop_inertia_lead = 1.0F;
    cfg.whole_body_stance.speed_loop_filter_hz = 6.0F;
    cfg.whole_body_stance.speed_loop_max_rad = 0.18F;
    cfg.whole_body_stance.speed_loop_max_rate_rad_s = 1.50F;
    cfg.whole_body_stance.speed_loop_release_pitch_rad = 0.06F;
    cfg.whole_body_stance.speed_loop_release_span_rad = 0.09F;
    cfg.whole_body_stance_owns_static_com = true;
    cfg.whole_body_gravity_feedforward = true;
    cfg.whole_body_gravity_feedforward_scale = 1.0F;
    cfg.whole_body_stance.hip_min_rad = robot_control::geometry::native_hip_min_rad;
    cfg.whole_body_stance.hip_max_rad = robot_control::geometry::native_hip_max_rad;
    cfg.whole_body_stance.knee_min_rad = robot_control::geometry::native_knee_min_rad;
    cfg.whole_body_stance.knee_max_rad = robot_control::geometry::native_knee_max_rad;
    // No landing/recovery motion is compiled in this firmware profile.
    // A fall therefore stops every actuator instead of holding the legs live.
    cfg.keep_legs_active_when_fallen = false;
    cfg.flight_observer.free_fall_debounce_s = 0.025F;
    cfg.flight_observer.support_loss_debounce_s = 0.040F;
    // Use the observer's free-fall threshold instead of classifying ordinary
    // height-transition unloading as support loss. The time debounce still
    // protects genuine flight and keeps this sensor-only.
    cfg.flight_observer.free_fall_accel_threshold_m_s2 = 3.5F;
    cfg.flight_observer.collision_accel_threshold_m_s2 = 18.0F;
    cfg.flight_observer.collision_hold_s = 0.006F;
    cfg.flight_observer.touchdown_debounce_s = 0.006F;
    cfg.flight_observer.encoder_compression_rate_threshold_m_s = 0.08F;
    cfg.flight_observer.encoder_compression_reversal_threshold_m_s = 0.03F;
    cfg.flight_observer.encoder_touchdown_hold_s = 0.006F;
    // Effort frames may still be emitted by the fake GDS68 transport for
    // frame testing, but they are not part of the firmware balance contract.
    cfg.flight_observer.torque_corroboration_enabled = false;
    controller_ = EmbeddedController(cfg);
    command_.target_joint_position_rad = {
        robot_control::geometry::neutral_hip_rad,
        robot_control::geometry::neutral_knee_rad, 0.0F,
        robot_control::geometry::neutral_hip_rad,
        robot_control::geometry::neutral_knee_rad, 0.0F};
    command_.target_body_height_m = robot_control::geometry::neutral_spawn_z_m;
    height_request_started_ = false;
  }

  void configureSensors() {
    // Keep online calibration deliberately slower in the plant bring-up than
    // the isolated estimator test. The native profile has wheel/contact
    // transients that are not present in a static sensor fixture.
    auto imu_config = ImuStateEstimator::Config{};
    imu_config.bias_learning_gain = 0.10F;
    imu_config.bias_innovation_gain_hz = 0.03F;
    // The kinematic dynamic-acceleration trust in ImuStateEstimator scales
    // this gain down during wheel catches and leg motion.
    imu_config.accel_correction_gain_hz = 5.0F;
    // The native actuator model currently saturates the MPC command well
    // before it realizes the requested ground acceleration. Do not feed that
    // open-loop request back into the attitude estimator; the encoder/gyro
    // kinematic IMU velocity is the measured compensation.
    imu_config.commanded_accel_blend = 0.0F;
    imu_estimator_.setConfig(imu_config);
    accel_id_ = mj_name2id(model_, mjOBJ_SENSOR, "mpu6500_accel");
    gyro_id_ = mj_name2id(model_, mjOBJ_SENSOR, "mpu6500_gyro");
    if (accel_id_ < 0 || gyro_id_ < 0) throw std::runtime_error("MJCF missing IMU sensors");
    // Look up named sensors so reordering the MJCF cannot silently swap a side.
    const std::array<const char*, 6> names = {
        "enc_left_hip_pos", "enc_left_knee_pos", "enc_left_wheel_pos",
        "enc_right_hip_pos", "enc_right_knee_pos", "enc_right_wheel_pos"};
    const std::array<const char*, 6> velocities = {
        "enc_left_hip_vel", "enc_left_knee_vel", "enc_left_wheel_vel",
        "enc_right_hip_vel", "enc_right_knee_vel", "enc_right_wheel_vel"};
    for (std::size_t i = 0; i < 6; ++i) {
      pos_sensor_id_[i] = mj_name2id(model_, mjOBJ_SENSOR, names[i]);
      vel_sensor_id_[i] = mj_name2id(model_, mjOBJ_SENSOR, velocities[i]);
      if (pos_sensor_id_[i] < 0 || vel_sensor_id_[i] < 0)
        throw std::runtime_error("MJCF missing named encoder sensor");
    }
    imu_bias_accel_.fill(0.0F);
    imu_bias_gyro_.fill(0.0F);
    for (float& value : imu_bias_accel_) value = normal(0.08F);
    for (float& value : imu_bias_gyro_) value = normal(0.008F);
  }

  float normal(float sigma) { return static_cast<float>(normal_dist_(rng_) * sigma); }

  static float quantize(float value, float quantum) {
    return quantum > 0.0F ? std::round(value / quantum) * quantum : value;
  }

  float sensorValue(int id, int component = 0) const {
    return static_cast<float>(data_->sensordata[model_->sensor_adr[id] + component]);
  }

  void enqueueSensors() {
    const double imu_period = 1.0 / options_.sensor_rate_hz;
    const double enc_period = 1.0 / options_.sensor_rate_hz;
    if (sim_time_s_ + 1.0e-12 >= next_imu_s_) {
      next_imu_s_ += imu_period;
      ImuSample sample;
      sample.stamp_s = sim_time_s_;
      sample.ready_s = sim_time_s_ + 0.002;
      const double dt = imu_period;
      // A paused desktop session represents a robot physically supported at
      // boot.  MuJoCo's accelerometer reports zero for a completely paused
      // free body, although a real stationary MPU6500 reports +1 g.  Supply
      // only that sensor-level startup fixture until the estimator is seeded;
      // no pose, quaternion, contact or ground-truth quantity enters the
      // controller.  Once seeded, packets always come from the live model.
      const bool startup_sensor_fixture = !imu_seeded_;
      for (int i = 0; i < 3; ++i) {
        imu_bias_accel_[i] += normal(0.012F * static_cast<float>(std::sqrt(dt)));
        imu_bias_gyro_[i] += normal(0.0006F * static_cast<float>(std::sqrt(dt)));
        const float fixture_accel = i == 2 ? 9.81F : 0.0F;
        const float accel = startup_sensor_fixture
            ? fixture_accel
            : sensorValue(accel_id_, i);
        const float gyro = startup_sensor_fixture
            ? 0.0F
            : sensorValue(gyro_id_, i);
        const float startup_accel_bias = startup_sensor_fixture
            ? 0.0F
            : imu_bias_accel_[i];
        sample.accel[i] = quantize(
            accel + startup_accel_bias + normal(0.045F), 0.0012F);
        sample.gyro[i] = quantize(gyro + imu_bias_gyro_[i] + normal(0.0025F), 0.00027F);
      }
      if (uniform_(rng_) >= 0.002) imu_queue_.push_back(sample);
    }
    if (sim_time_s_ + 1.0e-12 >= next_encoder_s_) {
      next_encoder_s_ += enc_period;
      EncoderSample sample;
      sample.stamp_s = sim_time_s_;
      sample.ready_s = sim_time_s_ + 0.002;
      motor_transport::CanFrame can_feedback;
      while (motor_transport_.receiveCan(can_feedback)) {
        const std::uint8_t node = static_cast<std::uint8_t>(can_feedback.id >> 5);
        const std::uint8_t command = static_cast<std::uint8_t>(can_feedback.id & 0x1FU);
        if (node < 1 || node > 4 || command != motor_frame::kMitControlCmdId ||
            can_feedback.dlc != can_feedback.data.size()) {
          continue;
        }
        const auto feedback = motor_frame::unpackMitFeedback(can_feedback.data);
        if (feedback.node_id != node) continue;
        const auto* binding = legBindingForNode(node);
        if (binding == nullptr) continue;
        const std::size_t joint = binding->joint_index;
        leg_feedback_position_[joint] = binding->driver_sign * feedback.pos_rad;
        leg_feedback_velocity_[joint] = binding->driver_sign * feedback.vel_rad_s;
        feedback_[joint] = binding->driver_sign * feedback.torque_nm;
      }
      for (std::size_t i = 0; i < 6; ++i) {
        float p = sensorValue(pos_sensor_id_[i]);
        float v = sensorValue(vel_sensor_id_[i]);
        if (i == 0 || i == 1 || i == 3 || i == 4) {
          p = leg_feedback_position_[i];
          v = leg_feedback_velocity_[i];
        } else {
          const std::size_t side = i == 2 ? 0 : 1;
          motor_transport_.setWheelMeasurement(side, p, v);
          motor_transport_.sendUart(
              static_cast<std::uint8_t>(side),
              motor_frame::buildReadPresent(
                  motor_transport_.wheel(side).motorId()));
          motor_frame::DxlFrame response;
          if (!motor_transport_.receiveUart(
                  static_cast<std::uint8_t>(side), response)) {
            continue;
          }
          const auto parsed = motor_frame::parseDxlResponse(
              response.bytes.data(), response.len);
          const auto present = motor_frame::parsePresentPayload(parsed);
          if (present.valid) {
            const float wrapped = motor_frame::wheelRawPositionToRad(
                present.position_raw);
            if (!wheel_feedback_position_seeded_[side]) {
              wheel_feedback_position_unwrapped_[side] = wrapped;
              wheel_feedback_position_seeded_[side] = true;
            } else {
              constexpr float kPi = 3.14159265358979323846F;
              constexpr float kTwoPi = 2.0F * kPi;
              float delta = wrapped - wheel_feedback_position_previous_[side];
              while (delta > kPi) delta -= kTwoPi;
              while (delta < -kPi) delta += kTwoPi;
              wheel_feedback_position_unwrapped_[side] += delta;
            }
            wheel_feedback_position_previous_[side] = wrapped;
            p = wheel_feedback_position_unwrapped_[side];
            v = motor_frame::wheelRawSpeedToRadPerSec(present.speed_raw);
          }
        }
        const float pq = (i == 2 || i == 5) ? 0.0061359F : 0.0003835F;
        sample.position[i] = quantize(p + normal(0.00035F), pq);
        sample.velocity[i] = quantize(v + normal(0.012F), 0.02F);
      }
      sample.effort_valid = options_.gds68_feedback && uniform_(rng_) >= 0.001;
      sample.effort = feedback_;
      if (sample.effort_valid) encoder_queue_.push_back(sample);
      else if (!options_.gds68_feedback || uniform_(rng_) >= 0.001) encoder_queue_.push_back(sample);
    }
  }

  // Rolling-contact kinematics hold only under the sensor-derived support
  // hypothesis; in flight the accelerometer falls back to gyro integration.
  ImuStateEstimator::KinematicVelocity imuKinematicVelocity(float gyro_rate) const {
    ImuStateEstimator::KinematicVelocity out;
    if (!imu_seeded_ || last_encoder_stamp_s_ <= 0.0 ||
        !controller_.flightEstimate().support_contact) {
      return out;
    }
    const float pitch_rate = gyro_rate - state_.imu.gyro_bias_rad_s;
    const auto& q = state_.joints.position_rad;
    const auto& qd = state_.joints.velocity_rad_s;
    for (const std::size_t hip : {std::size_t{0}, std::size_t{3}}) {
      const auto leg = robot_control::WholeBodyModel::imuVelocityThroughLeg(
          state_.imu.pitch_rad, pitch_rate, q[hip], q[hip + 1], qd[hip],
          qd[hip + 1], qd[hip + 2]);
      out.horizontal_m_s += 0.5F * leg.horizontal_m_s;
      out.vertical_m_s += 0.5F * leg.vertical_m_s;
    }
    out.valid = true;
    return out;
  }

  void consumeSensors() {
    while (!imu_queue_.empty() && imu_queue_.front().ready_s <= sim_time_s_ + 1.0e-12) {
      const ImuSample sample = imu_queue_.front();
      imu_queue_.pop_front();
      const float dt = last_imu_stamp_s_ > 0.0 ? static_cast<float>(sample.stamp_s - last_imu_stamp_s_) : 1.0F / static_cast<float>(options_.sensor_rate_hz);
      raw_accel_ = sample.accel;
      raw_gyro_ = sample.gyro;
      const float accel_mag = std::sqrt(
          sample.accel[0] * sample.accel[0] +
          sample.accel[1] * sample.accel[1] +
          sample.accel[2] * sample.accel[2]);
      const auto mode = controller_.mode();
      // The station latch is built from encoder odometry, corrected IMU
      // pitch/rate and support hypothesis in the core. Reuse that sensor-only
      // quiet evidence for gyro calibration; requiring a small instantaneous
      // MPC acceleration made bias adaptation starve during normal station
      // corrections.
      const bool station_quiet_latched =
          controller_.stationCaptureReady() &&
          controller_.stationDwellS() >= 0.25F;
      const bool stance_quiet = mode == robot_control::ControlMode::Balance &&
          std::abs(command_.target_forward_velocity_m_s) < 1.0e-3F &&
          std::abs(command_.target_yaw_rate_rad_s) < 1.0e-3F &&
          std::abs(state_.wheel.velocity_m_s) < 0.05F &&
        // Do not block bias tracking with a raw-rate gate narrower than the
        // sensor's slowly wandering bias. The estimator applies its own
        // corrected-rate and specific-force gates.
        std::abs(sample.gyro[1] - state_.imu.gyro_bias_rad_s) < 0.12F &&
        std::abs(state_.imu.pitch_rate_rad_s) < 0.12F &&
          std::abs(accel_mag - 9.81F) < 0.35F &&
          station_quiet_latched &&
          !controller_.heightMotionActive();
      const bool pose_changing =
          (mode == robot_control::ControlMode::PoseChange ||
           controller_.heightMotionActive()) &&
          !controller_.heightMotionSettling();
      const auto estimate = imu_estimator_.update(sample.accel, sample.gyro, dt,
          static_cast<float>(sample.stamp_s), 0.0F, pose_changing,
          stance_quiet, imuKinematicVelocity(sample.gyro[1]));
      imu_seeded_ = estimate.seeded;
      imu_tilt_valid_ = estimate.accel_tilt_valid;
      if (estimate.valid) {
        state_.imu.sample_dt_s = dt;
        state_.imu.pitch_rad = estimate.pitch_rad;
        state_.imu.pitch_rate_rad_s = estimate.raw_pitch_rate_rad_s;
        state_.imu.pitch_rate_bias_corrected_rad_s = estimate.pitch_rate_rad_s;
        state_.imu.pitch_rate_bias_corrected_valid = estimate.seeded;
        state_.imu.gyro_bias_rad_s = estimate.gyro_bias_rad_s;
        state_.imu.roll_rate_rad_s = estimate.roll_rate_rad_s;
        state_.imu.yaw_rate_rad_s = estimate.yaw_rate_rad_s;
        state_.imu.accel_magnitude_m_s2 = estimate.accel_magnitude_m_s2;
        state_.imu.hard_impact_event = estimate.hard_impact_event;
        body_accel_x_m_s2_ = estimate.body_accel_x_m_s2;
        accel_tilt_x_m_s2_ = estimate.accel_tilt_x_m_s2;
        body_accel_compensation_valid_ = estimate.body_accel_compensation_valid;
        accel_trust_ = estimate.accel_trust;
      }
      ++imu_sequence_;
      state_.imu_sample_sequence = imu_sequence_;
      last_imu_stamp_s_ = sample.stamp_s;
    }
    while (!encoder_queue_.empty() && encoder_queue_.front().ready_s <= sim_time_s_ + 1.0e-12) {
      const EncoderSample sample = encoder_queue_.front();
      encoder_queue_.pop_front();
      state_.joints.position_rad = sample.position;
      state_.joints.velocity_rad_s = sample.velocity;
      state_.joints.effort_nm = sample.effort;
      state_.joints.effort_feedback_valid = sample.effort_valid;
      ++encoder_sequence_;
      state_.encoder_sample_sequence = encoder_sequence_;
      last_encoder_stamp_s_ = sample.stamp_s;
    }
    updateSensorWheelOdometry();
  }

  void updateActuators() {
    const double dt = model_->opt.timestep;
    // Startup hold is keyed to the lifecycle edge, not absolute simulation
    // time.  RESET after a long run gets the same qualification window as the
    // first boot, while the static natural spawn prevents a late ENABLE from
    // inheriting the old pitch-0 gravity impulse.
    const bool boot_hold = !enabled_ && !estop_ &&
        (!disable_request_ || sim_time_s_ < disable_hold_until_s_);
    // Before ENABLE there is no operator-controlled motion request, so keep
    // the natural pose qualified rather than allowing a long GUI startup to
    // turn into a fall.  Once ENABLE arrives, the same hold is time-bounded
    // and hands ownership to the core as soon as it enables motors.
    const bool boot_window = !enable_request_ || sim_time_s_ < boot_hold_until_s_;
    const bool safe = enabled_ && !estop_ && !disable_request_ &&
        sim_time_s_ - last_command_s_ <= 0.10;
    std::array<float, 4> target_leg{};
    if (safe) target_leg = leg_command_;
    else if (boot_hold) {
      target_leg = {};
      const float target_hip = robot_control::geometry::neutral_hip_rad;
      const float target_knee = robot_control::geometry::neutral_knee_rad;
      for (std::size_t side = 0; side < 2; ++side) {
        const int q = side == 0 ? 7 : 10;
        const int qd = side == 0 ? 6 : 9;
        const float boot_hip_q = static_cast<float>(data_->qpos[q]);
        const float boot_knee_q = static_cast<float>(data_->qpos[q + 1]);
        const auto boot_k = legStiffness(boot_hip_q, boot_knee_q);
        const float hip_kd = 2.0F * std::sqrt(std::max(boot_k.hip, 0.0F) *
            robot_control::geometry::leg_hip_inertia_kgm2);
        const float knee_kd = 2.0F * std::sqrt(std::max(boot_k.knee, 0.0F) *
            robot_control::geometry::leg_knee_inertia_kgm2);
        const float boot_hip_err = target_hip - boot_hip_q;
        const float boot_knee_err = target_knee - boot_knee_q;
        target_leg[side * 2] = clampFinite(
            boot_k.hip * boot_hip_err + boot_k.cross * boot_knee_err -
                hip_kd * static_cast<float>(data_->qvel[qd]), -3.0F, 3.0F);
        target_leg[side * 2 + 1] = clampFinite(
            boot_k.knee * boot_knee_err + boot_k.cross * boot_hip_err -
                knee_kd * static_cast<float>(data_->qvel[qd + 1]), -3.0F, 3.0F);
      }
      // Keep the same CAD gravity compensation active during the bounded
      // sensor-qualification hold. Otherwise the robot can sag before the
      // core receives the first ENABLE-owned command and later rely on MPC to
      // recover a posture error that was created by the boot path itself.
      if (state_.floor_contact) {
        const auto left = robot_control::WholeBodyModel::legGravityTorque(
            state_.joints.position_rad[0], state_.joints.position_rad[1]);
        const auto left_body = robot_control::WholeBodyModel::bodySupportTorque(
            state_.joints.position_rad[0], state_.joints.position_rad[1]);
        const auto right = robot_control::WholeBodyModel::legGravityTorque(
            state_.joints.position_rad[3], state_.joints.position_rad[4]);
        const auto right_body = robot_control::WholeBodyModel::bodySupportTorque(
            state_.joints.position_rad[3], state_.joints.position_rad[4]);
        target_leg[0] += left.hip_nm + left_body.hip_nm;
        target_leg[1] += left.knee_nm + left_body.knee_nm;
        target_leg[2] += right.hip_nm + right_body.hip_nm;
        target_leg[3] += right.knee_nm + right_body.knee_nm;
      }
    }
    const float leg_alpha = static_cast<float>(dt / (0.003 + dt));
    for (std::size_t i = 0; i < 4; ++i) {
      leg_state_[i] += leg_alpha * (target_leg[i] - leg_state_[i]);
      const int qvel = kJointQvel[i < 2 ? i : i + 1];
      const float velocity = static_cast<float>(data_->qvel[qvel]);
      const float friction = 0.006F * velocity + (std::abs(velocity) > 0.02F ? std::copysign(0.08F, velocity) : std::clamp(leg_state_[i], -0.08F, 0.08F));
      data_->ctrl[i] = (safe || (boot_hold && boot_window))
          ? std::clamp(leg_state_[i] - friction, -3.0F, 3.0F)
          : 0.0;
      applied_leg_torque_[i] = static_cast<float>(data_->ctrl[i]);
      if (options_.gds68_feedback) {
        const auto& binding = motor_mapping::kVerifiedLegCanBindings[i];
        feedback_clock_[i] += dt;
        if (feedback_clock_[i] >= 0.002) {
          feedback_clock_[i] = std::fmod(feedback_clock_[i], 0.002);
          const int qpos = kJointQpos[binding.joint_index];
          const int qvel = kJointQvel[binding.joint_index];
          motor_transport_.publishLegFeedback(
              i,
              binding.driver_sign * static_cast<float>(data_->qpos[qpos]),
              binding.driver_sign * static_cast<float>(data_->qvel[qvel]),
              binding.driver_sign *
                  std::clamp(leg_state_[i] + feedback_bias_[i] + normal(0.03F),
                             -11.0F, 11.0F));
        }
      }
    }
    // Wheel actuation: the fake BL4820M12 driver (SimActuatorBus.hpp) owns
    // the torque-enable gate, the settle window and the velocity->torque
    // servo. This loop only steps its physics-rate lag model and applies
    // the torque it returns - no separate MPC/yaw torque math belongs here
    // any more, see docs/MOTOR_HAL_PLAN.md section 3 leak 1.
    for (std::size_t i = 0; i < 2; ++i) {
      const std::size_t j = i == 0 ? 2 : 5;
      auto& drv = motor_transport_.wheel(i);
      drv.step(static_cast<float>(dt));
      const float velocity = static_cast<float>(data_->qvel[kJointQvel[j]]);
      const float torque = drv.computeTorqueNm(velocity);
      data_->ctrl[4 + i] = safe ? torque : 0.0;
    }
  }

  // GDS68 leg stiffness/damping, host-computed and shipped as Kp/Kd fields
  // in the MIT frame (GDS68_DRIVER.md §4) - not a free host-side PD any
  // more, see docs/MOTOR_HAL_PLAN.md section 3 leak 2.
  static bool isHipJoint(std::size_t joint) {
    return joint == static_cast<std::size_t>(JointId::LeftHip) ||
        joint == static_cast<std::size_t>(JointId::RightHip);
  }

  // K_joint = J^T diag(k_foreaft, k_vertical) J at the live pose. Returns
  // {k_hip_hip, k_knee_knee, k_cross}.
  struct LegStiffness {
    float hip = 0.0F;
    float knee = 0.0F;
    float cross = 0.0F;
  };
  static LegStiffness legStiffness(float hip_rad, float knee_rad) {
    const auto com = WholeBodyModel::symmetricComOffsetJacobian(hip_rad, knee_rad);
    const auto leg = WholeBodyModel::legHeight(hip_rad, knee_rad);
    const float jx_h = com.d_com_offset_x_d_hip;
    const float jx_k = com.d_com_offset_x_d_knee;
    const float jz_h = leg.d_height_d_hip;
    const float jz_k = leg.d_height_d_knee;
    const float kx = kLegTaskStiffnessForeAftNPerM;
    const float kz = kLegTaskStiffnessVerticalNPerM;
    LegStiffness out;
    out.hip = kx * jx_h * jx_h + kz * jz_h * jz_h;
    out.knee = kx * jx_k * jx_k + kz * jz_k * jz_k;
    out.cross = kx * jx_h * jx_k + kz * jz_h * jz_k;
    return out;
  }
  static float legKpNmPerRad(std::size_t joint, float hip_rad, float knee_rad,
                              float kp_scale) {
    const auto k = legStiffness(hip_rad, knee_rad);
    return (isHipJoint(joint) ? k.hip : k.knee) * kp_scale;
  }
  static float legKdNmPerRadS(std::size_t joint, float hip_rad, float knee_rad,
                               float kd_scale) {
    const float inertia = isHipJoint(joint)
        ? robot_control::geometry::leg_hip_inertia_kgm2
        : robot_control::geometry::leg_knee_inertia_kgm2;
    // Damping stays critical for whatever stiffness that axis carries.
    const float kp = legKpNmPerRad(joint, hip_rad, knee_rad, 1.0F);
    return (2.0F * std::sqrt(std::max(kp, 0.0F) * inertia)) * kd_scale;
  }

  void applyMotorCommand(const MotorCommand& motors) {
    const bool was_enabled = enabled_;
    enabled_ = motors.enable_motors && !estop_;
    // Legs: pack the real MIT_Control frame from the commanded trajectory
    // and hand it to the fake GDS68 driver, which runs the SAME onboard PD
    // formula a real board runs, against the UNDELAYED physics-tick encoder
    // (data_->qpos/qvel) - never the noisy/latent sensor sample the
    // controller itself sees. See docs/MOTOR_HAL_PLAN.md section 3 leak 2
    // and section 4 ("SimGds68 phải mô phỏng những gì").
    for (std::size_t i = 0; i < 4; ++i) {
      const auto& binding = motor_mapping::kVerifiedLegCanBindings[i];
      const std::size_t j = binding.joint_index;
      auto& drv = motor_transport_.leg(i);
      if (enabled_) {
        // Set_Limits equivalent: a one-time bring-up value on real
        // hardware, not a per-tick channel in the MIT frame
        // (docs/MOTOR_HAL_PLAN.md section 3 leak 3). Only latch it from a
        // genuine enabled command - the stale-sensor fallback path calls
        // applyMotorCommand(MotorCommand{}), whose default-constructed
        // torque_nm is 0, which must never be mistaken for a real change of
        // the hardware limit.
        drv.setTorqueLimitNm(std::max(motors.torque_nm[j], 0.1F));
        if (!was_enabled) {
          motor_transport_.sendCan(
              motor_frame::canId(binding.node_id, motor_frame::kSetControllerModeCmdId),
              motor_frame::packSetControllerMode(
                  motor_frame::kControlModePosition,
                  motor_frame::kInputModeMit));
          motor_transport_.sendCan(
              motor_frame::canId(binding.node_id, motor_frame::kSetAxisStateCmdId),
              motor_frame::packSetAxisState(motor_frame::kAxisStateClosedLoop));
        }
        // Live-pose stiffness matrix for this side. The diagonal rides the
        // MIT frame's Kp; the off-diagonal is the coupling the hardware
        // cannot express, so it is evaluated here against the measured pose
        // and added to T_ff.
        const std::size_t hip_j = i < 2 ? static_cast<std::size_t>(JointId::LeftHip)
                                        : static_cast<std::size_t>(JointId::RightHip);
        const std::size_t knee_j = hip_j + 1U;
        const float hip_q = state_.joints.position_rad[hip_j];
        const float knee_q = state_.joints.position_rad[knee_j];
        const auto stiffness = legStiffness(hip_q, knee_q);
        const float other_error = isHipJoint(j)
            ? (motors.position_rad[knee_j] - knee_q)
            : (motors.position_rad[hip_j] - hip_q);
        const float cross_nm = stiffness.cross * other_error * motors.kp_scale[j];
        const auto mit_bytes = motor_frame::packMitCommand(
            binding.driver_sign * motors.position_rad[j],
            binding.driver_sign * motors.velocity_rad_s[j],
            legKpNmPerRad(j, hip_q, knee_q, motors.kp_scale[j]),
            legKdNmPerRadS(j, hip_q, knee_q, motors.kd_scale[j]),
            binding.driver_sign * (motors.feedforward_nm[j] + cross_nm));
        motor_transport_.sendCan(
            motor_frame::canId(binding.node_id, motor_frame::kMitControlCmdId),
            mit_bytes);
      } else {
        // disable = IDLE every tick, not silence: a driver with its own
        // watchdog does not guarantee it stops on silence alone (same
        // reasoning as the wheel Torque Disable path below).
        motor_transport_.sendCan(
            motor_frame::canId(binding.node_id, motor_frame::kSetAxisStateCmdId),
            motor_frame::packSetAxisState(motor_frame::kAxisStateIdle));
      }
      const float driver_torque = drv.computeTorqueNm(
          binding.driver_sign * static_cast<float>(data_->qpos[kJointQpos[j]]),
          binding.driver_sign * static_cast<float>(data_->qvel[kJointQvel[j]]));
      leg_command_[i] = binding.driver_sign * driver_torque;
    }
    // Wheels: the ONLY channel a real BL4820M12 exposes is Moving Speed
    // (WHEEL_MOTOR_PROTOCOL.md §4) - MotorCommand.velocity_rad_s[wheel] is
    // already the MPC's full ground+yaw wheel velocity intent
    // (LqrBalancer, left/right_wheel_velocity_rad_s), so it is the
    // complete Moving Speed target with nothing else to add.
    for (std::size_t i = 0; i < 2; ++i) {
      const std::size_t j = i == 0 ? 2 : 5;
      auto& drv = motor_transport_.wheel(i);
      motor_transport_.sendUart(
          static_cast<std::uint8_t>(i),
          motor_frame::buildTorqueEnable(drv.motorId(), enabled_));
      if (enabled_) {
        const float target = std::clamp(motors.velocity_rad_s[j], -41.887902F, 41.887902F);
        motor_transport_.sendUart(
            static_cast<std::uint8_t>(i),
            motor_frame::buildMovingSpeed(drv.motorId(), target));
      }
    }
    last_command_s_ = sim_time_s_;
  }

  static double smoothStep(double value) {
    value = std::clamp(value, 0.0, 1.0);
    return value * value * (3.0 - 2.0 * value);
  }

  double dancePhaseS() const {
    if (!dance_active_) return 0.0;
    const double cycle = 2.0 * kDanceHalfPeriodS;
    return std::fmod(std::max(0.0, sim_time_s_ - dance_started_s_), cycle);
  }

  float danceHeightTargetM() const {
    if (!dance_active_) return command_.target_body_height_m;
    return dance_height_target_m_;
  }

  void startDance(float direction) {
    dance_active_ = true;
    dance_started_s_ = sim_time_s_;
    dance_direction_ = direction < 0.0F ? -1.0F : 1.0F;
    // Start with the low pose so the skill has a full high-pose presentation
    // window after the first transition. Starting high consumed almost the
    // entire 28 s demo in the initial leg path and left only a short yaw turn.
    dance_height_target_m_ = kDanceLowHeightM;
    dance_height_request_active_ = false;
    height_request_rejected_ = false;
    dance_last_height_done_s_ = sim_time_s_ - kDanceHeightDwellS;
    command_.target_forward_velocity_m_s = 0.0F;
    command_.target_yaw_rate_rad_s = 0.0F;
    dance_yaw_command_rad_s_ = 0.0F;
    command_.request_pose_change = false;
  }

  void stopDance() {
    dance_active_ = false;
    command_.target_forward_velocity_m_s = 0.0F;
    command_.target_yaw_rate_rad_s = 0.0F;
    dance_height_request_active_ = false;
    height_request_rejected_ = false;
  }

  void updateDanceCommand() {
    if (!dance_active_) return;
    command_.target_forward_velocity_m_s = 0.0F;
    // The core retires the HEIGHT transaction at the C2 endpoint. The runner
    // still waits for the explicit settled acknowledgement before starting the
    // next dance phase, so the handoff cannot replay stale wheel intent.
    bool height_busy =
        (command_.request_height_change &&
         (!controller_.heightMotionSettled() || !height_request_started_)) ||
        controller_.heightMotionActive();

    // Do not stream HEIGHT commands: each one would restart the adaptive
    // capture gate. A completed path toggles the target only after the core
    // has acknowledged the endpoint transaction.
    if (dance_height_request_active_ && height_request_rejected_) {
      // Rejection is not completion: keep the same height target and wait for
      // a fresh adaptive capture instead of toggling the skill phase.
      dance_height_request_active_ = false;
      height_request_rejected_ = false;
      dance_last_height_done_s_ = sim_time_s_;
    } else if (dance_height_request_active_ && !height_busy) {
      dance_height_request_active_ = false;
      dance_last_height_done_s_ = sim_time_s_;
      const bool completed_low = dance_height_target_m_ <=
          kDanceLowHeightM + 1.0e-4F;
      if (completed_low) ++dance_low_height_cycles_;
      const bool completed_height_sequence =
          !completed_low && dance_low_height_cycles_ >= kDanceLowHeightCycles;
      dance_height_target_m_ = completed_height_sequence
          ? kDanceHighHeightM
          : (completed_low ? kDanceHighHeightM : kDanceLowHeightM);
    }
    const float target = danceHeightTargetM();
    const double dance_dwell_s = target <= kDanceLowHeightM + 1.0e-4F
        ? kDanceLowHeightDwellS : kDanceHeightDwellS;
    const bool dance_dwell_elapsed =
        sim_time_s_ - dance_last_height_done_s_ >= dance_dwell_s;
    const auto mode = controller_.mode();
    const bool ready = (mode == robot_control::ControlMode::Balance ||
                        mode == robot_control::ControlMode::Drive) &&
        !height_busy &&
        dance_dwell_elapsed &&
        controller_.stationEligible() && controller_.stationCaptureReady();
    if (ready && std::abs(target - command_.target_body_height_m) > 1.0e-4F &&
        setHeightTarget(target)) {
      dance_height_request_active_ = true;
      height_request_rejected_ = false;
      // The request edge and the first leg-path sample share this control tick.
      // Do not let the yaw task overlap that edge; the wheel torque budget is
      // the balance actuator while the leg trajectory establishes its new COM.
      height_busy = true;
    }

    // Yaw may overlap a height path only after the core has acknowledged that
    // transaction and while the same sensor-derived station capture is quiet.
    // Losing capture removes yaw immediately; the height path remains the
    // sole posture owner until the wheel state is safe again.
    // Keep yaw ownership continuous across the HEIGHT request edge. The
    // request flag is cleared for one IPC tick while the core accepts the new
    // path; using it alone steps the yaw target to zero exactly at that edge
    // and injects a differential-wheel impulse into pitch.
    const bool yaw_ready = !height_busy && controller_.heightMotionSettled() &&
        controller_.stationCaptureReady() &&
        controller_.stationDwellS() >= kDancePostHeightQuietS;
    // Yaw is a continuous dance task, not a new command at every height
    // endpoint. Reusing the endpoint dwell clock ramps the differential wheel
    // target back to zero on each pose edge and creates a repeatable pitch
    // impulse; only DANCE START owns the ramp origin.
    const double yaw_elapsed = std::max(0.0, sim_time_s_ - dance_started_s_);
    const float capture_distance = controller_.stationCaptureDistanceM();
    // A turn is presentation-only. Release it before the longitudinal
    // capture envelope is spent; letting yaw consume the full posture margin
    // makes a small wheel-friction asymmetry grow into a body-pitch fall.
    constexpr float kDanceYawReleaseDistanceM = 0.10F;
    const float yaw_capture_scale = std::clamp(
        (kDanceYawReleaseDistanceM - capture_distance) /
            std::max(kDanceYawReleaseDistanceM -
                         controller_.stationCaptureMarginM(), 1.0e-3F),
        0.0F, 1.0F);
    const float posture_yaw_scale =
        target > kDanceLowHeightM + 1.0e-4F
            ? 1.0F : kDanceLowPostureYawScale;
    dance_yaw_budget_rad_ = std::min(
        kDanceYawBudgetRad,
        dance_yaw_budget_rad_ + std::abs(dance_yaw_command_rad_s_) *
            std::max(static_cast<float>(state_.dt_s), 0.0F));
    const bool yaw_budget_available = dance_yaw_budget_rad_ <
        kDanceYawBudgetRad - 1.0e-4F;
    // Yaw is a presentation task, never a reason to spend the remaining
    // capture margin. Let longitudinal balance recover first, then ramp the
    // turn back in automatically on the next quiet station sample.
    const float desired_yaw_rate = dance_direction_ * kDanceYawRateRadS *
        posture_yaw_scale *
        (yaw_ready && yaw_budget_available ? static_cast<float>(smoothStep(
                         yaw_elapsed / kDanceYawRampS)) * yaw_capture_scale
                   : 0.0F);
    const float yaw_step = kDanceYawSlewRateRadS2 *
        std::max(static_cast<float>(state_.dt_s), 1.0e-4F);
    dance_yaw_command_rad_s_ += std::clamp(
        desired_yaw_rate - dance_yaw_command_rad_s_, -yaw_step, yaw_step);
    command_.target_yaw_rate_rad_s = dance_yaw_command_rad_s_;
  }

  void step() {
    state_.dt_s = static_cast<float>(model_->opt.timestep);
    readCommands();
    updateDanceCommand();
    updateActuators();
    // Before ENABLE the desktop model is paused, while the sensor queue above
    // continues with the bounded startup fixture.  After qualification an
    // ENABLE releases dynamics and the core only then owns balance.
    const bool lifecycle_paused =
        (!enable_request_ || estop_ ||
         (disable_request_ && sim_time_s_ >= disable_hold_until_s_));
    if (!lifecycle_paused) mj_step(model_, data_);
    sim_time_s_ += model_->opt.timestep;
    enqueueSensors();
    consumeSensors();
    const bool stale = last_imu_stamp_s_ <= 0.0 || last_encoder_stamp_s_ <= 0.0 ||
        sim_time_s_ - last_imu_stamp_s_ > 0.15 || sim_time_s_ - last_encoder_stamp_s_ > 0.15;
    const bool have_any_sensor = last_imu_stamp_s_ > 0.0 && last_encoder_stamp_s_ > 0.0;
    state_.emergency_stop = estop_ || (have_any_sensor && stale);
    if (!have_any_sensor) {
      // Do not latch the core EmergencyStop merely because the first delayed
      // sensor packet has not arrived.  This is startup qualification, not a
      // watchdog fault; the actuator boot hold remains bounded separately.
      applyMotorCommand(MotorCommand{});
    } else if (!stale) {
      ControlCommand command = command_;
      // Do not let an external ENABLE request arm balance before the
      // MPU6500 attitude estimator has acquired its stationary seed.  During
      // this qualification interval the bounded simulator startup hold may
      // keep the CAD-natural ENABLE reference from rolling, but the control core stays
      // Disabled.  This mirrors the real embedded boot contract and prevents
      // an unobservable zero-pitch estimate from becoming a torque decision.
      command.request_enable = enable_request_ && imu_seeded_;
      command.request_disable = estop_ || disable_request_;
      if (stationary_stop_request_) {
        controller_.requestStationaryStop(
            state_.wheel.position_m, state_.floor_contact);
        stationary_stop_request_ = false;
      }
      const MotorCommand motors = controller_.update(state_, command);
      // Wheel actuation authority travels ONLY through MotorCommand from
      // here on - see docs/MOTOR_HAL_PLAN.md section 3, leak 1. A real
      // BL4820M12 has no register for "ground acceleration"; the only thing
      // that reaches it is a Moving Speed target, so that is the only thing
      // applyMotorCommand may read for wheel intent.
      applyMotorCommand(motors);
      const auto controllerBasePose = [&]() {
        JointArray pose = controller_.lastOutput().position_rad;
        const auto& correction = controller_.wholeBodyStanceCorrection();
        for (const auto joint : {JointId::LeftHip, JointId::LeftKnee,
                                 JointId::RightHip, JointId::RightKnee}) {
          pose[static_cast<std::size_t>(joint)] -=
              correction[static_cast<std::size_t>(joint)];
        }
        return pose;
      };
      // A fresh POSE overrides an in-flight HEIGHT transaction. Without
      // clearing the level here, the old height request would restart as soon
      // as the pose interpolation finished.
      if (command_.request_pose_change && controller_.poseMotionActive()) {
        command_.request_height_change = false;
        height_request_started_ = false;
      }
      // STAND_UP is an edge-triggered recovery request.  If it was sent while
      // the robot was already balancing, do not leave a stale level latched:
      // a later fall must require a fresh operator command.
      if (command_.request_stand_up &&
          controller_.mode() != robot_control::ControlMode::StandingUp &&
          !controller_.standingUp()) {
        command_.request_stand_up = false;
      }
      if (command_.request_height_change &&
          controller_.heightMotionRejected()) {
        std::cout << "ERROR|height_path_rejected="
                  << command_.target_body_height_m << std::endl;
        // The IPC request is level-triggered, but its stored target is the
        // requested endpoint.  Preserve the controller's last safe actuator
        // pose when the adaptive transaction rejects that endpoint; otherwise
        // clearing HEIGHT on the next tick replays an unfinished pose jump.
        command_.target_joint_position_rad = controllerBasePose();
        // Make the rejected transaction retryable from the measured safe pose.
        // Keeping the requested endpoint here makes updateDanceCommand() see
        // no target change and strands the skill at a mid-height pose.
        command_.target_body_height_m = measuredBodyHeight();
        height_request_rejected_ = dance_active_;
        command_.request_height_change = false;
        height_request_started_ = false;
      }
      // POSE is a transaction at this IPC boundary. Retire it after the
      // planner reaches the endpoint so the GUI need not send POSE_DONE.
      if (command_.request_pose_change && !controller_.poseMotionActive()) {
        command_.request_pose_change = false;
      }
      if (command_.request_height_change && controller_.heightMotionActive()) {
        height_request_started_ = true;
      } else if (command_.request_height_change &&
                 controller_.heightMotionSettled() &&
                 !controller_.heightMotionActive() &&
                 (height_request_started_ ||
                  (controller_.heightMotionSettled() &&
                   std::abs(command_.target_body_height_m -
                            controller_.heightMotionTargetM()) <= 1.0e-4F))) {
        // Latch the pose the core actually reached, so the posture hold that
        // follows the transaction is the robot's real pose rather than a
        // pre-solved endpoint it may never have arrived at.
        command_.target_joint_position_rad = controllerBasePose();
        command_.request_height_change = false;
        height_request_started_ = false;
      }
      seen_ballistic_ = seen_ballistic_ || controller_.flightEstimate().ballistic;
      seen_touchdown_ = seen_touchdown_ || controller_.flightEstimate().touchdown_candidate;
    } else {
      ControlCommand command = command_;
      command.request_enable = false;
      command.request_disable = true;
      const MotorCommand motors = controller_.update(state_, command);
      applyMotorCommand(motors);
    }
    if (sim_time_s_ + 1.0e-12 >= next_telemetry_s_) {
      next_telemetry_s_ += 0.02;
      emitTelemetry();
    }
  }

  void readCommands() {
    if (!stdin_closed_) {
      pollfd descriptor{STDIN_FILENO, POLLIN, 0};
      while (poll(&descriptor, 1, 0) > 0 && (descriptor.revents & POLLIN)) {
        char buffer[512];
        const ssize_t count = ::read(STDIN_FILENO, buffer, sizeof(buffer));
        if (count == 0) { stdin_closed_ = true; break; }
        if (count < 0) break;
        input_buffer_.append(buffer, static_cast<std::size_t>(count));
        std::size_t newline = 0;
        while ((newline = input_buffer_.find('\n')) != std::string::npos) {
          const std::string line = input_buffer_.substr(0, newline);
          input_buffer_.erase(0, newline + 1);
          parseCommand(line);
        }
      }
    }
    if (command_socket_ < 0) return;
    for (;;) {
      char packet[512] = {};
      const ssize_t count = ::recv(command_socket_, packet, sizeof(packet) - 1,
                                   MSG_DONTWAIT);
      if (count < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return;
        return;
      }
      if (count == 0) return;
      packet[count] = '\0';
      parseCommand(std::string(packet, static_cast<std::size_t>(count)));
    }
  }

  void openCommandSocket() {
    if (options_.command_port == 0) return;
    command_socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (command_socket_ < 0) throw std::runtime_error("command UDP socket failed");
    int reuse = 1;
    (void)::setsockopt(command_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse,
                       sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(options_.command_port);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::bind(command_socket_, reinterpret_cast<sockaddr*>(&address),
               sizeof(address)) < 0) {
      ::close(command_socket_);
      command_socket_ = -1;
      throw std::runtime_error("command UDP bind failed");
    }
    const int flags = fcntl(command_socket_, F_GETFL, 0);
    if (flags >= 0) fcntl(command_socket_, F_SETFL, flags | O_NONBLOCK);
  }

  void parseCommand(const std::string& line) {
    std::istringstream stream(line);
    std::string op;
    stream >> op;
    if (op == "ENABLE") {
      // DISABLE is a recoverable safe stop. ESTOP remains separately latched.
      disable_request_ = false;
      enable_request_ = true;
      boot_hold_until_s_ = sim_time_s_ + boot_hold_duration_s_;
      disable_hold_until_s_ = sim_time_s_;
    }
    else if (op == "DISABLE") {
      stopDance();
      disable_request_ = true;
      enable_request_ = false;
      boot_hold_until_s_ = sim_time_s_ + disable_hold_duration_s_;
      disable_hold_until_s_ = boot_hold_until_s_;
      command_.request_pose_change = false;
      command_.request_height_change = false;
      height_request_started_ = false;
      command_.request_stand_up = false;
      command_.target_forward_velocity_m_s = 0.0F;
      command_.target_yaw_rate_rad_s = 0.0F;
    }
    else if (op == "ESTOP") {
      stopDance();
      estop_ = true;
      enable_request_ = false;
      boot_hold_until_s_ = sim_time_s_;
      disable_hold_until_s_ = sim_time_s_;
    }
    else if (op == "CLEAR_ESTOP") estop_ = false;
    else if (op == "RESET") {
      resetSimulation();
    } else if (op == "DRIVE") {
      stopDance();
      float forward = 0.0F;
      float yaw = 0.0F;
      stream >> forward >> yaw;
      forward = clampFinite(
          forward, -kNativeDriveCommandLimitMps,
          kNativeDriveCommandLimitMps);
      yaw = clampFinite(yaw, -0.8F, 0.8F);
      const bool motion_requested = std::abs(forward) > 1.0e-3F ||
          std::abs(yaw) > 1.0e-3F;
      const bool continuing_motion =
          std::abs(command_.target_forward_velocity_m_s) > 1.0e-3F ||
          std::abs(command_.target_yaw_rate_rad_s) > 1.0e-3F ||
          command_.request_height_change;
      const bool drive_ready =
          continuing_motion ||
          controller_.mode() == robot_control::ControlMode::Drive ||
          controller_.mode() == robot_control::ControlMode::PoseChange ||
          (controller_.mode() == robot_control::ControlMode::Balance &&
           controller_.stationCaptureReady());
      if (motion_requested && !drive_ready) {
        command_.target_forward_velocity_m_s = 0.0F;
        command_.target_yaw_rate_rad_s = 0.0F;
        std::cout << "DRIVE_REJECTED|reason=station_not_ready" << std::endl;
      } else {
        command_.target_forward_velocity_m_s = forward;
        command_.target_yaw_rate_rad_s = yaw;
      }
    }
    else if (op == "STOP") {
      stopDance();
      command_.target_forward_velocity_m_s = 0.0F;
      command_.target_yaw_rate_rad_s = 0.0F;
      stationary_stop_request_ = true;
    }
    else if (op == "POSE") {
      stopDance();
      stream >> command_.target_joint_position_rad[0] >> command_.target_joint_position_rad[1]
             >> command_.target_joint_position_rad[3] >> command_.target_joint_position_rad[4];
      command_.request_pose_change = true;
    } else if (op == "HEIGHT") {
      stopDance();
      float height_m = 0.0F;
      stream >> height_m;
      if (!setHeightTarget(height_m)) {
        std::cout << "ERROR|height_target_rejected=" << height_m << std::endl;
      } else {
        std::cout << "HEIGHT_ACCEPTED|target=" << height_m << std::endl;
      }
    } else if (op == "POSE_DONE") command_.request_pose_change = false;
    else if (op == "STAND_UP") {
      stopDance();
      command_.request_stand_up = true;
    } else if (op == "DANCE") {
      std::string action;
      stream >> action;
      if (action == "START") {
        float direction = 1.0F;
        stream >> direction;
        startDance(direction);
        std::cout << "DANCE_ACCEPTED|active=1|direction="
                  << dance_direction_ << std::endl;
      } else if (action == "STOP") {
        stopDance();
        std::cout << "DANCE_ACCEPTED|active=0" << std::endl;
      }
    }
    else if (op == "QUIT") quit_ = true;
  }

  bool setHeightTarget(float body_height_m) {
    if (!std::isfinite(body_height_m)) return false;
    // Command parsing can happen before the first delayed encoder packet. Use
    // the documented natural ENABLE branch for validation; the core re-solves
    // from the measured seed when the height transaction starts.
    const float hip_seed = robot_control::geometry::neutral_hip_rad;
    const float knee_seed = robot_control::geometry::neutral_knee_rad;
    const auto pose = WholeBodyModel::solveSymmetricComHeight(
        body_height_m, 0.0F, hip_seed, knee_seed,
        robot_control::geometry::native_hip_min_rad,
        robot_control::geometry::native_hip_max_rad,
        robot_control::geometry::native_knee_min_rad,
        robot_control::geometry::native_knee_max_rad);
    if (!pose.feasible) return false;
    // The solve above is only a feasibility gate. Do not preload its endpoint;
    // the core owns the leg target for the whole transaction and latches the
    // pose actually reached on completion.
    command_.target_body_height_m = body_height_m;
    command_.request_height_change = true;
    height_request_started_ = false;
    return true;
  }

  float measuredBodyHeight() const {
    const auto left = WholeBodyModel::legHeight(
        state_.joints.position_rad[0], state_.joints.position_rad[1]);
    const auto right = WholeBodyModel::legHeight(
        state_.joints.position_rad[3], state_.joints.position_rad[4]);
    const float height = robot_control::geometry::wheel_radius_m -
        0.5F * (left.height_m + right.height_m);
    return std::isfinite(height) ? height : command_.target_body_height_m;
  }

  void emitTelemetry() {
    const auto& flight = controller_.flightEstimate();
    const auto& lqr = controller_.lqrDebugState();
    const auto& output = controller_.lastOutput();
    const auto& whole_body = controller_.wholeBodyState();
    const auto& stance = controller_.wholeBodyStanceState();
    std::cout << "STATE|t=" << sim_time_s_ << "|mode=" << robot_control::toString(controller_.mode())
              << "|pitch=" << state_.imu.pitch_rad << "|pitch_rate=" << state_.imu.pitch_rate_rad_s
              << "|pitch_rate_corrected=" << state_.imu.pitch_rate_bias_corrected_rad_s
              << "|gyro_bias=" << state_.imu.gyro_bias_rad_s
              << "|accel_mag=" << state_.imu.accel_magnitude_m_s2
              << "|imu_seeded=" << imu_seeded_
              << "|imu_tilt_valid=" << imu_tilt_valid_
              << "|raw_accel_x=" << raw_accel_[0]
              << "|raw_accel_z=" << raw_accel_[2]
              << "|body_accel_x=" << body_accel_x_m_s2_
              << "|accel_tilt_x=" << accel_tilt_x_m_s2_
              << "|body_accel_comp_valid=" << body_accel_compensation_valid_
              << "|accel_trust=" << accel_trust_
              << "|raw_gyro_y=" << raw_gyro_[1]
              << "|support_contact=" << flight.support_contact
              << "|support_unloading=" << flight.support_unloading
              << "|support_loss_s=" << flight.support_loss_elapsed_s
              << "|flight_time_s=" << flight.flight_time_s
              << "|free_fall_confidence=" << flight.free_fall_confidence
              << "|collision_confidence=" << flight.collision_confidence
              << "|wheel_position=" << state_.wheel.position_m
              << "|wheel_velocity=" << state_.wheel.velocity_m_s
              << "|wheel_left_position=" << state_.wheel.left_position_m
              << "|wheel_right_position=" << state_.wheel.right_position_m
              << "|joint_lh=" << state_.joints.position_rad[0]
              << "|joint_lk=" << state_.joints.position_rad[1]
              << "|joint_rh=" << state_.joints.position_rad[3]
              << "|joint_rk=" << state_.joints.position_rad[4]
              << "|joint_v_lh=" << state_.joints.velocity_rad_s[0]
              << "|joint_v_lk=" << state_.joints.velocity_rad_s[1]
              << "|joint_v_rh=" << state_.joints.velocity_rad_s[3]
              << "|joint_v_rk=" << state_.joints.velocity_rad_s[4]
              << "|lqr_velocity=" << lqr.commanded_ground_velocity_m_s
              << "|lqr_velocity_reference=" << lqr.lqr_velocity_reference_m_s
              << "|station_velocity_target=" << lqr.station_velocity_target_m_s
              << "|lqr_anchor=" << lqr.target_ground_position_m
              << "|station_hold=" << controller_.stationHoldActive()
              << "|station_capture_distance=" << controller_.stationCaptureDistanceM()
              << "|station_capture_ready=" << controller_.stationCaptureReady()
              << "|station_dwell=" << controller_.stationDwellS()
              << "|station_eligible=" << controller_.stationEligible()
              << "|lqr_accel=" << lqr.optimal_command_m_s
              << "|lqr_adaptive_gain=" << lqr.adaptive_gain_scale
              << "|lqr_station_gain_blend=" << lqr.station_gain_blend
              << "|lqr_capture_distance=" << lqr.capture_distance_m
              << "|lqr_pitch_ref=" << lqr.lqr_pitch_ref_rad
              << "|lqr_pitch_state=" << lqr.lqr_pitch_state_rad
              << "|lqr_effective_length=" << lqr.effective_pendulum_length_m
              << "|wb_com_x=" << whole_body.com_offset_x_m
              << "|wb_com_line_angle=" << whole_body.com_line_angle_rad
              << "|wb_com_error=" << stance.com_error_m
              << "|wb_com_target_pitch=" << stance.com_target_pitch_rad
              << "|leg_ff_lh=" << output.feedforward_nm[0]
              << "|leg_ff_lk=" << output.feedforward_nm[1]
              << "|leg_ff_rh=" << output.feedforward_nm[3]
              << "|leg_ff_rk=" << output.feedforward_nm[4]
              << "|target_lh=" << output.position_rad[0]
              << "|target_lk=" << output.position_rad[1]
              << "|target_rh=" << output.position_rad[3]
              << "|target_rk=" << output.position_rad[4]
              << "|leg_cmd_lh=" << leg_command_[0]
              << "|leg_cmd_lk=" << leg_command_[1]
              << "|leg_cmd_rh=" << leg_command_[2]
              << "|leg_cmd_rk=" << leg_command_[3]
              << "|leg_applied_lh=" << applied_leg_torque_[0]
              << "|leg_applied_lk=" << applied_leg_torque_[1]
              << "|leg_applied_rh=" << applied_leg_torque_[2]
              << "|leg_applied_rk=" << applied_leg_torque_[3]
              << "|wb_axle_dz=" << whole_body.axle_height_difference_m
              << "|wb_stance_active=" << stance.active
              << "|wb_trim=" << stance.symmetric_trim_norm_rad
              << "|wb_diff_trim=" << stance.differential_trim_norm_rad
              << "|wb_speed_com=" << stance.speed_loop_com_m
              << "|wb_speed_hip=" << stance.speed_loop_hip_rad
              << "|wb_speed_lead=" << stance.speed_loop_lead_s
              << "|wb_speed_release=" << stance.speed_loop_release_scale

              << "|lqr_vel_err=" << lqr.leg_velocity_error_m_s
              << "|leg_cmd_rate_limited=" << controller_.legCommandRateLimitedTicks()
              << "|imu_fresh=" << (last_imu_stamp_s_ > 0.0 && sim_time_s_ - last_imu_stamp_s_ <= 0.15)
              << "|enc_fresh=" << (last_encoder_stamp_s_ > 0.0 && sim_time_s_ - last_encoder_stamp_s_ <= 0.15)
              << "|flight=" << robot_control::toString(flight.phase)
              << "|airborne=" << flight.airborne << "|settling=" << flight.support_settling
              << "|touchdown=" << flight.touchdown_candidate
              << "|seen_ballistic=" << seen_ballistic_ << "|seen_touchdown=" << seen_touchdown_
              << "|height_request=" << command_.request_height_change
              << "|height_target=" << command_.target_body_height_m
              << "|body_height=" << measuredBodyHeight()
              << "|height_motion_start=" << controller_.heightMotionStartM()
              << "|height_motion_target=" << controller_.heightMotionTargetM()
              << "|height_motion=" << controller_.heightMotionActive()
              << "|height_settling=" << controller_.heightMotionSettling()
              << "|height_settled=" << controller_.heightMotionSettled()
              << "|height_pending=" << controller_.heightMotionPending()
              << "|height_pending_elapsed=" << controller_.heightPendingElapsedS()
              << "|height_elapsed=" << controller_.heightMotionElapsedS()
              << "|height_duration=" << controller_.heightMotionDurationS()
              << "|height_rejected=" << controller_.heightMotionRejected()
              << "|skill=" << (dance_active_ ? "dance" : "none")
              << "|dance_phase=" << dancePhaseS()
              << "|dance_height_target=" << danceHeightTargetM()
              << "|dance_yaw_rate=" << command_.target_yaw_rate_rad_s
              << "|cmd_lh=" << output.position_rad[0]
              << "|cmd_lk=" << output.position_rad[1]
              << "|cmd_lw=" << output.velocity_rad_s[2]
              << "|cmd_rh=" << output.position_rad[3]
              << "|cmd_rk=" << output.position_rad[4]
              << "|cmd_rw=" << output.velocity_rad_s[5]
              << "|torque_valid=0|fz_l=0|fz_r=0|ground_truth_to_controller=0"
              << "|fall_pitch_limit=" << controller_.fallPitchThresholdRad()
              << "|loop_hz=" << options_.rate_hz << std::endl;
    // VIEW is explicitly debug-only.  The GUI may render it, but this line is
    // never parsed into RobotState or any controller decision.
    std::cout << "VIEW|t=" << sim_time_s_ << "|x=" << data_->qpos[0]
              << "|y=" << data_->qpos[1] << "|z=" << data_->qpos[2]
              << "|qw=" << data_->qpos[3] << "|qx=" << data_->qpos[4] << "|qy=" << data_->qpos[5]
              << "|qz=" << data_->qpos[6] << "|body_height=" << data_->qpos[2]
              << "|lh=" << data_->qpos[7] << "|lk=" << data_->qpos[8]
              << "|lw=" << data_->qpos[9] << "|rh=" << data_->qpos[10] << "|rk=" << data_->qpos[11]
              << "|rw=" << data_->qpos[12] << std::endl;
  }

  void setNonBlockingStdin() {
    const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
  }

  Options options_;
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  EmbeddedController controller_;
  ImuStateEstimator imu_estimator_;
  RobotState state_{};
  ControlCommand command_{};
  std::mt19937 rng_;
  std::normal_distribution<double> normal_dist_{0.0, 1.0};
  std::uniform_real_distribution<double> uniform_{0.0, 1.0};
  std::array<float, 3> imu_bias_accel_{};
  std::array<float, 3> imu_bias_gyro_{};
  std::array<float, 3> raw_accel_{};
  std::array<float, 3> raw_gyro_{};
  std::array<float, 4> feedback_bias_{{0.08F, -0.04F, 0.03F, -0.06F}};
  JointArray feedback_{};
  JointArray leg_feedback_position_{};
  JointArray leg_feedback_velocity_{};
  std::array<float, 4> feedback_clock_{};
  std::array<float, 4> leg_command_{};
  std::array<float, 4> applied_leg_torque_{};
  std::array<float, 4> leg_state_{};
  // Fake drivers standing in for the real GDS68 (legs) / BL4820M12 (wheels)
  // bus hardware; see SimActuatorBus.hpp and docs/MOTOR_HAL_PLAN.md section 4.
  // Node ids/motor ids match the hardware map in that doc section 1.
  robot_control::SimMotorTransport motor_transport_{};
  std::array<float, 2> wheel_feedback_position_previous_{};
  std::array<float, 2> wheel_feedback_position_unwrapped_{};
  std::array<bool, 2> wheel_feedback_position_seeded_{};
  std::deque<ImuSample> imu_queue_;
  std::deque<EncoderSample> encoder_queue_;
  double sim_time_s_ = 0.0;
  double next_imu_s_ = 0.0;
  double next_encoder_s_ = 0.0;
  double next_telemetry_s_ = 0.0;
  double last_imu_stamp_s_ = 0.0;
  double last_encoder_stamp_s_ = 0.0;
  double last_command_s_ = -1.0e9;
  std::uint64_t imu_sequence_ = 0;
  std::uint64_t encoder_sequence_ = 0;
  std::uint64_t odometry_imu_sequence_ = 0;
  std::uint64_t odometry_encoder_sequence_ = 0;
  bool odometry_seeded_ = false;
  std::array<float, 2> odometry_position_m_{};
  std::array<float, 2> odometry_velocity_m_s_{};
  std::array<float, 2> odometry_velocity_filtered_m_s_{};
  float accel_trust_ = 0.0F;
  JointArray odometry_previous_joints_{};
  float odometry_previous_pitch_ = 0.0F;
  bool odometry_velocity_initialized_ = false;
  float body_accel_x_m_s2_ = 0.0F;
  float accel_tilt_x_m_s2_ = 0.0F;
  bool body_accel_compensation_valid_ = false;
  int accel_id_ = -1;
  int gyro_id_ = -1;
  std::array<int, 6> pos_sensor_id_{};
  std::array<int, 6> vel_sensor_id_{};
  bool enabled_ = false;
  bool enable_request_ = false;
  bool disable_request_ = false;
  bool estop_ = false;
  bool quit_ = false;
  bool stdin_closed_ = false;
  int command_socket_ = -1;
  bool height_request_started_ = false;
  bool stationary_stop_request_ = false;
  bool dance_active_ = false;
  double dance_started_s_ = 0.0;
  float dance_direction_ = 1.0F;
  float dance_height_target_m_ = kDanceHighHeightM;
  unsigned dance_low_height_cycles_ = 0U;
  float dance_yaw_budget_rad_ = 0.0F;
  float dance_yaw_command_rad_s_ = 0.0F;
  bool dance_height_request_active_ = false;
  bool height_request_rejected_ = false;
  double dance_last_height_done_s_ = 0.0;
  static constexpr double boot_hold_duration_s_ = 1.0;
  static constexpr double disable_hold_duration_s_ = 1.0;
  double boot_hold_until_s_ = boot_hold_duration_s_;
  double disable_hold_until_s_ = 0.0;
  bool seen_ballistic_ = false;
  bool seen_touchdown_ = false;
  bool imu_seeded_ = false;
  bool imu_tilt_valid_ = false;
  std::string input_buffer_;
};

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parseOptions(argc, argv);
    NativeRunner runner(options);
    return runner.run();
  } catch (const std::exception& error) {
    std::cerr << "ERROR|" << error.what() << std::endl;
    return 2;
  }
}
