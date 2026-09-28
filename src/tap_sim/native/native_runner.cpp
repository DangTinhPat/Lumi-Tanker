#include <mujoco/mujoco.h>

#include "robot_control/GeneratedGeometry.hpp"
#include "robot_control/ImuStateEstimator.hpp"
#include "robot_hal/MotorFrameCodec.hpp"
#include "robot_control/WholeBodyModel.hpp"
#include "tap_fw/v3/V3Controller.hpp"
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
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

using robot_control::ControlCommand;
using robot_control::ImuStateEstimator;
using robot_control::JointArray;
using robot_control::JointId;
using robot_control::MotorCommand;
using robot_control::RobotState;
using robot_control::WholeBodyModel;
using tap_fw::v3::V3Controller;
using V3LegHeight = tap_fw::v3::LegHeight;
namespace motor_frame = robot_control::motor_frame;
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
struct Options {
  std::string model;
  std::string initial_keyframe = "v3_hang";
  double rate_hz = 3000.0;
  double sensor_rate_hz = 500.0;
  double run_for_s = 0.0;
  bool realtime = false;
  bool gds68_feedback = true;
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
    else if (arg == "--keyframe") options.initial_keyframe = value("--keyframe");
    else if (arg == "--rate") options.rate_hz = asDouble(value("--rate"), "--rate");
    else if (arg == "--sensor-rate") options.sensor_rate_hz = asDouble(value("--sensor-rate"), "--sensor-rate");
    else if (arg == "--run-for") options.run_for_s = asDouble(value("--run-for"), "--run-for");
    else if (arg == "--seed") options.seed = static_cast<unsigned>(std::stoul(value("--seed")));
    else if (arg == "--no-gds68") options.gds68_feedback = false;
    else if (arg == "--realtime") options.realtime = true;
    else if (arg == "--help") {
      std::cout << "tap_mujoco_native --model FILE [--keyframe NAME] [--rate 3000|4000] "
                    "[--sensor-rate 500] [--realtime] [--run-for SEC]\n";
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
    configureV3Controller();
    configureSensors();
    resetSensorPipeline();
    setNonBlockingStdin();
    std::cout << "READY|backend=mujoco_native|physics_hz=" << options_.rate_hz
              << "|control_hz=" << options_.rate_hz
              << "|sensor_hz=" << options_.sensor_rate_hz
              << "|ground_truth_to_controller=0"
              << "|gds68_feedback=" << (options_.gds68_feedback ? 1 : 0)
              << "|imu_startup_qualification=1"
              << "|scenario=v3_hang"
              << std::endl;
  }

  ~NativeRunner() {
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

  static float nativeToDriver(std::size_t joint, float native_rad) {
    return native_rad - robot_control::geometry::encoder_zero_rad[joint];
  }

  static float driverToNative(std::size_t joint, float driver_rad) {
    return driver_rad + robot_control::geometry::encoder_zero_rad[joint];
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
    const int home_keyframe = mj_name2id(
        model_, mjOBJ_KEY, options_.initial_keyframe.c_str());
    if (home_keyframe >= 0) {
    // Spawn at the selected keyframe; V3.1's default is its straight-hang zero.
      mj_resetDataKeyframe(model_, data_, home_keyframe);
    } else {
      throw std::runtime_error("missing initial keyframe: " +
                               options_.initial_keyframe);
    }
    std::fill(data_->qvel, data_->qvel + model_->nv, 0.0);
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
    // A disabled GDS68 still reports its absolute encoder position. Seed the
    // fake feedback in the driver's calibrated frame; zero-initialising it
    // makes native hip zero look like the collapsed low-body keyframe.
    for (std::size_t i = 0; i < 4; ++i) {
      const std::size_t joint = i < 2 ? i : i + 1;
      leg_feedback_position_[i] = nativeToDriver(
          joint, static_cast<float>(data_->qpos[kJointQpos[joint]]));
    }
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
    power_off_free_ = false;
    estop_ = false;
    enabled_ = false;
    leg_command_.fill(0.0F);
    leg_state_.fill(0.0F);
    // RESET drops every bus/driver state along with the command latches
    // above - a real re-power would too, and a stale CLOSED_LOOP/torque
    // target surviving a sim reset would be a false positive nothing else
    // catches.
    motor_transport_.reset();
    last_command_s_ = -1.0e9;
    boot_hold_until_s_ = sim_time_s_ + boot_hold_duration_s_;
    disable_hold_until_s_ = sim_time_s_;
  }

  void resetSimulation() {
    // Keep the runner's sensor/control clock monotonic. MuJoCo data time may
    // reset, but estimator timestamps must not jump backwards on RESET.
    mj_resetData(model_, data_);
    initializeState();
    state_ = sensorResetState();
    v3_controller_ = V3Controller(v3_config_);
    v3HomeReset();
    imu_estimator_.reset();
    resetSensorPipeline();
    resetCommandPipeline();
    imu_seeded_ = false;
    imu_tilt_valid_ = false;
  }

  static float v3HangHip(float knee_rad) {
    float hip = 0.4F;
    for (int i = 0; i < 30; ++i) {
      const auto leg = WholeBodyModel::legHeight(hip, knee_rad);
      if (std::abs(leg.x_m) < 1.0e-7F || std::abs(leg.d_x_d_hip) < 1.0e-6F) break;
      hip -= leg.x_m / leg.d_x_d_hip;
    }
    return hip;
  }

  void configureV3Controller() {
    v3_config_ = tap_fw::v3::defaultV3Config();
    v3_config_.algo = V3Controller::Algo::Pid;
    v3_config_.engage.immediate = true;
    v3_config_.height.balance_ff_scale = 0.0F;
    v3_hang_hip_rad_ = v3HangHip(0.0F);
    v3_controller_ = V3Controller(v3_config_);
    v3HomeReset();
  }

  void v3HomeReset() {
    v3_start_sent_ = false;
    v3_height_started_ = false;
    v3_start_rejected_reported_ = false;
    v3_lock_start_us_ = 0U;
  }

  MotorCommand updateV3Controller(bool sensors_ok) {
    MotorCommand motors{};
    V3Controller::Input input{};
    input.now_us = static_cast<std::uint64_t>(std::max(sim_time_s_, 0.0) * 1.0e6);
    input.dt_s = std::clamp(state_.dt_s, 1.0e-4F, 0.05F);
    input.pitch_rad = state_.imu.pitch_rad;
    input.pitch_rate_rad_s = state_.imu.pitch_rate_bias_corrected_valid
        ? state_.imu.pitch_rate_bias_corrected_rad_s
        : state_.imu.pitch_rate_rad_s;
    input.yaw_rate_rad_s = state_.imu.yaw_rate_rad_s;
    input.accel_x_m_s2 = raw_accel_[0];
    input.accel_z_m_s2 = raw_accel_[2];
    input.wheel_rad_s = {state_.joints.velocity_rad_s[2],
                         state_.joints.velocity_rad_s[5]};
    input.wheel_pos_rad = {state_.joints.position_rad[2],
                           state_.joints.position_rad[5]};
    const auto& q = state_.joints.position_rad;
    const float left_chain = q[0] + q[1] - v3_hang_hip_rad_;
    const float right_chain = q[3] + q[4] - v3_hang_hip_rad_;
    input.leg_chain_rad = 0.5F * (left_chain + right_chain);
    input.sensors_ok = sensors_ok && imu_seeded_;
    input.legs_ready = v3_lock_start_us_ != 0U &&
        input.now_us - v3_lock_start_us_ >= v3_config_.lock_settle_us &&
        std::abs(q[0] - v3_hang_hip_rad_) < 0.05F &&
        std::abs(q[1]) < 0.05F &&
        std::abs(q[3] - v3_hang_hip_rad_) < 0.05F &&
        std::abs(q[4]) < 0.05F;

    V3Controller::Command command{};
    command.disable = disable_request_ || estop_ || power_off_free_;
    command.forward_m_s = command_.target_forward_velocity_m_s;
    command.yaw_rad_s = command_.target_yaw_rate_rad_s;
    if (command_.request_height_change) {
      command.height_m = command_.target_body_height_m -
          robot_control::geometry::wheel_radius_m;
    }

    if (command.disable) {
      v3_controller_ = V3Controller(v3_config_);
      v3HomeReset();
      command.disable = true;
    } else if (enable_request_ && !v3_start_sent_ && input.sensors_ok) {
      const bool at_hang = std::abs(q[0] - v3_hang_hip_rad_) < 0.08F &&
          std::abs(q[1]) < 0.08F &&
          std::abs(q[3] - v3_hang_hip_rad_) < 0.08F &&
          std::abs(q[4]) < 0.08F;
      if (at_hang) {
        v3_start_sent_ = true;
        v3_lock_start_us_ = input.now_us;
        command.start = true;
      } else {
        enable_request_ = false;
        if (!v3_start_rejected_reported_) {
          std::cout << "ERROR|v3_start_rejected=legs_not_at_hang_zero" << std::endl;
          v3_start_rejected_reported_ = true;
        }
      }
    }

    v3_output_ = v3_controller_.update(input, command);
    motors.enable_motors = v3_output_.legs_locked || v3_output_.wheels_enabled;
    motors.torque_nm = {3.0F, 3.0F, 0.0F, 3.0F, 3.0F, 0.0F};
    motors.kp_scale = {1.0F, 1.0F, 0.0F, 1.0F, 1.0F, 0.0F};
    motors.kd_scale = {1.0F, 1.0F, 0.0F, 1.0F, 1.0F, 0.0F};
    motors.position_rad[0] = v3_hang_hip_rad_ + v3_output_.leg_hip_offset_rad;
    motors.position_rad[1] = v3_output_.leg_knee_offset_rad;
    motors.position_rad[3] = v3_hang_hip_rad_ + v3_output_.leg_hip_offset_rad;
    motors.position_rad[4] = v3_output_.leg_knee_offset_rad;
    motors.velocity_rad_s[2] = v3_output_.wheel_cmd_rad_s[0];
    motors.velocity_rad_s[5] = v3_output_.wheel_cmd_rad_s[1];
    v3_motor_command_ = motors;
    if (command_.request_height_change && v3_output_.state == V3Controller::State::Balancing) {
      if (v3_output_.legs_moving) v3_height_started_ = true;
      else if (v3_height_started_) {
        command_.request_height_change = false;
        v3_height_started_ = false;
      }
      if (std::abs(command.height_m - v3_controller_.heightTarget()) < 1.0e-4F) {
        command_.request_height_change = false;
        v3_height_started_ = false;
      }
    }
    return motors;
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
    // IDs are fixed by the model contract and validated by validate_model.py;
    // look up by the explicit names so a reordered XML cannot silently swap a side.
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
      const bool startup_sensor_fixture =
          !imu_seeded_;
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
        const std::size_t leg = static_cast<std::size_t>(node - 1U);
        leg_feedback_position_[leg] = feedback.pos_rad;
        leg_feedback_velocity_[leg] = feedback.vel_rad_s;
        feedback_[leg] = feedback.torque_nm;
      }
      for (std::size_t i = 0; i < 6; ++i) {
        float p = sensorValue(pos_sensor_id_[i]);
        float v = sensorValue(vel_sensor_id_[i]);
        if (i == 0 || i == 1 || i == 3 || i == 4) {
          const std::size_t leg = i < 2 ? i : i - 1;
          // The fake GDS68 reports its calibrated driver-frame position;
          // restore the CAD frame before it enters the controller.
          p = driverToNative(i, leg_feedback_position_[leg]);
          v = leg_feedback_velocity_[leg];
        } else {
          const std::size_t side = i == 2 ? 0 : 1;
          // BL4820M12 feedback is also zeroed at the home fixture.  The
          // native plant remains in the CAD frame internally.
          motor_transport_.setWheelMeasurement(
              side, nativeToDriver(i, p), v);
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
            p = driverToNative(i, wheel_feedback_position_unwrapped_[side]);
            v = motor_frame::wheelRawSpeedToRadPerSec(present.speed_raw);
          }
        }
        const float pq = (i == 2 || i == 5) ? 0.0061359F : 0.0003835F;
        sample.position[i] = quantize(p + normal(0.00035F), pq);
        sample.velocity[i] = quantize(v + normal(0.012F), 0.02F);
      }
      sample.effort_valid = options_.gds68_feedback && uniform_(rng_) >= 0.001;
      sample.effort[0] = feedback_[0]; sample.effort[1] = feedback_[1];
      sample.effort[3] = feedback_[2]; sample.effort[4] = feedback_[3];
      if (sample.effort_valid) encoder_queue_.push_back(sample);
      else if (!options_.gds68_feedback || uniform_(rng_) >= 0.001) encoder_queue_.push_back(sample);
    }
  }

  // Rolling-contact kinematics hold only under the sensor-derived support
  // hypothesis; in flight the accelerometer falls back to gyro integration.
  ImuStateEstimator::KinematicVelocity imuKinematicVelocity(float gyro_rate) const {
    (void)gyro_rate;
    return {};
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
      const bool balancing = v3_controller_.state() == V3Controller::State::Balancing;
      imu_estimator_.setConfig(balancing
          ? tap_fw::v3::balancingEstimatorConfig(ImuStateEstimator::Config{})
          : ImuStateEstimator::Config{});
      const bool stance_quiet = balancing &&
          std::abs(command_.target_forward_velocity_m_s) < 1.0e-3F &&
          std::abs(command_.target_yaw_rate_rad_s) < 1.0e-3F &&
          std::abs(state_.wheel.velocity_m_s) < 0.05F &&
        // Do not block bias tracking with a raw-rate gate narrower than the
        // sensor's slowly wandering bias. The estimator applies its own
        // corrected-rate and specific-force gates.
        std::abs(sample.gyro[1] - state_.imu.gyro_bias_rad_s) < 0.12F &&
          std::abs(state_.imu.pitch_rate_rad_s) < 0.12F &&
          std::abs(accel_mag - 9.81F) < 0.35F && !v3_output_.legs_moving;
      const bool pose_changing = v3_controller_.state() == V3Controller::State::Locking ||
          v3_output_.legs_moving;
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
    const bool boot_hold = !power_off_free_ && !enabled_ && !estop_ &&
        (!disable_request_ || sim_time_s_ < disable_hold_until_s_);
    // Before ENABLE there is no operator-controlled motion request, so keep
    // the natural pose qualified rather than allowing a long GUI startup to
    // turn into a fall.  Once ENABLE arrives, the same hold is time-bounded
    // and hands ownership to the core as soon as it enables motors.
    const bool boot_window = !enable_request_ || sim_time_s_ < boot_hold_until_s_;
    const bool safe = !power_off_free_ && enabled_ && !estop_ && !disable_request_ &&
        sim_time_s_ - last_command_s_ <= 0.10;
    std::array<float, 4> target_leg{};
    if (safe) target_leg = leg_command_;
    else if (boot_hold) {
      target_leg = {};
      const float target_hip = v3_hang_hip_rad_;
      const float target_knee = 0.0F;
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
        feedback_clock_[i] += dt;
        if (feedback_clock_[i] >= 0.002) {
          feedback_clock_[i] = std::fmod(feedback_clock_[i], 0.002);
          const int qpos = kJointQpos[i < 2 ? i : i + 1];
          const int qvel = kJointQvel[i < 2 ? i : i + 1];
          motor_transport_.publishLegFeedback(
              i,
              nativeToDriver(qpos - 7,
                             static_cast<float>(data_->qpos[qpos])),
              static_cast<float>(data_->qvel[qvel]),
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
      const std::size_t j = i < 2 ? i : i + 1;
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
              motor_frame::canId(static_cast<std::uint8_t>(i + 1),
                                 motor_frame::kSetControllerModeCmdId),
              motor_frame::packSetControllerMode(
                  motor_frame::kControlModePosition,
                  motor_frame::kInputModeMit));
          motor_transport_.sendCan(
              motor_frame::canId(static_cast<std::uint8_t>(i + 1),
                                 motor_frame::kSetAxisStateCmdId),
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
            std::clamp(
                nativeToDriver(j, motors.position_rad[j]),
                robot_control::geometry::encoder_min_rad[j],
                robot_control::geometry::encoder_max_rad[j]),
            motors.velocity_rad_s[j],
            legKpNmPerRad(j, hip_q, knee_q, motors.kp_scale[j]),
            legKdNmPerRadS(j, hip_q, knee_q, motors.kd_scale[j]),
            motors.feedforward_nm[j] + cross_nm);
        motor_transport_.sendCan(
            motor_frame::canId(static_cast<std::uint8_t>(i + 1),
                               motor_frame::kMitControlCmdId),
            mit_bytes);
      } else {
        // disable = IDLE every tick, not silence: a driver with its own
        // watchdog does not guarantee it stops on silence alone (same
        // reasoning as the wheel Torque Disable path below).
        motor_transport_.sendCan(
            motor_frame::canId(static_cast<std::uint8_t>(i + 1),
                               motor_frame::kSetAxisStateCmdId),
            motor_frame::packSetAxisState(motor_frame::kAxisStateIdle));
      }
      leg_command_[i] = drv.computeTorqueNm(
          nativeToDriver(j, static_cast<float>(data_->qpos[kJointQpos[j]])),
          static_cast<float>(data_->qvel[kJointQvel[j]]));
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

  void clearMotionCommand() {
    command_.target_forward_velocity_m_s = 0.0F;
    command_.target_yaw_rate_rad_s = 0.0F;
  }
  void step() {
    state_.dt_s = static_cast<float>(model_->opt.timestep);
    readCommands();
    updateActuators();
    // Hold the V3.1 standing fixture still through lock/qualification; dynamics
    // begin once its balance loop takes ownership.
    const bool waiting_for_v3_start = enable_request_ &&
        v3_controller_.state() != V3Controller::State::Balancing &&
        v3_controller_.state() != V3Controller::State::Fault;
    const bool lifecycle_paused = !power_off_free_ &&
        (!enable_request_ || estop_ || waiting_for_v3_start ||
         (disable_request_ && sim_time_s_ >= disable_hold_until_s_));
    if (!lifecycle_paused) mj_step(model_, data_);
    sim_time_s_ += model_->opt.timestep;
    enqueueSensors();
    consumeSensors();
    const bool stale = last_imu_stamp_s_ <= 0.0 || last_encoder_stamp_s_ <= 0.0 ||
        sim_time_s_ - last_imu_stamp_s_ > 0.15 || sim_time_s_ - last_encoder_stamp_s_ > 0.15;
    const bool have_any_sensor = last_imu_stamp_s_ > 0.0 && last_encoder_stamp_s_ > 0.0;
    state_.emergency_stop = estop_ || (have_any_sensor && stale);
    const bool sensors_ok = have_any_sensor && !stale && !estop_;
    const MotorCommand motors = updateV3Controller(sensors_ok);
    applyMotorCommand(power_off_free_ ? MotorCommand{} : motors);
    if (power_off_free_) std::fill(data_->ctrl, data_->ctrl + model_->nu, 0.0);
    if (sim_time_s_ + 1.0e-12 >= next_telemetry_s_) {
      next_telemetry_s_ += 0.02;
      emitTelemetry();
    }
  }

  void readCommands() {
    if (stdin_closed_) return;
    pollfd descriptor{STDIN_FILENO, POLLIN, 0};
    while (poll(&descriptor, 1, 0) > 0 && (descriptor.revents & POLLIN)) {
      char buffer[512];
      const ssize_t count = ::read(STDIN_FILENO, buffer, sizeof(buffer));
      if (count == 0) { stdin_closed_ = true; return; }
      if (count < 0) return;
      input_buffer_.append(buffer, static_cast<std::size_t>(count));
      std::size_t newline = 0;
      while ((newline = input_buffer_.find('\n')) != std::string::npos) {
        const std::string line = input_buffer_.substr(0, newline);
        input_buffer_.erase(0, newline + 1);
        parseCommand(line);
      }
    }
  }

  void parseCommand(const std::string& line) {
    std::istringstream stream(line);
    std::string op;
    stream >> op;
    if (op == "ENABLE") {
      // DISABLE is a recoverable safe stop. ESTOP remains separately latched.
      power_off_free_ = false;
      disable_request_ = false;
      enable_request_ = true;
      boot_hold_until_s_ = sim_time_s_ + boot_hold_duration_s_;
      disable_hold_until_s_ = sim_time_s_;
    }
    else if (op == "DISABLE") {
      clearMotionCommand();
      power_off_free_ = false;
      disable_request_ = true;
      enable_request_ = false;
      boot_hold_until_s_ = sim_time_s_ + disable_hold_duration_s_;
      disable_hold_until_s_ = boot_hold_until_s_;
      command_.request_pose_change = false;
      command_.request_height_change = false;
      command_.request_stand_up = false;
      command_.target_forward_velocity_m_s = 0.0F;
      command_.target_yaw_rate_rad_s = 0.0F;
    }
    else if (op == "POWER_OFF") {
      // Simulation-only fixture: emulate cutting motor power while leaving
      // MuJoCo gravity/contact dynamics running so a held squat can collapse.
      clearMotionCommand();
      power_off_free_ = true;
      disable_request_ = true;
      enable_request_ = false;
      command_.request_pose_change = false;
      command_.request_height_change = false;
      command_.request_stand_up = false;
      command_.target_forward_velocity_m_s = 0.0F;
      command_.target_yaw_rate_rad_s = 0.0F;
      std::cout << "POWER_OFF_ACCEPTED|free_dynamics=1" << std::endl;
    }
    else if (op == "ESTOP") {
      clearMotionCommand();
      estop_ = true;
      enable_request_ = false;
      boot_hold_until_s_ = sim_time_s_;
      disable_hold_until_s_ = sim_time_s_;
    }
    else if (op == "CLEAR_ESTOP") estop_ = false;
    else if (op == "RESET") {
      resetSimulation();
    } else if (op == "DRIVE") {
      clearMotionCommand();
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
          v3_controller_.state() == V3Controller::State::Balancing;
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
      clearMotionCommand();
      command_.target_forward_velocity_m_s = 0.0F;
      command_.target_yaw_rate_rad_s = 0.0F;
    }
    else if (op == "POSE") {
      std::cout << "ERROR|pose_unsupported_by_v3=1" << std::endl;
    } else if (op == "HEIGHT") {
      clearMotionCommand();
      float height_m = 0.0F;
      stream >> height_m;
      if (!setHeightTarget(height_m)) {
        std::cout << "ERROR|height_target_rejected=" << height_m << std::endl;
      } else {
        std::cout << "HEIGHT_ACCEPTED|target=" << height_m << std::endl;
      }
    } else if (op == "STAND_UP") {
      std::cout << "ERROR|stand_up_not_implemented=1" << std::endl;
    } else if (op == "DANCE") {
      std::cout << "ERROR|dance_unsupported_by_v3=1" << std::endl;
    }
    else if (op == "QUIT") quit_ = true;
  }

  bool setHeightTarget(float body_height_m) {
    if (!std::isfinite(body_height_m)) return false;
    const float target_axle_height = body_height_m -
        robot_control::geometry::wheel_radius_m;
    if (target_axle_height < V3LegHeight::minHeight() ||
        target_axle_height > V3LegHeight::maxHeight()) return false;
    command_.target_body_height_m = body_height_m;
    command_.request_height_change = true;
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
    const char* mode = "Disabled";
    switch (v3_controller_.state()) {
      case V3Controller::State::Idle: mode = "Disabled"; break;
      case V3Controller::State::Locking: mode = "Locking"; break;
      case V3Controller::State::Ready: mode = "Ready"; break;
      case V3Controller::State::Balancing: mode = "Balance"; break;
      case V3Controller::State::Fault: mode = "EmergencyStop"; break;
    }
    const bool fresh = last_imu_stamp_s_ > 0.0 && last_encoder_stamp_s_ > 0.0 &&
        sim_time_s_ - last_imu_stamp_s_ <= 0.15 &&
        sim_time_s_ - last_encoder_stamp_s_ <= 0.15;
    std::cout << "STATE|t=" << sim_time_s_ << "|mode=" << mode
              << "|controller=v3.1|v3_state=" << tap_fw::v3::toString(v3_controller_.state())
              << "|power_off_free=" << power_off_free_
              << "|pitch=" << state_.imu.pitch_rad << "|pitch_rate=" << state_.imu.pitch_rate_rad_s
              << "|pitch_rate_corrected=" << state_.imu.pitch_rate_bias_corrected_rad_s
              << "|gyro_bias=" << state_.imu.gyro_bias_rad_s
              << "|accel_mag=" << state_.imu.accel_magnitude_m_s2
              << "|imu_seeded=" << imu_seeded_
              << "|imu_tilt_valid=" << imu_tilt_valid_
              << "|raw_accel_x=" << raw_accel_[0]
              << "|raw_accel_z=" << raw_accel_[2]
              << "|wheel_position=" << state_.wheel.position_m
              << "|wheel_velocity=" << state_.wheel.velocity_m_s
              << "|joint_lh=" << state_.joints.position_rad[0]
              << "|joint_lk=" << state_.joints.position_rad[1]
              << "|joint_rh=" << state_.joints.position_rad[3]
              << "|joint_rk=" << state_.joints.position_rad[4]
              << "|enc_lh=" << nativeToDriver(0, state_.joints.position_rad[0])
              << "|enc_lk=" << nativeToDriver(1, state_.joints.position_rad[1])
              << "|enc_lw=" << nativeToDriver(2, state_.joints.position_rad[2])
              << "|enc_rh=" << nativeToDriver(3, state_.joints.position_rad[3])
              << "|enc_rk=" << nativeToDriver(4, state_.joints.position_rad[4])
              << "|enc_rw=" << nativeToDriver(5, state_.joints.position_rad[5])
              << "|joint_v_lh=" << state_.joints.velocity_rad_s[0]
              << "|joint_v_lk=" << state_.joints.velocity_rad_s[1]
              << "|joint_v_rh=" << state_.joints.velocity_rad_s[3]
              << "|joint_v_rk=" << state_.joints.velocity_rad_s[4]
              << "|v3_balance_ref=" << v3_controller_.balanceRef()
              << "|lqr_velocity=" << v3_controller_.groundVelocity()
              << "|lqr_anchor=" << v3_controller_.groundPosition()
              << "|station_velocity_target=0|station_hold="
              << (v3_controller_.state() == V3Controller::State::Balancing)
              << "|station_capture_ready="
              << (v3_controller_.state() == V3Controller::State::Balancing)
              << "|station_eligible="
              << (v3_controller_.state() == V3Controller::State::Balancing)
              << "|target_lh=" << v3_motor_command_.position_rad[0]
              << "|target_lk=" << v3_motor_command_.position_rad[1]
              << "|target_rh=" << v3_motor_command_.position_rad[3]
              << "|target_rk=" << v3_motor_command_.position_rad[4]
              << "|leg_cmd_lh=" << leg_command_[0]
              << "|leg_cmd_lk=" << leg_command_[1]
              << "|leg_cmd_rh=" << leg_command_[2]
              << "|leg_cmd_rk=" << leg_command_[3]
              << "|leg_applied_lh=" << applied_leg_torque_[0]
              << "|leg_applied_lk=" << applied_leg_torque_[1]
              << "|leg_applied_rh=" << applied_leg_torque_[2]
              << "|leg_applied_rk=" << applied_leg_torque_[3]
              << "|imu_fresh=" << fresh << "|enc_fresh=" << fresh
              << "|height_request=" << command_.request_height_change
              << "|height_target=" << command_.target_body_height_m
              << "|body_height=" << measuredBodyHeight()
              << "|height_motion_start=" << measuredBodyHeight()
              << "|height_motion_target=" << v3_controller_.heightTarget() +
                  robot_control::geometry::wheel_radius_m
              << "|height_motion=" << v3_output_.legs_moving
              << "|height_settling=0|height_settled=" << !v3_output_.legs_moving
              << "|height_pending=0|height_pending_elapsed=0|height_elapsed=0"
              << "|height_duration=0|height_rejected=0|skill=none"
              << "|yaw_rate_command=" << command_.target_yaw_rate_rad_s
              << "|cmd_lh=" << v3_motor_command_.position_rad[0]
              << "|cmd_lk=" << v3_motor_command_.position_rad[1]
              << "|cmd_lw=" << v3_motor_command_.velocity_rad_s[2]
              << "|cmd_rh=" << v3_motor_command_.position_rad[3]
              << "|cmd_rk=" << v3_motor_command_.position_rad[4]
              << "|cmd_rw=" << v3_motor_command_.velocity_rad_s[5]
              << "|v3_fault=" << (v3_controller_.lastFault() ? v3_controller_.lastFault() : "")
              << "|ground_truth_to_controller=0"
              << "|fall_pitch_limit=" << v3_config_.fall_pitch_rad
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
  V3Controller::Config v3_config_{};
  V3Controller v3_controller_{};
  V3Controller::Output v3_output_{};
  MotorCommand v3_motor_command_{};
  float v3_hang_hip_rad_ = 0.0F;
  bool v3_start_sent_ = false;
  bool v3_height_started_ = false;
  bool v3_start_rejected_reported_ = false;
  std::uint64_t v3_lock_start_us_ = 0U;
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
  std::array<float, 4> feedback_{};
  std::array<float, 4> leg_feedback_position_{};
  std::array<float, 4> leg_feedback_velocity_{};
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
  bool power_off_free_ = false;
  bool estop_ = false;
  bool quit_ = false;
  bool stdin_closed_ = false;
  static constexpr double boot_hold_duration_s_ = 1.0;
  static constexpr double disable_hold_duration_s_ = 1.0;
  double boot_hold_until_s_ = boot_hold_duration_s_;
  double disable_hold_until_s_ = 0.0;
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
