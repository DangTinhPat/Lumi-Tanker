#include <mujoco/mujoco.h>

#include "robot_control/GeneratedGeometry.hpp"
#include "robot_control/ImuStateEstimator.hpp"
#include "robot_control/WholeBodyModel.hpp"
#include "tap_fw/v3/CurrentTorque.hpp"
#include "tap_fw/v3/LegHeight.hpp"
#include "tap_fw/v3/V3Controller.hpp"
#include "SimActuatorBus.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <unistd.h>
#include <vector>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

namespace {
using tap_fw::v3::V3Controller;
namespace geometry = robot_control::geometry;
namespace motor_transport = robot_control::motor_transport;

struct Options {
  std::string model;
  double physics_hz = 3000.0;
  double sensor_hz = 500.0;
  double state_hz = 50.0;  // diagnostics only; VIEW remains 50 Hz
  std::uint16_t command_port = 0;
  double run_for_s = 0.0;
  double wheel_delay_s = 0.050;
  double wheel_lag_s = 0.075;
  double sensor_noise = 0.0;  // scale of documented synthetic sensor standard deviations
  unsigned noise_seed = 1;
  bool allow_model_mismatch = false;  // explicit plant-only robustness experiments
  bool realtime = false;
  bool lqr_prediction = true;
  bool gyro_hip_lead = true;
  std::string jump_spec;  // v3.5 jump parameters: key=value,key=value
  std::string standup_spec;  // stand-up parameters: key=value,key=value (--standup)
  std::string control_spec;  // explicit simulation-only A/B tuning overrides
  // Speed ceilings [m/s], 0 = the controller defaults: DRIVE clamp, planned speed cap, LQR ground speed cap.
  double drive_max = 4.1888, plan_vmax = 0.0, ground_vmax = 0.0, track_margin = -1.0;
  double motor_reserve = -1.0, lateral_accel = -1.0;
  std::string start_key = "spawn_extended";   // --start-key NAME: the MJCF keyframe the robot starts in ("home" = the lying pose)
  double wheel_torque = 0.13;   // regular/rated cap; --wheel-torque overrides every phase [N.m]
  bool wheel_torque_override = false;
  // --leg-actuator pos|delta_i: how the four leg joints are driven. pos (default, v3.5.2) models the GDS68
  // position loop the firmware commands with Set_Input_Pos (no Kt, no Iq feedback, no gravity model in the
  // controller); delta_i models the older current-compensated torque path.
  bool leg_pos = true;
  bool roll = false;     // --roll: roll balance on, with the firmware's layered response (off by default: the flat-ground tests)
  // Commands run at exact simulated times (--at SEC:COMMAND); stdin timing depends on
  // process scheduling, so headless tests use this to be reproducible.
  std::vector<std::pair<double, std::string>> schedule;
};

double number(const char* text, const char* option) {
  char* end = nullptr;
  const double value = std::strtod(text, &end);
  if (end == text || *end != '\0' || !std::isfinite(value))
    throw std::invalid_argument(std::string("invalid ") + option);
  return value;
}

Options parseOptions(int argc, char** argv) {
  Options out;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    auto next = [&]() -> const char* {
      if (i + 1 >= argc) throw std::invalid_argument("missing value for " + arg);
      return argv[++i];
    };
    if (arg == "--model") out.model = next();
    else if (arg == "--rate") out.physics_hz = number(next(), "--rate");
    else if (arg == "--sensor-rate") out.sensor_hz = number(next(), "--sensor-rate");
    else if (arg == "--state-hz") out.state_hz = number(next(), "--state-hz");
    else if (arg == "--wheel-delay") out.wheel_delay_s = number(next(), "--wheel-delay");
    else if (arg == "--wheel-lag") out.wheel_lag_s = number(next(), "--wheel-lag");
    else if (arg == "--sensor-noise") out.sensor_noise = number(next(), "--sensor-noise");
    else if (arg == "--allow-model-mismatch") out.allow_model_mismatch = true;
    else if (arg == "--noise-seed") {
      const double value = number(next(), "--noise-seed");
      if (value < 0 || value > 4294967295.0 || value != std::floor(value))
        throw std::invalid_argument("invalid --noise-seed");
      out.noise_seed = static_cast<unsigned>(value);
    }
    else if (arg == "--command-port") {
      const auto port = std::stoul(next());
      if (port > 65535U) throw std::invalid_argument("invalid --command-port");
      out.command_port = static_cast<std::uint16_t>(port);
    } else if (arg == "--run-for") out.run_for_s = number(next(), "--run-for");
    else if (arg == "--realtime") out.realtime = true;
    else if (arg == "--no-lqr-prediction") out.lqr_prediction = false;
    else if (arg == "--virtual-hip-lead") out.gyro_hip_lead = false;
    else if (arg == "--jump") out.jump_spec = next();
    else if (arg == "--standup") out.standup_spec = next();
    else if (arg == "--control") out.control_spec = next();
    else if (arg == "--drive-max") out.drive_max = number(next(), "--drive-max");
    else if (arg == "--plan-vmax") out.plan_vmax = number(next(), "--plan-vmax");
    else if (arg == "--ground-vmax") out.ground_vmax = number(next(), "--ground-vmax");
    else if (arg == "--track-margin") out.track_margin = number(next(), "--track-margin");
    else if (arg == "--start-key") out.start_key = next();
    else if (arg == "--wheel-torque") {
      out.wheel_torque = number(next(), "--wheel-torque");
      out.wheel_torque_override = true;
    }
    else if (arg == "--leg-actuator") {
      const std::string kind = next();
      if (kind != "pos" && kind != "delta_i") throw std::invalid_argument("--leg-actuator must be pos or delta_i");
      out.leg_pos = kind == "pos";
    }
    else if (arg == "--roll") out.roll = number(next(), "--roll") != 0.0;
    else if (arg == "--motor-reserve") out.motor_reserve = number(next(), "--motor-reserve");
    else if (arg == "--lateral-accel") out.lateral_accel = number(next(), "--lateral-accel");
    else if (arg == "--at") {
      const std::string value = next();
      const auto colon = value.find(':');
      if (colon == std::string::npos) throw std::invalid_argument("--at expects SEC:COMMAND");
      out.schedule.emplace_back(number(value.substr(0, colon).c_str(), "--at"),
                                value.substr(colon + 1));
    }
    else if (arg == "--help") {
      std::cout << "tap_mujoco_gim_v3 --model FILE [--rate HZ] [--sensor-rate HZ] "
                   "[--command-port PORT] [--wheel-delay SEC] [--wheel-lag SEC] "
                   "[--wheel-torque 0..0.20] [--leg-actuator pos|delta_i] [--standup key=val,...] "
                   "[--sensor-noise SCALE] [--noise-seed UINT] [--allow-model-mismatch] "
                   "[--state-hz HZ] [--control key=val,...] "
                   "[--no-lqr-prediction] [--virtual-hip-lead] [--jump key=val,...] [--at SEC:CMD] "
                   "[--realtime] [--run-for SEC]\n";
      std::exit(0);
    } else throw std::invalid_argument("unknown option: " + arg);
  }
  if (out.model.empty()) throw std::invalid_argument("--model is required");
  if (out.sensor_noise < 0 || out.sensor_noise > 10)
    throw std::invalid_argument("invalid --sensor-noise");
  if (out.physics_hz < 1000.0 || out.physics_hz > 4000.0 ||
      out.sensor_hz <= 0.0 || out.sensor_hz > out.physics_hz || out.run_for_s < 0.0 ||
      out.state_hz <= 0.0 || out.state_hz > out.sensor_hz ||
      out.wheel_delay_s < 0 || out.wheel_delay_s > .1 || out.wheel_lag_s < 0 || out.wheel_lag_s > .12 ||
      out.wheel_torque <= 0.0 || out.wheel_torque > 0.20)
    throw std::invalid_argument("invalid simulation rate or duration");
  return out;
}

class GimV3Runner {
 public:
  explicit GimV3Runner(Options options)
      : options_(std::move(options)), controller_(simulationControllerConfig(options_)) {
    noise_rng_.seed(options_.noise_seed);
    char error[1024]{};
    model_ = mj_loadXML(options_.model.c_str(), nullptr, error, sizeof(error));
    if (!model_) throw std::runtime_error(std::string("MuJoCo load failed: ") + error);
    model_->opt.timestep = 1.0 / options_.physics_hz;
    data_ = mj_makeData(model_);
    if (!data_) throw std::runtime_error("MuJoCo data allocation failed");
    // Base cap 0.13 N.m; recovery can request a bounded 0.20 N.m window.
    // --wheel-torque explicitly overrides both base and recovery phases.
    for (std::size_t side = 0; side < wheels_.size(); ++side) {
      wheels_[side] = robot_control::SimBl4012Wheel({static_cast<std::uint8_t>(side + 1), static_cast<float>(options_.wheel_torque)});
    }
    bindModel();
    wheel_delay_ticks_ = static_cast<std::size_t>(std::lround(options_.wheel_delay_s * options_.sensor_hz));
    configureImu();
    reset();
    setNonBlockingStdin();
    openCommandSocket();
    std::cout << "READY|backend=mujoco_gim_v3|model=GIM6010-8|controller=V3Controller"
              << "|leg_actuator=" << (options_.leg_pos ? "driver_pos" : "current_delta_i") << "|physics_hz=" << options_.physics_hz
              << "|control_hz=" << options_.sensor_hz
              << "|ground_truth_to_controller=0|wheel_regular_limit_nm=" << options_.wheel_torque
              << "|wheel_peak_limit_nm=" << (options_.wheel_torque_override ? options_.wheel_torque : 0.20)
              << "|mass_kg=" << geometry::total_mass_kg
              << "|plant_mass_kg=" << mj_getTotalmass(model_)
              << "|sensor_noise=" << options_.sensor_noise << "|noise_seed=" << options_.noise_seed
              << std::endl;
  }

  ~GimV3Runner() {
    if (socket_ >= 0) ::close(socket_);
    if (data_) mj_deleteData(data_);
    if (model_) mj_deleteModel(model_);
  }

  int run() {
    const auto wall_start = std::chrono::steady_clock::now();
    while (!quit_) {
      step();
      if (options_.run_for_s > 0.0 && sim_time_s_ >= options_.run_for_s) break;
      if (options_.realtime) {
        const auto deadline = wall_start + std::chrono::duration_cast<
            std::chrono::steady_clock::duration>(std::chrono::duration<double>(sim_time_s_));
        std::this_thread::sleep_until(deadline);
      }
    }
    std::cout << "EXIT|sim_time_s=" << sim_time_s_ << std::endl;
    return 0;
  }

 private:
  struct Joint {
    int joint = -1, qpos = -1, qvel = -1, position_sensor = -1, velocity_sensor = -1;
    int actuator = -1;
  };

  // Simulation-only: enable autonomous recovery; --standup overrides.
  static void applyStandUpSpec(tap_fw::v3::StandUp::Config& su, const std::string& spec) {
    su.enabled = true;
    su.hold_at_kneel = false;
    su.hold_after_flip = false;
    su.overlap_after_lift = true;
    su.fold_rate_rad_s = 0.80F;
    su.flip_hip_rate_max_rad_s = 3.00F;
    su.flip_pitch_kp_rad_s_per_rad = 10.0F;
    su.fold_pitch_limit_rad = 0.80F;
    su.catch_body_gain = 4.0F;
    su.catch_body_damping_s = 0.5F;
    su.catch_hip_rate_max_rad_s = 2.0F;
    su.catch_wheel_max_rad_s = 16.0F;
    su.catch_accel_max = 3.0F;
    su.catch_speed_gain = 0.0F;  // LQR brakes translation after the capture gate
    su.catch_blend_s = 0.08F;
    su.catch_settle_s = 0.02F;
    su.catch_speed_tol_m_s = 0.7F;
    su.catch_wheel_peak_s = 0.6F;
    std::stringstream list(spec);
    std::string item;
    while (std::getline(list, item, ',')) {
      const auto eq = item.find('=');
      if (eq == std::string::npos) continue;
      const std::string key = item.substr(0, eq);
      const float v = std::stof(item.substr(eq + 1));
      if (key == "fold_rate") su.fold_rate_rad_s = v;
      else if (key == "fold_travel") su.fold_travel_rad = v;
      else if (key == "fold_direction") su.fold_direction = v;
      else if (key == "fold_pitch_limit") su.fold_pitch_limit_rad = v;
      else if (key == "hold_at_kneel") su.hold_at_kneel = v >= 0.5F;
      else if (key == "hold_after_flip") su.hold_after_flip = v >= 0.5F;
      else if (key == "overlap_after_lift") su.overlap_after_lift = v >= 0.5F;
      else if (key == "lift_clearance") su.lift_clearance_m = v;
      else if (key == "lift_progress") su.lift_progress_rad = v;
      else if (key == "lift_confirm") su.lift_confirm_s = v;
      else if (key == "catch_blend") su.catch_blend_s = v;
      else if (key == "flip_kp") su.flip_pitch_kp_rad_s_per_rad = v;
      else if (key == "flip_kd") su.flip_pitch_kd = v;
      else if (key == "flip_rate") su.flip_hip_rate_max_rad_s = v;
      else if (key == "flip_travel") su.flip_travel_max_rad = v;
      else if (key == "pitch_tol") su.pitch_target_tol_rad = v;
      else if (key == "pitch_limit") su.pitch_limit_rad = v;
      else if (key == "catch_accel_gain") su.catch_accel_gain = v;
      else if (key == "catch_accel_damping") su.catch_accel_damping = v;
      else if (key == "catch_speed_gain") su.catch_speed_gain = v;
      else if (key == "catch_accel_max") su.catch_accel_max = v;
      else if (key == "catch_lead") su.catch_wheel_lead_s = v;
      else if (key == "catch_hip_gain") su.catch_hip_gain = v;
      else if (key == "catch_hip_damping") su.catch_hip_damping_s = v;
      else if (key == "catch_body_gain") su.catch_body_gain = v;
      else if (key == "catch_body_damping") su.catch_body_damping_s = v;
      else if (key == "catch_peak") su.catch_wheel_peak_s = v;
      else if (key == "kneel_hold") su.kneel_hold_s = v;
      else if (key == "catch_hip_accel") su.catch_hip_accel_rad_s2 = v;
      else if (key == "catch_capture_tol") su.catch_capture_tol_rad = v;
      else if (key == "catch_speed_tol") su.catch_speed_tol_m_s = v;
      else if (key == "catch_settle") su.catch_settle_s = v;
      else if (key == "catch_rate_max") su.catch_hip_rate_max_rad_s = v;
      else if (key == "catch_travel") su.catch_hip_travel_rad = v;
      else if (key == "catch_wheel_max") su.catch_wheel_max_rad_s = v;
      else if (key == "catch_lean_tol") su.catch_lean_tol_rad = v;
      else if (key == "timeout") su.timeout_s = v;
      else throw std::invalid_argument("unknown --standup key: " + key);
    }
  }

  static void applyJumpSpec(tap_fw::v3::JumpSequence::Config& jump, const std::string& spec) {
    jump.enabled = true;
    auto& pl = jump.planner;
    std::stringstream list(spec);
    std::string item;
    while (std::getline(list, item, ',')) {
      const auto eq = item.find('=');
      if (eq == std::string::npos) continue;
      const std::string key = item.substr(0, eq);
      const float v = std::stof(item.substr(eq + 1));
      // planner
      if (key == "target") pl.target_clearance_m = v;
      else if (key == "rise_min") pl.com_rise_min_m = v;
      else if (key == "rise_max") pl.com_rise_max_m = v;
      else if (key == "knee_acc") pl.knee_accel_rad_s2 = v;
      else if (key == "knee_rate_max") pl.max_knee_rate_rad_s = v;
      else if (key == "top_h") pl.top_height_m = v;
      else if (key == "tuck_min_knee") pl.tuck_min_knee_rad = v;
      else if (key == "hold") pl.hold_s = v;
      else if (key == "tuck_done") pl.tuck_done_frac = v;
      else if (key == "extend") pl.extend_fraction = v;
      else if (key == "tuck_turn") pl.tuck_body_turn_rad = v;
      else if (key == "hip_acc") pl.hip_accel_rad_s2 = v;
      else if (key == "td_margin") pl.touchdown_margin_s = v;
      else if (key == "derate") pl.torque_derate = v;
      else if (key == "min_dip") pl.min_dip_m = v;
      // knobs on the plan
      else if (key == "lift_factor") jump.lift_speed_factor = v;
      else if (key == "lift_acc") jump.lift_off_accel_ratio = v;
      else if (key == "flight_kp") jump.flight_kp_scale = v;
      else if (key == "flight_kd") jump.flight_kd_scale = v;
      else if (key == "tuck_rate") jump.tuck_start_rate_rad_s = v;
      else if (key == "thrust_extra") jump.thrust_extra_s = v;
      else if (key == "dip_extra") jump.dip_extra_m = v;
      else if (key == "flight_delay") jump.flight_delay_s = v;
      else if (key == "land_lead") jump.land_lead_s = v;
      else if (key == "thrust_kp") jump.thrust_kp_scale = v;
      else if (key == "thrust_kd") jump.thrust_kd_scale = v;
      else if (key == "touch_a") jump.touchdown_accel_m_s2 = v;
      else if (key == "min_flight") jump.min_flight_s = v;
      else if (key == "fb_delay") jump.flight_fb_delay_s = v;
      else if (key == "land_kp") jump.land_kp_scale = v;
      else if (key == "land_kd") jump.land_kd_scale = v;
      else if (key == "land_hkp") jump.land_hip_kp_scale = v;
      else if (key == "land_hkd") jump.land_hip_kd_scale = v;
      else if (key == "land_follow") jump.land_follow = v;
      else if (key == "land_wait") jump.land_wait_s = v;
      else if (key == "land_s") jump.land_s = v;
      else if (key == "recover_s") jump.recover_s = v;
      else if (key == "max_pitch") jump.max_pitch_rad = v;
      else if (key == "abort_pitch") jump.abort_pitch_rad = v;
      else if (key == "tkp") jump.thrust_pitch_kp = v;
      else if (key == "tkd") jump.thrust_pitch_kd = v;
      else if (key == "wheel_kick") jump.wheel_kick_rad_s = v;
      else if (key == "mom_wheel") jump.momentum_wheel_gain = v;
      else if (key == "mom_wheel_max") jump.momentum_wheel_max_rad_s = v;
      else if (key == "mom_dip") jump.momentum_in_dip = v > 0.5F;
      else if (key == "to_on") jump.takeoff.enabled = v > 0.5F;
      else if (key == "to_hold") jump.takeoff.hold_wheels = v > 0.5F;
      else if (key == "to_gain") jump.takeoff.gain = v;
      else if (key == "to_rate") jump.takeoff.rate_gain = v;
      else if (key == "to_max") jump.takeoff.lean_max_rad = v;
      else if (key == "to_tau") jump.takeoff.lean_tau_s = v;
      else if (key == "to_sign") jump.takeoff.sign = v;
      else if (key == "to_wl") jump.takeoff.momentum_weight = v;
      else if (key == "thrust_lean") jump.hop_thrust_lean_per_m_s = v;
      else if (key == "rest_lean") jump.rest_thrust_lean_rad = v;
      else if (key == "hop_lean") jump.hop_lean_rad = v;
      else if (key == "hop_stop") jump.hop_stop_s = v;
      else if (key == "run_up") jump.hop_run_up = v > 0.5F;
      else if (key == "ready_lean") jump.ready_lean_rad = v;
      else if (key == "ready_drop") jump.ready_drop_m = v;
      else if (key == "ready_time") jump.ready_time_s = v;
      else if (key == "hop_gimbal") jump.hop_gimbal = v;
      else if (key == "appr_settle") jump.approach_settle_s = v;
      else if (key == "ready_first") jump.ready_before_drive = v >= 0.5;
      else if (key == "td_window") jump.touchdown_window_s = v;
      else if (key == "appr_tol") jump.approach_tolerance_m_s = v;
      else if (key == "hop_acc1") jump.hop_accel_per_m_s = v;
      else if (key == "hop_scale") jump.hop_speed_scale = v;
      else if (key == "hop_acc") jump.hop_accel_m_s2 = v;
      else if (key == "hop_dec") jump.hop_decel_m_s2 = v;
      else if (key == "hop_jerk") jump.hop_jerk_m_s3 = v;
      else if (key == "hop_default") jump.hop_default_m = v;
      else if (key == "wheel_match") jump.flight_wheel_match = v > 0.5F;
      else if (key == "fa_wline") jump.flight.w_line = v;
      else if (key == "fa_wbody") jump.flight.w_body = v;
      else if (key == "fa_wshape") jump.flight.w_shape = v;
      else if (key == "fa_tilt") jump.flight.body_tilt_max_rad = v;
      else if (key == "fa_bias") jump.flight.line_bias_rad = v;
      else if (key == "fa_hipmax") jump.flight.hip_change_max_rad = v;
      else if (key == "fa_kneemax") jump.flight.knee_change_max_rad = v;
      else if (key == "fa_rate") jump.flight.change_rate_max_rad_s = v;
      else if (key == "fa_arrive") jump.flight.arrive_before_s = v;
      else if (key == "hip_gain") jump.thrust_hip_gain = v;
      else if (key == "dip_acc") jump.dip_accel_m_s2 = v;
      else throw std::invalid_argument("unknown --jump key: " + key);
    }
  }

  static V3Controller::Config simulationControllerConfig(const Options& options) {
    auto config = tap_fw::v3::defaultV3Config();
    config.lqr.sampling_period_s = static_cast<float>(1.0 / options.sensor_hz);
    // The extended startup fixture replaces the operator holding the robot.
    config.engage.immediate = true;
    config.engage.automatic = true;
    config.engage.wb_armed_only = false;
    // Roll balance (SET roll 1 on the robot) with the layered response the firmware boots with
    // (AppV3.cpp): level reference learned in place, turn lean-in, latch, loop tied to the pitch gimbal.
    config.roll.enabled = options.roll;
    config.roll.learn_ref = true;
    config.roll.cmd_preview = true;
    config.roll.latch = true;
    config.roll.gimbal_link = true;
    config.initial_knee_rad = tap_fw::v3::LegHeight::kKneeMaxRad;
    config.com_height_m = tap_fw::v3::LegHeight::forKnee(config.initial_knee_rad).com_above_axle_m;
    config.lqr_inertia_length = true;
    config.lqr.max_pendulum_length_m = 0.24F;
    // Keep normal balance authority at the 0.13 N.m rated/base cap; only the
    // short rocking phases below are allowed the separately confirmed peak.
    config.lqr.max_ground_accel_m_s2 = std::min(config.lqr.wheel_ground_mu * 9.81F,
        2.0F * 0.13F / (geometry::wheel_radius_m * geometry::total_mass_kg));
    config.lqr_prediction = {options.lqr_prediction, static_cast<float>(options.wheel_delay_s),
                                  static_cast<float>(options.wheel_lag_s)};
    config.lqr_lead_adaptive = false;
    config.lqr_lead_s = static_cast<float>(options.wheel_lag_s);
    config.tier.reflex_gate = true;
    config.tier.reflex_inertial_gate = true;
    // Virtual-rate lead feeds the commanded hip motion back into that same hip.
    config.tier.lead_gyro_only = options.gyro_hip_lead;
    // The current-controlled hip plant rings at ~31 Hz with 60 ms gyro lead.
    // Keep planned-lean preview and filtered body damping intact. Direct gyro
    // lead excites the fast loaded joint loop; its old 60 ms position-servo
    // tuning does not transfer to this current/impedance actuator model.
    config.tier.lead_s = 0.0F;
    config.tier.drive_governor.enabled = true;
    config.tier.plan_accel_m_s2 = .9F;
    config.tier.plan_jerk_m_s3 = 10.0F;
    config.tier.hip_advance_s = config.tier.preview_s;
    config.tier.height_exp = .15F;
    config.tier.body_damping_s = .08F;
    config.tier.body_damp_lpf_hz = 4.5F;
    config.tier.knee_follow_hz = 1.0F;
    config.tier.knee_follow_rate_rad_s = .6F;
    config.tier.sus_adaptive = true;
    config.trim_acc = false;
    applyJumpSpec(config.jump, options.jump_spec);
    // Use the same calibrated actuator lag as the balance predictor; an
    // explicit catch_lead override permits model-mismatch experiments.
    config.standup.catch_wheel_lead_s = static_cast<float>(options.wheel_lag_s);
    applyStandUpSpec(config.standup, options.standup_spec);
    if (options.plan_vmax > 0.0) config.tier.plan_vmax_m_s = static_cast<float>(options.plan_vmax);
    // motor_reserve < 0 keeps the default; 0 still caps at the no-load speed; >= 1.5 turns the envelope off.
    if (options.motor_reserve >= 0.0) config.tier.motor_reserve = static_cast<float>(options.motor_reserve);
    if (options.motor_reserve >= 1.5) config.tier.wheel_envelope = false;
    if (options.lateral_accel >= 0.0) config.tier.lateral_accel_max_m_s2 = static_cast<float>(options.lateral_accel);
    if (options.ground_vmax > 0.0) config.lqr.max_ground_velocity_m_s = static_cast<float>(options.ground_vmax);
    if (options.track_margin >= 0.0) config.lqr.command_velocity_tracking_margin_m_s = static_cast<float>(options.track_margin);
    config.height.rate_m_s = .015F;
    config.height.accel_m_s2 = .05F;
    config.height.jerk_m_s3 = .20F;
    config.tier.joint_rate_gain_low = 1.0F;
    std::stringstream tuning(options.control_spec);
    std::string item;
    while (std::getline(tuning, item, ',')) {
      const auto eq = item.find('=');
      if (eq == std::string::npos) throw std::invalid_argument("--control expects key=value");
      const std::string key = item.substr(0, eq);
      const double v = number(item.substr(eq+1).c_str(), "--control");
      if (v < 0) throw std::invalid_argument("negative --control value");
      if (key == "trunk") config.tier.trunk = v >= .5;
      else if (key == "body_damping") config.tier.body_damping_s = v;
      else if (key == "body_lpf") config.tier.body_damp_lpf_hz = v;
      else if (key == "hip_lead") config.tier.lead_s = v;
      else if (key == "hip_lead_lpf") config.tier.lead_lpf_hz = v;
      else if (key == "knee_lpf") config.tier.knee_follow_hz = v;
      else if (key == "plan_accel") config.tier.plan_accel_m_s2 = v;
      else if (key == "plan_jerk") config.tier.plan_jerk_m_s3 = v;
      else if (key == "governor") config.tier.drive_governor.enabled = v >= .5;
      else if (key == "balance_reserve") config.tier.drive_governor.balance_reserve = v;
      else if (key == "governor_torque") config.tier.drive_governor.wheel_torque_nm = v;
      else if (key == "preview") config.tier.preview = v >= .5;
      else if (key == "carrier") config.tier.wheel_carrier_gain = v;
      else if (key == "wheel_lead") config.lqr_lead_s = v;
      else if (key == "trim_gain") config.lqr_trim_gain = v;
      else throw std::invalid_argument("unknown --control key: " + key);
    }
    return config;
  }

  static int id(mjModel* model, mjtObj type, const char* name) {
    const int result = mj_name2id(model, type, name);
    if (result < 0) throw std::runtime_error(std::string("GIM MJCF missing ") + name);
    return result;
  }

  void bindModel() {
    const std::array<const char*, 6> names = {
        "hip_left", "knee_left", "wheel_left", "hip_right", "knee_right", "wheel_right"};
    const std::array<const char*, 6> pos = {
        "enc_left_hip_pos", "enc_left_knee_pos", "enc_left_wheel_pos",
        "enc_right_hip_pos", "enc_right_knee_pos", "enc_right_wheel_pos"};
    const std::array<const char*, 6> vel = {
        "enc_left_hip_vel", "enc_left_knee_vel", "enc_left_wheel_vel",
        "enc_right_hip_vel", "enc_right_knee_vel", "enc_right_wheel_vel"};
    const std::array<const char*, 6> act = {
        "left_hip_torque", "left_knee_torque", "left_wheel_torque",
        "right_hip_torque", "right_knee_torque", "right_wheel_torque"};
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      auto& joint = joints_[i];
      joint.joint = id(model_, mjOBJ_JOINT, names[i]);
      joint.qpos = model_->jnt_qposadr[joint.joint];
      joint.qvel = model_->jnt_dofadr[joint.joint];
      joint.position_sensor = id(model_, mjOBJ_SENSOR, pos[i]);
      joint.velocity_sensor = id(model_, mjOBJ_SENSOR, vel[i]);
      joint.actuator = id(model_, mjOBJ_ACTUATOR, act[i]);
      for (const auto binding : {std::pair<int, int>{joint.position_sensor, mjSENS_JOINTPOS},
                                 {joint.velocity_sensor, mjSENS_JOINTVEL}}) {
        if (model_->sensor_type[binding.first] != binding.second ||
            model_->sensor_dim[binding.first] != 1 ||
            model_->sensor_objtype[binding.first] != mjOBJ_JOINT ||
            model_->sensor_objid[binding.first] != joint.joint)
          throw std::runtime_error("GIM encoder must measure its joint, not world state");
      }
    }
    accel_sensor_ = id(model_, mjOBJ_SENSOR, "mpu6500_accel");
    body_id_ = id(model_, mjOBJ_BODY, "gim_body");
    left_wheel_body_ = id(model_, mjOBJ_BODY, "left_wheel");
    right_wheel_body_ = id(model_, mjOBJ_BODY, "right_wheel");
    gyro_sensor_ = id(model_, mjOBJ_SENSOR, "mpu6500_gyro");
    const int imu_site = id(model_, mjOBJ_SITE, "imu_site");
    for (const auto binding : {std::pair<int, int>{accel_sensor_, mjSENS_ACCELEROMETER},
                               {gyro_sensor_, mjSENS_GYRO}}) {
      if (model_->sensor_type[binding.first] != binding.second ||
          model_->sensor_dim[binding.first] != 3 ||
          model_->sensor_objtype[binding.first] != mjOBJ_SITE ||
          model_->sensor_objid[binding.first] != imu_site)
        throw std::runtime_error("GIM IMU must measure acceleration/gyro at its site");
    }
    if (model_->nq != 13 || model_->nv != 12 || model_->nu != 6 ||
        model_->nbody != 8 || (!options_.allow_model_mismatch &&
          std::abs(mj_getTotalmass(model_) - 3.02) > 1.0e-5))
      throw std::runtime_error("GIM MJCF joint or mass contract mismatch");
  }

  void configureImu() {
    robot_control::ImuStateEstimator::Config config{};
    config.accel_correction_gain_hz = 5.0F;
    config.commanded_accel_blend = 0.0F;
    config.seed_window_samples = 50;
    estimator_.setConfig(tap_fw::v3::balancingEstimatorConfig(config));
  }

  void setNonBlockingStdin() {
    const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
  }

  void openCommandSocket() {
    if (options_.command_port == 0) return;
    socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0) throw std::runtime_error("command socket creation failed");
    int reuse = 1;
    setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(options_.command_port);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::bind(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
      throw std::runtime_error("command socket bind failed");
    const int flags = fcntl(socket_, F_GETFL, 0);
    if (flags >= 0) fcntl(socket_, F_SETFL, flags | O_NONBLOCK);
  }

  double sensor(int sensor_id, int component = 0) const {
    return data_->sensordata[model_->sensor_adr[sensor_id] + component];
  }

  void readCommands() {
    if (!stdin_closed_) {
      pollfd fd{STDIN_FILENO, POLLIN, 0};
      while (poll(&fd, 1, 0) > 0 && (fd.revents & POLLIN)) {
        char buffer[512];
        const ssize_t count = ::read(STDIN_FILENO, buffer, sizeof(buffer));
        if (count == 0) { stdin_closed_ = true; break; }
        if (count < 0) break;
        input_buffer_.append(buffer, static_cast<std::size_t>(count));
        std::size_t end;
        while ((end = input_buffer_.find('\n')) != std::string::npos) {
          const auto line = input_buffer_.substr(0, end);
          input_buffer_.erase(0, end + 1);
          parseCommand(line);
        }
      }
    }
    if (socket_ < 0) return;
    for (;;) {
      char packet[512];
      const ssize_t count = recv(socket_, packet, sizeof(packet), 0);
      if (count <= 0) break;
      parseCommand(std::string(packet, static_cast<std::size_t>(count)));
    }
  }

  void parseCommand(const std::string& line) {
    std::istringstream stream(line);
    std::string op;
    stream >> op;
    if (op == "ENABLE") {
      // A robot that starts lying (--start-key other than spawn_extended) only becomes active through STANDUP:
      // the normal start-up would lock the legs to the extended pose and drag the lying robot there.
      if (options_.start_key != "spawn_extended" && !stood_up_) {
        std::cout << "ENABLE_REJECTED|reason=robot_starts_lying_use_STANDUP" << std::endl;
        return;
      }
      if (!estop_ && !enable_request_) start_pending_ = true;
      enable_request_ = true;
      disable_request_ = false;
    } else if (op == "DISABLE") {
      enable_request_ = false;
      disable_request_ = true;
      stood_up_ = false;                 // limp again: a lying start needs STANDUP once more
      command_.forward_m_s = command_.yaw_rad_s = 0.0F;
    } else if (op == "ESTOP") {
      estop_ = true;
      stood_up_ = false;
      enable_request_ = false;
      command_.forward_m_s = command_.yaw_rad_s = 0.0F;
    } else if (op == "CLEAR_ESTOP") estop_ = false;
    else if (op == "RESET") reset();
    else if (op == "STOP") command_.forward_m_s = command_.yaw_rad_s = 0.0F;
    else if (op == "GO") go_pending_ = true;
    else if (op == "STANDUP") {          // the rocking stand-up from the lying pose (start with --start-key home)
      if (!estop_ && !enable_request_) {
        standup_pending_ = true;
        // Body/root height above the floor, matching the GUI HEIGHT command.
        // The rocking phase keeps its own leg targets; this request takes
        // effect after hand-over to balance instead of extending to the stop.
        constexpr float kStandUpBodyHeightM = 0.130F;
        command_.height_m = std::clamp(
            kStandUpBodyHeightM - geometry::wheel_radius_m + geometry::origins[0].z,
            tap_fw::v3::LegHeight::minHeight(), controller_.supportedMaxHeight());
      }
      stood_up_ = !estop_;
      enable_request_ = true;
    }
    else if (op == "FLIP") standup_advance_pending_ = true;
    else if (op == "JUMP") {
      float distance = 0.0F;      // "JUMP" = the default hop, "JUMP 0" = vertical, "JUMP 0.05" = hop 5 cm
      jump_forward_m_ = (stream >> distance && std::isfinite(distance)) ? distance
                                                                          : std::numeric_limits<float>::quiet_NaN();
      jump_pending_ = true;
    }
    else if (op == "PUSH") {
      float force = 0.0F, duration = 0.0F;
      if (stream >> force >> duration && std::isfinite(force) && std::isfinite(duration) &&
          std::abs(force) <= 50.0F && duration > 0.0F && duration <= 2.0F) {
        push_force_n_ = force;
        push_until_s_ = sim_time_s_ + duration;
      } else std::cout << "ERROR|command=PUSH|reason=expected force_N (-50..50), duration_s (0..2)"
                       << std::endl;
    }
    else if (op == "DRIVE") {
      float forward = 0.0F, yaw = 0.0F;
      stream >> forward >> yaw;
      if ((forward != 0.0F || yaw != 0.0F) &&
          (!physics_started_ || output_.state != V3Controller::State::Balancing || estop_)) {
        std::cout << "DRIVE_REJECTED|reason=balance_not_ready" << std::endl;
        return;
      }
      const float drive_max = static_cast<float>(options_.drive_max);
      command_.forward_m_s = std::clamp(std::isfinite(forward) ? forward : 0.0F, -drive_max, drive_max);
      command_.yaw_rad_s = std::clamp(std::isfinite(yaw) ? yaw : 0.0F, -1.5F, 1.5F);
    } else if (op == "HEIGHT") {
      float body_height = 0.0F;
      stream >> body_height;
      if (std::isfinite(body_height)) {
        const float axle_height = body_height - geometry::wheel_radius_m +
                                  geometry::origins[0].z;
        command_.height_m = std::clamp(axle_height,
            tap_fw::v3::LegHeight::minHeight(), controller_.supportedMaxHeight());
        std::cout << "HEIGHT_ACCEPTED|target=" << command_.height_m + geometry::wheel_radius_m -
            geometry::origins[0].z << std::endl;
      }
    } else if (op == "POSE") {
      float lh = 0, lk = 0, rh = 0, rk = 0;
      if (stream >> lh >> lk >> rh >> rk) {
        const auto l = robot_control::WholeBodyModel::legHeight(
            tap_fw::v3::LegHeight::hangHipCad() + lh, lk);
        const auto r = robot_control::WholeBodyModel::legHeight(
            tap_fw::v3::LegHeight::hangHipCad() + rh, rk);
        command_.height_m = std::clamp(-0.5F * (l.height_m + r.height_m),
            tap_fw::v3::LegHeight::minHeight(), tap_fw::v3::LegHeight::maxHeight());
      }
    } else if (op == "QUIT") quit_ = true;
  }

  static void announcePlan(const tap_fw::v3::JumpPlan& p) {
    std::cout << "JUMP_PLAN|ok=" << p.ok << "|reason=" << p.reason
              << "|crouch_m=" << p.crouch_height_m << "|top_m=" << p.top_height_m
              << "|takeoff_m_s=" << p.takeoff_speed_m_s << "|flight_s=" << p.flight_time_s
              << "|com_rise_m=" << p.com_rise_m << "|knee_travel=" << p.predicted_knee_travel_rad
              << "|tuck_lift_m=" << p.predicted_tuck_lift_m << "|predicted_clearance_m=" << p.predicted_clearance_m
              << std::endl;
  }

  void reset() {
    controller_ = V3Controller(simulationControllerConfig(options_));
    announcePlan(controller_.prepareJump());
    estimator_.reset();
    mj_resetDataKeyframe(model_, data_, id(model_, mjOBJ_KEY, options_.start_key.c_str()));
    mj_forward(model_, data_);
    const float knee = controller_.config().initial_knee_rad;
    const auto spawn_pose = tap_fw::v3::LegHeight::forKnee(knee);
    const float hip = tap_fw::v3::LegHeight::hangHipCad() + spawn_pose.hip_offset_rad;
    const auto spawn_com = robot_control::WholeBodyModel{}.compute(hip, knee, hip, knee);
    const float hang_hip = tap_fw::v3::LegHeight::hangHipCad();
    const auto hang_com = robot_control::WholeBodyModel{}.compute(hang_hip, 0, hang_hip, 0);
    // V3's balance seed is in the virtual-pitch frame, not the raw IMU frame.
    const float virtual_offset = controller_.config().tier.virtual_pitch
        ? spawn_com.com_line_angle_rad - hang_com.com_line_angle_rad : 0.0F;
    controller_.setBalanceAngle(true,
        robot_control::WholeBodyModel::equilibriumPitch(spawn_com) + virtual_offset);
    enable_request_ = disable_request_ = estop_ = false;
    // The extended spawn is held by the start-up fixture (physics frozen) until the controller is ready; a robot
    // that starts in another keyframe (the lying pose) is on the floor under gravity from the first tick.
    physics_started_ = options_.start_key != "spawn_extended";
    enable_seen_ = false;
    push_force_n_ = 0;
    push_until_s_ = 0;
    wheel_command_history_ = {};
    wheel_history_index_ = 0;
    wheel_enable_elapsed_s_ = 0;
    wheel_target_rad_s_.fill(0);
    wheel_speed_reference_rad_s_.fill(0);
    start_pending_ = go_pending_ = jump_pending_ = standup_pending_ = standup_advance_pending_ = stood_up_ = false;
    command_ = {};
    command_.height_m = spawn_pose.axle_below_hip_m;
    output_ = {};
    for (std::size_t i = 0; i < previous_leg_target_.size(); ++i) {
      previous_leg_target_[i] = i % 2 == 0 ? spawn_pose.hip_offset_rad : knee;
    }
    leg_target_rate_.fill(0.0F);
    leg_target_current_.fill(0.0F);
    leg_applied_current_.fill(0.0F);
    leg_measured_current_.fill(0.0F);
    leg_pos_integrator_nm_.fill(0.0F);
    for (std::size_t side = 0; side < 2; ++side)
      wheels_[side].onTorqueEnable(false);
    imu_seeded_ = false;
  }

  float sensorNoise(float sigma) {
    if (options_.sensor_noise == 0.0 || !physics_started_) return 0.0F;
    return static_cast<float>(options_.sensor_noise) * sigma * noise_normal_(noise_rng_);
  }

  void updateController() {
    const float dt = static_cast<float>(1.0 / options_.sensor_hz);
    const std::uint64_t now_us = static_cast<std::uint64_t>(sim_time_s_ * 1.0e6);
    std::array<float, 3> accel{}, gyro{};
    if (!physics_started_) {
      // The extended startup fixture holds the mechanism stationary until
      // balance commands and the wheel torque-enable settle window are ready.
      accel[2] = 9.80665F;
    } else {
      for (int i = 0; i < 3; ++i) {
        accel[static_cast<std::size_t>(i)] = static_cast<float>(sensor(accel_sensor_, i)) + sensorNoise(.03F);
        gyro[static_cast<std::size_t>(i)] = static_cast<float>(sensor(gyro_sensor_, i)) + sensorNoise(.005F);
      }
    }
    const auto velocity = controller_.imuVelocity();
    const auto estimate = estimator_.update(accel, gyro, dt,
        static_cast<float>(sim_time_s_), 0.0F, output_.legs_moving, !enable_request_,
        {velocity.horizontal_m_s, velocity.vertical_m_s, physics_started_ && velocity.valid});
    imu_seeded_ = estimate.seeded || imu_seeded_;

    V3Controller::Input in{};
    in.now_us = now_us;
    in.dt_s = dt;
    in.pitch_rad = estimate.pitch_rad;
    in.pitch_rate_rad_s = estimate.pitch_rate_rad_s;
    in.yaw_rate_rad_s = estimate.yaw_rate_rad_s;
    in.roll_rate_rad_s = estimate.roll_rate_rad_s;
    in.accel_x_m_s2 = accel[0];
    in.accel_y_m_s2 = accel[1];
    in.accel_z_m_s2 = accel[2];
    in.legs_measured = true;
    in.sensors_ok = imu_seeded_ && estimate.valid;
    in.legs_ready = true;
    for (std::size_t side = 0; side < 2; ++side) {
      const std::size_t hip = 3 * side;
      const std::size_t knee = hip + 1;
      const std::size_t wheel = hip + 2;
      in.hip_rad[side] = static_cast<float>(sensor(joints_[hip].position_sensor)) + sensorNoise(.0005F);
      in.knee_rad[side] = static_cast<float>(sensor(joints_[knee].position_sensor)) + sensorNoise(.0005F);
      in.hip_rate_rad_s[side] = static_cast<float>(sensor(joints_[hip].velocity_sensor)) + sensorNoise(.01F);
      in.knee_rate_rad_s[side] = static_cast<float>(sensor(joints_[knee].velocity_sensor)) + sensorNoise(.01F);
      in.wheel_rad_s[side] = static_cast<float>(sensor(joints_[wheel].velocity_sensor)) + sensorNoise(.03F);
      in.wheel_pos_rad[side] = static_cast<float>(sensor(joints_[wheel].position_sensor)) + sensorNoise(.0005F);
      in.leg_chain_rad += 0.5F * (in.hip_rad[side] + in.knee_rad[side]);
      in.knee_current_a[side] = options_.leg_pos ? 0.0F : leg_measured_current_[2 * side + 1];
    }
    in.leg_current_valid = !options_.leg_pos;  // the POS firmware does not stream or use Iq
    for (std::size_t i = 0; i < previous_leg_target_.size(); ++i) {
      const std::size_t joint = (i / 2) * 3 + (i % 2);
      const float q = static_cast<float>(sensor(joints_[joint].position_sensor));
      in.legs_ready = in.legs_ready && std::abs(previous_leg_target_[i] - q) < 0.08F;
    }

    V3Controller::Command cmd{};
    cmd.start = start_pending_ && in.sensors_ok;
    if (cmd.start) start_pending_ = false;
    cmd.standup = standup_pending_ && in.sensors_ok;
    if (cmd.standup) standup_pending_ = false;
    cmd.standup_advance = standup_advance_pending_;
    standup_advance_pending_ = false;
    cmd.go = go_pending_;
    go_pending_ = false;
    cmd.jump = jump_pending_ && output_.state == V3Controller::State::Balancing;
    cmd.jump_forward_m = jump_forward_m_;
    jump_pending_ = false;
    cmd.disable = disable_request_ || estop_;
    cmd.forward_m_s = enable_request_ && !cmd.disable ? command_.forward_m_s : 0.0F;
    cmd.yaw_rad_s = enable_request_ && !cmd.disable ? command_.yaw_rad_s : 0.0F;
    cmd.height_m = command_.height_m;
    const auto previous_state = output_.state;
    output_ = controller_.update(in, cmd);
    if (previous_state == V3Controller::State::StandUp &&
        output_.state == V3Controller::State::Balancing) {
      // Emit at the actual control tick: 50 Hz STATE samples are too sparse
      // to check velocity continuity while the robot is accelerating.
      std::cout << "RECOVERY_HANDOFF|t=" << sim_time_s_
                << "|ground_velocity=" << controller_.groundVelocity()
                << "|plan_v=" << controller_.planSpeed()
                << "|lqr_vref=" << controller_.lqrDebug().lqr_velocity_reference_m_s
                << std::endl;
    }
    if (output_.jump_phase == 4 && last_jump_phase_ == 3) {
      const auto& f = controller_.jump().flightPlan();
      std::cout << "JUMP_FLIGHT|ok=" << f.ok << "|reason=" << f.reason << "|v_m_s=" << f.vertical_speed_m_s
                << "|tuck_start_s=" << f.tuck_start_s << "|knee_start=" << f.knee_start_rad << "|knee_rate0=" << f.knee_rate_start_rad_s
                << "|knee_tuck=" << f.knee_tuck_rad << "|knee_top=" << f.knee_top_rad
                << "|peak_knee_acc=" << f.peak_knee_accel_rad_s2
                << "|flight_s=" << f.flight_time_s
                << "|extend_start_s=" << f.extend_start_s << "|tuck_lift_m=" << f.tuck_lift_m
                << "|predicted_clearance_m=" << f.predicted_clearance_m << std::endl;
    }
    last_jump_phase_ = output_.jump_phase;
    if (output_.jump_refused != 0) {
      std::cout << "JUMP_REFUSED|reason="
                << (output_.jump_refused == 1 ? "tilted_or_held"
                    : output_.jump_refused == 3 ? "no_plan"
                    : output_.jump_refused == 5 ? "run_up_speed_not_reached"
                    : output_.jump_refused == 6 ? "backward_jump_not_supported" : "legs_still_moving") << std::endl;
    }

    const std::array<float, 4> commanded_target{
        output_.leg_hip_offset_rad + output_.leg_roll_hip_rad[0],
        output_.leg_knee_offset_rad + output_.leg_roll_knee_rad[0],
        output_.leg_hip_offset_rad + output_.leg_roll_hip_rad[1],
        output_.leg_knee_offset_rad + output_.leg_roll_knee_rad[1]};
    const auto target = output_.state == V3Controller::State::Fault
        ? previous_leg_target_ : commanded_target;
    {
      // Same filter as AppV3 (20 ms low-pass of the clamped common path rate), used only in stand-up.
      const float alpha = static_cast<float>(dt / (0.02 + dt));
      const std::array<float, 2> common{output_.leg_hip_offset_rad, output_.leg_knee_offset_rad};
      for (std::size_t k = 0; k < 2; ++k) {
        if (output_.state == V3Controller::State::StandUp) {
          const float raw = std::clamp((common[k] - standup_ff_prev_[k]) / static_cast<float>(dt), -6.0F, 6.0F);
          standup_ff_rate_[k] += alpha * (raw - standup_ff_rate_[k]);
        } else {
          standup_ff_rate_[k] = 0.0F;
        }
        standup_ff_prev_[k] = common[k];
      }
    }
    for (std::size_t i = 0; i < target.size(); ++i) {
      leg_target_rate_[i] = (target[i] - previous_leg_target_[i]) / dt;
      previous_leg_target_[i] = target[i];
      const std::size_t side = i / 2;
      const std::size_t joint = side * 3 + (i % 2);
      const float q = static_cast<float>(sensor(joints_[joint].position_sensor));
      const float qd = static_cast<float>(sensor(joints_[joint].velocity_sensor));
      if (options_.leg_pos) {
        // GDS68 position + passthrough: cascaded position -> velocity -> torque loop inside the driver.
        // The controller sends only the target (plus the roll velocity feed-forward); the driver has no gravity
        // model, so the integrator removes the static error. Gains are PROVISIONAL (the real driver
        // pos_gain / vel_gain / vel_integrator_gain are set on the driver, not in this repository): they
        // reproduce the stiffness and damping of the delta-I path (kJointKp, kJointKd).
        float tau = 0.0F;
        if (output_.legs_locked) {
          // Like LegLock::velFfTurns: the roll rate reference plus, in stand-up only, the path rate.
          // The firmware also feeds forward the filtered stand-up path rate (AppV3 ff_*_rate_).
          const float standup_ff = output_.state == V3Controller::State::StandUp ? standup_ff_rate_[i % 2] : 0.0F;
          const float vel_ff = standup_ff +
              (i % 2 == 0 ? output_.leg_roll_hip_rate[side] : output_.leg_roll_knee_rate[side]);
          const float vel_cmd = std::clamp(kPosGainPerS * (target[i] - q) + vel_ff,
                                           -kDriverVelLimitRadS, kDriverVelLimitRadS);
          const float vel_err = vel_cmd - qd;
          const float unsat = kVelGainNmSPerRad * vel_err + leg_pos_integrator_nm_[i];
          tau = std::clamp(unsat, -kTorqueLimitNm, kTorqueLimitNm);
          // Anti-windup: integrate only while the torque is not saturated against the error.
          if (tau == unsat) {
            leg_pos_integrator_nm_[i] = std::clamp(
                leg_pos_integrator_nm_[i] + kVelIntegratorGain * vel_err * static_cast<float>(dt),
                -kTorqueLimitNm, kTorqueLimitNm);
          }
        } else {
          leg_pos_integrator_nm_[i] = 0.0F;
        }
        leg_target_current_[i] = tau / kKtNmPerA;
      } else {
        const float hip = in.hip_rad[side], knee = in.knee_rad[side];
        const auto leg_gravity = robot_control::WholeBodyModel::legGravityTorque(hip, knee);
        const auto body_gravity = robot_control::WholeBodyModel::bodySupportTorque(hip, knee);
        const float gravity = i % 2 == 0
            ? leg_gravity.hip_nm + body_gravity.hip_nm
            : leg_gravity.knee_nm + body_gravity.knee_nm;
        const float requested = output_.legs_locked
            ? gravity + kJointKp * (i % 2 == 0 ? output_.leg_hip_kp_scale : output_.leg_kp_scale) *
                    (target[i] - q) +
                kJointKd * (i % 2 == 0 ? output_.leg_hip_kd_scale : output_.leg_kd_scale) *
                    (leg_target_rate_[i] - qd) : 0.0F;
        const auto iq = tap_fw::v3::currentTorqueCommand(requested,
            leg_measured_current_[i], output_.legs_locked,
            kKtNmPerA, kCurrentLimitA, kCurrentCompGain);
        leg_target_current_[i] = iq.torque_nm / kKtNmPerA;
      }
      in.legs_ready = in.legs_ready && std::abs(target[i] - q) < 0.08F;
    }
    for (std::size_t side = 0; side < 2; ++side) {
      auto& wheel = wheels_[side];
      wheel.onTorqueEnable(output_.wheels_enabled && !cmd.disable);
      float physical_speed = output_.wheel_cmd_rad_s[side];
      if (wheel_delay_ticks_ > 0) {
        physical_speed = wheel_command_history_[wheel_history_index_][side];
        wheel_command_history_[wheel_history_index_][side] = output_.wheel_cmd_rad_s[side];
      }
      wheel_target_rad_s_[side] = physical_speed;
    }
    if (wheel_delay_ticks_ > 0) wheel_history_index_ = (wheel_history_index_ + 1) % wheel_delay_ticks_;
    if (!output_.wheels_enabled) wheel_command_history_ = {};
    emitTelemetry(in, estimate);
  }

  void step() {
    while (next_scheduled_ < options_.schedule.size() &&
           options_.schedule[next_scheduled_].first <= sim_time_s_ + 1.0e-12) {
      parseCommand(options_.schedule[next_scheduled_++].second);
    }
    readCommands();
    const double dt = model_->opt.timestep;
    if (sim_time_s_ + 1.0e-12 >= next_control_s_) {
      next_control_s_ += 1.0 / options_.sensor_hz;
      updateController();
    }
    const bool active = enable_request_ && !estop_ &&
        output_.state != V3Controller::State::Idle;
    enable_seen_ = enable_seen_ || enable_request_;
    wheel_enable_elapsed_s_ = output_.wheels_enabled ? wheel_enable_elapsed_s_ + dt : 0;
    const bool balance_ready = output_.state == V3Controller::State::Balancing &&
        wheel_enable_elapsed_s_ >= options_.wheel_delay_s &&
        !wheels_[0].settling() && !wheels_[1].settling();
    physics_started_ = physics_started_ || balance_ready ||
        output_.state == V3Controller::State::Fault ||
        (enable_seen_ && (!enable_request_ || estop_));
    for (std::size_t i = 0; i < 4; ++i) {
      const double alpha = dt / (kCurrentLagS + dt);
      if (active && output_.legs_locked)
        leg_applied_current_[i] += alpha * (leg_target_current_[i] - leg_applied_current_[i]);
      else
        leg_applied_current_[i] = 0.0F;
      leg_measured_current_[i] = static_cast<float>(leg_applied_current_[i]);
      const std::size_t side = i / 2;
      const std::size_t joint = side * 3 + (i % 2);
      const int actuator = joints_[joint].actuator;
      data_->ctrl[actuator] = active && output_.legs_locked
          ? std::clamp(kKtNmPerA * leg_applied_current_[i], -kTorqueLimitNm, kTorqueLimitNm)
          : 0.0;
    }
    // Each measured Iq channel follows its own simulated actuator state.
    for (std::size_t side = 0; side < 2; ++side) {
      auto& wheel = wheels_[side];
      const bool recovery_peak = output_.state == V3Controller::State::StandUp &&
          controller_.standupState().wheel_peak_requested && !options_.wheel_torque_override;
      const float torque_cap = recovery_peak ? 0.20F : static_cast<float>(options_.wheel_torque);
      wheel.setMaxTorqueNm(torque_cap);
      // Reported wheel lag is speed-response lag, not motor-current lag.
      const double alpha = options_.wheel_lag_s > 0 ? dt / (options_.wheel_lag_s + dt) : 1;
      if (active && output_.wheels_enabled) {
        wheel_speed_reference_rad_s_[side] += static_cast<float>(alpha) *
            (wheel_target_rad_s_[side] - wheel_speed_reference_rad_s_[side]);
      } else wheel_speed_reference_rad_s_[side] = 0;
      wheel.onMovingSpeed(wheel_speed_reference_rad_s_[side]);
      wheel.step(static_cast<float>(dt));
      const std::size_t index = 3 * side + 2;
      const float physical_velocity = static_cast<float>(data_->qvel[joints_[index].qvel]);
      data_->ctrl[joints_[index].actuator] = active && output_.wheels_enabled
          ? wheel.computeTorqueNm(physical_velocity) : 0.0;
    }
    // A controller stop cannot suspend gravity or contact dynamics.
    data_->xfrc_applied[6 * body_id_] = sim_time_s_ < push_until_s_ ? push_force_n_ : 0.0F;
    if (physics_started_) mj_step(model_, data_);
    sim_time_s_ += dt;
    if (sim_time_s_ >= next_telemetry_s_) {
      next_telemetry_s_ += 0.02;
      emitView();
    }
  }

  void emitTelemetry(const V3Controller::Input& in,
                     const robot_control::ImuStateEstimator::Output& estimate) {
    if (sim_time_s_ < next_state_s_) return;
    next_state_s_ += 1.0 / options_.state_hz;
    const char* mode = tap_fw::v3::toString(output_.state);
    const auto wheel_l = static_cast<float>(sensor(joints_[2].velocity_sensor));
    const auto wheel_r = static_cast<float>(sensor(joints_[5].velocity_sensor));
    const auto com = robot_control::WholeBodyModel{}.compute(
        in.hip_rad[0], in.knee_rad[0], in.hip_rad[1], in.knee_rad[1]);
    std::cout << "STATE|t=" << sim_time_s_ << "|mode=" << mode
              << "|pitch=" << in.pitch_rad << "|pitch_rate=" << in.pitch_rate_rad_s
              << "|pitch_rate_corrected=" << in.pitch_rate_rad_s
              << "|reflex_gate=" << controller_.reflexGate()
              << "|lqr_prediction=" << controller_.config().lqr_prediction.enabled
              << "|plan_acceleration=" << controller_.plannedAcceleration()
              << "|plan_accel_limit=" << controller_.plannedAccelerationLimit()
              << "|drive_governor=" << controller_.config().tier.drive_governor.enabled
              << "|hip_trunk_request=" << controller_.trunkSwingRequest()
              << "|hip_trunk_command=" << controller_.trunkHip()
              << "|hip_level=" << controller_.levelHip()
              << "|trunk_authority=" << controller_.trunkAuthority()
              << "|trunk_workspace_limited=" << controller_.trunkWorkspaceLimited()
              << "|hip_target=" << controller_.hipCommand()
              << "|knee_target=" << controller_.kneeCommand()
              << "|hip_lead_gyro=" << controller_.config().tier.lead_gyro_only
              << "|accel_mag=" << estimate.accel_magnitude_m_s2
              << "|imu_seeded=" << imu_seeded_ << "|imu_tilt_valid=" << estimate.valid
              << "|wheel_velocity=" << 0.5F * geometry::wheel_radius_m * (wheel_l + wheel_r)
              << "|wheel_position=" << controller_.groundPosition()
              << "|wheel_left_position=" << geometry::wheel_radius_m *
                  (in.wheel_pos_rad[0] + in.pitch_rad + in.hip_rad[0] + in.knee_rad[0])
              << "|wheel_right_position=" << geometry::wheel_radius_m *
                  (in.wheel_pos_rad[1] + in.pitch_rad + in.hip_rad[1] + in.knee_rad[1])
              << "|joint_lh=" << in.hip_rad[0] << "|joint_lk=" << in.knee_rad[0]
              << "|joint_rh=" << in.hip_rad[1] << "|joint_rk=" << in.knee_rad[1]
              << "|enabled=" << enable_request_ << "|estop=" << estop_
              << "|startup_fixture=" << !physics_started_
              << "|drive_ready=" << (physics_started_ && output_.state == V3Controller::State::Balancing &&
                                         output_.wheels_enabled && in.sensors_ok && !estop_)
              << "|fault_reason=" << (controller_.lastFault() && *controller_.lastFault()
                                       ? controller_.lastFault() : "none")
              << "|height_request=" << (command_.height_m > 0.0F)
              << "|height_target=" << controller_.heightTarget() + geometry::wheel_radius_m -
                  geometry::origins[0].z
              << "|max_body_height=" << controller_.supportedMaxHeight() + geometry::wheel_radius_m -
                  geometry::origins[0].z
              << "|height_motion=" << output_.legs_moving
              << "|height_settled=" << (!output_.legs_moving && output_.legs_locked)
              << "|height_rejected=0|flight=unknown|airborne=unknown|touchdown=unknown"
              << "|torque_valid=1|skill=none|dance_height_target=0|dance_yaw_rate=0"
              << "|lqr_pitch_state=" << controller_.virtualPitch()
              << "|balance_ref=" << controller_.balanceRef()
              << "|ground_velocity=" << controller_.groundVelocity()
              << "|wheel_cmd_l=" << output_.wheel_cmd_rad_s[0]
              << "|wheel_cmd_r=" << output_.wheel_cmd_rad_s[1]
              << "|station_capture_distance=unknown|wb_com_line_angle=" << com.com_line_angle_rad
              << "|fz_l=unknown|fz_r=unknown|loop_hz=" << options_.sensor_hz
              << "|ground_truth_to_controller=0"
              << "|leg_iq_lh=" << leg_measured_current_[0]
              << "|leg_iq_lk=" << leg_measured_current_[1]
              << "|jump_phase=" << output_.jump_phase
              << "|td_off=" << controller_.jumpTouchdownOffset() << "|td_target=" << controller_.jumpTouchdownTarget() << "|td_pred=" << controller_.jumpTouchdownPredicted() << "|lqr_cmd_v=" << controller_.lqrDebug().commanded_ground_velocity_m_s << "|lqr_accel=" << controller_.lqrDebug().optimal_command_m_s << "|lqr_vref=" << controller_.lqrDebug().lqr_velocity_reference_m_s << "|lqr_v_seen=" << controller_.lqrSpeedSeen() << "|hop_drive=" << controller_.jumpDriveSpeed() << "|plan_v=" << controller_.planSpeed() << "|aac_air=" << (controller_.airborneState().contact == tap_fw::v3::AirborneEstimator::Contact::Air ? 1 : 0) << "|aac_left=" << controller_.airborneState().time_left_s << "|aac_l=" << controller_.airborneState().angular_momentum << "|aac_vx=" << controller_.airborneState().horizontal_speed_m_s << "|to_lean=" << controller_.takeoffState().lean_rad << "|wheel_tq_l=" << data_->ctrl[joints_[2].actuator] << "|wheel_tq_r=" << data_->ctrl[joints_[5].actuator] << "|su_phase=" << static_cast<int>(controller_.standupState().phase) << "|su_phase_name=" << tap_fw::v3::StandUp::name(controller_.standupState().phase) << "|su_lean=" << controller_.standupLean() << "|su_hip=" << controller_.standupState().hip_target_rad << "|su_wheel=" << controller_.standupState().wheel_rad_s
              << "|su_capture=" << controller_.standupState().capture_point_rad
              << "|su_rate=" << controller_.standupState().capture_rate_rad_s
              << "|su_hip_rate=" << controller_.standupState().hip_rate_rad_s
              << "|su_clearance=" << controller_.standupState().body_clearance_m
              << "|su_clearance_valid=" << controller_.standupState().body_clearance_valid
              << "|su_blend=" << controller_.standupState().catch_blend
              << "|su_peak=" << controller_.standupState().wheel_peak_requested
              << "|su_elapsed=" << controller_.standupState().phase_elapsed_s << "|sus_dz=" << controller_.suspensionOffset() << "|sus_db=" << controller_.suspensionDeadband() << "|roll_est=" << controller_.rollEstimate() << "|roll_ref=" << controller_.rollReference() << "|roll_dh=" << controller_.rollSideHeight() << "|to_err=" << controller_.takeoffState().predicted_error_rad << "|l_est=" << controller_.jumpMomentum().angular_momentum << "|jump_vz=" << controller_.jumpVz() << "|jump_az=" << controller_.jumpAz()
              << "|cmd_hip=" << output_.leg_hip_offset_rad << "|cmd_knee=" << output_.leg_knee_offset_rad
              << "|imu_accel_norm=" << std::sqrt(in.accel_x_m_s2 * in.accel_x_m_s2 +
                  in.accel_y_m_s2 * in.accel_y_m_s2 + in.accel_z_m_s2 * in.accel_z_m_s2)
              << "|held_quiet=" << controller_.heldQuiet()
              << "|held_outer_off=" << controller_.heldOuterOff()
              << "|wheel_clearance=" << wheelClearance()
              << "|ang_mom_y=" << angularMomentumY() << "|body_wy=" << data_->qvel[4]
              << "|com_x=" << data_->subtree_com[3 * body_id_] << "|com_z=" << data_->subtree_com[3 * body_id_ + 2]
              << "|leg_iq_rh=" << leg_measured_current_[2]
              << "|leg_iq_rk=" << leg_measured_current_[3]
              << std::endl;
  }

  // Logging only: angular momentum of the whole robot about its CoM (about +Y).
  double angularMomentumY() {
    mj_subtreeVel(model_, data_);
    return data_->subtree_angmom[3 * body_id_ + 1];
  }

  // Logging only: lowest wheel-centre height above the floor minus the radius.
  double wheelClearance() const {
    if (left_wheel_body_ < 0 || right_wheel_body_ < 0) return 0.0;
    return std::min(data_->xpos[3 * left_wheel_body_ + 2], data_->xpos[3 * right_wheel_body_ + 2]) -
           geometry::wheel_radius_m;
  }

  void emitView() {
    std::cout << "VIEW|t=" << sim_time_s_ << "|x=" << data_->qpos[0]
              << "|physics_t=" << data_->time
              << "|y=" << data_->qpos[1] << "|z=" << data_->qpos[2]
              << "|qw=" << data_->qpos[3] << "|qx=" << data_->qpos[4]
              << "|qy=" << data_->qpos[5] << "|qz=" << data_->qpos[6]
              << "|body_height=" << data_->qpos[2]
              << "|lh=" << data_->qpos[joints_[0].qpos]
              << "|lk=" << data_->qpos[joints_[1].qpos]
              << "|lw=" << data_->qpos[joints_[2].qpos]
              << "|rh=" << data_->qpos[joints_[3].qpos]
              << "|rk=" << data_->qpos[joints_[4].qpos]
              << "|rw=" << data_->qpos[joints_[5].qpos] << std::endl;
  }

  Options options_;
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  V3Controller controller_;
  std::mt19937 noise_rng_;
  std::normal_distribution<float> noise_normal_{0.0F, 1.0F};
  V3Controller::Command command_{};
  V3Controller::Output output_{};
  robot_control::ImuStateEstimator estimator_;
  std::array<robot_control::SimBl4012Wheel, 2> wheels_{{
      robot_control::SimBl4012Wheel({1}), robot_control::SimBl4012Wheel({2})}};
  std::array<std::array<float, 2>, 400> wheel_command_history_{};
  std::size_t wheel_history_index_ = 0, wheel_delay_ticks_ = 0;
  double wheel_enable_elapsed_s_ = 0;
  std::array<float, 2> wheel_target_rad_s_{}, wheel_speed_reference_rad_s_{};
  std::array<Joint, 6> joints_{};
  std::array<float, 4> previous_leg_target_{};
  std::array<float, 4> leg_target_rate_{};
  std::array<float, 4> leg_target_current_{};
  std::array<float, 4> leg_applied_current_{};
  std::array<float, 4> leg_measured_current_{};
  std::array<float, 4> leg_pos_integrator_nm_{};
  std::array<float, 2> standup_ff_rate_{}, standup_ff_prev_{};  // [hip, knee] common path rate (POS stand-up feed-forward)
  int accel_sensor_ = -1, gyro_sensor_ = -1, body_id_ = -1;
  int left_wheel_body_ = -1, right_wheel_body_ = -1;
  std::uint64_t socket_port_ = 0;
  int socket_ = -1;
  double sim_time_s_ = 0.0, next_control_s_ = 0.0, next_telemetry_s_ = 0.0,
         next_state_s_ = 0.0;
  bool enable_request_ = false, disable_request_ = false, estop_ = false;
  std::size_t next_scheduled_ = 0;
  int last_jump_phase_ = 0;
  bool start_pending_ = false, go_pending_ = false, jump_pending_ = false;
  bool standup_pending_ = false, standup_advance_pending_ = false, imu_seeded_ = false;
  bool stood_up_ = false;   // a lying start: STANDUP was given (ENABLE is refused until then)
  float jump_forward_m_ = std::numeric_limits<float>::quiet_NaN();
  bool physics_started_ = false;
  bool enable_seen_ = false;
  float push_force_n_ = 0.0F;
  double push_until_s_ = 0.0;
  bool quit_ = false, stdin_closed_ = false;
  std::string input_buffer_;

  static constexpr float kKtNmPerA = 0.47F;
  static constexpr float kCurrentLimitA = 8.0F;
  static constexpr float kCurrentCompGain = 0.2F;
  static constexpr float kCurrentLagS = 0.020F;
  static constexpr float kJointKp = 12.0F;
  static constexpr float kJointKd = 0.8F;
  static constexpr float kTorqueLimitNm = kKtNmPerA * kCurrentLimitA;
  // Provisional GDS68 POS-loop model (see updateController): kPosGainPerS * kVelGainNmSPerRad == kJointKp and
  // kVelGainNmSPerRad == kJointKd. Driver Set_Limits: 20 rotor turns/s over the 8:1 gear.
  static constexpr float kVelGainNmSPerRad = kJointKd;
  static constexpr float kPosGainPerS = kJointKp / kJointKd;
  static constexpr float kVelIntegratorGain = 5.0F;
  static constexpr float kDriverVelLimitRadS = 20.0F * 6.28318530718F / 8.0F;
};
}  // namespace

int main(int argc, char** argv) {
  try {
    GimV3Runner runner(parseOptions(argc, argv));
    return runner.run();
  } catch (const std::exception& error) {
    std::cerr << "ERROR|" << error.what() << std::endl;
    return 2;
  }
}
