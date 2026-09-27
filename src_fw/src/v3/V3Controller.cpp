#include "tap_fw/v3/V3Controller.hpp"

#include <algorithm>
#include <cmath>

#include "robot_control/GeneratedGeometry.hpp"
#include "robot_control/TunedConfig.hpp"
#include "robot_control/WholeBodyModel.hpp"

namespace tap_fw::v3 {

V3Controller::V3Controller(Config config) : config_(config), lqr_(config.lqr) {
  // One fixed pendulum: the legs never move in v3.
  lqr_.setComHeight(config_.com_height_m);
}

V3Controller::Output V3Controller::fault(const char* reason, const Output& base) {
  state_ = State::Fault;
  last_fault_ = reason;
  Output out = base;
  out.state = state_;
  out.legs_locked = legs_held_;  // wheels stop, legs keep holding
  out.wheels_enabled = false;
  out.wheel_cmd_rad_s = {0.0F, 0.0F};
  out.fault = reason;
  return out;
}

V3Controller::Output V3Controller::update(const Input& in, const Command& cmd) {
  // Axle motion of a rolling wheel on a rigid (locked) leg: R*(wheel + pitch).
  const float mean_rate = 0.5F * (in.wheel_rad_s[0] + in.wheel_rad_s[1]);
  const float mean_pos = 0.5F * (in.wheel_pos_rad[0] + in.wheel_pos_rad[1]);
  ground_v_ = config_.wheel_radius_m * (mean_rate + in.pitch_rate_rad_s);
  ground_x_ = config_.wheel_radius_m * (mean_pos + in.pitch_rad);

  Output out{};
  if (cmd.disable) {
    legs_held_ = false;
    state_ = State::Idle;
    out.state = state_;
    return out;
  }

  out.legs_locked = legs_held_;
  switch (state_) {
    case State::Idle:
    case State::Fault:
      if (cmd.start) {
        if (!in.sensors_ok) return fault("sensors not ready", out);
        state_ = State::Locking;
        state_since_us_ = in.now_us;
        runaway_since_us_ = 0U;
        out.engage_legs = !legs_held_;  // already holding after a fault: keep it
        legs_held_ = true;
        out.legs_locked = true;
      }
      break;
    case State::Locking:
      out.legs_locked = true;
      if (!in.sensors_ok) return fault("sensor feedback lost while locking", out);
      if (in.legs_ready && in.now_us - state_since_us_ >= config_.lock_settle_us) {
        state_ = State::Ready;
        state_since_us_ = in.now_us;
        ready_phase_ = 0;
        ready_entry_pitch_ = in.pitch_rad;
        still_since_us_ = 0U;
        still_n_ = 0;
      }
      break;
    case State::Ready: {
      // Wheels hold still (torque on, zero speed); no balance loop yet.
      out.legs_locked = true;
      out.wheels_enabled = true;
      if (!in.sensors_ok) return fault("sensor feedback lost while ready", out);
      const Engage& e = config_.engage;
      const float rate = std::abs(in.pitch_rate_rad_s);
      bool go = cmd.go || e.immediate;
      float armed_seed = in.pitch_rad;
      if (e.automatic && !go) {
        if (ready_phase_ == 0) {
          if (rate > e.move_rate_rad_s ||
              std::abs(in.pitch_rad - ready_entry_pitch_) > e.move_pitch_rad) {
            ready_phase_ = 1;
            still_since_us_ = 0U;
          }
        } else if (ready_phase_ == 1) {
          if (rate < e.still_rate_rad_s) {
            if (still_since_us_ == 0U) {
              still_since_us_ = in.now_us;
              still_sum_ = 0.0F;
              still_n_ = 0;
            }
            still_sum_ += in.pitch_rad;
            ++still_n_;
            if (in.now_us - still_since_us_ >= e.still_us) {
              ready_phase_ = 2;
              armed_pitch_ = still_sum_ / static_cast<float>(std::max(still_n_, 1));
            }
          } else {
            still_since_us_ = 0U;
          }
        } else if (rate > e.tip_rate_rad_s ||
                   std::abs(in.pitch_rad - armed_pitch_) > e.tip_pitch_rad) {
          go = true;  // released at the standing point: it starts to tip
        }
      }
      if (go) {
        // Balance angle seed: the standing point the operator just held on
        // the floor > a stored BALANCE measurement > the pitch right now.
        engageBalance(in, ready_phase_ == 2 ? armed_pitch_
                          : balance_set_    ? balance_rad_
                                            : armed_seed);
      } else {
        out.state = state_;
        return out;
      }
      break;
    }
    case State::Balancing:
      break;
  }

  if (state_ == State::Balancing) {
    out.legs_locked = true;
    out.wheels_enabled = true;
    if (!in.sensors_ok) return fault("sensor feedback lost while balancing", out);
    if (std::abs(in.pitch_rad) > config_.fall_pitch_rad) return fault("fallen (pitch limit)", out);
    const bool fast = std::abs(in.wheel_rad_s[0]) > config_.runaway_wheel_rad_s &&
        std::abs(in.wheel_rad_s[1]) > config_.runaway_wheel_rad_s;
    if (!fast) {
      runaway_since_us_ = 0U;
    } else if (runaway_since_us_ == 0U) {
      runaway_since_us_ = in.now_us;
    } else if (in.now_us - runaway_since_us_ > config_.runaway_us) {
      // PID: wheels racing without the robot falling means it is held or
      // carried (wheels in the air): re-zero instead of stopping. A real
      // fall is caught by the pitch limit.
      if (config_.algo != Algo::Pid) return fault("wheel runaway", out);
      heldRezero(in);
      out.wheel_cmd_rad_s = {0.0F, 0.0F};
      out.state = state_;
      return out;
    }
    if (config_.algo == Algo::Pid) {
      out.wheel_cmd_rad_s = pidUpdate(in, cmd);
      out.state = state_;
      return out;
    }
    robot_control::RobotState st{};
    st.dt_s = in.dt_s;
    st.imu.sample_dt_s = in.dt_s;
    st.imu.pitch_rad = in.pitch_rad;
    st.imu.pitch_rate_rad_s = in.pitch_rate_rad_s;
    st.imu.yaw_rate_rad_s = in.yaw_rate_rad_s;
    st.wheel.velocity_m_s = ground_v_;
    st.wheel.position_m = ground_x_;
    st.floor_contact = true;
    robot_control::ControlCommand control{};
    control.target_forward_velocity_m_s = cmd.forward_m_s;
    control.target_yaw_rate_rad_s = cmd.yaw_rad_s;
    const auto balance = lqr_.update(st, control, true, 0.0F, false);
    if (balance.fallen) return fault("fallen (balancer)", out);
    out.wheel_cmd_rad_s = {balance.left_wheel_velocity_rad_s,
                           balance.right_wheel_velocity_rad_s};
  }
  out.state = state_;
  return out;
}

void V3Controller::engageBalance(const Input& in, float seed) {
  state_ = State::Balancing;
  state_since_us_ = in.now_us;
  runaway_since_us_ = 0U;
  lqr_.reset(ground_x_, true);
  lqr_.setBalanceTrim(seed);
  pid_seed_ = pid_theta_b_ = pid_ref_ = seed;
  pid_v_filt_ = pid_cmd_m_s_ = 0.0F;
  pid_x0_ = ground_x_;
  pid_x_ = pid_x_target_ = 0.0F;
  pid_driving_ = false;
  body_accel_f_ = 0.0F;
  held_window_start_us_ = 0U;
  held_resets_ = 0;
  held_quiet_ = false;
  held_outer_off_ = false;
}

void V3Controller::heldRezero(const Input& in) {
  ++held_resets_;
  pid_cmd_m_s_ = 0.0F;
  pid_v_filt_ = 0.0F;
  pid_x0_ = ground_x_;
  pid_x_ = pid_x_target_ = 0.0F;
  pid_seed_ = pid_theta_b_ = pid_ref_ = in.pitch_rad;
  held_quiet_ = true;
  held_outer_off_ = true;
  held_quiet_since_us_ = in.now_us;
  held_window_start_us_ = 0U;
  runaway_since_us_ = 0U;
}

std::array<float, 2> V3Controller::pidUpdate(const Input& in, const Command& cmd) {
  const Pid& p = config_.pid;
  const float dt = in.dt_s;
  const float radius = std::max(config_.wheel_radius_m, 1.0e-4F);
  const float vmax = config_.lqr.max_ground_velocity_m_s;
  if (held_quiet_) {
    const float wheels = radius * 0.5F * (in.wheel_rad_s[0] + in.wheel_rad_s[1]);
    const bool tilted = std::abs(in.pitch_rad - pid_theta_b_) > 0.03F;
    const bool stopped = std::abs(wheels) < 0.05F &&
        in.now_us - held_quiet_since_us_ >= 200'000U;
    if (!tilted && !stopped) {
      pid_cmd_m_s_ = pid_v_filt_ = 0.0F;
      pid_x0_ = ground_x_;
      pid_x_ = pid_x_target_ = 0.0F;
      held_window_start_us_ = 0U;
      return {0.0F, 0.0F};
    }
    held_quiet_ = false;
    pid_x0_ = ground_x_;
  }
  const float alpha = std::clamp(2.0F * 3.14159265F * p.speed_lpf_hz * dt, 0.0F, 1.0F);
  pid_v_filt_ += alpha * (ground_v_ - pid_v_filt_);
  pid_x_ = ground_x_ - pid_x0_;
  const float target_v = std::clamp(cmd.forward_m_s, -vmax, vmax);
  // Driving moves the position target; on release, hold where it stopped.
  if (std::abs(target_v) > 1.0e-3F) {
    pid_x_target_ += target_v * dt;
    pid_driving_ = true;
  } else if (pid_driving_) {
    pid_x_target_ = pid_x_;
    pid_driving_ = false;
  }
  // Horizontal specific force = body acceleration (gravity is vertical).
  const float a_h = in.accel_x_m_s2 * std::cos(in.pitch_rad) +
      in.accel_z_m_s2 * std::sin(in.pitch_rad);
  body_accel_f_ += std::clamp(2.0F * 3.14159265F * 5.0F * dt, 0.0F, 1.0F) *
      (a_h - body_accel_f_);
  // The body moves on its own again (released on the floor): outer loop on.
  // Robot accelerometer noise: std 0.065 m/s^2 still, 0.18 with the wheels
  // spinning in the air (2026-09-27 log), hence 5 Hz and 50 ms sustained.
  held_free_ticks_ = std::abs(body_accel_f_) > 0.15F ? held_free_ticks_ + 1 : 0;
  if (held_outer_off_ && (std::abs(in.pitch_rate_rad_s) > 0.15F || held_free_ticks_ >= 25)) {
    held_outer_off_ = false;
  }
  if (held_outer_off_) {
    pid_v_filt_ = 0.0F;
    pid_x0_ = ground_x_;
    pid_x_ = pid_x_target_ = 0.0F;
  }
  const float ex = pid_x_ - pid_x_target_;
  // Outer loop: speed and position error -> lean correction. Too far forward
  // needs a backward (negative) lean, which the inner loop gets by
  // accelerating. theta_b integrates the position error slowly: it is the
  // balance angle the robot can stand still at (conditional integration:
  // frozen while the correction saturates or while driving).
  const float u = held_outer_off_ ? 0.0F : p.kv * (pid_v_filt_ - target_v) + p.kx * ex;
  if (std::abs(u) < p.corr_max_rad && !pid_driving_ && !held_outer_off_) {
    pid_theta_b_ = std::clamp(pid_theta_b_ - p.kix * ex * dt, pid_seed_ - p.ref_max_rad,
                              pid_seed_ + p.ref_max_rad);
  }
  pid_ref_ = pid_theta_b_ - std::clamp(u, -p.corr_max_rad, p.corr_max_rad);
  // Inner loop: nose-down (+pitch) error -> accelerate forward under it.
  const float accel = std::clamp(p.kp * (in.pitch_rad - pid_ref_) + p.kd * in.pitch_rate_rad_s,
                                 -p.accel_max_m_s2, p.accel_max_m_s2);
  const float wheel_ground = radius * 0.5F * (in.wheel_rad_s[0] + in.wheel_rad_s[1]);
  pid_cmd_m_s_ = std::clamp(pid_cmd_m_s_ + accel * dt, -vmax, vmax);
  // Anti-windup: the command may not run ahead of the measured wheel speed
  // by more than the margin (stalled wheel). It is never pulled up to a
  // faster wheel: after a held re-zero the wheels may still be spinning.
  if (pid_cmd_m_s_ > std::max(wheel_ground, 0.0F) + p.tracking_margin_m_s) {
    pid_cmd_m_s_ = std::max(wheel_ground, 0.0F) + p.tracking_margin_m_s;
  } else if (pid_cmd_m_s_ < std::min(wheel_ground, 0.0F) - p.tracking_margin_m_s) {
    pid_cmd_m_s_ = std::min(wheel_ground, 0.0F) - p.tracking_margin_m_s;
  }
  // Held/lifted: over a window the command changed but the body never
  // rotated (a free pendulum would have).
  held_max_rate_ = std::max(held_max_rate_, std::abs(in.pitch_rate_rad_s));
  held_accel_sum_ += std::abs(body_accel_f_);
  ++held_accel_n_;
  if (held_window_start_us_ == 0U) {
    held_window_start_us_ = in.now_us;
    held_v0_ = pid_cmd_m_s_;
    held_max_rate_ = 0.0F;
    held_accel_sum_ = 0.0F;
    held_accel_n_ = 0;
  } else if (in.now_us - held_window_start_us_ >= p.held_window_us) {
    const float mean_accel = held_accel_sum_ / static_cast<float>(std::max(held_accel_n_, 1));
    const bool held = std::abs(pid_cmd_m_s_ - held_v0_) > p.held_dv_m_s &&
        held_max_rate_ < p.held_rate_rad_s && mean_accel < p.held_accel_m_s2 &&
        !pid_driving_;
    held_window_start_us_ = 0U;
    if (held) {
      heldRezero(in);
      return {0.0F, 0.0F};
    }
  }
  const float sent = std::clamp(pid_cmd_m_s_ + accel * p.lead_s, -vmax, vmax);
  const float yaw = config_.lqr.half_wheel_base_m *
      std::clamp(cmd.yaw_rad_s, -config_.lqr.max_yaw_rate_rad_s, config_.lqr.max_yaw_rate_rad_s);
  const float sign = config_.lqr.left_wheel_sign;
  auto comp = [&](float w) {
    if (p.deadband_comp_rad_s <= 0.0F) return w;
    return w + p.deadband_comp_rad_s * std::tanh(w / std::max(p.deadband_eps_rad_s, 1.0e-3F));
  };
  return {comp(sign * (sent - yaw) / radius), comp(sign * (sent + yaw) / radius)};
}

const char* toString(V3Controller::State state) noexcept {
  switch (state) {
    case V3Controller::State::Idle: return "Idle";
    case V3Controller::State::Locking: return "Locking";
    case V3Controller::State::Ready: return "Ready";
    case V3Controller::State::Balancing: return "Balancing";
    case V3Controller::State::Fault: return "Fault";
  }
  return "?";
}

V3Controller::Config defaultV3Config() {
  V3Controller::Config cfg{};
  cfg.lqr = robot_control::tunedControllerConfig().lqr;
  cfg.lqr.wheel_pitch_feedback_scale = 1.0F;
  cfg.lqr.balance_trim_gain_rad_per_m = 0.30F;
  cfg.lqr.balance_trim_max_rad = 0.20F;
  cfg.lqr.balance_trim_pitch_gate_rad = 0.25F;
  cfg.lqr.command_velocity_tracking_margin_m_s = 0.25F;
  cfg.lqr.max_ground_velocity_m_s = 0.8F;
  cfg.wheel_radius_m = robot_control::geometry::wheel_radius_m;
  cfg.fall_pitch_rad = 0.60F;
  // COM height of the legs locked extended (hanging-straight pose).
  robot_control::WholeBodyModel model;
  const float hip = 0.0754F, knee = robot_control::geometry::native_knee_max_rad;
  cfg.com_height_m = model.compute(hip, knee, hip, knee).com_height_m;
  return cfg;
}

robot_control::ImuStateEstimator::Config balancingEstimatorConfig(
    robot_control::ImuStateEstimator::Config base) {
  base.accel_correction_gain_hz = 0.2F;
  base.r_measure = 1.0e6F;        // disables the fast Kalman accelerometer branch
  base.bias_learning_gain = 0.0F; // keep the 0.2 Hz innovation bias branch only
  return base;
}

}  // namespace tap_fw::v3
