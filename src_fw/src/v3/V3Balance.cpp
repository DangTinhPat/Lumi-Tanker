#include "tap_fw/v3/V3Controller.hpp"

#include "robot_control/BodyClearance.hpp"

#include <algorithm>
#include <cmath>

#include "robot_control/GeneratedGeometry.hpp"
#include "robot_control/TunedConfig.hpp"
#include "robot_control/WholeBodyModel.hpp"
#include "tap_fw/BoardConfig.hpp"

namespace tap_fw::v3 {

// Wheel balance laws: PID fallback, LQR, whole-body mode and their guards.

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
  // Operator speed ramps at drive_accel_m_s2 instead of stepping: a step to
  // the demo stick speed (0.55 m/s) felled the sim robot every time.
  const float cmd_v = std::clamp(cmd.forward_m_s, -vmax, vmax);
  const float dv_max = std::max(p.drive_accel_m_s2, 1.0e-3F) * dt;
  pid_drive_v_ += std::clamp(cmd_v - pid_drive_v_, -dv_max, dv_max);
  const float target_v = pid_drive_v_;
  // Driving tracks speed only: the position target follows the robot, and on
  // release it holds where the robot stopped. Integrating the target at the
  // commanded speed (as v3.1 did at 0.15 m/s) let the robot fall behind it
  // and then overshoot to catch up - 0.5 m/s commanded reached 0.96 m/s,
  // past the 0.8 m/s cap, and fell in the sim.
  if (std::abs(target_v) > 1.0e-3F || std::abs(cmd_v) > 1.0e-3F) {
    pid_x_target_ = pid_x_;
    pid_driving_ = true;
  } else if (pid_driving_) {
    pid_x_target_ = pid_x_;
    pid_driving_ = false;
  }
  // Horizontal specific force = body acceleration (gravity is vertical).
  const float a_h = in.accel_x_m_s2 * std::cos(body_pitch_) +
      in.accel_z_m_s2 * std::sin(body_pitch_);
  body_accel_f_ += std::clamp(2.0F * 3.14159265F * 5.0F * dt, 0.0F, 1.0F) *
      (a_h - body_accel_f_);
  // The body moves on its own again (released on the floor): outer loop on.
  // Robot accelerometer noise: std 0.065 m/s^2 still, 0.18 with the wheels
  // spinning in the air (2026-09-27 log), hence 5 Hz and 50 ms sustained.
  // Measured as the change of the raw body-frame accel_x since the held
  // re-zero, not the horizontal force through the pitch estimate: a still
  // body keeps its accel_x, while the gyro-dominant estimate drifts while
  // held and g * 0.015 rad of drift already read as "free" (two-body sim:
  // left the held state in the air 0.24 s after every re-zero, wheels
  // raced again).
  held_free_ticks_ = std::abs(ax_f_ - held_ax_ref_) > 0.15F ? held_free_ticks_ + 1 : 0;
  if (held_outer_off_ && (std::abs(body_rate_) > 0.15F || held_free_ticks_ >= 25)) {
    held_outer_off_ = false;
    pid_seed_ = pid_theta_b_ = releaseBalance(in) - ffPitch();
  }
  if (held_outer_off_) {
    // Held (stand, hands): nothing to balance, so the balance angle tracks
    // the body. Otherwise the height feed-forward (or any small offset) is
    // integrated into a slow wheel run-away; on release the pitch at that
    // moment becomes the seed.
    pid_seed_ = pid_theta_b_ = in.pitch_rad - ffPitch();
    pid_v_filt_ = 0.0F;
    pid_x0_ = ground_x_;
    pid_x_ = pid_x_target_ = 0.0F;
  }
  const float ex = pid_x_ - pid_x_target_;
  updateLearnedTrim(in, ex, pid_theta_b_ + ffPitch());
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
  // the balance angle moves with the height (CAD mass model).
  pid_ref_ = pid_theta_b_ + ffPitch() -
      std::clamp(u, -p.corr_max_rad, p.corr_max_rad);
  // Inner loop: nose-down (+pitch) error -> accelerate forward under it.
  const float accel = std::clamp(p.kp * (in.pitch_rad - pid_ref_) + p.kd * in.pitch_rate_rad_s,
                                 -p.accel_max_m_s2, p.accel_max_m_s2);
  const float wheel_ground = radius * 0.5F * (in.wheel_rad_s[0] + in.wheel_rad_s[1]);
  pid_cmd_m_s_ = std::clamp(pid_cmd_m_s_ + accel * dt, -vmax, vmax);
  // Held: nothing is balanced, so leak the command to zero (0.5 s). Without
  // it the uncorrected gyro bias (x kd) crept the wheels to ~2.7 rad/s on the
  // stand (2026-09-28 log). Release is caught by the pitch-rate / body-accel
  // exit above, which switches the full loop back on.
  if (held_outer_off_) pid_cmd_m_s_ -= pid_cmd_m_s_ * std::min(dt / 0.5F, 1.0F);
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
  // Legs moving (and shortly after): wheel commands change without the body
  // rotating by design, so the held test is suspended.
  if (legs_moved_us_ != 0U && in.now_us - legs_moved_us_ < config_.height.held_suppress_us) {
    held_window_start_us_ = 0U;
  }
  held_max_rate_ = std::max({held_max_rate_, std::abs(body_rate_), std::abs(theta_v_rate_)});
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
    // Held: no deadband push, so a near-zero command leaves the wheels
    // still, and no race in the air (Config::held_wheel_cap_m_s).
    if (held_outer_off_) {
      const float cap = std::max(config_.held_wheel_cap_m_s, 0.0F) / radius;
      return std::clamp(w, -cap, cap);
    }
    if (p.deadband_comp_rad_s <= 0.0F) return w;
    return w + p.deadband_comp_rad_s * std::tanh(w / std::max(p.deadband_eps_rad_s, 1.0e-3F));
  };
  return {comp(sign * (sent - yaw) / radius), comp(sign * (sent + yaw) / radius)};
}

float V3Controller::lqrInertiaLength() noexcept {
  // I_axle / (m l) of the whole robot at the commanded pose (CAD masses and
  // inertia; wheels roll with the axle). Recomputed only when the pose moves.
  if (std::abs(pose_.knee_offset_rad - lqr_leff_knee_) > 1.0e-3F ||
      std::abs(pose_.hip_offset_rad - lqr_leff_hip_) > 1.0e-3F) {
    lqr_leff_knee_ = pose_.knee_offset_rad;
    lqr_leff_hip_ = pose_.hip_offset_rad;
    const float hip = LegHeight::hangHipCad() + pose_.hip_offset_rad;
    const auto b = robot_control::WholeBodyModel{}.compute(hip, pose_.knee_offset_rad, hip,
                                                           pose_.knee_offset_rad);
    const float l = std::hypot(b.com_offset_x_m, b.com_above_axle_m);
    if (l > 0.01F && b.pitch_inertia_at_axle_kg_m2 > 0.0F) {
      lqr_leff_m_ = b.pitch_inertia_at_axle_kg_m2 /
          (robot_control::geometry::total_mass_kg * l);
    }
  }
  return lqr_leff_m_;
}

void V3Controller::resetGuard() {
  guard_.reset();
  guard_out_ = {};
  guard_raise_m_ = 0.0F;
  if (guard_scale_active_) {
    lqr_.setWheelPitchFeedbackScale(guard_scale_base_);
    guard_scale_active_ = false;
  }
}

// OscillationGuard.hpp: measures the pitch swings and turns the risk into the four reactions (see Config::Guard).
void V3Controller::updateGuard(const Input& in, float reference) {
  const Guard& g = config_.guard;
  if (g.mode <= 0 || state_ != State::Balancing || held_outer_off_ || held_quiet_) {
    resetGuard();
    return;
  }
  guard_out_ = guard_.update(in.pitch_rad - reference, in.dt_s, g.detect);
  const float risk = guard_out_.risk;
  const float dt = std::max(in.dt_s, 1.0e-4F);
  if (g.mode < 2) {
    if (guard_scale_active_) {
      lqr_.setWheelPitchFeedbackScale(guard_scale_base_);
      guard_scale_active_ = false;
    }
    guard_raise_m_ = 0.0F;
    return;
  }
  // 1) loop gain: a delayed loop that swings is usually over-gained at its crossover.
  if (!guard_scale_active_) guard_scale_base_ = lqr_.config().wheel_pitch_feedback_scale;
  if (risk > 0.02F) {
    guard_scale_active_ = true;
    lqr_.setWheelPitchFeedbackScale(guard_scale_base_ * (1.0F - std::clamp(g.gain_cut, 0.0F, 0.5F) * risk));
  } else if (guard_scale_active_) {
    lqr_.setWheelPitchFeedbackScale(guard_scale_base_);
    guard_scale_active_ = false;
  }
  // 2) body height: sustained risk lifts the body slowly (a taller pendulum has more delay margin); it is given
  // back more slowly still once the loop is calm.
  if (risk > g.raise_risk) {
    guard_raise_m_ = std::min(guard_raise_m_ + g.raise_rate_m_s * dt, std::max(g.raise_max_m, 0.0F));
  } else if (risk < 0.1F) {
    guard_raise_m_ = std::max(guard_raise_m_ - g.release_rate_m_s * dt, 0.0F);
  }
}

std::array<float, 2> V3Controller::lqrUpdate(const Input& in, const Command& cmd) {
  const Pid& p = config_.pid;  // shared held/lifted thresholds and deadband comp
  const float radius = std::max(config_.wheel_radius_m, 1.0e-4F);
  const float vmax = config_.lqr.max_ground_velocity_m_s;
  // The pendulum length tracks the current leg height every tick; with the gain
  // table built this is a cheap interpolation.
  lqr_.setComHeight(config_.lqr_inertia_length
      ? lqrInertiaLength() / std::max(config_.lqr.pendulum_length_per_com_height, 0.1F)
      : pose_.com_above_axle_m);
  updateGuard(in, lqr_.balanceTrim() + ffPitch());
  if (held_quiet_) {
    const float wheels = radius * 0.5F * (in.wheel_rad_s[0] + in.wheel_rad_s[1]);
    const bool tilted = std::abs(in.pitch_rad - (lqr_.balanceTrim() + ffPitch())) > 0.03F;
    const bool stopped = std::abs(wheels) < 0.05F &&
        in.now_us - held_quiet_since_us_ >= 200'000U;
    if (!tilted && !stopped) {
      lqr_.reset(ground_x_, true);
      lqr_x_target_m_ = ground_x_;
      stand_give_m_ = 0.0F;
      held_window_start_us_ = 0U;
      return {0.0F, 0.0F};
    }
    held_quiet_ = false;
  }
  const float target_v = std::clamp(cmd.forward_m_s, -vmax, vmax);
  // Driving moves the position anchor. It moves at LqrBalancer's own slewed
  // velocity target (last tick's), not the raw command, so the position term does
  // not fight the ramp at every start and stop. It keeps moving through the stop
  // ramp, then waits for the robot itself to stop (it lags the ramp) before
  // re-anchoring where it actually is.
  const float v_ref = lqr_.debugState().lqr_velocity_reference_m_s;
  if (std::abs(target_v) > 1.0e-3F || std::abs(v_ref) > 1.0e-3F) {
    lqr_x_target_m_ += stand_give_m_ + v_ref * in.dt_s;  // drive from where it is
    stand_give_m_ = 0.0F;
    // Leash: a climb or a stall must not bank position error for a catch-up
    // sprint past the wheel cap (sim TIER_ROUGH). Driving tracks the speed; the
    // anchor stays near the robot.
    const float leash = std::max(config_.drive_leash_m, 0.0F);
    if (leash > 0.0F) lqr_x_target_m_ = std::clamp(lqr_x_target_m_, ground_x_ - leash, ground_x_ + leash);
    pid_driving_ = true;
  } else if (pid_driving_ && std::abs(pid_v_filt_) < 0.02F) {
    lqr_x_target_m_ = ground_x_;
    pid_driving_ = false;
  }
  // Standing leash (Config::stand_leash_m): while a push runs the wheels near
  // their cap, the anchor the LQR sees gives way to within the leash of the robot
  // (stand_give_m_ off the station) and afterwards walks back at stand_return_m_s.
  // Slower motion (a hand pressing) keeps the full spring reach. Not for
  // stand_leash_release_s after an engage or release: a wrong seed trim runs the
  // wheels to the cap too, and there the spring is the brake (sim V4).
  const float stand_leash = std::max(config_.stand_leash_m, 0.0F);
  if (!pid_driving_ && stand_leash > 0.0F) {
    const bool fast = std::abs(pid_v_filt_) >= std::max(config_.stand_leash_from, 0.0F) * vmax;
    stand_leash_wait_s_ = std::max(stand_leash_wait_s_ - in.dt_s, 0.0F);
    // Latched until the robot has stopped: it still rolls ~0.3 m while
    // braking from the gate speed, and that is not station error either.
    if (fast && stand_leash_wait_s_ <= 0.0F) stand_leash_on_ = true;
    if (std::abs(pid_v_filt_) < 0.05F) stand_leash_on_ = false;
    if (stand_leash_on_) {
      stand_give_m_ = std::clamp(lqr_x_target_m_ + stand_give_m_, ground_x_ - stand_leash,
                                 ground_x_ + stand_leash) - lqr_x_target_m_;
    } else {
      const float step = std::max(config_.stand_return_m_s, 0.0F) * in.dt_s;
      stand_give_m_ -= std::clamp(stand_give_m_, -step, step);
    }
  } else {
    stand_give_m_ = 0.0F;
    stand_leash_on_ = false;
  }
  // Horizontal specific force (see pidUpdate): tells a free robot from a held
  // one, shared by both algorithms' held/lifted exit condition.
  const float a_h = in.accel_x_m_s2 * std::cos(body_pitch_) +
      in.accel_z_m_s2 * std::sin(body_pitch_);
  body_accel_f_ += std::clamp(2.0F * 3.14159265F * 5.0F * in.dt_s, 0.0F, 1.0F) *
      (a_h - body_accel_f_);
  held_free_ticks_ = std::abs(ax_f_ - held_ax_ref_) > 0.15F ? held_free_ticks_ + 1 : 0;
  if (held_outer_off_ && (std::abs(body_rate_) > 0.15F || held_free_ticks_ >= 25)) {
    held_outer_off_ = false;
    stand_leash_wait_s_ = std::max(config_.stand_leash_release_s, 0.0F);
    lqr_.setBalanceTrim(releaseBalance(in) - ffPitch());
    // Free again: walk the trim from the hold angle to the learned balance
    // angle (Config::learn_release_s) instead of balancing around wherever
    // the operator happened to put it down.
    trim_blend_ = learned_trim_valid_ && tierVirtual();
    if (trim_blend_) {
      trim_blend_rate_ = std::max(
          std::abs(learned_trim_rad_ - ffPitch() - lqr_.balanceTrim()) /
              std::max(config_.learn_release_s, 0.01F),
          0.01F);
    }
  }
  if (held_outer_off_) {
    // Held (stand, hands): the balance angle tracks the body and the anchor
    // stays here, so nothing is spent balancing a body that isn't falling.
    lqr_.setBalanceTrim(in.pitch_rad - ffPitch());
    lqr_x_target_m_ = ground_x_;
    stand_give_m_ = 0.0F;
  } else if (trim_blend_) {
    const float goal = learned_trim_rad_ - ffPitch();
    const float d = goal - lqr_.balanceTrim();
    const float step = trim_blend_rate_ * in.dt_s;
    if (std::abs(d) <= step) {
      lqr_.setBalanceTrim(goal);
      trim_blend_ = false;
    } else {
      lqr_.setBalanceTrim(lqr_.balanceTrim() + std::copysign(step, d));
    }
  } else if (!pid_driving_ &&
             std::abs(in.pitch_rad - (lqr_.balanceTrim() + ffPitch())) < p.corr_max_rad) {
    // Slow integral of the ground-position error into the balance trim (same job
    // as pidUpdate's kix): the LQR's own velocity-error trim can settle at zero
    // with a nonzero position offset, so the position error is integrated instead.
    // Frozen outside corr_max_rad of the current trim: a large seed error means
    // the regulator is still catching the initial transient, and winding the trim
    // up then overshoots (physics-sim T15). With the standing leash giving way,
    // the integral sees the error to the anchor the LQR is given; learning still
    // measures from the station.
    const float ex_station = ground_x_ - lqr_x_target_m_;
    const float ex = ex_station - stand_give_m_;
    lqr_.setBalanceTrim(lqr_.balanceTrim() - config_.lqr_trim_gain * ex * in.dt_s);
    if (config_.trim_acc && tierVirtual()) {
      // Ground acceleration from the wheel-derived ground speed (1 Hz then 0.7 Hz
      // low-pass); the trim follows theta_v - atan(a/g) (Config::trim_acc).
      if (!ga_init_) {
        ga_v1_ = ga_prev_ = ground_v_;
        ga_f_ = 0.0F;
        ga_init_ = true;
      }
      const float dt = std::max(in.dt_s, 1.0e-4F);
      ga_v1_ += std::clamp(2.0F * 3.14159265F * 1.0F * dt, 0.0F, 1.0F) * (ground_v_ - ga_v1_);
      const float acc_raw = (ga_v1_ - ga_prev_) / dt;
      ga_prev_ = ga_v1_;
      ga_f_ += std::clamp(2.0F * 3.14159265F * 0.7F * dt, 0.0F, 1.0F) * (acc_raw - ga_f_);
      if (std::abs(ga_f_) < config_.trim_acc_max_a_m_s2 && std::abs(in.pitch_rate_rad_s) < 0.3F) {
        const float target = in.pitch_rad - std::atan(ga_f_ / 9.80665F) - ffPitch();
        const float cur = lqr_.balanceTrim();
        lqr_.setBalanceTrim(cur + std::clamp(dt / std::max(config_.trim_acc_tau_s, 0.2F), 0.0F, 1.0F) *
                                      (target - cur));
      }
    } else {
      ga_init_ = false;
    }
    updateLearnedTrim(in, ex_station, lqr_.balanceTrim() + ffPitch());
  }
  lqr_.reanchorPosition(lqr_x_target_m_ + stand_give_m_);

  // LqrBalancer::Config::wheel_velocity_reference_from_measured is true, so the
  // noisy encoder-derived ground_v_ feeds the state vector's velocity term
  // directly; with the stiff velocity cost that noise is amplified into the solved
  // acceleration and, through the wheel-lag lead, into the sent command (jerky
  // chatter on the robot). Filter it with the same 5 Hz low-pass as the PID path
  // (pid_v_filt_).
  const float v_alpha = std::clamp(2.0F * 3.14159265F * p.speed_lpf_hz * in.dt_s, 0.0F, 1.0F);
  pid_v_filt_ += v_alpha * (ground_v_ - pid_v_filt_);

  robot_control::RobotState state{};
  state.dt_s = in.dt_s;
  state.floor_contact = true;
  state.imu.pitch_rad = in.pitch_rad;
  state.imu.pitch_rate_rad_s = in.pitch_rate_rad_s;
  state.wheel.position_m = ground_x_;
  state.wheel.velocity_m_s = pid_v_filt_;

  // The guard may switch the prediction on and stretch its lag. Keep the command history filled whenever the guard
  // can act, so a switch-on finds a valid queue instead of an empty one.
  const bool guard_acts = config_.guard.mode >= 2;
  auto predictor = config_.lqr_prediction;
  predictor.enabled = true;
  predictor.lag_s += guard_acts ? std::max(config_.guard.lag_boost_s, 0.0F) * guard_out_.risk : 0.0F;
  const bool guard_predict = guard_acts && guard_out_.risk > config_.guard.predict_risk;
  if ((config_.lqr_prediction.enabled || guard_predict) && !held_outer_off_ && !held_quiet_) {
    const float reference = lqr_.balanceTrim() + ffPitch();
    const float carrier = radius * (body_rate_ + .5F *
        (in.hip_rate_rad_s[0] + in.hip_rate_rad_s[1] +
         in.knee_rate_rad_s[0] + in.knee_rate_rad_s[1]));
    const auto future = lqr_prediction_.predict(
        {ground_x_, in.pitch_rad-reference, ground_v_, in.pitch_rate_rad_s},
        predictor, in.dt_s,
        config_.lqr_inertia_length ? lqrInertiaLength() :
            pose_.com_above_axle_m * config_.lqr.pendulum_length_per_com_height,
        carrier, std::min(config_.lqr.max_ground_accel_m_s2,
                          config_.lqr.wheel_ground_mu * 9.81F));
    state.wheel.position_m = future[0];
    state.wheel.velocity_m_s = future[2];
    state.imu.pitch_rad = future[1] + reference;
    state.imu.pitch_rate_rad_s = future[3];
  } else if (!guard_acts || held_outer_off_ || held_quiet_) {
    lqr_prediction_.reset();
  }

  robot_control::ControlCommand command{};
  command.target_forward_velocity_m_s = held_outer_off_ ? 0.0F : target_v;
  command.target_yaw_rate_rad_s = cmd.yaw_rad_s;

  // Maneuver preview (Tier::preview): the balancer tracks the plan preview_s late,
  // leaning by atan(a/g) with the plan's acceleration fed forward; the wheels get
  // the plan preview_s early (wheel_advance, below).
  const bool preview = config_.tier.preview && tierVirtual();
  lqr_.setExternalShaping(preview);
  float lean_ff = 0.0F, wheel_advance = 0.0F, accel_ff = 0.0F;
  if (preview) {
    const float plan_cap = planCap(vmax);
    updatePlan(held_outer_off_ ? 0.0F : std::clamp(target_v, -plan_cap, plan_cap), in.dt_s,
               held_outer_off_);
    const PlanSample now = planDelayed();
    command.target_forward_velocity_m_s = now.v;
    accel_ff = now.a;
    lean_ff = std::atan(now.a / 9.80665F);
    // Contact point ahead of/behind the CoM by h*a/g: while the lean
    // builds, the wheels run slower (accelerating) or faster (braking) than
    // the body by h*jerk/g - the only speed dip/bump a planned maneuver needs.
    const float h = std::max(pose_.com_above_axle_m, 0.03F);
    wheel_advance = (plan_v_ - now.v) - h / 9.80665F * plan_j_;
  }
  lqr_.setAccelFeedForward(accel_ff);

  // integrate=false while held: LqrBalancer decays the commanded speed to zero on
  // its own (~0.1 s) and freezes the balance-trim adaptation.
  const bool integrate = !held_outer_off_;
  const auto lqr_out = lqr_.update(state, command, integrate, ffPitch() + lean_ff, true);
  // updateHeight's balance-quality gate reads the pitch reference from pid_ref_.
  pid_ref_ = lqr_.debugState().lqr_pitch_ref_rad;

  // Held/lifted: over a window the command changed but the body never
  // rotated (a free pendulum would have) - same test as pidUpdate, tracking
  // the LQR's integrated ground-speed command instead of pid_cmd_m_s_.
  if (legs_moved_us_ != 0U && in.now_us - legs_moved_us_ < config_.height.held_suppress_us) {
    held_window_start_us_ = 0U;
  }
  const float lqr_cmd = lqr_.debugState().commanded_ground_velocity_m_s;
  held_max_rate_ = std::max({held_max_rate_, std::abs(body_rate_), std::abs(theta_v_rate_)});
  held_accel_sum_ += std::abs(body_accel_f_);
  ++held_accel_n_;
  if (held_window_start_us_ == 0U) {
    held_window_start_us_ = in.now_us;
    held_v0_ = lqr_cmd;
    held_max_rate_ = 0.0F;
    held_accel_sum_ = 0.0F;
    held_accel_n_ = 0;
  } else if (in.now_us - held_window_start_us_ >= p.held_window_us) {
    const float mean_accel = held_accel_sum_ / static_cast<float>(std::max(held_accel_n_, 1));
    const bool held = std::abs(lqr_cmd - held_v0_) > p.held_dv_m_s &&
        held_max_rate_ < p.held_rate_rad_s && mean_accel < p.held_accel_m_s2 &&
        !pid_driving_;
    held_window_start_us_ = 0U;
    if (held) {
      heldRezero(in);
      return {0.0F, 0.0F};
    }
  }

  // Wheel-lag lead: the BL4820 wheels are a first-order speed servo lagging the
  // command, and the raw LQR model assumes instant ground-acceleration authority;
  // without the lead the loop rings. The sent speed leads the LQR's solved
  // acceleration by lead_s (height-adaptive, Config::lqr_lead_adaptive). The
  // led-augmented ground speed is clamped back to vmax before conversion to wheel
  // rad/s: the lead is added after LqrBalancer's own clamp, and an unbounded lead
  // commanded wheel speeds far beyond what the wheels can track.
  //
  // The solved acceleration is noisy (gyro rate and wheel-speed jitter through a
  // stiff velocity cost), so it is low-passed at 20 Hz: well above the body
  // dynamics (few Hz) and the lag the lead compensates.
  const float lead_alpha = std::clamp(2.0F * 3.14159265F * 20.0F * in.dt_s, 0.0F, 1.0F);
  // The plan's own acceleration is advanced exactly by wheel_advance; the lead
  // only covers the feedback part.
  lqr_lead_filt_ += lead_alpha *
      (lqr_.debugState().optimal_command_m_s - accel_ff - lqr_lead_filt_);
  const float base_ground_v = lqr_out.ground_velocity_m_s;
  float lead_s = config_.lqr_lead_s;
  if (config_.lqr_lead_adaptive) {
    const float span = std::max(config_.lqr_lead_ratio_hi - config_.lqr_lead_ratio_lo, 1.0e-3F);
    const float w = std::clamp((config_.lqr_lead_ratio_hi - adaptive_ratio_) / span, 0.0F, 1.0F);
    lead_s = config_.lqr_lead_tall_s + (config_.lqr_lead_low_s - config_.lqr_lead_tall_s) * w;
  }
  lqr_lead_eff_ = lead_s;
  const float lead_v = lqr_lead_filt_ * lead_s + wheel_advance;
  const float sent_ground_v = std::clamp(base_ground_v + lead_v, -vmax, vmax);
  const float lead_wheel = config_.lqr.left_wheel_sign *
      (sent_ground_v - base_ground_v) / radius;
  // The LQR path has its own (smaller) deadband push: the tanh step adds loop
  // gain at every zero crossing, which is where the delayed-wheel wobble lives.
  const float dbc = config_.lqr_deadband_comp_rad_s;
  auto comp = [&](float w) {
    if (held_outer_off_) {
      const float cap = std::max(config_.held_wheel_cap_m_s, 0.0F) / radius;
      return std::clamp(w, -cap, cap);
    }
    if (dbc <= 0.0F) return w;
    return w + dbc * std::tanh(w / std::max(p.deadband_eps_rad_s, 1.0e-3F));
  };
  auto& carrier = wheel_carrier_rad_s_;
  if (tierVirtual() && config_.tier.wheel_carrier_compensation && !held_outer_off_ && !held_quiet_) {
    for (std::size_t side = 0; side < 2U; ++side) {
      // Native rolling kinematics: v_axle/R = wheel_relative + body_rate
      // + hip_rate + knee_rate. The input side already ADDS this carrier
      // in ground_v_. A partial inverse here informs the wheel servo of
      // carrier motion without adding full instantaneous feed-forward to
      // a delayed, already closed feedback loop. Use body gyro, not theta_v
      // rate (which contains a different COM Jacobian). Bound the measured
      // carrier at 3 rad/s, so the default correction is <=0.012 m/s.
      carrier[side] = wheel_leg_alpha_ * std::clamp(config_.tier.wheel_carrier_gain, 0.0F, 1.0F) *
          config_.lqr.left_wheel_sign * std::clamp(
          body_rate_ + in.hip_rate_rad_s[side] + in.knee_rate_rad_s[side], -3.0F, 3.0F);
    }
  }
  const std::array<float, 2> sent{
      comp(lqr_out.left_wheel_velocity_rad_s + lead_wheel - carrier[0]),
      comp(lqr_out.right_wheel_velocity_rad_s + lead_wheel - carrier[1])};
  {
    const float sent_speed = config_.lqr.left_wheel_sign * radius * .5F * (sent[0]+sent[1]);
    if (config_.guard.mode >= 2) {
      auto predictor = config_.lqr_prediction;
      predictor.enabled = true;
      lqr_prediction_.record(sent_speed, predictor, in.dt_s);
    } else {
      lqr_prediction_.record(sent_speed);
    }
  }
  return sent;
}

void V3Controller::wbReset(const Input& in) {
  (void)in;
  wb_x_ref_ = ground_x_;
  wb_xi_ = 0.0F;
  wb_theta_rate_ = 0.0F;
  wb_cmd_v_ = wb_cmd_f_ = 0.0F;
  wb_soft_ = 0.0F;
  wb_wheel_hist_.fill(0.0F);
  wb_hip_hist_.fill(0.0F);
  wb_knee_hist_.fill(0.0F);
  wb_sat_sign_ = 0;
  wb_sat_flips_.fill(0U);
  wb_sat_head_ = 0U;
  wb_ring_fault_ = false;
}

V3Controller::WbGain V3Controller::wbGain() const noexcept {
  const float h = pose_.axle_below_hip_m > 0.0F ? pose_.axle_below_hip_m : LegHeight::maxHeight();
  const WbTable& table = *wb_table_;
  std::size_t i = 0U;
  while (i + 2U < table.size() && table[i + 1U].height_m < h) ++i;
  const auto& lo = table[i];
  const auto& hi = table[i + 1U];
  const float a = std::clamp((h - lo.height_m) / std::max(hi.height_m - lo.height_m, 1e-4F),
                             0.0F, 1.0F);
  // The soft table shares the heights and equilibria (same model).
  const WbTable& soft = *wb_table_soft_;
  return {lo.theta0 + a * (hi.theta0 - lo.theta0), &lo.K, &hi.K, a, &soft[i].K, &soft[i + 1U].K};
}

std::array<float, 2> V3Controller::wbUpdate(const Input& in, const Command& cmd) {
  const Pid& p = config_.pid;  // shared held/lifted thresholds
  const float radius = std::max(config_.wheel_radius_m, 1.0e-4F);
  const float vmax = config_.lqr.max_ground_velocity_m_s;
  const WbGain g = wbGain();
  // Natural body pitch from the measured pose (Tier::level on): standing
  // still, the CoM is over the axle whatever the model says, so the IMU
  // pitch plus the CoM-line angle of the MEASURED legs is the equilibrium
  // lean as this robot reads it (IMU mount + CoM / leg-zero error). The
  // natural pitch at the commanded pose follows from it exactly. The old
  // route - learning it from the position error - took 9-11 s and left
  // 0.005 rad (sim, CoM 15 mm off or legs arranged 0.1 rad off); an extra
  // integral on the hip operating point fought the LQR (it rolled away).
  if (config_.tier.level && legs_measured_) {
    const float lean_meas = body_pitch_ +
        comLine(0.5F * (in.hip_rad[0] + in.hip_rad[1]), 0.5F * (in.knee_rad[0] + in.knee_rad[1])).angle;
    const bool quiet = !held_outer_off_ && !held_quiet_ && !pid_driving_ &&
        std::abs(ground_v_) < 0.05F && std::abs(body_rate_) < 0.15F &&
        std::abs(plan_a_) < 1.0e-3F && std::abs(wb_cmd_v_) < 0.3F * vmax;
    if (!wb_lean_init_) {
      wb_lean_lp_ = lean_meas;
      wb_lean_init_ = true;
    } else if (quiet) {
      wb_lean_lp_ += std::min(in.dt_s / std::max(config_.tier.level_tau_s, 0.05F), 1.0F) *
          (lean_meas - wb_lean_lp_);
    }
    wb_bias_ = std::clamp(wb_lean_lp_ - c_pose_ - g.theta0, -0.2F, 0.2F);
  }
  // Body pitch target: the level loop's (Tier::level, the gimbal) or the
  // model equilibrium at this height.
  const float theta_nat = g.theta0 + wb_bias_;  // natural body pitch as the IMU reads it
  const float theta_target = config_.tier.level ? config_.tier.level_pitch_rad : theta_nat;
  if (held_quiet_) {
    const float wheels = radius * 0.5F * (in.wheel_rad_s[0] + in.wheel_rad_s[1]);
    const bool tilted = std::abs(in.pitch_rad - wb_theta_ref_) > 0.03F;
    const bool stopped = std::abs(wheels) < 0.05F &&
        in.now_us - held_quiet_since_us_ >= 200'000U;
    if (!tilted && !stopped) {
      wbReset(in);
      held_window_start_us_ = 0U;
      return {0.0F, 0.0F};
    }
    held_quiet_ = false;
  }
  const float a_h = in.accel_x_m_s2 * std::cos(body_pitch_) +
      in.accel_z_m_s2 * std::sin(body_pitch_);
  body_accel_f_ += std::clamp(2.0F * 3.14159265F * 5.0F * in.dt_s, 0.0F, 1.0F) *
      (a_h - body_accel_f_);
  held_free_ticks_ = std::abs(ax_f_ - held_ax_ref_) > 0.15F ? held_free_ticks_ + 1 : 0;
  if (held_outer_off_ && (std::abs(body_rate_) > 0.15F || held_free_ticks_ >= 25)) {
    held_outer_off_ = false;
    // Free again: the pitch reference walks from the hold angle back to the
    // table equilibrium (as the tiers' learned-trim blend).
    wb_theta_ref_ = releaseBalance(in);
    wb_theta_rate_ = std::max(std::abs(theta_target - wb_theta_ref_) /
                                  std::max(config_.learn_release_s, 0.01F), 0.01F);
    wbReset(in);
  }
  if (held_outer_off_) {
    // Held: nothing to balance - the reference follows the body and the
    // anchor stays here.
    wb_theta_ref_ = in.pitch_rad;
    wb_x_ref_ = ground_x_;
    wb_xi_ = 0.0F;
  } else if (wb_theta_ref_ != theta_target) {
    const float d = theta_target - wb_theta_ref_;
    const float step = std::max(wb_theta_rate_, 0.01F) * in.dt_s;
    wb_theta_ref_ = std::abs(d) <= step ? theta_target : wb_theta_ref_ + std::copysign(step, d);
  }
  // Along the equilibrium manifold: tilting the body by d moves the CoM line
  // by d, which the hip takes back by -d / J (J = d com_line / d hip).
  const float lmax = std::max(config_.tier.level_max_rad, 0.0F);
  const float q_shift = std::clamp(-(wb_theta_ref_ - theta_nat) / std::max(j_hip_, 0.2F),
                                   -lmax, lmax);

  // Reference: the jerk-limited plan (Tier::preview's), position following it.
  const float target_v = std::clamp(cmd.forward_m_s, -vmax, vmax);
  const float plan_cap = planCap(vmax);
  updatePlan(held_outer_off_ ? 0.0F : std::clamp(target_v, -plan_cap, plan_cap), in.dt_s,
             held_outer_off_);
  // Preview (as the tiers' Tier::preview): the state reference is the plan
  // preview_s late with its lean atan(a/g) given to the hips (the body stays
  // at its target), while the wheels get the plan preview_s early plus the
  // h * jerk / g contact shift that builds the lean.
  const PlanSample late = planDelayed();
  const float v_ref = late.v;
  const float lean_ff = std::atan(late.a / 9.80665F);
  const float h_com = std::max(pose_.com_above_axle_m, 0.03F);
  const float v_wheel_ff = plan_v_ - h_com / 9.80665F * plan_j_;
  wb_x_ref_ += v_ref * in.dt_s;
  // Driving tracks speed only (as the tiers): the anchor is the robot itself
  // and the position integral holds; once the plan and the robot have
  // stopped it holds where it stopped. The integral is kept, not cleared: it
  // carries the static model error (CoM offset, IMU mount). A free-running
  // anchor wound it up while the robot lagged the plan and the robot then
  // ran 0.42 m/s for a 0.30 m/s command; an anchor allowed 5 cm ahead still
  // overshot 0.55 m/s to 0.60/0.70 (two-body sim).
  const float v_alpha = std::clamp(2.0F * 3.14159265F * p.speed_lpf_hz * in.dt_s, 0.0F, 1.0F);
  pid_v_filt_ += v_alpha * (ground_v_ - pid_v_filt_);
  const bool driving = std::abs(v_ref) > 1.0e-3F || std::abs(target_v) > 1.0e-3F;
  // The anchor keeps the position error it had when the drive began. At a
  // stand the error and the held integral balance each other (together they
  // carry the static model error); zeroing the error for the drive left the
  // integral alone - a steady push that held the body 0.02 rad tilted and
  // the speed ~10 % over the command through every cruise (sim).
  if (driving) {
    if (!pid_driving_) wb_drive_ex0_ = std::clamp(ground_x_ - wb_x_ref_, -0.3F, 0.3F);
    const float slack = std::max(config_.wb_drive_slack_m, 0.0F);
    const float anchor = ground_x_ - wb_drive_ex0_;
    wb_x_ref_ = std::clamp(wb_x_ref_, anchor - slack, anchor + slack);
    pid_driving_ = true;
  } else if (pid_driving_ && std::abs(pid_v_filt_) < 0.02F) {
    wb_x_ref_ = ground_x_ - wb_drive_ex0_;
    pid_driving_ = false;
  }
  // State (WholeBodyGains.hpp layout).
  const float q = 0.5F * (in.hip_rad[0] + in.hip_rad[1]);
  const float k = 0.5F * (in.knee_rad[0] + in.knee_rad[1]);
  const float qd = 0.5F * (in.hip_rate_rad_s[0] + in.hip_rate_rad_s[1]);
  const float kd = 0.5F * (in.knee_rate_rad_s[0] + in.knee_rate_rad_s[1]);
  const float q_lean = config_.wb_lean_ff
      ? std::clamp(lean_ff / std::max(j_hip_, 0.2F), -lmax, lmax) : 0.0F;
  const float q0 = pose_.hip_offset_rad + q_shift + q_lean;
  const float k0 = pose_.knee_offset_rad + kneeHold(dhip_);
  // Wheels near their cap (a hard push): balance first. The position
  // anchor gives way toward the robot and the integral holds, so the cap
  // is not spent on getting back to where the push started (the tiers'
  // LQR drifts the same way); it returns from where it stopped.
  const float sat = std::abs(wb_cmd_v_) / std::max(vmax, 1.0e-3F);
  const float give = std::clamp((sat - config_.wb_give_from) /
                                    std::max(1.0F - config_.wb_give_from, 1.0e-3F), 0.0F, 1.0F);
  if (give > 0.0F) {
    wb_x_ref_ += std::min(give * in.dt_s / 0.15F, 1.0F) * (ground_x_ - wb_x_ref_);
  }
  const float ex = ground_x_ - wb_x_ref_;
  if (!held_outer_off_ && !pid_driving_ && give <= 0.0F) {
    wb_xi_ = std::clamp(wb_xi_ + ex * in.dt_s, -1.0F, 1.0F);
    // Slow: the static part of the position error is a wrong natural pitch.
    // Not with the level integral on: both move the hip operating point, and
    // two integrators on one degree of freedom wound against each other
    // (sim, CoM -15 mm: bias +0.09 and still climbing, position integral at
    // its clamp, body 0.008 off).
    if (!config_.tier.level) {
      wb_bias_ = std::clamp(wb_bias_ - config_.wb_bias_gain * ex * in.dt_s, -0.2F, 0.2F);
    }
  }
  std::array<float, wb::kStates> X{};
  X[0] = ex;
  X[1] = in.pitch_rad - wb_theta_ref_;
  X[2] = q - q0;
  X[3] = k - k0;
  X[4] = ground_v_ - v_ref;
  X[5] = in.pitch_rate_rad_s;
  X[6] = qd;
  X[7] = kd;
  X[8] = wb_xi_;
  for (int i = 0; i < wb::kWheelDelay; ++i) X[9 + i] = wb_wheel_hist_[i];
  for (int i = 0; i < wb::kJointDelay; ++i) {
    X[9 + wb::kWheelDelay + i] = wb_hip_hist_[i];
    X[9 + wb::kWheelDelay + wb::kJointDelay + i] = wb_knee_hist_[i];
  }
  // Near the wheel cap the body may help the catch: blend toward the soft
  // table (looser body hold) as the wheel command passes wb_soft_from of the
  // cap - at once going up, back over wb_soft_return_s. The stiff table
  // alone fell where the soft one held (70 ms wheel dead time, backing into
  // a push); the soft one alone lets the body take 2-3x more of every push.
  {
    const float sat = std::abs(wb_cmd_v_) / std::max(vmax, 1.0e-3F);
    const float want = std::clamp((sat - config_.wb_soft_from) /
                                      std::max(1.0F - config_.wb_soft_from - 0.15F, 0.05F),
                                  0.0F, 1.0F);
    wb_soft_ = want >= wb_soft_ ? want
        : std::max(want, wb_soft_ - in.dt_s / std::max(config_.wb_soft_return_s, 0.01F));
  }
  float v[3] = {};
  for (int r = 0; r < 3; ++r) {
    float lo = 0.0F, hi = 0.0F, slo = 0.0F, shi = 0.0F;
    for (int c = 0; c < wb::kStates; ++c) {
      lo += (*g.lo)[r][c] * X[c];
      hi += (*g.hi)[r][c] * X[c];
      if (wb_soft_ > 0.0F) {
        slo += (*g.soft_lo)[r][c] * X[c];
        shi += (*g.soft_hi)[r][c] * X[c];
      }
    }
    const float stiff = lo + g.a * (hi - lo);
    const float soft = slo + g.a * (shi - slo);
    v[r] = -(stiff + wb_soft_ * (soft - stiff));
    if (r == 0) {  // diagnostics: the wheel row's terms (stiff table, low point)
      for (int c = 0; c < 9; ++c) wb_dbg_terms_[static_cast<std::size_t>(c)] = -(*g.lo)[0][c] * X[c];
      float hsum = 0.0F;
      for (int c = 9; c < wb::kStates; ++c) hsum += -(*g.lo)[0][c] * X[c];
      wb_dbg_terms_[9] = hsum;
    }
  }
  const float wmax = vmax / radius;
  const float w_dev = std::clamp(v[0], -wmax, wmax);
  const float hip_dev = std::clamp(v[1], -0.25F, 0.25F);
  const float knee_dev = std::clamp(v[2], -0.25F, 0.25F);
  for (int i = wb::kWheelDelay - 1; i > 0; --i) wb_wheel_hist_[i] = wb_wheel_hist_[i - 1];
  for (int i = wb::kJointDelay - 1; i > 0; --i) {
    wb_hip_hist_[i] = wb_hip_hist_[i - 1];
    wb_knee_hist_[i] = wb_knee_hist_[i - 1];
  }
  const float knee_cmd = std::clamp(k0 + knee_dev, LegHeight::kKneeMinRad,
                                    LegHeight::kKneeMaxRad);
  dhip_ = q_shift + q_lean + hip_dev;
  dknee_ = knee_cmd - pose_.knee_offset_rad;
  const float w = std::clamp(v_wheel_ff / radius + w_dev, -wmax, wmax);
  // The delay registers model what the actuators were really sent: the
  // clipped commands (at full height the knee cannot extend past 0, and the
  // wheels stop at wmax). Recording the unclipped ones told the gains the
  // knee/wheels had done what they could not.
  wb_wheel_hist_[0] = w - v_wheel_ff / radius;
  wb_hip_hist_[0] = hip_dev;
  wb_knee_hist_[0] = knee_cmd - k0;
  // Limit-cycle guard: the wheel command slamming from one cap to the other
  // over and over is not a push (one direction) but the loop ringing
  // through the caps (hardware 2026-10-01: +-20 rad/s at 1.5 Hz). Stop
  // rather than shake the robot apart.
  {
    const int sat = w >= 0.9F * wmax ? 1 : w <= -0.9F * wmax ? -1 : 0;
    if (sat != 0 && sat != wb_sat_sign_) {
      if (wb_sat_sign_ != 0) {
        wb_sat_flips_[wb_sat_head_] = in.now_us;
        wb_sat_head_ = (wb_sat_head_ + 1U) % wb_sat_flips_.size();
        const std::uint64_t oldest = wb_sat_flips_[wb_sat_head_];
        if (oldest != 0U && in.now_us - oldest < config_.wb_ring_window_us) {
          wb_ring_fault_ = true;
        }
      }
      wb_sat_sign_ = sat;
    }
  }
  wb_cmd_v_ = w * radius;

  // Held/lifted: same test as lqrUpdate, on the commanded ground speed as
  // the wheels follow it (their ~0.09 s lag). The raw command jitters with
  // joint-rate noise through the large joint-rate gains, and a 0.25 m/s
  // jitter across the 150 ms window read as "wheels sped up, body did not"
  // - false held re-zeroes in the sim with noisy hip rates.
  wb_cmd_f_ += std::clamp(in.dt_s / 0.09F, 0.0F, 1.0F) * (wb_cmd_v_ - wb_cmd_f_);
  // ...minus what the legs explain: with the legs pinning the body, the
  // wheels speed up UNDER a still body (the hips swing them forward) - the
  // very thing the held test reads as "wheels spin, body does not move".
  // The axle's speed relative to the hip from the measured joint rates
  // (leg Jacobian) is taken off before comparing.
  {
    const auto leg = robot_control::WholeBodyModel::legHeight(LegHeight::hangHipCad() + q, k);
    const float v_leg = leg.d_x_d_hip * qd + leg.d_x_d_knee * kd;
    wb_held_v_ = wb_cmd_f_ - config_.wb_held_leg_sign * v_leg;
  }
  if (legs_moved_us_ != 0U && in.now_us - legs_moved_us_ < config_.height.held_suppress_us) {
    held_window_start_us_ = 0U;
  }
  held_max_rate_ = std::max(held_max_rate_, std::abs(body_rate_));
  held_accel_sum_ += std::abs(body_accel_f_);
  ++held_accel_n_;
  if (held_window_start_us_ == 0U) {
    held_window_start_us_ = in.now_us;
    held_v0_ = wb_held_v_;
    held_max_rate_ = 0.0F;
    held_accel_sum_ = 0.0F;
    held_accel_n_ = 0;
  } else if (in.now_us - held_window_start_us_ >= p.held_window_us) {
    const float mean_accel = held_accel_sum_ / static_cast<float>(std::max(held_accel_n_, 1));
    const bool held = std::abs(wb_held_v_ - held_v0_) > p.held_dv_m_s &&
        held_max_rate_ < p.held_rate_rad_s && mean_accel < p.held_accel_m_s2 &&
        std::abs(target_v) < 1.0e-3F;
    held_window_start_us_ = 0U;
    if (held) {
      heldRezero(in);
      return {0.0F, 0.0F};
    }
  }
  pid_ref_ = wb_theta_ref_;

  const float yaw = config_.lqr.half_wheel_base_m *
      std::clamp(cmd.yaw_rad_s, -config_.lqr.max_yaw_rate_rad_s, config_.lqr.max_yaw_rate_rad_s);
  const float sign = config_.lqr.left_wheel_sign;
  auto cap = [&](float wheel) {
    if (!held_outer_off_) return wheel;
    const float c = std::max(config_.held_wheel_cap_m_s, 0.0F) / radius;
    return std::clamp(wheel, -c, c);
  };
  return {cap(sign * (wb_cmd_v_ - yaw) / radius), cap(sign * (wb_cmd_v_ + yaw) / radius)};
}


}  // namespace tap_fw::v3
