#include "tap_fw/v3/V3Controller.hpp"

#include <algorithm>
#include <cmath>

#include "robot_control/GeneratedGeometry.hpp"
#include "robot_control/TunedConfig.hpp"
#include "robot_control/WholeBodyModel.hpp"

namespace tap_fw::v3 {

V3Controller::V3Controller(Config config)
    : config_(config), lqr_(config.lqr),
      com_hang_m_(LegHeight::forKnee(0.0F).com_above_axle_m) {
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

V3Controller::Output V3Controller::update(const Input& raw, const Command& cmd_in) {
  // Roll on: the turn command is slewed (Roll::yaw_accel_rad_s2) so the
  // lean-in can build with the centripetal pull instead of after it.
  Command cmd = cmd_in;
  {
    const float ya = config_.roll.enabled ? config_.roll.yaw_accel_rad_s2 : 0.0F;
    const float target = std::clamp(cmd_in.yaw_rad_s, -config_.lqr.max_yaw_rate_rad_s,
                                    config_.lqr.max_yaw_rate_rad_s);
    if (ya > 0.0F) {
      const float d = ya * raw.dt_s;
      yaw_cmd_ += std::clamp(target - yaw_cmd_, -d, d);
    } else {
      yaw_cmd_ = target;
    }
    cmd.yaw_rad_s = yaw_cmd_;
  }
  // v3.2: pre-solve the LQR gain table over the height range, one point per
  // tick (idempotent once ready: near-zero cost after). Each point costs
  // ~50 ms on the ESP32-S3's single-precision FPU (LqrBalancer.hpp), which
  // blocks the whole single-threaded firmware loop() for that long - fine at
  // Idle/boot (nothing time-critical yet), but doing this during Locking
  // starves the wheel UART feedback past its 20 ms freshness window and
  // faults "sensor feedback lost while locking" almost immediately (found on
  // real hardware 2026-09-29: PID never called this, so it never happened
  // there). Run it unconditionally from the very first tick instead, so the
  // ~12-point/~0.6 s table is done well before the operator finishes
  // arranging the legs and pressing START.
  if (config_.algo == Algo::Lqr) {
    (void)lqr_.buildGainTableStep(config_.lqr_gain_table_points);
  }
  legs_measured_ = raw.legs_measured;
  body_pitch_ = raw.pitch_rad;
  imu_velocity_ = ImuVelocity{};  // set again at the end of a free balancing tick
  // Held/lifted: "did not rotate" needs BOTH the body (IMU) and theta_v
  // still. With tier 2 on, a free robot catching a push keeps its body level
  // (body rate ~0) while its CoM line swings: the body rate alone read that
  // as held in the two-body sim and re-zeroed it mid-catch (fell). The
  // release test ("moves on its own again") stays on the body rate: a held
  // body does not turn whatever the hips do.
  body_rate_ = raw.pitch_rate_rad_s;
  ax_f_ += std::clamp(2.0F * 3.14159265F * 5.0F * raw.dt_s, 0.0F, 1.0F) *
      (raw.accel_x_m_s2 - ax_f_);
  // Axle motion of a rolling wheel on a rigid (locked) leg: R*(wheel + pitch).
  float mean_rate = 0.5F * (raw.wheel_rad_s[0] + raw.wheel_rad_s[1]);
  float mean_pos = 0.5F * (raw.wheel_pos_rad[0] + raw.wheel_pos_rad[1]);
  predictWheels(raw, &mean_rate, &mean_pos);
  // The wheel encoder turns with the shin: add the leg chain (hip + knee)
  // to the axle odometry, as MotorFeedback::writeTo does (v3.1).
  if (!chain_init_) {
    chain_prev_rad_ = raw.leg_chain_rad;
    chain_rate_f_ = 0.0F;
    chain_init_ = true;
  }
  const float chain_rate = raw.dt_s > 0.0F
      ? (raw.leg_chain_rad - chain_prev_rad_) / raw.dt_s : 0.0F;
  chain_prev_rad_ = raw.leg_chain_rad;
  chain_rate_f_ += std::clamp(2.0F * 3.14159265F * 10.0F * raw.dt_s, 0.0F, 1.0F) *
      (chain_rate - chain_rate_f_);
  chain_rate_meas_ = 0.5F * (raw.hip_rate_rad_s[0] + raw.knee_rate_rad_s[0] +
                             raw.hip_rate_rad_s[1] + raw.knee_rate_rad_s[1]);
  // v3.3: the hips swing at up to ~1 rad/s; the 10 Hz filtered difference
  // lags that, so the tiers use the drivers' own velocity estimates.
  ground_v_ = config_.wheel_radius_m *
      (mean_rate + raw.pitch_rate_rad_s +
       (tierVirtual() || wholeBody() ? chain_rate_meas_ : chain_rate_f_));
  ground_x_ = config_.wheel_radius_m * (mean_pos + raw.pitch_rad + raw.leg_chain_rad);

  // v3.3 tier 1: everything below (engage, balance loop, fall, runaway and
  // held checks, height quality) works on the virtual pitch when enabled.
  updateVirtualPitch(raw);
  Input in = raw;
  in.pitch_rad = theta_v_;
  in.pitch_rate_rad_s = theta_v_rate_;
  if (state_ != State::Balancing) {
    sus_vz_ = sus_dz_ = sus_dk_ = 0.0F;
    wb_lean_init_ = false;
    if (excite_applied_) {
      knee_excite_rad_ = 0.0F;
      excite_applied_ = false;
      pose_ = LegHeight::forKnee(knee_cmd_rad_);
    }
  }
  if (state_ != State::Balancing) {
    updateTrunk(in, false, {0.0F, 0.0F});
    const float step = std::max(config_.tier.rate_rad_s, 0.0F) * in.dt_s;
    dknee_ -= std::clamp(dknee_, -step, step);  // whole-body knee offset back to the pose
  }

  Output out{};
  if (cmd.disable) {
    legs_held_ = false;
    dhip_ = dknee_ = trunk_alpha_ = level_beta_ = 0.0F;  // legs released: no offsets left over
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
        if (!legs_held_) {
          // Fresh lock: legs go to their zero, so the height starts at max.
          height_target_m_ = LegHeight::maxHeight();
          knee_target_rad_ = knee_cmd_rad_ = 0.0F;
          height_speed_m_s_ = 0.0F;
          pose_ = LegHeight::forKnee(0.0F);
        }
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
      const bool armed_only = e.wb_armed_only && config_.algo == Algo::WholeBody;
      bool go = !armed_only && (cmd.go || e.immediate);
      float armed_seed = in.pitch_rad;
      if ((e.automatic || armed_only) && !go) {
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
                          : learned_trim_valid_ ? learned_trim_rad_
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
    // Tier 2 can keep the body level while the whole robot falls (caught
    // above by theta_v); a body far off means the hips lost it.
    if (std::abs(raw.pitch_rad) > config_.fall_pitch_rad) {
      return fault("fallen (body pitch limit)", out);
    }
    if (tierVirtual()) {
      if (std::abs(legModelError()) <= config_.tier.model_check_rad) {
        model_bad_since_us_ = 0U;
      } else if (model_bad_since_us_ == 0U) {
        model_bad_since_us_ = in.now_us;
      } else if (in.now_us - model_bad_since_us_ >=
                 static_cast<std::uint64_t>(config_.tier.model_check_s * 1.0e6F)) {
        model_bad_since_us_ = 0U;
        return fault("virtual pitch: measured legs disagree with the pose (joint sign/zero?)",
                     out);
      }
    }
    // Operator drive (and its stop ramp) legitimately runs the wheels past
    // runaway_wheel_rad_s (15 rad/s = 0.6 m/s): the guard is for a held or
    // lifted robot, which is not being driven.
    if (std::abs(cmd.forward_m_s) > 1.0e-3F || std::abs(cmd.yaw_rad_s) > 1.0e-3F) {
      drive_seen_us_ = in.now_us;
    }
    const bool driving_recently =
        drive_seen_us_ != 0U && in.now_us - drive_seen_us_ < config_.drive_guard_hold_us;
    const bool fast = !driving_recently &&
        std::abs(in.wheel_rad_s[0]) > config_.runaway_wheel_rad_s &&
        std::abs(in.wheel_rad_s[1]) > config_.runaway_wheel_rad_s;
    if (!fast) {
      runaway_since_us_ = 0U;
    } else if (runaway_since_us_ == 0U) {
      runaway_since_us_ = in.now_us;
      runaway_start_pitch_ = in.pitch_rad;
    } else if (in.now_us - runaway_since_us_ > config_.runaway_us) {
      // Wheels racing without the robot falling means it is held or carried
      // (wheels in the air): re-zero instead of stopping, for both
      // algorithms. Gated on net pitch travel since the wheels started
      // racing (not just this instant): an active, unrecovered fall also
      // races the wheels, and zeroing the command mid-fall only guarantees
      // it completes - that case is left to run its course and hit the
      // pitch-limit fault above cleanly instead. 0.15 rad ~ a quarter of
      // fall_pitch_rad: room for a normal catch transient's wobble, not for
      // a body that kept moving away from where the wheels started racing.
      if (std::abs(in.pitch_rad - runaway_start_pitch_) < 0.15F) {
        heldRezero(in);
        out.wheel_cmd_rad_s = {0.0F, 0.0F};
        out.state = state_;
        return out;
      }
    }
    updateHeight(in, cmd);
    applyPoseOffsets(in);
    if (wholeBody()) {
      out.wheel_cmd_rad_s = wbUpdate(in, cmd);
      if (wb_ring_fault_) {
        wb_ring_fault_ = false;
        return fault("whole-body loop ringing (wheel command cap to cap)", out);
      }
    } else {
      out.wheel_cmd_rad_s = config_.algo == Algo::Pid ? pidUpdate(in, cmd) : lqrUpdate(in, cmd);
      updateTrunk(in, true, out.wheel_cmd_rad_s);
      if (kneeHoldOn()) dknee_ = kneeHold(dhip_);
    }
    out.leg_hip_offset_rad = hipCommand();
    out.leg_knee_offset_rad = kneeCommand();
    out.legs_moving = knee_cmd_rad_ != knee_target_rad_;
    updateRoll(raw, !held_outer_off_ && !held_quiet_ && legs_measured_);
  } else {
    updateRoll(raw, false);
  }
  out.leg_roll_hip_rad = roll_hip_;
  out.leg_roll_knee_rad = roll_knee_;
  out.leg_roll_hip_rate = roll_hip_rate_;
  out.leg_roll_knee_rate = roll_knee_rate_;
  wp_last_cmd_ = out.wheels_enabled ? 0.5F * (out.wheel_cmd_rad_s[0] + out.wheel_cmd_rad_s[1]) : 0.0F;
  updateImuVelocity(raw, state_ == State::Balancing && !held_outer_off_ && !held_quiet_);
  out.state = state_;
  return out;
}

void V3Controller::updateLearnedTrim(const Input& in, float ex, float current_trim_rad) {
  // Only while genuinely free, calm and standing (not driving): that is
  // exactly when the current trim IS the true balance angle, not just a
  // transient guess. tierVirtual() gate: without the virtual pendulum the
  // trim is not height-normalized, so a value learned at one height would
  // be wrong at another.
  const bool ready = tierVirtual() && state_ == State::Balancing && !held_outer_off_ &&
      !held_quiet_ && !pid_driving_ &&
      std::abs(ex) < config_.learn_position_tol_m &&
      std::abs(in.pitch_rate_rad_s) < config_.learn_rate_tol_rad_s;
  if (!ready) {
    learn_since_us_ = 0U;
    return;
  }
  if (learn_since_us_ == 0U) learn_since_us_ = in.now_us;
  if (in.now_us - learn_since_us_ <
      static_cast<std::uint64_t>(std::max(config_.learn_hold_s, 0.0F) * 1.0e6F)) {
    return;
  }
  if (!learned_trim_valid_) {
    learned_trim_rad_ = current_trim_rad;
    learned_trim_valid_ = true;
  } else {
    const float a = std::clamp(in.dt_s / std::max(config_.learn_tau_s, 0.1F), 0.0F, 1.0F);
    learned_trim_rad_ += a * (current_trim_rad - learned_trim_rad_);
  }
}

void V3Controller::engageBalance(const Input& in, float seed) {
  state_ = State::Balancing;
  state_since_us_ = in.now_us;
  runaway_since_us_ = 0U;
  lqr_.reset(ground_x_, true);  // anchors LQR position to ground_x_ too
  lqr_.setBalanceTrim(seed);
  lqr_x_target_m_ = ground_x_;
  lqr_lead_filt_ = 0.0F;
  model_bad_since_us_ = 0U;
  // theta_b is the balance angle at the power-up height; the height
  // feed-forward adds the model change on top (v3.1).
  pid_seed_ = pid_theta_b_ = seed - ffPitch();
  pid_ref_ = seed;
  pid_v_filt_ = pid_cmd_m_s_ = 0.0F;
  pid_x0_ = ground_x_;
  pid_x_ = pid_x_target_ = 0.0F;
  pid_driving_ = false;
  pid_drive_v_ = 0.0F;
  body_accel_f_ = 0.0F;
  held_window_start_us_ = 0U;
  held_resets_ = 0;
  held_quiet_ = false;
  held_outer_off_ = false;
  trim_blend_ = false;
  wbReset(in);
  wb_theta_ref_ = config_.tier.level ? config_.tier.level_pitch_rad : wbGain().theta0 + wb_bias_;
}

float V3Controller::releaseBalance(const Input& in) const {
  // Let go on the floor, the robot tips away from its balance point on the
  // pendulum's divergent mode, theta - theta_eq = rate / omega (capture
  // point). The held state ends only once it has tipped (rate or accel
  // test), ~0.02 rad past where it was put down: seeding the trim from the
  // pitch at that moment balanced around that tipped angle and rolled
  // 0.6 m before the position integral brought it back (physics-sim T8).
  const float omega = std::sqrt(9.80665F / std::max(pose_.com_above_axle_m, 0.05F));
  return in.pitch_rad - std::clamp(in.pitch_rate_rad_s / omega, -0.1F, 0.1F);
}

void V3Controller::heldRezero(const Input& in) {
  ++held_resets_;
  trim_blend_ = false;
  wbReset(in);
  wb_theta_ref_ = in.pitch_rad;
  held_ax_ref_ = ax_f_;
  held_free_ticks_ = 0;
  pid_cmd_m_s_ = 0.0F;
  pid_v_filt_ = 0.0F;
  pid_x0_ = ground_x_;
  pid_x_ = pid_x_target_ = 0.0F;
  pid_seed_ = pid_theta_b_ = in.pitch_rad - ffPitch();
  pid_ref_ = in.pitch_rad;
  pid_driving_ = false;
  pid_drive_v_ = 0.0F;
  // LQR: same job - zero the commanded speed, re-anchor position here, and
  // re-seed the trim so the balance angle tracks the body while held.
  lqr_.reset(ground_x_, true);
  lqr_.setBalanceTrim(in.pitch_rad - ffPitch());
  lqr_x_target_m_ = ground_x_;
  lqr_lead_filt_ = 0.0F;
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
  // v3.1: the balance angle moves with the height (CAD mass model).
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

std::array<float, 2> V3Controller::lqrUpdate(const Input& in, const Command& cmd) {
  const Pid& p = config_.pid;  // shared held/lifted thresholds and deadband comp
  const float radius = std::max(config_.wheel_radius_m, 1.0e-4F);
  const float vmax = config_.lqr.max_ground_velocity_m_s;
  // v3.2: the pendulum length tracks the current leg height every tick; once
  // the gain table is built (Locking) this is a cheap interpolation.
  lqr_.setComHeight(pose_.com_above_axle_m);
  if (held_quiet_) {
    const float wheels = radius * 0.5F * (in.wheel_rad_s[0] + in.wheel_rad_s[1]);
    const bool tilted = std::abs(in.pitch_rad - (lqr_.balanceTrim() + ffPitch())) > 0.03F;
    const bool stopped = std::abs(wheels) < 0.05F &&
        in.now_us - held_quiet_since_us_ >= 200'000U;
    if (!tilted && !stopped) {
      lqr_.reset(ground_x_, true);
      lqr_x_target_m_ = ground_x_;
      held_window_start_us_ = 0U;
      return {0.0F, 0.0F};
    }
    held_quiet_ = false;
  }
  const float target_v = std::clamp(cmd.forward_m_s, -vmax, vmax);
  // Driving moves the position anchor - same job as pid_x_target_, but
  // LqrBalancer takes an absolute anchor (reanchorPosition). The anchor moves
  // at LqrBalancer's own slewed velocity target (last tick's), not the raw
  // command: LqrBalancer ramps the target at 0.3 m/s^2, so a raw-command
  // anchor ran ahead of it (~0.5 m at 0.55 m/s) and the position term fought
  // the ramp at every start and stop. It keeps moving through the stop ramp
  // and then waits for the robot itself to stop (it lags the ramp: wheel lag,
  // and a balancer must lean back before it can brake) before re-anchoring
  // where it actually is - hold where it stopped, as the PID path does.
  // Freezing at the ramp's end left the robot 11-16 cm past the anchor,
  // pulled back slowly afterwards (physics-sim T20).
  const float v_ref = lqr_.debugState().lqr_velocity_reference_m_s;
  if (std::abs(target_v) > 1.0e-3F || std::abs(v_ref) > 1.0e-3F) {
    lqr_x_target_m_ += v_ref * in.dt_s;
    pid_driving_ = true;
  } else if (pid_driving_ && std::abs(pid_v_filt_) < 0.02F) {
    lqr_x_target_m_ = ground_x_;
    pid_driving_ = false;
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
    // Slow integral of the ground-position error into the balance trim -
    // same job as pidUpdate's kix on theta_b, and for the same reason: a
    // station-hold LQR's own velocity-error trim can settle at zero with a
    // nonzero position offset (a small steady pitch lean against a cheap
    // position cost holds it there forever). Position error is the signal
    // that actually needs to reach zero, so integrate from that instead.
    // Frozen outside corr_max_rad of the current trim (mirrors pidUpdate's
    // "!(|u| < corr_max_rad)" freeze): a large seed error means the LQR's
    // own regulator is still catching the initial transient, and winding
    // the trim up during that fight overshot and fell (physics-sim T15).
    const float ex = ground_x_ - lqr_x_target_m_;
    lqr_.setBalanceTrim(lqr_.balanceTrim() - config_.lqr_trim_gain * ex * in.dt_s);
    updateLearnedTrim(in, ex, lqr_.balanceTrim() + ffPitch());
  }
  lqr_.reanchorPosition(lqr_x_target_m_);

  // LqrBalancer::Config::wheel_velocity_reference_from_measured is true, so
  // the raw (noisy) encoder-derived ground_v_ feeds the state vector's
  // velocity term directly; with velocity_cost_scale=3000 (very stiff) that
  // noise gets amplified into ground_accel every tick, and the unfiltered
  // wheel-lag lead above then injects it straight into the sent wheel
  // command - a visible "vồ vập" (jerky) chatter even while pitch/position
  // stay flat (found on real hardware 2026-09-29). pidUpdate already solves
  // this for the PID path with a 5 Hz low-pass (pid_v_filt_); reuse the same
  // filter/state here instead of adding a second one.
  const float v_alpha = std::clamp(2.0F * 3.14159265F * p.speed_lpf_hz * in.dt_s, 0.0F, 1.0F);
  pid_v_filt_ += v_alpha * (ground_v_ - pid_v_filt_);

  robot_control::RobotState state{};
  state.dt_s = in.dt_s;
  state.floor_contact = true;
  state.imu.pitch_rad = in.pitch_rad;
  state.imu.pitch_rate_rad_s = in.pitch_rate_rad_s;
  state.wheel.position_m = ground_x_;
  state.wheel.velocity_m_s = pid_v_filt_;

  robot_control::ControlCommand command{};
  command.target_forward_velocity_m_s = held_outer_off_ ? 0.0F : target_v;
  command.target_yaw_rate_rad_s = cmd.yaw_rad_s;

  // v3.3 maneuver preview (Tier::preview): the balancer tracks the plan
  // preview_s late, leaning by atan(a/g) with the plan's acceleration fed
  // forward; the wheels get the plan preview_s early (below).
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

  // integrate=false while held: LqrBalancer decays the commanded speed to
  // zero on its own (~0.1 s) and freezes the balance-trim adaptation -
  // matching the PID path's leak-to-zero and frozen theta_b while held.
  const bool integrate = !held_outer_off_;
  const auto lqr_out = lqr_.update(state, command, integrate, ffPitch() + lean_ff, true);
  // v3.1: shares updateHeight's settled check with the PID path.
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

  // Wheel-lag lead (see pidUpdate's own p.lead_s comment): the BL4820 wheels
  // are a first-order speed servo lagging ~0.1-0.15 s behind the command, and
  // the raw LQR model assumes instant ground-acceleration authority - without
  // this the loop rings and falls at tau_s >= 0.12 (physics-sim T11 finding,
  // same failure PID had before lead_s was added). Same trick: lead the sent
  // speed by the LQR's own solved acceleration times lead_s - but, unlike the
  // first cut of this, clamp the LED-AUGMENTED ground speed back to vmax
  // before converting to wheel rad/s. LqrBalancer::update() already clamps
  // its own commanded_ground_velocity_m_s_ to vmax, but the lead term is
  // added on top of that afterwards and was NOT bounded again: raising
  // max_ground_accel_m_s2 (1.4 -> 8.0, the push-recovery fix) let lead_v hit
  // 8.0*0.12 = 0.96 m/s on its own, so total commands reached ~44 rad/s (vs
  // the ~12 rad/s the wheels can physically track) - runaway wheel commands
  // while pitch stayed flat, caught only by the held-window reset every
  // ~150 ms (found on real hardware 2026-09-29, robot never actually fell,
  // but re-zeroed every 1-2 s: exactly the "chạy trốn" pattern from the old
  // v1/v2 LQR). PID avoids this because it clamps pid_cmd_m_s_+accel*lead_s
  // to vmax itself (pidUpdate's `sent`); LQR now does the same.
  //
  // The raw per-tick ground_accel (LqrBalancer's -K*x solve) is itself noisy
  // - it reacts to gyro pitch_rate and (even filtered) wheel-speed jitter
  // through a state feedback with a very stiff velocity_cost_scale - and
  // rides straight into the sent command unfiltered, which read as jerky
  // ("vồ vập") on real hardware even after filtering the velocity state term
  // alone (2026-09-29). A 20 Hz low-pass here rejects that per-tick chatter
  // while staying well above genuine body dynamics (few Hz) and the lag this
  // lead exists to compensate (tau_s ~0.08-0.15 s).
  const float lead_alpha = std::clamp(2.0F * 3.14159265F * 20.0F * in.dt_s, 0.0F, 1.0F);
  // The plan's own acceleration is advanced exactly by wheel_advance; the
  // lead only covers the feedback part.
  lqr_lead_filt_ += lead_alpha *
      (lqr_.debugState().optimal_command_m_s - accel_ff - lqr_lead_filt_);
  const float base_ground_v = lqr_out.ground_velocity_m_s;
  const float lead_v = lqr_lead_filt_ * config_.lqr_lead_s + wheel_advance;
  const float sent_ground_v = std::clamp(base_ground_v + lead_v, -vmax, vmax);
  const float lead_wheel = config_.lqr.left_wheel_sign *
      (sent_ground_v - base_ground_v) / radius;
  // LQR has its own (smaller) deadband push: the tanh step adds loop gain
  // right at every zero crossing, which is where the delayed-wheel wobble
  // lives (see input_cost_scale in defaultV3Config).
  const float dbc = config_.lqr_deadband_comp_rad_s;
  auto comp = [&](float w) {
    if (held_outer_off_) {
      const float cap = std::max(config_.held_wheel_cap_m_s, 0.0F) / radius;
      return std::clamp(w, -cap, cap);
    }
    if (dbc <= 0.0F) return w;
    return w + dbc * std::tanh(w / std::max(p.deadband_eps_rad_s, 1.0e-3F));
  };
  return {comp(lqr_out.left_wheel_velocity_rad_s + lead_wheel),
          comp(lqr_out.right_wheel_velocity_rad_s + lead_wheel)};
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
  const float knee_cmd = std::clamp(k0 + knee_dev, LegHeight::kKneeMinRad, 0.0F);
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

V3Controller::ComLine V3Controller::comLine(float hip_offset, float knee_offset) const noexcept {
  namespace geo = robot_control::geometry;
  const float hip = LegHeight::hangHipCad() + hip_offset;
  const float knee = knee_offset;
  const auto body = robot_control::WholeBodyModel{}.compute(hip, knee, hip, knee);
  const auto jac = robot_control::WholeBodyModel::symmetricComOffsetJacobian(hip, knee);
  const float cx = body.com_offset_x_m +
      geo::parts[0].mass * config_.tier.body_com_dx_m / geo::total_mass_kg;
  const float cz = std::max(body.com_above_axle_m, 0.01F);
  const float den = cx * cx + cz * cz;
  return {std::atan2(cx, cz),
          (cz * jac.d_com_offset_x_d_hip - cx * jac.d_com_above_axle_d_hip) / den,
          (cz * jac.d_com_offset_x_d_knee - cx * jac.d_com_above_axle_d_knee) / den};
}

void V3Controller::updateVirtualPitch(const Input& in) {
  theta_v_ = in.pitch_rad;
  theta_v_rate_ = in.pitch_rate_rad_s;
  if (!config_.tier.virtual_pitch && !config_.tier.trunk) return;
  // Hang reference and the tier-2 gain at the height pose: recomputed only
  // when the model offset or the pose changes (the loop has ~0.4 ms spare).
  if (!(com_line_dx_ == config_.tier.body_com_dx_m) || !(j_hip_knee_ == pose_.knee_offset_rad)) {
    com_line_dx_ = config_.tier.body_com_dx_m;
    com_line_hang_ = comLine(0.0F, 0.0F).angle;
    j_hip_knee_ = pose_.knee_offset_rad;
    const ComLine pose = comLine(pose_.hip_offset_rad, pose_.knee_offset_rad);
    j_hip_ = pose.d_hip;
    c_pose_ = pose.angle;
  }
  if (!tierVirtual()) return;
  // Symmetric model on the mean of both legs (hand-arranged zeros differ a
  // little; the planar model has no roll anyway).
  hip_rate_meas_ = 0.5F * (in.hip_rate_rad_s[0] + in.hip_rate_rad_s[1]);
  knee_rate_meas_ = 0.5F * (in.knee_rate_rad_s[0] + in.knee_rate_rad_s[1]);
  hip_meas_ = 0.5F * (in.hip_rad[0] + in.hip_rad[1]);
  knee_meas_ = 0.5F * (in.knee_rad[0] + in.knee_rad[1]);
  const ComLine c = comLine(hip_meas_, knee_meas_);
  c_meas_ = c.angle;
  theta_v_ = in.pitch_rad + c.angle - com_line_hang_;
  // Analytic rate from the driver velocities: differentiating the angle and
  // filtering it would add lag to the balancer's D channel.
  theta_v_rate_ = in.pitch_rate_rad_s + config_.tier.joint_rate_gain *
      (c.d_hip * hip_rate_meas_ + c.d_knee * knee_rate_meas_);
}

void V3Controller::updateImuVelocity(const Input& raw, bool free) {
  imu_velocity_ = ImuVelocity{};
  // Whole-body too: without it the pitch estimate read every acceleration
  // as tilt and kept ~0.026 rad of it seconds into the cruise (gyro-
  // dominant blend) - the WB then held the body that much off level and ran
  // ~15 % over the commanded speed (sim). The joints come from the raw
  // input (the tiers' hip_meas_ is only kept with the virtual pitch).
  const bool wb = wholeBody() && legs_measured_;
  if (!config_.tier.imu_kinematic || !free || !(tierVirtual() || wb)) return;
  const float hip_off = wb ? 0.5F * (raw.hip_rad[0] + raw.hip_rad[1]) : hip_meas_;
  const float knee = wb ? 0.5F * (raw.knee_rad[0] + raw.knee_rad[1]) : knee_meas_;
  const float hip_rate = wb ? 0.5F * (raw.hip_rate_rad_s[0] + raw.hip_rate_rad_s[1]) : hip_rate_meas_;
  const float knee_rate = wb ? 0.5F * (raw.knee_rate_rad_s[0] + raw.knee_rate_rad_s[1]) : knee_rate_meas_;
  const float hip = LegHeight::hangHipCad() + hip_off;
  const float wheel = 0.5F * (raw.wheel_rad_s[0] + raw.wheel_rad_s[1]);
  const auto v = robot_control::WholeBodyModel::imuVelocityThroughLeg(
      raw.pitch_rad, raw.pitch_rate_rad_s, hip, knee, hip_rate, knee_rate,
      wheel, config_.wheel_radius_m);
  imu_velocity_ = {v.horizontal_m_s, v.vertical_m_s, std::isfinite(v.horizontal_m_s) &&
                                                         std::isfinite(v.vertical_m_s)};
}

void V3Controller::updateTrunk(const Input& in, bool active,
                               std::array<float, 2> wheel_cmd_rad_s) {
  const Tier& t = config_.tier;
  // Settle after engage and again after every held/lifted spell: the
  // release catch is tier 1's alone.
  if (!active || held_outer_off_ || held_quiet_) trunk_block_us_ = in.now_us;
  const std::uint64_t since = std::max(state_since_us_, trunk_block_us_);
  const bool settled = in.now_us - since >=
      static_cast<std::uint64_t>(std::max(t.settle_s, 0.0F) * 1.0e6F);
  // Lean of the CoM line from the balance point (in.pitch is theta_v),
  // split into the planned part (maneuver preview: known ahead, smooth) and
  // the residual (disturbances). The legs take the planned lean from the
  // plan itself, hip_advance_s early against the hip servo lag; only the
  // residual goes through the high-pass and the (filtered) lead, so sensor
  // noise and the lead's gain stop shaking the body during planned motion.
  const float lean = in.pitch_rad - (balanceRef() + ffPitch());
  float lean_plan_now = 0.0F, lean_plan_ahead = 0.0F;
  if (t.preview && tierVirtual() && plan_n_ > 0U) {
    const float g = 9.80665F;
    lean_plan_now = std::atan(planAt(t.preview_s).a / g);
    lean_plan_ahead = std::atan(planAt(std::max(t.preview_s - t.hip_advance_s, 0.0F)).a / g);
  }
  const float residual = lean - lean_plan_now;
  if (trunk_alpha_ <= 0.0F) {
    lean_lp_ = residual;  // start from the current lean: no step when it ramps in
    lean_rate_f_ = 0.0F;
  } else if (t.hpf_s > 0.0F) {
    lean_lp_ += std::min(in.dt_s / t.hpf_s, 1.0F) * (residual - lean_lp_);
  } else {
    lean_lp_ = 0.0F;
  }
  const float rate_alpha = t.lead_lpf_hz > 0.0F
      ? std::clamp(2.0F * 3.14159265F * t.lead_lpf_hz * in.dt_s, 0.0F, 1.0F) : 1.0F;
  lean_rate_f_ += rate_alpha * (in.pitch_rate_rad_s - lean_rate_f_);
  // Full-authority desired hip offset (alpha = 1, unclamped): the signal
  // that decides whether the legs are running out of range, independent of
  // how far alpha has already faded (see Tier::sat_margin).
  const float ratio = std::clamp(pose_.com_above_axle_m / std::max(com_hang_m_, 0.01F),
                                 0.1F, 1.0F);
  const float span = std::max(t.ratio_tall - t.ratio_low, 1.0e-3F);
  const float low_w = std::clamp((t.ratio_tall - ratio) / span, 0.0F, 1.0F);
  const float exponent = t.height_exp_tall + low_w * (t.height_exp - t.height_exp_tall);
  const float share = t.lean_share * std::pow(ratio, std::max(exponent, 0.0F));
  const float jh = std::max(j_hip_, 0.2F);
  const float lead = t.lead_s * std::pow(ratio, std::max(t.lead_exp, 0.0F));
  const float plan_share = t.plan_full ? t.lean_share : share;
  const float full_want = (plan_share * lean_plan_ahead +
                           share * (residual - lean_lp_ + lead * lean_rate_f_)) / jh;
  const float max_rad = std::max(t.max_rad, 1.0e-3F);
  const float margin = std::clamp(t.sat_margin, 0.0F, 0.99F);
  const float mag_ratio = std::abs(full_want) / max_rad;
  // Gate the fade on the wheel command too (see Tier::sat_margin): only
  // once tier 1 itself is near its own cap does giving up hip range early
  // pay for itself.
  const float wheel_cap = std::max(config_.lqr.max_ground_velocity_m_s, 0.05F) /
      std::max(config_.wheel_radius_m, 1.0e-4F);
  const float wheel_ratio = std::max(std::abs(wheel_cmd_rad_s[0]), std::abs(wheel_cmd_rad_s[1])) /
      wheel_cap;
  const bool wheel_saturating = wheel_ratio >= std::clamp(t.wheel_sat_margin, 0.0F, 1.0F);
  const float range_ceiling = (!wheel_saturating || mag_ratio <= margin)
      ? 1.0F : std::clamp((1.0F - mag_ratio) / (1.0F - margin), 0.0F, 1.0F);
  const bool on = active && settled && t.trunk && tierVirtual();
  const float target = on ? range_ceiling : 0.0F;
  const float up = t.ramp_s > 1.0e-3F ? in.dt_s / t.ramp_s : 1.0F;
  const float down = t.sat_fade_s > 1.0e-3F ? in.dt_s / t.sat_fade_s : 1.0F;
  const float diff = target - trunk_alpha_;
  trunk_alpha_ = std::clamp(trunk_alpha_ + std::clamp(diff, -down, up), 0.0F, 1.0F);
  const float dyn = std::clamp(trunk_alpha_ * full_want, -max_rad, max_rad);

  // Static level loop (Tier::level). body = theta_v - com_line terms, and
  // tier 1 holds theta_v at the balance point, so a hip offset of
  // (body - target) / J re-orients the body by exactly that much: the
  // integrator's closed loop is first order with level_tau_s at any height
  // (J cancels). Started once from the measured error (the first
  // release of a power-up levels over ramp_s instead of 3 * tau), then only
  // integrates with the dynamic part at full authority and the wheels
  // clear of their cap: a faded or saturated tier 2 hands lean back to the
  // body, which is not a posture error.
  const float lp = std::clamp(2.0F * 3.14159265F * t.level_lpf_hz * in.dt_s, 0.0F, 1.0F);
  body_lp_ += lp * (body_pitch_ - body_lp_);
  const bool level_on = on && t.level;
  const float lmax = std::max(t.level_max_rad, 0.0F);
  if (level_on && !level_init_) {
    level_i_ = std::clamp((body_lp_ - t.level_pitch_rad) / jh, -lmax, lmax);
    level_init_ = true;
  } else if (level_on && level_beta_ >= 1.0F && trunk_alpha_ >= 1.0F && !wheel_saturating) {
    level_i_ = std::clamp(level_i_ + in.dt_s / std::max(t.level_tau_s, 0.05F) *
                                         (body_lp_ - t.level_pitch_rad) / jh,
                          -lmax, lmax);
  }
  const float level_diff = (level_on ? 1.0F : 0.0F) - level_beta_;
  level_beta_ = std::clamp(level_beta_ + std::clamp(level_diff, -down, up), 0.0F, 1.0F);

  const float want = dyn + level_beta_ * level_i_;
  const float step = std::max(t.rate_rad_s, 0.0F) * in.dt_s;
  dhip_ += std::clamp(want - dhip_, -step, step);
}

void V3Controller::predictWheels(const Input& raw, float* rate, float* pos) noexcept {
  const WheelPredict& w = config_.wheel_pred;
  const float dt = std::max(raw.dt_s, 1.0e-4F);
  auto ticks = [&](float s) {
    return std::min<std::size_t>(static_cast<std::size_t>(std::lround(std::max(s, 0.0F) / dt)), kWpBuf - 1U);
  };
  // Model of the wheel servo driven by the commands actually sent.
  wp_cmd_[wp_head_] = wp_last_cmd_;
  const std::size_t da = ticks(w.act_delay_s);
  const float u = wp_cmd_[(wp_head_ + kWpBuf - da) % kWpBuf];
  wp_w_model_ += std::min(dt / std::max(w.tau_s, 1.0e-3F), 1.0F) * (u - wp_w_model_);
  wp_p_model_ += wp_w_model_ * dt;
  wp_w_[wp_head_] = wp_w_model_;
  wp_p_[wp_head_] = wp_p_model_;
  const std::size_t dm = ticks(w.meas_delay_s);
  const std::size_t old = (wp_head_ + kWpBuf - dm) % kWpBuf;
  wp_head_ = (wp_head_ + 1U) % kWpBuf;
  if (!(w.enabled || (wholeBody() && w.with_wb))) return;
  *rate += wp_w_model_ - wp_w_[old];
  *pos += wp_p_model_ - wp_p_[old];
}

void V3Controller::updateRoll(const Input& in, bool active) {
  const Roll& r = config_.roll;
  // Body roll against GRAVITY: gyro, pulled toward the accelerometer with
  // the turn's centripetal pull (measured ground speed x measured yaw rate)
  // taken out. The first version estimated against the apparent vertical
  // directly - but the gyro cannot see a centripetal change, so at a turn's
  // entry the estimate read the lean-in as a tilt the wrong way for ~0.5 s
  // and the loop fought its own feed-forward (sim: still 0.005 rad off 4.9 s
  // into a 3 s turn). The lean the turn needs is the target instead.
  const float a_lp = std::clamp(2.0F * 3.14159265F * 5.0F * in.dt_s, 0.0F, 1.0F);
  roll_v_lp_ += a_lp * (ground_v_ - roll_v_lp_);
  roll_w_lp_ += a_lp * (in.yaw_rate_rad_s - roll_w_lp_);
  const float a_c = roll_v_lp_ * roll_w_lp_;  // centripetal [m/s^2], + = toward the left
  const float acc_roll = std::atan2(in.accel_y_m_s2 - a_c, std::max(in.accel_z_m_s2, 0.1F));
  const float amag = std::sqrt(in.accel_x_m_s2 * in.accel_x_m_s2 + in.accel_y_m_s2 * in.accel_y_m_s2 +
                               in.accel_z_m_s2 * in.accel_z_m_s2);
  if (!roll_init_) {
    roll_est_ = acc_roll;
    roll_init_ = true;
  } else {
    // The roll gyro arrives raw (the estimator only de-biases pitch): its
    // bias is learned from the same error (PI complementary filter, ~5 s).
    roll_est_ += (in.roll_rate_rad_s - roll_bias_) * in.dt_s;
    if (std::abs(amag - 9.80665F) < 2.5F) {
      const float tau = std::max(r.est_tau_s, 0.02F);
      const float err = acc_roll - roll_est_;
      roll_est_ += std::min(in.dt_s / tau, 1.0F) * err;
      roll_bias_ = std::clamp(roll_bias_ - in.dt_s / (tau * 5.0F) * err, -0.2F, 0.2F);
    }
  }
  // Lean-in target: the resultant of gravity and the centripetal pull
  // between the wheels (left turn, a_c > 0: left side down, roll < 0).
  roll_target_ = r.target_rad - r.turn_ff * std::atan(std::clamp(a_c / 9.80665F, -0.3F, 0.3F));
  const float half = 0.5F * std::max(r.track_m, 0.05F);
  const float step = std::max(r.rate_m_s, 0.0F) * in.dt_s;
  if (!r.enabled || !active) {
    roll_dh_ -= std::clamp(roll_dh_, -step, step);  // back to even legs
    roll_i_ = roll_dh_;
    roll_e_lp_ = roll_est_ - roll_target_;
  } else {
    // Feed-forward: the side offset that sets the target roll on level
    // ground; the integral takes the ground and the legs' own error; the
    // proportional part reacts to a sudden tilt at once (gyro-fast).
    const float ff = half * std::tan(roll_target_ - r.target_rad);
    roll_e_lp_ += std::clamp(2.0F * 3.14159265F * std::max(r.err_lpf_hz, 0.1F) * in.dt_s, 0.0F, 1.0F) *
        ((roll_est_ - roll_target_) - roll_e_lp_);
    const float e = roll_e_lp_;  // + = left side too high
    const float room = std::max(std::min({r.max_dh_m, LegHeight::maxHeight() - pose_.axle_below_hip_m,
                                          pose_.axle_below_hip_m - LegHeight::minHeight()}),
                                0.0F);
    const float tau = r.vel_ff ? r.tau_vff : r.tau_s;
    const float kp = r.vel_ff ? r.kp_vff : r.kp;
    roll_i_ -= in.dt_s / std::max(tau, 0.02F) * half * std::sin(e);
    roll_i_ = std::clamp(roll_i_, -room - ff, room - ff);
    const float p = -kp * half * std::sin(e);
    const float want = std::clamp(roll_i_ + ff + p, -room, room);
    roll_dh_ += std::clamp(want - roll_dh_, -step, step);
  }
  // Side height offset -> joint offsets through the leg IK at the pose
  // (knee for the height, hip to keep the axle under the hip).
  if (roll_deriv_key_ != knee_cmd_rad_) {
    roll_deriv_key_ = knee_cmd_rad_;
    const float k0 = pose_.knee_offset_rad;
    const float k1 = std::clamp(k0 - 0.01F, LegHeight::kKneeMinRad, 0.0F);
    const LegPose p1 = LegHeight::forKnee(k1);
    const float dh = pose_.axle_below_hip_m - p1.axle_below_hip_m;
    roll_dk_dh_ = std::abs(dh) > 1.0e-6F ? (k0 - k1) / dh : 0.0F;
    roll_dq_dk_ = std::abs(k0 - k1) > 1.0e-6F ? (pose_.hip_offset_rad - p1.hip_offset_rad) / (k0 - k1) : 0.0F;
  }
  const float dk = roll_dh_ * roll_dk_dh_ * (r.sign < 0.0F ? -1.0F : 1.0F);
  const std::array<float, 2> knee{dk, -dk}, hip{dk * roll_dq_dk_, -dk * roll_dq_dk_};
  for (std::size_t i = 0; i < 2U; ++i) {
    roll_knee_rate_[i] = r.vel_ff && in.dt_s > 0.0F ? (knee[i] - roll_knee_[i]) / in.dt_s : 0.0F;
    roll_hip_rate_[i] = r.vel_ff && in.dt_s > 0.0F ? (hip[i] - roll_hip_[i]) / in.dt_s : 0.0F;
  }
  roll_knee_ = knee;
  roll_hip_ = hip;
}

float V3Controller::kneeHold(float dhip) noexcept {
  if (!kneeHoldOn() || dhip == 0.0F) {
    knee_hold_ = 0.0F;
    return 0.0F;
  }
  const float hip = LegHeight::hangHipCad() + pose_.hip_offset_rad + dhip;
  const float target = -pose_.axle_below_hip_m;  // legHeight's sign: z up
  float k = std::clamp(pose_.knee_offset_rad + knee_hold_, LegHeight::kKneeMinRad, 0.0F);
  for (int i = 0; i < 3; ++i) {
    const auto leg = robot_control::WholeBodyModel::legHeight(hip, k);
    if (std::abs(leg.d_height_d_knee) < 1.0e-4F) break;
    const float step = (leg.height_m - target) / leg.d_height_d_knee;
    k = std::clamp(k - step, LegHeight::kKneeMinRad, 0.0F);
    if (std::abs(step) < 1.0e-5F) break;
  }
  knee_hold_ = k - pose_.knee_offset_rad;
  return knee_hold_;
}

float V3Controller::planCap(float vmax) const noexcept {
  const Tier& t = config_.tier;
  if (!envelopeOn()) return std::clamp(t.plan_vmax_m_s, 0.05F, vmax);
  const float l = std::max(pose_.com_above_axle_m, 0.01F);
  return std::clamp(vmax - t.env_catch_rad * std::sqrt(9.80665F * l), 0.05F, vmax);
}

void V3Controller::updatePlan(float target_v, float dt, bool reset) {
  const Tier& t = config_.tier;
  if (reset || !plan_init_) {
    plan_v_ = reset ? 0.0F : target_v;
    plan_a_ = plan_j_ = 0.0F;
    plan_n_ = 0U;
    plan_init_ = true;
  }
  const float amax = envelopeOn() && wholeBody()
      ? std::max(9.80665F * std::tan(std::clamp(t.env_lean_rad, 0.0F, 0.3F)), 0.05F)
      : std::max(t.plan_accel_m_s2, 0.05F);
  float jmax = std::max(t.plan_jerk_m_s3, 0.1F);
  if (envelopeOn()) {
    jmax *= std::sqrt(std::max(com_hang_m_, 0.01F) / std::max(pose_.com_above_axle_m, 0.01F));
  }
  // Jerk-limited (S-curve) tracking of the operator speed: aim the
  // acceleration at the sign of the error left after ramping the current
  // acceleration back to zero at jmax, so it lands on the target without
  // overshoot.
  const float v_stop = plan_v_ + plan_a_ * std::abs(plan_a_) / (2.0F * jmax);
  const float err = target_v - v_stop;
  const float a_des = std::abs(err) < 1.0e-4F ? 0.0F : std::copysign(amax, err);
  const float da = std::clamp(a_des - plan_a_, -jmax * dt, jmax * dt);
  plan_a_ += da;
  plan_j_ = dt > 0.0F ? da / dt : 0.0F;
  plan_v_ += plan_a_ * dt;
  if (std::abs(target_v - plan_v_) < 2.0e-3F && std::abs(plan_a_) <= 2.0F * jmax * dt) {
    plan_v_ = target_v;
    plan_a_ = plan_j_ = 0.0F;
  }
  plan_buf_[plan_head_] = {plan_v_, plan_a_};
  plan_head_ = (plan_head_ + 1U) % plan_buf_.size();
  if (plan_n_ < plan_buf_.size()) ++plan_n_;
  plan_dt_ = dt;
}

V3Controller::PlanSample V3Controller::planDelayed() const noexcept {
  return planAt(config_.tier.preview_s);
}

V3Controller::PlanSample V3Controller::planAt(float delay_s) const noexcept {
  const float dt = std::max(plan_dt_, 1.0e-4F);
  std::size_t lag = static_cast<std::size_t>(std::lround(std::max(delay_s, 0.0F) / dt));
  lag = std::min({lag, plan_buf_.size() - 1U, plan_n_ > 0U ? plan_n_ - 1U : 0U});
  const std::size_t i = (plan_head_ + plan_buf_.size() - 1U - lag) % plan_buf_.size();
  return plan_n_ > 0U ? plan_buf_[i] : PlanSample{plan_v_, plan_a_};
}

void V3Controller::updateHeight(const Input& in, const Command& cmd) {
  const Height& h = config_.height;
  // Roll needs room both ways: the straight knee at the full height can only
  // fold, so with roll on the top is kept reserve_m lower.
  const float hmax = LegHeight::maxHeight() -
      (config_.roll.enabled ? std::clamp(config_.roll.reserve_m, 0.0F, 0.05F) : 0.0F);
  if (cmd.height_m > 0.0F || height_target_m_ > hmax + 1.0e-4F) {
    const float want = cmd.height_m > 0.0F ? cmd.height_m : height_target_m_;
    const float target = std::clamp(want, LegHeight::minHeight(), hmax);
    if (std::abs(target - height_target_m_) > 1.0e-4F) {
      height_target_m_ = target;
      knee_target_rad_ = LegHeight::forHeight(target).knee_offset_rad;
    }
  }
  if (knee_cmd_rad_ == knee_target_rad_) {
    height_speed_m_s_ = 0.0F;
    return;
  }
  // Speed quality: full while held (stand, hands: the wheels are not
  // carrying it, so leg motion cannot upset it), otherwise falling smoothly
  // to zero as the balance gets busy - never an on/off gate.
  float quality = 1.0F;
  if (held_quiet_) {
    quality = 0.0F;
  } else if (!held_outer_off_) {
    quality = std::clamp(1.0F - std::abs(in.pitch_rad - pid_ref_) / h.max_pitch_error_rad,
                         0.0F, 1.0F) *
        std::clamp(1.0F - std::abs(in.pitch_rate_rad_s) / h.max_pitch_rate_rad_s, 0.0F, 1.0F);
  }
  // Signed height speed (+ = up): capped by the quality and by the speed
  // that still stops at the target under accel_m_s2, and slew-limited.
  const float remaining = height_target_m_ - pose_.axle_below_hip_m;
  const float accel = std::max(h.accel_m_s2, 1.0e-3F);
  const float v_stop = std::sqrt(2.0F * accel * std::abs(remaining));
  const float v_des = std::copysign(std::min(h.rate_m_s * quality, v_stop), remaining);
  const float dv = accel * in.dt_s;
  height_speed_m_s_ += std::clamp(v_des - height_speed_m_s_, -dv, dv);
  // Knee step via the local d height/d knee (folding the knee lowers it).
  const LegPose ahead = LegHeight::forKnee(knee_cmd_rad_ - 0.01F);
  const float dh_dk = std::max((pose_.axle_below_hip_m - ahead.axle_below_hip_m) / 0.01F, 0.02F);
  const float step = height_speed_m_s_ * in.dt_s / dh_dk;
  const float next = knee_cmd_rad_ + step;
  const bool passed = knee_target_rad_ < knee_cmd_rad_ ? next <= knee_target_rad_
                                                       : next >= knee_target_rad_;
  if (passed || (std::abs(remaining) < 2.0e-4F && std::abs(height_speed_m_s_) < 2.0e-3F)) {
    knee_cmd_rad_ = knee_target_rad_;
    height_speed_m_s_ = 0.0F;
  } else {
    knee_cmd_rad_ = std::clamp(next, LegHeight::kKneeMinRad, 0.0F);
  }
  pose_ = LegHeight::forKnee(knee_cmd_rad_);
  legs_moved_us_ = in.now_us;
}

void V3Controller::updateSuspension(const Input& in) {
  const Tier& t = config_.tier;
  // Specific force along gravity at the IMU (hip origin): its slow mean is
  // gravity (and the accelerometer's scale/bias), the rest is the body's
  // vertical acceleration.
  const float fzw = -in.accel_x_m_s2 * std::sin(body_pitch_) + in.accel_z_m_s2 * std::cos(body_pitch_);
  if (!sus_init_) {
    sus_g_ = fzw;
    sus_init_ = true;
  }
  sus_g_ += std::clamp(in.dt_s / 2.0F, 0.0F, 1.0F) * (fzw - sus_g_);
  // Off during KNEETEST FLOOR: it would fight the excitation it measures.
  const bool on = t.suspension && tierVirtual() && !held_outer_off_ && !held_quiet_ &&
      knee_excite_rad_ == 0.0F;
  // Height <-> knee at the pose: the pose keeps the axle under the hip, so
  // the leg Jacobian's knee column is the path's own slope.
  const auto leg = robot_control::WholeBodyModel::legHeight(
      LegHeight::hangHipCad() + pose_.hip_offset_rad, knee_cmd_rad_);
  const float dh_dk = std::max(-leg.d_height_d_knee, 0.02F);  // axle_below_hip per knee rad
  if (on) {
    // Complementary: integrate the IMU, pulled towards the legs' height rate
    // (the body's vertical speed on a flat floor) with sus_leak_s.
    const float vz_legs = dh_dk * knee_rate_meas_;
    sus_vz_ += in.dt_s * (fzw - sus_g_) +
        in.dt_s / std::max(t.sus_leak_s, 0.05F) * (vz_legs - sus_vz_);
  } else {
    sus_vz_ = 0.0F;
  }
  // Relative to the planned height change: a commanded raise/lower is the
  // body moving on purpose (the sky-hook fought it: 90 mm ended 6 mm high).
  const float vz_err = sus_vz_ - height_speed_m_s_;
  const float db = std::max(t.sus_deadband_m_s, 0.0F);
  const float vz_eff = std::copysign(std::max(std::abs(vz_err) - db, 0.0F), vz_err);
  const float rate = on ? -t.sus_gain * vz_eff : 0.0F;
  const float lim = std::max(t.sus_max_m, 0.0F);
  sus_dz_ = std::clamp(sus_dz_ + in.dt_s * (rate - sus_dz_ / std::max(t.sus_return_s, 0.05F)),
                       -lim, lim);
  sus_dk_ = sus_dz_ / dh_dk;
}

void V3Controller::applyPoseOffsets(const Input& in) {
  updateSuspension(in);
  const float dk = knee_excite_rad_ + sus_dk_;
  if (dk == 0.0F) {
    if (excite_applied_) {
      pose_ = LegHeight::forKnee(knee_cmd_rad_);
      excite_applied_ = false;
    }
    return;
  }
  // forKnee costs a few Newton steps: only when the offset or the height
  // pose under it moved.
  if (excite_applied_ && std::abs(dk - applied_dk_) < 2.0e-4F && applied_base_ == knee_cmd_rad_) {
    return;
  }
  pose_ = LegHeight::forKnee(std::clamp(knee_cmd_rad_ + dk, LegHeight::kKneeMinRad, 0.0F));
  applied_dk_ = dk;
  applied_base_ = knee_cmd_rad_;
  excite_applied_ = true;
  // The joint test is a deliberate leg motion (held test off); the
  // suspension's few mm are not.
  if (knee_excite_rad_ != 0.0F) legs_moved_us_ = in.now_us;
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
  cfg.algo = V3Controller::Algo::Lqr;  // v3.2: LQR is the balancer (SET algo 1 = PID)
  cfg.lqr = robot_control::tunedControllerConfig().lqr;
  cfg.lqr.wheel_pitch_feedback_scale = 1.0F;
  // v3.2: LqrBalancer's own balance_trim integrates ground-SPEED error, which
  // can settle at zero at any station-hold position offset (a small residual
  // pitch error, cheap against the low position_cost_scale, holds a steady
  // nonzero position forever - measured: LQR T11, x_end crept to and stuck
  // near +0.10 m with eq +0.10 rad even after 200 s). V3Controller runs its
  // own POSITION-error trim integral instead (lqrUpdate, same job as the PID
  // path's kix on theta_b), so the built-in one stays off here.
  cfg.lqr.balance_trim_gain_rad_per_m = 0.0F;
  cfg.lqr.balance_trim_max_rad = 0.20F;
  cfg.lqr.balance_trim_pitch_gate_rad = 0.25F;
  cfg.lqr.command_velocity_tracking_margin_m_s = 0.25F;
  cfg.lqr.max_ground_velocity_m_s = 0.8F;
  // Demo turning: 1.5 rad/s (~86 deg/s) = 0.23 m/s per wheel at the 0.1545 m
  // half track; the default 0.8 capped the BLE stick well below that.
  cfg.lqr.max_yaw_rate_rad_s = 1.5F;
  // tunedControllerConfig() derives this from a 0.13 N.m torque spec that
  // predates v3.1's real BL4820 wheels (leftover from the old EmbeddedController
  // config) - PID's accel_max_m_s2 = 8.0 is the value actually validated on
  // this hardware (robustness sweep, see Pid::accel_max_m_s2's own comment),
  // so LQR gets the same real authority instead of a ~6x weaker stale cap.
  // Found 2026-09-29: a push the PID path recovered from made LQR fall,
  // because 1.4 m/s^2 could not command enough wheel accel in time.
  cfg.lqr.max_ground_accel_m_s2 = cfg.pid.accel_max_m_s2;
  // v3.2: hold ground position from the start, like the PID path
  // (LqrBalancer::update: position error is force-zeroed unless BOTH
  // hold_station and position_anchor_hold are set - v3.2 always passes
  // hold_station=true and manages the anchor itself via reanchorPosition).
  cfg.lqr.position_anchor_hold = true;
  // Real v3.1 CoM-above-axle range, deepest crouch to hang pose (measured
  // 2026-09-29 via LegHeight/WholeBodyModel: 0.0664-0.1593 m), with margin so
  // the gain table's endpoints are never clamped in normal use.
  cfg.lqr.min_pendulum_length_m = 0.05F;
  cfg.lqr.max_pendulum_length_m = 0.20F;
  // Station-gain blend must stay numerically inert (v3.2 does not use the
  // posture-capture feature): mirror every station_* cost scale onto its
  // base scale so gain_ and station_gain_ solve to the same values.
  // Gains for the real wheel delay. The BL4820 speed servo answers after
  // ~50 ms dead time + ~75 ms lag (fitted on the 2026-09-29/30 logs), not the
  // ~4 ms the sim had assumed, and the tuned gains (input cost 10, lead 0.12)
  // rang at ~4 Hz while accelerating ("pecking"). Swept in the sim with that
  // delay (40-64 ms, lag 60-120 ms, standing and driving 0.45 m/s), seed
  // errors +-0.035/+-0.08 rad, a 1 N push and a slope: input cost 20 with
  // lead 0.08 s and deadband push 0.3 cut the worst pitch-rate rms 1.1 ->
  // 0.25 rad/s with no falls and still meets every settling bound. Softer
  // (25+) was calmer but gave up push/slope recovery; 100+ falls.
  cfg.lqr.input_cost_scale = 20.0F;
  cfg.lqr.station_position_cost_scale = cfg.lqr.position_cost_scale;
  cfg.lqr.station_pitch_cost_scale = cfg.lqr.pitch_cost_scale;
  cfg.lqr.station_velocity_cost_scale = cfg.lqr.velocity_cost_scale;
  cfg.lqr.station_pitch_rate_cost_scale = cfg.lqr.pitch_rate_cost_scale;
  cfg.lqr.station_input_cost_scale = cfg.lqr.input_cost_scale;
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
