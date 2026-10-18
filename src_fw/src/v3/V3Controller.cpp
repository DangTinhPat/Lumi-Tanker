#include "tap_fw/v3/V3Controller.hpp"

#include "robot_control/BodyClearance.hpp"

#include <algorithm>
#include <cmath>

#include "robot_control/GeneratedGeometry.hpp"
#include "robot_control/TunedConfig.hpp"
#include "robot_control/WholeBodyModel.hpp"
#include "tap_fw/BoardConfig.hpp"

namespace tap_fw::v3 {

// State machine, safety transitions, controller handoffs and public defaults.

V3Controller::V3Controller(Config config)
    : config_(config), lqr_(config.lqr),
      com_reference_m_(LegHeight::forKnee(LegHeight::kKneeMaxRad).com_above_axle_m) {
  // Initial pendulum length (legs at the power-up pose); lqrUpdate follows the pose.
  lqr_.setComHeight(config_.com_height_m);
  knee_target_rad_ = knee_cmd_rad_ = LegHeight::kKneeMaxRad;
  pose_ = LegHeight::forKnee(LegHeight::kKneeMaxRad);
  height_target_m_ = pose_.axle_below_hip_m;
}

StandUp::Input V3Controller::recoveryInput(const Input& raw) const {
  StandUp::Input si{};
  si.dt_s = raw.dt_s;
  si.pitch_rad = raw.pitch_rad;
  si.pitch_rate_rad_s = raw.pitch_rate_rad_s;
  const auto capture = comLine(hip_meas_, knee_meas_);
  si.capture_lean_rad = raw.pitch_rad + capture.angle;
  si.capture_rate_rad_s = raw.pitch_rate_rad_s +
      capture.d_hip * hip_rate_meas_ + capture.d_knee * knee_rate_meas_;
  si.hip_jacobian = capture.d_hip;
  si.hip_measured_rad = hip_meas_;
  si.hip_measured_rate_rad_s = hip_rate_meas_;
  si.wheel_measured_rad_s = raw.wheel_rad_s;
  si.carrier_rate_rad_s = raw.pitch_rate_rad_s + hip_rate_meas_ + knee_rate_meas_;
  si.ground_speed_m_s = ground_v_;
  si.wheel_radius_m = config_.wheel_radius_m;
  if (config_.standup.floor_guard || config_.standup.guarded_handoff ||
      (config_.standup.overlap_after_lift && !config_.standup.hold_after_flip)) {
    const float hang = LegHeight::hangHipCad();
    si.body_clearance_m = robot_control::bodyClearance(raw.pitch_rad, roll_est_,
        {hang + raw.hip_rad[0], hang + raw.hip_rad[1]}, raw.knee_rad,
        config_.wheel_radius_m);
    si.body_clearance_valid = std::isfinite(si.body_clearance_m);
  }
  if (config_.standup.floor_guard &&
      (standup_.phase() == StandUp::Phase::Catching ||
       standup_.phase() == StandUp::Phase::Support)) {
    const float hang = LegHeight::hangHipCad();
    const auto floor = robot_control::bodyClearanceDamper(raw.pitch_rad, roll_est_,
        {raw.roll_rate_rad_s, raw.pitch_rate_rad_s, raw.yaw_rate_rad_s},
        {hang + raw.hip_rad[0], hang + raw.hip_rad[1]}, raw.knee_rad,
        raw.hip_rate_rad_s, raw.knee_rate_rad_s, config_.wheel_radius_m,
        config_.standup.floor_clearance_m, config_.standup.floor_approach_time_s,
        config_.standup.floor_prediction_lag_s, config_.standup.catch_hip_rate_max_rad_s,
        standup_state_.hip_target_rad - hip_meas_, capture.d_hip);
    si.floor_bounds_valid = floor.valid;
    si.floor_bounds_feasible = floor.feasible;
    si.floor_clearance_rate_m_s = floor.clearance_rate_m_s;
    si.floor_predicted_clearance_m = floor.predicted_clearance_m;
    si.floor_hip_rate_lower_rad_s = floor.hip_rate_lower_rad_s;
    si.floor_hip_rate_upper_rad_s = floor.hip_rate_upper_rad_s;
  }
  const float length = capture.pitch_inertia_at_axle /
      (robot_control::geometry::total_mass_kg * std::max(capture.height_m, 0.01F));
  si.capture_omega_rad_s = std::sqrt(9.80665F / std::max(length, 0.02F));
  si.sensors_ok = raw.sensors_ok && raw.legs_measured;
  return si;
}

V3Controller::Output V3Controller::fault(const char* reason, const Output& base) {
  if (state_ == State::StandUp && legs_held_) {
    // The adapter reads hipCommand()/kneeCommand() outside StandUp, including
    // Fault. Retain the actual recovery targets before changing state, or a
    // pitch/feedback trip would send the old nominal folded pose to the legs.
    pose_ = LegHeight::forKnee(standup_state_.knee_target_rad);
    dhip_ = standup_state_.hip_target_rad - pose_.hip_offset_rad;
    dknee_ = axle_hip_correction_ = 0.0F;
    knee_cmd_rad_ = knee_target_rad_ = standup_state_.knee_target_rad;
    height_target_m_ = pose_.axle_below_hip_m;
    recovery_pose_held_ = true;
  }
  lqr_prediction_.reset();
  resetGuard();
  state_ = State::Fault;
  last_fault_ = reason;
  #ifdef TAP_SPLIT_HOST_PROTOTYPE
  if (config_.split.enabled) {
    split_alpha_b_ = split_hold_s_ = split_b_ = 0.0F;
    split_frozen_ = true;
  }
  #endif

  Output out = base;
  out.state = state_;
  out.legs_locked = legs_held_;  // wheels stop, legs keep holding
  out.wheels_enabled = false;
  out.wheel_cmd_rad_s = {0.0F, 0.0F};
  out.fault = reason;
  return out;
}

V3Controller::Output V3Controller::update(const Input& raw, const Command& cmd_in) {
  wheel_carrier_rad_s_.fill(0.0F);
  bool recovery_roll_updated = false;
  bool recovery_just_engaged = false;
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
  // Pre-solve the LQR gain table over the height range, one point per tick
  // (idempotent once ready). Each point costs ~50 ms on the ESP32-S3, which
  // blocks the control loop for that long: fine at Idle/boot, but during Locking
  // it starves the wheel UART feedback past its 20 ms freshness window and
  // faults "sensor feedback lost while locking". So it runs from the very first
  // tick; the ~12-point / ~0.6 s table is done before the operator presses START.
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
  // to the axle odometry, as MotorFeedback::writeTo does.
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
  // The hips swing at up to ~1 rad/s and the 10 Hz filtered difference lags
  // that, so the tiers and the whole-body path use the drivers' velocity estimates.
  ground_v_ = config_.wheel_radius_m *
      (mean_rate + raw.pitch_rate_rad_s +
       (tierVirtual() || wholeBody() ? chain_rate_meas_ : chain_rate_f_));
  ground_x_ = config_.wheel_radius_m * (mean_pos + raw.pitch_rad + raw.leg_chain_rad);

  // Tier 1: the balance loop, fall, runaway, held checks and height quality use
  // the virtual pitch. Ready's operator-motion rate stays on the body IMU; its
  // balance seed still uses the virtual pitch.
  updateVirtualPitch(raw);
  standup_lean_ = theta_v_ + com_line_hang_;
  Input in = raw;
  in.pitch_rad = theta_v_;
  in.pitch_rate_rad_s = theta_v_rate_;
  if (state_ != State::Balancing) {
    wheel_leg_alpha_ = 0.0F;
    wheel_leg_quiet_since_us_ = 0U;
    sus_vz_ = sus_dz_ = sus_dk_ = 0.0F;
    sus_alpha_ = 0.0F;
    height_settled_since_us_ = 0U;
    wb_lean_init_ = false;
    if (excite_applied_) {
      knee_excite_rad_ = 0.0F;
      excite_applied_ = false;
      pose_ = LegHeight::forKnee(knee_cmd_rad_);
    }
  }
  if (state_ != State::Balancing && !recovery_pose_held_) {
    updateLegLayers(in, false, {0.0F, 0.0F});
    const float step = std::max(config_.tier.rate_rad_s, 0.0F) * in.dt_s;
    dknee_ -= std::clamp(dknee_, -step, step);  // whole-body knee offset back to the pose
    axle_hip_correction_ -= std::clamp(axle_hip_correction_, -step, step);
    axle_limited_ = false;
  }

  Output out{};
  if (cmd.disable) {
    lqr_prediction_.reset();
    resetGuard();
    standup_.reset();
    standup_state_ = {};
    recovery_pose_held_ = false;
    legs_held_ = false;
    #ifdef TAP_SPLIT_HOST_PROTOTYPE
    if (config_.split.enabled) {
      split_a_ = split_b_ = split_alpha_b_ = split_hold_s_ = split_integral_ = 0.0F;
      split_preview_ = split_model_target_ = 0.0F;
      split_was_enabled_ = false; split_frozen_ = true;
    }
    #endif

    dhip_ = dknee_ = trunk_alpha_ = level_beta_ = 0.0F;  // legs released: no offsets left over
    axle_hip_correction_ = 0.0F;
    axle_limited_ = false;
    state_ = State::Idle;
    out.state = state_;
    return out;
  }

  out.legs_locked = legs_held_;
  switch (state_) {
    case State::Idle:
    case State::Fault:
      if (cmd.standup && config_.standup.enabled) {
        // Fold to the support-pose encoder target, then capture the CoM with
        // hips and wheels; body leveling depends on the recovery profile.
        if (!in.sensors_ok || !legs_measured_) return fault("sensors not ready", out);
        if (config_.algo != Algo::Lqr || !config_.tier.virtual_pitch)
          return fault("stand-up requires virtual-pitch LQR", out);
        state_ = State::StandUp;
        recovery_pose_held_ = false;
        state_since_us_ = in.now_us;
        runaway_since_us_ = 0U;
        out.engage_legs = !legs_held_;
        legs_held_ = true;
        out.legs_locked = true;
        knee_target_rad_ = knee_cmd_rad_ = LegHeight::kKneeMinRad;
        pose_ = LegHeight::forKnee(knee_cmd_rad_);
        height_target_m_ = pose_.axle_below_hip_m;
        height_speed_m_s_ = height_accel_m_s2_ = 0.0F;
        height_startup_ = false;
        dhip_ = dknee_ = axle_hip_correction_ = 0.0F;
        standup_.config() = config_.standup;
        lqr_prediction_.reset();
        const float hip = 0.5F * (raw.hip_rad[0] + raw.hip_rad[1]);
        standup_.begin(hip, pose_.knee_offset_rad);
        standup_state_ = {};
        standup_state_.phase = standup_.phase();
        standup_state_.hip_target_rad = std::isfinite(hip) ? hip : 0.0F;
        standup_state_.knee_target_rad = pose_.knee_offset_rad;
        out.leg_hip_offset_rad = 0.5F * (raw.hip_rad[0] + raw.hip_rad[1]);
        out.leg_knee_offset_rad = pose_.knee_offset_rad;
        out.state = state_;
        return out;
      }
      if (cmd.start) {
        if (!in.sensors_ok) return fault("sensors not ready", out);
        recovery_pose_held_ = false;
        state_ = State::Locking;
        state_since_us_ = in.now_us;
        runaway_since_us_ = 0U;
        if (!legs_held_) {
          // Fresh lock: the legs hang straight (fully extended) at the power-up pose, which is the controller's
          // knee kKneeMaxRad / 184.8 mm (AppV3 adds that home offset to the encoder angles). The pose and the
          // height target start there, not at the folded knee 0 / 38 mm the encoder zero used to be taken for.
          knee_target_rad_ = knee_cmd_rad_ = LegHeight::kKneeMaxRad;
          pose_ = LegHeight::forKnee(LegHeight::kKneeMaxRad);
          height_target_m_ = pose_.axle_below_hip_m;
          height_speed_m_s_ = 0.0F;
          height_accel_m_s2_ = 0.0F;
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
        resetReadyStill();
      }
      break;
    case State::Ready: {
      // Wheels hold still (torque on, zero speed); no balance loop yet.
      out.legs_locked = true;
      out.wheels_enabled = true;
      if (!in.sensors_ok) return fault("sensor feedback lost while ready", out);
      const Engage& e = config_.engage;
      // Ready detects the operator carrying/holding/releasing the BODY.
      // The virtual rate includes driver joint velocities: at full height
      // their chatter can reset the still timer forever, even though the
      // joints' positions and the bias-corrected IMU body rate stay quiet.
      // It can also falsely trigger release after arming. Keep the virtual
      // pitch angle for the balance seed, but use IMU rate for this handshake.
      const float rate = std::abs(raw.pitch_rate_rad_s);
      const bool armed_only = e.wb_armed_only && config_.algo == Algo::WholeBody;
      bool go = !armed_only && (cmd.go || e.immediate);
      float armed_seed = in.pitch_rad;
      if ((e.automatic || armed_only) && !go) {
        if (ready_phase_ == 0) {
          if (rate > e.move_rate_rad_s ||
              std::abs(in.pitch_rad - ready_entry_pitch_) > e.move_pitch_rad) {
            ready_phase_ = 1;
            resetReadyStill();
          }
        } else if (ready_phase_ == 1) {
          if (readyStill(in, raw.pitch_rad, rate)) {
            ready_phase_ = 2;
            ready_armed_us_ = in.now_us;
            armed_pitch_ = still_sum_ / static_cast<float>(std::max(still_n_, 1));
            ready_tip_us_ = 0U;
          }
        } else {
          // A short rate spike tolerated by the quiet window must not
          // immediately release the robot on the following ARMED tick.
          const bool gap = in.now_us <= ready_last_us_ || in.now_us-ready_last_us_ > 20'000U;
          const auto dt_us = static_cast<std::uint64_t>(
              std::clamp(in.dt_s, 0.0F, 0.01F) * 1.0e6F);
          if (gap || rate <= e.tip_rate_rad_s) ready_tip_us_ = 0U;
          else ready_tip_us_ += dt_us;
          ready_last_us_ = in.now_us;
          go = ready_tip_us_ >= 20'000U ||
              std::abs(in.pitch_rad - armed_pitch_) > e.tip_pitch_rad ||
              (e.armed_go_s > 0.0F && in.now_us - ready_armed_us_ >=
                   static_cast<std::uint64_t>(e.armed_go_s * 1.0e6F));
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
    case State::StandUp: {
      out.legs_locked = true;
      out.wheels_enabled = false;     // set from the sequence below
      if (!in.sensors_ok || !legs_measured_)
        return fault("sensor feedback lost during the stand-up", out);
      if (cmd.standup_advance) standup_.advanceFromKneeling();
      if (config_.standup.floor_guard || config_.standup.guarded_handoff ||
          (config_.standup.overlap_after_lift && !config_.standup.hold_after_flip)) {
        if (!std::isfinite(raw.roll_rate_rad_s) || !std::isfinite(raw.accel_y_m_s2) ||
            !std::isfinite(raw.accel_z_m_s2) || !std::isfinite(raw.accel_x_m_s2))
          return fault("invalid IMU for recovery clearance", out);
        // Keep the existing IMU roll estimator alive during recovery, with
        // roll actuation disabled. Do not update it twice on the handoff tick.
        updateRoll(raw, false);
        recovery_roll_updated = true;
      }
      const auto si = recoveryInput(raw);
      standup_lean_ = si.capture_lean_rad;
      standup_state_ = standup_.update(si);
      if (standup_state_.phase == StandUp::Phase::Failed)
        return fault(StandUp::failureName(standup_state_.failure), out);
      if (standup_state_.phase == StandUp::Phase::Support) {
        // Preserve the last recovery command; rebasing to measured encoders
        // would step the target by the driver's tracking error at handoff.
        const float hip = standup_state_.hip_target_rad;
        const float knee = standup_state_.knee_target_rad;
        // Keep V3's nominal pose on its supported height manifold. The hip
        // offset from that pose carries the commanded recovery geometry through
        // the handoff, because updateHeight() recomputes pose_ from the knee.
        pose_ = LegHeight::forKnee(knee);
        dhip_ = hip - pose_.hip_offset_rad;
        axle_hip_correction_ = 0.0F;
        height_target_m_ = pose_.axle_below_hip_m;
        knee_cmd_rad_ = knee_target_rad_ = knee;
        // updateVirtualPitch cached the upright-hip Jacobian during the fold;
        // invalidate it for the newly measured kneeling configuration.
        j_hip_knee_ = std::numeric_limits<float>::quiet_NaN();
        updateVirtualPitch(raw);
        in.pitch_rad = theta_v_;
        in.pitch_rate_rad_s = theta_v_rate_;

        // Physical CoM zero is virtual pitch = -hang reference. Seed speed
        // and retain the commands still travelling through the wheel delay.
        const float recovery_trim = config_.standup.catch_capture_velocity
            ? standup_state_.capture_equilibrium_rad : 0.0F;
        engageBalance(in, recovery_trim - com_line_hang_, true);
        recovery_pose_held_ = config_.standup.capture_first;
        if (recovery_pose_held_) dknee_ = 0.0F;
        lqr_.seedForwardVelocity(ground_v_);
        pid_v_filt_ = ground_v_;
        // The preview planner owns LQR's velocity reference when enabled.
        // Seed it too: otherwise its stale zero overwrites the rolling seed
        // on this very tick, and the station loop banks the braking travel as
        // position error. Plan the stop from the measured capture velocity.
        plan_v_ = ground_v_;
        plan_a_ = plan_j_ = 0.0F;
        plan_head_ = plan_n_ = 0U;
        plan_init_ = true;
        pid_driving_ = true;  // reanchor after the planned stop, as after DRIVE
        // A stand-up handoff has already established its leg height. Skip the
        // normal powered-up startup-height ramp so it cannot overwrite the
        // kneeling hip offset before the wheel controller catches the COM.
        height_startup_ = false;
        height_target_m_ = pose_.axle_below_hip_m;
        recovery_just_engaged = true;
        break;
      }
      out.wheels_enabled = standup_state_.wheels_on;
      out.leg_hip_offset_rad = standup_state_.hip_target_rad;
      out.leg_knee_offset_rad = standup_state_.knee_target_rad;
      out.legs_moving = true;
      if (standup_state_.wheels_on) {
        out.wheel_cmd_rad_s = {standup_state_.wheel_rad_s, standup_state_.wheel_rad_s};
      }
      lqr_prediction_.record(config_.wheel_radius_m * standup_state_.wheel_rad_s,
                             config_.lqr_prediction, in.dt_s);
      out.state = state_;
      return out;
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
    // The operator permits more backward trunk lean in the retained recovery
    // pose. Keep the independent physical-COM/virtual-pitch fall check above
    // and the normal forward body bound. Clearance filtering still applies.
    const float body_pitch_limit = recovery_pose_held_ && config_.standup.capture_first &&
        raw.pitch_rad < 0.0F && config_.standup.back_pitch_limit_rad > 0.0F
        ? std::max(config_.fall_pitch_rad, config_.standup.back_pitch_limit_rad)
        : config_.fall_pitch_rad;
    if (std::abs(raw.pitch_rad) > body_pitch_limit) {
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
    // Ground speed as the accelerometer sees it (horizontal specific force
    // integrated, pulled to the wheels over runaway_v_tau_s): a free robot
    // reached its wheel speed by accelerating, a held one did not.
    const float a_world = raw.accel_x_m_s2 * std::cos(body_pitch_) +
        raw.accel_z_m_s2 * std::sin(body_pitch_);
    runaway_v_imu_ += in.dt_s * a_world +
        in.dt_s / std::max(config_.runaway_v_tau_s, 0.1F) * (ground_v_ - runaway_v_imu_);
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
      // ...and the wheels ran away from the body: the legs can pin the body
      // (a pushed robot at 0.67 m/s kept theta_v within 0.15 and was
      // re-zeroed mid-catch, sim TIER_GENERAL episode 37).
      if (std::abs(in.pitch_rad - runaway_start_pitch_) < 0.15F &&
          std::abs(ground_v_ - runaway_v_imu_) > config_.runaway_dv_m_s) {
        heldRezero(in);
        out.wheel_cmd_rad_s = {0.0F, 0.0F};
        out.state = state_;
        return out;
      }
    }
    if (!recovery_pose_held_) {
      updateHeight(in, cmd);        // nominal height profile (knee_cmd_rad_)
      applyPoseOffsets(in);         // suspension / KNEETEST offsets on top of it
      updateWheelLegCoordination(in);
    }
    if (wholeBody()) {
      axle_hip_correction_ = 0.0F;
      axle_limited_ = false;
      out.wheel_cmd_rad_s = wbUpdate(in, cmd);
      if (wb_ring_fault_) {
        wb_ring_fault_ = false;
        return fault("whole-body loop ringing (wheel command cap to cap)", out);
      }
    } else {
      out.wheel_cmd_rad_s = config_.algo == Algo::Pid ? pidUpdate(in, cmd) : lqrUpdate(in, cmd);
      // Keep recovery knee/height; StandUp optionally adds gated body leveling.
      // Optionally retain
      // the SAME bounded CoM hip allocator used in Catching: the 16:50 log
      // showed freezing that actuator at handoff left the wheel loop ringing.
      if (!recovery_pose_held_) {
        updateLegLayers(in, true, out.wheel_cmd_rad_s);
        coordinateTrunkAxle(in.dt_s);
      } else if (config_.standup.support_hip_capture && !recovery_just_engaged &&
                 !held_outer_off_ && !held_quiet_) {
        auto si = recoveryInput(raw);
        // Virtual-pitch LQR reference -> physical axle-to-COM reference.
        // Its speed/station loop owns this target after handoff; do not run
        // Catching's independent speed-to-lean outer loop at the same time.
        si.support_lean_ref_rad = pid_ref_ + com_line_hang_;
        standup_state_ = standup_.update(si);
        standup_lean_ = si.capture_lean_rad;
        if (standup_state_.phase == StandUp::Phase::Failed)
          return fault(StandUp::failureName(standup_state_.failure), out);
        dhip_ = standup_state_.hip_target_rad - pose_.hip_offset_rad;
        dknee_ = axle_hip_correction_ = 0.0F;
        // Support's wheel diagnostics describe the actual LQR owner, not the
        // inactive catch-wheel fields returned by the hip-only update.
        standup_state_.wheel_rad_s = 0.5F *
            (out.wheel_cmd_rad_s[0] + out.wheel_cmd_rad_s[1]);
        standup_state_.wheel_tracking_error_rad_s = std::max(
            std::abs(out.wheel_cmd_rad_s[0] - raw.wheel_rad_s[0]),
            std::abs(out.wheel_cmd_rad_s[1] - raw.wheel_rad_s[1]));
        standup_state_.wheel_limited =
            std::abs(out.wheel_cmd_rad_s[0]) >= config_.standup.catch_wheel_max_rad_s ||
            std::abs(out.wheel_cmd_rad_s[1]) >= config_.standup.catch_wheel_max_rad_s;
        standup_state_.wheel_accel_m_s2 = lqr_.debugState().optimal_command_m_s;
      }
    }
    out.leg_hip_offset_rad = hipCommand();
    out.leg_knee_offset_rad = kneeCommand();
    out.legs_moving = recovery_pose_held_
        ? (config_.standup.support_hip_capture && !held_outer_off_ && !held_quiet_ &&
           std::abs(standup_state_.hip_rate_rad_s) > 1.0e-5F)
        : knee_cmd_rad_ != kneeGoalRad();
    if (!recovery_roll_updated)
      updateRoll(raw, !recovery_pose_held_ && !held_outer_off_ && !held_quiet_ && legs_measured_);
  } else {
    if (!recovery_roll_updated) updateRoll(raw, false);
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

void V3Controller::engageBalance(const Input& in, float seed, bool preserve_wheel_history) {
  recovery_pose_held_ = false;
  if (!preserve_wheel_history) lqr_prediction_.reset();
  resetGuard();
  state_ = State::Balancing;
  ga_init_ = false;
  runaway_v_imu_ = ground_v_;
  state_since_us_ = in.now_us;
  height_startup_ = true;
  // The slow startup stage first moves to the supported height (knee reserve for
  // trunk/roll travel). A stored deep-crouch request is applied afterwards at the
  // normal operator height speed.
  const float hmax = supportedMaxHeight();
  startup_height_m_ = std::min(LegHeight::forKnee(knee_cmd_rad_).axle_below_hip_m, hmax);
  height_speed_m_s_ = 0.0F;
  height_accel_m_s2_ = 0.0F;
  height_settled_since_us_ = 0U;
  sus_vz_ = sus_dz_ = sus_dk_ = sus_alpha_ = 0.0F;
  runaway_since_us_ = 0U;
  lqr_.reset(ground_x_, true);  // anchors LQR position to ground_x_ too
  lqr_.setBalanceTrim(seed);
  lqr_x_target_m_ = ground_x_;
  stand_give_m_ = 0.0F;
  stand_leash_wait_s_ = std::max(config_.stand_leash_release_s, 0.0F);
  stand_leash_on_ = false;
  lqr_lead_filt_ = 0.0F;
  model_bad_since_us_ = 0U;
  // theta_b is the balance angle at the power-up height; the height
  // feed-forward (PID / tiers-off paths) adds the model change on top.
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
  stand_give_m_ = 0.0F;
  held_quiet_ = true;
  held_outer_off_ = true;
  held_quiet_since_us_ = in.now_us;
  held_window_start_us_ = 0U;
  runaway_since_us_ = 0U;
}

void V3Controller::resetReadyStill() noexcept {
  still_since_us_ = ready_last_us_ = 0U;
  still_quiet_us_ = still_bad_us_ = still_bad_run_us_ = ready_tip_us_ = 0U;
  still_sum_ = 0.0F;
  still_n_ = 0;
  still_quiet_ = false;
}

bool V3Controller::readyStill(const Input& in, float body_pitch, float body_rate) noexcept {
  const auto& e = config_.engage;
  const float enter = e.still_rate_rad_s;
  const float exit = 1.5F * enter;
  const bool gap = ready_last_us_ != 0U &&
      (in.now_us <= ready_last_us_ || in.now_us-ready_last_us_ > 20'000U);
  // A real carry/tip, clock gap, or >0.86 degree change restarts the whole
  // window. The angle bound rejects slow monotonic motion even below the
  // rate threshold. Use BODY angle/rate, not encoder-induced virtual motion.
  if (gap || !std::isfinite(body_rate) || !std::isfinite(body_pitch) || !std::isfinite(in.pitch_rad) ||
      body_rate > 2.0F * exit) {
    resetReadyStill();
    return false;
  }
  if (still_since_us_ == 0U) {
    if (body_rate >= enter) return false;
    still_since_us_ = ready_last_us_ = in.now_us;
    still_body_min_ = still_body_max_ = body_pitch;
    still_quiet_ = true;
    return false;
  }
  const auto elapsed_us = in.now_us-ready_last_us_;
  const auto dt_us = std::min<std::uint64_t>(elapsed_us, 10'000U);
  ready_last_us_ = in.now_us;
  still_body_min_ = std::min(still_body_min_, body_pitch);
  still_body_max_ = std::max(still_body_max_, body_pitch);
  if (still_body_max_-still_body_min_ > std::max(e.still_span_rad, 0.0F)) {
    resetReadyStill();
    return false;
  }
  if (body_rate >= exit) still_quiet_ = false;
  else if (body_rate < enter) still_quiet_ = true;
  if (still_quiet_) {
    still_quiet_us_ += dt_us;
    still_bad_run_us_ = 0U;
    still_sum_ += in.pitch_rad;  // virtual pitch remains the balance seed
    ++still_n_;
  } else {
    still_bad_us_ += dt_us;
    still_bad_run_us_ += dt_us;
  }
  // Default 700 ms quiet plus at most 100 ms disturbed in total; no run
  // of disturbance longer than 70 ms. Brief vibrations lose only their
  // own time; they do not throw away the entire preceding quiet interval.
  if (still_bad_us_ > e.still_us/7U || still_bad_run_us_ > e.still_us/10U) {
    resetReadyStill();
    return false;
  }
  return still_quiet_ && still_quiet_us_ >= e.still_us && still_n_ > 0;
}

const char* toString(V3Controller::State state) noexcept {
  switch (state) {
    case V3Controller::State::Idle: return "Idle";
    case V3Controller::State::Locking: return "Locking";
    case V3Controller::State::Ready: return "Ready";
    case V3Controller::State::Balancing: return "Balancing";
    case V3Controller::State::Fault: return "Fault";
    case V3Controller::State::StandUp: return "StandUp";
  }
  return "?";
}

V3Controller::Config defaultV3Config() {
  V3Controller::Config cfg{};
  cfg.algo = V3Controller::Algo::Lqr;  // LQR is the shipped balancer (SET algo 1 = PID)
  cfg.lqr = robot_control::tunedControllerConfig().lqr;
  cfg.lqr.wheel_pitch_feedback_scale = 1.0F;
  // LqrBalancer's own balance_trim integrates ground-SPEED error, which can
  // settle at zero at any station-hold position offset (it stuck near +0.10 m in
  // the physics tests). V3Controller runs its own position-error trim integral
  // instead (lqrUpdate), so the built-in one stays off.
  cfg.lqr.balance_trim_gain_rad_per_m = 0.0F;
  cfg.lqr.balance_trim_max_rad = 0.20F;
  cfg.lqr.balance_trim_pitch_gate_rad = 0.25F;
  // Wheel limits as in v3.5.2: ground-speed cap = the wheel loop's command cap at 1000 rpm x 0.04 m = 4.1888 m/s
  // (the Kpower BL4012 datasheet no-load speed is 950 rpm = 3.98 m/s at 12 V), command-vs-measured margin 0.25 m/s.
  // SET vmax / SET margin change them at run time (0.8 m/s was the earlier cap).
  cfg.lqr.command_velocity_tracking_margin_m_s = 0.25F;
  cfg.lqr.max_ground_velocity_m_s = 4.1888F;
  // Demo turning: 1.5 rad/s (~86 deg/s) = 0.23 m/s per wheel at the 0.1545 m
  // half track; the default 0.8 capped the BLE stick well below that.
  cfg.lqr.max_yaw_rate_rad_s = 1.5F;
  // tunedControllerConfig() derives this from a 0.13 N.m torque spec that
  // predates the BL4820 wheels; the PID's accel_max_m_s2 = 8.0 is the value
  // validated on this hardware, so the LQR gets the same authority (a ~6x weaker
  // stale cap could not recover a push).
  cfg.lqr.max_ground_accel_m_s2 = cfg.pid.accel_max_m_s2;
  // Hold ground position from the start (LqrBalancer::update force-zeroes the
  // position error unless both hold_station and position_anchor_hold are set;
  // V3Controller always passes hold_station=true and manages the anchor itself via
  // reanchorPosition).
  cfg.lqr.position_anchor_hold = true;
  // Provisional GIM CAD CoM range; keep margin around the low knee stop.
  cfg.lqr.min_pendulum_length_m = 0.035F;
  cfg.lqr.max_pendulum_length_m = 0.20F;
  // Input cost for the real wheel delay (the BL4820 speed servo answers after
  // ~50 ms dead time + a ~75 ms lag): 20 with the lead and a deadband push of 0.3
  // cut the worst pitch-rate rms 1.1 -> 0.25 rad/s in the delayed-wheel sweep with
  // no falls. Softer (25+) gave up push/slope recovery; 100+ falls.
  cfg.lqr.input_cost_scale = 20.0F;
  // The station-gain blend (posture capture) is not used: mirror every station_*
  // cost scale onto its base scale so both gain sets solve to the same values.
  cfg.lqr.station_position_cost_scale = cfg.lqr.position_cost_scale;
  cfg.lqr.station_pitch_cost_scale = cfg.lqr.pitch_cost_scale;
  cfg.lqr.station_velocity_cost_scale = cfg.lqr.velocity_cost_scale;
  cfg.lqr.station_pitch_rate_cost_scale = cfg.lqr.pitch_rate_cost_scale;
  cfg.lqr.station_input_cost_scale = cfg.lqr.input_cost_scale;
  cfg.wheel_radius_m = robot_control::geometry::wheel_radius_m;
  cfg.fall_pitch_rad = 0.60F;
  // COM height of the legs locked extended (hanging-straight pose).
  robot_control::WholeBodyModel model;
  const float knee = robot_control::geometry::native_knee_max_rad;
  const float hip = LegHeight::hangHipCad() + LegHeight::forKnee(knee).hip_offset_rad;
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
