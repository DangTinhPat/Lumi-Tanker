#include "tap_fw/v3/V3Controller.hpp"
#include "robot_control/HotPath.hpp"

#include "robot_control/BodyClearance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "robot_control/GeneratedGeometry.hpp"
#include "robot_control/TunedConfig.hpp"
#include "robot_control/WholeBodyModel.hpp"
#include "tap_fw/BoardConfig.hpp"

namespace tap_fw::v3 {

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
  si.knee_measured_rad = knee_meas_;
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
  hop_.abort();
  hopEnd();
  hop_arm_ = false;
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

namespace {

// Desired acceleration of a jerk-limited S-curve towards target: a ramp-down at k*jmax lands on the target
// (a = sqrt(2 k jmax |err|)), a linear blend of width tau closes the last bit, and a stop speed that already
// overshoots brakes at amax. tau <= 0 is bang-bang.
float sCurveAccel(float target, float v, float a, float amax, float jmax, float tau) {
  const float v_stop = v + a * std::abs(a) / (2.0F * jmax);
  const float err = target - v;
  const float err_stop = target - v_stop;
  if (tau <= 1.0e-3F) return std::abs(err_stop) < 1.0e-4F ? 0.0F : std::copysign(amax, err_stop);
  constexpr float kFollow = 0.8F;
  const float mag = std::min(std::sqrt(2.0F * kFollow * jmax * std::abs(err)), std::abs(err) / tau);
  if (err * err_stop < 0.0F) return std::copysign(amax, err_stop);
  return std::clamp(std::copysign(mag, err), -amax, amax);
}

}  // namespace

TAP_HOT V3Controller::Output V3Controller::update(const Input& raw, const Command& cmd_in) {
  wheel_carrier_rad_s_.fill(0.0F);
  bool recovery_roll_updated = false;
  bool recovery_just_engaged = false;
  // Roll on: the turn command is slewed (Roll::yaw_accel_rad_s2) so the
  // lean-in can build with the centripetal pull instead of after it.
  Command cmd = cmd_in;
  {
    float ya = config_.tier.yaw_accel_rad_s2;
    if (config_.roll.enabled && config_.roll.yaw_accel_rad_s2 > 0.0F)
      ya = ya > 0.0F ? std::min(ya, config_.roll.yaw_accel_rad_s2) : config_.roll.yaw_accel_rad_s2;
    const float target = std::clamp(cmd_in.yaw_rad_s, -config_.lqr.max_yaw_rate_rad_s,
                                    config_.lqr.max_yaw_rate_rad_s);
    if (ya > 0.0F && raw.dt_s > 0.0F) {
      const float jy = std::max(config_.tier.yaw_jerk_rad_s3, ya);
      const float a_des = sCurveAccel(target, yaw_cmd_, yaw_acc_, ya, jy, 0.05F);
      yaw_acc_ += std::clamp(a_des - yaw_acc_, -jy * raw.dt_s, jy * raw.dt_s);
      yaw_cmd_ += yaw_acc_ * raw.dt_s;
      if (std::abs(target - yaw_cmd_) < 1.0e-3F && std::abs(yaw_acc_) <= 2.0F * jy * raw.dt_s) {
        yaw_cmd_ = target;
        yaw_acc_ = 0.0F;
      }
    } else {
      yaw_cmd_ = target;
      yaw_acc_ = 0.0F;
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
  // Yaw gyro zero-rate offset. Quiet = no turn asked, nearly stopped, body not rocking. Balancing: the wheels' own
  // yaw (R (wR - wL) / track) is the reference, so a robot that really turns is not mistaken for bias. Otherwise
  // (lying, ready, stand-up) the robot sits still and the gyro itself is the reference.
  {
    const float tau = config_.tier.yaw_bias_tau_s;
    if (tau <= 1.0e-3F) {
      yaw_bias_ = 0.0F;
    } else if (raw.dt_s > 0.0F && std::abs(yaw_cmd_) < 0.01F && std::abs(raw.pitch_rate_rad_s) < 0.5F) {
      const float track = 2.0F * std::max(config_.lqr.half_wheel_base_m, 0.05F);
      const float yaw_wheels = config_.wheel_radius_m * (raw.wheel_rad_s[1] - raw.wheel_rad_s[0]) / track;
      const bool balancing = state_ == State::Balancing;
      const float reference = balancing ? yaw_wheels : 0.0F;
      const float seen = raw.yaw_rate_rad_s - reference;
      const bool quiet = balancing ? std::abs(ground_v_) < 0.08F && std::abs(yaw_wheels) < 0.3F
                                   : std::abs(raw.yaw_rate_rad_s - yaw_bias_) < 0.15F;
      if (quiet && std::abs(seen) < 0.2F) {
        const float t = balancing ? tau : 0.5F * tau;
        yaw_bias_ += std::clamp(raw.dt_s / t, 0.0F, 1.0F) * (seen - yaw_bias_);
      }
    }
  }
  Input in = raw;
  in.yaw_rate_rad_s = raw.yaw_rate_rad_s - yaw_bias_;
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
    hop_.abort();
    hopEnd();
    standup_.reset();
    standup_state_ = {};
    recovery_pose_held_ = false;
    hip_release_ramp_ = false;
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
        const bool seed_measured = config_.standup.balance_trim_seed_measured;
        engageBalance(in, seed_measured ? releaseBalance(in) : recovery_trim - com_line_hang_, true);
        trim_window_on_ = config_.standup.balance_trim_window_rad > 0.0F;
        trim_window_since_us_ = in.now_us;
        if (trim_window_on_ && !seed_measured) {
          lqr_.setBalanceTrim(lqr_.balanceTrim() + config_.standup.balance_trim_seed_offset_rad);
        }
        trim_window_center_ = lqr_.balanceTrim();
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
        height_rise_ = config_.standup.post_handoff_rise;
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
    if (cmd.hop) {
      hop_arm_ = true;
      hop_arm_until_us_ = in.now_us + static_cast<std::uint64_t>(std::max(config_.hop.arm_wait_s, 0.0F) * 1.0e6F);
      hop_req_clearance_m_ = cmd.hop_clearance_m;
      hop_refusal_ = HopSequence::Refusal::None;
    }
    if (hop_arm_ && !hop_.active()) hopBegin(in, cmd);
    if (hop_.active()) {
      cmd.forward_m_s = 0.0F;   // the hop is a stand-still manoeuvre
      cmd.yaw_rad_s = 0.0F;
    }
    if (tierVirtual() && !hop_.active()) {
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
    const bool fast = !driving_recently && !hop_.active() &&
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
    const StandUp::Config& rise = config_.standup;
    const std::uint64_t held_us = in.now_us - state_since_us_;
    const bool rise_calm = std::abs(theta_v_ - pid_ref_) <= rise.rise_calm_pitch_rad &&
        std::abs(theta_v_rate_) <= rise.rise_calm_rate_rad_s;
    if (recovery_pose_held_ && height_rise_ && !recovery_just_engaged &&
        held_us >= static_cast<std::uint64_t>(std::max(rise.rise_delay_s, 0.0F) * 1.0e6F) &&
        (rise_calm || (rise.rise_wait_max_s > 0.0F && held_us >= static_cast<std::uint64_t>(
            (rise.rise_delay_s + rise.rise_wait_max_s) * 1.0e6F)))) {
      if (cmd.height_m > pose_.axle_below_hip_m + 0.005F) {
        // Hand the held kneeling pose to the two-tier balance. dhip_ already carries the hip StandUp set, so
        // the hip command is continuous; the knee plan starts from the held knee.
        recovery_pose_held_ = false;
        dknee_ = axle_hip_correction_ = 0.0F;
        j_hip_knee_ = std::numeric_limits<float>::quiet_NaN();
        hip_release_ramp_ = rise.release_hip_rate_rad_s > 0.0F;
      } else {
        height_rise_ = false;
      }
    }
    // The hop owns the leg path (hipCommand/kneeCommand) from its start to the end of Recover; the normal
    // height / suspension / tier-2 layers rest meanwhile and pick up from the unchanged pre-hop state.
    if (hop_.active()) {
      model_bad_since_us_ = 0U;
      hopStep(in);
    }
    if (!recovery_pose_held_ && !hip_release_ramp_ && !hop_leg_override_) {
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
      float hop_ff = 0.0F;
      if (hop_.active() && config_.hop.wheel_ff_gain != 0.0F && hop_.phase() != HopSequence::Phase::Flight &&
          hop_.phase() != HopSequence::Phase::Land && hop_.phase() != HopSequence::Phase::Recover) {
        // Shank rotation of the planned knee path (hip counter-rotates along the standing manifold), led by the
        // wheel's dead time and lag so the wheel's ground speed stays put while the shank swings.
        float acc = 0.0F;
        const float r = hop_.plannedKneeRate(config_.hop.wheel_delay_s, &acc);
        const float k = hop_knee_cmd_;
        const float rho = (LegHeight::forKnee(std::min(k + 0.01F, LegHeight::kKneeMaxRad)).hip_offset_rad -
                           LegHeight::forKnee(k).hip_offset_rad) / 0.01F;
        hop_ff = config_.hop.wheel_ff_gain * (1.0F + rho) * (r + config_.hop.wheel_lag_s * acc);
      }
      if (hop_wheels_free_) {
        // The wheels keep their last balance command in ground-speed terms: the rotation of the shank in the world
        // (body + hip + knee rate) is taken out so the legs' motion does not roll the axle.
        const float carrier = hop_carrier_lp_ - hop_carrier0_;
        out.wheel_cmd_rad_s = {
            hop_last_wheel_cmd_[0] - config_.lqr.left_wheel_sign * config_.hop.wheel_carrier_gain * carrier,
            hop_last_wheel_cmd_[1] - config_.lqr.left_wheel_sign * config_.hop.wheel_carrier_gain * carrier};
      } else {
        out.wheel_cmd_rad_s = config_.algo == Algo::Pid ? pidUpdate(in, cmd) : lqrUpdate(in, cmd);
        hop_last_wheel_cmd_ = out.wheel_cmd_rad_s;
      }
      if (hop_ff != 0.0F) {
        for (auto& w : out.wheel_cmd_rad_s) w -= config_.lqr.left_wheel_sign * hop_ff;
      }
      // Keep recovery knee/height; StandUp optionally adds gated body leveling.
      // Optionally retain
      // the SAME bounded CoM hip allocator used in Catching: the 16:50 log
      // showed freezing that actuator at handoff left the wheel loop ringing.
      if (!recovery_pose_held_ && !hop_leg_override_) {
        updateLegLayers(in, true, out.wheel_cmd_rad_s);
        coordinateTrunkAxle(in.dt_s);
      } else if (hop_tier2_on_) {
        updateLegLayers(in, true, out.wheel_cmd_rad_s);   // hip reflex only; the hop owns the knee and the axle
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
    if (hop_wheels_limp_) {
      out.wheels_enabled = false;   // the wheels coast through the push and the flight
      out.wheel_cmd_rad_s = {0.0F, 0.0F};
    }
    out.leg_hip_offset_rad = hipCommand();
    out.leg_knee_offset_rad = kneeCommand();
    out.legs_moving = hop_leg_override_ ? true : recovery_pose_held_
        ? (config_.standup.support_hip_capture && !held_outer_off_ && !held_quiet_ &&
           std::abs(standup_state_.hip_rate_rad_s) > 1.0e-5F)
        : knee_cmd_rad_ != kneeGoalRad();
    if (!recovery_roll_updated)
      updateRoll(raw, !recovery_pose_held_ && !hop_leg_override_ && !held_outer_off_ && !held_quiet_ && legs_measured_);
  } else {
    if (!recovery_roll_updated) updateRoll(raw, false);
  }
  out.leg_roll_hip_rad = roll_hip_;
  out.leg_roll_knee_rad = roll_knee_;
  out.leg_roll_hip_rate = roll_hip_rate_;
  out.leg_roll_knee_rate = roll_knee_rate_;
  wp_last_cmd_ = out.wheels_enabled ? 0.5F * (out.wheel_cmd_rad_s[0] + out.wheel_cmd_rad_s[1]) : 0.0F;
  updateImuVelocity(raw, state_ == State::Balancing && !held_outer_off_ && !held_quiet_ && !hop_wheels_free_);
  out.state = state_;
  return out;
}

TAP_HOT void V3Controller::updateLearnedTrim(const Input& in, float ex, float current_trim_rad) {
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

void V3Controller::engageBalance(const Input& in, float seed, bool /*preserve_wheel_history*/) {
  recovery_pose_held_ = false;
  hip_release_ramp_ = false;
  trim_window_on_ = false;
  height_rise_ = false;
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

TAP_HOT float V3Controller::lqrInertiaLength() noexcept {
  // I_axle / (m l) of the whole robot at the commanded pose (CAD masses and
  // inertia; wheels roll with the axle). Recomputed only when the pose moves.
  // A captured recovery pose can have a substantial hip offset from the
  // nominal height manifold. Use that retained geometry for wheel gains.
  const float knee = recovery_pose_held_ ? kneeCommand() : pose_.knee_offset_rad;
  const float hip_offset = recovery_pose_held_ ? hipCommand() : pose_.hip_offset_rad;
  if (std::abs(knee - lqr_leff_knee_) > 1.0e-3F ||
      std::abs(hip_offset - lqr_leff_hip_) > 1.0e-3F) {
    lqr_leff_knee_ = knee;
    lqr_leff_hip_ = hip_offset;
    const float hip = LegHeight::hangHipCad() + hip_offset;
    const auto b = robot_control::WholeBodyModel{}.compute(hip, knee, hip, knee);
    const float l = std::hypot(b.com_offset_x_m, b.com_above_axle_m);
    if (l > 0.01F && b.pitch_inertia_at_axle_kg_m2 > 0.0F) {
      lqr_leff_m_ = b.pitch_inertia_at_axle_kg_m2 /
          (robot_control::geometry::total_mass_kg * l);
    }
  }
  return lqr_leff_m_;
}

TAP_HOT std::array<float, 2> V3Controller::lqrUpdate(const Input& in, const Command& cmd) {
  const Pid& p = config_.pid;  // shared held/lifted thresholds and deadband comp
  const float radius = std::max(config_.wheel_radius_m, 1.0e-4F);
  // Retaining the recovery pose must not suddenly unlock the driving limits.
  // The 16:07 log crossed from 16 rad/s in Catching to 69 rad/s afterwards.
  const bool recovery_bounds = recovery_pose_held_ && config_.standup.coordinated_speed;
  const float vmax = recovery_bounds
      ? std::min(config_.lqr.max_ground_velocity_m_s, config_.standup.catch_wheel_max_rad_s * radius)
      : config_.lqr.max_ground_velocity_m_s;
  const float saved_vmax = lqr_.config().max_ground_velocity_m_s;
  const float saved_amax = lqr_.config().max_ground_accel_m_s2;
  const float recovery_amax = std::min(saved_amax, config_.standup.catch_accel_max);
  // The pendulum length tracks the current leg height every tick; with the gain
  // table built this is a cheap interpolation.
  lqr_.setComHeight(config_.lqr_inertia_length
      ? lqrInertiaLength() / std::max(config_.lqr.pendulum_length_per_com_height, 0.1F)
      : pose_.com_above_axle_m);
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
      // Stand-up handoff: use every sample (the centre of the oscillation), not only the quiet turning
      // points the gate keeps (StandUp::Config::balance_trim_ungated_tau_s).
      // The stand-up trim mode (ungated observer, +-window around the seed) is for the first seconds after the
      // handoff. Left on it clamps the lean a drive needs to +-0.06 rad around the seed and feeds the noisy ground
      // acceleration into the trim all the time; every run from the 21/10 drives came from a stand-up and rippled
      // 2-4x more than the 15:20 stood-up runs. A drive command or 8 s of balancing returns to the 15:20 observer.
      if (trim_window_on_ &&
          (std::abs(cmd.forward_m_s) > 1.0e-3F || std::abs(cmd.yaw_rad_s) > 1.0e-3F ||
           in.now_us - trim_window_since_us_ > 8'000'000U)) {
        trim_window_on_ = false;
      }
      const bool ungated = trim_window_on_ && config_.standup.balance_trim_ungated_tau_s > 0.0F;
      if (ungated || (std::abs(ga_f_) < config_.trim_acc_max_a_m_s2 && std::abs(in.pitch_rate_rad_s) < 0.3F)) {
        const float target = in.pitch_rad - std::atan(ga_f_ / 9.80665F) - ffPitch();
        const float cur = lqr_.balanceTrim();
        const float tau = ungated ? std::max(config_.standup.balance_trim_ungated_tau_s, 0.2F)
                                  : std::max(config_.trim_acc_tau_s, 0.2F);
        lqr_.setBalanceTrim(cur + std::clamp(dt / tau, 0.0F, 1.0F) * (target - cur));
      }
    } else {
      ga_init_ = false;
    }
    updateLearnedTrim(in, ex_station, lqr_.balanceTrim() + ffPitch());
  }
  if (trim_window_on_) {
    // Stand-up handoff: the adaptive trim may refine the seed but not wander off it.
    const float w = std::max(config_.standup.balance_trim_window_rad, 0.0F);
    lqr_.setBalanceTrim(std::clamp(lqr_.balanceTrim(), trim_window_center_ - w, trim_window_center_ + w));
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
  state.imu.yaw_rate_rad_s = in.yaw_rate_rad_s;  // the yaw-rate feedback of LqrBalancer (it was left at 0: 2x open loop)
  state.wheel.position_m = ground_x_;
  state.wheel.velocity_m_s = pid_v_filt_;

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
  if (recovery_bounds) {
    lqr_.setMaxGroundVelocity(vmax);
    lqr_.setMaxGroundAccel(recovery_amax);
  }
  const auto lqr_out = lqr_.update(state, command, integrate, ffPitch() + lean_ff, true);
  if (recovery_bounds) {
    // Preserve normal driving configuration and runtime SET overrides.
    lqr_.setMaxGroundVelocity(saved_vmax);
    lqr_.setMaxGroundAccel(saved_amax);
  }
  // updateHeight's balance-quality gate reads the pitch reference from pid_ref_.
  pid_ref_ = lqr_.debugState().lqr_pitch_ref_rad;

  // Held/lifted: over a window the command changed but the body never
  // rotated (a free pendulum would have) - same test as pidUpdate, tracking
  // the LQR's integrated ground-speed command instead of pid_cmd_m_s_.
  if (legs_moved_us_ != 0U && in.now_us - legs_moved_us_ < config_.height.held_suppress_us) {
    held_window_start_us_ = 0U;
  }
  const float lqr_cmd = lqr_.debugState().commanded_ground_velocity_m_s;
  // Held/lifted means the wheel command moves while the body does not rotate. A drive and, above all, the braking
  // after it do exactly that when the wheels ripple at 3 Hz: the 21/10 stop from 1.5 m/s was read as "lifted" at 0.7 m/s,
  // the wheel command was zeroed (wcmd=0.00, ho=1, trim re-seeded) and the robot fell. While a speed plan is
  // running, and for 2.5 s after it, the test is off.
  if (std::abs(target_v) > 0.05F || std::abs(plan_v_) > 0.05F) held_suppress_until_us_ = in.now_us + 2'500'000U;
  if (in.now_us < held_suppress_until_us_) held_window_start_us_ = 0U;
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
  std::array<float, 2> sent{
      comp(lqr_out.left_wheel_velocity_rad_s + lead_wheel - carrier[0]),
      comp(lqr_out.right_wheel_velocity_rad_s + lead_wheel - carrier[1])};
  if (recovery_bounds) {
    for (auto& w : sent)
      w = std::clamp(w, -config_.standup.catch_wheel_max_rad_s, config_.standup.catch_wheel_max_rad_s);
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

TAP_HOT void V3Controller::resetReadyStill() noexcept {
  still_since_us_ = ready_last_us_ = 0U;
  still_quiet_us_ = still_bad_us_ = still_bad_run_us_ = ready_tip_us_ = 0U;
  still_sum_ = 0.0F;
  still_n_ = 0;
  still_quiet_ = false;
}

TAP_HOT bool V3Controller::readyStill(const Input& in, float body_pitch, float body_rate) noexcept {
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

TAP_HOT V3Controller::ComLine V3Controller::comLine(float hip_offset, float knee_offset) const noexcept {
  for (const auto& entry : com_cache_) {
    if (entry.valid && entry.hip == hip_offset && entry.knee == knee_offset &&
        entry.dx == config_.tier.body_com_dx_m) return entry.value;
  }
  namespace geo = robot_control::geometry;
  const float hip = LegHeight::hangHipCad() + hip_offset;
  const float knee = knee_offset;
  const auto body = robot_control::WholeBodyModel{}.compute(hip, knee, hip, knee);
  const auto jac = robot_control::WholeBodyModel::symmetricComOffsetJacobian(hip, knee);
  const float cx = body.com_offset_x_m +
      geo::parts[0].mass * config_.tier.body_com_dx_m / geo::total_mass_kg;
  const float cz = std::max(body.com_above_axle_m, 0.01F);
  const float den = cx * cx + cz * cz;
  const ComLine result{std::atan2(cx, cz),
          (cz * jac.d_com_offset_x_d_hip - cx * jac.d_com_above_axle_d_hip) / den,
          (cz * jac.d_com_offset_x_d_knee - cx * jac.d_com_above_axle_d_knee) / den, cz,
          body.pitch_inertia_at_axle_kg_m2};
  com_cache_[com_cache_next_] = {hip_offset, knee_offset, config_.tier.body_com_dx_m, result, true};
  com_cache_next_ = (com_cache_next_ + 1U) % com_cache_.size();
  return result;
}

TAP_HOT float V3Controller::legModelError() const noexcept {
  // The sign guard/trace needs only the angle, not a second full Jacobian.
  namespace geo = robot_control::geometry;
  const float hip = LegHeight::hangHipCad() + hipCommand(), knee = kneeCommand();
  const auto model = robot_control::WholeBodyModel{}.compute(hip, knee, hip, knee);
  const float x = model.com_offset_x_m + geo::parts[0].mass *
      config_.tier.body_com_dx_m / geo::total_mass_kg;
  return c_meas_ - std::atan2(x, std::max(model.com_above_axle_m, 0.01F));
}

TAP_HOT void V3Controller::updateVirtualPitch(const Input& in) {
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
  com_meas_height_m_ = c.height_m;
  theta_v_ = in.pitch_rad + c.angle - com_line_hang_;
  // Analytic rate from the driver velocities: differentiating the angle and
  // filtering it would add lag to the balancer's D channel.
  const Tier& t = config_.tier;
  float rate_gain = t.joint_rate_gain;
  if (t.joint_rate_gain_low >= 0.0F) {
    const float ratio = std::clamp(c.height_m / std::max(com_reference_m_, 0.01F), 0.1F, 1.0F);
    const float low_w = std::clamp((t.ratio_tall - ratio) / std::max(t.ratio_tall - t.ratio_low, 1.0e-3F),
                                   0.0F, 1.0F);
    rate_gain += low_w * (t.joint_rate_gain_low - t.joint_rate_gain);
  }
  theta_v_rate_ = in.pitch_rate_rad_s + rate_gain *
      (c.d_hip * hip_rate_meas_ + c.d_knee * knee_rate_meas_);
}

TAP_HOT void V3Controller::updateImuVelocity(const Input& raw, bool free) {
  imu_velocity_ = ImuVelocity{};
  // Whole-body too: without it the pitch estimate read every acceleration
  // as tilt and kept ~0.026 rad of it seconds into the cruise (gyro-
  // dominant blend) - the WB then held the body that much off level and ran
  // ~15 % over the commanded speed (sim). The joints come from the raw
  // input (the tiers' hip_meas_ is only kept with the virtual pitch).
  const bool wb = wholeBody() && legs_measured_;
  if (!config_.tier.imu_kinematic || !free || !(tierVirtual() || wb)) return;
  // Roll makes the two legs asymmetric: average velocities AFTER FK, not
  // joint angles before FK. Use the same supported-leg kinematics as skyhook.
  robot_control::WholeBodyModel::PlanarVelocity v{};
  for (std::size_t side = 0; side < 2U; ++side) {
    const auto s = robot_control::WholeBodyModel::imuVelocityThroughLeg(
        raw.pitch_rad, raw.pitch_rate_rad_s,
        LegHeight::hangHipCad() + raw.hip_rad[side], raw.knee_rad[side],
        raw.hip_rate_rad_s[side], raw.knee_rate_rad_s[side], raw.wheel_rad_s[side],
        config_.wheel_radius_m);
    v.horizontal_m_s += 0.5F * s.horizontal_m_s;
    v.vertical_m_s += 0.5F * s.vertical_m_s;
  }
  imu_velocity_ = {v.horizontal_m_s, v.vertical_m_s, std::isfinite(v.horizontal_m_s) &&
                                                         std::isfinite(v.vertical_m_s)};
}

TAP_HOT void V3Controller::updateWheelLegCoordination(const Input& in) noexcept {
  // A commanded height change already has a paired hip/knee trajectory
  // and balance feed-forward. Do not classify its own carrier motion as a
  // new external push. Use the existing tier law during that transition,
  // then smoothly hand back to disturbance damping once the plan is quiet.
  const bool moving = height_startup_ || knee_cmd_rad_ != knee_target_rad_ ||
      std::abs(height_speed_m_s_) > 1.0e-5F || std::abs(height_accel_m_s2_) > 1.0e-4F;
  if (moving || held_outer_off_ || held_quiet_) wheel_leg_quiet_since_us_ = 0U;
  else if (wheel_leg_quiet_since_us_ == 0U) wheel_leg_quiet_since_us_ = in.now_us;
  const bool settled = wheel_leg_quiet_since_us_ != 0U &&
      in.now_us - wheel_leg_quiet_since_us_ >= 1'000'000U;
  const float target = settled ? 1.0F : 0.0F;
  const float step = std::max(in.dt_s, 0.0F) / (settled ? 0.5F : 0.1F);
  wheel_leg_alpha_ += std::clamp(target-wheel_leg_alpha_, -step, step);
}

#ifdef TAP_SPLIT_HOST_PROTOTYPE
// Slow posture allocation and transient capture are separate tasks, following
// the task separation in wheel-legged whole-body control (IROS 2021,
// doi:10.1109/iros51168.2021.9636371). This bounded position-driver prototype
// is NOT full-body MPC; rotor-side FK remains an uncertain estimate.
float V3Controller::splitModelPosture() const noexcept {
  const float pitch = config_.tier.level_pitch_rad;
  const float depth = std::max(pose_.axle_below_hip_m * std::cos(pitch), LegHeight::minHeight());
  const float reach = LegHeight::maxHeight();
  const float xmax = std::sqrt(std::max(reach*reach-depth*depth, 0.0F));
  const float desired = com_line_hang_ + balanceRef() - pitch;
  float x = 0.0F;
  auto target = pose_;
  for (int i = 0; i < 5; ++i) {
    const auto at = LegHeight::forAxleOnPlane(x, depth, pitch);
    if (!at.ok) break;
    target.hip_offset_rad = at.hip_offset_rad;
    target.knee_offset_rad = at.knee_offset_rad;
    const float angle = comLine(at.hip_offset_rad, at.knee_offset_rad).angle;
    const auto next = LegHeight::forAxleOnPlane(std::min(x+0.0001F,xmax), depth,pitch);
    if (!next.ok || next.x_world_m-at.x_world_m < 1.0e-6F) break;
    const float jac = (comLine(next.hip_offset_rad,next.knee_offset_rad).angle-angle) /
                      (next.x_world_m-at.x_world_m);
    if (!std::isfinite(jac) || std::abs(jac)<0.1F) break;
    x = std::clamp(x+(desired-angle)/jac,-xmax,xmax);
  }
  return std::clamp(target.hip_offset_rad-pose_.hip_offset_rad,
                    -config_.tier.level_max_rad,config_.tier.level_max_rad);
}

void V3Controller::updatePosture(const Input& in, bool on, bool saturated) {
  const float dt = std::clamp(in.dt_s,0.0F,0.01F);
  const Split& p = config_.split;
  const float tau = std::max(p.posture_tau_s,1.5F);
  body_lp_ += std::clamp(dt*2.0F*3.14159265F*config_.tier.level_lpf_hz,0.0F,1.0F) *
              (body_pitch_-body_lp_);
  split_model_target_ = splitModelPosture();
  const float error = (body_lp_-config_.tier.level_pitch_rad) / std::max(j_hip_,0.2F);
  split_frozen_ = !on || saturated || split_alpha_b_>0.001F;
  // The learned wheel trim remains owned by LQR. A uses its bounded model
  // target, and integrates only body-level error (no second CoM integrator).
  const float increment = dt/tau*error;
  if (!split_frozen_ && (!axle_limited_ || increment*axle_limit_direction_<=0.0F))
    split_integral_ = std::clamp(split_integral_+increment,
                              -config_.tier.level_max_rad,config_.tier.level_max_rad);
  const float want = on ? std::clamp(split_model_target_+split_integral_,
                    -config_.tier.level_max_rad,config_.tier.level_max_rad) : 0.0F;
  const float rate = std::max(p.posture_rate_rad_s,0.0F)*dt;
  // tau governs the slow level adaptation. Do not filter the already
  // planned posture a second time: that delayed zero-offset correction.
  split_a_ += std::clamp(want-split_a_,-rate,rate);
}

void V3Controller::updateShock(const Input&, bool, float, float) {
  // Stage B is deliberately unavailable until the slow-posture gate passes.
  // A failed gate must not be hidden by turning the fast reflex back on.
  split_alpha_b_ = split_hold_s_ = split_b_ = 0.0F;
}

#endif

TAP_HOT void V3Controller::updateLegLayers(const Input& in, bool active,
                                  std::array<float,2> wheel_cmd) {
#ifndef TAP_SPLIT_HOST_PROTOTYPE
  updateTrunk(in,active,wheel_cmd);
#else
  if (!config_.split.enabled) {
    split_was_enabled_=false;
    updateTrunk(in,active,wheel_cmd); return;
  }
  const float dt=std::clamp(in.dt_s,0.0F,0.01F);
  if (!split_was_enabled_) {
    split_a_=dhip_; split_b_=split_alpha_b_=split_hold_s_=split_quiet_s_=0.0F;
    split_integral_=0.0F; split_armed_=true; split_was_enabled_=true;
    body_lp_=body_pitch_;
  }
  // A is the continuously planned posture owner; it does not inherit the
  // old fast-reflex's two-second enable delay. Its own slew and the exact
  // workspace projection bound startup motion. Held/lifted blocks it.
  const bool on=active && config_.tier.trunk && tierVirtual() &&
    !held_outer_off_ && !held_quiet_;
  const float cap=std::max(config_.lqr.max_ground_velocity_m_s,0.05F)/config_.wheel_radius_m;
  const float wr=std::max(std::abs(wheel_cmd[0]),std::abs(wheel_cmd[1]))/cap;
  float plan=0.0F;
  if (config_.tier.preview && plan_n_>0U) plan=std::atan(planAt(config_.tier.preview_s).a/9.80665F);
  updateShock(in,on,wr,plan);
  updatePosture(in,on,wr>=config_.tier.wheel_sat_margin);
  split_preview_=on ? config_.tier.lean_share*plan/std::max(j_hip_,0.2F) : 0.0F;
  const float wanted=split_a_+split_b_+split_preview_;
  const float step=config_.tier.rate_rad_s*dt;
  dhip_+=std::clamp(wanted-dhip_,-step,step);
  trunk_alpha_=on ? 1.0F : 0.0F;
  level_i_=split_a_; level_beta_=on ? 1.0F : 0.0F;
  if (!active) { split_integral_=0.0F; split_b_=split_alpha_b_=0.0F; }
#endif
}

TAP_HOT void V3Controller::updateTrunk(const Input& in, bool active,
                               std::array<float, 2> wheel_cmd_rad_s) {
  const Tier& t = config_.tier;
  // Settle after engage and again after every held/lifted spell: the
  // release catch is tier 1's alone.
  if (!active || held_outer_off_ || held_quiet_) trunk_block_us_ = in.now_us;
  const std::uint64_t since = std::max(state_since_us_, trunk_block_us_);
  const bool settled = in.now_us - since >=
      static_cast<std::uint64_t>(std::max(t.settle_s, 0.0F) * 1.0e6F);
  // The fast path sees whole-system lean, not body counter-rotation against the
  // joints. The slow level loop below sees the body itself, so static trim and
  // sensor/model offsets do not set its posture.
  const float lean = in.pitch_rad - (balanceRef() + ffPitch());
  float lean_plan_now = 0.0F, lean_plan_ahead = 0.0F;
  if (t.preview && tierVirtual() && plan_n_ > 0U) {
    const float g = 9.80665F;
    lean_plan_now = std::atan(planAt(t.preview_s).a / g);
    lean_plan_ahead = std::atan(planAt(std::max(t.preview_s - t.hip_advance_s, 0.0F)).a / g);
  }
  const float residual = lean - lean_plan_now;
  if (trunk_alpha_ <= 0.0F) {
    lean_lp_ = residual;
    lean_rate_f_ = 0.0F;
  } else if (t.hpf_s > 0.0F) {
    lean_lp_ += std::min(in.dt_s / t.hpf_s, 1.0F) * (residual - lean_lp_);
  } else {
    lean_lp_ = 0.0F;
  }
  const float rate_alpha = t.lead_lpf_hz > 0.0F
      ? std::clamp(2.0F * 3.14159265F * t.lead_lpf_hz * in.dt_s, 0.0F, 1.0F) : 1.0F;
  const float lean_rate_in = t.lead_gyro_only ? body_rate_ : in.pitch_rate_rad_s;
  lean_rate_f_ += rate_alpha * (lean_rate_in - lean_rate_f_);
  // Gain schedule from the CURRENT encoder/FK CoM height, not the requested
  // height. Smoothed over 50 ms so knee compliance/encoder noise does not modulate
  // the gains on every CAN sample. sqrt(l/g) is the pendulum's natural time scale
  // (Pratt et al., Capture Point, Humanoids 2006).
  const float observed_h = tierVirtual() ? com_meas_height_m_ : pose_.com_above_axle_m;
  const float measured_ratio = std::clamp(observed_h / std::max(com_reference_m_, 0.01F), 0.1F, 1.0F);
  if (!adaptive_init_ || !active) {
    adaptive_ratio_ = measured_ratio;
    adaptive_init_ = true;
  } else {
    adaptive_ratio_ += std::clamp(in.dt_s / 0.05F, 0.0F, 1.0F) * (measured_ratio-adaptive_ratio_);
  }
  const float ratio = adaptive_ratio_;
  const float span = std::max(t.ratio_tall - t.ratio_low, 1.0e-3F);
  const float low_w = std::clamp((t.ratio_tall - ratio) / span, 0.0F, 1.0F);
  const float exponent = t.height_exp_tall + low_w * (t.height_exp - t.height_exp_tall);
  const float share = t.lean_share * std::pow(ratio, std::max(exponent, 0.0F));
  const float jh = std::max(j_hip_, 0.2F);
  const float lead = t.lead_s * std::pow(ratio, std::max(t.lead_exp, 0.0F));
  const float plan_share = t.plan_full ? t.lean_share : share;
  const float disturbance_share = share * std::clamp(t.disturbance_share, 0.0F, 1.0F);
  const float fast_residual = residual - lean_lp_;
  const float disturbance_motion = std::max(std::abs(fast_residual)/0.02F,
                                            std::abs(lean_rate_f_)/0.20F);
  const float damping_weight = std::clamp((disturbance_motion - 0.1F)/0.9F, 0.0F, 1.0F);
  const float response = wheel_leg_alpha_ * damping_weight;
  const float rate_lead = lead * lean_rate_f_;
  const float plan_want = plan_share * lean_plan_ahead / jh;
  // Viscous damping idea: Hogan, Impedance Control Part I (1985),
  // doi:10.1115/1.3140702. This bounded position-driver proxy is not a torque
  // impedance controller or a passivity proof for delayed actuators. It is
  // allowed only with a resolved whole-system disturbance (response weight):
  // body/joint counter-rotation with a stationary axle->COM line keeps it at zero.
  // body_rate_ itself stays raw: held/armed detection and the reflex gate read it.
  if (trunk_alpha_ <= 0.0F || t.body_damp_lpf_hz <= 0.0F) {
    body_rate_damp_f_ = body_rate_;
  } else {
    body_rate_damp_f_ += std::clamp(2.0F * 3.14159265F * t.body_damp_lpf_hz * in.dt_s, 0.0F, 1.0F) *
        (body_rate_ - body_rate_damp_f_);
  }
  const float body_damping = std::max(t.body_damping_s, 0.0F) * std::sqrt(ratio) *
      response * body_rate_damp_f_ / jh;
  // Tier::hip_deadband_rad: small leans stay with the wheels.
  const float fast_signal = fast_residual + rate_lead;
  const float hip_db = std::max(t.hip_deadband_rad, 0.0F);
  const float fast_used = std::copysign(std::max(std::abs(fast_signal) - hip_db, 0.0F), fast_signal);
  float reflex_gate = 1.0F;
  if (t.reflex_gate) {
    if (trunk_alpha_ <= 0.0F) { reflex_w_ = 0.0F; gate_rate_lp_ = std::abs(body_rate_); }
    const float a = std::clamp(2.0F * 3.14159265F * 5.0F * in.dt_s, 0.0F, 1.0F);
    gate_rate_lp_ += a * (std::abs(body_rate_) - gate_rate_lp_);
    const float rate_m = (gate_rate_lp_ - t.reflex_rate_lo) /
        std::max(t.reflex_rate_hi - t.reflex_rate_lo, 1.0e-3F);
    const float lean_m = (std::abs(fast_residual) - t.reflex_lean_lo) /
        std::max(t.reflex_lean_hi - t.reflex_lean_lo, 1.0e-3F);
    const float want_w = std::clamp(std::max(rate_m, lean_m), 0.0F, 1.0F);
    const float up_g = in.dt_s / std::max(t.reflex_attack_s, 1.0e-3F);
    const float down_g = in.dt_s / std::max(t.reflex_release_s, 1.0e-3F);
    reflex_w_ += std::clamp(want_w - reflex_w_, -down_g, up_g);
    reflex_gate = reflex_w_;
  } else {
    reflex_w_ = 1.0F;
  }
  float speed_gain = 1.0F;
  if (t.speed_ref_m_s > 0.0F) {
    const float v = std::max(std::abs(pid_v_filt_), tierVirtual() && plan_n_ > 0U ? std::abs(plan_v_) : 0.0F);
    const float q = v / t.speed_ref_m_s;
    speed_gain = std::max(1.0F / (1.0F + q * q), std::clamp(t.speed_floor, 0.0F, 1.0F));
  }
  trunk_speed_gain_ = speed_gain;
  const float disturbance_want = reflex_gate * speed_gain * (disturbance_share * fast_used / jh +
      std::clamp(body_damping, -0.04F, 0.04F));
  // Full-authority desired hip offset (alpha = 1, unclamped): decides whether the
  // legs are running out of range, independent of how far alpha has faded
  // (see Tier::sat_margin).
  const float full_want = plan_want + disturbance_want;
  // Horizontal workspace at the current nominal gravity-frame depth. Dynamic
  // authority is limited here before the exact Cartesian projection in
  // coordinateTrunkAxle; the static level loop has its own limit and anti-windup.
  const float depth = std::max(pose_.axle_below_hip_m * std::cos(body_pitch_), LegHeight::minHeight());
  const float reach = LegHeight::maxHeight();
  const float xmax = std::sqrt(std::max((reach-depth)*(reach+depth), 0.0F));
  const float xbase = std::abs(pose_.axle_below_hip_m * std::sin(body_pitch_));
  const float travel = std::atan2(std::max(xmax-xbase, 0.0F), std::max(depth, 0.01F));
  const float max_rad = std::min(std::max(t.max_rad, 0.0F), travel);
  const float sched = t.adaptive_schedule ? std::sqrt(ratio) : 1.0F;
  const float adaptive_rate = std::max(t.rate_rad_s, 0.0F) * sched;
  const float level_tau = std::max(t.level_tau_s * sched, 0.5F);
  trunk_schedule_ = {ratio, lead, adaptive_rate, max_rad, jh, level_tau};
  const float margin = std::clamp(t.sat_margin, 0.0F, 0.99F);
  const float mag_ratio = std::abs(full_want) / std::max(max_rad, 1.0e-3F);
  // The fade is gated on the wheel command too (see Tier::sat_margin).
  const float wheel_cap = std::max(config_.lqr.max_ground_velocity_m_s, 0.05F) /
      std::max(config_.wheel_radius_m, 1.0e-4F);
  const float wheel_ratio = std::max(std::abs(wheel_cmd_rad_s[0] + wheel_carrier_rad_s_[0]),
                                    std::abs(wheel_cmd_rad_s[1] + wheel_carrier_rad_s_[1])) /
      wheel_cap;
  const bool wheel_saturating = wheel_ratio >= std::clamp(t.wheel_sat_margin, 0.0F, 1.0F);
  gimbal_frozen_ = wheel_saturating;
  const float range_ceiling = (!wheel_saturating || mag_ratio <= margin)
      ? 1.0F : std::clamp((1.0F - mag_ratio) / (1.0F - margin), 0.0F, 1.0F);
  // Not gated on !height_startup_: max_rad above already tracks the real
  // Cartesian workspace at the current height (near 0 at full extension, opening
  // as the reserve crouch proceeds); a second all-or-nothing gate only cost
  // settle time (v3_tier_tests V15/V17/V25).
  const bool on = active && settled && t.trunk && tierVirtual();
  const float target = on ? range_ceiling : 0.0F;
  const float up = t.ramp_s > 1.0e-3F ? in.dt_s / t.ramp_s : 1.0F;
  const float down = t.sat_fade_s > 1.0e-3F ? in.dt_s / t.sat_fade_s : 1.0F;
  const float diff = target - trunk_alpha_;
  trunk_alpha_ = std::clamp(trunk_alpha_ + std::clamp(diff, -down, up), 0.0F, 1.0F);
  const float dyn = std::clamp(trunk_alpha_ * full_want, -max_rad, max_rad);

  // Slow IMU body-level feedback (the "gimbal"), independent of the dynamic
  // virtual lean.
  const float lp = std::clamp(2.0F * 3.14159265F * t.level_lpf_hz * in.dt_s, 0.0F, 1.0F);
  body_lp_ += lp * (body_pitch_ - body_lp_);
  const bool level_on = on && t.level;
  const float lmax = std::max(t.level_max_rad, 0.0F);
  if (level_on && !level_init_) {
    // Seed from the measured error, not zero: level_beta_ below already ramps
    // 0 -> 1 over the same schedule as trunk_alpha_, so the output is bumpless
    // regardless of the seed; a zero seed costs a ~3*tau_s crawl (v3_tier_tests
    // V17/V25 settle-time bounds).
    level_i_ = std::clamp((body_lp_ - t.level_pitch_rad) / jh, -lmax, lmax);
    level_init_ = true;
  } else if (level_on && level_beta_ >= 1.0F && trunk_alpha_ >= 1.0F && !wheel_saturating) {
    const float increment = in.dt_s / level_tau *
        (body_lp_ - t.level_pitch_rad) / jh;
    // Do not wind farther into an unreachable axle target. Still allow
    // unwinding when the body error reverses, even at the workspace edge.
    if (!axle_limited_ || increment * axle_limit_direction_ <= 0.0F)
      level_i_ = std::clamp(level_i_ + increment, -lmax, lmax);
  }
  const float level_diff = (level_on ? 1.0F : 0.0F) - level_beta_;
  level_beta_ = std::clamp(level_beta_ + std::clamp(level_diff, -down, up), 0.0F, 1.0F);

  const float want = dyn + level_beta_ * level_i_;
  float step = adaptive_rate * in.dt_s;
  if (hip_release_ramp_) {
    step = std::min(step, std::max(config_.standup.release_hip_rate_rad_s, 1.0e-3F) * in.dt_s);
    if (std::abs(want - dhip_) < 0.01F) hip_release_ramp_ = false;
  }
  dhip_ += std::clamp(want - dhip_, -step, step);
}

TAP_HOT void V3Controller::predictWheels(const Input& raw, float* rate, float* pos) noexcept {
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

TAP_HOT void V3Controller::updateRoll(const Input& in, bool active) {
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
  // Filtered roll rate (bias removed): the reflex damping and the learning gate.
  roll_w_f_ += std::clamp(2.0F * 3.14159265F * 10.0F * in.dt_s, 0.0F, 1.0F) *
      ((in.roll_rate_rad_s - roll_bias_) - roll_w_f_);
  if (state_ != State::Balancing || !r.enabled) roll_fail_ = false;
  // Level reference: learned in place after free balancing has been calm for a
  // while, with the side offsets at zero (Roll::learn_ref). The same flat-floor
  // assumption as a manual calibration, but it follows each power-up.
  if (!active) {
    roll_ref_valid_ = false;
    roll_learn_n_ = 0;
    roll_learn_sum_ = 0.0F;
    roll_learn_t0_us_ = 0U;
    roll_active_since_us_ = 0U;
  } else if (r.learn_ref && !roll_ref_valid_) {
    if (roll_active_since_us_ == 0U) roll_active_since_us_ = in.now_us;
    const float since_s = static_cast<float>(in.now_us - roll_active_since_us_) * 1.0e-6F;
    if (since_s >= r.learn_delay_s) {
      const bool calm = std::abs(roll_w_f_) < r.learn_rate_rad_s &&
          std::abs(ground_v_) < r.learn_speed_m_s && std::abs(yaw_cmd_) < 1.0e-3F &&
          std::abs(roll_dh_) < 5.0e-4F;
      if (!calm) {
        roll_learn_n_ = 0;
        roll_learn_sum_ = 0.0F;
        roll_learn_t0_us_ = 0U;
      } else {
        if (roll_learn_t0_us_ == 0U) roll_learn_t0_us_ = in.now_us;
        roll_learn_sum_ += roll_est_;
        ++roll_learn_n_;
        if (static_cast<float>(in.now_us - roll_learn_t0_us_) * 1.0e-6F >= r.learn_window_s) {
          roll_ref_ = roll_learn_sum_ / static_cast<float>(roll_learn_n_);
          roll_ref_valid_ = true;
        }
      }
    }
  }
  const bool use_ref = r.learn_ref && roll_ref_valid_;
  const float ref = use_ref ? roll_ref_ : r.target_rad;
  const bool gimbal_open = !r.gimbal_link || (config_.tier.level && level_beta_ >= 0.99F);
  const bool level_ready = r.enabled && active && !roll_fail_ && gimbal_open &&
      (!r.learn_ref || roll_ref_valid_);
  roll_alpha_ += std::clamp((level_ready ? 1.0F : 0.0F) - roll_alpha_,
                            -in.dt_s / 0.5F, in.dt_s / std::max(r.ramp_s, 0.05F));
  // Lean-in target: the resultant of gravity and the centripetal pull
  // between the wheels (left turn, a_c > 0: left side down, roll < 0). With
  // cmd_preview it follows the commanded (slewed) yaw, which leads the wheels.
  const float a_lean = r.cmd_preview ? roll_v_lp_ * yaw_cmd_ : a_c;
  roll_target_ = ref - r.turn_ff * std::atan(std::clamp(a_lean / 9.80665F, -0.3F, 0.3F));
  roll_tgt_rate_f_ += std::clamp(2.0F * 3.14159265F * 10.0F * in.dt_s, 0.0F, 1.0F) *
      ((roll_target_ - roll_tgt_prev_) / std::max(in.dt_s, 1.0e-4F) - roll_tgt_rate_f_);
  roll_tgt_prev_ = roll_target_;
  const float half = 0.5F * std::max(r.track_m, 0.05F);
  // Feasible roll travel opens at the slow common-height rate. Keep the
  // normal roll slew so it can use that travel immediately on a cross slope.
  const float step = std::max(r.rate_m_s, 0.0F) * in.dt_s;
  if (!level_ready) {
    roll_sat_s_ = 0.0F;
    roll_dh_ -= std::clamp(roll_dh_, -step, step);  // back to even legs
    roll_i_ = roll_dh_;
    roll_e_lp_ = roll_est_ - roll_target_;
  } else {
    // Feed-forward: the side offset that sets the target roll on level
    // ground; the integral takes the ground and the legs' own error; the
    // proportional part reacts to a sudden tilt at once (gyro-fast).
    const float ff = half * std::tan(roll_target_ - ref);
    roll_e_lp_ += std::clamp(2.0F * 3.14159265F * std::max(r.err_lpf_hz, 0.1F) * in.dt_s, 0.0F, 1.0F) *
        ((roll_est_ - roll_target_) - roll_e_lp_);
    const float e = roll_e_lp_;  // + = left side too high
    // Shared hip/knee commands already include trunk, knee hold and
    // suspension. Reserve roll range around THAT pose, not the height plan.
    const auto common = robot_control::WholeBodyModel::legHeight(
        LegHeight::hangHipCad() + hipCommand(), kneeCommand());
    const float h = -common.height_m, x2 = common.x_m * common.x_m;
    const float hi = std::sqrt(std::max(LegHeight::maxHeight() * LegHeight::maxHeight() - x2, 0.0F));
    const float lo = std::sqrt(std::max(LegHeight::minHeight() * LegHeight::minHeight() - x2, 0.0F));
    const float room = std::max(std::min({r.max_dh_m, hi - h, h - lo}), 0.0F);
    const float tau = r.vel_ff ? r.tau_vff : r.tau_s;
    const float kp = r.vel_ff ? r.kp_vff : r.kp;
    if (!(r.gimbal_link && gimbal_frozen_)) {
      roll_i_ -= in.dt_s / std::max(tau, 0.02F) * half * std::sin(e);
    }
    roll_i_ = std::clamp(roll_i_, -room - ff, room - ff);
    const float p = -kp * half * std::sin(e);
    // Reflex: damping on the error rate (body rate minus the target's own rate).
    const float w_e = roll_w_f_ - roll_tgt_rate_f_;
    const float w_d = std::copysign(std::max(std::abs(w_e) - r.damp_db_rad_s, 0.0F), w_e);
    const float d = std::clamp(-r.damp_s * half * w_d, -r.damp_max_m, r.damp_max_m);
    const float want = std::clamp(roll_alpha_ * (roll_i_ + ff + p + d), -room, room);
    roll_dh_ += std::clamp(want - roll_dh_, -step, step);
    roll_dh_ = std::clamp(roll_dh_, -room, room);
    // Latch: pinned at the travel limit with the error still large = the loop
    // cannot level the body; fade out instead of fighting (Roll::latch).
    if (r.latch && room > 0.002F && std::abs(roll_dh_) >= 0.9F * room &&
        std::abs(roll_e_lp_) > r.fail_err_rad) {
      roll_sat_s_ += in.dt_s;
      if (roll_sat_s_ >= r.fail_s) roll_fail_ = true;
    } else {
      roll_sat_s_ = std::max(roll_sat_s_ - in.dt_s, 0.0F);
    }
  }
  // Side height offset -> joint offsets through the leg IK at the pose
  // (knee for the height, hip to keep the axle under the hip).
  // Exact per-side IK: retain the shared axle fore/aft coordinate while
  // changing its vertical distance. No cached linear Jacobian can become
  // stale when suspension/trunk changes the common pose at a fixed knee plan.
  const float hc = hipCommand(), kc = kneeCommand();
  const auto common = robot_control::WholeBodyModel::legHeight(LegHeight::hangHipCad() + hc, kc);
  std::array<float, 2> knee{}, hip{};
  for (std::size_t side = 0; side < 2U; ++side) {
    const float dh = (side == 0U ? 1.0F : -1.0F) * roll_dh_ * (r.sign < 0.0F ? -1.0F : 1.0F);
    if (std::abs(dh) < 1.0e-7F) continue;
    const float h = std::max(-common.height_m + dh, 0.0F);
    const auto p = LegHeight::forAxleOnPlane(common.x_m, h, 0.0F);
    if (!p.ok) continue;
    hip[side] = p.hip_offset_rad - hc;
    knee[side] = p.knee_offset_rad - kc;
  }
  for (std::size_t i = 0; i < 2U; ++i) {
    roll_knee_rate_[i] = r.vel_ff && in.dt_s > 0.0F ? (knee[i] - roll_knee_[i]) / in.dt_s : 0.0F;
    roll_hip_rate_[i] = r.vel_ff && in.dt_s > 0.0F ? (hip[i] - roll_hip_[i]) / in.dt_s : 0.0F;
  }
  roll_knee_ = knee;
  roll_hip_ = hip;
}

TAP_HOT void V3Controller::coordinateTrunkAxle(float dt_s) noexcept {
  axle_hip_correction_ = 0.0F;
  axle_limited_ = false;
  axle_limit_direction_ = 0.0F;
  if (!kneeHoldOn()) { dknee_ = 0.0F; return; }
  const Tier& t = config_.tier;
  const bool follow = t.knee_follow_hz > 0.0F;
  // Below 2 micrometres of requested axle motion, avoid subtracting nearly
  // equal squared radii at full extension. Keep the nominal zero solution
  // (a following knee first walks back to it).
  if (std::abs(dhip_) < 1.0e-5F && !(follow && std::abs(dknee_) > 1.0e-5F)) {
    dknee_ = 0.0F;
    return;
  }
  // Keep the nominal pose's vertical projection at the CURRENT body pitch.
  // Thus dhip=0 introduces no new posture correction on the lifting stand
  // or during the max-height wheel catch. Cancel the arc caused by trunk
  // swing, including its pitch coupling, without making a second fast body
  // feedback loop. Height/suspension still own the nominal vertical target.
  const float h = pose_.axle_below_hip_m;
  // Preserve the existing tier's fore/aft calibration where its fixed-hip
  // solution is feasible. That provisional FK specifies only X; the paired
  // Cartesian solve below owns both final joints and the height constraint.
  const float provisional_knee = pose_.knee_offset_rad + kneeHold(dhip_);
  const auto provisional = robot_control::WholeBodyModel::legHeight(
      LegHeight::hangHipCad() + pose_.hip_offset_rad + dhip_, provisional_knee);
  const float requested_x = std::cos(body_pitch_)*provisional.x_m +
      std::sin(body_pitch_)*provisional.height_m;
  // At the deepest fold, h*cos(pitch) can fall below the supported minimum
  // radius. Keep the plane at least that deep so X can pass continuously
  // through zero instead of jumping between disconnected annulus branches.
  const float depth = std::max(h * std::cos(body_pitch_), LegHeight::minHeight());
  const auto target = LegHeight::forAxleOnPlane(requested_x, depth, body_pitch_);
  if (!target.ok) {
    axle_limited_ = true;
    if (!follow) { dknee_ = 0.0F; return; }
  }
  const float want_dknee = target.ok ? target.knee_offset_rad - pose_.knee_offset_rad : 0.0F;
  if (follow) {
    // dknee follows dhip^2: low-pass + rate limit it. The hip below is solved for
    // the knee actually sent, so the axle's fore/aft stays exact and only its
    // depth lags by the compensation's few millimetres.
    const float a = std::clamp(2.0F * 3.14159265F * t.knee_follow_hz * dt_s, 0.0F, 1.0F);
    const float step = std::max(t.knee_follow_rate_rad_s, 0.0F) * dt_s;
    dknee_ += std::clamp(a * (want_dknee - dknee_), -step, step);
    if (!target.ok) return;
    axle_hip_correction_ = LegHeight::hipForKneeAtX(
        pose_.knee_offset_rad + dknee_, target.x_world_m, body_pitch_) -
        pose_.hip_offset_rad - dhip_;
  } else {
    axle_hip_correction_ = target.hip_offset_rad - pose_.hip_offset_rad - dhip_;
    dknee_ = want_dknee;
  }
  axle_limited_ = target.limited;
  axle_limit_direction_ = target.x_world_m - requested_x;
}

TAP_HOT float V3Controller::kneeHold(float dhip) noexcept {
  if (!kneeHoldOn() || dhip == 0.0F) {
    knee_hold_ = 0.0F;
    return 0.0F;
  }
  const float hip = LegHeight::hangHipCad() + pose_.hip_offset_rad + dhip;
  const float target = -pose_.axle_below_hip_m;  // legHeight's sign: z up
  float k = std::clamp(pose_.knee_offset_rad + knee_hold_, LegHeight::kKneeMinRad,
                       LegHeight::kKneeMaxRad);
  for (int i = 0; i < 3; ++i) {
    const auto leg = robot_control::WholeBodyModel::legHeight(hip, k);
    if (std::abs(leg.d_height_d_knee) < 1.0e-4F) break;
    const float step = (leg.height_m - target) / leg.d_height_d_knee;
    k = std::clamp(k - step, LegHeight::kKneeMinRad, LegHeight::kKneeMaxRad);
    if (std::abs(step) < 1.0e-5F) break;
  }
  knee_hold_ = k - pose_.knee_offset_rad;
  return knee_hold_;
}

TAP_HOT float V3Controller::planCap(float vmax) const noexcept {
  const Tier& t = config_.tier;
  if (!envelopeOn()) return std::clamp(t.plan_vmax_m_s, 0.05F, vmax);
  const float l = std::max(pose_.com_above_axle_m, 0.01F);
  return std::clamp(vmax - t.env_catch_rad * std::sqrt(9.80665F * l), 0.05F, vmax);
}

TAP_HOT void V3Controller::updatePlan(float target_v, float dt, bool reset) {
  const Tier& t = config_.tier;
  if (reset || !plan_init_) {
    plan_v_ = reset ? 0.0F : target_v;
    plan_a_ = plan_j_ = 0.0F;
    plan_scale_ = 1.0F;
    plan_n_ = 0U;
    plan_init_ = true;
  }
  // Higher speeds get a proportionally larger acceleration/jerk budget (Tier::plan_scale_*); at or below
  // plan_scale_from_m_s nothing changes.
  // The scale is latched while the plan moves, so a stop brakes with the budget that built the speed, and drops
  // back to 1 once the plan is at rest.
  const float scale_from = std::max(t.plan_scale_from_m_s, 0.05F);
  const float demand = std::clamp(std::max(std::abs(target_v), std::abs(plan_v_)) / scale_from, 1.0F,
                                  std::max(t.plan_scale_max, 1.0F));
  plan_scale_ = std::abs(plan_v_) < 0.02F && std::abs(target_v) < 0.02F ? 1.0F : std::max(plan_scale_, demand);
  float amax = 0.0F, jmax = 0.0F;
  planBudget(std::max(std::abs(plan_v_), std::abs(target_v)), plan_scale_, amax, jmax);
  // Jerk-limited S-curve towards the operator speed (see sCurveAccel): the acceleration ramps up at jmax, holds
  // amax, and ramps down at the same jmax so it reaches zero exactly on the target. Up and down are symmetric,
  // there is no exponential tail, and the hip (which follows the plan acceleration) moves once to build the lean,
  // holds, and comes back once. plan_smooth_tau_s is the width of the linear blend closing the last bit of error
  // (it stops the square root chattering at the control tick); tau <= 0 gives the old bang-bang law.
  const float a_des = sCurveAccel(target_v, plan_v_, plan_a_, amax, jmax, t.plan_smooth_tau_s);
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

TAP_HOT void V3Controller::planBudget(float v_abs, float scale, float& amax, float& jmax) const noexcept {
  const Tier& t = config_.tier;
  amax = envelopeOn() && wholeBody()
      ? std::max(9.80665F * std::tan(std::clamp(t.env_lean_rad, 0.0F, 0.3F)), 0.05F)
      : std::max(t.plan_accel_m_s2, 0.05F);
  jmax = std::max(t.plan_jerk_m_s3, 0.1F);
  amax *= scale;
  jmax *= scale;
  if (t.plan_taper_to_m_s > t.plan_taper_from_m_s) {
    const float u = std::clamp((v_abs - t.plan_taper_from_m_s) / (t.plan_taper_to_m_s - t.plan_taper_from_m_s), 0.0F, 1.0F);
    const float taper = 1.0F - (1.0F - std::clamp(t.plan_taper_floor, 0.05F, 1.0F)) * u;
    amax *= taper;
    jmax *= taper;
  }
  if (envelopeOn()) {
    jmax *= std::sqrt(std::max(com_reference_m_, 0.01F) / std::max(pose_.com_above_axle_m, 0.01F));
  }
}

float V3Controller::planStopDistance(float lag_s) const noexcept {
  const float dir = plan_v_ >= 0.0F ? 1.0F : -1.0F;
  float v = std::abs(plan_v_), a = plan_a_ * dir, dist = 0.0F;
  const float v0 = v;
  constexpr float kDt = 0.02F;
  for (int i = 0; i < 600 && v > 1.0e-3F; ++i) {
    float amax = 0.0F, jmax = 0.0F;
    planBudget(v, plan_scale_, amax, jmax);
    const float a_des = sCurveAccel(0.0F, v, a, amax, jmax, config_.tier.plan_smooth_tau_s);
    a += std::clamp(a_des - a, -jmax * kDt, jmax * kDt);
    v += a * kDt;
    if (v <= 0.0F) break;
    dist += v * kDt;
  }
  return dir * (dist + v0 * std::max(lag_s, 0.0F));
}

float V3Controller::yawStopAngle(float lag_s) const noexcept {
  float ya = config_.tier.yaw_accel_rad_s2;
  if (config_.roll.enabled && config_.roll.yaw_accel_rad_s2 > 0.0F)
    ya = ya > 0.0F ? std::min(ya, config_.roll.yaw_accel_rad_s2) : config_.roll.yaw_accel_rad_s2;
  const float dir = yaw_cmd_ >= 0.0F ? 1.0F : -1.0F;
  float v = std::abs(yaw_cmd_), dist = 0.0F;
  const float v0 = v;
  if (ya > 0.0F) {
    const float jy = std::max(config_.tier.yaw_jerk_rad_s3, ya);
    float a = yaw_acc_ * dir;
    constexpr float kDt = 0.01F;
    for (int i = 0; i < 600 && v > 1.0e-3F; ++i) {
      const float a_des = sCurveAccel(0.0F, v, a, ya, jy, 0.05F);
      a += std::clamp(a_des - a, -jy * kDt, jy * kDt);
      v += a * kDt;
      if (v <= 0.0F) break;
      dist += v * kDt;
    }
  }
  return dir * (dist + v0 * std::max(lag_s, 0.0F));
}

V3Controller::PlanSample V3Controller::planDelayed() const noexcept {
  return planAt(config_.tier.preview_s);
}

TAP_HOT V3Controller::PlanSample V3Controller::planAt(float delay_s) const noexcept {
  const float dt = std::max(plan_dt_, 1.0e-4F);
  std::size_t lag = static_cast<std::size_t>(std::lround(std::max(delay_s, 0.0F) / dt));
  lag = std::min({lag, plan_buf_.size() - 1U, plan_n_ > 0U ? plan_n_ - 1U : 0U});
  const std::size_t i = (plan_head_ + plan_buf_.size() - 1U - lag) % plan_buf_.size();
  return plan_n_ > 0U ? plan_buf_[i] : PlanSample{plan_v_, plan_a_};
}

float V3Controller::heightStoppingDistance(float speed_m_s, float accel_m_s2,
                                           float max_accel_m_s2,
                                           float max_jerk_m_s3) noexcept {
  const float speed = std::max(speed_m_s, 0.0F);
  const float accel = std::max(max_accel_m_s2, 1.0e-3F);
  const float jerk = std::max(max_jerk_m_s3, 1.0e-3F);
  const float a0 = std::clamp(accel_m_s2, -accel, accel);
  if (speed <= 1.0e-7F && a0 <= 0.0F) return 0.0F;

  // Apply maximum reverse jerk until -accel, then maximum deceleration. This
  // includes the extra distance caused by the acceleration ramp; v^2/(2a)
  // alone stops too early once the profile is jerk limited.
  const float ramp_s = std::max((a0 + accel) / jerk, 0.0F);
  const float speed_after_ramp = speed + a0 * ramp_s - 0.5F * jerk * ramp_s * ramp_s;
  const float distance_after_ramp = speed * ramp_s + 0.5F * a0 * ramp_s * ramp_s -
      (jerk * ramp_s * ramp_s * ramp_s) / 6.0F;
  if (speed_after_ramp <= 0.0F) {
    // The speed reaches zero before the acceleration ramp reaches its limit.
    const float stop_s = (a0 + std::sqrt(std::max(a0 * a0 + 2.0F * jerk * speed, 0.0F))) /
        jerk;
    const float distance = speed * stop_s + 0.5F * a0 * stop_s * stop_s -
        (jerk * stop_s * stop_s * stop_s) / 6.0F;
    return std::max(distance, 0.0F);
  }
  return std::max(distance_after_ramp + speed_after_ramp * speed_after_ramp / (2.0F * accel),
                  0.0F);
}

float V3Controller::heightStopTarget() const noexcept {
  const float speed = height_speed_m_s_;
  const float direction = std::copysign(1.0F, speed);
  const Height& h = config_.height;
  const float max_accel = height_startup_
      ? std::min(h.accel_m_s2, h.startup_accel_m_s2) : h.accel_m_s2;
  const float max_jerk = height_startup_
      ? std::min(h.jerk_m_s3, h.startup_jerk_m_s3) : h.jerk_m_s3;
  const float distance = heightStoppingDistance(std::abs(speed),
      height_accel_m_s2_ * direction, max_accel, max_jerk);
  const float current = LegHeight::forKnee(knee_cmd_rad_).axle_below_hip_m;
  const float hmax = supportedMaxHeight();
  return std::clamp(current + direction * distance, LegHeight::minHeight(), hmax);
}

TAP_HOT void V3Controller::updateHeight(const Input& in, const Command& cmd) {
  const Height& h = config_.height;
  // A straight knee has no height-preserving fore/aft workspace. Reserve
  // the existing 12 mm for paired trunk motion even if roll is switched off.
  const float hmax = supportedMaxHeight();
  if (height_startup_ || cmd.height_m > 0.0F || height_target_m_ > hmax + 1.0e-4F) {
    const float want = height_startup_ ? startup_height_m_
        : cmd.height_m > 0.0F ? cmd.height_m : height_target_m_;
    const float target = std::clamp(want, LegHeight::minHeight(), hmax);
    if (std::abs(target - height_target_m_) > 1.0e-4F) {
      height_target_m_ = target;
      knee_target_rad_ = LegHeight::forHeight(target).knee_offset_rad;
    }
  }
  if (knee_cmd_rad_ == knee_target_rad_) {
    height_rise_ = false;
    height_speed_m_s_ = 0.0F;
    height_accel_m_s2_ = 0.0F;
    // Keep the delay even on a retry already at the reserved height.
    if (in.now_us - state_since_us_ >=
        static_cast<std::uint64_t>(std::max(h.startup_delay_s, 0.0F) * 1.0e6F)) {
      height_startup_ = false;
    }
    return;
  }
  if (height_startup_ && in.now_us - state_since_us_ <
      static_cast<std::uint64_t>(std::max(h.startup_delay_s, 0.0F) * 1.0e6F)) {
    height_speed_m_s_ = 0.0F;
    height_accel_m_s2_ = 0.0F;
    return;
  }
  // Speed quality: full while held (stand, hands: the wheels are not
  // carrying it, so leg motion cannot upset it), otherwise falling smoothly
  // to zero as the balance gets busy - never an on/off gate.
  float quality = 1.0F;
  if (held_quiet_) {
    quality = 0.0F;
  } else if (!held_outer_off_) {
    const StandUp::Config& su = config_.standup;
    const float err_lim = height_rise_ ? std::max(su.rise_max_pitch_error_rad, h.max_pitch_error_rad)
                                       : h.max_pitch_error_rad;
    const float rate_lim = height_rise_ ? std::max(su.rise_max_pitch_rate_rad_s, h.max_pitch_rate_rad_s)
                                        : h.max_pitch_rate_rad_s;
    quality = std::clamp(1.0F - std::abs(in.pitch_rad - pid_ref_) / err_lim, 0.0F, 1.0F) *
        std::clamp(1.0F - std::abs(in.pitch_rate_rad_s) / rate_lim, 0.0F, 1.0F);
  }
  // Signed height speed (+ = up): cap speed to the balance quality and to
  // the jerk-aware distance needed to stop at the target. Then shape both
  // acceleration and speed so reversals remain continuous too.
  // pose_ may include last tick's suspension/excitation offset. The nominal
  // height planner must not integrate that offset back into its own target.
  const auto planned = LegHeight::forKnee(knee_cmd_rad_);
  const float remaining = height_target_m_ - planned.axle_below_hip_m;
  if (height_rise_ && remaining > 0.0F) {
    quality = std::max(quality, std::clamp(config_.standup.rise_min_quality, 0.0F, 1.0F));
  }
  const float accel = std::max(height_startup_
      ? std::min(h.accel_m_s2, h.startup_accel_m_s2) : h.accel_m_s2, 1.0e-3F);
  const float jerk = std::max(height_startup_
      ? std::min(h.jerk_m_s3, h.startup_jerk_m_s3) : h.jerk_m_s3, 1.0e-3F);
  const float rate = std::max(height_startup_
      ? std::min(h.rate_m_s, h.startup_rate_m_s) : h.rate_m_s, 0.0F);
  const float direction = std::copysign(1.0F, remaining);
  const float max_speed = rate * quality;
  // Reserve enough speed headroom to ramp a positive acceleration back to
  // zero under the jerk limit. Without this anticipatory reduction, the
  // actual profile velocity can exceed rate by a^2/(2j) while acceleration
  // unwinds, even though the requested velocity is clamped to max_speed.
  const float accel_toward_target = std::max(height_accel_m_s2_ * direction, 0.0F);
  const float unwind_speed = 0.5F * accel_toward_target * accel_toward_target / jerk;
  float speed_limit = std::max(max_speed - unwind_speed, 0.0F);
  if (max_speed > 0.0F && height_speed_m_s_ * direction > 0.0F) {
    const float current_accel_toward_target = height_accel_m_s2_ * direction;
    float lo = 0.0F;
    float hi = std::max(max_speed, height_speed_m_s_ * direction);
    if (heightStoppingDistance(hi, current_accel_toward_target, accel, jerk) >
        std::abs(remaining)) {
      for (int i = 0; i < 18; ++i) {
        const float mid = 0.5F * (lo + hi);
        if (heightStoppingDistance(mid, current_accel_toward_target, accel, jerk) <=
            std::abs(remaining)) {
          lo = mid;
        } else {
          hi = mid;
        }
      }
      speed_limit = std::min(speed_limit, lo);
    }
  }
  const float v_des = direction * speed_limit;
  const float dt = std::max(in.dt_s, 1.0e-4F);
  const float accel_des = std::clamp((v_des - height_speed_m_s_) / dt, -accel, accel);
  height_accel_m_s2_ += std::clamp(accel_des - height_accel_m_s2_, -jerk * dt, jerk * dt);
  height_accel_m_s2_ = std::clamp(height_accel_m_s2_, -accel, accel);
  height_speed_m_s_ += height_accel_m_s2_ * dt;
  // Integrate in the quantity whose speed is limited, then use exact IK.
  // The old 0.2 mm endpoint snap could jump ~0.1 mm in one 2 ms tick,
  // exceeding the startup rate even when its internal speed was near zero.
  const float next_height = planned.axle_below_hip_m + height_speed_m_s_ * in.dt_s;
  const bool passed = remaining < 0 ? next_height <= height_target_m_
                                   : next_height >= height_target_m_;
  // Treat micrometre-scale FK/float residuals as reached so the speed
  // profile cannot stall below its endpoint forever.
  constexpr float kHeightEndpointToleranceM = 1.0e-5F;
  if (passed || std::abs(remaining) < kHeightEndpointToleranceM) {
    knee_cmd_rad_ = knee_target_rad_;
    height_speed_m_s_ = 0.0F;
    height_accel_m_s2_ = 0.0F;
    height_startup_ = false;
  } else {
    knee_cmd_rad_ = LegHeight::forHeight(next_height).knee_offset_rad;
  }
  pose_ = LegHeight::forKnee(knee_cmd_rad_);
  legs_moved_us_ = in.now_us;
}

TAP_HOT void V3Controller::updateSuspension(const Input& in) {
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
  // Startup reserve motion is a commanded posture change, not a bump.
  // Do not integrate its transient and release it as a knee offset later.
  if (height_startup_) height_settled_since_us_ = 0U;
  else if (height_settled_since_us_ == 0U) height_settled_since_us_ = in.now_us;
  const bool settled = height_settled_since_us_ != 0U &&
      in.now_us - height_settled_since_us_ >=
      static_cast<std::uint64_t>(std::max(t.sus_startup_settle_s, 0.0F) * 1.0e6F);
  const float target = on && settled ? 1.0F : 0.0F;
  const float blend_step = in.dt_s / std::max(t.sus_startup_ramp_s, 1.0e-3F);
  sus_alpha_ += std::clamp(target - sus_alpha_, -blend_step, blend_step);
  if (on && settled) {
    // Complementary: integrate the IMU, pulled towards the legs' height rate
    // (the body's vertical speed on a flat floor) with sus_leak_s.
    float vz_legs = 0.0F;
    for (std::size_t side = 0; side < 2U; ++side) {
      const auto v = robot_control::WholeBodyModel::imuVelocityThroughLeg(
          body_pitch_, body_rate_, LegHeight::hangHipCad() + in.hip_rad[side], in.knee_rad[side],
          in.hip_rate_rad_s[side], in.knee_rate_rad_s[side], in.wheel_rad_s[side],
          config_.wheel_radius_m);
      vz_legs += 0.5F * v.vertical_m_s;
    }
    sus_vz_ += in.dt_s * (fzw - sus_g_) +
        in.dt_s / std::max(t.sus_leak_s, 0.05F) * (vz_legs - sus_vz_);
  } else {
    sus_vz_ = 0.0F;
  }
  // Relative to the planned height change: a commanded raise/lower is the
  // body moving on purpose (damping it fought the plan: 90 mm ended 6 mm high).
  const float vz_err = sus_vz_ - height_speed_m_s_;
  float db = std::max(t.sus_deadband_m_s, 0.0F);
  bool sus_active = on;
  if (t.sus_adaptive) {
    if (on && settled) {
      sus_vz2_ += std::clamp(in.dt_s / std::max(t.sus_noise_tau_s, 0.2F), 0.0F, 1.0F) *
          (vz_err * vz_err - sus_vz2_);
    } else {
      sus_vz2_ = 0.0F;
    }
    db = std::max(db, t.sus_noise_k * std::sqrt(sus_vz2_));
    const bool waited = in.now_us - state_since_us_ >=
        static_cast<std::uint64_t>(std::max(t.sus_engage_delay_s, 0.0F) * 1.0e6F);
    const bool calm = std::abs(ground_v_) < t.sus_calm_speed_m_s &&
        std::abs(body_rate_) < t.sus_calm_rate_rad_s;
    sus_active = on && waited && calm;
  }
  sus_db_eff_ = db;
  const float vz_eff = std::copysign(std::max(std::abs(vz_err) - db, 0.0F), vz_err);
  const float rate = sus_active ? -t.sus_gain * sus_alpha_ * vz_eff : 0.0F;
  const float lim = std::max(t.sus_max_m, 0.0F);
  sus_dz_ = std::clamp(sus_dz_ + in.dt_s * (rate - sus_dz_ / std::max(t.sus_return_s, 0.05F)),
                       -lim, lim);
  // Invert the SAME coordinated hip/knee path used by applyPoseOffsets.
  // A partial knee Jacobian at a mixed (offset hip / nominal knee) pose
  // does not describe that path. Clamp the height state too: no windup at
  // a straight knee or at the lower end of the supported model range.
  const auto base = LegHeight::forKnee(knee_cmd_rad_);
  const float target_height = std::clamp(base.axle_below_hip_m + sus_dz_,
                                        LegHeight::minHeight(), LegHeight::maxHeight());
  sus_dz_ = target_height - base.axle_below_hip_m;
  sus_dk_ = std::abs(sus_dz_) < 1.0e-7F ? 0.0F
      : LegHeight::forHeight(target_height).knee_offset_rad - knee_cmd_rad_;
}

TAP_HOT void V3Controller::applyPoseOffsets(const Input& in) {
  updateSuspension(in);
  const float dk = knee_excite_rad_ + sus_dk_;
  if (dk == 0.0F) {
    if (excite_applied_) {
      pose_ = LegHeight::forKnee(knee_cmd_rad_);
      excite_applied_ = false;
    }
    return;
  }
  // Evaluate the nominal pose/COM only when the offset or the height pose
  // under it moved; the analytic IK needs no Newton iterations.
  if (excite_applied_ && std::abs(dk - applied_dk_) < 2.0e-4F && applied_base_ == knee_cmd_rad_) {
    return;
  }
  pose_ = LegHeight::forKnee(std::clamp(knee_cmd_rad_ + dk, LegHeight::kKneeMinRad,
                                       LegHeight::kKneeMaxRad));
  applied_dk_ = dk;
  applied_base_ = knee_cmd_rad_;
  excite_applied_ = true;
  // The joint test is a deliberate leg motion (held test off); the
  // suspension's few mm are not.
  if (knee_excite_rad_ != 0.0F) legs_moved_us_ = in.now_us;
}

// ---- Small vertical hop (HopSequence) ----

// Hip offset that keeps the model CoM line at c_ref for a knee command (Newton from a warm start). The CoM
// height above the axle at that pose goes to *height_m. Used only while a hop runs.
TAP_HOT float V3Controller::hopHipFor(float knee, float hip_warm, float c_ref, float* height_m) const noexcept {
  float hip = hip_warm;
  for (int i = 0; i < 3; ++i) {
    const ComLine c = comLine(hip, knee);
    const float err = c.angle - c_ref;
    if (std::abs(err) < 2.0e-4F) break;
    const float d = std::abs(c.d_hip) > 0.05F ? c.d_hip : std::copysign(0.05F, c.d_hip);
    hip -= err / d;
  }
  hip = std::clamp(hip, hop_hip_start_ - 0.6F, hop_hip_start_ + 0.6F);
  if (height_m != nullptr) *height_m = comLine(hip, knee).height_m;
  return hip;
}

void V3Controller::hopBegin(const Input& in, const Command& cmd) {
  using R = HopSequence::Refusal;
  const HopSequence::Config& h = config_.hop;
  // Conditions that clear by themselves (a wobble, a settling height) are waited for up to arm_wait_s after the
  // request; the others end the request at once.
  auto refuse = [&](R r) {
    const bool soft = r == R::Settling || r == R::Tilted || r == R::Moving;
    hop_refusal_ = r;
    if (!soft || in.now_us >= hop_arm_until_us_) hop_arm_ = false;
    else hop_refusal_ = R::None;
  };
  if (!h.enabled) return refuse(R::Disabled);
  if (hop_.active()) return refuse(R::Busy);
  if (state_ != State::Balancing) return refuse(R::NotBalancing);
  if (!in.sensors_ok || !legs_measured_ || !tierVirtual()) return refuse(R::Sensors);
  if (held_outer_off_ || held_quiet_ || recovery_pose_held_ || hip_release_ramp_ || height_rise_)
    return refuse(R::NotBalancing);
  if (in.now_us - state_since_us_ < static_cast<std::uint64_t>(std::max(h.start_min_balance_s, 0.0F) * 1.0e6F))
    return refuse(R::Settling);
  if (height_startup_ || knee_cmd_rad_ != knee_target_rad_ || std::abs(height_speed_m_s_) > 1.0e-4F)
    return refuse(R::Moving);
  if (std::abs(theta_v_ - pid_ref_) > h.start_max_pitch_err_rad || std::abs(theta_v_rate_) > h.start_max_rate_rad_s ||
      std::abs(body_rate_) > h.start_max_rate_rad_s)
    return refuse(R::Tilted);
  // Right phase of the wobble: slightly ahead of the balance angle, not swinging forward (HopSequence::Config).
  if (theta_v_ - pid_ref_ < h.start_min_tv_err_rad || theta_v_rate_ < h.start_rate_lo_rad_s ||
      theta_v_rate_ > h.start_rate_hi_rad_s)
    return refuse(R::Tilted);
  if (std::abs(ground_v_) > h.start_max_speed_m_s || std::abs(cmd.forward_m_s) > 1.0e-3F ||
      std::abs(cmd.yaw_rad_s) > 1.0e-3F)
    return refuse(R::Moving);

  hop_.config() = h;
  if (hop_req_clearance_m_ > 0.0F) hop_.config().clearance_m = std::clamp(hop_req_clearance_m_, 0.005F, 0.05F);
  const float knee0 = kneeCommand();
  const float hip0 = hipCommand();
  hop_hip_start_ = hip0;
  hop_c_ref_ = comLine(hip0, knee0).angle;
  // CoM height gained per knee rad near the top of the stroke, with the CoM line held (same plan numbers as
  // HopSequence::begin).
  const float bottom = std::max(std::min(knee0 - std::max(h.dip_rad, 0.0F), h.dip_knee_rad), h.knee_min_rad);
  const float top = bottom + std::min(h.stroke_rad, h.knee_max_rad - bottom);
  float z1 = 0.0F, z2 = 0.0F;
  const float hip1 = hopHipFor(top, hip0, hop_c_ref_, &z1);
  (void)hopHipFor(top - 0.05F, hip1, hop_c_ref_, &z2);
  const float dz_dk = (z1 - z2) / 0.05F;
  if (!hop_.begin(knee0, dz_dk)) return refuse(R::Height);

  hop_arm_ = false;
  hop_dhip0_ = dhip_;
  hop_tier2_on_ = false;
  hop_trim_ = lqr_.balanceTrim();
  hop_tv0_ = theta_v_;
  hop_tv_goal_ = pid_ref_;
  hop_in_flight_ = false;
  hop_carrier0_ = hop_carrier_lp_ = body_rate_ + 0.5F * (hip_rate_meas_ + knee_rate_meas_) * 2.0F;
  hop_tv_corr_ = 0.0F;
  hop_hip_cmd_ = hip0;
  hop_knee_cmd_ = knee0;
  hop_hip_rate_ = hop_knee_rate_ = 0.0F;
  hop_leg_override_ = true;
  hop_wheels_free_ = false;
  held_suppress_until_us_ = in.now_us + 2'500'000U;
}

void V3Controller::hopStep(const Input& in) {
  HopSequence::Input hi{};
  hi.dt_s = in.dt_s;
  hi.accel_norm_m_s2 = std::sqrt(in.accel_x_m_s2 * in.accel_x_m_s2 + in.accel_y_m_s2 * in.accel_y_m_s2 +
                                 in.accel_z_m_s2 * in.accel_z_m_s2);
  {
    // Rotation rate of the shank in the world as the leg path commands it, 40 ms low-pass (the wheel lags anyway).
    const float c_now = body_rate_ + (hop_.active() ? hop_hip_rate_ + hop_knee_rate_ : 0.0F);
    hop_carrier_lp_ += std::clamp(in.dt_s / (0.04F + in.dt_s), 0.0F, 1.0F) * (c_now - hop_carrier_lp_);
  }
  hi.calm = std::abs(theta_v_ - hop_tv_goal_) < 0.05F && std::abs(theta_v_rate_) < 0.5F && std::abs(body_rate_) < 0.6F &&
            std::abs(pid_v_filt_) < 0.15F;
  const HopSequence::Output o = hop_.update(hi);
  legs_moved_us_ = in.now_us;  // a deliberate leg motion: the held / lifted test stays off
  held_suppress_until_us_ = std::max(held_suppress_until_us_, in.now_us + 2'500'000U);
  hop_tier2_on_ = config_.hop.tier2_after_touchdown &&
                  (o.phase == HopSequence::Phase::Land || o.phase == HopSequence::Phase::Recover);
  hop_wheels_free_ = o.wheels_free || o.wheels_limp;
  hop_wheels_limp_ = o.wheels_limp;
  if (o.touchdown_edge) {
    hop_wheels_limp_ = false;
    hop_wheels_free_ = false;
    hopResume(in);
  }
  if (hop_.active()) {
    const float prev_hip = hop_hip_cmd_;
    hop_knee_cmd_ = o.knee_target_rad;
    float knee_for_hip = hop_knee_cmd_;
    if (config_.hop.hip_pred_s > 0.0F && legs_measured_ && o.phase != HopSequence::Phase::Flight &&
        o.phase != HopSequence::Phase::Land) {
      knee_for_hip = knee_meas_ + config_.hop.hip_pred_s * knee_rate_meas_;
    }
    float hip_plan = hopHipFor(knee_for_hip, hop_hip_cmd_ - hop_tv_corr_,
                               hop_c_ref_ + config_.hop.thrust_lean_rad * hop_.thrustLeanWeight(), nullptr);
    if ((config_.hop.tv_kp > 0.0F || config_.hop.tv_kd_s > 0.0F) && o.phase != HopSequence::Phase::Flight) {
      const float jh = comLine(hip_plan, knee_for_hip).d_hip;
      const float e = theta_v_ - hop_tv0_;
      const float want = -(config_.hop.tv_kp * e + config_.hop.tv_kd_s * theta_v_rate_) /
          (std::abs(jh) > 0.05F ? jh : std::copysign(0.05F, jh));
      hop_tv_corr_ = std::clamp(want, -config_.hop.tv_max_rad, config_.hop.tv_max_rad);
    } else if (o.phase == HopSequence::Phase::Flight) {
      // keep the last correction in the air
    } else {
      hop_tv_corr_ = 0.0F;
    }
    hop_hip_cmd_ = hip_plan + hop_tv_corr_;
    if (hop_tier2_on_) hop_hip_cmd_ += dhip_ - hop_dhip0_;   // tier-2 reflex after the touchdown (Land / Recover)
    if (config_.hop.flight_sens > 0.0F && o.phase == HopSequence::Phase::Flight) {
      // Flight: swing the hip so the CoM-axle line is upright at the planned touchdown.
      if (!hop_in_flight_) {
        hop_in_flight_ = true;
        hop_flight_hip0_ = prev_hip;
        hop_flight_corr_ = 0.0F;
      }
      const float t_rem = hop_.flightRemainingS();
      const float pred = theta_v_ + theta_v_rate_ * t_rem - hop_tv_goal_;
      const float want = std::clamp(-pred / config_.hop.flight_sens, -config_.hop.flight_hip_max_rad,
                                    config_.hop.flight_hip_max_rad);
      const float step = config_.hop.flight_hip_rate_rad_s * in.dt_s;
      hop_flight_corr_ += std::clamp(want - hop_flight_corr_, -step, step);
      hop_hip_cmd_ = hop_flight_hip0_ + hop_flight_corr_;
    } else if (o.phase != HopSequence::Phase::Flight) {
      hop_in_flight_ = false;
    }
    hop_knee_rate_ = o.knee_rate_rad_s;
    hop_hip_rate_ = in.dt_s > 1.0e-4F ? std::clamp((hop_hip_cmd_ - prev_hip) / in.dt_s, -8.0F, 8.0F) : 0.0F;
  } else {
    hopEnd();
  }
}

// Touchdown: the wheel balance loop takes over again from the measured state, with the pre-hop balance trim and
// no stale position error (the anchor re-latches once the robot is still, as after a DRIVE).
void V3Controller::hopResume(const Input& in) {
  lqr_.reset(ground_x_, true);
  lqr_.setBalanceTrim(hop_trim_);
  lqr_x_target_m_ = ground_x_;
  stand_give_m_ = 0.0F;
  lqr_lead_filt_ = 0.0F;
  lqr_.seedForwardVelocity(ground_v_);
  pid_v_filt_ = ground_v_;
  plan_v_ = ground_v_;
  plan_a_ = plan_j_ = 0.0F;
  plan_head_ = plan_n_ = 0U;
  plan_init_ = true;
  pid_driving_ = true;
  ga_init_ = false;
  held_window_start_us_ = 0U;
  runaway_since_us_ = 0U;
  held_suppress_until_us_ = in.now_us + 2'500'000U;
}

void V3Controller::hopEnd() noexcept {
  hop_leg_override_ = false;
  hop_wheels_free_ = false;
  hop_wheels_limp_ = false;
  hop_tier2_on_ = false;
  hop_hip_rate_ = hop_knee_rate_ = 0.0F;
  if (hop_.config().enabled) config_.hop.thrust_gain = hop_.config().thrust_gain;   // hop-to-hop adaptation
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
  cfg.lqr.command_velocity_tracking_margin_m_s = 0.25F;
  cfg.lqr.max_ground_velocity_m_s = 4.19F;  // 1000 rpm, the wheel command clamp (rated torque only to 400 rpm = 1.67 m/s)
  // Demo turning: 1.5 rad/s (~86 deg/s) = 0.23 m/s per wheel at the 0.1545 m
  // half track; the default 0.8 capped the BLE stick well below that.
  cfg.lqr.target_yaw_slew_rate_rad_s2 = 0.0F;  // the yaw profile is Tier::yaw_accel/jerk (the stock 1 rad/s^2 capped turning)
  cfg.lqr.yaw_rate_feedback_kp = 0.6F;  // gyro yaw-rate feedback, now really wired; gain margin ~9 dB at 55 ms command lag
  cfg.lqr.max_yaw_rate_rad_s = 12.0F;  // DRIVE yaw ceiling; the GUI slider chooses the rate (was 1.5)
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
