#include "robot_control/EmbeddedController.hpp"

#include <algorithm>
#include <cmath>

namespace robot_control {

namespace {

constexpr auto left_hip = static_cast<std::size_t>(JointId::LeftHip);
constexpr auto left_knee = static_cast<std::size_t>(JointId::LeftKnee);
constexpr auto left_wheel = static_cast<std::size_t>(JointId::LeftWheel);
constexpr auto right_hip = static_cast<std::size_t>(JointId::RightHip);
constexpr auto right_knee = static_cast<std::size_t>(JointId::RightKnee);
constexpr auto right_wheel = static_cast<std::size_t>(JointId::RightWheel);

[[nodiscard]] bool usesBalance(ControlMode mode) {
  return mode == ControlMode::Balance || mode == ControlMode::BalanceRamp ||
         mode == ControlMode::Drive ||
         mode == ControlMode::PoseChange;
}

bool validInput(const RobotState& state, const ControlCommand& command) {
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (!std::isfinite(state.joints.position_rad[i]) ||
        !std::isfinite(state.joints.velocity_rad_s[i]) ||
        !std::isfinite(command.target_joint_position_rad[i])) return false;
  }
  return std::isfinite(state.dt_s) && state.dt_s > 0 &&
      std::isfinite(state.imu.pitch_rad) &&
      std::isfinite(state.imu.pitch_rate_rad_s) &&
      (std::isfinite(state.imu.sample_dt_s) && state.imu.sample_dt_s >= 0.0F) &&
      std::isfinite(state.imu.roll_rate_rad_s) &&
      std::isfinite(state.imu.yaw_rate_rad_s) &&
      std::isfinite(state.imu.accel_magnitude_m_s2) &&
      std::isfinite(state.wheel.position_m) &&
      std::isfinite(state.wheel.velocity_m_s) &&
      std::isfinite(state.wheel.left_position_m) &&
      std::isfinite(state.wheel.right_position_m) &&
      std::isfinite(state.wheel.left_velocity_m_s) &&
      std::isfinite(state.wheel.right_velocity_m_s) &&
      std::isfinite(command.target_forward_velocity_m_s) &&
      std::isfinite(command.target_yaw_rate_rad_s) &&
      std::isfinite(command.target_body_height_m);
}

[[nodiscard]] float quinticBlend(float ratio) {
  ratio = std::clamp(ratio, 0.0F, 1.0F);
  const float ratio2 = ratio * ratio;
  const float ratio3 = ratio2 * ratio;
  return ratio3 * (10.0F + ratio * (-15.0F + 6.0F * ratio));
}

[[nodiscard]] EmbeddedController::Config normalizeConfig(
    EmbeddedController::Config config) {
  config.state_machine.fall_pitch_rad = config.lqr.fall_pitch_rad;
  return config;
}

}  // namespace

EmbeddedController::EmbeddedController() : EmbeddedController(Config{}) {}

EmbeddedController::EmbeddedController(Config config)
    : config_(normalizeConfig(config)),
      lqr_(config_.lqr),
      whole_body_(config_.whole_body),
      whole_body_stance_(config_.whole_body_stance),
      joint_motion_(config_.joint_motion),
      state_machine_(config_.state_machine),
      flight_observer_(config_.flight_observer) {}

float EmbeddedController::updateBalancePitchRef(const RobotState& state,
                                               const ControlCommand& command) {
  constexpr auto lk = static_cast<std::size_t>(JointId::LeftKnee);
  constexpr auto rk = static_cast<std::size_t>(JointId::RightKnee);
  const bool static_com_geometry_valid =
      std::abs(state.joints.position_rad[lk]) +
          std::abs(state.joints.position_rad[rk]) > 0.70F;
  // Keep the MPC's inverted-pendulum arm matched to the live CoM height, so a
  // squat (shorter, faster pendulum) does not slowly diverge. Inert at the
  // straight-leg stance.
  lqr_.setComHeight(last_whole_body_.com_height_m);

  float target = 0.0F;
  // The geometric pitch reference belongs to the selected body height, not
  // only to the brief interval in which the legs are moving.  Dropping it at
  // the endpoint made a settled squat look like a neutral pose and produced a
  // slow self-roll immediately after the transition.  Natural ENABLE remains
  // the explicit zero-reference branch.
  const bool non_neutral_height =
      std::abs(height_motion_target_m_ - geometry::neutral_spawn_z_m) > 1.0e-4F;
  const bool height_transaction = height_motion_active_ ||
      height_motion_pending_ || posture_motion_needs_station_reanchor_;
  const bool stationary_command =
      std::abs(command.target_forward_velocity_m_s) < 0.01F &&
      std::abs(command.target_yaw_rate_rad_s) < 0.05F;
  const bool stand_up_recovery = state_machine_.mode() == ControlMode::StandingUp ||
      stand_up_stage_ != StandUpStage::Idle;
  // The stance allocator owns the slow body-pitch posture in grounded
  // Balance/Drive. MPC still catches the residual pitch/rate transient; this
  // gate prevents the old geometric FK pitch reference from becoming a
  // second equilibrium that the wheels and legs chase independently. A live
  // C2 height trajectory is its own temporary leg owner.
  const bool leg_body_pitch_owner =
      config_.whole_body_stance_owns_static_com &&
      config_.whole_body_stance.enabled && state.floor_contact &&
      usesBalance(state_machine_.mode()) && !height_motion_pending_ &&
      !height_motion_active_ &&
      !joint_motion_.active() && !stand_up_recovery &&
      static_com_geometry_valid;
  // Do not make the WBC wait for pitch to already be at the target: that
  // would make the MPC pitch reference the very thing preventing the leg task
  // from correcting it. Once a supported, zero-drive station owns the local
  // anchor, the leg task owns the static COM target (including a height
  // transition). The wheel MPC remains the dynamic capture loop for DRIVE.
  const bool stance_owns_static_com =
      config_.whole_body_stance_owns_static_com &&
      config_.whole_body_stance.enabled && state.floor_contact &&
      stationary_command &&
      !height_motion_active_ && !height_motion_pending_ &&
      !joint_motion_.active() &&
      station_hold_active_ && station_capture_ready_ &&
      static_com_geometry_valid;
  // Rolling Drive keeps the geometric pitch reference until the rolling
  // contact allocator exists.  The leg COM position task is deliberately
  // stationary-only, so a rolling wheel is not treated as a fixed stance
  // contact.
  const bool stance_com_owner = stance_owns_static_com;
  const bool height_compensation =
      (height_transaction || non_neutral_height) &&
      config_.height_whole_body_compensation && !stance_com_owner;
  // Only the short station re-anchor cooldown holds the wheel reference at
  // zero. During the height path itself the live FK equilibrium is the
  // reference, so changing COM geometry does not become wheel translation.
  const bool posture_com_task_owns_pitch =
      config_.whole_body_stance_owns_static_com &&
      config_.whole_body_stance.enabled && state.floor_contact &&
      stationary_command &&
      static_com_geometry_valid &&
      station_reanchor_cooldown_s_ > 0.0F && station_capture_ready_;
  if ((leg_body_pitch_owner || config_.whole_body_compensation ||
       height_compensation ||
       stance_com_owner) && !posture_com_task_owns_pitch &&
      !stand_up_recovery) {
    // Once the leg COM task owns the supported posture, the wheel MPC must
    // hold the body at zero pitch.  The legs then move the body-frame COM
    // line to cancel the measured geometric bias instead of asking the body
    // to lean and handing that bias back to the wheels.
    const float pitch_gain = leg_body_pitch_owner || stance_com_owner
        ? 0.0F
        : height_compensation
        ? config_.height_whole_body_pitch_gain
        : config_.whole_body_pitch_gain;
    target = leg_body_pitch_owner || stance_com_owner
        ? 0.0F
        : pitch_gain * WholeBodyModel::equilibriumPitch(last_whole_body_);

    // Slow integral trim on top of the feed-forward, to absorb the leg sag and
    // dynamic load the FK model does not see. It only adapts when the robot is
    // genuinely settled at a static pose - a steady wheel command, no drive
    // input, no pose motion, near the current target - so a transient never
    // pushes it. Same idea as the removed PID's equilibrium-pitch estimator.
    const float steady_v = lqr_.debugState().commanded_ground_velocity_m_s;
    // Only trim once the legs are genuinely bent - at the straight-leg stance
    // (bring-up, normal balancing) the knee sum is ~0 and the trim can never
    // wind up.
    const float knee_bend = std::abs(state.joints.position_rad[lk]) +
                            std::abs(state.joints.position_rad[rk]);
    const bool legs_bent = knee_bend > 0.15F;
    const bool settled =
        state_machine_.mode() == ControlMode::Balance &&
        !joint_motion_.active() &&
        std::abs(command.target_forward_velocity_m_s) < 1.0e-3F &&
        std::abs(command.target_yaw_rate_rad_s) < 1.0e-3F &&
        std::abs(state.imu.pitch_rate_rad_s) < 0.15F &&
        std::abs(state.imu.pitch_rad - balance_pitch_ref_rad_) < 0.05F &&
        std::abs(steady_v) < config_.whole_body_trim_velocity_band;
    if (!leg_body_pitch_owner && config_.whole_body_trim_gain_hz > 0.0F &&
        legs_bent && settled) {
      // Dwell: require trim_dwell_s of continuously-settled time before the
      // integrator is allowed to move, so a pose transition never kicks it.
      trim_dwell_elapsed_s_ += state.dt_s;
      if (trim_dwell_elapsed_s_ >= config_.whole_body_trim_dwell_s) {
        pitch_trim_rad_ -=
            config_.whole_body_trim_gain_hz * steady_v * state.dt_s;
        pitch_trim_rad_ = std::clamp(pitch_trim_rad_,
                                     -config_.max_whole_body_trim_rad,
                                     config_.max_whole_body_trim_rad);
      }
    } else {
      trim_dwell_elapsed_s_ = 0.0F;
      // Bleed the trim back out as the legs straighten so no stale bias is left
      // once the robot stands back up.
      if (!legs_bent) {
        const float decay = std::clamp(state.dt_s / 0.5F, 0.0F, 1.0F);
        pitch_trim_rad_ -= decay * pitch_trim_rad_;
      }
    }
    if (!leg_body_pitch_owner) target += pitch_trim_rad_;
    target = std::clamp(target, -config_.max_balance_pitch_ref_rad,
                        config_.max_balance_pitch_ref_rad);
  }

  const float period = config_.balance_pitch_ref_period_s;
  if (period <= 0.0F) {
    balance_pitch_ref_rad_ = target;
  } else {
    const float alpha =
        std::clamp(std::max(state.dt_s, 1.0e-4F) / period, 0.0F, 1.0F);
    balance_pitch_ref_rad_ += alpha * (target - balance_pitch_ref_rad_);
  }
  return balance_pitch_ref_rad_;
}

LqrBalancer::Output EmbeddedController::runBalance(
    const RobotState& state, const ControlCommand& command, bool integrate) {
  // A low-specific-force sample is an early unloading hypothesis, not yet a
  // confirmed airborne interval. Keep the wheel MPC's grounded state alive
  // through that reversible window; otherwise LqrBalancer deliberately zeros
  // its catch command and a false support gap becomes a pitch fall. Once the
  // FlightObserver reaches BallisticFlight/TouchdownCandidate this window is
  // closed and the normal no-ground-wrench behavior remains unchanged.
  const bool transient_support_window = integrate && !state.floor_contact &&
      !last_flight_estimate_.airborne &&
      !last_flight_estimate_.support_settling &&
      last_flight_estimate_.phase == FlightObserver::Phase::TransientUnloading;
  const bool balance_support = state.floor_contact || transient_support_window;
  RobotState lqr_state = state;
  if (transient_support_window) {
    lqr_state.floor_contact = true;
    lqr_state.left_floor_contact = true;
    lqr_state.right_floor_contact = true;
  }
  // Station-keeping arms only after a sensor-derived grounded Balance state has
  // been quiet for a dwell. The anchor is local encoder odometry, acquired at
  // this edge; it is released for drive and pose transitions.
  const bool zero_drive_command =
      std::abs(command.target_forward_velocity_m_s) < 1.0e-3F &&
      std::abs(command.target_yaw_rate_rad_s) < 1.0e-3F;
  // The leg path owns the controller until its C2 endpoint. After that edge,
  // the station re-anchor cooldown handles the short estimator/MPC handoff.
  const bool posture_motion_active =
      height_motion_active_ || height_motion_pending_ ||
      joint_motion_.active() ||
      (height_motion_endpoint_reached_ && !height_motion_settled_);
  const bool yaw_only_command = !zero_drive_command &&
      std::abs(command.target_forward_velocity_m_s) < 1.0e-3F &&
      std::abs(command.target_yaw_rate_rad_s) >= 1.0e-3F;
  const bool posture_station_mode =
      state_machine_.mode() == ControlMode::Balance ||
      state_machine_.mode() == ControlMode::PoseChange ||
      (state_machine_.mode() == ControlMode::Drive &&
       (yaw_only_command || zero_drive_command));
  // A dance/turn may rotate while the legs change height.  It is still a
  // station task in translation: release the yaw differential, not the
  // captured wheel-position anchor, while the posture path is active.
  const bool posture_yaw_station_requested =
      config_.lqr.hold_station_during_pose && posture_motion_active &&
      posture_station_mode && yaw_only_command;
  const bool posture_station_requested =
      config_.lqr.hold_station_during_pose && posture_motion_active &&
      posture_station_mode &&
      (zero_drive_command || posture_yaw_station_requested);
  // A pure yaw request from an already parked robot must retain the local
  // position anchor. Releasing it turns any residual pitch-capture velocity
  // into an unintended circular drive while the operator is only turning.
  // Preserve an already-qualified translation anchor across the Balance ->
  // Drive mode edge of an in-place turn. `station_hold_active_` can be cleared
  // by the preceding tick's command handoff before the state machine reports
  // Drive, while the previous eligibility is still valid for this yaw task.
  const bool yaw_only_station_hold = yaw_only_command &&
      (station_hold_active_ || station_eligible_ ||
       posture_yaw_station_requested);
  const bool yaw_only_handoff = yaw_only_command &&
      !yaw_only_handoff_done_;
  if (!yaw_only_command) yaw_only_handoff_done_ = false;
  else if (yaw_only_handoff) yaw_only_handoff_done_ = true;
  if (posture_motion_needs_station_reanchor_ &&
      !height_motion_active_ && !height_motion_pending_ &&
      !joint_motion_.active()) {
    // The leg path may have required a bounded wheel catch. Re-anchor at the
    // supported endpoint so the next settled posture does not pull back
    // through a long, unstable correction to the pre-motion origin. Preserve
    // the measured velocity; only the position reference changes.
    station_dwell_s_ = 0.0F;
    // This is a transaction handoff, not a normal hold update.  Even when
    // position_anchor_hold is enabled for the LQR, retaining the pre-pose
    // anchor here makes the next pose pull the wheels back through the drift
    // accumulated while the legs moved.
    lqr_.reanchorPosition(state.wheel.position_m);
    // Re-seed the outer station reference from the new zero-drive operating
    // point without discarding the LQR's measured catch state. A drive stop
    // still clears the commanded velocity explicitly.
    if (zero_drive_command) {
      lqr_.setStationVelocityReference(0.0F);
    } else {
      lqr_.clearForwardVelocityReference();
    }
    height_motion_endpoint_anchor_m_ = state.wheel.position_m;
    station_hold_active_ = config_.lqr.hold_station_during_pose &&
        balance_support;
    station_reanchor_cooldown_s_ =
        std::max(config_.posture_reanchor_settle_s, 0.0F);
    posture_motion_needs_station_reanchor_ = false;
  }
  // A yaw-only command is represented as Drive by the state machine, but it
  // is still a station task in translation: the differential wheel speeds
  // should create heading change around the captured local anchor.  Requiring
  // Balance here released that anchor at the exact yaw hand-off and allowed a
  // stale forward MPC reference to become an unintended arc drive.  Ordinary
  // forward/reverse Drive remains excluded by yaw_only_station_hold.
  const bool balancing_mode =
      state_machine_.mode() == ControlMode::Balance ||
      state_machine_.mode() == ControlMode::Drive ||
      state_machine_.mode() == ControlMode::PoseChange;
  const bool station_eligible =
      balance_support && balancing_mode &&
      (zero_drive_command || yaw_only_station_hold) &&
      (!posture_motion_active || posture_station_requested);
  station_eligible_ = station_eligible;
  const float commanded_velocity =
      lqr_.debugState().commanded_ground_velocity_m_s;
  // Station anchoring must not erase the MPC's braking state while a stop is
  // still in progress.  A fixed velocity band is wrong across poses and
  // speeds: the same residual velocity is easy to capture for a short
  // pendulum and much more disruptive for a tall one.  Express the gate in
  // the robot's current capture distance instead.  This is the distance a
  // wheel correction still needs to absorb the measured/controller velocity,
  // pitch displacement and pitch rate around the live linearized pendulum.
  const float pendulum_length = std::clamp(
      config_.lqr.pendulum_length_per_com_height *
          std::max(last_whole_body_.com_height_m, 0.02F),
      config_.lqr.min_pendulum_length_m,
      config_.lqr.max_pendulum_length_m);
  const float gravity = std::max(config_.lqr.gravity_m_s2, 0.1F);
  const float omega = std::sqrt(gravity / std::max(pendulum_length, 0.02F));
  const float pending_velocity = std::max(
      std::abs(state.wheel.velocity_m_s), std::abs(commanded_velocity));
  // This gate answers whether the sensor stream is quiet enough to start or
  // finish a leg transaction.  Keep it on the measured body signal; the
  // virtual pendulum remains the balance controller's state, but a FK/encoder
  // transient must not make the posture transaction own the robot too early.
  const float pitch_error = state.imu.pitch_rad - balance_pitch_ref_rad_;
  const float pitch_rate_for_capture = state.imu.pitch_rate_rad_s;
  const float raw_capture_distance =
      pending_velocity / std::max(omega, 1.0e-3F) +
      pendulum_length * std::abs(std::sin(pitch_error)) +
      std::abs(pitch_rate_for_capture) /
          std::max(omega * omega, 1.0e-3F);
  posture_capture_distance_m_ = raw_capture_distance;
  // Any station task must account for translation already lost from its
  // fixed odometry anchor. Ignore only the explicitly configured small
  // kinematic/encoder envelope; expose excess to the capture gate so a
  // chained height transaction cannot inherit that drift.
  const float anchor_error = std::abs(
      state.wheel.position_m - lqr_.debugState().target_ground_position_m);
  const bool station_task = (zero_drive_command || yaw_only_command) &&
      (station_hold_active_ || station_eligible);
  const float anchor_capture_envelope = std::max(
      std::max(config_.height_capture_pause_distance_m, 0.0F),
      std::max(config_.station_capture_margin_m, 0.0F));
  const float anchor_excess = std::max(
      anchor_error - anchor_capture_envelope, 0.0F);
  const float capture_distance_with_anchor = raw_capture_distance +
      (station_task ? anchor_excess : 0.0F);
  // Encoder velocity is quantized and can jump by several cm/s from one
  // packet to the next.  Filter the derived capture distance, rather than
  // filtering velocity with a pose-specific constant.  The filter is
  // re-seeded whenever station eligibility is lost, so a new stop always
  // starts from the real measured state and a drive command cannot leak old
  // settling history into the next anchor decision.
  if (!station_eligible || !station_capture_filter_initialized_) {
    station_capture_distance_filtered_m_ = capture_distance_with_anchor;
    station_capture_filter_initialized_ = station_eligible;
  } else {
    const float alpha = std::clamp(
        state.dt_s / std::max(config_.station_capture_filter_period_s,
                              1.0e-3F),
        0.0F, 1.0F);
    station_capture_distance_filtered_m_ += alpha *
        (capture_distance_with_anchor - station_capture_distance_filtered_m_);
  }
  station_capture_distance_m_ = station_capture_distance_filtered_m_;
  const float capture_margin = std::max(config_.station_capture_margin_m, 0.0F);
  // The filtered value rejects encoder packet noise before the settling dwell
  // can arm the anchor.  The raw value is still used to seed/restart the
  // filter whenever eligibility is lost, so a new stop cannot inherit a
  // previously quiet state.
  // `station_eligible` already excludes an active posture transaction.  Do
  // not add a fixed knee-velocity threshold here: encoder quantization and
  // load-dependent leg settling can otherwise prevent anchoring forever even
  // after the capture state is stable.  Any remaining body motion is already
  // represented by the IMU terms in station_capture_distance_m_.
  station_capture_ready_ = station_capture_distance_m_ <= capture_margin;
  const bool parked_quiet = station_eligible && station_capture_ready_;
  if (!station_eligible) {
    station_dwell_s_ = 0.0F;
    station_hold_active_ = false;
  } else if (!station_hold_active_) {
    station_dwell_s_ = parked_quiet ? station_dwell_s_ + state.dt_s : 0.0F;
    if (station_dwell_s_ <
        std::max(config_.station_settle_dwell_s, 0.0F)) {
      // Still qualifying the local stationary state.
    } else {
      station_hold_active_ = true;
      // Capture the settled support point without resetting the LQR velocity
      // state. A startup/stop catch may have moved the wheels before station
      // hold qualified; returning to the pre-catch anchor would turn that
      // harmless capture into a repeatable drift during the next skill.
      lqr_.reanchorPosition(state.wheel.position_m);
    }
  } else if (height_motion_endpoint_reached_) {
    // Keep the endpoint station dwell metric available for diagnostics; the
    // normal station loop owns residual encoder-position return after handoff.
    const bool endpoint_quiet = station_eligible &&
        posture_capture_distance_m_ <= capture_margin;
    station_dwell_s_ = endpoint_quiet
        ? station_dwell_s_ + std::max(state.dt_s, 0.0F)
        : 0.0F;
  }
  if (yaw_only_handoff) {
    // Do not carry a residual forward target from a preceding drive command or
    // station pitch-capture correction into an in-place turn. The measured
    // wheel velocity remains in the MPC state and is still actively braked on
    // the next tick.
    lqr_.clearForwardVelocityReference();
  }
  // A stop/yaw handoff needs station feedback before the quiet latch can be
  // acquired: the latch is the result of capture, not the permission to begin
  // capturing.  Keep one MPC owner for both phases; before the latch it
  // returns to the existing anchor, after the latch it remains unchanged.
  const bool hold_station = station_hold_active_ || station_eligible_;
  // A completed height transaction is no longer a posture motion.  The old
  // recovery aliases kept the wheel controller in its 0.25 m/s transient envelope
  // whenever the station anchor still had error, so a normal station return
  // could remain in "posture" mode forever and spend the wheel catch budget.
  // Keep the capture distance in the station latch; only the live C2 path may
  // widen the posture command envelope.
  const bool lqr_posture_motion_active = posture_motion_active;
  // The station velocity decay remains disabled for this CAD profile: it
  // removes the retained catch command before the low-torque wheel servo has
  // settled. Position/velocity station feedback owns the slow return instead.
  lqr_.setStationVelocityDecayEnabled(lqr_posture_motion_active);
  lqr_.setPostureMotionActive(lqr_posture_motion_active);
  // A pending height request is only an operator-side transaction gate. The
  // balance loop must continue with its normal geometric pitch reference and
  // its existing velocity/warm state while the gate waits for capture. Reset
  // or direct IMU-following here turns a harmless posture request into a wheel
  // impulse and makes the result depend on the exact command timing.
  const float pitch_ref = updateBalancePitchRef(state, command);
  const bool stationary_height_transaction = zero_drive_command &&
      (posture_motion_active || height_motion_active_ ||
       (height_motion_endpoint_reached_ && !height_motion_settled_));
  // Keep the posture station gain through the short endpoint handoff.  The
  // gain family itself must not switch on the same tick as the leg owner;
  // cooldown then gives the tall stance time to settle before normal station
  // gains resume.
  lqr_.setStationPositionMode(stationary_height_transaction);
  // The height path is task-space motion of the legs, not a request to drive
  // the robot across the floor. Keep the local anchor enabled while the
  // operator changes height; the live geometric pitch reference handles the
  // changing CoM. Completion re-anchors at the supported endpoint, so the
  // next posture does not pull through the previous transaction's catch.
  if (station_reanchor_cooldown_s_ > 0.0F) {
    // Let the wheel catch residual endpoint momentum without spending it
    // against the pre-settle support point. Re-seed the anchor from the live
    // encoder during this short handoff; the first fixed anchor was captured
    // before the loaded leg pose finished moving.
    lqr_.reanchorPosition(state.wheel.position_m);
    height_motion_endpoint_anchor_m_ = state.wheel.position_m;
  }
  // A zero DRIVE command first enters a braking transient.  Do not hold the
  // pre-stop position during that transient: wheel inertia would carry the
  // robot past the release point and the position state would then pull it
  // visibly backward.  The quiet station latch re-anchors and enables this
  // position state after capture; posture motion keeps it enabled throughout.
  const bool position_anchor_active = station_hold_active_ ||
      posture_motion_active;
  lqr_.setPositionAnchorHold(config_.lqr.position_anchor_hold &&
                              position_anchor_active &&
                              !posture_motion_needs_station_reanchor_ &&
                              station_reanchor_cooldown_s_ <= 0.0F);
  if (height_motion_endpoint_reached_ && station_reanchor_cooldown_s_ <= 0.0F) {
    // The leg endpoint can move the wheel odometry origin by a few centimetres.
    // Move the station target at the live pendulum bandwidth instead of
    // re-anchoring in one tick; the latter injects a position step into MPC.
    const float endpoint_omega = std::sqrt(
        std::max(config_.lqr.gravity_m_s2, 0.1F) /
        std::max(lqr_.activePendulumLengthM(), 0.02F));
    const float anchor_alpha = 1.0F - std::exp(-endpoint_omega *
                                                std::max(state.dt_s, 0.0F));
    lqr_.movePositionAnchorToward(height_motion_endpoint_anchor_m_,
                                  anchor_alpha);
  }
  // A pending height request remains a station task. The anchor is captured
  // once at the pending edge and the bounded station return prevents the
  // quiet-state gate from disabling its own feedback loop.
  const auto output = lqr_.update(lqr_state, command, integrate, pitch_ref,
                                  hold_station);
  last_leg_velocity_error_m_s_ = output.velocity_error_m_s;
  if (height_capture_recovery_active_ && station_capture_ready_ &&
      std::abs(state.imu.pitch_rate_rad_s) <=
          std::max(config_.posture_settle_pitch_rate_rad_s, 0.0F) &&
      std::abs(state.wheel.velocity_m_s) <=
          std::max(config_.posture_settle_wheel_velocity_m_s, 0.0F)) {
    height_capture_recovery_active_ = false;
  }
  return output;
}

void EmbeddedController::resetBalance(float ground_position_m, bool floor_contact) {
  lqr_.reset(ground_position_m, floor_contact);
}

void EmbeddedController::requestStationaryStop(float ground_position_m,
                                               bool floor_contact) noexcept {
  // Do not reset a moving inverted pendulum: that removes the catch state at
  // the exact instant it is needed and turns a stop into a fall. Let LQR brake
  // from its measured velocity; only a quiet robot may discard its old anchor.
  if (floor_contact && std::abs(lqr_.debugState().commanded_ground_velocity_m_s) <
          0.05F) {
    lqr_.reanchorPosition(ground_position_m);
    lqr_.clearForwardVelocityReference();
  }
  stationary_stop_hold_s_ = 1.0F;
}

void EmbeddedController::setBalanceFallPitch(float fall_pitch_rad) noexcept {
  lqr_.setFallPitchRad(fall_pitch_rad);
}

const LqrBalancer::DebugState& EmbeddedController::lqrDebugState() const noexcept {
  return lqr_.debugState();
}

void EmbeddedController::reset(const RobotState& state) {
  // Reset is called at a sensor-stream boundary where legacy contact telemetry
  // may still be present in the compatibility fields.  Do not seed any
  // controller state from those fields.  The reset contract is a neutral
  // startup assumption; the first update() immediately replaces the local
  // contact hypothesis from FlightObserver's MPU6500 signals.
  resetBalance(state.wheel.position_m, true);
  setBalanceFallPitch(config_.lqr.fall_pitch_rad);
  joint_motion_.reset(state.joints.position_rad);
  height_motion_active_ = false;
  height_motion_endpoint_reached_ = false;
  height_motion_settled_ = false;
  height_motion_rejected_ = false;
  height_capture_recovery_active_ = false;
  height_motion_pending_ = false;
  height_pending_anchor_captured_ = false;
  height_pending_elapsed_s_ = 0.0F;
  height_rejected_target_valid_ = false;
  height_rejected_target_m_ = geometry::neutral_spawn_z_m;
  height_motion_start_m_ = geometry::neutral_spawn_z_m;
  height_motion_target_m_ = geometry::neutral_spawn_z_m;
  height_motion_elapsed_s_ = 0.0F;
  height_capture_pause_elapsed_s_ = 0.0F;
  height_motion_duration_s_ = 0.0F;
  height_motion_last_hip_rad_ = geometry::neutral_hip_rad;
  height_motion_last_knee_rad_ = geometry::neutral_knee_rad;
  height_motion_endpoint_anchor_m_ = state.wheel.position_m;
  height_pending_pose_.fill(0.0F);
  posture_motion_needs_station_reanchor_ = false;
  state_machine_.reset();
  balance_pitch_ref_rad_ = 0.0F;
  last_leg_velocity_error_m_s_ = 0.0F;
  posture_abort_blend_ = 0.0F;
  pitch_trim_rad_ = 0.0F;
  trim_dwell_elapsed_s_ = 0.0F;
  station_dwell_s_ = 0.0F;
  posture_settle_dwell_s_ = 0.0F;
  station_hold_active_ = false;
  station_reanchor_cooldown_s_ = 0.0F;
  yaw_only_handoff_done_ = false;
  station_capture_distance_m_ = 0.0F;
  posture_capture_distance_m_ = 0.0F;
  station_capture_distance_filtered_m_ = 0.0F;
  station_capture_ready_ = false;
  station_eligible_ = false;
  station_capture_filter_initialized_ = false;
  whole_body_stance_.reset();
  last_whole_body_stance_ = WholeBodyStanceController::Output{};
  whole_body_gravity_feedforward_.fill(0.0F);
  flight_observer_.reset(state);
  last_flight_estimate_ = FlightObserver::Output{};
  stand_up_stage_ = StandUpStage::Idle;
  stand_up_settle_cycles_ = 0;
  stand_up_settle_elapsed_s_ = 0.0F;
  stand_up_fall_sign_ = 1.0F;
  last_output_ = MotorCommand{};
  last_imu_sample_sequence_ = 0;
  last_encoder_sample_sequence_ = 0;
  sensor_sequence_initialized_ = false;
}

bool EmbeddedController::standingUp() const noexcept {
  return stand_up_stage_ != StandUpStage::Idle;
}

MotorCommand EmbeddedController::update(const RobotState& input_state,
                                        const ControlCommand& input) {
  const float finish_dt = std::max(input_state.dt_s, 0.0F);
  const auto finish = [this, finish_dt, &input_state](MotorCommand output) {
    // Single chokepoint for every return path - see the config note on
    // leg_command_max_rate_rad_s. Only rate-limit while the motors were and
    // remain armed; an enable/disable edge legitimately re-references the
    // command to the measured pose and must not be dragged through it.
    if (config_.leg_command_max_rate_rad_s > 0.0F && finish_dt > 0.0F &&
        output.enable_motors && last_output_.enable_motors) {
      const float max_step = config_.leg_command_max_rate_rad_s * finish_dt;
      for (const auto joint : {left_hip, left_knee, right_hip, right_knee}) {
        const float previous = last_output_.position_rad[joint];
        if (std::isfinite(previous) && std::isfinite(output.position_rad[joint])) {
          const float limited = std::clamp(
              output.position_rad[joint], previous - max_step,
              previous + max_step);
          if (limited != output.position_rad[joint]) {
            ++leg_command_rate_limited_ticks_;
          }
          output.position_rad[joint] = limited;
        }
      }
    }
    // Leg position integral on the final command (see Config).
    const float ki = config_.leg_position_integral_gain_nm_per_rad_s;
    if (ki > 0.0F && output.enable_motors && finish_dt > 0.0F) {
      const float limit = std::max(config_.leg_position_integral_max_nm, 0.0F);
      for (const auto joint : {left_hip, left_knee, right_hip, right_knee}) {
        const float error = output.position_rad[joint] -
            input_state.joints.position_rad[joint];
        if (!std::isfinite(error) || !(output.kp_scale[joint] > 0.0F)) continue;
        leg_position_integral_nm_[joint] = std::clamp(
            leg_position_integral_nm_[joint] + ki * error * finish_dt, -limit, limit);
        output.feedforward_nm[joint] += leg_position_integral_nm_[joint];
      }
    } else if (!output.enable_motors) {
      leg_position_integral_nm_.fill(0.0F);
    }
    last_output_ = output;
    return output;
  };
  if (!validInput(input_state, input)) {
    RobotState invalid = input_state;
    invalid.emergency_stop = true;
    (void)state_machine_.update(invalid, {}, false, false);
    finishStandUp(input_state);
    joint_motion_.reset(input_state.joints.position_rad);
    MotorCommand safe;
    writeSafeStop(safe, input_state);
    return finish(safe);
  }

  // A native/embedded actuator loop may run at 3--4 kHz while MPU6500 and
  // encoder packets arrive at a lower rate. Replaying the same packet on
  // every control tick would advance stateful control and debounce logic
  // several times for one physical observation. Keep the last valid motor
  // command between packets; urgent lifecycle requests still take effect
  // immediately, and legacy callers with zero sequence numbers retain their
  // original every-call semantics.
  const bool sequence_tracked = input_state.imu_sample_sequence != 0 ||
      input_state.encoder_sample_sequence != 0;
  bool fresh_sensor_sample = true;
  if (sequence_tracked) {
    fresh_sensor_sample = !sensor_sequence_initialized_ ||
        input_state.imu_sample_sequence != last_imu_sample_sequence_ ||
        input_state.encoder_sample_sequence != last_encoder_sample_sequence_;
    last_imu_sample_sequence_ = input_state.imu_sample_sequence;
    last_encoder_sample_sequence_ = input_state.encoder_sample_sequence;
    sensor_sequence_initialized_ = true;
  }
  const bool urgent_request = input.request_disable ||
      input.request_emergency_stop || input_state.emergency_stop ||
      (input.request_enable &&
       (state_machine_.mode() == ControlMode::Disabled ||
        state_machine_.mode() == ControlMode::EmergencyStop)) ||
      (input.request_pose_change && !joint_motion_.active()) ||
      (input.request_stand_up && !standingUp());
  if (sequence_tracked && !fresh_sensor_sample && !urgent_request &&
      state_machine_.mode() != ControlMode::Disabled) {
    // last_output_ is already rate-limited and finalized. Re-entering the
    // common finish path here only recopies the command on every physics tick.
    return last_output_;
  }

  // Contact fields from the simulator are not trusted. The flight observer
  // derives a support hypothesis from MPU6500 signals; state-machine and
  // balance decisions below use this local sensor-derived copy.
  RobotState state = input_state;
  ControlCommand command = input;
  state.emergency_stop = state.emergency_stop || command.request_emergency_stop;
  // Keep the balance/WBC model on the fresh encoder pose. The measured model
  // is the only geometry the actuator feedback has actually reached.
  last_whole_body_ = whole_body_.compute(
      state.joints.position_rad[left_hip], state.joints.position_rad[left_knee],
      state.joints.position_rad[right_hip], state.joints.position_rad[right_knee]);
  if (sequence_tracked && state.imu.sample_dt_s > 0.0F &&
      std::isfinite(state.imu.sample_dt_s)) {
    // All stateful control calculations below advance on a fresh sensor
    // interval.  The cached command is what bridges that interval to the
    // faster actuator loop.
    state.dt_s = std::clamp(state.imu.sample_dt_s, 1.0e-4F, 0.05F);
  }
  bool pose_retarget = false;
  if (command.request_pose_change && joint_motion_.active()) {
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (std::abs(command.target_joint_position_rad[i] -
                   joint_motion_.target()[i]) > 1.0e-4F) {
        pose_retarget = true;
        break;
      }
    }
  }
  const bool pose_interrupt = command.request_pose_change &&
      (height_motion_active_ || height_motion_pending_ ||
       height_motion_endpoint_reached_ || pose_retarget);
  if (pose_interrupt) {
    // POSE is the operator's explicit override. Retire the task-space height
    // owner at its last emitted pose, then let the same rate-limited planner
    // take over. This makes a new button press effective mid-transition.
    height_motion_active_ = false;
    height_motion_endpoint_reached_ = false;
    height_motion_settled_ = false;
    height_motion_pending_ = false;
    height_pending_anchor_captured_ = false;
    height_pending_elapsed_s_ = 0.0F;
    height_capture_pause_elapsed_s_ = 0.0F;
    height_motion_rejected_ = false;
    height_rejected_target_valid_ = false;
    if (joint_motion_.active()) joint_motion_.reset(state.joints.position_rad);
    posture_motion_needs_station_reanchor_ = false;
    lqr_.reanchorPosition(state.wheel.position_m);
    lqr_.clearForwardVelocityReference();
    command.request_height_change = false;
  }
  station_reanchor_cooldown_s_ = std::max(
      0.0F, station_reanchor_cooldown_s_ - std::max(state.dt_s, 0.0F));
  stationary_stop_hold_s_ = std::max(
      0.0F, stationary_stop_hold_s_ - std::max(state.dt_s, 0.0F));
  const bool posture_was_active =
      height_motion_active_ || joint_motion_.active() ||
      (height_motion_endpoint_reached_ && !height_motion_settled_);
  // The next transaction must wait for a settled sensor state, not merely a
  // small instantaneous wheel velocity.  A leg path can finish with the body
  // still exchanging pitch energy; starting the next path at that edge makes
  // the MPC interpret the transient as a drive request.
  const float settle_dt = std::max(state.dt_s, 0.0F);
  const float settle_pitch_error = std::abs(
      state.imu.pitch_rad - balance_pitch_ref_rad_);
  const float settle_pitch_rate = std::abs(state.imu.pitch_rate_rad_s);
  const bool height_endpoint_settling =
      height_motion_endpoint_reached_ && !height_motion_settled_ &&
      !height_motion_active_;
  const bool yaw_only_posture_command =
      std::abs(command.target_forward_velocity_m_s) < 1.0e-3F &&
      std::abs(command.target_yaw_rate_rad_s) >= 1.0e-3F;
  // StateMachine labels a pure turn as Drive.  It is still a valid quiet
  // posture-start mode because translation remains zero and runBalance keeps
  // the station anchor for this command.
  const bool height_station_mode =
      state_machine_.mode() == ControlMode::Balance ||
      (state_machine_.mode() == ControlMode::Drive &&
       yaw_only_posture_command);
  const bool posture_sensor_quiet =
      (!posture_was_active || height_endpoint_settling) &&
      !posture_motion_needs_station_reanchor_ &&
      height_station_mode && state.floor_contact &&
      settle_pitch_rate <=
          std::max(config_.posture_settle_pitch_rate_rad_s, 0.0F) &&
      settle_pitch_error <= std::max(config_.posture_settle_pitch_error_rad, 0.0F) &&
      std::abs(state.wheel.velocity_m_s) <=
          std::max(config_.posture_settle_wheel_velocity_m_s, 0.0F) &&
      settle_pitch_rate <=
          std::max(config_.posture_settle_pitch_rate_rad_s, 0.0F);
  if (posture_sensor_quiet && !height_endpoint_settling) {
    posture_settle_dwell_s_ += settle_dt;
  } else if (!height_endpoint_settling) {
    posture_settle_dwell_s_ = 0.0F;
  }
  // The GUI normally queues posture commands until this capture state is
  // quiet.  Keep the same guard in the portable core: an ESP32/serial caller
  // can send a valid request before the IMU and wheel estimator have settled,
  // and starting then would turn the capture transient into an unintended
  // translation.  A valid request waits; only an infeasible FK target is
  // rejected.
  const bool posture_quiet_bands =
      state_machine_.mode() == ControlMode::Balance &&
      !posture_motion_needs_station_reanchor_ &&
      station_reanchor_cooldown_s_ <= 0.0F &&
      settle_pitch_rate < 0.35F &&
      std::abs(state.wheel.velocity_m_s) < 0.12F;
  // A HEIGHT request can reuse a quiet sensor sample even during the short
  // completion bookkeeping window.  The adaptive capture distance remains
  // the authority; the cooldown is only a legacy fallback gate.
  const bool adaptive_capture_sensor_quiet =
      !posture_was_active &&
      height_station_mode && state.floor_contact &&
      settle_pitch_rate < 0.35F &&
      std::abs(state.wheel.velocity_m_s) <=
          std::max(config_.posture_settle_wheel_velocity_m_s, 0.0F);
  const bool height_posture_sensor_quiet =
      (posture_quiet_bands && (!sequence_tracked || state.floor_contact)) ||
      (sequence_tracked && adaptive_capture_sensor_quiet);
  const auto posture_start_ready = [&]() {
    // Legacy/core callers do not expose sensor sequence freshness and cannot
    // observe the native station telemetry. Preserve their synchronous API
    // contract with the same basic measured quiet bands; the native multi-rate
    // path below requires the full adaptive capture latch.
    if (!sequence_tracked) {
      return height_station_mode &&
          (!sequence_tracked || state.floor_contact) &&
          settle_pitch_rate < 0.35F &&
          std::abs(state.wheel.velocity_m_s) < 0.12F;
    }
    // Do not gate a request on station_hold_active_: that latch is itself
    // disabled by neither motion nor a pending request. Readiness must come
    // from the current sensor-derived capture state and measured quiet bands.
    // A new HEIGHT arriving on the completion edge may reuse the already
    // captured station state. The adaptive capture metric is stronger
    // evidence than waiting through another fixed re-anchor/dwell window.
    return station_hold_active_ && height_posture_sensor_quiet &&
        (!yaw_only_posture_command || station_capture_ready_) &&
        posture_settle_dwell_s_ >=
            std::max(config_.posture_settle_dwell_s, 0.0F);
  };
  if (!input.request_height_change) {
    height_motion_rejected_ = false;
    height_motion_pending_ = false;
    height_pending_anchor_captured_ = false;
    height_pending_elapsed_s_ = 0.0F;
    height_capture_pause_elapsed_s_ = 0.0F;
    height_rejected_target_valid_ = false;
  }
  // The native IPC request remains asserted for one tick after the path
  // reaches its endpoint so the runner can retire the transaction.  Do not
  // interpret that held, already-consumed target as a new path merely because
  // height_motion_active_ is now false; doing so restarts the same transition
  // forever and makes the public HEIGHT transaction never complete.  A
  // genuinely different target still retargets both an idle and an active
  // path.
  const bool same_rejected_height = input.request_height_change &&
      height_motion_rejected_ && height_rejected_target_valid_ &&
      std::abs(input.target_body_height_m - height_rejected_target_m_) <= 1.0e-4F;
  const bool height_target_changed = input.request_height_change &&
      !same_rejected_height &&
      std::abs(input.target_body_height_m - height_motion_target_m_) > 1.0e-4F;
  const bool retry_pending_height = input.request_height_change &&
      !same_rejected_height && height_motion_pending_ &&
      !height_motion_active_;
  // A slider is a stream of new targets, not a sequence of independent pose
  // transactions.  While a path is already running, retarget it from the
  // last emitted joint pose immediately; waiting for the stationary gate here
  // would leave the old path active while also arming a second pending path.
  const bool active_height_retarget = height_motion_active_ &&
      height_target_changed;
  const bool settled_height_retarget = height_motion_endpoint_reached_ &&
      height_target_changed;
  if (height_target_changed || retry_pending_height) {
    if (height_target_changed) {
      height_motion_rejected_ = false;
      height_motion_pending_ = false;
      height_pending_anchor_captured_ = false;
      height_motion_endpoint_reached_ = false;
      height_motion_settled_ = false;
      height_pending_elapsed_s_ = 0.0F;
      height_rejected_target_valid_ = false;
    }
    // A height request owns the posture path.  If the operator moves the
    // slider while a transaction is active, retarget from the last pose that
    // was actually emitted to the motor layer, not from a noisy/asymmetric
    // encoder sample and not from the beginning of the old path.
    if (!height_motion_active_) {
      joint_motion_.reset(state.joints.position_rad);
    }
    const auto left = WholeBodyModel::legHeight(
        state.joints.position_rad[left_hip], state.joints.position_rad[left_knee]);
    const auto right = WholeBodyModel::legHeight(
        state.joints.position_rad[right_hip], state.joints.position_rad[right_knee]);
    float current_height = geometry::wheel_radius_m - geometry::origins[0].z -
        0.5F * (left.height_m + right.height_m);
    float seed_hip = 0.5F *
        (state.joints.position_rad[left_hip] + state.joints.position_rad[right_hip]);
    float seed_knee = 0.5F *
        (state.joints.position_rad[left_knee] + state.joints.position_rad[right_knee]);
    if (height_motion_active_) {
      seed_hip = height_motion_last_hip_rad_;
      seed_knee = height_motion_last_knee_rad_;
      const auto planned = WholeBodyModel::legHeight(seed_hip, seed_knee);
      current_height = geometry::wheel_radius_m - geometry::origins[0].z -
          planned.height_m;
    }
    // Solve each named height on the canonical CAD branch, not from the
    // previous endpoint.  Seeding natural from deep squat otherwise keeps
    // Newton on the nearby deep solution and reports a natural target while
    // leaving the knees visually deep.
    const auto target = WholeBodyModel::solveSymmetricHeight(
        input.target_body_height_m, geometry::neutral_hip_rad,
        geometry::neutral_knee_rad,
        config_.hip_min_rad, config_.hip_max_rad,
        config_.knee_min_rad, config_.knee_max_rad);
    const float pending_timeout = std::max(config_.height_pending_timeout_s, 0.0F);
    const bool pending_timeout_reached = height_motion_pending_ &&
        pending_timeout > 0.0F &&
        height_pending_elapsed_s_ >= pending_timeout;
    const bool regular_start_ready = posture_start_ready();
    const bool fallback_sensor_safe =
        height_station_mode &&
        (!sequence_tracked || state.floor_contact) &&
        std::abs(state.imu.pitch_rate_rad_s) < 0.65F &&
        std::abs(state.wheel.velocity_m_s) <=
            std::max(config_.posture_settle_wheel_velocity_m_s, 0.0F) &&
        station_capture_ready_;
    const bool fallback_start_ready = pending_timeout_reached &&
        fallback_sensor_safe;
    const bool height_start_gate = active_height_retarget ||
        settled_height_retarget || regular_start_ready || fallback_start_ready;
    const bool height_already_satisfied = !height_motion_active_ &&
        std::abs(input.target_body_height_m - current_height) <=
            // Endpoint load sag may be several millimetres in the fake
            // driver, but that must not turn a real deep-squat request into a
            // no-op. Keep the request-side deadband tight and reserve the
            // larger configured tolerance for endpoint settling.
            std::min(std::max(config_.height_reached_tolerance_m, 0.0F),
                     0.0025F);
    if (target.feasible && std::isfinite(current_height) &&
        height_start_gate && height_already_satisfied) {
      height_motion_active_ = false;
      height_motion_pending_ = false;
      height_motion_endpoint_reached_ = false;
      height_motion_settled_ = true;
      height_motion_rejected_ = false;
      height_motion_target_m_ = input.target_body_height_m;
      height_motion_start_m_ = current_height;
      height_motion_elapsed_s_ = 0.0F;
      height_motion_duration_s_ = 0.0F;
      height_pending_anchor_captured_ = false;
      height_pending_elapsed_s_ = 0.0F;
    } else if (target.feasible && std::isfinite(current_height) &&
               height_start_gate) {
      // A new height transaction starts from the current encoder odometry
      // origin. Reusing the previous transaction's anchor turns a harmless
      // posture request into a command to pull the robot back across the
      // floor before the legs have even moved.
      if (!height_motion_active_ || settled_height_retarget) {
        // A posture transaction owns a new local support point.  Reusing the
        // previous station anchor would pull the wheels back through the
        // displacement accumulated while the capture transient was settling.
        lqr_.reanchorPosition(state.wheel.position_m);
        // The previous station/drive catch is no longer a valid feed-forward
        // reference for this new geometry.  Seed the station reference from
        // the measured wheel velocity so the handoff has no velocity step.
        lqr_.clearForwardVelocityReference();
        lqr_.setStationVelocityReference(state.wheel.velocity_m_s);
      }
      height_motion_active_ = true;
      height_motion_endpoint_reached_ = false;
      height_motion_settled_ = false;
      height_capture_recovery_active_ = false;
      height_motion_pending_ = false;
      height_pending_anchor_captured_ = false;
      height_pending_elapsed_s_ = 0.0F;
      height_motion_start_m_ = current_height;
      height_motion_target_m_ = input.target_body_height_m;
      height_motion_elapsed_s_ = 0.0F;
      height_capture_pause_elapsed_s_ = 0.0F;
      height_motion_last_hip_rad_ = seed_hip;
      height_motion_last_knee_rad_ = seed_knee;
      const float delta = std::abs(height_motion_target_m_ -
                                    height_motion_start_m_);
      const bool lowering = height_motion_target_m_ < height_motion_start_m_;
      const float configured_rate = lowering
          ? config_.height_lowering_rate_m_s
          : config_.height_max_rate_m_s;
      const float rate = std::max(configured_rate, 1.0e-4F);
      height_motion_duration_s_ = std::max(
          config_.height_min_duration_s, delta / rate);
      if (fallback_start_ready && !regular_start_ready &&
          !active_height_retarget) {
        // The measured state is safe but the long dwell/capture gate did not
        // qualify in time. Preserve smoothness by reducing the task rate;
        // never bypass the support and velocity checks above.
        const float scale = std::clamp(
            config_.height_pending_fallback_rate_scale, 0.1F, 1.0F);
        height_motion_duration_s_ = std::max(
            height_motion_duration_s_, delta / std::max(rate * scale, 1.0e-4F));
      }
      if (config_.height_max_duration_s > 0.0F) {
        height_motion_duration_s_ = std::min(
            height_motion_duration_s_, config_.height_max_duration_s);
      }

      // Precompute a C2 joint path once.  Start from the last actuator target
      // when it is valid, rather than replacing the gravity-supporting target
      // with a sagged encoder sample.  The live encoders still seed the
      // inverse-FK feasibility check; this preserves the loaded actuator
      // preload through the ownership handoff.
      height_path_[0] = state.joints.position_rad;
      if (last_output_.enable_motors) {
        bool command_seed_valid = true;
        for (const float value : last_output_.position_rad) {
          command_seed_valid = command_seed_valid && std::isfinite(value);
        }
        if (command_seed_valid) {
          height_path_[0] = last_output_.position_rad;
          const auto& held_correction = whole_body_stance_.correction();
          for (const auto joint : {left_hip, left_knee, right_hip, right_knee}) {
            height_path_[0][joint] -= held_correction[joint];
          }
        }
      }
      // The actuator target may be above/below the loaded encoder pose.  The
      // task trajectory must start at the measured body height; using the
      // stale target height here makes a nearly-complete request replay a
      // second squat and injects an avoidable support impulse.
      const float path_start_height = current_height;
      // The actuator path is seeded from the last command, while the encoder
      // can still lag that command during a loaded transition.  Recompute the
      // transaction duration from the same emitted pose; otherwise the
      // planner can retire early and jump to the endpoint before the motors
      // have followed the path.
      if (std::isfinite(path_start_height)) {
        height_motion_start_m_ = path_start_height;
        const float path_delta = std::abs(
            height_motion_target_m_ - height_motion_start_m_);
        const bool path_lowering =
            height_motion_target_m_ < height_motion_start_m_;
        const float path_rate = std::max(
            path_lowering ? config_.height_lowering_rate_m_s
                          : config_.height_max_rate_m_s,
            1.0e-4F);
        height_motion_duration_s_ = std::max(
            config_.height_min_duration_s, path_delta / path_rate);
        if (config_.height_max_duration_s > 0.0F) {
          height_motion_duration_s_ = std::min(
              height_motion_duration_s_, config_.height_max_duration_s);
        }
      }
      const auto& held_correction = whole_body_stance_.correction();
      float solve_hip = height_path_[0][left_hip];
      float solve_knee = height_path_[0][left_knee];
      for (std::size_t i = 1; i < kHeightPathSamples; ++i) {
        const float path_ratio = static_cast<float>(i) /
            static_cast<float>(kHeightPathSamples - 1);
        const float blend = quinticBlend(path_ratio);
        height_path_[i] = height_path_[0];
        // Solve every intermediate sample in task space so extension and
        // lowering preserve the axle fore/aft coordinate alike.  Near the
        // straight branch the continuation seed and bounded fallback keep a
        // marginal Newton sample from producing an invalid actuator target.
        const float sample_height = path_start_height + blend *
            (height_motion_target_m_ - path_start_height);
        const auto sample = WholeBodyModel::solveSymmetricHeight(
            sample_height, solve_hip, solve_knee, config_.hip_min_rad,
            config_.hip_max_rad, config_.knee_min_rad,
            config_.knee_max_rad);
        if (sample.feasible) {
          solve_hip = sample.hip_rad;
          solve_knee = sample.knee_rad;
        } else {
          // The endpoint was already validated above. Keep a bounded
          // fallback for a numerically marginal intermediate sample rather
          // than emitting an uninitialised actuator target.
          solve_hip = height_path_[0][left_hip] + blend *
              (target.hip_rad - height_path_[0][left_hip]);
          solve_knee = height_path_[0][left_knee] + blend *
              (target.knee_rad - height_path_[0][left_knee]);
        }
        if (i == kHeightPathSamples - 1) {
          solve_hip = target.hip_rad;
          solve_knee = target.knee_rad;
        }
        // The first emitted sample must be exactly the actuator-held pose.
        // The task solve is symmetric, while the measured/held pose can have
        // a small left-right and COM bias; assigning it directly at i=1
        // creates a hidden joint step before the quintic path has moved.
        height_path_[i][left_hip] = height_path_[0][left_hip] + blend *
            (solve_hip - held_correction[left_hip] -
             height_path_[0][left_hip]);
        height_path_[i][left_knee] = height_path_[0][left_knee] + blend *
            (solve_knee - held_correction[left_knee] -
             height_path_[0][left_knee]);
        height_path_[i][right_hip] = height_path_[0][right_hip] + blend *
            (solve_hip - held_correction[right_hip] -
             height_path_[0][right_hip]);
        height_path_[i][right_knee] = height_path_[0][right_knee] + blend *
            (solve_knee - held_correction[right_knee] -
             height_path_[0][right_knee]);
        height_path_[i][left_wheel] = 0.0F;
        height_path_[i][right_wheel] = 0.0F;
      }
    } else if (!target.feasible || !std::isfinite(current_height)) {
      height_motion_rejected_ = true;
      height_rejected_target_valid_ = true;
      height_rejected_target_m_ = input.target_body_height_m;
    } else {
      // Preserve the IPC level request and retry on the next fresh sensor
      // packet. Capture the station anchor once; repeatedly re-anchoring here
      // would recreate the deadlock by making position error identically zero.
      height_motion_pending_ = true;
      // Mark this target as the pending transaction identity. Without this,
      // every fresh sample sees target != the last completed target, clears the
      // pending timer, and recreates an infinite wait before timeout can fire.
      height_motion_target_m_ = input.target_body_height_m;
      if (!height_pending_anchor_captured_) {
        if (!config_.lqr.position_anchor_hold) {
          lqr_.reanchorPosition(state.wheel.position_m);
        }
        height_pending_anchor_captured_ = true;
        height_pending_elapsed_s_ = 0.0F;
      }
      if (height_target_changed) {
        height_pending_pose_ = last_output_.enable_motors
            ? last_output_.position_rad : state.joints.position_rad;
        const auto& held_correction = whole_body_stance_.correction();
        for (const auto joint : {left_hip, left_knee, right_hip, right_knee}) {
          height_pending_pose_[joint] -= held_correction[joint];
        }
      }
    }
  }

  if (height_motion_pending_ && !height_motion_active_) {
    height_pending_elapsed_s_ += std::max(state.dt_s, 0.0F);
    const float timeout = std::max(config_.height_pending_timeout_s, 0.0F);
    const float abort_timeout = std::max(
        config_.height_pending_abort_timeout_s, timeout + 1.0F);
    if (abort_timeout > 0.0F && height_pending_elapsed_s_ >= abort_timeout) {
      // Hand the leg target back continuously - see posture_abort_blend_s.
      // Capture the BASE pose, not the final command: last_output_ already
      // carries the stance controller's correction, and this value is used
      // as the base that the same correction is added to again. Capturing
      // the sum would apply it twice and step by exactly that much, which is
      // the trap the runner-side rejection guard also fell into.
      if (config_.posture_abort_blend_s > 0.0F && last_output_.enable_motors) {
        const auto& held = whole_body_stance_.correction();
        posture_abort_pose_ = last_output_.position_rad;
        for (const auto joint : {left_hip, left_knee, right_hip, right_knee}) {
          posture_abort_pose_[joint] -= held[joint];
        }
        posture_abort_blend_ = 1.0F;
      }
      height_motion_pending_ = false;
      height_pending_anchor_captured_ = false;
      height_motion_rejected_ = true;
      height_rejected_target_valid_ = true;
      height_rejected_target_m_ = input.target_body_height_m;
      height_pending_elapsed_s_ = 0.0F;
    }
  }

  const float capture_margin = std::max(config_.station_capture_margin_m, 1.0e-4F);
  // Capture loss slows the task below; it is not completion. Keeping ownership
  // with the active path lets the wheel MPC arrest measured motion while the
  // same C2 trajectory continues from its current phase.

  if (height_motion_active_) {
    float motion_dt = std::max(state.dt_s, 0.0F);
    bool capture_aborted = false;
    const bool lowering = height_motion_target_m_ < height_motion_start_m_;
    // Height is a task-space command, but the same metres-per-radian dof
    // becomes poorly conditioned as the leg approaches its straight branch.
    // Pace from the analytic CAD Jacobian instead of using one rate that is
    // safe only for the neutral pose.  The lower bound keeps a valid request
    // from getting numerically stuck at a singular sample; capture-distance
    // pacing above still has authority to pause it completely.
    const auto live_leg = WholeBodyModel::legHeight(
        height_motion_last_hip_rad_, height_motion_last_knee_rad_);
    const auto neutral_leg = WholeBodyModel::legHeight(
        geometry::neutral_hip_rad, geometry::neutral_knee_rad);
    const float live_det = live_leg.d_x_d_hip * live_leg.d_height_d_knee -
        live_leg.d_x_d_knee * live_leg.d_height_d_hip;
    const float neutral_det = neutral_leg.d_x_d_hip *
        neutral_leg.d_height_d_knee - neutral_leg.d_x_d_knee *
        neutral_leg.d_height_d_hip;
    if (std::isfinite(live_det) && std::isfinite(neutral_det) &&
        std::abs(neutral_det) > 1.0e-6F) {
      const float conditioning = std::clamp(
          std::abs(live_det) / std::abs(neutral_det), 0.0F, 1.0F);
      if (lowering) {
        // The determinant is already a two-axis conditioning measure. Squaring
        // it double-penalizes deep squat and turns a valid lowering command
        // into a very long transition. Use its square root as the adaptive
        // rate fraction.
        motion_dt *= std::sqrt(std::max(conditioning, 1.0e-6F));
      } else {
        // Retain the more conservative extension schedule that has cleared
        // the straight-leg native envelope; the extension branch approaches
        // the singularity from the other side and needs that extra damping.
        motion_dt *= std::clamp(conditioning * conditioning, 0.25F, 1.0F);
      }
    }
    // A height path may let the wheels catch pitch, but it must stop adding
    // leg motion once the support point itself has left the drift envelope.
    // Use the larger physical-capture/anchor metric so a quiet pitch signal
    // cannot hide a 2 cm translation from this supervisor.
    const float physical_capture_distance = std::isfinite(
        posture_capture_distance_m_) ? posture_capture_distance_m_ : 0.0F;
    const float height_capture_distance = std::max(
        physical_capture_distance, station_capture_distance_m_);
    const float capture_pause_distance = std::max(
        config_.height_capture_pause_distance_m, 0.0F);
    const bool capture_paused = capture_pause_distance > 0.0F &&
        height_capture_distance >= capture_pause_distance;
    const auto left_live_leg = WholeBodyModel::legHeight(
        state.joints.position_rad[left_hip],
        state.joints.position_rad[left_knee]);
    const auto right_live_leg = WholeBodyModel::legHeight(
        state.joints.position_rad[right_hip],
        state.joints.position_rad[right_knee]);
    const float measured_live_height = geometry::wheel_radius_m - 0.5F *
        (left_live_leg.height_m + right_live_leg.height_m);
    const bool near_height_endpoint = std::isfinite(measured_live_height) &&
        std::abs(measured_live_height - height_motion_target_m_) <=
            std::max(config_.height_reached_tolerance_m, 0.0025F);
    if (capture_paused) {
      // Hold the current C2 pose. The wheel MPC is the only recovery owner;
      // resuming the leg path while the anchor error is growing merely feeds
      // more energy into the same inverted-pendulum transient.
      // The abort timeout is wall-clock sensor time, not task progress.  The
      // latter is deliberately slowed by conditioning/capture scheduling and
      // would let a stalled transaction live indefinitely.
      if (near_height_endpoint) {
        // The measured FK height is already inside the endpoint tolerance.
        // Finish the transaction and let the endpoint dwell wait for capture;
        // rejecting here made max -> natural/deep look like a failed button
        // press even though the legs had reached the requested pose.
        capture_aborted = true;
        height_motion_active_ = false;
        height_motion_endpoint_reached_ = true;
        height_motion_settled_ = false;
        height_motion_pending_ = false;
        height_pending_pose_ = last_output_.enable_motors
            ? last_output_.position_rad : state.joints.position_rad;
        const auto& held = whole_body_stance_.correction();
        for (const auto joint : {left_hip, left_knee, right_hip, right_knee}) {
          height_pending_pose_[joint] -= held[joint];
        }
        height_motion_endpoint_anchor_m_ = state.wheel.position_m;
        station_dwell_s_ = 0.0F;
        station_capture_filter_initialized_ = false;
        height_capture_pause_elapsed_s_ = 0.0F;
      } else {
        height_capture_pause_elapsed_s_ += std::max(state.dt_s, 0.0F);
        const float abort_after = std::max(config_.height_capture_abort_s, 0.0F);
        if (abort_after > 0.0F &&
            height_capture_pause_elapsed_s_ >= abort_after) {
        // A paused path that never recaptures is no longer a smooth
        // transition; it is a stale posture owner. Retire it at the last
        // actuator target and let the caller retry from that safe pose.
          capture_aborted = true;
          height_motion_active_ = false;
          height_motion_endpoint_reached_ = false;
          height_motion_settled_ = false;
          height_capture_recovery_active_ = true;
          height_motion_pending_ = false;
          height_pending_anchor_captured_ = false;
          height_pending_elapsed_s_ = 0.0F;
          height_motion_rejected_ = true;
          height_rejected_target_valid_ = true;
          height_rejected_target_m_ = height_motion_target_m_;
          height_capture_pause_elapsed_s_ = 0.0F;
          // Keep the handoff continuous when capture never returns.  The last
          // output already includes stance correction; store the base pose so
          // the correction is not applied twice on the next control pass.
          if (config_.posture_abort_blend_s > 0.0F && last_output_.enable_motors) {
            const auto& held = whole_body_stance_.correction();
            posture_abort_pose_ = last_output_.position_rad;
            for (const auto joint : {left_hip, left_knee, right_hip, right_knee}) {
              posture_abort_pose_[joint] -= held[joint];
            }
            posture_abort_blend_ = 1.0F;
            command.target_joint_position_rad = posture_abort_pose_;
          } else {
            command.target_joint_position_rad = last_output_.enable_motors
                ? last_output_.position_rad : state.joints.position_rad;
          }
        }
      }
      motion_dt = 0.0F;
    } else if (height_capture_distance > capture_margin) {
      height_capture_pause_elapsed_s_ = 0.0F;
      // Keep the task C2-continuous while reducing its effective speed. This
      // is based on the same sensor-only capture metric used by station hold.
      const float capture_ratio = std::clamp(
          capture_margin /
              std::max(height_capture_distance, capture_margin),
          0.0F, 1.0F);
      const float scale = std::clamp(
          capture_ratio * capture_ratio,
          std::clamp(config_.height_capture_slowdown_min_scale, 0.02F, 1.0F),
          1.0F);
      motion_dt *= scale;
    } else {
      height_capture_pause_elapsed_s_ = 0.0F;
    }
    // Adaptive lowering pacing must not make a valid target appear stuck
    // forever. Use the configured transaction ceiling as a deadline-derived
    // floor. Extension keeps its validated conditioning schedule.
    if (!capture_paused && lowering && motion_dt > 0.0F &&
        config_.height_max_duration_s > 0.0F) {
      const float deadline_scale = std::clamp(
          height_motion_duration_s_ /
              std::max(config_.height_max_duration_s, height_motion_duration_s_),
          0.0F, 1.0F);
      motion_dt = std::max(motion_dt, std::max(state.dt_s, 0.0F) * deadline_scale);
    }
    if (!capture_aborted) {
      height_motion_elapsed_s_ += motion_dt;
      const float ratio = std::clamp(
          height_motion_elapsed_s_ / std::max(height_motion_duration_s_, 1.0e-3F),
          0.0F, 1.0F);
      const float path_position = quinticBlend(ratio) *
          static_cast<float>(kHeightPathSamples - 1);
      const auto lower = static_cast<std::size_t>(std::min(
          path_position, static_cast<float>(kHeightPathSamples - 2)));
      const float blend = path_position - static_cast<float>(lower);
      JointArray pose{};
      for (std::size_t i = 0; i < pose.size(); ++i) {
        pose[i] = height_path_[lower][i] + blend *
            (height_path_[lower + 1][i] - height_path_[lower][i]);
      }
      // Let the legs absorb a bounded support-point displacement while the
      // height path is active.  The wheel anchor error is a translation task,
      // not a pitch reference; solve it orthogonally to height so the capture
      // correction does not create another squat/stand disturbance.
      const float station_error = state.wheel.position_m -
          lqr_.debugState().target_ground_position_m;
      const float desired_axle_correction = std::clamp(
          0.60F * station_error, -0.018F, 0.018F);
      const auto apply_axle_capture = [&](std::size_t hip,
                                           std::size_t knee) {
        const auto leg = WholeBodyModel::legHeight(pose[hip], pose[knee]);
        const float determinant = leg.d_x_d_hip * leg.d_height_d_knee -
            leg.d_x_d_knee * leg.d_height_d_hip;
        if (!std::isfinite(determinant) || std::abs(determinant) < 1.0e-6F) {
          return;
        }
        const float delta_hip = desired_axle_correction *
            leg.d_height_d_knee / determinant;
        const float delta_knee = -desired_axle_correction *
            leg.d_height_d_hip / determinant;
        const float max_step = 0.35F * std::max(state.dt_s, 0.0F);
        pose[hip] += std::clamp(delta_hip, -max_step, max_step);
        pose[knee] += std::clamp(delta_knee, -max_step, max_step);
      };
      apply_axle_capture(left_hip, left_knee);
      apply_axle_capture(right_hip, right_knee);
      height_motion_last_hip_rad_ = pose[left_hip];
      height_motion_last_knee_rad_ = pose[left_knee];
      command.target_joint_position_rad = pose;
      {
      // The posture target is updated in task space every sensor sample; do
      // not also start the legacy joint-space planner for this transaction.
      command.request_pose_change = false;
      if (ratio >= 1.0F) {
        if (!height_motion_endpoint_reached_) {
          // Capture the endpoint anchor for the station handoff. The height
          // transaction itself retires at the C2 endpoint; the controller's
          // separate re-anchor cooldown keeps the estimator/MPC handoff
          // bounded without holding the IPC request forever.
          height_motion_endpoint_reached_ = true;
          height_motion_endpoint_anchor_m_ = state.wheel.position_m;
          station_dwell_s_ = 0.0F;
          station_capture_filter_initialized_ = false;
        }
        height_pending_pose_ = pose;
        height_motion_active_ = false;
        // The C2 path has reached its target, but the loaded fake driver can
        // still move the support point while the final joint target settles.
        // Keep the endpoint in the settle phase so the station re-anchor is
        // taken from the post-sag encoder position, not from this early edge.
        height_motion_settled_ = false;
        posture_settle_dwell_s_ = 0.0F;
      }
      }
    }
  }
  if (height_motion_endpoint_reached_ && !height_motion_settled_ &&
      !height_motion_active_) {
    const auto left_endpoint_leg = WholeBodyModel::legHeight(
        state.joints.position_rad[left_hip],
        state.joints.position_rad[left_knee]);
    const auto right_endpoint_leg = WholeBodyModel::legHeight(
        state.joints.position_rad[right_hip],
        state.joints.position_rad[right_knee]);
    const float measured_height = geometry::wheel_radius_m - 0.5F *
        (left_endpoint_leg.height_m + right_endpoint_leg.height_m);
    const float joint_rate = std::max({
        std::abs(state.joints.velocity_rad_s[left_hip]),
        std::abs(state.joints.velocity_rad_s[left_knee]),
        std::abs(state.joints.velocity_rad_s[right_hip]),
        std::abs(state.joints.velocity_rad_s[right_knee])});
    const float endpoint_capture_tolerance = std::max(
        std::max(config_.station_capture_margin_m, 0.0F),
        std::max(config_.height_capture_pause_distance_m, 0.0F));
    const bool endpoint_quiet = std::isfinite(measured_height) &&
        // The C2 task path has already reached its requested FK endpoint.
        // Do not re-gate retirement on loaded encoder sag here; that sag is
        // exactly what the native endpoint tolerance models and otherwise
        // leaves HEIGHT asserted indefinitely while the pose is already held.
        joint_rate <= std::max(
            2.0F * std::max(config_.posture_settle_pitch_rate_rad_s, 0.0F),
            0.12F) &&
        station_capture_distance_m_ <= endpoint_capture_tolerance;
    if (endpoint_quiet) {
      posture_settle_dwell_s_ += std::max(state.dt_s, 0.0F);
    } else {
      posture_settle_dwell_s_ = 0.0F;
    }
    if (posture_settle_dwell_s_ >=
        std::max(config_.posture_settle_dwell_s, 0.0F)) {
      height_motion_settled_ = true;
      posture_settle_dwell_s_ = 0.0F;
    }
    // Keep the final C2 pose on the actuator path until the physical endpoint
    // is quiet; the IPC request retires only after this dwell.
    command.target_joint_position_rad = height_pending_pose_;
    command.request_pose_change = false;
  }
  if (height_motion_pending_ && !height_motion_active_ &&
      last_output_.enable_motors) {
    // A HEIGHT request may be accepted at the IPC boundary before the
    // adaptive station-capture gate is ready.  The runner has already stored
    // the requested endpoint in command.target_joint_position_rad, but that
    // endpoint is not allowed to reach the actuator owner while the request
    // is pending; doing so bypasses the smooth height path and caused the
    // deep->shallow transition to jump directly to its new leg pose.
    command.target_joint_position_rad = height_pending_pose_;
  }
  command.target_joint_position_rad = standUpPose(command.target_joint_position_rad);
  if (command.request_pose_change && !joint_motion_.active() &&
      stand_up_stage_ == StandUpStage::Idle) {
    if (pose_interrupt || posture_start_ready()) {
      float pose_duration_s = config_.pose_change_duration_s;
      if (config_.pose_max_joint_velocity_rad_s > 0.0F) {
        float largest_delta_rad = 0.0F;
        for (std::size_t i = 0; i < kJointCount; ++i) {
          largest_delta_rad = std::max(
              largest_delta_rad,
              std::abs(command.target_joint_position_rad[i] -
                       state.joints.position_rad[i]));
        }
        // smoothStep's peak velocity is 1.5 * displacement / duration.
        pose_duration_s = std::max(
            pose_duration_s,
            1.5F * largest_delta_rad / config_.pose_max_joint_velocity_rad_s);
        if (config_.pose_max_duration_s > 0.0F) {
          pose_duration_s = std::min(pose_duration_s,
                                     config_.pose_max_duration_s);
        }
      }
      JointArray pose_start = state.joints.position_rad;
      if (pose_interrupt && last_output_.enable_motors) {
        pose_start = last_output_.position_rad;
        const auto& held = whole_body_stance_.correction();
        for (const auto joint : {left_hip, left_knee, right_hip, right_knee}) {
          pose_start[joint] -= held[joint];
        }
      }
      joint_motion_.startMove(pose_start,
                              command.target_joint_position_rad,
                              pose_duration_s);
    } else if (last_output_.enable_motors) {
      command.target_joint_position_rad = last_output_.position_rad;
    }
  }
  const bool posture_is_active =
      height_motion_active_ || joint_motion_.active() ||
      (height_motion_endpoint_reached_ && !height_motion_settled_);
  if (posture_was_active && !posture_is_active) {
    posture_motion_needs_station_reanchor_ = true;
  }

  const ControlMode previous_mode = state_machine_.mode();
  if (config_.fixture_hold_slew_rad_s > 0.0F &&
      previous_mode == ControlMode::FixtureHold && !fixture_hold_arrived_) {
    // Do not release onto legs that are still folding toward the stance.
    command.request_fixture_release = false;
  }
  last_flight_estimate_ = flight_observer_.update(state);
  state.floor_contact = last_flight_estimate_.support_contact;
  state.left_floor_contact = last_flight_estimate_.left_support_contact;
  state.right_floor_contact = last_flight_estimate_.right_support_contact;
  // HEIGHT uses the task-space path directly and therefore does not start the
  // legacy JointMotionPlanner.  It is still a posture motion: the state
  // machine and IMU adapter must see the same dynamic-motion gate while the
  // legs are extending, otherwise a transient accelerometer/gyro excursion is
  // interpreted as a stationary Balance fall.
  const bool pose_motion_active = height_motion_active_ ||
      joint_motion_.active();
  ControlMode mode = state_machine_.update(
      state, command, pose_motion_active, standingUp(),
      last_flight_estimate_.airborne || last_flight_estimate_.support_settling);
  // Keep the sensor-derived fall latch authoritative even when a caller
  // supplies a custom StateMachine configuration. This duplicates no sensor
  // source: it is the same MPU6500 pitch and the same airborne hypothesis
  // already passed to StateMachine::update(). It prevents a high-rate wrapper
  // from continuing to label a clearly tipped, supported body as Drive.
  const bool balance_mode = mode == ControlMode::Balance ||
      mode == ControlMode::Drive || mode == ControlMode::PoseChange;
  if (balance_mode &&
      std::abs(state.imu.pitch_rad) > config_.state_machine.fall_pitch_rad &&
      !last_flight_estimate_.airborne &&
      !last_flight_estimate_.support_settling) {
    state_machine_.forceFallen();
    mode = state_machine_.mode();
  }
  if (usesBalance(mode) && !usesBalance(previous_mode)) {
    setBalanceFallPitch(config_.lqr.fall_pitch_rad);
    resetBalance(state.wheel.position_m, state.floor_contact);
    // Only a fixture release is an operator-chosen balance point; a
    // StandingUp/recovery entry happens at whatever pitch the rise left.
    if (config_.seed_balance_trim_from_pitch && state.floor_contact &&
        previous_mode == ControlMode::FixtureHold) {
      lqr_.setBalanceTrim(state.imu.pitch_rad - balance_pitch_ref_rad_);
    }
    // Fresh re-entry into Balance/Drive/PoseChange (e.g. after Fallen or
    // StandingUp) must not replay a stale latched airborne/catch offset from
    // before the mode left balancing.
    flight_observer_.reset(state);
    last_flight_estimate_ = FlightObserver::Output{};
  }

  MotorCommand output{};

  if (mode == ControlMode::StandingUp) {
    if (stand_up_stage_ == StandUpStage::Idle) {
      beginStandUp(state);
    }
    advanceStandUp(state);
    if (state_machine_.mode() == ControlMode::Fallen) {
      writeSafeStop(output, state);
      return finish(output);
    }
    JointArray joint_target = joint_motion_.update(state.dt_s);
    // Tuck and Rise are one recovery transaction. Moving the legs while
    // forcing the wheels to zero removes the only actuator that can catch the
    // pitch generated by the leg trajectory, so both stages use the same
    // wheel-balance owner with the joint trajectory as its position task.
    ControlCommand recovery_command = command;
    recovery_command.target_forward_velocity_m_s = 0.0F;
    recovery_command.target_yaw_rate_rad_s = 0.0F;
    const bool wheel_capture_window = state.floor_contact &&
        std::abs(state.imu.pitch_rad) <=
            std::max(config_.stand_up.wheel_balance_pitch_rad, 0.0F) &&
        std::abs(state.imu.pitch_rate_rad_s) <=
            std::max(config_.stand_up.wheel_balance_pitch_rate_rad_s, 0.0F);
    LqrBalancer::Output balance{};
    if (wheel_capture_window) {
      balance = runBalance(state, recovery_command, true);
    } else {
      // Keep the LQR reference decaying deterministically, but do not
      // publish its stale wheel velocity while the body is outside the
      // supported recovery branch. The leg trajectory remains live.
      (void)runBalance(state, recovery_command, false);
    }
    if (wheel_capture_window && balance.fallen) {
      finishStandUp(state);
      state_machine_.abortStandUp();
      writeSafeStop(output, state);
      return finish(output);
    }
    // The trajectory owns the leg position target; the stance controller only
    // clamps it here. Recompute from live state on every recovery tick.
    last_whole_body_stance_ = whole_body_stance_.update(
        state, joint_target, last_whole_body_, false, state.dt_s, false);
    joint_target = last_whole_body_stance_.target;
    writeBalancedCommand(output, balance, joint_target);
    return finish(output);
  }

  // Left the recovery for any other reason (done, tipped, e-stop): make sure
  // the balance controller is clean before it runs again.
  if (stand_up_stage_ != StandUpStage::Idle) {
    finishStandUp(state);
  }

  if (mode == ControlMode::Enabling) {
    resetBalance(state.wheel.position_m, state.floor_contact);
    writePostureHold(output, command.target_joint_position_rad, state);
    return finish(output);
  }

  if (mode == ControlMode::SensorQualify) {
    // No actuator torque while the fixture and IMU are being qualified.
    writeSafeStop(output, state);
    return finish(output);
  }

  if (mode == ControlMode::FixtureHold && config_.fixture_hold_slew_rad_s > 0.0F) {
    // Body held up in the air: bring the legs from the entry pose to the
    // stance target at a bounded joint speed.
    resetBalance(state.wheel.position_m, state.floor_contact);
    constexpr std::array<std::array<std::size_t, 2>, 2> kLegPairs{{
        {left_hip, left_knee}, {right_hip, right_knee}}};
    const auto distanceTo = [](const std::array<std::size_t, 2>& pair,
                               const JointArray& from, const JointArray& to) {
      return std::max(std::abs(to[pair[0]] - from[pair[0]]),
                      std::abs(to[pair[1]] - from[pair[1]]));
    };
    if (previous_mode != ControlMode::FixtureHold) {
      fixture_hold_start_ = state.joints.position_rad;
      fixture_hold_target_ = state.joints.position_rad;
      fixture_hold_elapsed_s_ = 0.0F;
      fixture_hold_arrived_ = false;
      fixture_hold_fade_done_ = false;
      for (std::size_t leg = 0; leg < 2U; ++leg) {
        fixture_hold_leg_elapsed_s_[leg] = 0.0F;
        fixture_hold_leg_on_via_[leg] = config_.fixture_hold_via_enabled &&
            distanceTo(kLegPairs[leg], fixture_hold_start_, config_.fixture_hold_via_rad) > 0.02F &&
            distanceTo(kLegPairs[leg], fixture_hold_start_,
                       command.target_joint_position_rad) > 0.10F;
      }
    }
    const float dt = std::max(state.dt_s, 0.0F);
    fixture_hold_elapsed_s_ += dt;
    const float tolerance = std::max(config_.fixture_hold_arrival_tolerance_rad, 0.0F);
    bool arrived = true;
    for (std::size_t leg = 0; leg < 2U; ++leg) {
      const auto& pair = kLegPairs[leg];
      fixture_hold_leg_elapsed_s_[leg] += dt;
      const JointArray& goal = fixture_hold_leg_on_via_[leg]
          ? config_.fixture_hold_via_rad : command.target_joint_position_rad;
      // Minimum-jerk segment: peak speed = 1.875 * distance / duration.
      const float distance = distanceTo(pair, fixture_hold_start_, goal);
      // Already there: nothing to move, the next segment/release may follow.
      const float duration = distance < 1.0e-3F
          ? 0.0F
          : std::max(1.875F * distance / config_.fixture_hold_slew_rad_s, 0.5F);
      const float tau = duration <= 0.0F
          ? 1.0F
          : std::clamp(fixture_hold_leg_elapsed_s_[leg] / duration, 0.0F, 1.0F);
      const float blend = tau * tau * tau * (10.0F + tau * (-15.0F + 6.0F * tau));
      for (const auto joint : pair) {
        fixture_hold_target_[joint] = fixture_hold_start_[joint] +
            blend * (goal[joint] - fixture_hold_start_[joint]);
      }
      if (fixture_hold_leg_on_via_[leg] && tau >= 1.0F) {
        // This leg reached the via pose: its second segment starts there.
        fixture_hold_leg_on_via_[leg] = false;
        for (const auto joint : pair) fixture_hold_start_[joint] = fixture_hold_target_[joint];
        fixture_hold_leg_elapsed_s_[leg] = 0.0F;
      }
      arrived = arrived && !fixture_hold_leg_on_via_[leg] && tau >= 1.0F;
      for (const auto joint : pair) {
        arrived = arrived && std::abs(state.joints.position_rad[joint] -
                                      command.target_joint_position_rad[joint]) <= tolerance;
      }
    }
    // Position gain and feed-forward fade in over the first 0.5 s after entry.
    const float fade = fixture_hold_fade_done_
        ? 1.0F : std::clamp(fixture_hold_elapsed_s_ / 0.5F, 0.0F, 1.0F);
    if (fade >= 1.0F) fixture_hold_fade_done_ = true;
    fixture_hold_arrived_ = arrived;
    writePostureHold(output, fixture_hold_target_, state);
    for (const auto joint : {left_hip, left_knee, right_hip, right_knee}) {
      output.kp_scale[joint] *= fade;
    }
    // In the air the legs carry only their own weight: no body-support share.
    output.feedforward_nm.fill(0.0F);
    if (config_.whole_body_gravity_feedforward) {
      const float scale = std::clamp(
          config_.whole_body_gravity_feedforward_scale, 0.0F, 1.0F);
      const auto left = WholeBodyModel::legGravityTorque(
          state.joints.position_rad[left_hip], state.joints.position_rad[left_knee]);
      const auto right = WholeBodyModel::legGravityTorque(
          state.joints.position_rad[right_hip], state.joints.position_rad[right_knee]);
      const float ff_scale = scale * fade;
      output.feedforward_nm[left_hip] = ff_scale * left.hip_nm;
      output.feedforward_nm[left_knee] = ff_scale * left.knee_nm;
      output.feedforward_nm[right_hip] = ff_scale * right.hip_nm;
      output.feedforward_nm[right_knee] = ff_scale * right.knee_nm;
    }
    return finish(output);
  }

  if (mode == ControlMode::FixtureHold) {
    // The fixture carries the body. Hold the measured pose instead of pulling
    // toward a nominal target that may differ by a few encoder counts.
    resetBalance(state.wheel.position_m, state.floor_contact);
    writePostureHold(output, state.joints.position_rad, state);
    return finish(output);
  }

  if (mode == ControlMode::Disabled || mode == ControlMode::EmergencyStop) {
    joint_motion_.reset(state.joints.position_rad);
    height_motion_active_ = false;
    height_motion_endpoint_reached_ = false;
    height_motion_settled_ = false;
    height_capture_recovery_active_ = false;
    height_motion_pending_ = false;
    height_pending_anchor_captured_ = false;
    height_pending_elapsed_s_ = 0.0F;
    height_motion_rejected_ = false;
    height_rejected_target_valid_ = false;
    whole_body_stance_.reset();
    last_whole_body_stance_ = WholeBodyStanceController::Output{};
    writeSafeStop(output, state);
    return finish(output);
  }

  // StateMachine::update() has its OWN, separate absolute-pitch "tipped"
  // check (std::abs(state.imu.pitch_rad) > fall_pitch_rad) that forces
  // ControlMode::Fallen independently of LqrBalancer's own relative-pitch
  // check below - in the common case (pitch_ref near 0) the two trip at
  // nearly the same real pitch, so keep_legs_active_when_fallen must cover
  // THIS path too, not only the balance.fallen branch further down, or it
  // would still total-safe-stop the legs in the exact scenario it exists
  // for. Legacy behavior (false) is unchanged: total safe-stop, same as
  // Disabled/EmergencyStop above.
  if (mode == ControlMode::Fallen && !config_.keep_legs_active_when_fallen) {
    joint_motion_.reset(state.joints.position_rad);
    whole_body_stance_.reset();
    last_whole_body_stance_ = WholeBodyStanceController::Output{};
    writeSafeStop(output, state);
    return finish(output);
  }

  JointArray joint_target =
      joint_motion_.active() ? joint_motion_.update(state.dt_s)
                             : command.target_joint_position_rad;
  // Continuous handoff out of an abandoned posture transaction.
  if (posture_abort_blend_ > 0.0F) {
    if (joint_motion_.active()) {
      posture_abort_blend_ = 0.0F;
    } else {
      for (const auto joint : {left_hip, left_knee, right_hip, right_knee}) {
        joint_target[joint] = posture_abort_blend_ * posture_abort_pose_[joint] +
            (1.0F - posture_abort_blend_) * joint_target[joint];
      }
      posture_abort_blend_ = std::max(0.0F, posture_abort_blend_ -
          state.dt_s / std::max(config_.posture_abort_blend_s, 1.0e-3F));
    }
  }
  // The stance trim is downstream of pose interpolation. It cannot rewrite an
  // active height/pose transaction.
  const bool stationary_stance_request =
      std::abs(command.target_forward_velocity_m_s) < 0.01F &&
      std::abs(command.target_yaw_rate_rad_s) < 0.05F;
  // Near max stand the knee Jacobian is close to its straight-leg singularity.
  // Letting the slow COM integrator operate there turns tiny encoder/COM noise
  // into a large symmetric trim, which moves the axle and makes the wheels
  // chase pitch. Leave static COM ownership to the LQR
  // until both legs have enough bend for a well-conditioned task solve.
  const bool static_com_geometry_valid =
      std::abs(state.joints.position_rad[left_knee]) +
          std::abs(state.joints.position_rad[right_knee]) > 0.70F;
  // The slow COM/body-pitch task is a grounded posture owner, not a
  // station-only trim. It may add a bounded correction during rolling Drive;
  // the dedicated C2 height trajectory remains the sole slow leg owner while
  // it is active, with the wheel MPC covering the transient.
  const bool stationary_static_com_request =
      config_.whole_body_stance_owns_static_com &&
      config_.whole_body_stance.enabled && state.floor_contact &&
      usesBalance(mode) && !height_motion_pending_ &&
      !height_motion_active_ && !joint_motion_.active() &&
      !height_capture_recovery_active_ && stationary_stop_hold_s_ <= 0.0F &&
      static_com_geometry_valid;
  const float stance_commanded_velocity =
      lqr_.debugState().commanded_ground_velocity_m_s;
  // WBC must not wait for the pitch error to enter a narrow band: that is the
  // error it is meant to remove. Station capture already combines measured
  // wheel speed, pitch and pitch rate; use that adaptive latch plus a dwell
  // here, while leaving the fast MPC in charge of the transient.
  const bool station_quiet_for_wbc = station_hold_active_ &&
      station_capture_ready_ &&
      state.floor_contact &&
      std::abs(state.wheel.velocity_m_s) <=
          std::max(config_.posture_settle_wheel_velocity_m_s, 0.0F) &&
      std::abs(stance_commanded_velocity) <=
          std::max(config_.posture_settle_wheel_velocity_m_s, 0.0F) &&
      std::abs(state.imu.pitch_rate_rad_s) <=
          std::max(config_.posture_settle_pitch_rate_rad_s, 0.0F) &&
      station_dwell_s_ >=
          std::max(config_.station_settle_dwell_s, 0.0F);
  // The slow leg COM task remains valid while rolling: it changes the
  // body-frame COM relative to the axle instead of asking station hold to
  // fake that motion.
  const bool stance_com_task_request =
      config_.whole_body_stance.enabled && state.floor_contact &&
      (mode == ControlMode::Balance || mode == ControlMode::Drive ||
       mode == ControlMode::PoseChange) &&
      stationary_static_com_request &&
      (station_hold_active_ || station_eligible_ || height_motion_active_ ||
       mode == ControlMode::Drive || mode == ControlMode::PoseChange);
  const float com_task_pitch_limit =
      std::max(config_.whole_body_stance.max_pitch_for_trim_rad, 0.0F);
  const bool leg_com_task_allowed = stance_com_task_request &&
      std::abs(state.imu.pitch_rad) <= com_task_pitch_limit &&
      std::abs(state.imu.pitch_rate_rad_s) <=
          std::max(config_.whole_body_stance.max_pitch_rate_for_trim_rad_s, 1.5F);
  const float live_stance_omega = std::sqrt(
      std::max(config_.whole_body_stance.gravity_m_s2, 0.1F) /
      std::max(last_whole_body_.com_above_axle_m, 0.02F));
  const bool stance_trim_allowed =
      (mode == ControlMode::Balance || mode == ControlMode::Drive ||
       mode == ControlMode::PoseChange) &&
      state.floor_contact &&
      (stationary_stance_request || leg_com_task_allowed) &&
      (station_hold_active_ || height_motion_active_ ||
       joint_motion_.active() || leg_com_task_allowed) &&
      (height_motion_pending_ || height_motion_active_ || joint_motion_.active() ||
       station_quiet_for_wbc || leg_com_task_allowed) &&
      std::abs(state.imu.pitch_rate_rad_s) <
          (leg_com_task_allowed
               ? std::max(2.0F * live_stance_omega, 0.8F)
               : ((height_motion_active_ || height_motion_pending_) ? 2.0F
                                                                    : 0.8F)) &&
      std::abs(state.imu.roll_rate_rad_s) < 0.8F;
  // The leg speed loop is an explicit Drive owner. It must not run during a
  // stationary height change or yaw-only turn, where the LQR already owns
  // translation. Keep it alive through a zero-command braking transient:
  // dropping the leg COM correction on the same sample as DRIVE->STOP leaves
  // the wheels moving without the body-side speed-loop support and creates a
  // large pitch impulse. Station hold takes ownership after capture completes.
  const bool zero_drive_for_speed =
      std::abs(command.target_forward_velocity_m_s) < 1.0e-3F &&
      std::abs(command.target_yaw_rate_rad_s) < 1.0e-3F;
  const float commanded_velocity_for_speed =
      lqr_.debugState().commanded_ground_velocity_m_s;
  const bool stop_braking = zero_drive_for_speed && !station_hold_active_ &&
      (std::abs(state.wheel.velocity_m_s) > 0.03F ||
       std::abs(commanded_velocity_for_speed) > 0.03F ||
       std::abs(state.imu.pitch_rate_rad_s) > 0.15F ||
       std::abs(state.imu.pitch_rad - balance_pitch_ref_rad_) > 0.05F);
  const bool leg_speed_loop_allowed = state.floor_contact &&
      (((mode == ControlMode::Drive || mode == ControlMode::PoseChange) &&
        (std::abs(command.target_forward_velocity_m_s) >= 0.01F ||
         height_motion_pending_)) ||
       stop_braking) &&
      !height_capture_recovery_active_;
  last_whole_body_stance_ = whole_body_stance_.update(
      state, joint_target, last_whole_body_, stance_trim_allowed, state.dt_s,
      leg_com_task_allowed,
      last_leg_velocity_error_m_s_, leg_speed_loop_allowed);
  joint_target = last_whole_body_stance_.target;

  whole_body_gravity_feedforward_.fill(0.0F);
  if (config_.whole_body_gravity_feedforward && state.floor_contact &&
      !last_flight_estimate_.airborne) {
    const float scale = std::clamp(
        config_.whole_body_gravity_feedforward_scale, 0.0F, 1.0F);
    const auto left = WholeBodyModel::legGravityTorque(
        joint_target[left_hip], joint_target[left_knee]);
    const auto left_body = WholeBodyModel::bodySupportTorque(
        joint_target[left_hip], joint_target[left_knee]);
    const auto right = WholeBodyModel::legGravityTorque(
        joint_target[right_hip], joint_target[right_knee]);
    const auto right_body = WholeBodyModel::bodySupportTorque(
        joint_target[right_hip], joint_target[right_knee]);
    whole_body_gravity_feedforward_[left_hip] =
        scale * (left.hip_nm + left_body.hip_nm);
    whole_body_gravity_feedforward_[left_knee] =
        scale * (left.knee_nm + left_body.knee_nm);
    whole_body_gravity_feedforward_[right_hip] =
        scale * (right.hip_nm + right_body.hip_nm);
    whole_body_gravity_feedforward_[right_knee] =
        scale * (right.knee_nm + right_body.knee_nm);
  }
  if (mode == ControlMode::Fallen) {
    setBalanceFallPitch(config_.lqr.fall_pitch_rad);
  }
  const LqrBalancer::Output balance =
      runBalance(state, command, usesBalance(mode));
  joint_target = standUpPose(joint_target);

  if (balance.fallen) {
    if (!config_.keep_legs_active_when_fallen) {
      // Legacy behavior, preserved exactly (default): total safe-stop.
      state_machine_.forceFallen();
      joint_motion_.reset(state.joints.position_rad);
      writeSafeStop(output, state);
      return finish(output);
    }
    // LQR's reduced model no longer trusts a wheel command at this pitch
    // (see LqrBalancer::Config::fall_pitch_rad) - stop the wheels, but do NOT
    // force ControlMode::Fallen or disable the legs. `balance` here is still
    // a validly-constructed Output (LqrBalancer returns immediately on the
    // fall check, before touching any field) - its defaults are exactly what
    // this branch needs: ground_velocity_m_s/wheel velocities at 0 (wheels
    // actively brake to a stop, not freewheel - torque_nm stays at
    // wheel_max_torque_nm below), leg_kp_scale/leg_kd_scale at a normal 1.0
    // (modulated only by impedance, same as any other tick), so reusing
    // writeBalancedCommand as-is is correct, not a shortcut.
    writeBalancedCommand(output, balance, joint_target);
    return finish(output);
  }

  writeBalancedCommand(output, balance, joint_target);
  if (mode == ControlMode::BalanceRamp) {
    // The fixture-release command starts with a bounded wheel authority ramp;
    // leg position hold remains at full stiffness so the body does not sag.
    const float ramp = std::max(0.20F, state_machine_.balanceRampProgress());
    output.velocity_rad_s[left_wheel] *= ramp;
    output.velocity_rad_s[right_wheel] *= ramp;
  }
  return finish(output);
}

ControlMode EmbeddedController::mode() const noexcept {
  return state_machine_.mode();
}

void EmbeddedController::writeSafeStop(MotorCommand& output,
                                       const RobotState& state) const {
  output.enable_motors = false;
  output.position_rad = state.joints.position_rad;
  // Invalid encoders cannot supply a hold target; motors remain disabled.
  for (auto& position : output.position_rad) {
    if (!std::isfinite(position)) position = 0.0F;
  }
  output.velocity_rad_s.fill(0.0F);
  output.torque_nm.fill(0.0F);
  output.kp_scale.fill(0.0F);
  output.kd_scale.fill(0.0F);
}

void EmbeddedController::writePostureHold(
    MotorCommand& output, const JointArray& joint_target,
    const RobotState& state) const {
  output.enable_motors = true;
  output.position_rad = joint_target;
  output.velocity_rad_s.fill(0.0F);
  output.torque_nm.fill(0.0F);
  output.feedforward_nm.fill(0.0F);
  output.kp_scale.fill(1.0F);
  // Use the same critically-damped joint servo as the native boot hold.  A
  // zero kd here leaves a gap at the Enabling->Balance edge and makes the
  // qualification/boot path a different controller from the core hold.
  output.kd_scale.fill(1.0F);

  output.position_rad[left_wheel] = 0.0F;
  output.position_rad[right_wheel] = 0.0F;
  output.torque_nm[left_hip] = config_.leg_max_torque_nm;
  output.torque_nm[left_knee] = config_.leg_max_torque_nm;
  output.torque_nm[right_hip] = config_.leg_max_torque_nm;
  output.torque_nm[right_knee] = config_.leg_max_torque_nm;
  if (config_.whole_body_gravity_feedforward && state.floor_contact) {
    const float scale = std::clamp(
        config_.whole_body_gravity_feedforward_scale, 0.0F, 1.0F);
    const auto left = WholeBodyModel::legGravityTorque(
        state.joints.position_rad[left_hip],
        state.joints.position_rad[left_knee]);
    const auto left_body = WholeBodyModel::bodySupportTorque(
        state.joints.position_rad[left_hip],
        state.joints.position_rad[left_knee]);
    const auto right = WholeBodyModel::legGravityTorque(
        state.joints.position_rad[right_hip],
        state.joints.position_rad[right_knee]);
    const auto right_body = WholeBodyModel::bodySupportTorque(
        state.joints.position_rad[right_hip],
        state.joints.position_rad[right_knee]);
    output.feedforward_nm[left_hip] = scale * (left.hip_nm + left_body.hip_nm);
    output.feedforward_nm[left_knee] = scale * (left.knee_nm + left_body.knee_nm);
    output.feedforward_nm[right_hip] = scale * (right.hip_nm + right_body.hip_nm);
    output.feedforward_nm[right_knee] = scale * (right.knee_nm + right_body.knee_nm);
  }
}

void EmbeddedController::writeBalancedCommand(
    MotorCommand& output, const LqrBalancer::Output& balance,
    const JointArray& joint_target) const {
  output.enable_motors = true;
  output.position_rad = joint_target;
  output.velocity_rad_s.fill(0.0F);
  output.torque_nm.fill(0.0F);
  output.feedforward_nm.fill(0.0F);
  output.kp_scale.fill(1.0F);
  output.kd_scale.fill(1.0F);

  output.velocity_rad_s[left_wheel] = balance.left_wheel_velocity_rad_s;
  output.velocity_rad_s[right_wheel] = balance.right_wheel_velocity_rad_s;
  output.position_rad[left_wheel] = 0.0F;
  output.position_rad[right_wheel] = 0.0F;

  output.kp_scale[left_hip] = balance.leg_kp_scale;
  output.kp_scale[right_hip] = balance.leg_kp_scale;
  output.kp_scale[left_knee] = balance.leg_kp_scale;
  output.kp_scale[right_knee] = balance.leg_kp_scale;
  output.kd_scale[left_hip] = balance.leg_kd_scale;
  output.kd_scale[right_hip] = balance.leg_kd_scale;
  output.kd_scale[left_knee] = balance.leg_kd_scale;
  output.kd_scale[right_knee] = balance.leg_kd_scale;

  output.torque_nm[left_hip] = config_.leg_max_torque_nm;
  output.torque_nm[left_knee] = config_.leg_max_torque_nm;
  output.torque_nm[right_hip] = config_.leg_max_torque_nm;
  output.torque_nm[right_knee] = config_.leg_max_torque_nm;
  output.torque_nm[left_wheel] = config_.wheel_max_torque_nm;
  output.torque_nm[right_wheel] = config_.wheel_max_torque_nm;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    output.feedforward_nm[i] += whole_body_gravity_feedforward_[i];
  }
}

JointArray EmbeddedController::standUpPose(const JointArray& pose) const {
  JointArray out = pose;
  for (const auto id : {left_hip, right_hip}) {
    out[id] = std::clamp(out[id], config_.hip_min_rad, config_.hip_max_rad);
  }
  for (const auto id : {left_knee, right_knee}) {
    out[id] = std::clamp(out[id], config_.knee_min_rad, config_.knee_max_rad);
  }
  return out;
}

void EmbeddedController::beginStandUp(const RobotState& state) {
  stand_up_stage_ = StandUpStage::Tuck;
  stand_up_settle_cycles_ = 0;
  // Which way is the robot lying? +1 face-down (pitch ~ +pi/2), -1 belly-up.
  stand_up_fall_sign_ = (state.imu.pitch_rad >= 0.0F) ? 1.0F : -1.0F;
  joint_motion_.startMove(state.joints.position_rad,
                          standUpPose(config_.stand_up.tuck_pose_rad),
                          config_.stand_up.tuck_duration_s);
  // Do not carry a pre-fall stance correction into Tuck/Rise.
  whole_body_stance_.reset();
  last_whole_body_stance_ = WholeBodyStanceController::Output{};
  whole_body_gravity_feedforward_.fill(0.0F);
  balance_pitch_ref_rad_ = 0.0F;
  last_leg_velocity_error_m_s_ = 0.0F;
  posture_abort_blend_ = 0.0F;
  pitch_trim_rad_ = 0.0F;
  trim_dwell_elapsed_s_ = 0.0F;
  resetBalance(state.wheel.position_m, state.floor_contact);
  setBalanceFallPitch(config_.stand_up.fall_pitch_rad);
  stand_up_settle_elapsed_s_ = 0.0F;
}

void EmbeddedController::advanceStandUp(const RobotState& state) {
  switch (stand_up_stage_) {
    case StandUpStage::Idle:
      break;

    case StandUpStage::Tuck:
      if (!joint_motion_.active()) {
        stand_up_stage_ = StandUpStage::Rise;
        joint_motion_.startMove(joint_motion_.target(),
                                standUpPose(config_.stand_up.stand_pose_rad),
                                config_.stand_up.rise_duration_s);
      }
      break;

    case StandUpStage::Rise:
      if (!joint_motion_.active()) {
        stand_up_stage_ = StandUpStage::Settle;
        stand_up_settle_cycles_ = 0;
        stand_up_settle_elapsed_s_ = 0.0F;
        resetBalance(state.wheel.position_m, state.floor_contact);
      }
      break;

    case StandUpStage::Settle:
      stand_up_settle_elapsed_s_ += std::max(state.dt_s, 0.0F);
      if (std::abs(state.imu.pitch_rad) < config_.stand_up.settle_pitch_rad &&
          std::abs(state.imu.pitch_rate_rad_s) < 0.2F && state.floor_contact) {
        if (++stand_up_settle_cycles_ >= config_.stand_up.settle_cycles) {
          finishStandUp(state);
          break;
        }
      } else {
        stand_up_settle_cycles_ = 0;
      }
      if (stand_up_settle_elapsed_s_ >=
          config_.stand_up.max_settle_duration_s) {
        finishStandUp(state);
        state_machine_.abortStandUp();
      }
      break;
  }
}

void EmbeddedController::finishStandUp(const RobotState& state) {
  stand_up_stage_ = StandUpStage::Idle;
  stand_up_settle_cycles_ = 0;
  stand_up_settle_elapsed_s_ = 0.0F;
  setBalanceFallPitch(config_.lqr.fall_pitch_rad);
  resetBalance(state.wheel.position_m, state.floor_contact);
}

}  // namespace robot_control
