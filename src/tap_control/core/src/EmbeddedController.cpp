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
  return mode == ControlMode::Balance || mode == ControlMode::Drive ||
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
      std::isfinite(state.wheel.position_m) &&
      std::isfinite(state.wheel.velocity_m_s) &&
      std::isfinite(command.target_forward_velocity_m_s) &&
      std::isfinite(command.target_yaw_rate_rad_s);
}

[[nodiscard]] EmbeddedController::Config normalizeConfig(
    EmbeddedController::Config config) {
  config.state_machine.fall_pitch_rad = config.mpc.fall_pitch_rad;
  return config;
}

}  // namespace

EmbeddedController::EmbeddedController() : EmbeddedController(Config{}) {}

EmbeddedController::EmbeddedController(Config config)
    : config_(normalizeConfig(config)),
      mpc_(config_.mpc),
      whole_body_(config_.whole_body),
      joint_motion_(config_.joint_motion),
      state_machine_(config_.state_machine),
      landing_reflex_(config_.landing_reflex),
      landing_impedance_(config_.landing_impedance),
      landing_controller_(config_.landing_controller),
      flight_observer_(config_.flight_observer),
      trajectory_observer_(config_.trajectory_observer) {}

float EmbeddedController::updateBalancePitchRef(const RobotState& state,
                                               const ControlCommand& command) {
  constexpr auto lh = static_cast<std::size_t>(JointId::LeftHip);
  constexpr auto lk = static_cast<std::size_t>(JointId::LeftKnee);
  constexpr auto rh = static_cast<std::size_t>(JointId::RightHip);
  constexpr auto rk = static_cast<std::size_t>(JointId::RightKnee);
  // FK describes actual encoder geometry, including asymmetry and servo lag.
  const auto& q = state.joints.position_rad;
  last_whole_body_ = whole_body_.compute(q[lh], q[lk], q[rh], q[rk]);
  // Keep the MPC's inverted-pendulum arm matched to the live CoM height, so a
  // squat (shorter, faster pendulum) does not slowly diverge. Inert at the
  // straight-leg stance.
  mpc_.setComHeight(last_whole_body_.com_height_m);

  float target = 0.0F;
  if (config_.whole_body_compensation) {
    // Feed-forward: the geometric equilibrium pitch for the current leg pose.
    target = config_.whole_body_pitch_gain *
             WholeBodyModel::equilibriumPitch(last_whole_body_);

    // Slow integral trim on top of the feed-forward, to absorb the leg sag and
    // dynamic load the FK model does not see. It only adapts when the robot is
    // genuinely settled at a static pose - a steady wheel command, no drive
    // input, no pose motion, near the current target - so a transient never
    // pushes it. Same idea as the removed PID's equilibrium-pitch estimator.
    const float steady_v = mpc_.debugState().commanded_ground_velocity_m_s;
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
    if (config_.whole_body_trim_gain_hz > 0.0F && legs_bent && settled) {
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
    target += pitch_trim_rad_;
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

MpcBalancer::Output EmbeddedController::runBalance(
    const RobotState& state, const ControlCommand& command, bool integrate) {
  constexpr auto lk = static_cast<std::size_t>(JointId::LeftKnee);
  constexpr auto rk = static_cast<std::size_t>(JointId::RightKnee);
  // Station-keeping (MpcBalancer's slow position hold) arms only after the robot
  // has sat quiet in a bent pose for a dwell time: bent legs, no drive input,
  // legs not slewing, low pitch rate. So it never runs during bring-up, driving,
  // or a pose transition - only once the robot is parked in a squat, where it
  // bleeds the residual creep the MPC's weak position authority leaves.
  const float knee_bend = std::abs(state.joints.position_rad[lk]) +
                          std::abs(state.joints.position_rad[rk]);
  const float knee_rate = std::abs(state.joints.velocity_rad_s[lk]) +
                          std::abs(state.joints.velocity_rad_s[rk]);
  const bool parked_quiet =
      knee_bend > 0.6F && knee_rate < 0.05F && !joint_motion_.active() &&
      std::abs(command.target_forward_velocity_m_s) < 1.0e-3F &&
      std::abs(command.target_yaw_rate_rad_s) < 1.0e-3F &&
      std::abs(state.imu.pitch_rate_rad_s) < 0.12F;
  station_dwell_s_ = parked_quiet ? station_dwell_s_ + state.dt_s : 0.0F;
  const bool hold_station = station_dwell_s_ > 0.5F;
  return mpc_.update(state, command, integrate,
                     updateBalancePitchRef(state, command), hold_station);
}

void EmbeddedController::resetBalance(float ground_position_m, bool floor_contact) {
  mpc_.reset(ground_position_m, floor_contact);
}

void EmbeddedController::setBalanceFallPitch(float fall_pitch_rad) noexcept {
  mpc_.setFallPitchRad(fall_pitch_rad);
}

MpcBalancer::DebugState EmbeddedController::mpcDebugState() const noexcept {
  return mpc_.debugState();
}

void EmbeddedController::reset(const RobotState& state) {
  // Reset is called at a sensor-stream boundary where a ROS adapter may still
  // contain simulator telemetry in the legacy contact fields.  Do not seed
  // any controller state from those fields.  The reset contract is a neutral
  // startup assumption; the first update() immediately replaces the local
  // contact hypothesis from FlightObserver's MPU6500 signals.
  resetBalance(state.wheel.position_m, true);
  setBalanceFallPitch(config_.mpc.fall_pitch_rad);
  joint_motion_.reset(state.joints.position_rad);
  state_machine_.reset();
  balance_pitch_ref_rad_ = 0.0F;
  pitch_trim_rad_ = 0.0F;
  trim_dwell_elapsed_s_ = 0.0F;
  station_dwell_s_ = 0.0F;
  landing_recovery_hold_elapsed_s_ = 0.0F;
  last_whole_body_ = WholeBodyModel::Result{};
  landing_reflex_.reset(true);
  landing_impedance_.reset(true, true);
  landing_controller_.reset();
  flight_observer_.reset(state);
  trajectory_observer_.reset(state);
  last_landing_output_ = LandingController::Output{};
  last_flight_estimate_ = FlightObserver::Output{};
  last_trajectory_estimate_ = TrajectoryObserver::Output{};
  last_landing_forward_velocity_m_s_ = state.wheel.velocity_m_s;
  last_landing_uses_trajectory_observer_ = false;
  last_supported_forward_velocity_m_s_ = state.wheel.velocity_m_s;
  last_supported_trajectory_velocity_valid_ = false;
  stand_up_stage_ = StandUpStage::Idle;
  stand_up_settle_cycles_ = 0;
  stand_up_settle_elapsed_s_ = 0.0F;
  stand_up_fall_sign_ = 1.0F;
}

bool EmbeddedController::standingUp() const noexcept {
  return stand_up_stage_ != StandUpStage::Idle;
}

MotorCommand EmbeddedController::update(const RobotState& input_state,
                                        const ControlCommand& input) {
  if (!validInput(input_state, input)) {
    RobotState invalid = input_state;
    invalid.emergency_stop = true;
    (void)state_machine_.update(invalid, {}, false, false);
    finishStandUp(input_state);
    joint_motion_.reset(input_state.joints.position_rad);
    MotorCommand safe;
    writeSafeStop(safe, input_state);
    return safe;
  }
  // Contact fields arriving from a ROS adapter are not trusted. The flight
  // observer derives a support hypothesis from MPU6500 signals; all landing,
  // state-machine and MPC decisions below use this local sensor-derived copy.
  RobotState state = input_state;
  ControlCommand command = input;
  command.target_joint_position_rad = standUpPose(input.target_joint_position_rad);
  if (command.request_pose_change && !joint_motion_.active() &&
      stand_up_stage_ == StandUpStage::Idle) {
    joint_motion_.startMove(state.joints.position_rad,
                            command.target_joint_position_rad,
                            config_.pose_change_duration_s);
  }

  const ControlMode previous_mode = state_machine_.mode();
  last_trajectory_estimate_ = trajectory_observer_.update(state);
  last_flight_estimate_ = flight_observer_.update(state);
  state.floor_contact = last_flight_estimate_.support_contact;
  state.left_floor_contact = last_flight_estimate_.left_support_contact;
  state.right_floor_contact = last_flight_estimate_.right_support_contact;
  const ControlMode mode = state_machine_.update(
      state, command, joint_motion_.active(), standingUp(),
      last_flight_estimate_.airborne || last_flight_estimate_.support_settling);
  if (usesBalance(mode) && !usesBalance(previous_mode)) {
    setBalanceFallPitch(config_.mpc.fall_pitch_rad);
    resetBalance(state.wheel.position_m, state.floor_contact);
    // Fresh re-entry into Balance/Drive/PoseChange (e.g. after Fallen or
    // StandingUp) must not replay a stale latched airborne/catch offset from
    // before the mode left balancing.
    landing_reflex_.reset(state.floor_contact);
    landing_impedance_.reset(state.left_floor_contact, state.right_floor_contact);
    landing_controller_.reset();
    flight_observer_.reset(state);
    trajectory_observer_.reset(state);
    last_flight_estimate_ = FlightObserver::Output{};
    last_trajectory_estimate_ = TrajectoryObserver::Output{};
    last_landing_output_ = LandingController::Output{};
    last_supported_forward_velocity_m_s_ = state.wheel.velocity_m_s;
    last_supported_trajectory_velocity_valid_ = false;
    landing_recovery_hold_elapsed_s_ = 0.0F;
  }

  MotorCommand output{};

  if (mode == ControlMode::StandingUp) {
    if (stand_up_stage_ == StandUpStage::Idle) {
      beginStandUp(state);
    }
    advanceStandUp(state);
    if (state_machine_.mode() == ControlMode::Fallen) {
      writeSafeStop(output, state);
      return output;
    }
    const JointArray joint_target = joint_motion_.update(state.dt_s);
    if (stand_up_stage_ == StandUpStage::Tuck) {
      // Genuflect first: fold the legs with the wheels held still so the
      // contact points travel under the centre of mass.
      writePostureHold(output, joint_target);
    } else {
      // Rise / Settle: the wheel balancer already drives the wheels under the
      // torso as the legs extend. Ignore any pending drive command here.
      ControlCommand recovery_command = command;
      recovery_command.target_forward_velocity_m_s = 0.0F;
      recovery_command.target_yaw_rate_rad_s = 0.0F;
      const MpcBalancer::Output balance =
          runBalance(state, recovery_command, true);
      if (balance.fallen) {
        finishStandUp(state);
        state_machine_.abortStandUp();
        writeSafeStop(output, state);
        return output;
      }
      writeBalancedCommand(output, balance, joint_target, LandingImpedance::Output{},
                           LandingController::Output{});
    }
    return output;
  }

  // Left the recovery for any other reason (done, tipped, e-stop): make sure
  // the balance controller is clean before it runs again.
  if (stand_up_stage_ != StandUpStage::Idle) {
    finishStandUp(state);
  }

  if (mode == ControlMode::Enabling) {
    resetBalance(state.wheel.position_m, state.floor_contact);
    writePostureHold(output, command.target_joint_position_rad);
    return output;
  }

  if (mode == ControlMode::Disabled || mode == ControlMode::EmergencyStop) {
    joint_motion_.reset(state.joints.position_rad);
    writeSafeStop(output, state);
    return output;
  }

  // StateMachine::update() has its OWN, separate absolute-pitch "tipped"
  // check (std::abs(state.imu.pitch_rad) > fall_pitch_rad) that forces
  // ControlMode::Fallen independently of MpcBalancer's own relative-pitch
  // check below - in the common case (pitch_ref near 0) the two trip at
  // nearly the same real pitch, so keep_legs_active_when_fallen must cover
  // THIS path too, not only the balance.fallen branch further down, or it
  // would still total-safe-stop the legs in the exact scenario it exists
  // for. Legacy behavior (false) is unchanged: total safe-stop, same as
  // Disabled/EmergencyStop above.
  if (mode == ControlMode::Fallen && !config_.keep_legs_active_when_fallen) {
    joint_motion_.reset(state.joints.position_rad);
    writeSafeStop(output, state);
    return output;
  }

  JointArray joint_target =
      joint_motion_.active() ? joint_motion_.update(state.dt_s)
                             : command.target_joint_position_rad;
  const LandingController::Output landing_output =
      landing_controller_.update(state, joint_target, &last_flight_estimate_);
  last_landing_output_ = landing_output;
  const bool landing_fallen_recovery =
      config_.landing_fallen_recovery_enabled &&
      config_.keep_legs_active_when_fallen && mode == ControlMode::Fallen &&
      state.floor_contact &&
      std::abs(state.imu.pitch_rad) <= config_.landing_recovery_control_pitch_rad;
  if (landing_fallen_recovery) {
    setBalanceFallPitch(std::max(config_.mpc.fall_pitch_rad,
                                 config_.landing_recovery_control_pitch_rad));
  } else if (mode == ControlMode::Fallen) {
    setBalanceFallPitch(config_.mpc.fall_pitch_rad);
    landing_recovery_hold_elapsed_s_ = 0.0F;
  }
  ControlCommand balance_command = command;
  // A drive-off command can remain valid through flight, but applying it at
  // full scale on the first support tick injects a second pitch transient
  // while the landing impulse is still being absorbed. LandingController's
  // bounded handoff keeps the wheel balance loop alive and only scales the
  // requested motion; it does not use simulator pose/contact.
  if (landing_output.drive_velocity_scale < 1.0F) {
    balance_command.target_forward_velocity_m_s *=
        landing_output.drive_velocity_scale;
    balance_command.target_yaw_rate_rad_s *=
        landing_output.drive_velocity_scale;
  }
  if (landing_fallen_recovery) {
    balance_command.target_forward_velocity_m_s = 0.0F;
    balance_command.target_yaw_rate_rad_s = 0.0F;
  }
  const MpcBalancer::Output balance =
      runBalance(state, balance_command,
                 (usesBalance(mode) || landing_fallen_recovery) &&
                     !landing_output.suspend_wheel_balance);
  if (landing_fallen_recovery && !balance.fallen) {
    const bool handoff_ready =
        std::abs(state.imu.pitch_rad) <= config_.landing_recovery_handoff_pitch_rad &&
        std::abs(state.imu.pitch_rate_rad_s) <=
            config_.landing_recovery_pitch_rate_rad_s;
    if (handoff_ready) {
      landing_recovery_hold_elapsed_s_ += state.dt_s;
      if (landing_recovery_hold_elapsed_s_ >=
          config_.landing_recovery_hold_s) {
        state_machine_.recoverToBalance();
        setBalanceFallPitch(config_.mpc.fall_pitch_rad);
        landing_recovery_hold_elapsed_s_ = 0.0F;
      }
    } else {
      landing_recovery_hold_elapsed_s_ = 0.0F;
    }
  } else {
    landing_recovery_hold_elapsed_s_ = 0.0F;
  }
  // Legs (reflex/impedance) computed BEFORE the fallen branch below and
  // shared by both outcomes - see Config::keep_legs_active_when_fallen's own
  // comment for why: unlike the wheel-drive MPC (which needs real
  // ground/friction authority and legitimately has none to trust once its
  // own linear-model-derived fall_pitch_rad trips), the leg mechanisms below
  // (capture-point hip placement, differential roll correction, active knee
  // extension, impedance) never depended on that authority and remain
  // physically valid regardless of how far pitch has gone.
  // Reactive airborne/touchdown leg-target offset (see LandingReflex.hpp).
  // Only reached in Balance/Drive/PoseChange (every other mode returns
  // earlier above). All-zero config makes this an exact no-op every cycle.
  const bool reflex_floor_contact = !last_flight_estimate_.airborne &&
                                    state.floor_contact;
  const float previous_landing_forward_velocity_m_s =
      last_landing_forward_velocity_m_s_;
  const bool previous_landing_uses_trajectory_observer =
      last_landing_uses_trajectory_observer_;
  last_landing_forward_velocity_m_s_ = state.wheel.velocity_m_s;
  last_landing_uses_trajectory_observer_ = false;
  const float min_trajectory_confidence = std::clamp(
      config_.trajectory_observer_min_confidence, 0.0F, 1.0F);
  const bool trajectory_sample_valid =
      last_trajectory_estimate_.valid &&
      last_trajectory_estimate_.confidence >= min_trajectory_confidence &&
      std::isfinite(last_trajectory_estimate_.forward_velocity_m_s);
  const bool unloading =
      (config_.landing_reflex.unloading_accel_mps2 > 0.0F &&
       state.imu.accel_magnitude_m_s2 <=
           config_.landing_reflex.unloading_accel_mps2) ||
      // Protect the supported-velocity latch from the first unloaded encoder
      // sample even when the optional reflex pre-arm is disabled. The
      // FlightObserver deliberately debounces this same MPU6500 free-fall
      // signature, so its phase can still read Grounded for one or two ticks.
      state.imu.accel_magnitude_m_s2 <=
          config_.flight_observer.free_fall_accel_threshold_m_s2;
  // Preserve the last supported estimate before selecting the landing input.
  // Do not overwrite it on an unloading sample: that sample is exactly where
  // an unloaded wheel encoder can report zero despite the robot's real
  // horizontal launch velocity still being nonzero.
  if (!last_flight_estimate_.airborne && !unloading &&
      trajectory_sample_valid) {
    last_supported_forward_velocity_m_s_ =
        last_trajectory_estimate_.forward_velocity_m_s;
    last_supported_trajectory_velocity_valid_ = true;
  }
  const bool support_lost_for_landing = !state.floor_contact || unloading;
  // If the first low-specific-force sample coincides with the balance-mode
  // handoff, the preceding trajectory output may have been cleared as part
  // of the observer reset. Preserve the last landing input in that narrow
  // event instead of allowing an unloaded encoder zero to erase it.
  if (support_lost_for_landing && !last_supported_trajectory_velocity_valid_ &&
      previous_landing_uses_trajectory_observer &&
      std::isfinite(previous_landing_forward_velocity_m_s)) {
    last_supported_forward_velocity_m_s_ =
        previous_landing_forward_velocity_m_s;
    last_supported_trajectory_velocity_valid_ = true;
  }
  if (support_lost_for_landing &&
      last_supported_trajectory_velocity_valid_) {
    // This is the pre-liftoff latch.  LandingReflex itself latches this value
    // on its liftoff edge and holds the resulting hip target through flight.
    last_landing_forward_velocity_m_s_ =
        last_supported_forward_velocity_m_s_;
    last_landing_uses_trajectory_observer_ = true;
  } else if (trajectory_sample_valid) {
    last_landing_forward_velocity_m_s_ =
        last_trajectory_estimate_.forward_velocity_m_s;
    last_landing_uses_trajectory_observer_ = true;
  }
  const JointArray reflex_offset = landing_reflex_.update(
      reflex_floor_contact, state.imu.pitch_rate_rad_s,
      state.imu.roll_rate_rad_s, state.imu.accel_magnitude_m_s2,
      last_landing_forward_velocity_m_s_, last_whole_body_.com_height_m,
      joint_target[left_hip], joint_target[left_knee], state.dt_s);
  // Virtual leg-length spring-damper via gain scheduling (see
  // LandingImpedance.hpp): softens Kp / raises Kd on real touchdown so the
  // leg yields under whatever real impact force is already acting on it,
  // instead of being driven anywhere. Same scope as the reflex above;
  // default (1.0/1.0) config makes this an exact no-op every cycle.
  const LandingImpedance::Output impedance = landing_impedance_.update(
      state.left_floor_contact, state.right_floor_contact,
      state.joints.position_rad[left_hip],
      state.joints.position_rad[left_knee], state.joints.position_rad[right_hip],
      state.joints.position_rad[right_knee], state.joints.velocity_rad_s[left_hip],
      state.joints.velocity_rad_s[left_knee], state.joints.velocity_rad_s[right_hip],
      state.joints.velocity_rad_s[right_knee], state.imu.accel_magnitude_m_s2,
      state.dt_s, state.imu.hard_impact_event);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    joint_target[i] += reflex_offset[i] + landing_output.position_offset_rad[i];
  }
  joint_target = standUpPose(joint_target);

  if (balance.fallen) {
    if (!config_.keep_legs_active_when_fallen) {
      // Legacy behavior, preserved exactly (default): total safe-stop.
      state_machine_.forceFallen();
      joint_motion_.reset(state.joints.position_rad);
      writeSafeStop(output, state);
      return output;
    }
    // MPC's own linear model no longer trusts a wheel command at this pitch
    // (see MpcBalancer::Config::fall_pitch_rad) - stop the wheels, but do NOT
    // force ControlMode::Fallen or disable the legs. `balance` here is still
    // a validly-constructed Output (MpcBalancer returns immediately on the
    // fall check, before touching any field) - its defaults are exactly what
    // this branch needs: ground_velocity_m_s/wheel velocities at 0 (wheels
    // actively brake to a stop, not freewheel - torque_nm stays at
    // wheel_max_torque_nm below), leg_kp_scale/leg_kd_scale at a normal 1.0
    // (modulated only by impedance, same as any other tick), so reusing
    // writeBalancedCommand as-is is correct, not a shortcut.
    writeBalancedCommand(output, balance, joint_target, impedance,
                         landing_output);
    return output;
  }

  writeBalancedCommand(output, balance, joint_target, impedance, landing_output);
  return output;
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
    MotorCommand& output, const JointArray& joint_target) const {
  output.enable_motors = true;
  output.position_rad = joint_target;
  output.velocity_rad_s.fill(0.0F);
  output.torque_nm.fill(0.0F);
  output.kp_scale.fill(1.0F);
  // kd_scale was decorative until the node started consuming it for the
  // landing-recovery damping boost (see MpcBalancer::Config::landing_kd_boost).
  // That boost is only ever produced by the balance path (writeBalancedCommand),
  // so keep posture-hold's kd contribution at 0 - unchanged from when kd_scale
  // was unread everywhere.
  output.kd_scale.fill(0.0F);

  output.position_rad[left_wheel] = 0.0F;
  output.position_rad[right_wheel] = 0.0F;
  output.torque_nm[left_hip] = config_.leg_max_torque_nm;
  output.torque_nm[left_knee] = config_.leg_max_torque_nm;
  output.torque_nm[right_hip] = config_.leg_max_torque_nm;
  output.torque_nm[right_knee] = config_.leg_max_torque_nm;
}

void EmbeddedController::writeBalancedCommand(
    MotorCommand& output, const MpcBalancer::Output& balance,
    const JointArray& joint_target,
    const LandingImpedance::Output& impedance,
    const LandingController::Output& landing) const {
  output.enable_motors = true;
  output.position_rad = joint_target;
  output.velocity_rad_s.fill(0.0F);
  output.torque_nm.fill(0.0F);
  output.feedforward_nm.fill(0.0F);
  output.kp_scale.fill(1.0F);
  output.kd_scale.fill(1.0F);

  output.velocity_rad_s[left_wheel] =
      landing.suspend_wheel_balance ? 0.0F : balance.left_wheel_velocity_rad_s;
  output.velocity_rad_s[right_wheel] =
      landing.suspend_wheel_balance ? 0.0F : balance.right_wheel_velocity_rad_s;
  output.position_rad[left_wheel] = 0.0F;
  output.position_rad[right_wheel] = 0.0F;

  // LandingImpedance's Impact schedule remains knee-focused. Hip scaling is
  // opt-in and intended for pre-touchdown airborne compliance only; defaults
  // are exactly 1.0, preserving the previous hip behavior.
  output.kp_scale[left_hip] = balance.leg_kp_scale * impedance.left_hip_kp_scale *
                              landing.kp_scale[left_hip];
  output.kp_scale[right_hip] = balance.leg_kp_scale * impedance.right_hip_kp_scale *
                               landing.kp_scale[right_hip];
  // Per-side (ROLL axis, 2026-09-16): each knee scaled by ITS OWN side's
  // impedance output, not a shared value - see LandingImpedance.hpp.
  output.kp_scale[left_knee] = balance.leg_kp_scale * impedance.left_leg_kp_scale *
                               landing.kp_scale[left_knee];
  output.kp_scale[right_knee] = balance.leg_kp_scale * impedance.right_leg_kp_scale *
                                landing.kp_scale[right_knee];

  output.kd_scale[left_hip] = balance.leg_kd_scale * impedance.left_hip_kd_scale *
                              landing.kd_scale[left_hip];
  output.kd_scale[right_hip] = balance.leg_kd_scale * impedance.right_hip_kd_scale *
                               landing.kd_scale[right_hip];
  output.kd_scale[left_knee] = balance.leg_kd_scale * impedance.left_leg_kd_scale *
                               landing.kd_scale[left_knee];
  output.kd_scale[right_knee] = balance.leg_kd_scale * impedance.right_leg_kd_scale *
                                landing.kd_scale[right_knee];

  output.torque_nm[left_hip] = config_.leg_max_torque_nm;
  output.torque_nm[left_knee] = config_.leg_max_torque_nm;
  output.torque_nm[right_hip] = config_.leg_max_torque_nm;
  output.torque_nm[right_knee] = config_.leg_max_torque_nm;
  output.torque_nm[left_wheel] = config_.wheel_max_torque_nm;
  output.torque_nm[right_wheel] = config_.wheel_max_torque_nm;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    output.feedforward_nm[i] += landing.feedforward_nm[i];
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
  setBalanceFallPitch(config_.mpc.fall_pitch_rad);
  resetBalance(state.wheel.position_m, state.floor_contact);
}

}  // namespace robot_control
