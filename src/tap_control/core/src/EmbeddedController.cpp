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
      landing_impedance_(config_.landing_impedance) {}

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

void EmbeddedController::resetBalance(float ground_position_m) {
  mpc_.reset(ground_position_m);
}

void EmbeddedController::setBalanceFallPitch(float fall_pitch_rad) noexcept {
  mpc_.setFallPitchRad(fall_pitch_rad);
}

MpcBalancer::DebugState EmbeddedController::mpcDebugState() const noexcept {
  return mpc_.debugState();
}

void EmbeddedController::reset(const RobotState& state) {
  resetBalance(state.wheel.position_m);
  setBalanceFallPitch(config_.mpc.fall_pitch_rad);
  joint_motion_.reset(state.joints.position_rad);
  state_machine_.reset();
  balance_pitch_ref_rad_ = 0.0F;
  pitch_trim_rad_ = 0.0F;
  trim_dwell_elapsed_s_ = 0.0F;
  station_dwell_s_ = 0.0F;
  last_whole_body_ = WholeBodyModel::Result{};
  landing_reflex_.reset();
  landing_impedance_.reset();
  stand_up_stage_ = StandUpStage::Idle;
  stand_up_settle_cycles_ = 0;
  stand_up_settle_elapsed_s_ = 0.0F;
  stand_up_fall_sign_ = 1.0F;
}

bool EmbeddedController::standingUp() const noexcept {
  return stand_up_stage_ != StandUpStage::Idle;
}

MotorCommand EmbeddedController::update(const RobotState& state,
                                        const ControlCommand& input) {
  if (!validInput(state, input)) {
    RobotState invalid = state;
    invalid.emergency_stop = true;
    (void)state_machine_.update(invalid, {}, false, false);
    finishStandUp(state);
    joint_motion_.reset(state.joints.position_rad);
    MotorCommand safe;
    writeSafeStop(safe, state);
    return safe;
  }
  ControlCommand command = input;
  command.target_joint_position_rad = standUpPose(input.target_joint_position_rad);
  if (command.request_pose_change && !joint_motion_.active() &&
      stand_up_stage_ == StandUpStage::Idle) {
    joint_motion_.startMove(state.joints.position_rad,
                            command.target_joint_position_rad,
                            config_.pose_change_duration_s);
  }

  const ControlMode previous_mode = state_machine_.mode();
  const ControlMode mode = state_machine_.update(
      state, command, joint_motion_.active(), standingUp());
  if (usesBalance(mode) && !usesBalance(previous_mode)) {
    setBalanceFallPitch(config_.mpc.fall_pitch_rad);
    resetBalance(state.wheel.position_m);
    // Fresh re-entry into Balance/Drive/PoseChange (e.g. after Fallen or
    // StandingUp) must not replay a stale latched airborne/catch offset from
    // before the mode left balancing.
    landing_reflex_.reset();
    landing_impedance_.reset();
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
      writeBalancedCommand(output, balance, joint_target, LandingImpedance::Output{});
    }
    return output;
  }

  // Left the recovery for any other reason (done, tipped, e-stop): make sure
  // the balance controller is clean before it runs again.
  if (stand_up_stage_ != StandUpStage::Idle) {
    finishStandUp(state);
  }

  if (mode == ControlMode::Enabling) {
    resetBalance(state.wheel.position_m);
    writePostureHold(output, command.target_joint_position_rad);
    return output;
  }

  if (mode == ControlMode::Disabled ||
      mode == ControlMode::EmergencyStop ||
      mode == ControlMode::Fallen) {
    joint_motion_.reset(state.joints.position_rad);
    writeSafeStop(output, state);
    return output;
  }

  const MpcBalancer::Output balance =
      runBalance(state, command,
                 usesBalance(mode));
  if (balance.fallen) {
    state_machine_.forceFallen();
    joint_motion_.reset(state.joints.position_rad);
    writeSafeStop(output, state);
    return output;
  }

  JointArray joint_target =
      joint_motion_.active() ? joint_motion_.update(state.dt_s)
                             : command.target_joint_position_rad;
  // Reactive airborne/touchdown leg-target offset (see LandingReflex.hpp).
  // Only reached in Balance/Drive/PoseChange (every other mode returns
  // earlier above). All-zero config makes this an exact no-op every cycle.
  const JointArray reflex_offset = landing_reflex_.update(
      state.floor_contact, state.imu.pitch_rate_rad_s, state.dt_s);
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
      state.joints.velocity_rad_s[right_knee], state.dt_s);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    joint_target[i] += reflex_offset[i];
  }
  joint_target = standUpPose(joint_target);
  writeBalancedCommand(output, balance, joint_target, impedance);
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
    const LandingImpedance::Output& impedance) const {
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

  // LandingImpedance's gain schedule applies to the KNEE ONLY - every
  // mechanism in this file's history that touched the hip in any way
  // (force, position, or gain scaling) measured worse than doing nothing on
  // the 20 cm ledge test; only knee-only exclusion of the hip ever measured
  // better. Hip kp/kd stay exactly at the existing balance scale. At
  // default (impedance inactive) the knee scale below is exactly 1.0.
  output.kp_scale[left_hip] = balance.leg_kp_scale;
  output.kp_scale[right_hip] = balance.leg_kp_scale;
  // Per-side (ROLL axis, 2026-09-16): each knee scaled by ITS OWN side's
  // impedance output, not a shared value - see LandingImpedance.hpp.
  output.kp_scale[left_knee] = balance.leg_kp_scale * impedance.left_leg_kp_scale;
  output.kp_scale[right_knee] = balance.leg_kp_scale * impedance.right_leg_kp_scale;

  output.kd_scale[left_hip] = balance.leg_kd_scale;
  output.kd_scale[right_hip] = balance.leg_kd_scale;
  output.kd_scale[left_knee] = balance.leg_kd_scale * impedance.left_leg_kd_scale;
  output.kd_scale[right_knee] = balance.leg_kd_scale * impedance.right_leg_kd_scale;

  output.torque_nm[left_hip] = config_.leg_max_torque_nm;
  output.torque_nm[left_knee] = config_.leg_max_torque_nm;
  output.torque_nm[right_hip] = config_.leg_max_torque_nm;
  output.torque_nm[right_knee] = config_.leg_max_torque_nm;
  output.torque_nm[left_wheel] = config_.wheel_max_torque_nm;
  output.torque_nm[right_wheel] = config_.wheel_max_torque_nm;
  // LandingImpedance never sets a position target or feedforward torque -
  // feedforward_nm stays at 0 here and is only ever set by the jump path.
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
  resetBalance(state.wheel.position_m);
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
        resetBalance(state.wheel.position_m);
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
  resetBalance(state.wheel.position_m);
}

}  // namespace robot_control
