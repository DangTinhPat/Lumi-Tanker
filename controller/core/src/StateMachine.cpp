#include "robot_control/StateMachine.hpp"

#include <algorithm>
#include <cmath>

namespace robot_control {

namespace {
bool startupPoseValid(const RobotState& state,
                      const StateMachine::Config& config) {
  const float hip_error_left = std::abs(
      state.joints.position_rad[static_cast<std::size_t>(JointId::LeftHip)] -
      geometry::neutral_hip_rad);
  const float hip_error_right = std::abs(
      state.joints.position_rad[static_cast<std::size_t>(JointId::RightHip)] -
      geometry::neutral_hip_rad);
  const float knee_error_left = std::abs(
      state.joints.position_rad[static_cast<std::size_t>(JointId::LeftKnee)] -
      geometry::neutral_knee_rad);
  const float knee_error_right = std::abs(
      state.joints.position_rad[static_cast<std::size_t>(JointId::RightKnee)] -
      geometry::neutral_knee_rad);
  return std::abs(state.imu.pitch_rad) <= config.startup_pitch_rad &&
      std::abs(state.imu.pitch_rate_rad_s) <= config.startup_pitch_rate_rad_s &&
      hip_error_left <= config.startup_joint_tolerance_rad &&
      hip_error_right <= config.startup_joint_tolerance_rad &&
      knee_error_left <= config.startup_joint_tolerance_rad &&
      knee_error_right <= config.startup_joint_tolerance_rad;
}
}  // namespace

StateMachine::StateMachine() : StateMachine(Config{}) {}

StateMachine::StateMachine(Config config) : config_(config) {}

void StateMachine::reset() {
  mode_ = ControlMode::Disabled;
  cycles_in_mode_ = 0;
  stand_up_requested_ = false;
  enable_requested_ = false;
}

ControlMode StateMachine::update(const RobotState& state,
                                 const ControlCommand& command,
                                 bool pose_motion_active,
                                 bool stand_up_active,
                                 bool airborne) {
  const bool stand_trigger = command.request_stand_up && !stand_up_requested_;
  stand_up_requested_ = command.request_stand_up;
  const bool enable_trigger = command.request_enable && !enable_requested_;
  enable_requested_ = command.request_enable;
  if (state.emergency_stop || command.request_disable) {
    enter(ControlMode::EmergencyStop);
    return mode_;
  }

  if (config_.fixture_startup_enabled) {
    if (!command.request_enable &&
        (mode_ == ControlMode::SensorQualify ||
         mode_ == ControlMode::FixtureHold ||
         mode_ == ControlMode::BalanceRamp)) {
      enter(ControlMode::Disabled);
      return mode_;
    }
    if (mode_ == ControlMode::SensorQualify) {
      if (startupPoseValid(state, config_)) {
        if (++cycles_in_mode_ >= config_.sensor_qualify_cycles) {
          enter(ControlMode::FixtureHold);
        }
      } else {
        cycles_in_mode_ = 0;
      }
      return mode_;
    }
    if (mode_ == ControlMode::FixtureHold) {
      if (command.request_fixture_release) enter(ControlMode::BalanceRamp);
      return mode_;
    }
    if (mode_ == ControlMode::BalanceRamp) {
      if (++cycles_in_mode_ >= config_.balance_ramp_cycles) {
        enter(ControlMode::Balance);
      }
      return mode_;
    }
  }

  const bool tipped = std::abs(state.imu.pitch_rad) > config_.fall_pitch_rad;

  // E-stop stays latched even if the robot tips or a recovery trigger arrives.
  if (mode_ == ControlMode::EmergencyStop) {
    if (command.request_enable && !tipped) {
      enter(config_.fixture_startup_enabled ? ControlMode::SensorQualify
                                             : ControlMode::Enabling);
    }
    return mode_;
  }

  // The stand-up recovery keeps running even while the torso is far from
  // upright, so it is handled before the generic fall check.
  if (mode_ == ControlMode::StandingUp) {
    if (stand_up_active) {
      return mode_;
    }
    enter(tipped ? ControlMode::Fallen : ControlMode::Balance);
    return mode_;
  }

  // STAND_UP is a recovery transaction, not a normal posture command. A
  // balanced robot must use HEIGHT/POSE; allowing recovery from Balance/Drive
  // folds the legs under a live body and creates an avoidable pitch impulse.
  if (stand_trigger &&
      (mode_ == ControlMode::Disabled || mode_ == ControlMode::Fallen)) {
    enter(ControlMode::StandingUp);
    return mode_;
  }

  // A large pitch during a confirmed support-loss interval is expected in a
  // ledge landing. Do not convert flight into Fallen before touchdown; the
  // leg landing path still needs to run and the wheel MPC has no ground
  // authority in that interval.
  if (tipped && !airborne) {
    enter(ControlMode::Fallen);
    return mode_;
  }

  switch (mode_) {
    case ControlMode::Disabled:
      if (command.request_enable) {
        enter(config_.fixture_startup_enabled ? ControlMode::SensorQualify
                                              : ControlMode::Enabling);
      }
      break;

    case ControlMode::SensorQualify:
    case ControlMode::FixtureHold:
    case ControlMode::BalanceRamp:
      break;

    case ControlMode::StandingUp:
      break;

    case ControlMode::Enabling:
      if (++cycles_in_mode_ >= config_.enable_cycles) {
        enter(ControlMode::Balance);
      }
      break;

    case ControlMode::Balance:
    case ControlMode::Drive:
      if (command.request_pose_change || pose_motion_active) {
        enter(ControlMode::PoseChange);
      } else if (command.target_forward_velocity_m_s != 0.0F ||
                 command.target_yaw_rate_rad_s != 0.0F) {
        enter(ControlMode::Drive);
      } else {
        enter(ControlMode::Balance);
      }
      break;

    case ControlMode::PoseChange:
      if (!pose_motion_active && !command.request_pose_change) {
        enter(ControlMode::Balance);
      }
      break;

    case ControlMode::Fallen:
    case ControlMode::EmergencyStop:
      // A normal enable request must not re-arm a robot that is still
      // visibly tipped.  Recovery has an explicit STAND_UP path with its own
      // guarded sequence; allowing Fallen -> Enabling here caused a
      // Fallen/Enabling oscillation in the landing trace and could reapply
      // balance torques while the body was already past the fall limit.
      // Fallen is a latched safety state.  A held ENABLE level must not
      // re-arm the wheels after a transient pitch recovery; require a new
      // rising edge (or the explicit STAND_UP path above).
      if (enable_trigger && !state.emergency_stop && !tipped) {
        enter(ControlMode::Enabling);
      }
      break;
  }

  return mode_;
}

ControlMode StateMachine::mode() const noexcept { return mode_; }

float StateMachine::balanceRampProgress() const noexcept {
  if (mode_ != ControlMode::BalanceRamp || config_.balance_ramp_cycles == 0U) {
    return mode_ == ControlMode::Balance ? 1.0F : 0.0F;
  }
  return std::clamp(
      static_cast<float>(cycles_in_mode_) /
          static_cast<float>(config_.balance_ramp_cycles),
      0.0F, 1.0F);
}

void StateMachine::enter(ControlMode next_mode) {
  if (mode_ == next_mode) {
    return;
  }
  mode_ = next_mode;
  cycles_in_mode_ = 0;
}

const char* toString(ControlMode mode) noexcept {
  switch (mode) {
    case ControlMode::Disabled:
      return "Disabled";
    case ControlMode::SensorQualify:
      return "SensorQualify";
    case ControlMode::FixtureHold:
      return "FixtureHold";
    case ControlMode::BalanceRamp:
      return "BalanceRamp";
    case ControlMode::Enabling:
      return "Enabling";
    case ControlMode::StandingUp:
      return "StandingUp";
    case ControlMode::Balance:
      return "Balance";
    case ControlMode::Drive:
      return "Drive";
    case ControlMode::PoseChange:
      return "PoseChange";
    case ControlMode::Fallen:
      return "Fallen";
    case ControlMode::EmergencyStop:
      return "EmergencyStop";
  }
  return "Unknown";
}

}  // namespace robot_control
