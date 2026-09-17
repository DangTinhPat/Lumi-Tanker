#include "robot_control/StateMachine.hpp"

#include <cmath>

namespace robot_control {

StateMachine::StateMachine() : StateMachine(Config{}) {}

StateMachine::StateMachine(Config config) : config_(config) {}

void StateMachine::reset() {
  mode_ = ControlMode::Disabled;
  cycles_in_mode_ = 0;
  stand_up_requested_ = false;
}

ControlMode StateMachine::update(const RobotState& state,
                                 const ControlCommand& command,
                                 bool pose_motion_active,
                                 bool stand_up_active,
                                 bool airborne) {
  const bool stand_trigger = command.request_stand_up && !stand_up_requested_;
  stand_up_requested_ = command.request_stand_up;
  if (state.emergency_stop || command.request_disable) {
    enter(ControlMode::EmergencyStop);
    return mode_;
  }

  const bool tipped = std::abs(state.imu.pitch_rad) > config_.fall_pitch_rad;

  // E-stop stays latched even if the robot tips or a recovery trigger arrives.
  if (mode_ == ControlMode::EmergencyStop) {
    if (command.request_enable && !tipped) {
      enter(ControlMode::Enabling);
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

  if (stand_trigger &&
      (mode_ == ControlMode::Disabled || mode_ == ControlMode::Fallen ||
       mode_ == ControlMode::Balance || mode_ == ControlMode::Drive)) {
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
        enter(ControlMode::Enabling);
      }
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
      if (command.request_enable && !state.emergency_stop) {
        enter(ControlMode::Enabling);
      }
      break;
  }

  return mode_;
}

ControlMode StateMachine::mode() const noexcept { return mode_; }

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
