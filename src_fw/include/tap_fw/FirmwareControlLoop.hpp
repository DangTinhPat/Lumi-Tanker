#pragma once

#include <cstdint>

#include "robot_control/EmbeddedController.hpp"
#include "tap_fw/ControllerActuatorBridge.hpp"

namespace tap_fw {

// The ESP firmware's one-step boundary. It keeps the portable controller
// unchanged while making the runtime order explicit: sample state in, compute
// MotorCommand, then emit protocol frames through the selected transport.
class FirmwareControlLoop {
 public:
  FirmwareControlLoop(motor_transport::Transport& transport,
                      robot_control::EmbeddedController::Config controller_config = {},
                      ControllerActuatorBridge::Config actuator_config = {}) noexcept;

  [[nodiscard]] ControllerActuatorBridge::Result step(
      const robot_control::RobotState& state,
      const robot_control::ControlCommand& command,
      std::uint64_t now_us) noexcept;

  [[nodiscard]] const robot_control::MotorCommand& last_command() const noexcept {
    return last_command_;
  }

  [[nodiscard]] robot_control::EmbeddedController& controller() noexcept {
    return controller_;
  }

 private:
  robot_control::EmbeddedController controller_;
  ControllerActuatorBridge actuators_;
  robot_control::MotorCommand last_command_{};
};

}  // namespace tap_fw
