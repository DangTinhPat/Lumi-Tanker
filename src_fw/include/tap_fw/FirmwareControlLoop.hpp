#pragma once

#include <cstdint>

#include "robot_control/EmbeddedController.hpp"
#include "tap_fw/ControllerActuatorBridge.hpp"
#include "tap_fw/FirmwareProfile.hpp"

namespace tap_fw {

// The ESP firmware's one-step boundary. It keeps the portable controller
// unchanged while making the runtime order explicit: sample state in, compute
// MotorCommand, then emit protocol frames through the selected transport.
class FirmwareControlLoop {
 public:
  FirmwareControlLoop(motor_transport::Transport& transport,
                      robot_control::EmbeddedController::Config controller_config =
                          firmwareControllerConfig(),
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
  [[nodiscard]] const robot_control::EmbeddedController& controller() const noexcept {
    return controller_;
  }

  void setLegOffsets(const robot_control::JointArray* offsets) noexcept {
    actuators_.setLegOffsets(offsets);
  }
  void onCanReceived(const motor_transport::CanFrame& frame) noexcept {
    actuators_.onCanReceived(frame);
  }
  void setLegCurrentLimit(float amps) noexcept { actuators_.setLegCurrentLimit(amps); }
  void setLegFixedGains(float kp, float kd) noexcept { actuators_.setLegFixedGains(kp, kd); }
  [[nodiscard]] float legFixedKp() const noexcept { return actuators_.legFixedKp(); }
  [[nodiscard]] float legFixedKd() const noexcept { return actuators_.legFixedKd(); }
  [[nodiscard]] const robot_control::JointArray& legTargetIntegral() const noexcept {
    return actuators_.legTargetIntegral();
  }
  [[nodiscard]] float legCurrentLimit() const noexcept { return actuators_.legCurrentLimit(); }
  [[nodiscard]] std::uint32_t mitDeferred() const noexcept {
    return actuators_.mitDeferred();
  }

 private:
  robot_control::EmbeddedController controller_;
  ControllerActuatorBridge actuators_;
  robot_control::MotorCommand last_command_{};
};

}  // namespace tap_fw
