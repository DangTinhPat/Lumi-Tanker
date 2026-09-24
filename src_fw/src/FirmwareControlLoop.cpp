#include "tap_fw/FirmwareControlLoop.hpp"

namespace tap_fw {

FirmwareControlLoop::FirmwareControlLoop(
    motor_transport::Transport& transport,
    robot_control::EmbeddedController::Config controller_config,
    ControllerActuatorBridge::Config actuator_config) noexcept
    : controller_([&controller_config] {
        controller_config.state_machine.fixture_startup_enabled = true;
        return controller_config;
      }()),
      actuators_(transport, actuator_config) {}

ControllerActuatorBridge::Result FirmwareControlLoop::step(
    const robot_control::RobotState& state,
    const robot_control::ControlCommand& command,
    std::uint64_t now_us) noexcept {
  last_command_ = controller_.update(state, command);
  return actuators_.apply(last_command_, state, now_us);
}

}  // namespace tap_fw
