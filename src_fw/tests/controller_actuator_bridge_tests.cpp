#include <cassert>
#include <cstdint>
#include <vector>

#include "robot_hal/MotorTransport.hpp"
#include "tap_fw/ControllerActuatorBridge.hpp"
#include "tap_fw/FirmwareControlLoop.hpp"

namespace {
using namespace robot_control;
using namespace robot_control::motor_frame;
using namespace robot_control::motor_transport;

struct FakeTransport final : Transport {
  struct CanWrite { std::uint16_t id; MitBytes data; };
  struct UartWrite { std::uint8_t bus; DxlFrame frame; };
  std::vector<CanWrite> can;
  std::vector<UartWrite> uart;
  bool open = true;

  bool sendCan(std::uint16_t id, const MitBytes& data) noexcept override {
    can.push_back({id, data});
    return true;
  }
  bool receiveCan(CanFrame&) noexcept override { return false; }
  bool sendUart(std::uint8_t bus, const DxlFrame& frame) noexcept override {
    uart.push_back({bus, frame});
    return true;
  }
  bool receiveUart(std::uint8_t, DxlFrame&) noexcept override { return false; }
  bool isOpen() const noexcept override { return open; }
  void close() noexcept override { open = false; }
};

MotorCommand command() {
  MotorCommand out;
  out.enable_motors = true;
  out.position_rad = {0.3F, -1.0F, 0.0F, 0.3F, -1.0F, 0.0F};
  out.velocity_rad_s = {0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 1.0F};
  out.kp_scale.fill(1.0F);
  out.kd_scale.fill(1.0F);
  return out;
}
}  // namespace

int main() {
  FakeTransport transport;
  tap_fw::ControllerActuatorBridge bridge(transport);
  RobotState state;

  assert(bridge.apply(command(), state, 0) ==
         tap_fw::ControllerActuatorBridge::Result::Sent);
  // 4 mode + 4 axis + 4 MIT CAN frames, and 2 wheel enable frames.
  assert(transport.can.size() == 12U);
  assert(transport.uart.size() == 2U);
  assert(bridge.apply(command(), state, 1000) ==
         tap_fw::ControllerActuatorBridge::Result::RateLimited);
  assert(bridge.apply(command(), state, 2000) ==
         tap_fw::ControllerActuatorBridge::Result::Sent);
  assert(transport.can.size() == 16U);
  assert(transport.uart.size() == 2U);
  // Wheel speed is held until the documented 20 ms settle window expires.
  assert(bridge.apply(command(), state, 20'000) ==
         tap_fw::ControllerActuatorBridge::Result::Sent);
  assert(transport.uart.size() == 4U);
  assert(bridge.apply(command(), state, 22'000) ==
         tap_fw::ControllerActuatorBridge::Result::Sent);
  assert(transport.uart.size() == 6U);
  const auto parsed = parseDxlWrite(transport.uart.back().frame);
  assert(parsed.ok && parsed.address == kDxlMovingSpeedAddr);

  MotorCommand stop;
  assert(bridge.apply(stop, state, 24'000) ==
         tap_fw::ControllerActuatorBridge::Result::Sent);
  assert(!bridge.enabled());

  FakeTransport loop_transport;
  tap_fw::FirmwareControlLoop loop(loop_transport);
  state.joints.position_rad[static_cast<std::size_t>(JointId::LeftHip)] =
      geometry::neutral_hip_rad;
  state.joints.position_rad[static_cast<std::size_t>(JointId::LeftKnee)] =
      geometry::neutral_knee_rad;
  state.joints.position_rad[static_cast<std::size_t>(JointId::RightHip)] =
      geometry::neutral_hip_rad;
  state.joints.position_rad[static_cast<std::size_t>(JointId::RightKnee)] =
      geometry::neutral_knee_rad;
  ControlCommand control;
  control.request_enable = true;
  auto loop_result = loop.step(state, control, 0);
  assert(loop_result == tap_fw::ControllerActuatorBridge::Result::Sent);
  assert(!loop.last_command().enable_motors);
  for (unsigned i = 1; i <= 50U; ++i) {
    loop_result = loop.step(state, control, static_cast<std::uint64_t>(i) * 2'000U);
  }
  assert(loop_result == tap_fw::ControllerActuatorBridge::Result::Sent);
  assert(loop.controller().mode() == ControlMode::FixtureHold);
  assert(loop.last_command().enable_motors);
  control.request_fixture_release = true;
  loop_result = loop.step(state, control, 102'000U);
  assert(loop_result == tap_fw::ControllerActuatorBridge::Result::Sent);
  assert(loop.controller().mode() == ControlMode::BalanceRamp);
  return 0;
}
