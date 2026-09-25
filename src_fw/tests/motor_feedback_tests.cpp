#include <cassert>
#include <cmath>
#include <cstdint>

#include "robot_hal/MotorFrameCodec.hpp"
#include "robot_hal/MotorTransport.hpp"
#include "tap_fw/MotorFeedback.hpp"

namespace {
using namespace robot_control;
using namespace robot_control::motor_frame;
using namespace robot_control::motor_transport;

bool near(float a, float b, float eps = 0.032F) {
  return std::abs(a - b) <= eps;
}

struct PollTransport final : Transport {
  unsigned can_sends = 0U;
  unsigned uart_sends[2]{};

  bool sendCan(std::uint16_t, const MitBytes&) noexcept override {
    ++can_sends;
    return true;
  }
  bool receiveCan(CanFrame&) noexcept override { return false; }
  bool sendUart(std::uint8_t bus, const DxlFrame&) noexcept override {
    if (bus >= 2U) return false;
    ++uart_sends[bus];
    return true;
  }
  bool receiveUart(std::uint8_t, DxlFrame&) noexcept override { return false; }
  bool isOpen() const noexcept override { return true; }
  void close() noexcept override {}
};

CanFrame legFeedback(std::uint8_t node, float position, float velocity,
                     float torque) {
  CanFrame frame{};
  frame.id = canId(node, kMitControlCmdId);
  frame.dlc = 8U;
  packMitFeedback(node, position, velocity, torque, frame.data);
  return frame;
}

}  // namespace

int main() {
  PollTransport transport;
  tap_fw::MotorFeedback polling;
  assert(polling.poll(transport, 0U, false));
  assert(transport.can_sends == 4U);
  assert(transport.uart_sends[0] == 1U && transport.uart_sends[1] == 1U);
  assert(polling.poll(transport, 1'000U, false));
  assert(transport.can_sends == 4U);
  assert(transport.uart_sends[0] == 1U && transport.uart_sends[1] == 1U);
  assert(polling.poll(transport, 2'000U, false));
  assert(transport.uart_sends[0] == 2U && transport.uart_sends[1] == 2U);
  assert(polling.poll(transport, 10'000U, false));
  assert(transport.can_sends == 8U);

  tap_fw::MotorFeedback feedback;
  const CanFrame lh = legFeedback(3U, 0.4F, 0.2F, 1.0F);
  const CanFrame lk = legFeedback(4U, 0.5F, 0.3F, 1.5F);
  const CanFrame rh = legFeedback(1U, 0.6F, 0.4F, 2.0F);
  const CanFrame rk = legFeedback(2U, 0.7F, 0.5F, 2.5F);
  assert(feedback.ingestCan(lh, 1'000U));
  assert(feedback.ingestCan(lk, 1'000U));
  assert(feedback.ingestCan(rh, 1'000U));
  assert(feedback.ingestCan(rk, 1'000U));

  const auto left_first = buildPresentResponse(1U, 1020U, 10, 0);
  const auto right_first = buildPresentResponse(1U, 500U, -10, 0);
  assert(feedback.ingestUart(0U, left_first, 1'000U));
  assert(feedback.ingestUart(1U, right_first, 1'000U));
  assert(feedback.ready(21'000U));
  assert(!feedback.ready(21'001U));

  const auto left_next = buildPresentResponse(1U, 4U, 12, 0);
  const auto right_next = buildPresentResponse(1U, 490U, -12, 0);
  assert(feedback.ingestUart(0U, left_next, 2'000U));
  assert(feedback.ingestUart(1U, right_next, 2'000U));

  RobotState state{};
  feedback.writeTo(state);
  assert(near(state.joints.position_rad[0], 0.4F));
  assert(near(state.joints.position_rad[1], -0.5F));
  assert(near(state.joints.position_rad[3], -0.6F));
  assert(near(state.joints.position_rad[4], 0.7F));
  assert(near(state.joints.velocity_rad_s[0], 0.2F));
  assert(near(state.joints.velocity_rad_s[1], -0.3F));
  assert(near(state.joints.velocity_rad_s[3], -0.4F));
  assert(near(state.joints.velocity_rad_s[4], 0.5F));

  constexpr float kRadiansPerCount = 6.28318530717958647692F / 1024.0F;
  assert(near(state.joints.position_rad[2], 8.0F * kRadiansPerCount));
  assert(near(state.joints.position_rad[5], -10.0F * kRadiansPerCount));
  assert(near(state.joints.velocity_rad_s[2], 12.0F * 6.28318530718F / 60.0F));
  assert(near(state.joints.velocity_rad_s[5], -12.0F * 6.28318530718F / 60.0F));

  CanFrame wrong_command = lh;
  wrong_command.id = canId(3U, kSetAxisStateCmdId);
  assert(!feedback.ingestCan(wrong_command, 3'000U));
  auto bad_wheel = buildPresentResponse(2U, 0U, 0, 0);
  assert(!feedback.ingestUart(0U, bad_wheel, 3'000U));

  return 0;
}
