#include <cassert>
#include <cmath>
#include <cstdint>

#include "robot_hal/MotorFrameCodec.hpp"
#include "robot_hal/MotorMapping.hpp"
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

  // The robot's GDS68 drivers send MIT feedback with DLC 6.
  tap_fw::MotorFeedback short_frames;
  CanFrame dlc6 = legFeedback(3U, 0.25F, 0.0F, 0.0F);
  dlc6.dlc = 6U;
  dlc6.data[6] = 0U;
  dlc6.data[7] = 0U;
  assert(short_frames.ingestCan(dlc6, 1'000U));
  RobotState dlc6_state{};
  short_frames.writeTo(dlc6_state);
  assert(near(dlc6_state.joints.position_rad[0], 0.25F));
  CanFrame dlc5 = dlc6;
  dlc5.dlc = 5U;
  assert(!short_frames.ingestCan(dlc5, 1'000U));

  CanFrame wrong_command = lh;
  wrong_command.id = canId(3U, kSetAxisStateCmdId);
  assert(!feedback.ingestCan(wrong_command, 3'000U));
  auto bad_wheel = buildPresentResponse(2U, 0U, 0, 0);
  assert(!feedback.ingestUart(0U, bad_wheel, 3'000U));

  // GDS68 encoder estimates (0x009) are the leg feedback while axes are IDLE.
  const auto estimateFrame = [](std::uint8_t node, float turns, float turns_s) {
    CanFrame frame{};
    frame.id = canId(node, kGetEncoderEstimatesCmdId);
    frame.dlc = 8U;
    frame.data = packEncoderEstimates(turns, turns_s);
    return frame;
  };
  const auto decoded = unpackEncoderEstimates(packEncoderEstimates(1.25F, -0.5F));
  assert(decoded.pos_turns == 1.25F && decoded.vel_turns_s == -0.5F);

  constexpr float kTwoPi = 6.28318530717958647692F;
  const float per_turn = kTwoPi / motor_mapping::kGds68EstimateTurnsPerOutputTurn;
  tap_fw::MotorFeedback idle;
  assert(idle.ingestCan(estimateFrame(3U, 2.0F, 0.5F), 100'000U));   // LH, +1
  assert(idle.ingestCan(estimateFrame(4U, 2.0F, 0.5F), 100'000U));   // LK, -1
  assert(idle.ingestCan(estimateFrame(1U, 2.0F, 0.5F), 100'000U));   // RH, -1
  assert(idle.ingestCan(estimateFrame(2U, 2.0F, 0.5F), 100'000U));   // RK, +1
  assert(idle.allLegsFresh(100'000U));
  RobotState idle_state{};
  idle.writeTo(idle_state);
  assert(near(idle_state.joints.position_rad[0], 2.0F * per_turn, 1.0e-5F));
  assert(near(idle_state.joints.position_rad[1], -2.0F * per_turn, 1.0e-5F));
  assert(near(idle_state.joints.position_rad[3], -2.0F * per_turn, 1.0e-5F));
  assert(near(idle_state.joints.position_rad[4], 2.0F * per_turn, 1.0e-5F));
  assert(near(idle_state.joints.velocity_rad_s[0], 0.5F * per_turn, 1.0e-5F));
  assert(near(idle_state.joints.velocity_rad_s[1], -0.5F * per_turn, 1.0e-5F));
  assert(idle_state.joints.effort_nm[0] == 0.0F);

  // Short frames, unknown nodes and non-finite values are rejected.
  auto short_estimate = estimateFrame(3U, 1.0F, 0.0F);
  short_estimate.dlc = 4U;
  assert(!idle.ingestCan(short_estimate, 101'000U));
  assert(!idle.ingestCan(estimateFrame(9U, 1.0F, 0.0F), 101'000U));
  assert(!idle.ingestCan(estimateFrame(3U, std::nanf(""), 0.0F), 101'000U));

  // Fresh MIT feedback owns the joint; estimates resume once it goes stale.
  assert(idle.ingestCan(legFeedback(3U, 0.4F, 0.0F, 0.0F), 102'000U));
  assert(!idle.ingestCan(estimateFrame(3U, 5.0F, 0.0F), 110'000U));
  idle.writeTo(idle_state);
  assert(near(idle_state.joints.position_rad[0], 0.4F));
  assert(idle.ingestCan(estimateFrame(3U, 5.0F, 0.0F), 122'001U));
  idle.writeTo(idle_state);
  assert(near(idle_state.joints.position_rad[0], 5.0F * per_turn, 1.0e-5F));

  return 0;
}
