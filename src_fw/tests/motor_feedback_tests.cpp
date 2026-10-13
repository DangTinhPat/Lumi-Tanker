#include <cassert>
#include <cmath>
#include <cstdint>

#include "robot_control/GeneratedGeometry.hpp"
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

CanFrame iqFeedback(std::uint8_t node, float setpoint_a, float measured_a) {
  CanFrame frame{};
  frame.id = canId(node, kGetIqCmdId);
  frame.dlc = 8U;
  frame.data = packIqFeedback(setpoint_a, measured_a);
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
  assert(feedback.ingestCan(iqFeedback(3U, 0.2F, 0.4F), 1'000U));
  assert(feedback.ingestCan(iqFeedback(4U, 0.2F, 0.5F), 1'000U));
  assert(feedback.ingestCan(iqFeedback(1U, 0.2F, 0.6F), 1'000U));
  assert(feedback.ingestCan(iqFeedback(2U, 0.2F, 0.7F), 1'000U));
  assert(feedback.allLegCurrentsFresh(21'000U));
  assert(!feedback.allLegCurrentsFresh(21'001U));
  // Core sign follows the verified motor map (LH/LK inverted, RH/RK positive).
  assert(near(feedback.legCurrentA(0U), -0.4F));
  assert(near(feedback.legCurrentA(1U), -0.5F));
  assert(near(feedback.legCurrentA(3U), 0.6F));
  assert(near(feedback.legCurrentA(4U), 0.7F));

  const auto left_first = buildPresentResponse(2U, 1020U, 10, 0);
  const auto right_first = buildPresentResponse(1U, 500U, -10, 0);
  assert(feedback.ingestUart(0U, left_first, 1'000U));
  assert(feedback.ingestUart(1U, right_first, 1'000U));
  assert(feedback.ready(21'000U));
  assert(!feedback.ready(21'001U));

  const auto left_next = buildPresentResponse(2U, 4U, 12, 0);
  const auto right_next = buildPresentResponse(1U, 490U, -12, 0);
  assert(feedback.ingestUart(0U, left_next, 2'000U));
  assert(feedback.ingestUart(1U, right_next, 2'000U));

  RobotState state{};
  feedback.writeTo(state);
  // Signs: LH -1, LK -1, RH +1, RK +1.
  assert(near(state.joints.position_rad[0], -0.4F));
  assert(near(state.joints.position_rad[1], -0.5F));
  assert(near(state.joints.position_rad[3], 0.6F));
  assert(near(state.joints.position_rad[4], 0.7F));
  assert(near(state.joints.velocity_rad_s[0], -0.2F));
  assert(near(state.joints.velocity_rad_s[1], -0.3F));
  assert(near(state.joints.velocity_rad_s[3], 0.4F));
  assert(near(state.joints.velocity_rad_s[4], 0.5F));

  constexpr float kRadiansPerCount = 6.28318530717958647692F / 1024.0F;
  // Core wheel = driver_sign * driver reading, on the joint each UART bus is
  // bound to (MotorMapping.hpp): bus 0 moved +8 counts at 12 raw, bus 1 -10
  // counts at -12 raw.
  const auto& bus0 = motor_mapping::kWheelUartBindings[0];
  const auto& bus1 = motor_mapping::kWheelUartBindings[1];
  const float speed_scale = motor_mapping::kWheelPresentSpeedScale;
  assert(near(state.joints.position_rad[bus0.joint_index],
              bus0.driver_sign * 8.0F * kRadiansPerCount));
  assert(near(state.joints.position_rad[bus1.joint_index],
              bus1.driver_sign * -10.0F * kRadiansPerCount));
  assert(near(state.joints.velocity_rad_s[bus0.joint_index],
              bus0.driver_sign * speed_scale * 12.0F * 6.28318530718F / 60.0F));
  assert(near(state.joints.velocity_rad_s[bus1.joint_index],
              bus1.driver_sign * speed_scale * -12.0F * 6.28318530718F / 60.0F));

  // The robot's GDS68 drivers send MIT feedback with DLC 6.
  tap_fw::MotorFeedback short_frames;
  CanFrame dlc6 = legFeedback(3U, 0.25F, 0.0F, 0.0F);
  dlc6.dlc = 6U;
  dlc6.data[6] = 0U;
  dlc6.data[7] = 0U;
  assert(short_frames.ingestCan(dlc6, 1'000U));
  RobotState dlc6_state{};
  short_frames.writeTo(dlc6_state);
  assert(near(dlc6_state.joints.position_rad[0], -0.25F));  // LH sign -1
  CanFrame dlc5 = dlc6;
  dlc5.dlc = 5U;
  assert(!short_frames.ingestCan(dlc5, 1'000U));
  CanFrame short_iq = iqFeedback(3U, 0.1F, 0.2F);
  short_iq.dlc = 7U;
  assert(!short_frames.ingestCan(short_iq, 1'000U));

  CanFrame wrong_command = lh;
  wrong_command.id = canId(3U, kSetAxisStateCmdId);
  assert(!feedback.ingestCan(wrong_command, 3'000U));
  auto bad_wheel = buildPresentResponse(1U, 0U, 0, 0);
  assert(!feedback.ingestUart(0U, bad_wheel, 3'000U));

  // Get_Encoder_Estimates (0x009) is kept as the raw driver estimate (v3 leg
  // lock) but is not a joint source for the MIT path.
  {
    tap_fw::MotorFeedback estimates;
    CanFrame frame{};
    frame.id = canId(3U, kGetEncoderEstimatesCmdId);
    frame.dlc = 8U;
    frame.data = packEncoderEstimates(2.0F, 0.5F);
    assert(estimates.ingestCan(frame, 1'000U));
    assert(!estimates.allLegsFresh(1'000U));
    const auto& est = estimates.legEstimate(0U);  // node 3 = LH
    assert(est.valid && near(est.pos_turns, 2.0F) && near(est.vel_turns_s, 0.5F) &&
           est.stamp_us == 1'000U);
  }

  // Driver heartbeats: stale, axis error and not-closed-loop are all faults.
  {
    tap_fw::MotorFeedback axes;
    const auto heartbeat = [&](std::uint8_t node, std::uint8_t state,
                               std::uint32_t error, std::uint64_t now) {
      CanFrame frame{};
      frame.id = canId(node, kHeartbeatCmdId);
      frame.dlc = 8U;
      frame.data[0] = static_cast<std::uint8_t>(error & 0xFFU);
      frame.data[1] = static_cast<std::uint8_t>((error >> 8U) & 0xFFU);
      frame.data[2] = static_cast<std::uint8_t>((error >> 16U) & 0xFFU);
      frame.data[3] = static_cast<std::uint8_t>(error >> 24U);
      frame.data[4] = state;
      return axes.ingestCan(frame, now);
    };
    constexpr std::size_t kNone = robot_control::kJointCount;
    assert(axes.firstLegAxisFault(0U, false) == 0U);  // No heartbeat yet (LH).
    for (std::uint8_t node = 1U; node <= 4U; ++node) {
      assert(heartbeat(node, kAxisStateIdle, 0U, 1'000U));
    }
    assert(axes.firstLegAxisFault(1'000U, false) == kNone);  // IDLE is fine to enable.
    assert(axes.firstLegAxisFault(1'000U, true) == 0U);      // ...but is not holding.
    for (std::uint8_t node = 1U; node <= 4U; ++node) {
      assert(heartbeat(node, kAxisStateClosedLoop, 0U, 2'000U));
    }
    assert(axes.firstLegAxisFault(2'000U, true) == kNone);
    assert(heartbeat(2U, kAxisStateClosedLoop, 0x40U, 3'000U));  // RK error.
    assert(axes.firstLegAxisFault(3'000U, false) == 4U);
    assert(axes.legAxis(4U).error == 0x40U);
    assert(heartbeat(2U, kAxisStateClosedLoop, 0U, 3'000U));
    // Heartbeat older than kHeartbeatMaxAgeUs is stale.
    assert(axes.firstLegAxisFault(2'000U + tap_fw::board::kHeartbeatMaxAgeUs + 1U,
                                  true) == 0U);
    CanFrame short_heartbeat{};
    short_heartbeat.id = canId(3U, kHeartbeatCmdId);
    short_heartbeat.dlc = 4U;
    assert(!axes.ingestCan(short_heartbeat, 4'000U));
  }

  // Leg homing on the fixture: driver positions restart at 0 on power-up,
  // the fixture holds the neutral pose, so core = sign * raw + offset.
  {
    tap_fw::MotorFeedback homing;
    const auto feedAll = [&](std::uint64_t now, float velocity) {
      // Raw driver positions per node (1 RH, 2 RK, 3 LH, 4 LK).
      assert(homing.ingestCan(legFeedback(3U, 0.10F, velocity, 0.0F), now));
      assert(homing.ingestCan(legFeedback(4U, -0.20F, velocity, 0.0F), now));
      assert(homing.ingestCan(legFeedback(1U, 0.30F, velocity, 0.0F), now));
      assert(homing.ingestCan(legFeedback(2U, -0.40F, velocity, 0.0F), now));
    };
    using Home = tap_fw::MotorFeedback::HomeResult;
    assert(homing.homeLegs(1'000U) == Home::StaleFeedback);
    feedAll(1'000U, 0.5F);
    assert(homing.homeLegs(1'000U) == Home::Moving);
    assert(!homing.legsHomed());
    feedAll(2'000U, 0.0F);
    assert(homing.homeLegs(2'000U) == Home::Homed);
    assert(homing.legsHomed());

    RobotState homed{};
    homing.writeTo(homed);
    assert(near(homed.joints.position_rad[0], geometry::neutral_hip_rad, 1.0e-5F));
    assert(near(homed.joints.position_rad[1], geometry::neutral_knee_rad, 1.0e-5F));
    assert(near(homed.joints.position_rad[3], geometry::neutral_hip_rad, 1.0e-5F));
    assert(near(homed.joints.position_rad[4], geometry::neutral_knee_rad, 1.0e-5F));
    // offset = neutral - sign*raw, per joint and sign (LH -1, LK -1, RH +1, RK +1).
    const auto& offsets = homing.legOffsets();
    assert(near(offsets[0], geometry::neutral_hip_rad + 0.10F, 1.0e-3F));
    assert(near(offsets[1], geometry::neutral_knee_rad - 0.20F, 1.0e-3F));
    assert(near(offsets[3], geometry::neutral_hip_rad - 0.30F, 1.0e-3F));
    assert(near(offsets[4], geometry::neutral_knee_rad + 0.40F, 1.0e-3F));

    // A later reading moves in core coordinates by sign * driver delta.
    assert(homing.ingestCan(legFeedback(3U, 0.30F, 0.0F, 0.0F), 3'000U));  // LH driver +0.2
    assert(homing.ingestCan(legFeedback(1U, 0.50F, 0.0F, 0.0F), 3'000U));  // RH driver +0.2
    homing.writeTo(homed);
    assert(near(homed.joints.position_rad[0], geometry::neutral_hip_rad - 0.20F, 1.0e-3F));
    assert(near(homed.joints.position_rad[3], geometry::neutral_hip_rad + 0.20F, 1.0e-3F));

    // A short silence (CAN bus-off recovery, drivers still counting) keeps
    // the zero; a leg that comes back at ~0 after a non-zero position
    // rebooted, which drops it once.
    const std::uint64_t silent = 3'000U + tap_fw::board::kLegZeroLossUs + 1'000U;
    assert(!homing.superviseLegZero(silent));
    assert(homing.legsHomed());
    const auto feedRest = [&](std::uint64_t now) {
      assert(homing.ingestCan(legFeedback(4U, -0.20F, 0.0F, 0.0F), now));
      assert(homing.ingestCan(legFeedback(1U, 0.50F, 0.0F, 0.0F), now));
      assert(homing.ingestCan(legFeedback(2U, -0.40F, 0.0F, 0.0F), now));
    };
    assert(homing.ingestCan(legFeedback(3U, 0.31F, 0.0F, 0.0F), silent + 1'000U));  // continuous
    feedRest(silent + 1'000U);
    assert(!homing.superviseLegZero(silent + 2'000U));
    assert(homing.legsHomed());
    homing.writeTo(homed);
    assert(near(homed.joints.position_rad[0], geometry::neutral_hip_rad - 0.21F, 1.0e-3F));

    const std::uint64_t silent2 = silent + 2'000U + tap_fw::board::kLegZeroLossUs + 1'000U;
    assert(!homing.superviseLegZero(silent2));
    assert(homing.ingestCan(legFeedback(3U, 0.0F, 0.0F, 0.0F), silent2 + 1'000U));  // rebooted
    assert(homing.superviseLegZero(silent2 + 2'000U));
    assert(!homing.legsHomed());
    assert(!homing.superviseLegZero(silent2 + 3'000U));
    homing.writeTo(homed);
    assert(near(homed.joints.position_rad[0], 0.0F, 1.0e-3F));  // Uncalibrated again.
    assert(near(homing.legOffsets()[0], 0.0F));

    // Silent past kLegZeroForgetUs: dropped without waiting for the answer.
    feedRest(silent2 + 4'000U);
    assert(homing.ingestCan(legFeedback(3U, 0.0F, 0.0F, 0.0F), silent2 + 4'000U));
    assert(homing.homeLegs(silent2 + 4'000U) == Home::Homed);
    assert(!homing.superviseLegZero(silent2 + 4'000U + tap_fw::board::kLegZeroForgetUs));
    assert(homing.superviseLegZero(silent2 + 4'000U + tap_fw::board::kLegZeroForgetUs + 1'000U));
    assert(!homing.legsHomed());
  }

  return 0;
}
