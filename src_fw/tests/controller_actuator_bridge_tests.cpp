#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
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
  // Wide envelope: these checks are about frames and signs, not the clamp.
  out.torque_nm.fill(50.0F);
  return out;
}

bool near(float a, float b, float eps = 0.01F) {
  return std::abs(a - b) < eps;
}
}  // namespace

int main() {
  FakeTransport transport;
  tap_fw::ControllerActuatorBridge bridge(transport);
  RobotState state;

  assert(bridge.apply(command(), state, 0) ==
         tap_fw::ControllerActuatorBridge::Result::Sent);
  // 4 Set_Limits + 4 mode + 4 axis + 4 MIT CAN frames, 2 wheel enable frames.
  assert(transport.can.size() == 16U);
  {
    const unsigned nodes[4] = {3U, 4U, 1U, 2U};
    for (unsigned i = 0; i < 4U; ++i) {
      assert(transport.can[i].id == canId(nodes[i], kSetLimitsCmdId));
      float amps = 0.0F;
      std::memcpy(&amps, &transport.can[i].data[4], 4U);
      assert(near(amps, tap_fw::board::kLegCurrentLimitA));
    }
  }
  assert(transport.uart.size() == 2U);
  assert(transport.can[4].id == canId(3, kSetControllerModeCmdId));
  assert(transport.can[5].id == canId(3, kSetAxisStateCmdId));
  assert(transport.can[6].id == canId(4, kSetControllerModeCmdId));
  assert(transport.can[7].id == canId(4, kSetAxisStateCmdId));
  assert(transport.can[8].id == canId(1, kSetControllerModeCmdId));
  assert(transport.can[9].id == canId(1, kSetAxisStateCmdId));
  assert(transport.can[10].id == canId(2, kSetControllerModeCmdId));
  assert(transport.can[11].id == canId(2, kSetAxisStateCmdId));
  // Hip signs: LH (node 3) -1, RH (node 1) +1; knees LK (4) -1, RK (2) +1.
  assert(transport.can[12].id == canId(3, kMitControlCmdId));
  assert(near(unpackMitCommand(transport.can[12].data).pos_rad, -0.3F));
  assert(transport.can[13].id == canId(4, kMitControlCmdId));
  assert(near(unpackMitCommand(transport.can[13].data).pos_rad, 1.0F));
  assert(transport.can[14].id == canId(1, kMitControlCmdId));
  assert(near(unpackMitCommand(transport.can[14].data).pos_rad, 0.3F));
  assert(transport.can[15].id == canId(2, kMitControlCmdId));
  assert(near(unpackMitCommand(transport.can[15].data).pos_rad, -1.0F));
  assert(bridge.apply(command(), state, 1000) ==
         tap_fw::ControllerActuatorBridge::Result::RateLimited);
  // Legs run at 250 Hz: no MIT at 2 ms, the next four at 4 ms.
  assert(bridge.apply(command(), state, 2000) ==
         tap_fw::ControllerActuatorBridge::Result::Sent);
  assert(transport.can.size() == 16U);
  assert(transport.uart.size() == 2U);
  assert(bridge.apply(command(), state, 4000) ==
         tap_fw::ControllerActuatorBridge::Result::Sent);
  assert(transport.can.size() == 20U);
  assert(transport.uart.size() == 2U);
  // Wheel speed is held until the documented 20 ms settle window expires.
  assert(bridge.apply(command(), state, 20'000) ==
         tap_fw::ControllerActuatorBridge::Result::Sent);
  assert(transport.uart.size() == 4U);
  for (std::size_t wheel = 2U; wheel < 4U; ++wheel) {
    const auto write = parseDxlWrite(transport.uart[wheel].frame);
    assert(write.ok && write.address == kDxlMovingSpeedAddr);
    const auto raw_speed = static_cast<std::int16_t>(
        static_cast<std::uint16_t>(write.data[0]) |
        (static_cast<std::uint16_t>(write.data[1]) << 8U));
    // Forward core command -> driver raw = driver_sign * speed.
    const float sign = robot_control::motor_mapping::kWheelUartBindings[wheel - 2U].driver_sign;
    assert(static_cast<float>(raw_speed) * sign > 0.0F);
  }
  assert(bridge.apply(command(), state, 22'000) ==
         tap_fw::ControllerActuatorBridge::Result::Sent);
  assert(transport.uart.size() == 6U);
  const auto parsed = parseDxlWrite(transport.uart.back().frame);
  assert(parsed.ok && parsed.address == kDxlMovingSpeedAddr);

  MotorCommand stop;
  const std::size_t before_stop = transport.can.size();
  assert(bridge.apply(stop, state, 24'000) ==
         tap_fw::ControllerActuatorBridge::Result::Sent);
  assert(!bridge.enabled());
  assert(transport.can.size() == before_stop + 4U);  // IDLE on the edge.
  // Disabled repeats at 50 Hz, not every control tick.
  assert(bridge.apply(stop, state, 26'000) ==
         tap_fw::ControllerActuatorBridge::Result::RateLimited);
  assert(bridge.apply(stop, state, 42'000) ==
         tap_fw::ControllerActuatorBridge::Result::RateLimited);
  assert(bridge.apply(stop, state, 44'000) ==
         tap_fw::ControllerActuatorBridge::Result::Sent);
  assert(transport.can.size() == before_stop + 8U);
  for (std::size_t i = before_stop; i < transport.can.size(); ++i) {
    assert((transport.can[i].id & 0x1FU) == kSetAxisStateCmdId);
    assert(unpackSetAxisState(transport.can[i].data) == kAxisStateIdle);
  }

  // Leg torque envelope: a far target is pulled in so that
  // |kp*(pos - q) + T_ff| stays within torque_nm.
  {
    FakeTransport clamp_transport;
    tap_fw::ControllerActuatorBridge clamp_bridge(clamp_transport);
    RobotState clamp_state;
    clamp_state.joints.position_rad = {geometry::neutral_hip_rad,
                                       geometry::neutral_knee_rad, 0.0F,
                                       geometry::neutral_hip_rad,
                                       geometry::neutral_knee_rad, 0.0F};
    MotorCommand far = command();
    far.position_rad = clamp_state.joints.position_rad;
    far.position_rad[0] += 2.0F;             // LH: 2 rad away.
    far.feedforward_nm[1] = 10.0F;           // LK: oversized feed-forward.
    far.torque_nm.fill(3.0F);
    assert(clamp_bridge.apply(far, clamp_state, 0) ==
           tap_fw::ControllerActuatorBridge::Result::Sent);
    bool saw_lh = false;
    bool saw_lk = false;
    for (const auto& frame : clamp_transport.can) {
      if ((frame.id & 0x1FU) != kMitControlCmdId) continue;
      const auto node = frame.id >> 5U;
      const auto mit = unpackMitCommand(frame.data);
      if (node == 3U) {  // LH, zero offset: driver q = sign * neutral hip.
        const float sign = robot_control::motor_mapping::kVerifiedLegCanBindings[0].driver_sign;
        const float driver_q = sign * geometry::neutral_hip_rad;
        const float torque = mit.kp * (mit.pos_rad - driver_q) + mit.tff_nm;
        assert(std::abs(torque) <= 3.0F + 0.05F);
        const float core_step = sign * (mit.pos_rad - driver_q);
        assert(core_step > 0.0F);   // Still toward the target.
        assert(core_step < 2.0F);
        saw_lh = true;
      }
      if (node == 4U) {  // LK, sign -1: T_ff clamped to the envelope.
        assert(std::abs(mit.tff_nm) <= 3.0F + 0.05F);
        const float q_driver = -geometry::neutral_knee_rad;
        const float torque = mit.kp * (mit.pos_rad - q_driver) + mit.tff_nm;
        assert(std::abs(torque) <= 3.0F + 0.05F);
        saw_lk = true;
      }
    }
    assert(saw_lh && saw_lk);
  }

  // Leg zero offsets: driver = sign * (core - offset).
  {
    FakeTransport offset_transport;
    tap_fw::ControllerActuatorBridge offset_bridge(offset_transport);
    JointArray offsets{};
    offsets[0] = 1.0F;   // LH, node 3, sign -1
    offsets[3] = 0.5F;   // RH, node 1, sign +1
    offset_bridge.setLegOffsets(&offsets);
    RobotState offset_state;
    offset_state.joints.position_rad = {1.2F, 0.0F, 0.0F, 0.7F, 0.0F, 0.0F};
    MotorCommand hold = command();
    hold.position_rad = offset_state.joints.position_rad;
    assert(offset_bridge.apply(hold, offset_state, 0) ==
           tap_fw::ControllerActuatorBridge::Result::Sent);
    for (const auto& frame : offset_transport.can) {
      if ((frame.id & 0x1FU) != kMitControlCmdId) continue;
      const auto node = frame.id >> 5U;
      const auto mit = unpackMitCommand(frame.data);
      if (node == 3U) assert(near(mit.pos_rad, -0.2F));
      if (node == 1U) assert(near(mit.pos_rad, 0.2F));
    }
  }

  // One MIT frame in flight per node: the answer uses the command's CAN id.
  {
    FakeTransport pace_transport;
    tap_fw::ControllerActuatorBridge::Config pace_config{};
    pace_config.mit_reply_timeout_us = 10'000U;
    tap_fw::ControllerActuatorBridge pace(pace_transport, pace_config);
    RobotState pace_state;
    const auto mitNodes = [&](std::size_t from) {
      std::vector<unsigned> nodes;
      for (std::size_t i = from; i < pace_transport.can.size(); ++i) {
        if ((pace_transport.can[i].id & 0x1FU) == kMitControlCmdId) {
          nodes.push_back(pace_transport.can[i].id >> 5U);
        }
      }
      return nodes;
    };
    assert(pace.apply(command(), pace_state, 0) == tap_fw::ControllerActuatorBridge::Result::Sent);
    assert(mitNodes(0).size() == 4U);
    std::size_t mark = pace_transport.can.size();
    assert(pace.apply(command(), pace_state, 4'000) == tap_fw::ControllerActuatorBridge::Result::Sent);
    assert(mitNodes(mark).empty());  // no answers yet: nothing re-sent
    assert(pace.mitDeferred() == 4U);
    CanFrame answer;
    answer.id = canId(3, kMitControlCmdId);
    answer.dlc = 6U;
    pace.onCanReceived(answer);
    mark = pace_transport.can.size();
    assert(pace.apply(command(), pace_state, 8'000) == tap_fw::ControllerActuatorBridge::Result::Sent);
    const auto only = mitNodes(mark);
    assert(only.size() == 1U && only[0] == 3U);
    mark = pace_transport.can.size();
    assert(pace.apply(command(), pace_state, 12'000) == tap_fw::ControllerActuatorBridge::Result::Sent);
    assert(mitNodes(mark).size() == 3U);  // the others timed out (10 ms)
  }

  // Drivers that realize only the Kp spring (T_ff ignored, as on the robot):
  // the target integral still brings a loaded joint onto its command, and the
  // spring never exceeds the torque envelope.
  {
    FakeTransport spring_transport;
    tap_fw::ControllerActuatorBridge::Config spring_config{};
    spring_config.leg_target_integral_per_s = tap_fw::board::kLegTargetIntegralPerS;
    spring_config.leg_fixed_kp_nm_per_rad = 0.0F;  // model stiffness path
    tap_fw::ControllerActuatorBridge spring(spring_transport, spring_config);
    RobotState legs;
    legs.joints.position_rad = {0.73F, -0.87F, 0.0F, 0.73F, -0.87F, 0.0F};
    MotorCommand stance = command();
    stance.position_rad = {0.99F, -1.146F, 0.0F, 0.99F, -1.146F, 0.0F};
    stance.velocity_rad_s.fill(0.0F);
    stance.feedforward_nm.fill(2.5F);  // ignored by these drivers
    stance.torque_nm.fill(3.0F);
    const float load[6] = {2.0F, -2.0F, 0.0F, 2.0F, -2.0F, 0.0F};  // resists the move
    float worst_spring = 0.0F;
    for (std::uint64_t t = 0; t < 4'000'000U; t += 4'000U) {
      const std::size_t from = spring_transport.can.size();
      (void)spring.apply(stance, legs, t);
      for (std::size_t i = from; i < spring_transport.can.size(); ++i) {
        const auto& frame = spring_transport.can[i];
        if ((frame.id & 0x1FU) != kMitControlCmdId) continue;
        const auto node = frame.id >> 5U;
        const auto mit = unpackMitCommand(frame.data);
        for (const auto& leg : robot_control::motor_mapping::kVerifiedLegCanBindings) {
          if (leg.node_id != node) continue;
          const std::size_t j = leg.joint_index;
          const float target = leg.driver_sign * mit.pos_rad;  // zero offsets
          const float spring_nm = mit.kp * (target - legs.joints.position_rad[j]);
          worst_spring = std::max(worst_spring, std::abs(spring_nm));
          legs.joints.position_rad[j] += (spring_nm - load[j]) / 2.0F * 0.004F;
        }
      }
    }
    const float err_hip = std::abs(legs.joints.position_rad[0] - 0.99F);
    const float err_knee = std::abs(legs.joints.position_rad[1] + 1.146F);
    assert(err_hip < 0.01F && err_knee < 0.01F);
    assert(worst_spring <= 3.0F + 0.05F);
    assert(spring.legTargetIntegral()[0] > 0.1F);  // offset carries the load
  }

  // Two-wheel lock (default board profile): one fixed Kp for every leg joint,
  // no target integral, target sent exactly as commanded.
  {
    FakeTransport lock_transport;
    tap_fw::ControllerActuatorBridge lock(lock_transport);
    RobotState legs;
    legs.joints.position_rad = {0.30F, 0.20F, 0.0F, 0.30F, 0.20F, 0.0F};  // pushed off
    MotorCommand hold = command();
    hold.position_rad = {0.05F, 0.46F, 0.0F, 0.05F, 0.46F, 0.0F};
    hold.torque_nm.fill(tap_fw::board::kLegLockTorqueNm);
    for (std::uint64_t t = 0; t < 400'000U; t += 4'000U) (void)lock.apply(hold, legs, t);
    bool saw = false;
    for (const auto& frame : lock_transport.can) {
      if ((frame.id & 0x1FU) != kMitControlCmdId) continue;
      const auto mit = unpackMitCommand(frame.data);
      if (tap_fw::board::kLegsLockedAtZeroPose) {
        assert(near(mit.kp, tap_fw::board::kLegLockKpNmPerRad, 0.2F));
        assert(near(mit.kd, tap_fw::board::kLegLockKdNmSPerRad, 0.01F));
      }
      saw = true;
    }
    assert(saw);
    for (std::size_t j : {0U, 1U, 3U, 4U}) assert(lock.legTargetIntegral()[j] == 0.0F);
  }

  // Kp cap: the straight (hanging) leg maps to hip Kp ~43 before the cap.
  {
    FakeTransport cap_transport;
    tap_fw::ControllerActuatorBridge::Config cap_config{};
    cap_config.leg_fixed_kp_nm_per_rad = 0.0F;  // model stiffness path
    tap_fw::ControllerActuatorBridge cap_bridge(cap_transport, cap_config);
    RobotState straight;
    straight.joints.position_rad = {0.0945F, 0.4584F, 0.0F, 0.0945F, 0.4584F, 0.0F};
    MotorCommand hold = command();
    hold.position_rad = straight.joints.position_rad;
    hold.torque_nm.fill(3.0F);
    assert(cap_bridge.apply(hold, straight, 0) ==
           tap_fw::ControllerActuatorBridge::Result::Sent);
    bool saw = false;
    for (const auto& frame : cap_transport.can) {
      if ((frame.id & 0x1FU) != kMitControlCmdId) continue;
      const auto mit = unpackMitCommand(frame.data);
      assert(mit.kp <= tap_fw::board::kLegMaxKpNmPerRad + 0.05F);
      saw = true;
    }
    assert(saw);
  }

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
  // The BLE endpoint's stance target; FixtureHold folds the legs to it.
  control.target_joint_position_rad = state.joints.position_rad;
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
