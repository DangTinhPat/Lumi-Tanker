#include "tap_fw/MotorFeedback.hpp"

#include <cmath>
#include <limits>

#include "robot_control/GeneratedGeometry.hpp"
#include "robot_control/WheelFrame.hpp"
#include "robot_hal/MotorFrameCodec.hpp"
#include "robot_hal/MotorMapping.hpp"

namespace tap_fw {
namespace {

namespace frame = robot_control::motor_frame;
namespace mapping = robot_control::motor_mapping;

constexpr std::array<std::size_t, 4> kLegJoints{{0U, 1U, 3U, 4U}};
constexpr std::array<std::size_t, 2> kWheelJoints{{2U, 5U}};
constexpr std::uint16_t kWheelEncoderCounts = 1024U;
constexpr std::int32_t kWheelHalfTurn = kWheelEncoderCounts / 2;
constexpr float kWheelRadiansPerCount =
    6.28318530717958647692F / static_cast<float>(kWheelEncoderCounts);

float neutralPose(std::size_t joint) noexcept {
  return joint == 0U || joint == 3U ? robot_control::geometry::neutral_hip_rad
                                    : robot_control::geometry::neutral_knee_rad;
}

}  // namespace

bool MotorFeedback::poll(motor_transport::Transport& transport,
                         std::uint64_t now_us,
                         bool legs_enabled) noexcept {
  bool ok = true;
  if (!legs_enabled && (!can_probe_initialized_ ||
      now_us - last_can_probe_us_ >= board::kCanProbePeriodUs)) {
    can_probe_initialized_ = true;
    last_can_probe_us_ = now_us;
    const auto zero_torque_probe = frame::packMitCommand(0.0F, 0.0F, 0.0F,
                                                          0.0F, 0.0F);
    for (const auto& leg : board::MotorIds{}.leg_can) {
      ok = transport.sendCan(frame::canId(leg.node_id, frame::kMitControlCmdId),
                             zero_torque_probe) && ok;
    }
  }

  if (now_us - last_vbus_poll_us_ >= board::kBusVoltagePollUs) {
    last_vbus_poll_us_ = now_us;
    const auto legs = board::MotorIds{}.leg_can;
    vbus_poll_slot_ = (vbus_poll_slot_ + 1U) % legs.size();
    motor_transport::CanFrame request{};
    request.id = frame::canId(legs[vbus_poll_slot_].node_id,
                              frame::kGetBusVoltageCurrentCmdId);
    request.dlc = 0U;
    ok = transport.sendCanFrame(request) && ok;
  }

  if (!wheel_poll_initialized_ ||
      now_us - last_wheel_poll_us_ >= board::kWheelFeedbackPeriodUs) {
    wheel_poll_initialized_ = true;
    last_wheel_poll_us_ = now_us;
    const auto bindings = board::MotorIds{}.wheel_uart;
    for (std::size_t bus = 0U; bus < bindings.size(); ++bus) {
      ok = transport.sendUart(static_cast<std::uint8_t>(bus),
                              frame::buildReadPresent(bindings[bus].motor_id)) && ok;
    }
  }
  return ok;
}

bool MotorFeedback::ingestCan(const motor_transport::CanFrame& input,
                              std::uint64_t now_us) noexcept {
  const auto node_id = static_cast<std::uint8_t>(input.id >> 5U);
  const auto command_id = static_cast<std::uint8_t>(input.id & 0x1FU);
  const auto binding = [&]() -> const mapping::LegCanBinding* {
    for (const auto& item : mapping::kVerifiedLegCanBindings) {
      if (item.node_id == node_id) return &item;
    }
    return nullptr;
  }();
  if (binding == nullptr || input.dlc > 8U) return false;

  // ODrive heartbeat: axis_error u32 LE, axis_state u8.
  if (command_id == frame::kHeartbeatCmdId) {
    if (input.dlc < 5U) return false;
    auto& axis = axis_[binding->joint_index];
    axis.error = static_cast<std::uint32_t>(input.data[0]) |
        (static_cast<std::uint32_t>(input.data[1]) << 8U) |
        (static_cast<std::uint32_t>(input.data[2]) << 16U) |
        (static_cast<std::uint32_t>(input.data[3]) << 24U);
    axis.state = input.data[4];
    axis.stamp_us = now_us;
    axis.valid = true;
    return true;
  }

  if (command_id == frame::kGetEncoderEstimatesCmdId) {
    if (input.dlc < 8U) return false;
    const auto est = frame::unpackEncoderEstimates(input.data);
    if (!std::isfinite(est.pos_turns) || !std::isfinite(est.vel_turns_s)) return false;
    auto& out = estimate_[binding->joint_index];
    out.pos_turns = est.pos_turns;
    out.vel_turns_s = est.vel_turns_s;
    out.stamp_us = now_us;
    out.valid = true;
    return true;
  }

  if (command_id == frame::kGetBusVoltageCurrentCmdId) {
    if (input.dlc < 4U) return false;
    const float volts = frame::unpackBusVoltageCurrent(input.data).voltage_v;
    if (!std::isfinite(volts)) return false;
    bus_voltage_min_v_ = bus_voltage_seen_ ? std::min(bus_voltage_min_v_, volts) : volts;
    bus_voltage_seen_ = true;
    return true;
  }

  if (command_id == frame::kGetIqCmdId) {
    if (input.dlc < 8U) return false;
    const auto iq = frame::unpackIqFeedback(input.data);
    if (!std::isfinite(iq.setpoint_a) || !std::isfinite(iq.measured_a)) return false;
    const std::size_t joint = binding->joint_index;
    current_a_[joint] = binding->driver_sign * iq.measured_a;
    current_stamp_us_[joint] = now_us;
    current_valid_[joint] = true;
    ++sequence_;
    return true;
  }

  // GDS68 MIT feedback uses bytes 0-5 only and arrives with DLC 6 on the
  // robot (verified 2026-09-25). Get_Encoder_Estimates (0x009) is not a
  // joint source: its scale and zero differ from MIT and are unverified, and
  // the drivers already answer the MIT probes while IDLE.
  if (command_id != frame::kMitControlCmdId || input.dlc < 6U) return false;
  return ingestMit(*binding, node_id, input, now_us);
}

std::size_t MotorFeedback::firstLegAxisFault(
    std::uint64_t now_us, bool require_closed_loop) const noexcept {
  for (const std::size_t joint : kLegJoints) {
    const auto& axis = axis_[joint];
    const bool stale = !axis.valid || now_us < axis.stamp_us ||
        now_us - axis.stamp_us > board::kHeartbeatMaxAgeUs;
    if (stale || axis.error != 0U ||
        (require_closed_loop && axis.state != frame::kAxisStateClosedLoop)) {
      return joint;
    }
  }
  return robot_control::kJointCount;
}

bool MotorFeedback::ingestMit(const mapping::LegCanBinding& binding,
                              std::uint8_t node_id,
                              const motor_transport::CanFrame& input,
                              std::uint64_t now_us) noexcept {
  const auto feedback = frame::unpackMitFeedback(input.data);
  if (feedback.node_id != node_id || !std::isfinite(feedback.pos_rad) ||
      !std::isfinite(feedback.vel_rad_s) || !std::isfinite(feedback.torque_nm)) {
    return false;
  }

  const std::size_t joint = binding.joint_index;
  const float uncalibrated = binding.driver_sign * feedback.pos_rad;
  if (legs_homed_ && leg_silent_[joint] &&
      std::abs(uncalibrated) < board::kLegRebootZeroRad &&
      std::abs(leg_uncalibrated_rad_[joint]) > board::kLegRebootMinTravelRad) {
    // Back from silence at ~0 after a non-zero position: driver rebooted.
    dropLegZero();
  }
  leg_silent_[joint] = false;
  leg_uncalibrated_rad_[joint] = uncalibrated;
  position_rad_[joint] = leg_uncalibrated_rad_[joint] + leg_offset_rad_[joint];
  velocity_rad_s_[joint] = binding.driver_sign * feedback.vel_rad_s;
  effort_nm_[joint] = binding.driver_sign * feedback.torque_nm;
  stamp_us_[joint] = now_us;
  valid_[joint] = true;
  ++sequence_;
  return true;
}

MotorFeedback::HomeResult MotorFeedback::homeLegs(std::uint64_t now_us) noexcept {
  robot_control::JointArray pose{};
  for (const std::size_t joint : kLegJoints) pose[joint] = neutralPose(joint);
  return homeLegsAt(pose, now_us);
}

MotorFeedback::HomeResult MotorFeedback::homeLegsAt(
    const robot_control::JointArray& pose, std::uint64_t now_us) noexcept {
  if (!allLegsFresh(now_us)) return HomeResult::StaleFeedback;
  for (const std::size_t joint : kLegJoints) {
    if (std::abs(velocity_rad_s_[joint]) > board::kHomingMaxJointSpeedRadS) {
      return HomeResult::Moving;
    }
  }
  for (const std::size_t joint : kLegJoints) {
    leg_offset_rad_[joint] = pose[joint] - leg_uncalibrated_rad_[joint];
    position_rad_[joint] = pose[joint];
  }
  legs_homed_ = true;
  return HomeResult::Homed;
}

void MotorFeedback::restoreLegOffsets(const robot_control::JointArray& offsets) noexcept {
  for (const std::size_t joint : kLegJoints) {
    leg_offset_rad_[joint] = offsets[joint];
    position_rad_[joint] = leg_uncalibrated_rad_[joint] + offsets[joint];
    leg_silent_[joint] = false;
  }
  legs_homed_ = true;
}

void MotorFeedback::dropLegZero() noexcept {
  legs_homed_ = false;
  leg_zero_dropped_ = true;
  for (const std::size_t leg : kLegJoints) {
    leg_offset_rad_[leg] = 0.0F;
    position_rad_[leg] = leg_uncalibrated_rad_[leg];
  }
}

bool MotorFeedback::superviseLegZero(std::uint64_t now_us) noexcept {
  if (legs_homed_) {
    for (const std::size_t joint : kLegJoints) {
      if (valid_[joint] && now_us >= stamp_us_[joint] &&
          now_us - stamp_us_[joint] <= board::kLegZeroLossUs) {
        continue;
      }
      // Silent: keep the zero, judge it when the leg talks again.
      leg_silent_[joint] = true;
      if (!valid_[joint] || now_us < stamp_us_[joint] ||
          now_us - stamp_us_[joint] > board::kLegZeroForgetUs) {
        dropLegZero();
        break;
      }
    }
  }
  const bool dropped = leg_zero_dropped_;
  leg_zero_dropped_ = false;
  return dropped;
}

bool MotorFeedback::ingestUart(std::uint8_t bus,
                              const frame::DxlFrame& input,
                              std::uint64_t now_us) noexcept {
  const auto bindings = board::MotorIds{}.wheel_uart;
  if (bus >= bindings.size()) return false;
  const auto parsed = frame::parseDxlResponse(input.bytes.data(), input.len);
  if (!parsed.ok || parsed.err != 0U ||
      parsed.id != bindings[bus].motor_id) return false;
  const auto present = frame::parsePresentPayload(parsed);
  if (!present.valid) return false;

  if (!wheel_position_seeded_[bus]) {
    wheel_previous_raw_[bus] = present.position_raw & 0x03FFU;
    wheel_position_seeded_[bus] = true;
  } else {
    const auto raw = static_cast<std::uint16_t>(present.position_raw & 0x03FFU);
    std::int32_t delta = static_cast<std::int32_t>(raw) -
                         static_cast<std::int32_t>(wheel_previous_raw_[bus]);
    if (delta > kWheelHalfTurn) delta -= kWheelEncoderCounts;
    if (delta < -kWheelHalfTurn) delta += kWheelEncoderCounts;
    wheel_unwrapped_rad_[bus] += static_cast<float>(delta) *
        kWheelRadiansPerCount * bindings[bus].driver_sign;
    wheel_previous_raw_[bus] = raw;
  }

  const std::size_t joint = bindings[bus].joint_index;
  position_rad_[joint] = wheel_unwrapped_rad_[bus];
  velocity_rad_s_[joint] = bindings[bus].driver_sign *
      frame::wheelRawSpeedToRadPerSec(present.speed_raw) *
      mapping::kWheelPresentSpeedScale;
  effort_nm_[joint] = 0.0F;
  stamp_us_[joint] = now_us;
  valid_[joint] = true;
  ++sequence_;
  return true;
}

void MotorFeedback::writeTo(robot_control::RobotState& state) const noexcept {
  state.joints.position_rad = position_rad_;
  state.joints.velocity_rad_s = velocity_rad_s_;
  state.joints.effort_nm = effort_nm_;
  state.joints.effort_feedback_valid = false;
  state.encoder_sample_sequence = sequence_;

  const auto observation = robot_control::WheelFrame::observe(
      state.joints, state.imu.pitch_rad, state.imu.pitch_rate_rad_s,
      robot_control::geometry::wheel_radius_m);
  if (!observation.valid) return;

  const float radius = robot_control::geometry::wheel_radius_m;
  const auto sideOdometry = [&](std::size_t hip, std::size_t wheel,
                                float& position, float& velocity) {
    const std::size_t knee = hip + 1U;
    position = radius * (state.joints.position_rad[wheel] +
        state.joints.position_rad[hip] + state.joints.position_rad[knee] +
        state.imu.pitch_rad);
    velocity = radius * (state.joints.velocity_rad_s[wheel] +
        state.joints.velocity_rad_s[hip] +
        state.joints.velocity_rad_s[knee] + state.imu.pitch_rate_rad_s);
  };
  sideOdometry(0U, 2U, state.wheel.left_position_m,
               state.wheel.left_velocity_m_s);
  sideOdometry(3U, 5U, state.wheel.right_position_m,
               state.wheel.right_velocity_m_s);
  state.wheel.position_m = observation.axle.position_m;
  state.wheel.velocity_m_s = observation.axle.velocity_m_s;
}

bool MotorFeedback::fresh(std::size_t joint,
                          std::uint64_t now_us) const noexcept {
  return valid_[joint] && now_us >= stamp_us_[joint] &&
      now_us - stamp_us_[joint] <= board::kFeedbackMaxAgeUs;
}

bool MotorFeedback::allLegsFresh(std::uint64_t now_us) const noexcept {
  for (const std::size_t joint : kLegJoints) {
    if (!fresh(joint, now_us)) return false;
  }
  return true;
}

bool MotorFeedback::allWheelsFresh(std::uint64_t now_us) const noexcept {
  for (const std::size_t joint : kWheelJoints) {
    if (!fresh(joint, now_us)) return false;
  }
  return true;
}

bool MotorFeedback::allWheelsFreshWithin(std::uint64_t now_us, std::uint32_t max_age_us) const noexcept {
  for (const std::size_t joint : kWheelJoints) {
    if (!valid_[joint] || now_us < stamp_us_[joint] || now_us - stamp_us_[joint] > max_age_us) return false;
  }
  return true;
}

std::uint32_t MotorFeedback::oldestWheelAgeUs(std::uint64_t now_us) const noexcept {
  std::uint64_t oldest = 0;
  for (const std::size_t joint : kWheelJoints) {
    if (!valid_[joint] || now_us < stamp_us_[joint]) return 0xFFFFFFFFU;
    oldest = std::max<std::uint64_t>(oldest, now_us - stamp_us_[joint]);
  }
  return static_cast<std::uint32_t>(std::min<std::uint64_t>(oldest, 0xFFFFFFFEU));
}

bool MotorFeedback::ready(std::uint64_t now_us) const noexcept {
  return allLegsFresh(now_us) && allWheelsFresh(now_us);
}

bool MotorFeedback::readyToHold(std::uint64_t now_us) const noexcept {
  for (const std::size_t joint : kLegJoints) {
    if (!valid_[joint] || now_us < stamp_us_[joint] ||
        now_us - stamp_us_[joint] > board::kLegRideThroughUs) {
      return false;
    }
  }
  return allWheelsFresh(now_us);
}

float MotorFeedback::takeMinBusVoltage() noexcept {
  const float out = bus_voltage_seen_ ? bus_voltage_min_v_
                                      : std::numeric_limits<float>::quiet_NaN();
  bus_voltage_seen_ = false;
  return out;
}

}  // namespace tap_fw
