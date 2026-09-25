#include "tap_fw/MotorFeedback.hpp"

#include <cmath>

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
  if (input.dlc != 8U) return false;
  const auto node_id = static_cast<std::uint8_t>(input.id >> 5U);
  const auto command_id = static_cast<std::uint8_t>(input.id & 0x1FU);
  if (command_id != frame::kMitControlCmdId) return false;

  const auto binding = [&]() -> const mapping::LegCanBinding* {
    for (const auto& item : mapping::kVerifiedLegCanBindings) {
      if (item.node_id == node_id) return &item;
    }
    return nullptr;
  }();
  if (binding == nullptr) return false;

  const auto feedback = frame::unpackMitFeedback(input.data);
  if (feedback.node_id != node_id || !std::isfinite(feedback.pos_rad) ||
      !std::isfinite(feedback.vel_rad_s) || !std::isfinite(feedback.torque_nm)) {
    return false;
  }

  const std::size_t joint = binding->joint_index;
  position_rad_[joint] = binding->driver_sign * feedback.pos_rad;
  velocity_rad_s_[joint] = binding->driver_sign * feedback.vel_rad_s;
  effort_nm_[joint] = binding->driver_sign * feedback.torque_nm;
  stamp_us_[joint] = now_us;
  valid_[joint] = true;
  ++sequence_;
  return true;
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
      frame::wheelRawSpeedToRadPerSec(present.speed_raw);
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

bool MotorFeedback::ready(std::uint64_t now_us) const noexcept {
  return allLegsFresh(now_us) && allWheelsFresh(now_us);
}

}  // namespace tap_fw
