#include "tap_fw/v3/LegLock.hpp"

#include <algorithm>
#include <cmath>

#include "robot_hal/MotorFrameCodec.hpp"
#include "robot_control/WholeBodyModel.hpp"
#include "tap_fw/v3/CurrentTorque.hpp"
#include "tap_fw/v3/LegHeight.hpp"

namespace tap_fw::v3 {
namespace frame = robot_control::motor_frame;

namespace {
constexpr float kTwoPi = 6.28318530717958647692F;
}

bool LegLock::engage(const std::array<float, kLegs>& start_turns, std::uint64_t now_us,
                     motor_transport::Transport& transport, bool ramp_to_zero) noexcept {
  ramp_to_zero_ = ramp_to_zero;
  start_turns_ = start_turns;
  engage_us_ = now_us;
  engaged_ = true;
  next_send_us_ = now_us + config_.resend_period_us;
  bool ok = true;
  const auto legs = board::MotorIds{}.leg_can;
  for (std::size_t i = 0; i < kLegs; ++i) {
    const auto node = legs[i].node_id;
    ok = transport.sendCan(frame::canId(node, frame::kSetLimitsCmdId),
                           frame::packSetLimits(config_.velocity_limit_turns_s,
                                                config_.current_limit_a)) && ok;
  }
  for (std::size_t i = 0; i < kLegs; ++i) {
    const auto node = legs[i].node_id;
    ok = transport.sendCan(frame::canId(node, frame::kSetControllerModeCmdId),
                           frame::packSetControllerMode(requiresCurrentFeedback()
                                                            ? frame::kControlModeTorque
                                                            : frame::kControlModePosition,
                                                        frame::kInputModePassthrough)) && ok;
  }
  // The first set-point equals the measured pose, so closing the loop does
  // not snap the leg anywhere; the ramp then walks it to 0.
  ok = sendTargets(now_us, transport) && ok;
  for (std::size_t i = 0; i < kLegs; ++i) {
    ok = transport.sendCan(frame::canId(legs[i].node_id, frame::kSetAxisStateCmdId),
                           frame::packSetAxisState(frame::kAxisStateClosedLoop)) && ok;
  }
  return ok;
}

float LegLock::targetTurns(std::size_t leg, std::uint64_t now_us) const noexcept {
  if (leg >= kLegs) return 0.0F;
  // Stand-up supplies the measured pose as the offsets and owns the path: close the driver loop at each
  // joint's own measured position (the two sides need not be symmetric), no homing ramp on top.
  if (!ramp_to_zero_ && now_us <= engage_us_) return start_turns_[leg];
  const float elapsed = now_us > engage_us_
      ? static_cast<float>(now_us - engage_us_) * 1.0e-6F : 0.0F;
  const float tau = config_.ramp_s > 0.0F
      ? std::clamp(elapsed / config_.ramp_s, 0.0F, 1.0F) : 1.0F;
  // Minimum-jerk blend: zero speed and acceleration at both ends.
  const float blend = tau * tau * tau * (10.0F + tau * (-15.0F + 6.0F * tau));
  // Height offsets (core rad) in driver turns: hip joints 0/3, knees 1/4.
  const auto binding = board::MotorIds{}.leg_can[leg];
  const bool knee = binding.joint_index == 1U || binding.joint_index == 4U;
  const std::size_t side = binding.joint_index < 3U ? 0U : 1U;  // LH LK | RH RK
  const float offset_core = knee ? knee_offset_rad_ + side_knee_rad_[side]
                                 : hip_offset_rad_ + side_hip_rad_[side];
  const float offset_turns = offset_core * binding.driver_sign * config_.gear_ratio / kTwoPi;
  return ramp_to_zero_ ? start_turns_[leg] * (1.0F - blend) + offset_turns : offset_turns;
}

float LegLock::velFfTurns(std::size_t leg) const noexcept {
  if (leg >= kLegs) return 0.0F;
  const auto binding = board::MotorIds{}.leg_can[leg];
  const bool knee = binding.joint_index == 1U || binding.joint_index == 4U;
  const std::size_t side = binding.joint_index < 3U ? 0U : 1U;
  const float rate_core = knee ? common_knee_rate_ + side_knee_rate_[side]
                               : common_hip_rate_ + side_hip_rate_[side];
  return rate_core * binding.driver_sign * config_.gear_ratio / kTwoPi;
}

bool LegLock::atZero(std::uint64_t now_us) const noexcept {
  return engaged_ && (!ramp_to_zero_ || config_.ramp_s <= 0.0F ||
      (now_us > engage_us_ &&
       static_cast<float>(now_us - engage_us_) * 1.0e-6F >= config_.ramp_s));
}

bool LegLock::sendTargets(std::uint64_t now_us,
                          motor_transport::Transport& transport) noexcept {
  last_send_us_ = now_us;
  bool ok = true;
  const auto legs = board::MotorIds{}.leg_can;
  if (!requiresCurrentFeedback()) {
    for (std::size_t i = 0; i < kLegs; ++i) {
      // Protocol units: rotor turns, 0.001 rotor turns/s, 0.001 Nm.
      // Zero torque feed-forward keeps the POS path independent of Kt/Iq.
      const float velocity_milli = std::clamp(velFfTurns(i) * 1000.0F,
                                             -32767.0F, 32767.0F);
      ok = transport.sendCan(frame::canId(legs[i].node_id, frame::kSetInputPosCmdId),
          frame::packSetInputPos(targetTurns(i, now_us),
              static_cast<std::int16_t>(std::lround(velocity_milli)), 0)) && ok;
    }
    return ok;
  }
  std::array<float, kLegs> gravity_torque_nm{};
  for (std::size_t side = 0; side < 2U; ++side) {
    const std::size_t hip = side == 0U ? 0U : 2U;
    const std::size_t knee = hip + 1U;
    const float hip_sign = legs[hip].driver_sign;
    const float knee_sign = legs[knee].driver_sign;
    const float hip_cad = LegHeight::hangHipCad() +
        hip_sign * feedback_position_rad_[hip];
    const float knee_cad = knee_sign * feedback_position_rad_[knee];
    const auto leg_gravity = robot_control::WholeBodyModel::legGravityTorque(
        hip_cad, knee_cad);
    const auto body_gravity = robot_control::WholeBodyModel::bodySupportTorque(
        hip_cad, knee_cad);
    gravity_torque_nm[hip] = hip_sign * (leg_gravity.hip_nm + body_gravity.hip_nm);
    gravity_torque_nm[knee] = knee_sign * (leg_gravity.knee_nm + body_gravity.knee_nm);
  }
  for (std::size_t i = 0; i < kLegs; ++i) {
    const float turns = targetTurns(i, now_us);
    // Torque mode uses output-joint Nm. A stale Iq sample suppresses the command.
    const float target_rad = turns * kTwoPi / config_.gear_ratio;
    const float target_rate = velFfTurns(i) * kTwoPi / config_.gear_ratio;
    const float kp = config_.joint_kp_nm_per_rad * kp_scale_[i % 2U];
    const float kd = config_.joint_kd_nm_s_per_rad * kd_scale_[i % 2U];
    const float requested_nm = gravity_torque_nm[i] + kp *
            (target_rad - feedback_position_rad_[i]) +
        kd * (target_rate - feedback_velocity_rad_s_[i]);
    const auto current = currentTorqueCommand(requested_nm, feedback_current_a_[i],
        feedback_valid_[i], config_.kt_nm_per_a, config_.current_limit_a,
        config_.current_comp_gain);
    ok = transport.sendCan(frame::canId(legs[i].node_id, frame::kSetInputTorqueCmdId),
                           frame::packSetInputTorque(current.torque_nm)) && ok;
  }
  return ok;
}

bool LegLock::update(std::uint64_t now_us, motor_transport::Transport& transport) noexcept {
  if (!engaged_) return true;
  if (now_us < next_send_us_) return true;
  // Anchor to the deadline, not the actual send time: now+3 ms on a 2 ms
  // clock would silently remain 250 Hz. Skip missed cycles after a stall.
  const auto period = std::max(config_.resend_period_us, 1U);
  next_send_us_ += period;
  if (now_us >= next_send_us_) next_send_us_ = now_us + period;
  return sendTargets(now_us, transport);
}

bool LegLock::release(motor_transport::Transport& transport) noexcept {
  engaged_ = false;
  bool ok = true;
  for (const auto& leg : board::MotorIds{}.leg_can) {
    ok = transport.sendCan(frame::canId(leg.node_id, frame::kSetAxisStateCmdId),
                           frame::packSetAxisState(frame::kAxisStateIdle)) && ok;
  }
  return ok;
}

}  // namespace tap_fw::v3
