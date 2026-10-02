#include "tap_fw/v3/LegLock.hpp"

#include <algorithm>
#include <cmath>

#include "robot_hal/MotorFrameCodec.hpp"

namespace tap_fw::v3 {
namespace frame = robot_control::motor_frame;

namespace {
constexpr float kTwoPi = 6.28318530717958647692F;
}

bool LegLock::engage(const std::array<float, kLegs>& start_turns, std::uint64_t now_us,
                     motor_transport::Transport& transport) noexcept {
  start_turns_ = start_turns;
  engage_us_ = now_us;
  engaged_ = true;
  bool ok = true;
  const auto legs = board::MotorIds{}.leg_can;
  knee_mit_ = {};
  for (std::size_t i = 0; i < kLegs; ++i) {
    const auto node = legs[i].node_id;
    const std::int32_t input_mode = jointMit(i) ? frame::kInputModeMit
                                                : frame::kInputModePassthrough;
    ok = transport.sendCan(frame::canId(node, frame::kSetControllerModeCmdId),
                           frame::packSetControllerMode(frame::kControlModePosition,
                                                        input_mode)) && ok;
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
  return start_turns_[leg] * (1.0F - blend) + offset_turns;
}

float LegLock::velFfTurns(std::size_t leg) const noexcept {
  if (leg >= kLegs) return 0.0F;
  const auto binding = board::MotorIds{}.leg_can[leg];
  const bool knee = binding.joint_index == 1U || binding.joint_index == 4U;
  const std::size_t side = binding.joint_index < 3U ? 0U : 1U;
  const float rate_core = knee ? side_knee_rate_[side] : side_hip_rate_[side];
  return rate_core * binding.driver_sign * config_.gear_ratio / kTwoPi;
}

bool LegLock::jointMit(std::size_t leg) const noexcept {
  if (leg >= kLegs) return false;
  const auto binding = board::MotorIds{}.leg_can[leg];
  const bool knee = binding.joint_index == 1U || binding.joint_index == 4U;
  return config_.mode == Mode::Mit || (knee && config_.knee_mit);
}

bool LegLock::atZero(std::uint64_t now_us) const noexcept {
  return engaged_ && (config_.ramp_s <= 0.0F ||
      (now_us > engage_us_ &&
       static_cast<float>(now_us - engage_us_) * 1.0e-6F >= config_.ramp_s));
}

bool LegLock::sendTargets(std::uint64_t now_us,
                          motor_transport::Transport& transport) noexcept {
  last_send_us_ = now_us;
  bool ok = true;
  const auto legs = board::MotorIds{}.leg_can;
  for (std::size_t i = 0; i < kLegs; ++i) {
    const float turns = targetTurns(i, now_us);
    const bool mit = jointMit(i);
    const auto id = frame::canId(legs[i].node_id, mit ? frame::kMitControlCmdId
                                                      : frame::kSetInputPosCmdId);
    const bool knee = legs[i].joint_index == 1U || legs[i].joint_index == 4U;
    if (mit && knee && config_.knee_mit) {
      // Knee VMC: per-side gains; torque and rate from the core frame.
      const auto& m = knee_mit_[legs[i].joint_index < 3U ? 0U : 1U];
      const float joint_rad = turns * kTwoPi / config_.gear_ratio;
      ok = transport.sendCan(id, frame::packMitCommand(joint_rad, legs[i].driver_sign * m.vel_rad_s,
                                                       m.kp, m.kd, legs[i].driver_sign * m.tff_nm)) && ok;
    } else if (!mit) {
      // vel_ff in milli-turns/s (int16): only the roll side rate, 0 otherwise.
      const float vff = std::clamp(velFfTurns(i) * 1000.0F, -32767.0F, 32767.0F);
      ok = transport.sendCan(id, frame::packSetInputPos(
                                     turns, static_cast<std::int16_t>(std::lround(vff)), 0)) && ok;
    } else {
      // MIT works in joint radians (after the 8:1 gear), driver frame.
      const float joint_rad = turns * kTwoPi / config_.gear_ratio;
      ok = transport.sendCan(id, frame::packMitCommand(joint_rad, 0.0F,
                                                       config_.mit_kp_nm_per_rad,
                                                       config_.mit_kd_nm_s_per_rad,
                                                       0.0F)) && ok;
    }
  }
  return ok;
}

bool LegLock::update(std::uint64_t now_us, motor_transport::Transport& transport) noexcept {
  if (!engaged_) return true;
  if (now_us - last_send_us_ < config_.resend_period_us) return true;
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

const char* toString(LegLock::Mode mode) noexcept {
  return mode == LegLock::Mode::Position ? "POS" : "MIT";
}

}  // namespace tap_fw::v3
