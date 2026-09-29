#include "tap_fw/v3/RockerLower.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "robot_control/WholeBodyModel.hpp"

namespace tap_fw::v3 {

float RockerLower::nToP(float hip_rad, float knee_rad, float pitch_rad) noexcept {
  const auto body = robot_control::WholeBodyModel{}.compute(hip_rad, knee_rad,
                                                            hip_rad, knee_rad);
  const float radius = std::hypot(body.com_offset_x_m, body.com_above_axle_m);
  return radius * std::sin(pitch_rad + body.com_line_angle_rad);
}

void RockerLower::reset(float hip_rad, float knee_rad) noexcept {
  phase_ = Phase::Lowering;
  hip_cmd_rad_ = hip_rad;
  knee_hold_rad_ = knee_rad;
  best_n_to_p_m_ = std::numeric_limits<float>::infinity();
}

RockerLower::Output RockerLower::update(const Input& in) noexcept {
  Output out{};
  out.n_to_p_m = nToP(in.hip_rad, in.knee_rad, in.pitch_rad);
  if (phase_ == Phase::Lowering || phase_ == Phase::Flinging) {
    if (!in.sensors_ok || !std::isfinite(out.n_to_p_m) ||
        in.pitch_rad + in.hip_rad < config_.leg_angle_min_rad) {
      phase_ = Phase::Aborted;
      hip_cmd_rad_ = in.hip_rad;
    } else if (phase_ == Phase::Lowering) {
      // Slow fold until the floor, or earlier if N-P turns back up.
      best_n_to_p_m_ = std::min(best_n_to_p_m_, std::abs(out.n_to_p_m));
      hip_cmd_rad_ = std::max(hip_cmd_rad_ - config_.hip_rate_rad_s * in.dt_s,
                              config_.hip_floor_rad);
      // Judge the command, not the encoder: the loaded hip servo sits
      // 0.05-0.07 rad short of its target.
      if (std::abs(out.n_to_p_m) > best_n_to_p_m_ + config_.rise_margin_m ||
          hip_cmd_rad_ <= config_.hip_floor_rad) {
        phase_ = Phase::Flinging;
      }
    } else {
      hip_cmd_rad_ = std::max(hip_cmd_rad_ - config_.fling_rate_rad_s * in.dt_s,
                              config_.fling_lock_rad);
      if (hip_cmd_rad_ <= config_.fling_lock_rad) phase_ = Phase::Pivot;
    }
  }
  out.phase = phase_;
  out.hip_target_rad = hip_cmd_rad_;
  out.knee_target_rad = knee_hold_rad_;
  out.wheel_rad_s = phase_ == Phase::Lowering ? config_.wheel_roll_rad_s : 0.0F;
  return out;
}

const char* toString(RockerLower::Phase phase) noexcept {
  switch (phase) {
    case RockerLower::Phase::Lowering: return "lowering";
    case RockerLower::Phase::Flinging: return "fling";
    case RockerLower::Phase::Pivot: return "pivot";
    case RockerLower::Phase::Aborted: return "aborted";
  }
  return "?";
}

}  // namespace tap_fw::v3
