#include "robot_control/LandingReflex.hpp"

#include <algorithm>
#include <cmath>

namespace robot_control {

namespace {
constexpr auto kLh = static_cast<std::size_t>(JointId::LeftHip);
constexpr auto kLk = static_cast<std::size_t>(JointId::LeftKnee);
constexpr auto kRh = static_cast<std::size_t>(JointId::RightHip);
constexpr auto kRk = static_cast<std::size_t>(JointId::RightKnee);
}  // namespace

LandingReflex::LandingReflex() : LandingReflex(Config{}) {}

void LandingReflex::reset() {
  phase_ = Phase::Grounded;
  prev_floor_contact_ = true;
  latched_hip_offset_rad_ = 0.0F;
  catch_elapsed_s_ = 0.0F;
  airborne_elapsed_s_ = 0.0F;
}

JointArray LandingReflex::update(bool floor_contact, float pitch_rate_rad_s,
                                 float dt_s) {
  JointArray offset{};
  if (!std::isfinite(pitch_rate_rad_s) || !std::isfinite(dt_s) ||
      dt_s <= 0.0F) {
    // Invalid input: stay put rather than latch a phase off bad data.
    return offset;
  }

  const bool liftoff_edge = !floor_contact && prev_floor_contact_;
  prev_floor_contact_ = floor_contact;

  if (liftoff_edge && phase_ != Phase::Airborne) {
    phase_ = Phase::Airborne;
    latched_hip_offset_rad_ =
        std::clamp(config_.hip_offset_per_pitch_rate * pitch_rate_rad_s,
                   -config_.max_hip_offset_rad, config_.max_hip_offset_rad);
    catch_elapsed_s_ = 0.0F;
    airborne_elapsed_s_ = 0.0F;
  } else if (phase_ == Phase::Airborne) {
    airborne_elapsed_s_ += dt_s;
    // Level check, not edge-triggered: a spuriously-true contact reading
    // this soon after liftoff (sim_pose heuristic noise, see
    // min_airborne_before_touchdown_s above) must not consume the one
    // false->true edge and get stuck Airborne forever once rejected - retry
    // every tick floor_contact reads true until either the debounce elapses
    // (real touchdown, transition to Catching) or contact drops again
    // (confirmed still airborne, no state change either way).
    if (floor_contact &&
        airborne_elapsed_s_ >= config_.min_airborne_before_touchdown_s) {
      phase_ = Phase::Catching;
      catch_elapsed_s_ = 0.0F;
    }
  }

  float knee_offset = 0.0F;
  float hip_offset = 0.0F;
  if (phase_ == Phase::Airborne) {
    knee_offset = -std::abs(config_.airborne_knee_crouch_rad);
    hip_offset = latched_hip_offset_rad_;
  } else if (phase_ == Phase::Catching) {
    catch_elapsed_s_ += dt_s;
    const float hold = std::max(config_.catch_hold_s, 0.0F);
    const float blend = std::max(config_.catch_blend_s, 1.0e-3F);
    float scale = 1.0F;
    if (catch_elapsed_s_ > hold) {
      scale = std::clamp(1.0F - (catch_elapsed_s_ - hold) / blend, 0.0F, 1.0F);
    }
    knee_offset = -std::abs(config_.airborne_knee_crouch_rad) * scale;
    hip_offset = latched_hip_offset_rad_ * scale;
    if (scale <= 0.0F) {
      phase_ = Phase::Grounded;
    }
  }

  offset[kLh] = hip_offset;
  offset[kRh] = hip_offset;
  offset[kLk] = knee_offset;
  offset[kRk] = knee_offset;
  return offset;
}

}  // namespace robot_control
