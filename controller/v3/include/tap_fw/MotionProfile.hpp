#pragma once
// Speed profile of the finite moves (MOVE / TURN): constant cruise, then the speed that stops
// exactly at the target with a fixed deceleration, v = sqrt(2 a |remaining|). Pure function, so
// the host tests cover it; the closed loop around it lives in AppV3::updateMotion.
#include <algorithm>
#include <cmath>

namespace tap_fw {

// Signed speed toward the target. remaining = target - progress (sign = direction to go).
inline float approachSpeed(float remaining, float cruise, float decel) noexcept {
  const float v = std::min(std::fabs(cruise), std::sqrt(2.0F * std::max(decel, 1.0e-3F) * std::fabs(remaining)));
  return std::copysign(v, remaining);
}

}  // namespace tap_fw
