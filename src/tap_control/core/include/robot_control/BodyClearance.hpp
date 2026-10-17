#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "robot_control/GeneratedBodyEnvelope.hpp"
#include "robot_control/WholeBodyModel.hpp"

namespace robot_control {

// Flat-floor body-clearance lower estimate from IMU attitude and leg encoders.
// Wheels cannot penetrate the floor: this bounds the root height from below.
// Ignoring cylinder half-width is conservative. Body component boxes bound the
// CAD hulls from outside; they never create contacts or change simulator physics.
// Accuracy still depends on attitude, CAD, wheel deformation and floor slope.
inline float bodyClearance(float pitch, float roll,
                           const std::array<float, 2>& hip_cad,
                           const std::array<float, 2>& knee_cad,
                           float radius) noexcept {
  if (!std::isfinite(pitch) || !std::isfinite(roll) ||
      !std::isfinite(radius) || radius <= 0.0F)
    return std::numeric_limits<float>::quiet_NaN();
  const std::array<float, 3> up{
      -std::sin(pitch), std::cos(pitch) * std::sin(roll),
      std::cos(pitch) * std::cos(roll)};
  const float extent = radius * std::sqrt(std::max(0.0F, 1.0F - up[1] * up[1]));
  float root_lower = -std::numeric_limits<float>::infinity();
  for (unsigned side = 0; side < 2; ++side) {
    if (!std::isfinite(hip_cad[side]) || !std::isfinite(knee_cad[side]))
      return std::numeric_limits<float>::quiet_NaN();
    const auto leg = WholeBodyModel::legHeight(hip_cad[side], knee_cad[side]);
    const auto& hip = geometry::origins[3 * side];
    const float axle_y = hip.y + geometry::origins[3 * side + 1].y +
                         geometry::origins[3 * side + 2].y;
    const float axle_up = up[0] * (hip.x + leg.x_m) + up[1] * axle_y +
                          up[2] * (hip.z + leg.height_m);
    root_lower = std::max(root_lower, extent - axle_up);
  }
  float body_lower = std::numeric_limits<float>::infinity();
  for (const auto& box : geometry::body_envelope) {
    float lower = 0.0F;
    for (unsigned axis = 0; axis < 3; ++axis)
      lower += up[axis] * (up[axis] >= 0.0F ? box.lo[axis] : box.hi[axis]);
    body_lower = std::min(body_lower, lower);
  }
  return root_lower + body_lower;
}

}  // namespace robot_control
