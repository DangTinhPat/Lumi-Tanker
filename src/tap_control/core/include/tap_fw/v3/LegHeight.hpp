#pragma once

#include "robot_control/GeneratedGeometry.hpp"

namespace tap_fw::v3 {

// v3.1 height planner for the locked-leg pendulum.
//
// Leg zero is the gravity-hang pose the operator arranges before powering
// the motors. On 2026-09-26 the right leg moved from that pose to the CAD
// natural pose read knee -1.069 rad against CAD -1.068, so the hang pose is
// knee = 0 in the CAD frame, with the hip at the angle that puts the axle
// straight below the hip (x = 0): hip = +0.388 rad CAD, 188.6 mm hip-axle.
//
// Lowering the body folds the knee (core knee folding is negative) and turns
// the hip to keep the axle straight below the hip, as Impulse keeps its foot
// at X = 0. The body (3.37 of 4.62 kg) then stays over the axle and the
// wheels only absorb a small residual. Results are joint OFFSETS from each
// leg's own power-up zero (core frame, rad), so hand-arranged legs that are
// only roughly equal still move by the same amount.
struct LegPose {
  float knee_offset_rad = 0.0F;   // core knee angle relative to zero (<= 0)
  float hip_offset_rad = 0.0F;    // core hip angle relative to zero
  float axle_below_hip_m = 0.0F;  // hip-to-axle vertical distance
  float com_above_axle_m = 0.0F;  // whole-body CoM height above the axle
  // Change of the balance (equilibrium) pitch versus the power-up pose,
  // from the CAD mass model: feed-forward for the balance reference.
  float balance_pitch_delta_rad = 0.0F;
  bool ok = false;
};

class LegHeight {
 public:
  // Keep the V3.1 height path inside TPA's current native simulation envelope.
  // This preview limit is not a measured hardware stop.
  static constexpr float kKneeMinRad = robot_control::geometry::native_knee_min_rad;

  // Pose for a knee fold (clamped to [kKneeMinRad, 0]).
  [[nodiscard]] static LegPose forKnee(float knee_offset_rad) noexcept;
  // Pose for a hip-to-axle height (m); clamped to the allowed range.
  [[nodiscard]] static LegPose forHeight(float axle_below_hip_m) noexcept;
  [[nodiscard]] static float maxHeight() noexcept;  // hang pose
  [[nodiscard]] static float minHeight() noexcept;  // at kKneeMinRad
};

}  // namespace tap_fw::v3
