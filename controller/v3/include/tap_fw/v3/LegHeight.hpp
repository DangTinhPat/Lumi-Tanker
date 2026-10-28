#pragma once

namespace tap_fw::v3 {

// Height planner: leg poses as a function of the hip-to-axle height.
//
// GIM CAD zero is the 22 degree lower knee stop. The core knee coordinate is
// the internal angle increase from this pose, hence [0, 113 degrees].
struct LegPose {
  float knee_offset_rad = 0.0F;   // core knee angle relative to CAD zero (>= 0)
  float hip_offset_rad = 0.0F;    // core hip angle relative to zero
  float axle_below_hip_m = 0.0F;  // hip-to-axle vertical distance
  float com_above_axle_m = 0.0F;  // whole-body CoM height above the axle
  // Change of the balance (equilibrium) pitch versus the power-up pose,
  // from the CAD mass model: feed-forward for the balance reference.
  float balance_pitch_delta_rad = 0.0F;
  bool ok = false;
};

// Joint offsets for a wheel axle target relative to its hip. The target
// plane is horizontal in the gravity frame; +X is forward, depth is down.
struct LegAxleTarget {
  float hip_offset_rad = 0.0F;
  float knee_offset_rad = 0.0F;
  float x_world_m = 0.0F;
  bool limited = false;
  bool ok = false;
};

class LegHeight {
 public:
  static constexpr float kKneeMinRad = 0.0F;
  static constexpr float kKneeMaxRad = 1.9722222222222222F;

  // Pose for a knee angle relative to CAD zero (clamped to [0, 113 degrees]).
  [[nodiscard]] static LegPose forKnee(float knee_offset_rad) noexcept;
  // Pose for a hip-to-axle height (m); clamped to the allowed range.
  [[nodiscard]] static LegPose forHeight(float axle_below_hip_m) noexcept;
  [[nodiscard]] static float maxHeight() noexcept;  // at the 135 deg knee limit
  [[nodiscard]] static float minHeight() noexcept;  // at CAD knee zero (22 deg)
  // CAD hip angle that places the axle directly below the hip at the requested knee.
  [[nodiscard]] static float hangHipCad() noexcept;  // GIM CAD hip zero at knee 22 deg
  // Cartesian IK, height first: project X into the supported knee workspace
  // at the requested depth, then solve BOTH joints on the existing branch.
  // pitch uses the native +Y convention R(p): (cx+sz, -sx+cz).
  [[nodiscard]] static LegAxleTarget forAxleOnPlane(
      float x_world_m, float depth_world_m, float body_pitch_rad) noexcept;
  // Hip offset that puts the axle at x_world for a knee that is NOT the exact
  // IK knee (a filtered one): X stays exact, the depth follows the knee. Equals
  // forAxleOnPlane's hip when the knee is its solution.
  [[nodiscard]] static float hipForKneeAtX(
      float knee_offset_rad, float x_world_m, float body_pitch_rad) noexcept;
};

}  // namespace tap_fw::v3
