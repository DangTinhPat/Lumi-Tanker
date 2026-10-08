#pragma once

namespace tap_fw::v3 {

// Height planner: leg poses as a function of the hip-to-axle height.
//
// Leg zero is the gravity-hang pose the operator arranges before powering
// the motors. A leg measured in that pose read knee -1.069 rad against the CAD
// natural pose -1.068, so the hang pose is knee = 0 in the CAD frame, with the
// hip at the angle that puts the axle straight below the hip (x = 0):
// hip = +0.388 rad CAD, 188.6 mm hip-axle.
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
  // Deepest fold allowed: -1.38 rad, about 78 mm hip-axle. The CAD knee
  // envelope (-1.2 rad) is a simulation value, not a measured stop; the robot
  // has reached 78 mm. The physics plant balances at CoM 43 % of the hang height.
  static constexpr float kKneeMinRad = -1.38F;

  // Pose for a knee fold (clamped to [kKneeMinRad, 0]).
  [[nodiscard]] static LegPose forKnee(float knee_offset_rad) noexcept;
  // Pose for a hip-to-axle height (m); clamped to the allowed range.
  [[nodiscard]] static LegPose forHeight(float axle_below_hip_m) noexcept;
  [[nodiscard]] static float maxHeight() noexcept;  // hang pose
  [[nodiscard]] static float minHeight() noexcept;  // at kKneeMinRad
  // CAD hip angle of the hang pose (hip offset 0): core offsets convert to
  // CAD joint angles as hip_cad = hangHipCad() + hip, knee_cad = knee.
  [[nodiscard]] static float hangHipCad() noexcept;
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
