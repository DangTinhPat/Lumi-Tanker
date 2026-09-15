#pragma once

#include "robot_control/GeneratedGeometry.hpp"

namespace robot_control {

// Exact planar FK from the seven current URDF link inertials. Both legs use
// native +Y axes. No polynomial fit or mirrored Upkie joint convention.
class WholeBodyModel {
 public:
  struct Config {
    float wheel_radius_m = geometry::wheel_radius_m;
  };
  struct Result {
    float com_offset_x_m = 0.0F;
    float com_height_m = 0.217302F;
    float com_above_axle_m = 0.187302F;
    float pitch_inertia_at_axle_kg_m2 = 0.0F;
    float axle_height_difference_m = 0.0F;
  };

  // Single-leg axle height AND fore/aft position above the hip origin
  // (body-frame z, x) and their exact analytic partial derivatives w.r.t.
  // hip/knee angle, from the same 2-link planar chain compute() already
  // walks (hip -> rotate by hip -> rotate by hip+knee). Identical for both
  // legs: native +Y axes make the thigh/shin x,z offsets side-symmetric
  // (only their y offset differs, which this planar FK never uses). The
  // 2x2 Jacobian [[d_height_d_hip, d_height_d_knee], [d_x_d_hip,
  // d_x_d_knee]] lets LandingImpedance solve for a coordinated (hip, knee)
  // motion that changes leg height by a target amount while holding the
  // foot's fore/aft contact position fixed - a real squat, not an
  // independent per-joint offset.
  struct LegHeight {
    float height_m = 0.0F;
    float d_height_d_hip = 0.0F;
    float d_height_d_knee = 0.0F;
    float x_m = 0.0F;
    float d_x_d_hip = 0.0F;
    float d_x_d_knee = 0.0F;
  };
  [[nodiscard]] static LegHeight legHeight(float hip, float knee) noexcept;

  WholeBodyModel() = default;
  explicit WholeBodyModel(Config config) : config_(config) {}
  [[nodiscard]] Result compute(float left_hip, float left_knee,
                               float right_hip, float right_knee) const;
  [[nodiscard]] static float equilibriumPitch(const Result& r) noexcept;

 private:
  Config config_;
};
}  // namespace robot_control
