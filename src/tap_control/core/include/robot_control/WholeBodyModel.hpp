#pragma once

#include "robot_control/GeneratedGeometry.hpp"

namespace robot_control {

// Exact planar FK from the seven current CAD-link inertials. Both legs use
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
    // Angle of the axle-to-CoM line in the body frame. The virtual
    // pendulum state is IMU body pitch plus this geometry angle.
    float com_line_angle_rad = 0.0F;
    float pitch_inertia_at_axle_kg_m2 = 0.0F;
    float axle_height_difference_m = 0.0F;
  };

  struct SymmetricComJacobian {
    float d_com_offset_x_d_hip = 0.0F;
    float d_com_offset_x_d_knee = 0.0F;
    float d_com_above_axle_d_hip = 0.0F;
    float d_com_above_axle_d_knee = 0.0F;
  };

  struct LegGravityTorque {
    float hip_nm = 0.0F;
    float knee_nm = 0.0F;
  };

  // Single-leg axle height AND fore/aft position above the hip origin
  // (body-frame z, x) and their exact analytic partial derivatives w.r.t.
  // hip/knee angle, from the same 2-link planar chain compute() already
  // walks (hip -> rotate by hip -> rotate by hip+knee+rebase). Identical for both
  // legs: native +Y axes make the thigh/shin x,z offsets side-symmetric
  // (only their y offset differs, which this planar FK never uses). The
  // 2x2 Jacobian [[d_height_d_hip, d_height_d_knee], [d_x_d_hip,
  // d_x_d_knee]] lets stance and landing controllers solve coordinated
  // (hip, knee) motion while holding the axle's fore/aft coordinate fixed.
  struct LegHeight {
    float height_m = 0.0F;
    float d_height_d_hip = 0.0F;
    float d_height_d_knee = 0.0F;
    float x_m = 0.0F;
    float d_x_d_hip = 0.0F;
    float d_x_d_knee = 0.0F;
  };

  struct SymmetricPose {
    float hip_rad = geometry::neutral_hip_rad;
    float knee_rad = geometry::neutral_knee_rad;
    float body_height_m = geometry::neutral_spawn_z_m;
    float axle_x_m = 0.0F;
    float residual_m = 0.0F;
    bool feasible = false;
  };
  [[nodiscard]] static LegHeight legHeight(float hip, float knee) noexcept;

  // Analytic derivative of the symmetric CAD pose's COM offset relative to
  // the mean wheel axle.  This is used by the embedded stance trim instead of
  // finite differences, so its cost and numerical behaviour are deterministic
  // on the ESP32 target.
  [[nodiscard]] static SymmetricComJacobian symmetricComOffsetJacobian(
      float hip, float knee) noexcept;

  // Quasi-static gravity torque of the measured distal leg links.  This is a
  // link-gravity feed-forward term, not a full body/contact inverse-dynamics
  // result: body load sharing and joint friction remain calibration items.
  [[nodiscard]] static LegGravityTorque legGravityTorque(
      float hip, float knee, float gravity_m_s2 = 9.81F) noexcept;

  // Quasi-static body-load share transmitted through one wheel axle. This is
  // intentionally separate from legGravityTorque(): TorqueResidualEstimator
  // models distal-link gravity, while stance feed-forward may add the CAD/BOM
  // body load split without changing that residual reference.
  [[nodiscard]] static LegGravityTorque bodySupportTorque(
      float hip, float knee, float gravity_m_s2 = 9.81F) noexcept;

  // Solve the sagittal two-link FK for a symmetric body height while holding
  // the natural ENABLE axle fore/aft coordinate.  This is the native height
  // command path: it does not duplicate link lengths in the GUI and it
  // rejects targets that cannot be reached inside the supplied joint
  // envelope.  The returned residual is the Euclidean FK error in metres.
  [[nodiscard]] static SymmetricPose solveSymmetricHeight(
      float body_height_m,
      float hip_seed_rad = geometry::neutral_hip_rad,
      float knee_seed_rad = geometry::neutral_knee_rad,
      float hip_min_rad = geometry::native_hip_min_rad,
      float hip_max_rad = geometry::native_hip_max_rad,
      float knee_min_rad = geometry::native_knee_min_rad,
      float knee_max_rad = geometry::native_knee_max_rad) noexcept;

  // Solve a symmetric pose at a specified knee angle.  It is used to derive
  // the admissible height range from the same FK and to avoid GUI-side
  // geometry copies.
  [[nodiscard]] static SymmetricPose solveSymmetricKnee(
      float knee_rad,
      float hip_seed_rad = geometry::neutral_hip_rad,
      float hip_min_rad = geometry::native_hip_min_rad,
      float hip_max_rad = geometry::native_hip_max_rad,
      float knee_min_rad = geometry::native_knee_min_rad,
      float knee_max_rad = geometry::native_knee_max_rad) noexcept;

  WholeBodyModel() = default;
  explicit WholeBodyModel(Config config) : config_(config) {}
  [[nodiscard]] Result compute(float left_hip, float left_knee,
                               float right_hip, float right_knee) const;
  [[nodiscard]] static float equilibriumPitch(const Result& r) noexcept;

 private:
  Config config_;
};
}  // namespace robot_control
