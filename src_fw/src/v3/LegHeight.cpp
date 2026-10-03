#include "tap_fw/v3/LegHeight.hpp"

#include <algorithm>
#include <cmath>

#include "robot_control/WholeBodyModel.hpp"
#include "robot_control/GeneratedGeometry.hpp"

namespace tap_fw::v3 {
namespace {

using robot_control::WholeBodyModel;

// CAD hip angle that puts the axle straight below the hip for a knee angle.
float hipForAxleBelow(float knee_cad) noexcept {
  // The hip rotates the entire two-link vector. Its argument gives the
  // exact downward solution; no iterative FK inside the 500 Hz loop.
  const auto leg = WholeBodyModel::legHeight(0.0F, knee_cad);
  return std::atan2(leg.x_m, -leg.height_m);
}

struct Reference {
  float hip_cad;
  float com_above_axle_m;
  float balance_pitch_rad;
};

Reference hangReference() noexcept {
  Reference r{};
  r.hip_cad = hipForAxleBelow(0.0F);
  const auto body = WholeBodyModel{}.compute(r.hip_cad, 0.0F, r.hip_cad, 0.0F);
  r.com_above_axle_m = body.com_above_axle_m;
  r.balance_pitch_rad = WholeBodyModel::equilibriumPitch(body);
  return r;
}

}  // namespace

LegPose LegHeight::forKnee(float knee_offset_rad) noexcept {
  LegPose out{};
  if (!std::isfinite(knee_offset_rad)) return out;
  static const Reference ref = hangReference();
  const float knee = std::clamp(knee_offset_rad, kKneeMinRad, 0.0F);
  const float hip = hipForAxleBelow(knee);
  const auto leg = WholeBodyModel::legHeight(hip, knee);
  const auto body = WholeBodyModel{}.compute(hip, knee, hip, knee);
  out.knee_offset_rad = knee;
  out.hip_offset_rad = hip - ref.hip_cad;
  out.axle_below_hip_m = -leg.height_m;
  out.com_above_axle_m = body.com_above_axle_m;
  out.balance_pitch_delta_rad =
      WholeBodyModel::equilibriumPitch(body) - ref.balance_pitch_rad;
  out.ok = std::abs(leg.x_m) < 1.0e-4F;
  return out;
}

float LegHeight::hangHipCad() noexcept {
  static const Reference ref = hangReference();
  return ref.hip_cad;
}

LegAxleTarget LegHeight::forAxleOnPlane(float x_world_m, float depth_world_m,
                                      float body_pitch_rad) noexcept {
  LegAxleTarget out{};
  if (!std::isfinite(x_world_m) || !std::isfinite(depth_world_m) ||
      !std::isfinite(body_pitch_rad) || depth_world_m <= 0.0F) return out;
  const float h = std::min(depth_world_m, maxHeight());
  const float xmin = std::sqrt(std::max(minHeight() * minHeight() - h * h, 0.0F));
  const float xmax = std::sqrt(std::max(maxHeight() * maxHeight() - h * h, 0.0F));
  const float x = std::copysign(std::clamp(std::abs(x_world_m), xmin, xmax), x_world_m);
  out.limited = std::abs(x - x_world_m) > 1.0e-6F || h != depth_world_m;

  // Specify the end-effector target before solving joints (Pinocchio IK
  // example: https://docs.ros.org/en/rolling/p/pinocchio/doc/b-examples/d-inverse-kinematics.html).
  // TAP's planar two-link chain admits an analytic solve. Use the actual
  // CAD vectors and knee rebase, not nominal supplier lengths or a mirrored
  // right leg. The supported knee branch is monotonic in radius.
  namespace geo = robot_control::geometry;
  const float ax = geo::origins[1].x, az = geo::origins[1].z;
  const float bx = geo::origins[2].x, bz = geo::origins[2].z;
  const float a = std::hypot(ax, az), b = std::hypot(bx, bz);
  const float cos_delta = std::clamp((x*x + h*h - a*a - b*b) / (2.0F*a*b), -1.0F, 1.0F);
  const float knee = std::clamp(std::atan2(bz, bx) - std::atan2(az, ax) -
      geo::knee_zero_rebase_rad - std::acos(cos_delta), kKneeMinRad, 0.0F);
  const auto zero_hip = WholeBodyModel::legHeight(0.0F, knee);
  const float hip_below = std::atan2(zero_hip.height_m, zero_hip.x_m) + 1.57079632679F;
  out.hip_offset_rad = hip_below - hangHipCad() - std::atan2(x, h) - body_pitch_rad;
  out.knee_offset_rad = knee;
  out.x_world_m = x;
  out.ok = std::isfinite(out.hip_offset_rad) && std::isfinite(knee);
  return out;
}

// Constant for the model: computed once (they were a full forKnee each call,
// several times per control tick).
float LegHeight::maxHeight() noexcept {
  static const float h = forKnee(0.0F).axle_below_hip_m;
  return h;
}
float LegHeight::minHeight() noexcept {
  static const float h = forKnee(kKneeMinRad).axle_below_hip_m;
  return h;
}

LegPose LegHeight::forHeight(float axle_below_hip_m) noexcept {
  if (!std::isfinite(axle_below_hip_m)) return LegPose{};
  const float target = std::clamp(axle_below_hip_m, minHeight(), maxHeight());
  // At X=0 the requested height is the two-link radius. Use the same
  // supported branch as forAxleOnPlane, rather than repeatedly computing
  // the whole robot COM during a secant search. Roll calls this twice per
  // tick and the hybrid suspension once even at a fixed operator height.
  namespace geo = robot_control::geometry;
  const float ax = geo::origins[1].x, az = geo::origins[1].z;
  const float bx = geo::origins[2].x, bz = geo::origins[2].z;
  const float a2 = ax*ax + az*az, b2 = bx*bx + bz*bz;
  const float cosine = std::clamp((target*target-a2-b2) /
      (2.0F*std::sqrt(a2*b2)), -1.0F, 1.0F);
  const float knee = std::atan2(bz,bx)-std::atan2(az,ax) -
      geo::knee_zero_rebase_rad-std::acos(cosine);
  return forKnee(knee);
}

}  // namespace tap_fw::v3
