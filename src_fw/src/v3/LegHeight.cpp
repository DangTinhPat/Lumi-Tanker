#include "tap_fw/v3/LegHeight.hpp"

#include <algorithm>
#include <cmath>

#include "robot_control/WholeBodyModel.hpp"

namespace tap_fw::v3 {
namespace {

using robot_control::WholeBodyModel;

// CAD hip angle that puts the axle straight below the hip for a knee angle.
float hipForAxleBelow(float knee_cad, float hip_seed) noexcept {
  float hip = hip_seed;
  for (int i = 0; i < 30; ++i) {
    const auto leg = WholeBodyModel::legHeight(hip, knee_cad);
    if (std::abs(leg.x_m) < 1.0e-7F || std::abs(leg.d_x_d_hip) < 1.0e-6F) break;
    hip -= leg.x_m / leg.d_x_d_hip;
  }
  return hip;
}

struct Reference {
  float hip_cad;
  float com_above_axle_m;
  float balance_pitch_rad;
};

Reference hangReference() noexcept {
  Reference r{};
  r.hip_cad = hipForAxleBelow(0.0F, 0.4F);
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
  // Hip moves about 0.52 rad per rad of knee fold; seed from that.
  const float hip = hipForAxleBelow(knee, ref.hip_cad - 0.52F * knee);
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

float LegHeight::maxHeight() noexcept { return forKnee(0.0F).axle_below_hip_m; }
float LegHeight::minHeight() noexcept { return forKnee(kKneeMinRad).axle_below_hip_m; }

LegPose LegHeight::forHeight(float axle_below_hip_m) noexcept {
  if (!std::isfinite(axle_below_hip_m)) return LegPose{};
  const float target = std::clamp(axle_below_hip_m, minHeight(), maxHeight());
  // Height falls monotonically as the knee folds: bisect on the knee.
  float lo = kKneeMinRad, hi = 0.0F;
  for (int i = 0; i < 40; ++i) {
    const float mid = 0.5F * (lo + hi);
    if (forKnee(mid).axle_below_hip_m > target) hi = mid; else lo = mid;
  }
  return forKnee(0.5F * (lo + hi));
}

}  // namespace tap_fw::v3
