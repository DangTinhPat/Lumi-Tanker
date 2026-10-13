// The motor zero is the extended GIM pose; CAD knee zero is the lower stop.
#include <cassert>
#include <cmath>
#include <cstdio>

#include "robot_control/WholeBodyModel.hpp"
#include "tap_fw/v3/LegHeight.hpp"

using tap_fw::v3::LegHeight;
using tap_fw::v3::LegPose;

int main() {
  // Power-up pose is the 135 degree CAD upper stop, not CAD joint zero.
  const LegPose hang = LegHeight::forKnee(0.0F);
  assert(hang.ok);
  assert(std::abs(hang.knee_offset_rad) < 1.0e-6F && std::abs(hang.hip_offset_rad) < 1.0e-5F);
  assert(std::abs(LegHeight::cadKnee(0.0F) - 1.9722222F) < 1.0e-6F);
  assert(std::abs(hang.axle_below_hip_m - 0.18477F) < 0.0001F);
  assert(std::abs(LegHeight::forKnee(LegHeight::kKneeMinRad).axle_below_hip_m - 0.03816F) < 0.0001F);
  assert(std::abs(hang.balance_pitch_delta_rad) < 1.0e-5F);

  float last_height = hang.axle_below_hip_m + 1.0F;
  for (float knee = 0.0F; knee >= LegHeight::kKneeMinRad - 1.0e-6F; knee -= 0.05F) {
    const LegPose p = LegHeight::forKnee(knee);
    assert(p.ok);                           // axle straight below the hip
    assert(p.axle_below_hip_m < last_height);  // monotonic
    last_height = p.axle_below_hip_m;
    // Folding the knee (negative) turns the hip positive (foot toward the
    // tail) by about half the fold.
    if (knee < -0.01F) {
      const float ratio = -p.hip_offset_rad / knee;
      assert(ratio > 0.45F && ratio < 0.6F);
    }
    // The CAD COM shift is finite and monotonic as the leg folds.
    assert(p.balance_pitch_delta_rad >= -1.0e-4F && p.balance_pitch_delta_rad < 0.20F);
  }
  // Clamped at both ends.
  assert(std::abs(LegHeight::forKnee(0.5F).knee_offset_rad) < 1.0e-6F);
  assert(std::abs(LegHeight::forKnee(-2.0F).knee_offset_rad - LegHeight::kKneeMinRad) < 1.0e-6F);

  // Height request round-trips through the knee.
  for (float h : {0.184F, 0.17F, 0.155F, 0.14F, 0.12F, 0.095F, 0.08F}) {
    const LegPose p = LegHeight::forHeight(h);
    assert(p.ok && std::abs(p.axle_below_hip_m - h) < 2.0e-4F);
  }
  // Cartesian targets must agree with independent CAD FK after translating
  // the motor-relative knee offset to the CAD joint coordinate.
  for (float h : {0.08F, 0.12F, 0.17F}) {
    for (float x : {-0.02F, 0.0F, 0.02F}) {
      for (float pitch : {-0.1F, 0.0F, 0.1F}) {
        const auto q = LegHeight::forAxleOnPlane(x, h, pitch);
        assert(q.ok && !q.limited);
        const auto fk = robot_control::WholeBodyModel::legHeight(
            LegHeight::hangHipCad() + q.hip_offset_rad, LegHeight::cadKnee(q.knee_offset_rad));
        const float world_x = std::cos(pitch) * fk.x_m + std::sin(pitch) * fk.height_m;
        const float world_z = -std::sin(pitch) * fk.x_m + std::cos(pitch) * fk.height_m;
        assert(std::abs(world_x - x) < 1.0e-5F);
        assert(std::abs(world_z + h) < 1.0e-5F);
      }
    }
  }
  assert(std::abs(LegHeight::forHeight(1.0F).axle_below_hip_m - LegHeight::maxHeight()) < 1.0e-5F);
  assert(std::abs(LegHeight::forHeight(0.0F).axle_below_hip_m - LegHeight::minHeight()) < 1.0e-5F);

  std::printf("leg_height_tests: range %.1f-%.1f mm, all checks passed\n",
              LegHeight::minHeight() * 1000.0F, LegHeight::maxHeight() * 1000.0F);
  return 0;
}
