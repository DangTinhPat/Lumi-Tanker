// v3.1 height planner: offsets from the gravity-hang zero keep the axle
// straight below the hip; height falls monotonically with the knee fold.
#include <cassert>
#include <cmath>
#include <cstdio>

#include "robot_control/WholeBodyModel.hpp"
#include "tap_fw/v3/LegHeight.hpp"

using tap_fw::v3::LegHeight;
using tap_fw::v3::LegPose;

int main() {
  // Power-up pose: no offsets, 188.6 mm hip-axle (CAD).
  const LegPose hang = LegHeight::forKnee(0.0F);
  assert(hang.ok);
  assert(std::abs(hang.knee_offset_rad) < 1.0e-6F && std::abs(hang.hip_offset_rad) < 1.0e-5F);
  assert(std::abs(hang.axle_below_hip_m - 0.1886F) < 0.001F);
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
    // Body stays over the axle: the balance pitch moves by < 0.07 rad over
    // the allowed range.
    assert(p.balance_pitch_delta_rad >= -1.0e-4F && p.balance_pitch_delta_rad < 0.07F);
  }
  // Clamped at both ends.
  assert(std::abs(LegHeight::forKnee(0.5F).knee_offset_rad) < 1.0e-6F);
  assert(std::abs(LegHeight::forKnee(-2.0F).knee_offset_rad - LegHeight::kKneeMinRad) < 1.0e-6F);

  // Height request round-trips through the knee.
  for (float h : {0.185F, 0.17F, 0.155F, 0.14F, 0.12F}) {
    const LegPose p = LegHeight::forHeight(h);
    assert(p.ok && std::abs(p.axle_below_hip_m - h) < 2.0e-4F);
  }
  assert(std::abs(LegHeight::forHeight(1.0F).axle_below_hip_m - LegHeight::maxHeight()) < 1.0e-5F);
  assert(std::abs(LegHeight::forHeight(0.0F).axle_below_hip_m - LegHeight::minHeight()) < 1.0e-5F);

  std::printf("leg_height_tests: range %.1f-%.1f mm, all checks passed\n",
              LegHeight::minHeight() * 1000.0F, LegHeight::maxHeight() * 1000.0F);
  return 0;
}
