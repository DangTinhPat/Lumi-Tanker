#pragma once

#include <cmath>

namespace tap_fw::v3 {

// Excitation schedules shared by the firmware joint tests (AppV3 HIPTEST /
// KNEETEST) and the host sims that check them before they run on the robot.

// Stand (wheels off): 0-2 s steps +A, 0, -A, 0 (0.5 s each), then 1/2/4/8 Hz
// sines for 1 s each, 6-6.5 s back at 0.
inline constexpr float kStandTestDurationS = 6.5F;
inline float standTestOffset(float t, float a) {
  constexpr float kTwoPi = 6.28318530718F;
  if (t < 0.5F) return a;
  if (t < 1.0F) return 0.0F;
  if (t < 1.5F) return -a;
  if (t < 2.0F) return 0.0F;
  if (t < 6.0F) {
    const float seg = t - 2.0F;
    const int k = static_cast<int>(seg);  // 0..3 -> 1, 2, 4, 8 Hz
    const float f = static_cast<float>(1 << k);
    return a * std::sin(kTwoPi * f * (seg - static_cast<float>(k)));
  }
  return 0.0F;
}

// Floor (balancing, KNEETEST FLOOR): 0.5 s moving the swing centre in, then
// whole cycles of 0.5 Hz x1, 1 Hz x2, 2 Hz x3, 3 Hz x4, then 0.5 s moving
// the centre back out. Whole cycles start and end at the centre and the
// centre moves on a half-cosine, so the offset never steps (a 0.05 rad knee
// step at the start tilted the CoM line 0.08 rad in the two-body sim). The
// amplitude falls as 1/f above 1 Hz: the legs move the CoM faster than the
// wheels follow, and at a full 0.05 rad the CoM line swung 0.058 rad at 2 Hz
// and 0.071 at 3 Hz (sim, 189 mm) - close to the 0.08 abort.
inline constexpr float kFloorTestDurationS = 0.5F + 2.0F + 2.0F + 1.5F + 4.0F / 3.0F + 0.5F;
inline float floorTestOffset(float t, float a, float centre) {
  constexpr float kTwoPi = 6.28318530718F;
  constexpr float kPi = 3.14159265359F;
  constexpr float kFreq[4] = {0.5F, 1.0F, 2.0F, 3.0F};
  constexpr float kCycles[4] = {1.0F, 2.0F, 3.0F, 4.0F};
  if (t < 0.0F || t >= kFloorTestDurationS) return 0.0F;
  if (t < 0.5F) return centre * 0.5F * (1.0F - std::cos(kPi * t / 0.5F));
  float t0 = 0.5F;
  for (int i = 0; i < 4; ++i) {
    const float len = kCycles[i] / kFreq[i];
    const float amp = a * (kFreq[i] > 1.0F ? 1.0F / kFreq[i] : 1.0F);
    if (t < t0 + len) return centre + amp * std::sin(kTwoPi * kFreq[i] * (t - t0));
    t0 += len;
  }
  return centre * 0.5F * (1.0F + std::cos(kPi * (t - t0) / 0.5F));
}

}  // namespace tap_fw::v3
