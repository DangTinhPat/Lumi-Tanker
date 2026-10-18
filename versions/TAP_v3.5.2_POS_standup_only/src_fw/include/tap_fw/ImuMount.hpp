#pragma once

#include <array>
#include <cstddef>

namespace tap_fw::imu_mount {

// MPU6500 mounting on the PCB. The controller consumes the robot body frame
// (+X forward, +Y left, +Z up; pitch = rotation about +Y, nose-down positive).
// body[i] = sign[i] * chip[chip_axis[i]], for both accelerometer and gyro.
struct AxisMap {
  std::array<std::size_t, 3> chip_axis;
  std::array<float, 3> sign;
};

// Chip axes as mounted (2026-09-26: the PCB rotates the IMU 90 deg about Z;
// chip Z stays up). A physical MPU6500 is right-handed, so with chip Z up
// only these two readings of "X left, Y forward" exist:
//   chip Y forward, chip X right -> {{1, 0, 2}, {+1, -1, +1}}
//   chip X left,   chip Y back   -> {{1, 0, 2}, {-1, +1, +1}}
inline constexpr AxisMap kChipYForwardXRight{{1U, 0U, 2U}, {1.0F, -1.0F, 1.0F}};
inline constexpr AxisMap kChipXLeftYBack{{1U, 0U, 2U}, {-1.0F, 1.0F, 1.0F}};
inline constexpr AxisMap kIdentity{{0U, 1U, 2U}, {1.0F, 1.0F, 1.0F}};

// Determinant of the signed permutation: +1 for a rotation. A mirror (-1)
// would make gyro pitch and accelerometer pitch disagree in sign.
constexpr float determinant(const AxisMap& map) {
  float m[3][3]{};
  for (std::size_t i = 0; i < 3U; ++i) m[i][map.chip_axis[i]] = map.sign[i];
  return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
         m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
         m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

constexpr bool isRotation(const AxisMap& map) {
  const auto& a = map.chip_axis;
  const bool permutation = a[0] < 3U && a[1] < 3U && a[2] < 3U &&
      a[0] != a[1] && a[1] != a[2] && a[0] != a[2];
  for (float s : map.sign) {
    if (s != 1.0F && s != -1.0F) return false;
  }
  return permutation && determinant(map) == 1.0F;
}

static_assert(isRotation(kChipYForwardXRight));
static_assert(isRotation(kChipXLeftYBack));
static_assert(isRotation(kIdentity));

inline std::array<float, 3> toBody(const AxisMap& map,
                                   const std::array<float, 3>& chip) noexcept {
  return {map.sign[0] * chip[map.chip_axis[0]],
          map.sign[1] * chip[map.chip_axis[1]],
          map.sign[2] * chip[map.chip_axis[2]]};
}

}  // namespace tap_fw::imu_mount
