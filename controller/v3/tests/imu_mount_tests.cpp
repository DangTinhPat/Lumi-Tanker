// PCB IMU mounting: chip Y forward, chip X right, chip Z up.

#include <cassert>
#include <cmath>
#include <cstdio>

#include "tap_fw/BoardConfig.hpp"

int main() {
  using namespace tap_fw;
  const auto& map = board::kImuAxisMap;
  static_assert(imu_mount::isRotation(board::kImuAxisMap));
  // A left-handed reading of the chip (X left and Y forward, Z up) is rejected.
  constexpr imu_mount::AxisMap mirror{{1U, 0U, 2U}, {1.0F, 1.0F, 1.0F}};
  static_assert(!imu_mount::isRotation(mirror));

  // Nose-down by 0.3 rad about body +Y: body accel = (-g sin, 0, g cos).
  // Chip Y (forward) reads -g sin; chip X (right) reads 0.
  const float g = 9.80665F;
  const float th = 0.3F;
  const auto body = imu_mount::toBody(map, {0.0F, -g * std::sin(th), g * std::cos(th)});
  const float pitch = std::atan2(-body[0], body[2]);
  assert(std::abs(pitch - th) < 1.0e-5F);
  assert(std::abs(body[1]) < 1.0e-6F);

  // Nose-down rotation rate about body +Y (left) = about chip -X.
  const auto rate = imu_mount::toBody(map, {-0.5F, 0.0F, 0.0F});
  assert(rate[1] == 0.5F && rate[0] == 0.0F && rate[2] == 0.0F);
  // Yaw about chip Z (up) is unchanged; left side down (roll) = chip Y rate.
  assert(imu_mount::toBody(map, {0.0F, 0.0F, 0.7F})[2] == 0.7F);
  assert(imu_mount::toBody(map, {0.0F, 0.2F, 0.0F})[0] == 0.2F);
  std::puts("imu_mount_tests: all checks passed");
  return 0;
}
