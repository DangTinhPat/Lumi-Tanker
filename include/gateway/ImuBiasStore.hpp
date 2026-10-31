#pragma once
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include "gateway/hcr/hcr_protocol.h"

namespace gateway {
// Gyro bias measured by CAL_IMU, kept across resets. A bad record is ignored,
// never trusted: boot then falls back to the manual CAL_IMU path.
struct ImuBiasRecord {
  static constexpr std::uint32_t kMagic = 0x42554D49U;  // "IMUB"
  static constexpr std::uint16_t kVersion = 1;
  // CAL_IMU accepts |gyro| < .35 rad/s, so a larger stored value is corrupt.
  static constexpr float kMaxBiasRadS = .5F;

  std::uint32_t magic = 0;
  std::uint16_t version = 0, reserved = 0;
  float bias[3] = {0, 0, 0};
  std::uint32_t crc = 0;

  static ImuBiasRecord make(const std::array<float, 3>& bias) noexcept {
    ImuBiasRecord r;
    r.magic = kMagic; r.version = kVersion;
    for (unsigned i = 0; i < 3; ++i) r.bias[i] = bias[i];
    r.crc = checksum(r);
    return r;
  }
  bool valid() const noexcept {
    if (magic != kMagic || version != kVersion || reserved != 0 || crc != checksum(*this)) return false;
    for (float b : bias)
      if (!std::isfinite(b) || std::abs(b) > kMaxBiasRadS) return false;
    return true;
  }
  std::array<float, 3> value() const noexcept { return {bias[0], bias[1], bias[2]}; }

 private:
  static std::uint32_t checksum(const ImuBiasRecord& r) noexcept {
    return hcr_crc32(reinterpret_cast<const std::uint8_t*>(&r), offsetof(ImuBiasRecord, crc));
  }
};
static_assert(sizeof(ImuBiasRecord) == 24, "ImuBiasRecord is stored as raw bytes");
}  // namespace gateway

#ifdef ARDUINO
#include <Preferences.h>
namespace gateway {
// NVS namespace "hcr" survives OTA; only a full-chip erase clears it.
inline bool loadImuBias(std::array<float, 3>& bias) noexcept {
  Preferences prefs;
  if (!prefs.begin("hcr", true)) return false;
  ImuBiasRecord r;
  const bool ok = prefs.getBytesLength("imu_bias") == sizeof r &&
                  prefs.getBytes("imu_bias", &r, sizeof r) == sizeof r && r.valid();
  prefs.end();
  if (ok) bias = r.value();
  return ok;
}
// Flash write: call from a low-priority task, never from the 500 us service tick.
inline bool saveImuBias(const std::array<float, 3>& bias) noexcept {
  Preferences prefs;
  if (!prefs.begin("hcr", false)) return false;
  const auto r = ImuBiasRecord::make(bias);
  const bool ok = prefs.putBytes("imu_bias", &r, sizeof r) == sizeof r;
  prefs.end();
  return ok;
}
}  // namespace gateway
#endif
