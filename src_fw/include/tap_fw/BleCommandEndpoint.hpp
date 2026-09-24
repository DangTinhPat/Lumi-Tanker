#pragma once

#include <cstddef>
#include <cstdint>

#include "robot_control/Types.hpp"

namespace tap_fw::ble {

// Nordic-UART-style UUIDs keep the service easy to find from generic BLE
// tools such as DaBLE. The phone writes ASCII commands to the RX characteristic;
// no notification characteristic is required for the one-way POC.
inline constexpr char kDeviceName[] = "TAP-RB";
inline constexpr char kServiceUuid[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
inline constexpr char kCommandCharacteristicUuid[] =
    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";

enum class WriteResult {
  Accepted,
  Heartbeat,
  ResetRequested,
  Rejected,
};

class CommandEndpoint {
 public:
  struct Config {
    // A phone must refresh the one-way command lease before this expires.
    // This is the only BLE-side safety fallback; IMU/encoder control remains
    // inside the controller.
    std::uint64_t heartbeat_timeout_us = 500'000U;
    float max_forward_velocity_m_s = 0.25F;
    float max_yaw_rate_rad_s = 0.80F;
  };

  CommandEndpoint() noexcept;
  explicit CommandEndpoint(Config config) noexcept;

  [[nodiscard]] WriteResult onWrite(const std::uint8_t* data,
                                    std::size_t length,
                                    std::uint64_t now_us) noexcept;

  [[nodiscard]] robot_control::ControlCommand command(
      std::uint64_t now_us) const noexcept;

  void onDisconnect() noexcept;

  [[nodiscard]] bool timedOut(std::uint64_t now_us) const noexcept;
  [[nodiscard]] const Config& config() const noexcept { return config_; }

 private:
  static constexpr std::size_t kMaxCommandBytes = 96U;

  void resetCommand() noexcept;
  void safeStop() noexcept;
  void refresh(std::uint64_t now_us) noexcept;
  [[nodiscard]] bool motionAllowed() const noexcept;

  Config config_{};
  robot_control::ControlCommand command_{};
  std::uint64_t last_rx_us_ = 0U;
  bool has_rx_ = false;
  bool emergency_latched_ = false;
};

}  // namespace tap_fw::ble
