#pragma once

#include <array>
#include <cstdint>

#include <Arduino.h>
#include <HardwareSerial.h>

#include "tap_fw/BusTransport.hpp"

namespace tap_hw {

class Esp32TwaiPort {
 public:
  bool begin() noexcept;
  [[nodiscard]] tap_fw::CanPort port() noexcept;
  // Restarts the controller after a bus-off once the recovery delay elapses.
  void service(std::uint64_t now_us) noexcept;
  // Bring-up diagnostics: bus state, error counters, the standard IDs seen
  // since the previous call and the latest heartbeat/encoder estimate per
  // node. Read-only; never transmits.
  void printDiagnostics(Print& out) noexcept;

 private:
  static bool send(void* context,
                   const tap_fw::motor_transport::CanFrame& frame) noexcept;
  static bool receive(void* context,
                      tap_fw::motor_transport::CanFrame& frame) noexcept;

  static constexpr std::size_t kNodeCount = 64U;

  bool started_ = false;
  std::array<std::uint16_t, 0x800> rx_count_{};
  std::array<std::array<std::uint8_t, 8>, kNodeCount> heartbeat_{};
  std::array<std::array<std::uint8_t, 8>, kNodeCount> estimate_{};
  std::array<bool, kNodeCount> heartbeat_seen_{};
  std::array<bool, kNodeCount> estimate_seen_{};
  std::uint64_t bus_off_since_us_ = 0U;
  std::uint32_t bus_off_events_ = 0U;
  std::uint32_t recoveries_ = 0U;
  std::uint64_t next_test_tx_us_ = 0U;
  std::uint32_t test_attempts_ = 0U;
  std::uint32_t test_queue_fail_ = 0U;
  std::uint32_t alert_tx_success_ = 0U;
  std::uint32_t alert_tx_failed_ = 0U;
  std::uint32_t alert_bus_error_ = 0U;
  std::uint32_t alert_arb_lost_ = 0U;
  std::uint32_t alert_err_passive_ = 0U;
  std::uint32_t alert_bus_off_ = 0U;
  std::uint32_t tx_ok_ = 0U;
  std::uint32_t tx_fail_ = 0U;
  std::uint32_t rx_total_ = 0U;
};

class Esp32UartPort {
 public:
  Esp32UartPort(HardwareSerial& serial, int tx_gpio, int rx_gpio) noexcept
      : serial_(serial), tx_gpio_(tx_gpio), rx_gpio_(rx_gpio) {}

  bool begin(std::uint32_t baud) noexcept;
  [[nodiscard]] tap_fw::UartPort port() noexcept;

 private:
  static bool send(void* context,
                   const tap_fw::motor_frame::DxlFrame& frame) noexcept;
  static bool receive(void* context,
                      tap_fw::motor_frame::DxlFrame& frame) noexcept;
  bool readFrame(tap_fw::motor_frame::DxlFrame& frame) noexcept;
  void resetParser(std::uint8_t last_byte) noexcept;

  HardwareSerial& serial_;
  int tx_gpio_;
  int rx_gpio_;
  std::array<std::uint8_t, 16> receive_buffer_{};
  std::uint8_t receive_size_ = 0U;
  std::uint8_t expected_size_ = 0U;
  bool started_ = false;
};

}  // namespace tap_hw
