#pragma once

#include <array>
#include <cstdint>

#include <Arduino.h>
#include <HardwareSerial.h>

#include "tap_fw/BusTransport.hpp"
#include "tap_fw/DeferredFormat.hpp"

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
  Esp32UartPort(int uart_num, int gpio, int unused_rx_gpio,
                std::uint8_t motor_id, const char* name) noexcept
      : uart_num_(uart_num), gpio_(gpio), unused_rx_gpio_(unused_rx_gpio),
        motor_id_(motor_id), name_(name) {}

  bool begin(std::uint32_t baud) noexcept;
  [[nodiscard]] tap_fw::UartPort port() noexcept;
  // Opt-in bounded speed-write bursts; encoder deadline still has priority.
  void setSpeedPriority(bool enabled) noexcept;
  [[nodiscard]] bool speedPriority() noexcept;

  struct Diagnostics {
    std::uint32_t rx = 0U, read_timeouts = 0U, write_timeouts = 0U, motor_errors = 0U;
    std::uint32_t bad_frames = 0U, mode_writes = 0U, probes = 0U, dropped = 0U;
    std::uint32_t gap_max_us = 0U;             // longest gap between encoder samples since the last report
    std::uint32_t rtt_max_us = 0U, late = 0U;  // reply latency: max since last report, count > 3 ms
    std::uint32_t tx_hold_max_us = 0U, tx_late = 0U;
    std::uint32_t speed_tx = 0U, coalesced = 0U, unchanged = 0U, cancelled = 0U;
    std::uint32_t speed_queue_max_us = 0U, pending_max_us = 0U, speed_tx_age_us = 0U;
    std::int16_t sent_speed_raw = 0;
    bool speed_priority = false;
    std::uint32_t reply_age_ms = 0U;  // since the last valid frame (saturates)
    std::int32_t drv_mode = -1;
    std::int32_t status_return = -1;   // reg 0x10: 2 = driver acks every instruction, 1 = reads only
    bool synced = false, silent = false;
  };
  [[nodiscard]] Diagnostics diagnostics(bool reset_maxima = false) noexcept;
  // One "WBUS ..." status line for the serial log.
  void format(tap_fw::DeferredLine& line) noexcept;

 private:
  static bool send(void* context, const tap_fw::motor_frame::DxlFrame& frame) noexcept;
  static bool receive(void* context, tap_fw::motor_frame::DxlFrame& frame) noexcept;
  static void taskEntry(void* context) noexcept;

  struct Response { tap_fw::motor_frame::DxlFrame frame{}; std::uint64_t stamp_us = 0U; };
  struct Pending {
    tap_fw::motor_frame::DxlFrame frame{};
    std::uint8_t attempts = 0U;
    bool speed = false;
    std::int16_t speed_raw = 0;
    std::uint32_t epoch = 0U;
    std::int64_t queued_us = 0, latest_us = 0;
  };

  void taskMain() noexcept;
  bool installUart(std::uint32_t baud) noexcept;
  void run() noexcept;
  void bootSync(std::int64_t now) noexcept;
  bool transact(const tap_fw::motor_frame::DxlFrame& request, tap_fw::motor_frame::DxlParsed* reply,
                std::int64_t window_us, const Pending* pending = nullptr) noexcept;
  bool transmit(const tap_fw::motor_frame::DxlFrame& request) noexcept;
  bool readReply(tap_fw::motor_frame::DxlFrame& out, bool block) noexcept;
  void drive() noexcept;
  void release() noexcept;
  [[nodiscard]] bool popPending(Pending& out) noexcept;
  [[nodiscard]] bool speedPending() noexcept;
  [[nodiscard]] bool stopPending() noexcept;

  int uart_num_;
  int gpio_;
  int unused_rx_gpio_;
  std::uint8_t motor_id_;
  const char* name_;
  std::uint32_t baud_ = 1'000'000U;
  QueueHandle_t response_queue_ = nullptr;
  TaskHandle_t task_ = nullptr;
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;       // command queue + diagnostics
  portMUX_TYPE wire_mux_ = portMUX_INITIALIZER_UNLOCKED;  // TX turnaround, <= 500 us
  volatile bool init_done_ = false, init_ok_ = false;

  // Producer side (control task), guarded by mux_.
  static constexpr std::size_t kQueue = 8U;
  std::array<Pending, kQueue> queue_{};
  std::size_t q_head_ = 0U, q_count_ = 0U;
  std::uint32_t stop_epoch_ = 0U;
  bool poll_pending_ = false;
  bool speed_priority_ = false;
  // Last Moving Speed that reached the driver; an unchanged speed is not re-sent until kSpeedKeepaliveUs.
  bool last_speed_valid_ = false;
  std::int16_t last_speed_raw_ = 0;
  std::int64_t last_speed_tx_us_ = 0;
  std::int64_t last_speed_wire_us_ = 0;
  Diagnostics diag_{};

  // Worker-owned state.
  std::array<std::uint8_t, 16> rx_{};
  std::size_t rx_size_ = 0U;
  std::array<std::uint8_t, 16> echo_{};
  std::size_t echo_len_ = 0U, echo_pos_ = 0U;
  std::int64_t echo_deadline_us_ = 0, last_reply_us_ = 0, boot_us_ = 0, next_probe_us_ = 0;
  std::int64_t last_read_ok_us_ = 0, boot_next_us_ = 0, mode_write_us_ = 0;
  std::int16_t drv_mode_ = -1;
  std::uint8_t boot_stage_ = 0U, boot_tries_ = 0U;
  bool synced_ = false, prefer_read_ = false;
  std::uint8_t writes_since_read_ = 0U;
  // Status Return Level (reg 0x10) read at boot. At 1 the driver answers reads only, so a write is
  // fire-and-forget: the bus is free ~0.3 ms after it instead of waiting for an acknowledgement.
  std::int16_t status_return_ = -1;
  bool ack_writes_ = true;
};

}  // namespace tap_hw
