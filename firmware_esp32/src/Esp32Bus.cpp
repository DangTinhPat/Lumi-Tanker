#include "tap_hw/Esp32Bus.hpp"

#include <algorithm>
#include <cstring>

#include <driver/twai.h>
#include <soc/gpio_struct.h>
#include <soc/gpio_sig_map.h>
#include <hal/uart_ll.h>
#include <hal/gpio_ll.h>
#include <esp_timer.h>
#include <esp_rom_gpio.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_err.h>

#include "robot_hal/MotorFrameCodec.hpp"
#include "tap_fw/BoardConfig.hpp"

namespace tap_hw {
namespace {

namespace frame = robot_control::motor_frame;
using CanFrame = tap_fw::motor_transport::CanFrame;

}  // namespace

bool Esp32TwaiPort::begin() noexcept {
  if (started_) return true;
  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
      static_cast<gpio_num_t>(tap_fw::board::kCanTxGpio),
      static_cast<gpio_num_t>(tap_fw::board::kCanRxGpio),
      tap_fw::board::kCanListenOnly ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
  general.tx_queue_len = 32U;
  general.rx_queue_len = 64U;
  const twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
  const twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&general, &timing, &filter) != ESP_OK) return false;
  if (twai_start() != ESP_OK) {
    (void)twai_driver_uninstall();
    return false;
  }
  if (tap_fw::board::kCanTxSingleFrameTest) {
    (void)twai_reconfigure_alerts(
        TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_ERROR |
            TWAI_ALERT_ARB_LOST | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF,
        nullptr);
  }
  started_ = true;
  return true;
}

tap_fw::CanPort Esp32TwaiPort::port() noexcept {
  return tap_fw::CanPort{this, &Esp32TwaiPort::send,
                         &Esp32TwaiPort::receive};
}

bool Esp32TwaiPort::send(void* context, const CanFrame& input) noexcept {
  auto* self = static_cast<Esp32TwaiPort*>(context);
  if (self == nullptr || !self->started_ || tap_fw::board::kCanListenOnly ||
      tap_fw::board::kCanTxSingleFrameTest ||
      input.id > 0x07FFU ||
      input.dlc > input.data.size()) return false;

  twai_message_t message{};
  message.identifier = input.id;
  message.extd = 0U;
  message.rtr = 0U;
  message.data_length_code = input.dlc;
  std::memcpy(message.data, input.data.data(), input.dlc);
  const bool ok = twai_transmit(&message, 0U) == ESP_OK;
  ok ? ++self->tx_ok_ : ++self->tx_fail_;
  return ok;
}

bool Esp32TwaiPort::receive(void* context, CanFrame& output) noexcept {
  auto* self = static_cast<Esp32TwaiPort*>(context);
  if (self == nullptr || !self->started_) return false;

  twai_message_t message{};
  while (twai_receive(&message, 0U) == ESP_OK) {
    ++self->rx_total_;
    if (message.extd == 0U && message.identifier <= 0x07FFU) {
      auto& count = self->rx_count_[message.identifier];
      if (count != 0xFFFFU) ++count;
      const std::size_t node = message.identifier >> 5U;
      const auto command = static_cast<std::uint8_t>(message.identifier & 0x1FU);
      if (message.data_length_code == 8U &&
          command == frame::kHeartbeatCmdId) {
        std::memcpy(self->heartbeat_[node].data(), message.data, 8U);
        self->heartbeat_seen_[node] = true;
      } else if (message.data_length_code == 8U &&
                 command == frame::kGetEncoderEstimatesCmdId) {
        std::memcpy(self->estimate_[node].data(), message.data, 8U);
        self->estimate_seen_[node] = true;
      }
    }
    if (message.extd != 0U || message.rtr != 0U ||
        message.identifier > 0x07FFU || message.data_length_code > 8U) {
      continue;
    }
    output = CanFrame{};
    output.id = static_cast<std::uint16_t>(message.identifier);
    output.dlc = message.data_length_code;
    std::memcpy(output.data.data(), message.data, output.dlc);
    return true;
  }
  return false;
}

void Esp32TwaiPort::service(std::uint64_t now_us) noexcept {
  if (!started_) return;
  twai_status_info_t status{};
  if (twai_get_status_info(&status) != ESP_OK) return;

  if (tap_fw::board::kCanTxSingleFrameTest) {
    std::uint32_t alerts = 0U;
    if (twai_read_alerts(&alerts, 0U) == ESP_OK) {
      if (alerts & TWAI_ALERT_TX_SUCCESS) ++alert_tx_success_;
      if (alerts & TWAI_ALERT_TX_FAILED) ++alert_tx_failed_;
      if (alerts & TWAI_ALERT_BUS_ERROR) ++alert_bus_error_;
      if (alerts & TWAI_ALERT_ARB_LOST) ++alert_arb_lost_;
      if (alerts & TWAI_ALERT_ERR_PASS) ++alert_err_passive_;
      if (alerts & TWAI_ALERT_BUS_OFF) ++alert_bus_off_;
    }
    if (status.state == TWAI_STATE_RUNNING && now_us >= next_test_tx_us_) {
      next_test_tx_us_ = now_us + tap_fw::board::kCanTxTestPeriodUs;
      twai_message_t message{};
      message.identifier = tap_fw::board::kCanTxTestId;
      message.ss = 1U;  // Single shot: one attempt, no automatic retransmit.
      message.data_length_code = 8U;
      ++test_attempts_;
      if (twai_transmit(&message, 0U) != ESP_OK) ++test_queue_fail_;
    }
  }

  if (status.state == TWAI_STATE_BUS_OFF) {
    if (bus_off_since_us_ == 0U) {
      bus_off_since_us_ = now_us == 0U ? 1U : now_us;
      ++bus_off_events_;
    } else if (now_us - bus_off_since_us_ >=
               tap_fw::board::kCanBusOffRecoveryDelayUs) {
      // Clears the TX queue and waits for 128 x 11 recessive bits.
      (void)twai_initiate_recovery();
    }
  } else if (status.state == TWAI_STATE_STOPPED && bus_off_since_us_ != 0U) {
    // Recovery finished; the driver stays stopped until restarted.
    if (twai_start() == ESP_OK) {
      bus_off_since_us_ = 0U;
      ++recoveries_;
    }
  }
}

void Esp32TwaiPort::printDiagnostics(Print& out) noexcept {
  if (!started_) {
    out.println("CANDIAG driver not started");
    return;
  }
  twai_status_info_t status{};
  if (twai_get_status_info(&status) != ESP_OK) {
    out.println("CANDIAG status read failed");
    return;
  }
  static const char* const kStates[] = {"STOPPED", "RUNNING", "BUS_OFF",
                                        "RECOVERING"};
  const unsigned state = static_cast<unsigned>(status.state);
  out.printf("CANDIAG mode=%s state=%s tec=%lu rec=%lu bus_err=%lu "
             "arb_lost=%lu rx_missed=%lu tx_q=%lu tx_ok=%lu tx_fail=%lu "
             "rx_total=%lu bus_off=%lu recovered=%lu\n",
             tap_fw::board::kCanListenOnly ? "LISTEN_ONLY" : "NORMAL",
             state < 4U ? kStates[state] : "?",
             static_cast<unsigned long>(status.tx_error_counter),
             static_cast<unsigned long>(status.rx_error_counter),
             static_cast<unsigned long>(status.bus_error_count),
             static_cast<unsigned long>(status.arb_lost_count),
             static_cast<unsigned long>(status.rx_missed_count),
             static_cast<unsigned long>(status.msgs_to_tx),
             static_cast<unsigned long>(tx_ok_),
             static_cast<unsigned long>(tx_fail_),
             static_cast<unsigned long>(rx_total_),
             static_cast<unsigned long>(bus_off_events_),
             static_cast<unsigned long>(recoveries_));
  if (tap_fw::board::kCanTxSingleFrameTest) {
    out.printf("CANTEST id=0x%03X attempts=%lu queue_fail=%lu tx_success=%lu "
               "tx_failed=%lu bus_error=%lu arb_lost=%lu err_passive=%lu "
               "bus_off=%lu\n",
               static_cast<unsigned>(tap_fw::board::kCanTxTestId),
               static_cast<unsigned long>(test_attempts_),
               static_cast<unsigned long>(test_queue_fail_),
               static_cast<unsigned long>(alert_tx_success_),
               static_cast<unsigned long>(alert_tx_failed_),
               static_cast<unsigned long>(alert_bus_error_),
               static_cast<unsigned long>(alert_arb_lost_),
               static_cast<unsigned long>(alert_err_passive_),
               static_cast<unsigned long>(alert_bus_off_));
  }
  out.print("CANDIAG rx_ids:");
  bool any = false;
  for (std::size_t id = 0U; id < rx_count_.size(); ++id) {
    if (rx_count_[id] == 0U) continue;
    any = true;
    out.printf(" 0x%03X(node=%u cmd=0x%02X)x%u", static_cast<unsigned>(id),
               static_cast<unsigned>(id >> 5U),
               static_cast<unsigned>(id & 0x1FU),
               static_cast<unsigned>(rx_count_[id]));
    rx_count_[id] = 0U;
  }
  out.println(any ? "" : " none");

  for (std::size_t node = 0U; node < kNodeCount; ++node) {
    if (!heartbeat_seen_[node] && !estimate_seen_[node]) continue;
    out.printf("CANDIAG node=%u", static_cast<unsigned>(node));
    if (heartbeat_seen_[node]) {
      // ODrive heartbeat: axis_error u32, axis_state u8, ...
      const auto& d = heartbeat_[node];
      out.printf(" axis_error=0x%08lX axis_state=%u",
                 static_cast<unsigned long>(
                     d[0] | (d[1] << 8) | (d[2] << 16) |
                     (static_cast<std::uint32_t>(d[3]) << 24)),
                 static_cast<unsigned>(d[4]));
    }
    if (estimate_seen_[node]) {
      frame::MitBytes bytes{};
      std::copy(estimate_[node].begin(), estimate_[node].end(), bytes.begin());
      const auto estimate = frame::unpackEncoderEstimates(bytes);
      out.printf(" est_pos=%+.4fturn est_vel=%+.4fturn/s",
                 static_cast<double>(estimate.pos_turns),
                 static_cast<double>(estimate.vel_turns_s));
    }
    out.println();
  }
}

namespace {
// A free wheel answers in ~1.3 ms, a held or forced one (carried by hand, pressed against the floor)
// answers later. The bus carries one motor and nothing else waits on it, so wait long enough for the
// late reply instead of discarding it and asking again: the controller's freshness limit is 20 ms.
constexpr std::int64_t kReadWindowUs = 8'000;
constexpr std::int64_t kWriteWindowUs = 8'000;
// When the last encoder sample is older than this, the next transaction is a read: a speed write
// (latest-wins, the motor keeps its last speed) can wait, the controller's freshness limit is 20 ms.
constexpr std::int64_t kReadUrgentUs = 6'000;
constexpr std::int64_t kSilentUs = 100'000;      // no valid frame for this long = silent driver
constexpr std::int64_t kProbeUs = 100'000;       // transaction period for a silent driver
constexpr std::int64_t kBootReadPeriodUs = 50'000;
constexpr std::int64_t kBootTimeoutUs = 2'000'000;
constexpr std::int64_t kModeSettleUs = 150'000;  // reg 0x11 is a flash write on the driver
constexpr std::uint8_t kMaxAttempts = 3U;
// The driver holds its last Moving Speed (TEST1's GUI refreshes it every 250 ms), so a speed that did not
// change is not re-written on every control tick: each write costs the driver a parse and an ack, and a wheel
// held or turned by hand answered ever later while it was flooded with Moving Speed 0.
constexpr std::int64_t kSpeedKeepaliveUs = 100'000;
constexpr std::int64_t kResponseMaxAgeUs = 20'000;
}  // namespace

bool Esp32UartPort::begin(std::uint32_t baud) noexcept {
  if (task_ != nullptr) return init_ok_;
  baud_ = baud;
  response_queue_ = xQueueCreate(8U, sizeof(Response));
  if (response_queue_ == nullptr) return false;
  // The UART is installed inside the worker so its interrupt lives on core 0,
  // away from the control task and the CAN driver on core 1.
  if (xTaskCreatePinnedToCore(&Esp32UartPort::taskEntry, name_, 4096U, this, 5, &task_, 0) != pdPASS) {
    task_ = nullptr;
    return false;
  }
  for (unsigned i = 0U; i < 200U && !init_done_; ++i) vTaskDelay(pdMS_TO_TICKS(5));
  return init_done_ && init_ok_;
}

tap_fw::UartPort Esp32UartPort::port() noexcept {
  return tap_fw::UartPort{this, &Esp32UartPort::send, &Esp32UartPort::receive};
}

void Esp32UartPort::setSpeedPriority(bool enabled) noexcept {
  taskENTER_CRITICAL(&mux_);
  speed_priority_ = enabled;
  taskEXIT_CRITICAL(&mux_);
}

bool Esp32UartPort::speedPriority() noexcept {
  taskENTER_CRITICAL(&mux_);
  const bool enabled = speed_priority_;
  taskEXIT_CRITICAL(&mux_);
  return enabled;
}

void Esp32UartPort::taskEntry(void* context) noexcept {
  auto* self = static_cast<Esp32UartPort*>(context);
  if (self == nullptr) { vTaskDelete(nullptr); return; }
  self->taskMain();
}

void Esp32UartPort::taskMain() noexcept {
  init_ok_ = installUart(baud_);
  init_done_ = true;
  if (!init_ok_) { vTaskDelete(nullptr); return; }
  boot_us_ = esp_timer_get_time();
  last_reply_us_ = boot_us_;
  boot_next_us_ = boot_us_ + 100'000;
  for (;;) run();
}

bool Esp32UartPort::installUart(std::uint32_t baud) noexcept {
  const auto port = static_cast<uart_port_t>(uart_num_);
  const auto data_pin = static_cast<gpio_num_t>(gpio_);
  const auto unused_rx_pin = static_cast<gpio_num_t>(unused_rx_gpio_);
  // Match the working EC4012 S3 bridge: the PCB's separate RX pad is tied to
  // DATA electrically, but it must not drive or pull that net. UART RX is
  // routed to the DATA pad itself, which is released after every request.
  if (gpio_reset_pin(unused_rx_pin) != ESP_OK ||
      gpio_set_direction(unused_rx_pin, GPIO_MODE_INPUT) != ESP_OK ||
      gpio_set_pull_mode(unused_rx_pin, GPIO_FLOATING) != ESP_OK ||
      gpio_set_level(data_pin, 1) != ESP_OK ||
      gpio_set_direction(data_pin, GPIO_MODE_INPUT) != ESP_OK ||
      gpio_set_pull_mode(data_pin, GPIO_PULLUP_ONLY) != ESP_OK) return false;
  uart_config_t config{};
  config.baud_rate = static_cast<int>(baud);
  config.data_bits = UART_DATA_8_BITS;
  config.parity = UART_PARITY_DISABLE;
  config.stop_bits = UART_STOP_BITS_1;
  config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  config.source_clk = UART_SCLK_APB;
  if (uart_driver_install(port, 1024U, 0U, 0U, nullptr, 0U) != ESP_OK ||
      uart_param_config(port, &config) != ESP_OK ||
      uart_set_pin(port, gpio_, gpio_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) return false;
  // uart_set_pin(TX=RX) leaves the pad input-only; make it input+output and
  // reconnect the UART TX signal by hand (drive()/release() switch it).
  if (gpio_set_direction(data_pin, GPIO_MODE_INPUT_OUTPUT) != ESP_OK ||
      gpio_set_level(data_pin, 1) != ESP_OK ||
      gpio_set_pull_mode(data_pin, GPIO_PULLUP_ONLY) != ESP_OK) return false;
  release();
  (void)uart_set_rx_full_threshold(port, 96U);
  (void)uart_set_rx_timeout(port, 2U);
  return true;
}

void Esp32UartPort::drive() noexcept {
  esp_rom_gpio_connect_out_signal(gpio_, uart_num_ == 1 ? U1TXD_OUT_IDX : U2TXD_OUT_IDX, false, false);
}

void Esp32UartPort::release() noexcept {
  esp_rom_gpio_connect_out_signal(gpio_, SIG_GPIO_OUT_IDX, false, false);
  gpio_ll_output_disable(&GPIO, static_cast<gpio_num_t>(gpio_));
}

bool Esp32UartPort::send(void* context, const frame::DxlFrame& input) noexcept {
  auto* self = static_cast<Esp32UartPort*>(context);
  if (self == nullptr || self->task_ == nullptr || !self->init_ok_ || input.len == 0U ||
      input.len > input.bytes.size()) return false;
  const auto read = frame::parseDxlRead(input);
  if (read.ok) {
    if (read.address != frame::kDxlPresentAddr || read.length != frame::kDxlPresentLen) return false;
    taskENTER_CRITICAL(&self->mux_);
    self->poll_pending_ = true;
    taskEXIT_CRITICAL(&self->mux_);
    xTaskNotifyGive(self->task_);
    return true;
  }
  const auto write = frame::parseDxlWrite(input);
  if (!write.ok || write.id != self->motor_id_ ||
      (write.address != frame::kDxlTorqueEnableAddr && write.address != frame::kDxlMovingSpeedAddr)) return false;
  const bool speed = write.address == frame::kDxlMovingSpeedAddr;
  if (write.data_len != (speed ? 2U : 1U)) return false;
  const auto speed_raw = static_cast<std::int16_t>(write.data[0] | (write.data[1] << 8));
  const std::int64_t now = esp_timer_get_time();
  bool queued = true;
  taskENTER_CRITICAL(&self->mux_);
  if (!speed && write.data[0] == 0U) {
    // Stop supersedes all commands not yet on the wire. Never replay an old
    // torque-enable or speed after a newer torque-disable request.
    self->diag_.cancelled += self->q_count_;
    self->q_head_ = self->q_count_ = 0U;
    ++self->stop_epoch_;
  }
  const std::size_t tail = (self->q_head_ + self->q_count_ + kQueue - 1U) % kQueue;
  const bool tail_speed = self->q_count_ != 0U && self->queue_[tail].speed;
  if (!speed) {
    self->last_speed_valid_ = false;        // after a torque change the next speed always goes out
  } else if (!tail_speed && self->last_speed_valid_ && speed_raw == self->last_speed_raw_ &&
             now - self->last_speed_tx_us_ < kSpeedKeepaliveUs) {
    ++self->diag_.unchanged;
    taskEXIT_CRITICAL(&self->mux_);
    return true;                            // unchanged and recently written: nothing to send
  }
  if (speed && tail_speed) {
    self->queue_[tail].frame = input;       // latest speed wins; order against torque writes is kept
    self->queue_[tail].speed_raw = speed_raw;
    self->queue_[tail].attempts = 0U;
    self->queue_[tail].latest_us = now;
    ++self->diag_.coalesced;
  } else if (self->q_count_ < kQueue) {
    self->queue_[(self->q_head_ + self->q_count_) % kQueue] =
        Pending{input, 0U, speed, speed_raw, self->stop_epoch_, now, now};
    ++self->q_count_;
  } else {
    queued = false;
    ++self->diag_.dropped;
  }
  taskEXIT_CRITICAL(&self->mux_);
  if (queued && self->task_ != nullptr) xTaskNotifyGive(self->task_);   // no wait for the idle tick
  return queued;
}

bool Esp32UartPort::receive(void* context, frame::DxlFrame& output) noexcept {
  auto* self = static_cast<Esp32UartPort*>(context);
  if (self == nullptr || self->response_queue_ == nullptr) return false;
  Response response{};
  Response newest{};
  bool have = false;
  // Hand back the freshest sample (older ones add nothing: the wheel odometry unwraps from deltas), and
  // never stamp an old reply with the current tick.
  for (unsigned n = 0U; n < 8U && xQueueReceive(self->response_queue_, &response, 0U) == pdTRUE; ++n) {
    if (esp_timer_get_time() - static_cast<std::int64_t>(response.stamp_us) > kResponseMaxAgeUs) continue;
    newest = response;
    have = true;
  }
  if (have) output = newest.frame;
  return have;
}

bool Esp32UartPort::popPending(Pending& out) noexcept {
  bool found = false;
  taskENTER_CRITICAL(&mux_);
  while (q_count_ != 0U) {
    Pending candidate = queue_[q_head_];
    q_head_ = (q_head_ + 1U) % kQueue;
    --q_count_;
    // Until the driver's mode is confirmed only Torque Disable may go out.
    const auto write = frame::parseDxlWrite(candidate.frame);
    const bool disable = write.ok && write.address == frame::kDxlTorqueEnableAddr && write.data[0] == 0U;
    if (!synced_ && !disable) { ++diag_.dropped; continue; }
    // A duplicate can have been queued while the previous write awaited ACK.
    // Recheck here so it does not cost another complete write/read exchange.
    if (candidate.speed && last_speed_valid_ && candidate.speed_raw == last_speed_raw_ &&
        esp_timer_get_time() - last_speed_tx_us_ < kSpeedKeepaliveUs) {
      ++diag_.unchanged;
      continue;
    }
    out = candidate;
    found = true;
    break;
  }
  taskEXIT_CRITICAL(&mux_);
  return found;
}

bool Esp32UartPort::speedPending() noexcept {
  taskENTER_CRITICAL(&mux_);
  const bool pending = speed_priority_ && q_count_ != 0U && queue_[q_head_].speed;
  taskEXIT_CRITICAL(&mux_);
  return pending;
}

bool Esp32UartPort::stopPending() noexcept {
  taskENTER_CRITICAL(&mux_);
  const bool pending = q_count_ != 0U && !queue_[q_head_].speed &&
      queue_[q_head_].frame.bytes[5] == frame::kDxlTorqueEnableAddr &&
      queue_[q_head_].frame.bytes[6] == 0U;
  taskEXIT_CRITICAL(&mux_);
  return pending;
}

Esp32UartPort::Diagnostics Esp32UartPort::diagnostics(bool reset_maxima) noexcept {
  taskENTER_CRITICAL(&mux_);
  Diagnostics snapshot = diag_;
  snapshot.speed_priority = speed_priority_;
  snapshot.speed_tx_age_us = last_speed_wire_us_ == 0 ? 999'999U :
      static_cast<std::uint32_t>(std::min<std::int64_t>(esp_timer_get_time() - last_speed_wire_us_, 999'999));
  if (reset_maxima) {
    diag_.rtt_max_us = diag_.gap_max_us = diag_.tx_hold_max_us = 0U;
    diag_.speed_queue_max_us = diag_.pending_max_us = 0U;
  }
  taskEXIT_CRITICAL(&mux_);
  const std::int64_t age = (esp_timer_get_time() - last_reply_us_) / 1000;
  snapshot.reply_age_ms = age < 0 ? 0U : static_cast<std::uint32_t>(std::min<std::int64_t>(age, 99'999));
  snapshot.drv_mode = drv_mode_;
  snapshot.status_return = status_return_;
  snapshot.synced = synced_;
  snapshot.silent = esp_timer_get_time() - last_reply_us_ > kSilentUs;
  return snapshot;
}

void Esp32UartPort::format(tap_fw::DeferredLine& b) noexcept {
  const auto d = diagnostics(true);
  b.str("WBUS ").str(name_)
      .str(" id=").u32(motor_id_)
      .str(" sync=").i32(d.synced ? 1 : 0)
      .str(" drvmode=").i32(d.drv_mode)
      .str(" srl=").i32(d.status_return)
      .str(" silent=").i32(d.silent ? 1 : 0)
      .str(" age_ms=").u32(d.reply_age_ms)
      .str(" rx=").u32(d.rx)
      .str(" rto=").u32(d.read_timeouts)
      .str(" wto=").u32(d.write_timeouts)
      .str(" err=").u32(d.motor_errors)
      .str(" bad=").u32(d.bad_frames)
      .str(" modew=").u32(d.mode_writes)
      .str(" probes=").u32(d.probes)
      .str(" drop=").u32(d.dropped)
      .str(" rtt_max_us=").u32(d.rtt_max_us)
      .str(" late=").u32(d.late)
      .str(" gap_max_us=").u32(d.gap_max_us)
      .str(" tx_hold_us=").u32(d.tx_hold_max_us)
      .str(" tx_late=").u32(d.tx_late)
      .str(" prio=").i32(d.speed_priority ? 1 : 0)
      .str(" speed_tx=").u32(d.speed_tx)
      .str(" queue_max_us=").u32(d.speed_queue_max_us)
      .str(" pending_max_us=").u32(d.pending_max_us)
      .str(" speed_age_us=").u32(d.speed_tx_age_us)
      .str(" sent_raw=").i32(d.sent_speed_raw)
      .str(" merged=").u32(d.coalesced)
      .str(" unchanged=").u32(d.unchanged)
      .str(" cancelled=").u32(d.cancelled);
}

bool Esp32UartPort::transmit(const frame::DxlFrame& request) noexcept {
  // TX-done task wakeup is too late for this motor: an 80 us READ held DATA
  // for 180-309 us in the instrumented robot log. This worker is the sole TX
  // owner and the IDF TX software buffer is disabled. Send one bounded frame
  // through the FIFO and release DATA at hardware idle, without scheduling
  // another task between the last stop bit and high-Z. RX still uses IDF.
  const auto port = static_cast<uart_port_t>(uart_num_);
  auto* hw = UART_LL_GET_HW(port);
  taskENTER_CRITICAL(&wire_mux_);
  const auto started = esp_timer_get_time();
  bool ok = uart_ll_is_tx_idle(hw) && uart_ll_get_txfifo_len(hw) >= request.len;
  if (ok) {
    drive();
    uart_ll_write_txfifo(hw, request.bytes.data(), request.len);
    while (!uart_ll_is_tx_idle(hw) && esp_timer_get_time() - started < 500) {}
    ok = uart_ll_is_tx_idle(hw);
  }
  release();
  const auto held = static_cast<std::uint32_t>(esp_timer_get_time() - started);
  if (!ok) uart_ll_txfifo_rst(hw);
  taskEXIT_CRITICAL(&wire_mux_);
  taskENTER_CRITICAL(&mux_);
  diag_.tx_hold_max_us = std::max(diag_.tx_hold_max_us, held);
  if (held > request.len * 10U + 50U) ++diag_.tx_late;
  taskEXIT_CRITICAL(&mux_);
  return ok;
}

bool Esp32UartPort::readReply(frame::DxlFrame& output, bool block) noexcept {
  // Drop the locally received TX echo, then parse checksummed motor packets.
  const auto port = static_cast<uart_port_t>(uart_num_);
  for (unsigned n = 0U; n < 64U; ++n) {
    std::uint8_t byte = 0U;
    // The first byte may block for up to one tick (it returns the moment the UART ISR delivers a byte), so
    // waiting for a reply yields the core instead of spinning: two spinning wheel workers at priority 4
    // starved the serial task for ~1 s and the session lease expired.
    if (uart_read_bytes(port, &byte, 1U, (block && n == 0U) ? 1U : 0U) != 1) return false;
    // The shared RX input hears our TX too. Hold a matching prefix until it
    // proves to be the complete echo; if it differs, replay the held bytes so
    // a genuine motor response with the same FF FF ID prefix is not lost.
    std::array<std::uint8_t, 17> incoming{};
    std::size_t count = 1U;
    incoming[0] = byte;
    if (echo_len_ != 0U && esp_timer_get_time() <= echo_deadline_us_ &&
        echo_pos_ < echo_len_ && byte == echo_[echo_pos_]) {
      if (++echo_pos_ == echo_len_) echo_len_ = echo_pos_ = 0U;
      continue;
    }
    if (echo_pos_ != 0U) {
      std::copy_n(echo_.begin(), echo_pos_, incoming.begin());
      incoming[echo_pos_] = byte;
      count = echo_pos_ + 1U;
    }
    echo_len_ = echo_pos_ = 0U;
    for (std::size_t i = 0U; i < count; ++i) {
      if (rx_size_ == rx_.size()) {
        std::memmove(rx_.data(), rx_.data() + 1U, rx_size_ - 1U);
        --rx_size_;
      }
      rx_[rx_size_++] = incoming[i];
      while (rx_size_ != 0U) {
        if (rx_[0] != 0xFFU || (rx_size_ >= 2U && rx_[1] != 0xFFU)) {
          std::memmove(rx_.data(), rx_.data() + 1U, rx_size_ - 1U);
          --rx_size_;
          continue;
        }
        if (rx_size_ < 4U) break;
        const std::size_t total = static_cast<std::size_t>(rx_[3]) + 4U;
        if (total < 8U || total > rx_.size()) {
          std::memmove(rx_.data(), rx_.data() + 1U, rx_size_ - 1U);
          --rx_size_;
          continue;
        }
        if (rx_size_ < total) break;
        frame::DxlFrame candidate{};
        candidate.len = static_cast<std::uint8_t>(total);
        std::copy_n(rx_.begin(), total, candidate.bytes.begin());
        const bool valid = frame::parseDxlResponse(candidate.bytes.data(), candidate.len).ok;
        if (valid) {
          std::memmove(rx_.data(), rx_.data() + total, rx_size_ - total);
          rx_size_ -= total;
          output = candidate;
          return true;
        }
        taskENTER_CRITICAL(&mux_);
        ++diag_.bad_frames;
        taskEXIT_CRITICAL(&mux_);
        std::memmove(rx_.data(), rx_.data() + 1U, rx_size_ - 1U);
        --rx_size_;
      }
    }
  }
  return false;
}

bool Esp32UartPort::transact(const frame::DxlFrame& request, frame::DxlParsed* reply,
                             std::int64_t window_us, const Pending* pending) noexcept {
  if (pending != nullptr) {
    taskENTER_CRITICAL(&mux_);
    const bool superseded = pending->epoch != stop_epoch_;
    taskEXIT_CRITICAL(&mux_);
    if (superseded) return false;
  }
  // Anything left from an earlier exchange stays in the parser: a late Present Data reply is still a
  // valid sample, and replies to other registers are rejected by the [INS, ADDR] match below.
  echo_len_ = request.len;
  echo_pos_ = 0U;
  std::copy_n(request.bytes.begin(), request.len, echo_.begin());
  echo_deadline_us_ = esp_timer_get_time() + request.len * 10U + 3'000U;
  const bool is_read = frame::parseDxlRead(request).ok;
  if (!transmit(request)) { echo_len_ = echo_pos_ = 0U; return false; }
  echo_deadline_us_ = esp_timer_get_time() + 3'000U;
  const std::int64_t released = esp_timer_get_time();
  if (pending != nullptr && pending->speed) {
    // Timestamp actual TX completion, not enqueue/ACK completion. This does
    // not prove the motor applied the speed; it isolates ESP32 queue latency.
    taskENTER_CRITICAL(&mux_);
    ++diag_.speed_tx;
    diag_.sent_speed_raw = pending->speed_raw;
    last_speed_wire_us_ = released;
    diag_.speed_queue_max_us = std::max(diag_.speed_queue_max_us,
        static_cast<std::uint32_t>(std::min<std::int64_t>(released - pending->latest_us, 999'999)));
    diag_.pending_max_us = std::max(diag_.pending_max_us,
        static_cast<std::uint32_t>(std::min<std::int64_t>(released - pending->queued_us, 999'999)));
    taskEXIT_CRITICAL(&mux_);
  }
  if (!is_read && !ack_writes_) {
    // Status Return Level 1: no acknowledgement is coming. Leave the driver a moment to take the
    // write before the next frame instead of waiting for a reply that never arrives.
    esp_rom_delay_us(120);
    return true;
  }
  const std::int64_t deadline = released + window_us;
  while (esp_timer_get_time() < deadline) {
    frame::DxlFrame response{};
    if (!readReply(response, true)) continue;
    const auto parsed = frame::parseDxlResponse(response.bytes.data(), response.len);
    // The driver echoes [LEN, INS, ADDR] of the request in bytes 5..7.
    if (!parsed.ok || parsed.id != motor_id_ || response.bytes[6] != request.bytes[4] ||
        response.bytes[7] != request.bytes[5]) continue;
    last_reply_us_ = esp_timer_get_time();
    if (parsed.err != 0U) {
      taskENTER_CRITICAL(&mux_);
      ++diag_.motor_errors;
      taskEXIT_CRITICAL(&mux_);
      return false;
    }
    taskENTER_CRITICAL(&mux_);
    ++diag_.rx;
    const auto rtt = static_cast<std::uint32_t>(std::min<std::int64_t>(last_reply_us_ - released, 99'999));
    if (rtt > diag_.rtt_max_us) diag_.rtt_max_us = rtt;
    if (rtt > 3000U) ++diag_.late;
    taskEXIT_CRITICAL(&mux_);
    if (reply != nullptr) *reply = parsed;
    if (is_read && request.bytes[5] == frame::kDxlPresentAddr) {
      // A driver browning out sends frames with a valid checksum and garbage inside (the log of the 12:30
      // blackout shows a wheel speed of -2930 rad/s). Position is 10 bit; speed cannot exceed the no-load RPM.
      const auto present = frame::parsePresentPayload(parsed);
      if (!present.valid || present.position_raw > 1023U || std::abs(static_cast<int>(present.speed_raw)) > 1100) {
        taskENTER_CRITICAL(&mux_);
        ++diag_.bad_frames;
        taskEXIT_CRITICAL(&mux_);
        return false;
      }
      taskENTER_CRITICAL(&mux_);
      if (last_read_ok_us_ != 0) {
        const auto gap = static_cast<std::uint32_t>(std::min<std::int64_t>(last_reply_us_ - last_read_ok_us_, 999'999));
        if (gap > diag_.gap_max_us) diag_.gap_max_us = gap;
      }
      taskEXIT_CRITICAL(&mux_);
      last_read_ok_us_ = last_reply_us_;
      const Response item{response, static_cast<std::uint64_t>(last_reply_us_)};
      if (xQueueSend(response_queue_, &item, 0U) != pdTRUE) {
        Response old{};
        (void)xQueueReceive(response_queue_, &old, 0U);   // keep the freshest sample
        (void)xQueueSend(response_queue_, &item, 0U);
      }
    }
    return true;
  }
  taskENTER_CRITICAL(&mux_);
  if (is_read) ++diag_.read_timeouts; else ++diag_.write_timeouts;
  taskEXIT_CRITICAL(&mux_);
  return false;
}

void Esp32UartPort::bootSync(std::int64_t now) noexcept {
  // Read the driver's Velocity Mode (0x11) first; write Speed mode only when it
  // differs (every write is a flash write on the motor), then read it back.
  // The driver is never driven while its mode is unknown: a Moving Speed sent
  // to a motor left in Position mode made one wheel stop answering.
  if (now < boot_next_us_) return;
  if (boot_stage_ == 1U) {                               // mode write sent: wait for the flash write
    if (now - mode_write_us_ < kModeSettleUs) return;
    boot_stage_ = 2U;
    drv_mode_ = -1;
  }
  if (drv_mode_ >= 0) {
    if (drv_mode_ == static_cast<std::int16_t>(frame::kDxlVelModeSpeed)) {
      if (status_return_ < 0) {                           // how does this driver acknowledge writes?
        const std::uint8_t sp[2] = {0x10U, 1U};
        frame::DxlParsed sr{};
        if (transact(frame::detail::buildFrame(motor_id_, frame::kDxlInsRead, sp, 2U), &sr, kReadWindowUs) &&
            sr.payload_len == 1U) {
          status_return_ = sr.payload[0];
          ack_writes_ = status_return_ >= 2;
        }
        boot_next_us_ = now + kBootReadPeriodUs;
        return;
      }
      synced_ = true;
      return;
    }
    if (boot_stage_ == 2U || boot_tries_ >= 2U) {        // written but not taken: start over later
      boot_us_ = now; boot_next_us_ = now + 1'000'000; boot_stage_ = 0U; boot_tries_ = 0U; drv_mode_ = -1;
      return;
    }
    ++boot_tries_;
    frame::DxlParsed ack{};
    (void)transact(frame::buildVelocityMode(motor_id_, frame::kDxlVelModeSpeed), &ack, kWriteWindowUs);
    taskENTER_CRITICAL(&mux_);
    ++diag_.mode_writes;
    taskEXIT_CRITICAL(&mux_);
    mode_write_us_ = now;
    boot_stage_ = 1U;
    boot_next_us_ = now + kModeSettleUs;
    drv_mode_ = -1;
    return;
  }
  if (now - boot_us_ > kBootTimeoutUs) {                 // no answer: stay un-synced, try again later
    boot_us_ = now;
    boot_next_us_ = now + 1'000'000;
    return;
  }
  const std::uint8_t params[2] = {frame::kDxlVelocityModeAddr, 1U};
  frame::DxlParsed reply{};
  if (transact(frame::detail::buildFrame(motor_id_, frame::kDxlInsRead, params, 2U), &reply, kReadWindowUs) &&
      reply.payload_len == 1U) {
    drv_mode_ = reply.payload[0];
  }
  boot_next_us_ = now + kBootReadPeriodUs;
}

void Esp32UartPort::run() noexcept {
  const std::int64_t now = esp_timer_get_time();
  const bool stop_pending = stopPending();
  if (!synced_) {
    if (!stop_pending) bootSync(now);
    Pending pending{};
    if (popPending(pending)) (void)transact(pending.frame, nullptr, kWriteWindowUs);  // torque-off only
    vTaskDelay(1U);
    return;
  }
  const bool silent = now - last_reply_us_ > kSilentUs;
  if (silent) {
    if (now < next_probe_us_ && !stop_pending) { vTaskDelay(1U); return; }
    next_probe_us_ = now + kProbeUs;
    taskENTER_CRITICAL(&mux_);
    ++diag_.probes;
    taskEXIT_CRITICAL(&mux_);
  }
  bool poll = false;
  taskENTER_CRITICAL(&mux_);
  poll = poll_pending_;
  taskEXIT_CRITICAL(&mux_);
  // Normal mode alternates write/read. Priority mode permits at most two
  // consecutive speed writes, and NEVER overrides the encoder's 6 ms urgency.
  // No transaction is preempted on the single-wire half-duplex bus.
  // A failed read must not monopolize the bus until feedback recovers.
  // After each completed read attempt, allow one queued command before
  // retrying. Stale feedback still stops the controller at its normal limit.
  const bool read_urgent = poll && !silent && writes_since_read_ != 0U &&
      now - last_read_ok_us_ > kReadUrgentUs;
  const bool speed_priority = writes_since_read_ < 2U && speedPending();
  if (stop_pending || ((!(prefer_read_ && poll) || speed_priority) && !read_urgent) || silent) {
    Pending pending{};
    if (popPending(pending)) {
      const bool ok = transact(pending.frame, nullptr, kWriteWindowUs, &pending);
      if (writes_since_read_ < 2U) ++writes_since_read_;
      if (pending.speed) {
        taskENTER_CRITICAL(&mux_);
        last_speed_valid_ = ok && pending.epoch == stop_epoch_;
        last_speed_raw_ = pending.speed_raw;
        last_speed_tx_us_ = esp_timer_get_time();
        taskEXIT_CRITICAL(&mux_);
      }
      if (!ok && !pending.speed && ++pending.attempts < kMaxAttempts) {
        taskENTER_CRITICAL(&mux_);                        // retry lifecycle writes, in order
        if (q_count_ < kQueue && pending.epoch == stop_epoch_) {
          q_head_ = (q_head_ + kQueue - 1U) % kQueue;
          queue_[q_head_] = pending;
          ++q_count_;
        }
        taskEXIT_CRITICAL(&mux_);
      }
      prefer_read_ = true;
      if (!silent) return;
    }
  }
  if (poll) {
    taskENTER_CRITICAL(&mux_);
    poll_pending_ = false;
    taskEXIT_CRITICAL(&mux_);
    (void)transact(frame::buildReadPresent(motor_id_), nullptr, kReadWindowUs);
    writes_since_read_ = 0U;
    prefer_read_ = false;
    return;
  }
  (void)ulTaskNotifyTake(pdTRUE, 1U);   // idle: sleep until a command or poll request arrives (max 1 tick)
}

}  // namespace tap_hw
