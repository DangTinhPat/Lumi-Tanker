#include "tap_hw/Esp32Bus.hpp"

#include <algorithm>
#include <cstring>

#include <driver/twai.h>
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

bool Esp32UartPort::begin(std::uint32_t baud) noexcept {
  if (started_) return true;
  serial_.begin(baud, SERIAL_8N1, rx_gpio_, tx_gpio_);
  started_ = true;
  return true;
}

tap_fw::UartPort Esp32UartPort::port() noexcept {
  return tap_fw::UartPort{this, &Esp32UartPort::send,
                         &Esp32UartPort::receive};
}

bool Esp32UartPort::send(void* context, const frame::DxlFrame& input) noexcept {
  auto* self = static_cast<Esp32UartPort*>(context);
  if (self == nullptr || !self->started_ || input.len == 0U ||
      input.len > input.bytes.size()) return false;
  return self->serial_.write(input.bytes.data(), input.len) == input.len;
}

bool Esp32UartPort::receive(void* context, frame::DxlFrame& output) noexcept {
  auto* self = static_cast<Esp32UartPort*>(context);
  return self != nullptr && self->started_ && self->readFrame(output);
}

bool Esp32UartPort::readFrame(frame::DxlFrame& output) noexcept {
  while (serial_.available() > 0) {
    const int value = serial_.read();
    if (value < 0) break;
    const auto byte = static_cast<std::uint8_t>(value);

    if (receive_size_ == 0U) {
      if (byte == 0xFFU) receive_buffer_[receive_size_++] = byte;
      continue;
    }
    if (receive_size_ == 1U && byte != 0xFFU) {
      resetParser(byte);
      continue;
    }
    if (receive_size_ >= receive_buffer_.size()) {
      resetParser(byte);
      continue;
    }

    receive_buffer_[receive_size_++] = byte;
    if (receive_size_ == 4U) {
      const std::uint16_t total =
          static_cast<std::uint16_t>(receive_buffer_[3]) + 4U;
      if (total < 8U || total > receive_buffer_.size()) {
        resetParser(byte);
        continue;
      }
      expected_size_ = static_cast<std::uint8_t>(total);
    }
    if (expected_size_ != 0U && receive_size_ == expected_size_) {
      frame::DxlFrame candidate{};
      candidate.len = receive_size_;
      std::copy_n(receive_buffer_.begin(), receive_size_, candidate.bytes.begin());
      receive_size_ = 0U;
      expected_size_ = 0U;
      if (frame::parseDxlResponse(candidate.bytes.data(), candidate.len).ok) {
        output = candidate;
        return true;
      }
    }
  }
  return false;
}

void Esp32UartPort::resetParser(std::uint8_t last_byte) noexcept {
  receive_size_ = last_byte == 0xFFU ? 1U : 0U;
  expected_size_ = 0U;
  if (receive_size_ != 0U) receive_buffer_[0] = 0xFFU;
}

}  // namespace tap_hw
