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
      static_cast<gpio_num_t>(tap_fw::board::kCanRxGpio), TWAI_MODE_NORMAL);
  general.tx_queue_len = 32U;
  general.rx_queue_len = 64U;
  const twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
  const twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&general, &timing, &filter) != ESP_OK) return false;
  if (twai_start() != ESP_OK) {
    (void)twai_driver_uninstall();
    return false;
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
  if (self == nullptr || !self->started_ || input.id > 0x07FFU ||
      input.dlc > input.data.size()) return false;

  twai_message_t message{};
  message.identifier = input.id;
  message.extd = 0U;
  message.rtr = 0U;
  message.data_length_code = input.dlc;
  std::memcpy(message.data, input.data.data(), input.dlc);
  return twai_transmit(&message, 0U) == ESP_OK;
}

bool Esp32TwaiPort::receive(void* context, CanFrame& output) noexcept {
  auto* self = static_cast<Esp32TwaiPort*>(context);
  if (self == nullptr || !self->started_) return false;

  twai_message_t message{};
  while (twai_receive(&message, 0U) == ESP_OK) {
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
