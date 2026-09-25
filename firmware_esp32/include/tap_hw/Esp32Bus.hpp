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

 private:
  static bool send(void* context,
                   const tap_fw::motor_transport::CanFrame& frame) noexcept;
  static bool receive(void* context,
                      tap_fw::motor_transport::CanFrame& frame) noexcept;

  bool started_ = false;
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
