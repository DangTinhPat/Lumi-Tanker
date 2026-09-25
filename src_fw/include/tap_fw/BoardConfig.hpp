#pragma once

#include <array>
#include <cstdint>

#include "robot_hal/MotorMapping.hpp"

namespace tap_fw::board {

// POC hardware mapping from docs/data/POC - Robot version 1.
inline constexpr int kButtonGpio = 6;
inline constexpr int kLed1Gpio = 39;
inline constexpr int kLed2Gpio = 40;
inline constexpr int kLed3Gpio = 41;
inline constexpr int kSpareGpio = 42;
inline constexpr int kUsbDmGpio = 19;
inline constexpr int kUsbDpGpio = 20;
inline constexpr int kDebugUartTxGpio = 43;
inline constexpr int kDebugUartRxGpio = 44;
inline constexpr int kBootGpio = 0;
// EN is the ESP32-S3 reset input, not a normal GPIO number.
inline constexpr int kI2cSdaGpio = 8;
inline constexpr int kI2cSclGpio = 9;
inline constexpr int kCanTxGpio = 4;
inline constexpr int kCanRxGpio = 5;
inline constexpr int kUart1TxGpio = 17;
inline constexpr int kUart1RxGpio = 18;
inline constexpr int kUart2TxGpio = 15;
inline constexpr int kUart2RxGpio = 16;
inline constexpr int kImuCsGpio = 10;
inline constexpr int kImuMosiGpio = 11;
inline constexpr int kImuSclkGpio = 12;
inline constexpr int kImuMisoGpio = 13;
inline constexpr int kImuIntGpio = 14;
inline constexpr std::uint32_t kCanBitrate = 1'000'000U;
inline constexpr std::uint32_t kMotorUartBaud = 1'000'000U;

struct MotorIds {
  std::array<robot_control::motor_mapping::LegCanBinding, 4> leg_can =
      robot_control::motor_mapping::kVerifiedLegCanBindings;
  std::array<std::uint8_t, 2> wheel_uart = {
      robot_control::motor_mapping::kLeftWheelUartId,
      robot_control::motor_mapping::kRightWheelUartId};
};

}  // namespace tap_fw::board
