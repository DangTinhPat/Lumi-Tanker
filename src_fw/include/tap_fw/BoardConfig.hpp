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
inline constexpr std::uint32_t kCanBitrate = 500'000U;
inline constexpr std::uint32_t kMotorUartBaud = 1'000'000U;
inline constexpr std::uint32_t kControlPeriodUs = 2'000U;
inline constexpr std::uint32_t kFeedbackMaxAgeUs = 20'000U;
inline constexpr std::uint32_t kCanProbePeriodUs = 10'000U;
inline constexpr std::uint32_t kWheelFeedbackPeriodUs = 2'000U;
inline constexpr std::uint32_t kBlePollPeriodUs = 10'000U;
inline constexpr bool kWheelDirectionsVerified =
    robot_control::motor_mapping::kWheelDirectionsVerified;
inline constexpr bool kWheelSpeedScaleVerified =
    robot_control::motor_mapping::kWheelSpeedScaleVerified;
inline constexpr bool kImuFrameVerified = false;
inline constexpr bool kLegEncoderEstimateScaleVerified =
    robot_control::motor_mapping::kGds68EncoderEstimateScaleVerified;
// Bench option: open TWAI listen-only so leg encoder estimates can be read
// while the CAN TX path is being debugged. Nothing is transmitted or
// acknowledged on CAN, and enable stays refused while this is true.
inline constexpr bool kCanListenOnly = false;
// Bench option: TWAI in normal mode, but every firmware CAN transmission is
// suppressed. Instead one single-shot frame with an unassigned ID (node 62,
// cmd 0, no GDS68 addresses it) is sent per second, so a TX failure can be
// separated from the firmware's traffic pattern. Enable stays refused.
inline constexpr bool kCanTxSingleFrameTest = false;
inline constexpr std::uint16_t kCanTxTestId = 0x7C0U;
inline constexpr std::uint32_t kCanTxTestPeriodUs = 1'000'000U;
// After a bus-off, wait this long before starting TWAI recovery so a broken
// TX path does not keep disturbing the bus.
inline constexpr std::uint32_t kCanBusOffRecoveryDelayUs = 1'000'000U;

struct MotorIds {
  std::array<robot_control::motor_mapping::LegCanBinding, 4> leg_can =
      robot_control::motor_mapping::kVerifiedLegCanBindings;
  std::array<robot_control::motor_mapping::WheelUartBinding, 2> wheel_uart =
      robot_control::motor_mapping::kWheelUartBindings;
};

}  // namespace tap_fw::board
