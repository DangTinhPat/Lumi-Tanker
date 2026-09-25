#pragma once

#include <cstdint>

#include <Arduino.h>
#include <HardwareSerial.h>

#include "robot_control/ImuStateEstimator.hpp"
#include "robot_control/Types.hpp"
#include "tap_fw/BoardConfig.hpp"
#include "tap_fw/ControllerActuatorBridge.hpp"
#include "tap_fw/DabbleGamepadMapper.hpp"
#include "tap_fw/FirmwareControlLoop.hpp"
#include "tap_fw/MotorFeedback.hpp"
#include "tap_hw/Esp32Bus.hpp"
#include "tap_hw/Mpu6500Spi.hpp"

namespace tap_hw {

class FirmwareApp {
 public:
  FirmwareApp() noexcept;
  bool begin() noexcept;
  void loop() noexcept;

 private:
  void pollBle(std::uint64_t now_us) noexcept;
  void drainMotorFeedback(std::uint64_t now_us) noexcept;
  void updateImu(std::uint64_t now_us) noexcept;
  void controlTick(std::uint64_t now_us) noexcept;
  [[nodiscard]] bool imuFresh(std::uint64_t now_us) const noexcept;
  [[nodiscard]] bool sensorsReady(std::uint64_t now_us) const noexcept;
  [[nodiscard]] bool readyForEnable(std::uint64_t now_us) const noexcept;
  void report(std::uint64_t now_us) noexcept;

  Mpu6500Spi imu_{};
  robot_control::ImuStateEstimator estimator_{};
  tap_fw::ble::CommandEndpoint endpoint_{};
  tap_fw::ble::DabbleGamepadMapper gamepad_;
  Esp32TwaiPort can_{};
  Esp32UartPort uart1_{Serial1, tap_fw::board::kUart1TxGpio,
                       tap_fw::board::kUart1RxGpio};
  Esp32UartPort uart2_{Serial2, tap_fw::board::kUart2TxGpio,
                       tap_fw::board::kUart2RxGpio};
  tap_fw::CallbackBusTransport transport_;
  tap_fw::MotorFeedback feedback_{};
  tap_fw::FirmwareControlLoop control_loop_{transport_};
  robot_control::RobotState state_{};
  Mpu6500Spi::Sample imu_sample_{};
  robot_control::ImuStateEstimator::Output imu_output_{};
  std::uint64_t next_control_us_ = 0U;
  std::uint64_t next_ble_poll_us_ = 0U;
  std::uint64_t ble_advertise_at_us_ = 0U;
  std::uint64_t last_imu_us_ = 0U;
  std::uint64_t last_report_us_ = 0U;
  std::uint64_t imu_sequence_ = 0U;
  int last_mode_ = -1;
  bool can_ready_ = false;
  bool uart1_ready_ = false;
  bool uart2_ready_ = false;
  bool imu_ready_ = false;
  bool ble_link_up_ = false;
  bool ble_inputs_armed_ = false;
  bool reported_enable_block_ = false;
};

}  // namespace tap_hw
