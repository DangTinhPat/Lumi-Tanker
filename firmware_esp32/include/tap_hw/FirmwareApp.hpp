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
#include "tap_fw/ImpulseStartupSequencer.hpp"
#include "tap_fw/LegHoming.hpp"
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
  void pollSerialConsole() noexcept;
  void drainMotorFeedback(std::uint64_t now_us) noexcept;
  void updateImu(std::uint64_t now_us) noexcept;
  void controlTick(std::uint64_t now_us) noexcept;
  [[nodiscard]] bool imuFresh(std::uint64_t now_us) const noexcept;
  [[nodiscard]] bool sensorsReady(std::uint64_t now_us) const noexcept;
  [[nodiscard]] bool readyForEnable(std::uint64_t now_us) const noexcept;
  // First Start on the fixture: capture the leg zero. Returns true when homed.
  enum class HomeGate { Ready, Waiting, Failed };
  HomeGate homeLegsIfNeeded(std::uint64_t now_us) noexcept;
  bool homeLegsNow(std::uint64_t now_us, const char** reason) noexcept;
  // side: 'A' = each leg its own pose, 'R'/'L' = that leg copied to both.
  void teachNatural(std::uint64_t now_us, char side) noexcept;
  void applyStartupLegPose(robot_control::ControlCommand& command) const noexcept;
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
  tap_fw::ImpulseStartupSequencer startup_sequencer_{};
  robot_control::RobotState state_{};
  Mpu6500Spi::Sample imu_chip_sample_{};
  Mpu6500Spi::Sample imu_sample_{};  // body frame
  robot_control::ImuStateEstimator::Output imu_output_{};
  std::uint64_t next_control_us_ = 0U;
  std::uint64_t next_ble_poll_us_ = 0U;
  std::uint64_t ble_advertise_at_us_ = 0U;
  std::uint64_t last_imu_us_ = 0U;
  std::uint64_t last_report_us_ = 0U;
  std::uint64_t last_can_diag_us_ = 0U;
  std::uint64_t imu_sequence_ = 0U;
  // Phone packet cadence (BLE lease) and control-cycle timing (Rule 4).
  std::uint64_t last_phone_rx_us_ = 0U;
  std::uint64_t phone_gap_max_us_ = 0U;
  std::uint32_t phone_packets_ = 0U;
  std::uint32_t cycle_max_us_ = 0U;
  std::uint64_t cycle_sum_us_ = 0U;
  std::uint32_t cycle_count_ = 0U;
  int last_mode_ = -1;
  int last_startup_stage_ = -1;
  bool can_ready_ = false;
  bool uart1_ready_ = false;
  bool uart2_ready_ = false;
  bool imu_ready_ = false;
  bool ble_link_up_ = false;
  bool ble_inputs_armed_ = false;
  bool reported_enable_block_ = false;
  bool reported_home_block_ = false;
  bool motion_lease_expired_ = false;
  bool gyro_quiet_ = false;
  // Wheel speed-scale check by hand: integral of the reported speed versus
  // the position-encoder travel since boot, per wheel (joints 2 and 5).
  float wheel_speed_integral_rad_[2]{};
  float wheel_start_rad_[2]{};
  bool wheel_start_valid_ = false;
  bool legs_enabled_ = false;
  std::uint64_t wheel_fast_since_us_ = 0U;
  std::uint64_t trace_until_us_ = 0U;
  std::uint64_t next_trace_us_ = 0U;
  std::uint64_t legs_enabled_since_us_ = 0U;
  std::uint64_t trace_start_us_ = 0U;
  tap_fw::HangPose hang_pose_{};
  robot_control::JointArray startup_extended_pose_{};
  bool startup_extended_pose_valid_ = false;
  bool fixture_arrived_reported_ = false;
  std::uint64_t home_wait_since_us_ = 0U;
  std::uint64_t hang_still_since_us_ = 0U;
  std::uint64_t transport_error_since_us_ = 0U;
  std::uint64_t next_leg_trace_us_ = 0U;
  // Operator-taught natural stance (console TEACH NATURAL, kept in NVS).
  robot_control::JointArray taught_natural_{};
  bool taught_natural_valid_ = false;
  std::uint32_t transport_error_events_ = 0U;
  char console_line_[64]{};
  std::size_t console_length_ = 0U;
};

}  // namespace tap_hw
