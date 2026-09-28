#pragma once

#include <array>
#include <cstdint>

#include <Arduino.h>

#include "robot_control/ImuStateEstimator.hpp"
#include "robot_control/Types.hpp"
#include "tap_fw/BoardConfig.hpp"
#include "tap_fw/BusTransport.hpp"
#include "tap_fw/MotorFeedback.hpp"
#include "tap_fw/v3/LegLock.hpp"
#include "tap_fw/v3/V3Controller.hpp"
#include "tap_hw/Esp32Bus.hpp"
#include "tap_hw/Mpu6500Spi.hpp"

namespace tap_hw {

// Firmware v3: two-wheel inverted pendulum. The legs are arranged before the
// motor supply is switched on and locked at that power-up pose (encoder 0 of
// each GDS68, driver position mode); only the wheels balance. USB serial only
// (tools/tap_serial_control_gui.py), no BLE.
class AppV3 {
 public:
  AppV3() noexcept;
  bool begin() noexcept;
  void loop() noexcept;

 private:
  void pollSerial(std::uint64_t now_us) noexcept;
  void handleLine(const char* line, std::uint64_t now_us) noexcept;
  void drainFeedback(std::uint64_t now_us) noexcept;
  void updateImu(std::uint64_t now_us) noexcept;
  void controlTick(std::uint64_t now_us) noexcept;
  void driveWheels(bool enabled, const std::array<float, 2>& cmd_rad_s,
                   std::uint64_t now_us) noexcept;
  void report(std::uint64_t now_us) noexcept;
  void printTunables() noexcept;

  [[nodiscard]] bool imuFresh(std::uint64_t now_us) const noexcept;
  [[nodiscard]] bool legsFresh(std::uint64_t now_us) const noexcept;
  [[nodiscard]] float legJointRad(std::size_t leg) const noexcept;
  // v3.1: joint angle (driver frame, rad) the lock is holding this leg at.
  [[nodiscard]] float legTargetRad(std::size_t leg) const noexcept;
  [[nodiscard]] float legChainRad() const noexcept;
  [[nodiscard]] const char* legAxisProblem(std::uint64_t now_us, bool need_closed_loop) const noexcept;

  Mpu6500Spi mpu_{};
  robot_control::ImuStateEstimator estimator_{};
  Esp32TwaiPort can_{};
  Esp32UartPort uart1_{Serial1, tap_fw::board::kUart1TxGpio, tap_fw::board::kUart1RxGpio};
  Esp32UartPort uart2_{Serial2, tap_fw::board::kUart2TxGpio, tap_fw::board::kUart2RxGpio};
  tap_fw::CallbackBusTransport transport_;
  tap_fw::MotorFeedback feedback_{};
  tap_fw::v3::LegLock legs_{};
  tap_fw::v3::V3Controller ctrl_;

  Mpu6500Spi::Sample chip_{};
  Mpu6500Spi::Sample body_{};
  robot_control::ImuStateEstimator::Output imu_{};
  robot_control::RobotState state_{};

  std::uint64_t next_control_us_ = 0U;
  std::uint64_t last_imu_us_ = 0U;
  std::uint64_t last_report_us_ = 0U;
  std::uint64_t next_trace_us_ = 0U;
  std::uint64_t last_rx_us_ = 0U;
  std::uint64_t state_since_us_ = 0U;
  std::uint64_t engage_us_ = 0U;
  std::uint64_t wheels_on_since_us_ = 0U;
  bool wheels_on_ = false;
  bool start_pending_ = false;
  bool disable_pending_ = false;
  bool go_pending_ = false;
  bool go_after_ready_ = false;
  float height_request_m_ = 0.0F;  // SET height; 0 = no request
  // HMOVE hold-to-run: +1 raise / -1 lower while the GUI keeps repeating it;
  // released (0) or silent for kHeightHoldUs -> freeze at the current height.
  int hmove_dir_ = 0;
  std::uint64_t hmove_last_us_ = 0U;
  void freezeHeight() noexcept;
  void setHeightMove(int dir, std::uint64_t now_us) noexcept;

  // BLE (Dabble GamePad): joystick drives, Start = START, Select = GO,
  // Triangle / Cross held = raise / lower the height. USB serial stays on.
  void pollBle(std::uint64_t now_us) noexcept;
  std::uint64_t next_ble_poll_us_ = 0U;
  std::uint64_t last_phone_rx_us_ = 0U;
  std::uint64_t ble_advertise_at_us_ = 0U;
  bool ble_link_up_ = false;
  bool ble_inputs_armed_ = false;
  bool ble_start_prev_ = false;
  bool ble_select_prev_ = false;
  int ble_height_dir_ = 0;
  bool ble_driving_ = false;  // GO from Idle/Fault: start, then engage at Ready
  bool estop_ = false;
  // TILTTEST: wheels follow pitch only (no velocity loop), for checking the
  // tilt->wheel direction with the robot lifted.
  bool tilt_test_ = false;
  float tilt_ref_rad_ = 0.0F;
  // 0.5 s average of pitch and of its spread, for measuring the balance angle
  // (operator holds the robot at its tipping point and sends BALANCE).
  float pitch_avg_ = 0.0F;
  float pitch_spread_ = 0.0F;
  bool can_ready_ = false;
  bool uart_ready_ = false;
  bool imu_ready_ = false;
  float drive_forward_ = 0.0F;
  float drive_yaw_ = 0.0F;
  int last_state_ = -1;
  char line_[96]{};
  std::size_t line_length_ = 0U;
  std::array<float, 2> last_wheel_cmd_{};
};

}  // namespace tap_hw
