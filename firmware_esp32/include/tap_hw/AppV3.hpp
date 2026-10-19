#pragma once

#include <algorithm>
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
#include "tap_hw/OutSink.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

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
  // FreeRTOS split: a control task (core 1, above the Arduino loop) runs
  // every 2 ms - commands, BLE, buses, IMU, controller, telemetry snapshots -
  // and the Arduino loop only moves serial bytes (input lines to a queue,
  // deferred numeric formatting and output from OutSink to the UART).
  // All controller state lives in the
  // control task; the loop touches none of it. If the task cannot be made,
  // the old single loop runs everything (rtos_ false).
  static void controlTaskEntry(void* arg) noexcept;
  // Serial in/out on core 0: on core 1 under the control task it starved
  // whenever a tick ran past 2 ms (LQR + roll during a height ramp:
  // ~2.8 ms), the GUI heartbeat stopped arriving and the 0.5 s session
  // lease disabled the robot right after engage (hardware 2026-10-01).
  static void commsTaskEntry(void* arg) noexcept;
  TaskHandle_t comms_task_ = nullptr;
  void step(bool task) noexcept;
  void controlPart(std::uint64_t now_us) noexcept;
  struct LineItem { char text[96]; };
  OutSink out_{};
  TaskHandle_t control_task_ = nullptr;
  QueueHandle_t line_q_ = nullptr;
  bool rtos_ = false;
  std::uint32_t lines_dropped_ = 0U;
  void pollSerial(std::uint64_t now_us) noexcept;
  void handleLine(const char* line, std::uint64_t now_us) noexcept;
  void drainFeedback(std::uint64_t now_us) noexcept;
  void updateImu(std::uint64_t now_us) noexcept;
  void controlTick(std::uint64_t now_us) noexcept;
  void driveWheels(bool enabled, const std::array<float, 2>& cmd_rad_s,
                   std::uint64_t now_us) noexcept;
  void report(std::uint64_t now_us) noexcept;
  void printTunables() noexcept;
  bool tunables_pending_ = false;

  [[nodiscard]] bool imuFresh(std::uint64_t now_us) const noexcept;
  [[nodiscard]] bool legsFresh(std::uint64_t now_us) const noexcept;
  [[nodiscard]] float legJointRad(std::size_t leg) const noexcept;
  // Driver velocity estimate (0x009) of a LegLock joint [rad/s], driver frame.
  [[nodiscard]] float legJointRate(std::size_t leg) const noexcept;
  // joint angle (driver frame, rad) the lock is holding this leg at.
  [[nodiscard]] float legTargetRad(std::size_t leg) const noexcept;
  [[nodiscard]] float legChainRad() const noexcept;
  // Straight-leg start (body on the stand, legs hanging straight down, motor supply switched on in that pose): the
  // encoder zero is the fully extended pose, hip -0.9861 rad and knee 1.9722 rad (135 deg) in the CAD frame the
  // controller works in (LegHeight::forKnee(kKneeMaxRad), 184.8 mm axle below the hip). Measured joints get this
  // offset added and every leg target has it subtracted; without it the controller modelled the folded pose
  // (knee 0, 38 mm) while the robot stood tall.
  static constexpr float kStraightHipRad = -0.98611111F;
  static constexpr float kStraightKneeRad = 1.97222222F;
  // Lying start (SET lying 1): the motor supply was switched on with the robot in the `home` lying pose, so the
  // driver zero is that pose: hip 1.3055 rad, knee 0 in the same CAD frame. STANDUP works from here.
  static constexpr float kHomeHipRad = 1.30550628F;
  static constexpr float kHomeKneeRad = 0.0F;
  bool lying_start_ = false;
  bool standup_pending_ = false;  // STANDUP request (SET lying 1, SET standup 1)
  // Stand-up POS velocity feed-forward: filtered rate of the commanded hip/knee path [core rad/s].
  float ff_hip_prev_ = 0.0F, ff_knee_prev_ = 0.0F, ff_hip_rate_ = 0.0F, ff_knee_rate_ = 0.0F;
  std::uint64_t ff_prev_us_ = 0U;
  [[nodiscard]] float homeHip() const noexcept { return lying_start_ ? kHomeHipRad : kStraightHipRad; }
  [[nodiscard]] float homeKnee() const noexcept { return lying_start_ ? kHomeKneeRad : kStraightKneeRad; }
  [[nodiscard]] const char* legAxisProblem(std::uint64_t now_us, bool need_closed_loop) const noexcept;

  Mpu6500Spi mpu_{};
  robot_control::ImuStateEstimator estimator_{};
  Esp32TwaiPort can_{};
  // Transport bus 0 = left wheel (id 2), bus 1 = right wheel (id 1); see MotorMapping.hpp.
  // Kpower BL4012 wheel drivers: one half-duplex single-wire bus per wheel (ported from v3.5.2).
  Esp32UartPort uart1_{2, tap_fw::board::kWheelLeftGpio,
                       tap_fw::board::kWheelLeftUnusedRxGpio, 2U, "left"};
  Esp32UartPort uart2_{1, tap_fw::board::kWheelRightGpio,
                       tap_fw::board::kWheelRightUnusedRxGpio, 1U, "right"};
  tap_fw::CallbackBusTransport transport_;
  tap_fw::MotorFeedback feedback_{};
  tap_fw::v3::LegLock legs_{};
  tap_fw::v3::V3Controller ctrl_;

  Mpu6500Spi::Sample chip_{};
  Mpu6500Spi::Sample body_{};
  robot_control::ImuStateEstimator::Output imu_{};
  robot_control::RobotState state_{};

  std::uint64_t next_control_us_ = 0U;
  // Loop timing (SENS reports and resets them): real tick period fed to the
  // controller, worst tick execution time, ticks started >1 period late.
  std::uint64_t last_tick_us_ = 0U;
  float tick_dt_s_ = 0.002F;
  std::uint32_t tick_exec_max_us_ = 0U;
  std::uint32_t tick_imu_max_us_ = 0U;
  std::uint32_t tick_ctrl_max_us_ = 0U;
  std::uint32_t tick_period_max_us_ = 0U, tick_overrun_ = 0U;
  std::uint32_t tick_report_max_us_ = 0U;
  // Per-section maxima of the control tick [us] (SENS prof=[update legs
  // wheels trace]) and the mean controller time (upd_avg).
  std::array<std::uint32_t, 4> prof_max_{};
  std::uint32_t prof_upd_sum_ = 0U, prof_upd_n_ = 0U;
  void prof(int i, std::uint64_t us) noexcept {
    prof_max_[static_cast<std::size_t>(i)] =
        std::max(prof_max_[static_cast<std::size_t>(i)], static_cast<std::uint32_t>(us));
  }
  std::uint32_t pre_serial_max_us_ = 0U;  // per loop() pass, not only control ticks
  std::uint32_t pre_ble_max_us_ = 0U;
  std::uint32_t pre_bus_max_us_ = 0U;
  std::uint32_t tick_late_ = 0U;
  float hip_vel_sq_ = 0.0F;  // hip velocity estimate noise, per SENS report
  std::uint32_t hip_vel_n_ = 0U;
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
  // Joint tests and recordings, one RAM recorder (250 Hz, <= 8 s) dumped
  // afterwards as "RC" lines (tools/fit_leg_servo.py):
  //  - HIPTEST / KNEETEST [amp]: robot on the stand, legs locked, wheels off;
  //    steps + 1/2/4/8 Hz sines on both hips / knees (GDS68 lag, no load);
  //  - KNEETEST FLOOR [amp]: balancing on the floor, knee sines at 0.5-3 Hz
  //    through the height pose (knee under the body's weight);
  //  - REC [s]: record only (pushes, bumps, turns).
  enum class RecKind : std::uint8_t { None, HipStand, KneeStand, KneeFloor, Free };
  RecKind rec_kind_ = RecKind::None;
  // Start a free 8 s / 250 Hz REC at every stand-up handoff into Balancing, to identify the wheel
  // command-to-speed response (wL, wR, wcmd) in the real operating regime. SET recbal 0/1.
  bool rec_balance_auto_ = true;
  bool rec_active_ = false;
  bool rec_dumping_ = false;
  bool rec_aborted_ = false;
  std::uint64_t rec_start_us_ = 0U;
  std::uint64_t rec_next_us_ = 0U;
  float rec_duration_s_ = 0.0F;
  float rec_amp_ = 0.0F;
  float rec_centre_ = 0.0F;
  float rec_cmd_ = 0.0F;  // current excitation (rad)
  std::uint32_t rec_resend_us_ = 4'000U;
  std::size_t rec_n_ = 0U;
  std::size_t rec_dump_i_ = 0U;
  [[nodiscard]] bool recStand() const noexcept {
    return rec_kind_ == RecKind::HipStand || rec_kind_ == RecKind::KneeStand;
  }
  void startRecording(RecKind kind, float amp, float duration_s, std::uint64_t now_us) noexcept;
  void stopRecording(bool aborted) noexcept;
  [[nodiscard]] float recExcitation(std::uint64_t now_us) const noexcept;
  void recordSample(std::uint64_t now_us, float hip_cmd, float knee_cmd,
                    float wheel_cmd) noexcept;
  void dumpRecording() noexcept;
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
  int last_ready_phase_ = -1;
  char line_[96]{};
  std::size_t line_length_ = 0U;
  std::array<float, 2> last_wheel_cmd_{};
};

}  // namespace tap_hw
