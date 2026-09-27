#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE

#include "tap_hw/FirmwareApp.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>

#include <DabbleESP32.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <esp_timer.h>

#include "robot_control/WholeBodyModel.hpp"
#include "tap_fw/SerialTuning.hpp"

namespace tap_hw {
namespace {

constexpr float kGravity = 9.80665F;
constexpr std::uint64_t kStatusPeriodUs = 200'000U;
constexpr std::uint64_t kCanDiagPeriodUs = 1'000'000U;
constexpr std::uint64_t kBleAdvertiseRestartDelayUs = 300'000U;

std::atomic<int> g_ble_links{0};
std::atomic<bool> g_ble_needs_advertising{false};

void onGattsEvent(esp_gatts_cb_event_t event, esp_gatt_if_t,
                  esp_ble_gatts_cb_param_t*) {
  if (event == ESP_GATTS_CONNECT_EVT) {
    g_ble_links.fetch_add(1);
  } else if (event == ESP_GATTS_DISCONNECT_EVT) {
    int links = g_ble_links.load();
    while (links > 0 && !g_ble_links.compare_exchange_weak(links, links - 1)) {}
    g_ble_needs_advertising.store(true);
  }
}

void writeCommand(tap_fw::ble::CommandEndpoint& endpoint, const char* text,
                  std::uint64_t now_us) {
  std::size_t length = 0U;
  while (text[length] != '\0') ++length;
  (void)endpoint.onWrite(reinterpret_cast<const std::uint8_t*>(text), length,
                         now_us);
}

robot_control::EmbeddedController::Config selectedControllerConfig() {
#if defined(TAP_IMPULSE_STYLE_V2)
  return tap_fw::impulseStyleFirmwareControllerConfig();
#else
  return tap_fw::firmwareControllerConfig();
#endif
}

tap_fw::ImpulseStartupSequencer::Config selectedStartupConfig() noexcept {
#if defined(TAP_IMPULSE_STYLE_V2)
  return {tap_fw::board::kImpulsePrepositionTimeoutUs,
          tap_fw::board::kImpulseBalanceRampTimeoutUs,
          tap_fw::board::kImpulseBalanceWarmupUs};
#else
  return {};
#endif
}

}  // namespace

static tap_fw::ble::DabbleGamepadMapper::Config gamepadConfig() noexcept {
  tap_fw::ble::DabbleGamepadMapper::Config config{};
  config.auto_fixture_release = tap_fw::board::kAutoFixtureRelease;
  return config;
}

FirmwareApp::FirmwareApp() noexcept
    : gamepad_(endpoint_, gamepadConfig()),
      transport_(can_.port(), uart1_.port(), uart2_.port()),
      control_loop_(transport_, selectedControllerConfig()),
      startup_sequencer_(selectedStartupConfig()) {
  // The bridge converts core joint targets with the same zero the feedback
  // path applies, so both sides stay on one reference.
  control_loop_.setLegOffsets(&feedback_.legOffsets());
}

bool FirmwareApp::begin() noexcept {
  // Without a TX ring buffer every status line blocks the control loop until
  // it has drained at 115200 baud (~11.5 bytes/ms).
  Serial.setTxBufferSize(4096U);
  Serial.begin(115200);
  delay(300);
  pinMode(tap_fw::board::kLed1Gpio, OUTPUT);
  pinMode(tap_fw::board::kLed2Gpio, OUTPUT);
  pinMode(tap_fw::board::kLed3Gpio, OUTPUT);
  digitalWrite(tap_fw::board::kLed1Gpio, LOW);
  digitalWrite(tap_fw::board::kLed2Gpio, LOW);
  digitalWrite(tap_fw::board::kLed3Gpio, LOW);

#if defined(TAP_IMPULSE_STYLE_V2)
  Serial.println("TAP ESP32-S3 firmware: Impulse-style staged startup v2");
#else
  Serial.println("TAP ESP32-S3 firmware: sensor + motor-bus bring-up");
#endif
  Serial.printf("TWAI TX/RX=%d/%d bitrate=%lu\n",
                tap_fw::board::kCanTxGpio, tap_fw::board::kCanRxGpio,
                static_cast<unsigned long>(tap_fw::board::kCanBitrate));
  Serial.printf("UART1 TX/RX=%d/%d UART2 TX/RX=%d/%d baud=%lu\n",
                tap_fw::board::kUart1TxGpio, tap_fw::board::kUart1RxGpio,
                tap_fw::board::kUart2TxGpio, tap_fw::board::kUart2RxGpio,
                static_cast<unsigned long>(tap_fw::board::kMotorUartBaud));

  can_ready_ = can_.begin();
  uart1_ready_ = uart1_.begin(tap_fw::board::kMotorUartBaud);
  uart2_ready_ = uart2_.begin(tap_fw::board::kMotorUartBaud);
  imu_ready_ = imu_.begin();
  digitalWrite(tap_fw::board::kLed1Gpio, imu_ready_ ? HIGH : LOW);
  Serial.printf("TWAI=%s UART1=%s UART2=%s MPU6500 WHO_AM_I=0x%02X (%s)\n",
                can_ready_ ? "PASS" : "FAIL",
                uart1_ready_ ? "READY" : "FAIL",
                uart2_ready_ ? "READY" : "FAIL",
                imu_.whoAmI(), imu_ready_ ? "PASS" : "FAIL");
  if (!tap_fw::board::kWheelDirectionsVerified) {
    Serial.println("ACTUATION LOCKED: wheel direction mapping is provisional");
  }
  if (!tap_fw::board::kWheelSpeedScaleVerified) {
    Serial.println("ACTUATION LOCKED: wheel raw-speed scale is unverified");
  }
  if (!tap_fw::board::kImuFrameVerified) {
    Serial.println("ACTUATION LOCKED: MPU6500 axis frame has not been verified");
  }
  hang_pose_ = tap_fw::hangPose(tap_fw::board::kKneeExtensionStopRad);
  {
    Preferences prefs;
    float stored[4]{};
    if (prefs.begin("tap", true)) {
      taught_natural_valid_ = prefs.getBytes("natural", stored, sizeof(stored)) == sizeof(stored);
      prefs.end();
    }
    if (taught_natural_valid_) {
      taught_natural_[0] = stored[0];
      taught_natural_[1] = stored[1];
      taught_natural_[3] = stored[2];
      taught_natural_[4] = stored[3];
      Serial.printf("NATURAL taught: LH=%+.4f LK=%+.4f RH=%+.4f RK=%+.4f rad\n",
                    stored[0], stored[1], stored[2], stored[3]);
    } else {
      Serial.println("NATURAL: CAD pose (no TEACH NATURAL stored)");
    }
  }
#if defined(TAP_IMPULSE_STYLE_V2)
  Serial.printf("STARTUP v2: lift and steady the body; extend both legs straight "
                "down with knees at their extension stops. Start captures the "
                "extended pose (hip reference model=%+.4f rad) and holds it "
                "through balance startup. Actuation stays locked until mapping "
                "is verified.\n",
                hang_pose_.hip_rad);
  if (!tap_fw::board::kManualExtendedPoseMappingVerified) {
    Serial.println("ACTUATION LOCKED: calibrate the physical knee extension-stop "
                   "angle and verify the manual hip pose mapping first.");
  }
  if (!tap_fw::board::kImpulseExtendedLegBalanceVerified) {
    Serial.println("ACTUATION LOCKED: extended-leg balance startup has not been "
                   "validated on TAP.");
  }
#else
  if (tap_fw::board::kLegHomingPose == tap_fw::board::LegHomingPose::GravityHang) {
    Serial.printf("LEG ZERO: not homed. Hold the body up with both legs hanging "
                  "free; Start #1 (or console HOME HANG) homes them (hang pose "
                  "hip=%+.4f knee=%+.4f rad at pitch 0), then the legs fold to "
                  "the stance in the air.\n", hang_pose_.hip_rad, hang_pose_.knee_rad);
  } else if (tap_fw::board::kLegHomingPose == tap_fw::board::LegHomingPose::ManualExtended) {
    Serial.printf("TWO-WHEEL MODE: arrange the legs, stand the robot on its wheels and "
                  "hold it still at the balance point; one START captures the leg pose "
                  "(zero model hip=%+.4f knee=%+.4f rad), locks the four leg joints "
                  "there and balances on the wheels.\n", hang_pose_.hip_rad,
                  hang_pose_.knee_rad);
  } else {
    Serial.println("LEG ZERO: not homed. Mount the robot on the fixture (neutral "
                   "pose); the first Start homes the legs.");
  }
  Serial.println(tap_fw::board::kAutoFixtureRelease
      ? "Start: ENABLE + fixture release (straight into balance)"
      : "Start #1: home legs + FixtureHold. Stand the robot at its balance "
        "point, then Start #2: release into balance (angle seeded from pitch)");
#endif
  if (tap_fw::board::kCanListenOnly) {
    Serial.println("ACTUATION LOCKED: CAN is listen-only (bench feedback mode)");
  }
  if (tap_fw::board::kCanTxSingleFrameTest) {
    Serial.println("ACTUATION LOCKED: CAN single-frame TX test; motor CAN TX suppressed");
  }
  if (tap_fw::board::kCanBitrate == 500'000U) {
    Serial.println("CAN configured at 500 kbit/s (GDS68 bring-up rate)");
  }

  // Precompute the LQR gain schedule before anything can enable. Solving it
  // online took 52 ms inside the first balance tick on the robot and stalled
  // every feedback path past its freshness limit.
  const auto table_start_ms = millis();
  while (!control_loop_.controller().buildLqrGainTableStep(
      tap_fw::board::kLqrGainTablePoints)) {
    delay(1);
  }
  Serial.printf("LQR gain schedule: %u points in %lu ms\n",
                tap_fw::board::kLqrGainTablePoints,
                static_cast<unsigned long>(millis() - table_start_ms));

  if (tap_fw::board::kBleEnabled) {
    Dabble.begin("TAP-RB");
    BLEDevice::setCustomGattsHandler(onGattsEvent);
    Serial.println("Dabble BLE advertising as TAP-RB");
  } else {
    Serial.println("BLE off: control over USB serial (tools/tap_serial_control_gui.py)");
  }

  const std::uint64_t now_us = static_cast<std::uint64_t>(esp_timer_get_time());
  next_control_us_ = now_us;
  next_ble_poll_us_ = now_us;
  return can_ready_ && uart1_ready_ && uart2_ready_ && imu_ready_;
}

void FirmwareApp::loop() noexcept {
  const auto now_us = static_cast<std::uint64_t>(esp_timer_get_time());
  pollBle(now_us);
  pollSerialConsole();
  can_.service(now_us);
  drainMotorFeedback(now_us);
  if (feedback_.superviseLegZero(now_us)) {
    digitalWrite(tap_fw::board::kLed2Gpio, LOW);
#if defined(TAP_IMPULSE_STYLE_V2)
    writeCommand(endpoint_, "DISABLE", now_us);
    Serial.println("LEG ZERO lost: driver reset suspected; manually realign the "
                   "natural pose and press Start to re-home; controller disabled.");
#else
    Serial.println("LEG ZERO lost: leg feedback silent (driver reboot?). "
                   "Re-home on the fixture before the next Start.");
#endif
  }

  if (now_us >= next_control_us_) {
    next_control_us_ = now_us + tap_fw::board::kControlPeriodUs;
    const auto cycle_start = static_cast<std::uint64_t>(esp_timer_get_time());
    updateImu(now_us);
    controlTick(now_us);
    const bool poll_ok = feedback_.poll(
        transport_, now_us, control_loop_.last_command().enable_motors);
    const auto cycle_us = static_cast<std::uint32_t>(
        static_cast<std::uint64_t>(esp_timer_get_time()) - cycle_start);
    cycle_max_us_ = std::max(cycle_max_us_, cycle_us);
    cycle_sum_us_ += cycle_us;
    ++cycle_count_;
    if (!poll_ok && control_loop_.last_command().enable_motors) {
      writeCommand(endpoint_, "DISABLE", now_us);
      Serial.println("Motor feedback poll failed; controller disabled");
    }
    report(now_us);
  }
  delayMicroseconds(100U);
}

void FirmwareApp::pollSerialConsole() noexcept {
  // Bounded per loop so a flood of console bytes cannot starve control.
  for (int budget = 32; budget > 0 && Serial.available() > 0; --budget) {
    const int c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (console_length_ + 1U < sizeof(console_line_)) {
        console_line_[console_length_++] = static_cast<char>(c);
      }
      continue;
    }
    console_line_[console_length_] = '\0';
    console_length_ = 0U;
    if (console_line_[0] == '\0') continue;
    const auto command_now_us = static_cast<std::uint64_t>(esp_timer_get_time());
    if (std::strcmp(console_line_, "START") == 0) {
      const bool session_live = endpoint_.motionAllowed() &&
          !endpoint_.timedOut(command_now_us);
      if (!tap_fw::board::kAutoFixtureRelease && session_live) {
        writeCommand(endpoint_, "FIXTURE_RELEASE", command_now_us);
      } else {
        writeCommand(endpoint_, "ENABLE", command_now_us);
        if (tap_fw::board::kAutoFixtureRelease) {
          writeCommand(endpoint_, "FIXTURE_RELEASE", command_now_us);
        }
      }
      Serial.println("SERIAL START requested");
      continue;
    }
    const bool endpoint_command =
        std::strcmp(console_line_, "HEARTBEAT") == 0 ||
        std::strcmp(console_line_, "DISABLE") == 0 ||
        std::strcmp(console_line_, "ESTOP") == 0 ||
        std::strcmp(console_line_, "CLEAR_ESTOP") == 0 ||
        std::strcmp(console_line_, "STOP") == 0 ||
        std::strcmp(console_line_, "FIXTURE_RELEASE") == 0 ||
        std::strncmp(console_line_, "DRIVE ", 6U) == 0 ||
        std::strncmp(console_line_, "HEIGHT ", 7U) == 0;
    if (endpoint_command) {
      std::size_t length = 0U;
      while (console_line_[length] != '\0') ++length;
      const auto result = endpoint_.onWrite(
          reinterpret_cast<const std::uint8_t*>(console_line_), length,
          command_now_us);
      Serial.printf("SERIAL command %s (%s)\n", console_line_,
                    result == tap_fw::ble::WriteResult::Accepted ? "accepted" :
                    result == tap_fw::ble::WriteResult::Heartbeat ? "heartbeat" :
                    "rejected");
      continue;
    }
    if (std::strncmp(console_line_, "ZERO SET ", 9U) == 0) {
      // Offsets printed by an earlier LEG ZERO line of this motor power cycle.
      float lh = 0.0F, lk = 0.0F, rh = 0.0F, rk = 0.0F;
      if (control_loop_.last_command().enable_motors ||
          std::sscanf(console_line_ + 9, "%f %f %f %f", &lh, &lk, &rh, &rk) != 4) {
        Serial.println("ZERO SET refused: usage ZERO SET <LH> <LK> <RH> <RK>, motors off");
        continue;
      }
      robot_control::JointArray offsets{};
      offsets[0] = lh;
      offsets[1] = lk;
      offsets[3] = rh;
      offsets[4] = rk;
      feedback_.restoreLegOffsets(offsets);
      // A restored zero was captured at the extended pose.
      startup_extended_pose_ = tap_fw::hangingLegPose(hang_pose_, 0.0F);
      startup_extended_pose_valid_ = true;
      digitalWrite(tap_fw::board::kLed2Gpio, HIGH);
      Serial.printf("LEG ZERO restored: offsets LH=%+.4f LK=%+.4f RH=%+.4f RK=%+.4f\n",
                    lh, lk, rh, rk);
      continue;
    }
    if (std::strncmp(console_line_, "TEACH NATURAL", 13U) == 0) {
      const char* side = console_line_ + 13;
      while (*side == ' ') ++side;
      teachNatural(static_cast<std::uint64_t>(esp_timer_get_time()),
                   *side == 'R' || *side == 'L' ? *side : 'A');
      continue;
    }
    if (std::strncmp(console_line_, "SET leg_kp ", 11U) == 0 ||
        std::strncmp(console_line_, "SET leg_kd ", 11U) == 0) {
      const bool is_kp = console_line_[9] == 'p';
      const float value = std::strtof(console_line_ + 11, nullptr);
      const float max = is_kp ? 60.0F : 3.0F;
      if (!(value >= 0.0F && value <= max)) {
        Serial.printf("SET %s refused: 0 .. %.0f\n", is_kp ? "leg_kp" : "leg_kd", max);
        continue;
      }
      const float kp = is_kp ? value : control_loop_.legFixedKp();
      const float kd = is_kp ? control_loop_.legFixedKd() : value;
      control_loop_.setLegFixedGains(kp, kd);
      Serial.printf("LEG gains Kp=%.2f N.m/rad Kd=%.3f N.m.s/rad (next leg frame)\n", kp, kd);
      continue;
    }
    if (std::strncmp(console_line_, "SET leg_current ", 16U) == 0) {
      const float amps = std::strtof(console_line_ + 16, nullptr);
      if (!(amps >= 0.5F && amps <= 10.5F)) {
        Serial.println("SET leg_current refused: 0.5 .. 10.5 A (GIM6010-8 rating)");
        continue;
      }
      control_loop_.setLegCurrentLimit(amps);
      Serial.printf("LEG current limit %.2f A (~%.2f N.m at Kt 0.47), sent on the next "
                    "command cycle / enable\n", amps, amps * 0.47F);
      continue;
    }
    if (std::strcmp(console_line_, "TEACH CLEAR") == 0) {
      Preferences prefs;
      if (prefs.begin("tap", false)) {
        prefs.remove("natural");
        prefs.end();
      }
      taught_natural_valid_ = false;
      Serial.println("NATURAL: taught pose cleared, CAD pose in use");
      continue;
    }
    if (std::strcmp(console_line_, "HOME HANG") == 0) {
      // Re-take the leg zero now: body held still, legs hanging free.
      if (control_loop_.last_command().enable_motors) {
        Serial.println("HOME refused: motors enabled (press Select first)");
        continue;
      }
      const char* reason = nullptr;
      if (!homeLegsNow(static_cast<std::uint64_t>(esp_timer_get_time()), &reason)) {
        Serial.printf("HOME failed: %s\n", reason);
      }
      continue;
    }
    char reply[160]{};
    if (tap_fw::handleTuningLine(console_line_, control_loop_.controller(), reply,
                                 sizeof(reply))) {
      Serial.println(reply);
    } else {
      Serial.printf("TUNE error: unknown command '%s' (GET | SET <key> <v> | TRIM RESET | "
                    "HOME HANG | ZERO SET | TEACH NATURAL [R|L] | TEACH CLEAR)\n",
                    console_line_);
    }
  }
}

void FirmwareApp::pollBle(std::uint64_t now_us) noexcept {
  if (!tap_fw::board::kBleEnabled) return;
  if (now_us < next_ble_poll_us_) return;
  next_ble_poll_us_ = now_us + tap_fw::board::kBlePollPeriodUs;
  // Bytes waiting from the phone mean a fresh Dabble packet; the D-pad lease
  // below is measured against this, not against our own re-writes.
  if (esp32ble.available() > 0) {
    if (last_phone_rx_us_ != 0U) {
      phone_gap_max_us_ = std::max(phone_gap_max_us_, now_us - last_phone_rx_us_);
    }
    last_phone_rx_us_ = now_us;
    ++phone_packets_;
  }
  Dabble.processInput();

  const bool link_up = g_ble_links.load() > 0;
  if (link_up != ble_link_up_) {
    ble_link_up_ = link_up;
    ble_inputs_armed_ = false;
    digitalWrite(tap_fw::board::kLed3Gpio, link_up ? HIGH : LOW);
    Serial.printf("BLE %s\n", link_up ? "connected" : "disconnected");
    if (!link_up) endpoint_.onDisconnect();
  }
  if (g_ble_needs_advertising.exchange(false)) {
    ble_advertise_at_us_ = now_us + kBleAdvertiseRestartDelayUs;
  }
  if (ble_advertise_at_us_ != 0U && now_us >= ble_advertise_at_us_) {
    ble_advertise_at_us_ = 0U;
    BLEDevice::startAdvertising();
    Serial.println("BLE advertising restarted");
  }

  tap_fw::ble::GamepadState input{};
  if (link_up) {
    input = tap_fw::ble::GamepadState{
        GamePad.isUpPressed(), GamePad.isDownPressed(),
        GamePad.isLeftPressed(), GamePad.isRightPressed(),
        GamePad.isStartPressed(), GamePad.isSelectPressed(),
        GamePad.isTrianglePressed(), GamePad.isCirclePressed(),
        GamePad.isCrossPressed(), GamePad.isSquarePressed()};
    const bool any_pressed = input.up || input.down || input.left ||
        input.right || input.start || input.select || input.triangle ||
        input.circle || input.cross || input.square;
    if (!ble_inputs_armed_) {
      if (!any_pressed) ble_inputs_armed_ = true;
      input = tap_fw::ble::GamepadState{};
    }
    // A held D-pad only drives while the phone keeps sending; a frozen app
    // leaves the last state latched in the Dabble library.
    const bool lease_expired = last_phone_rx_us_ == 0U ||
        now_us - last_phone_rx_us_ > tap_fw::board::kBleMotionLeaseUs;
    if (lease_expired && (input.up || input.down || input.left || input.right)) {
      input.up = input.down = input.left = input.right = false;
      if (!motion_lease_expired_) {
        Serial.println("BLE motion lease expired: no phone packet, D-pad ignored");
      }
      motion_lease_expired_ = true;
    } else if (!lease_expired) {
      motion_lease_expired_ = false;
    }
  }
  if (link_up) gamepad_.update(input, now_us);
}

void FirmwareApp::drainMotorFeedback(std::uint64_t now_us) noexcept {
  tap_fw::motor_transport::CanFrame can_frame;
  unsigned can_count = 0U;
  while (can_count < 64U && transport_.receiveCan(can_frame)) {
    (void)feedback_.ingestCan(can_frame, now_us);
    control_loop_.onCanReceived(can_frame);
    ++can_count;
  }
  for (std::uint8_t bus = 0U; bus < 2U; ++bus) {
    tap_fw::motor_frame::DxlFrame uart_frame;
    unsigned uart_count = 0U;
    while (uart_count < 8U && transport_.receiveUart(bus, uart_frame)) {
      (void)feedback_.ingestUart(bus, uart_frame, now_us);
      ++uart_count;
    }
  }
}

void FirmwareApp::updateImu(std::uint64_t now_us) noexcept {
  if (!imu_ready_ || !imu_.read(imu_chip_sample_)) return;
  // Everything downstream (estimator, homing, leg kinematics) is body frame.
  imu_sample_.accel_m_s2 = tap_fw::imu_mount::toBody(
      tap_fw::board::kImuAxisMap, imu_chip_sample_.accel_m_s2);
  imu_sample_.gyro_rad_s = tap_fw::imu_mount::toBody(
      tap_fw::board::kImuAxisMap, imu_chip_sample_.gyro_rad_s);
  feedback_.writeTo(state_);

  const float dt_s = last_imu_us_ == 0U
      ? static_cast<float>(tap_fw::board::kControlPeriodUs) * 1.0e-6F
      : static_cast<float>(now_us - last_imu_us_) * 1.0e-6F;
  const float accel_magnitude = std::sqrt(
      imu_sample_.accel_m_s2[0] * imu_sample_.accel_m_s2[0] +
      imu_sample_.accel_m_s2[1] * imu_sample_.accel_m_s2[1] +
      imu_sample_.accel_m_s2[2] * imu_sample_.accel_m_s2[2]);
  const float gyro_magnitude = std::sqrt(
      imu_sample_.gyro_rad_s[0] * imu_sample_.gyro_rad_s[0] +
      imu_sample_.gyro_rad_s[1] * imu_sample_.gyro_rad_s[1] +
      imu_sample_.gyro_rad_s[2] * imu_sample_.gyro_rad_s[2]);

  robot_control::ImuKinematicVelocity kinematic{};
  if (tap_fw::board::kImuWheelKinematicCompensation && feedback_.ready(now_us)) {
    const auto left = robot_control::WholeBodyModel::imuVelocityThroughLeg(
        imu_output_.pitch_rad, imu_sample_.gyro_rad_s[1],
        state_.joints.position_rad[0], state_.joints.position_rad[1],
        state_.joints.velocity_rad_s[0], state_.joints.velocity_rad_s[1],
        state_.joints.velocity_rad_s[2]);
    const auto right = robot_control::WholeBodyModel::imuVelocityThroughLeg(
        imu_output_.pitch_rad, imu_sample_.gyro_rad_s[1],
        state_.joints.position_rad[3], state_.joints.position_rad[4],
        state_.joints.velocity_rad_s[3], state_.joints.velocity_rad_s[4],
        state_.joints.velocity_rad_s[5]);
    kinematic.horizontal_m_s = 0.5F * (left.horizontal_m_s + right.horizontal_m_s);
    kinematic.vertical_m_s = 0.5F * (left.vertical_m_s + right.vertical_m_s);
    kinematic.valid = std::isfinite(kinematic.horizontal_m_s) &&
        std::isfinite(kinematic.vertical_m_s);
  }

  const auto command = endpoint_.command(now_us);
  const bool pose_changing = command.request_pose_change ||
      command.request_height_change ||
      control_loop_.controller().mode() == robot_control::ControlMode::PoseChange;
  const bool quiet = std::abs(accel_magnitude - kGravity) < 0.35F &&
      gyro_magnitude < 0.12F;
  // Homing uses the gyro only: an accelerometer offset of a few tenths of
  // m/s^2 would otherwise make the |a| - g test fail on a robot at rest.
  gyro_quiet_ = gyro_magnitude < 0.12F;
  const auto output = estimator_.update(
      imu_sample_.accel_m_s2, imu_sample_.gyro_rad_s, dt_s,
      static_cast<float>(now_us) * 1.0e-6F, 0.0F, pose_changing, quiet,
      kinematic);
  if (!output.valid) return;

  imu_output_ = output;
  last_imu_us_ = now_us;
  ++imu_sequence_;
  state_.imu_sample_sequence = imu_sequence_;
  state_.imu.pitch_rad = output.pitch_rad;
  state_.imu.sample_dt_s = dt_s;
  state_.imu.pitch_rate_rad_s = output.raw_pitch_rate_rad_s;
  state_.imu.pitch_rate_bias_corrected_rad_s = output.pitch_rate_rad_s;
  state_.imu.pitch_rate_bias_corrected_valid = output.seeded;
  state_.imu.gyro_bias_rad_s = output.gyro_bias_rad_s;
  state_.imu.roll_rate_rad_s = output.roll_rate_rad_s;
  state_.imu.yaw_rate_rad_s = output.yaw_rate_rad_s;
  state_.imu.accel_magnitude_m_s2 = output.accel_magnitude_m_s2;
  state_.imu.hard_impact_event = output.hard_impact_event;
  feedback_.writeTo(state_);
  digitalWrite(tap_fw::board::kLed1Gpio, HIGH);
}

void FirmwareApp::controlTick(std::uint64_t now_us) noexcept {
  state_.dt_s = static_cast<float>(tap_fw::board::kControlPeriodUs) * 1.0e-6F;
  feedback_.writeTo(state_);
  if (feedback_.allWheelsFresh(now_us)) {
    if (!wheel_start_valid_) {
      wheel_start_rad_[0] = state_.joints.position_rad[2];
      wheel_start_rad_[1] = state_.joints.position_rad[5];
      wheel_start_valid_ = true;
    }
    wheel_speed_integral_rad_[0] += state_.joints.velocity_rad_s[2] * state_.dt_s;
    wheel_speed_integral_rad_[1] += state_.joints.velocity_rad_s[5] * state_.dt_s;
  }

  auto command = endpoint_.command(now_us);
  if (command.request_enable) {
    switch (homeLegsIfNeeded(now_us)) {
      case HomeGate::Ready:
        break;
      case HomeGate::Waiting:
        // Keep the Start transaction pending until the selected reference pose
        // has stayed still long enough; no actuator enable is issued yet.
        command.request_enable = false;
        break;
      case HomeGate::Failed:
        writeCommand(endpoint_, "DISABLE", now_us);
        command = endpoint_.command(now_us);
        break;
    }
  } else {
    reported_home_block_ = false;
    home_wait_since_us_ = 0U;
    hang_still_since_us_ = 0U;
  }
  feedback_.writeTo(state_);
  // Enable gate only at the enable edge. While running, the ride-through
  // supervision below applies (a CAN bus-off made the strict 20 ms freshness
  // here disable a holding robot, 2026-09-26).
  if (command.request_enable && !control_loop_.last_command().enable_motors &&
      !readyForEnable(now_us)) {
    writeCommand(endpoint_, "DISABLE", now_us);
    command = endpoint_.command(now_us);
    if (!reported_enable_block_) {
      Serial.printf("ENABLE refused: IMU=%d motors=%d leg-axes=%d wheel-directions=%d wheel-scale=%d imu-frame=%d legs-homed=%d can-tx=%d buses=%d\n",
                    imuFresh(now_us) && imu_output_.seeded,
                    feedback_.ready(now_us),
                    feedback_.firstLegAxisFault(now_us, false) ==
                        robot_control::kJointCount,
                    tap_fw::board::kWheelDirectionsVerified,
                    tap_fw::board::kWheelSpeedScaleVerified,
                    tap_fw::board::kImuFrameVerified,
                    feedback_.legsHomed(),
                    !tap_fw::board::kCanListenOnly,
                    can_ready_ && uart1_ready_ && uart2_ready_);
      reported_enable_block_ = true;
    }
  } else if (!command.request_enable) {
    reported_enable_block_ = false;
  }
#if defined(TAP_IMPULSE_STYLE_V2)
  const auto mode_before_sequence = control_loop_.controller().mode();
  const bool balancing = mode_before_sequence == robot_control::ControlMode::Balance ||
      mode_before_sequence == robot_control::ControlMode::Drive ||
      mode_before_sequence == robot_control::ControlMode::PoseChange;
  const auto startup = startup_sequencer_.update({
      now_us,
      command.request_enable,
      command.request_disable || command.request_emergency_stop,
      command.request_fixture_release,
      mode_before_sequence == robot_control::ControlMode::FixtureHold,
      control_loop_.controller().fixtureHoldArrived(),
      mode_before_sequence == robot_control::ControlMode::BalanceRamp,
      balancing});
  if (startup.failed) {
    writeCommand(endpoint_, "DISABLE", now_us);
    command = endpoint_.command(now_us);
    Serial.printf("STARTUP v2 failed in %s; controller disabled\n",
                  tap_fw::toString(startup.stage));
  } else {
    if (!startup.allow_fixture_release) command.request_fixture_release = false;
    if (!startup.allow_drive) {
      command.target_forward_velocity_m_s = 0.0F;
      command.target_yaw_rate_rad_s = 0.0F;
      command.request_pose_change = false;
      command.request_height_change = false;
      command.request_stand_up = false;
    }
    const int stage = static_cast<int>(startup.stage);
    if (stage != last_startup_stage_) {
      last_startup_stage_ = stage;
      Serial.printf("STARTUP v2 stage=%s pitch=%+.4f\n",
                    tap_fw::toString(startup.stage), state_.imu.pitch_rad);
      if (startup.stage == tap_fw::ImpulseStartupSequencer::Stage::Run) {
        Serial.println("BALANCE MODE active after 1 s warmup; verify wheel contact "
                       "and stable self-support before removing body support. "
                       "This is a controller-state message, not a balance proof.");
      }
    }
  }
#endif
  // Enabled: tolerate a short leg CAN outage (bus-off recovery); IMU and
  // wheels must stay fresh.
  if (control_loop_.last_command().enable_motors &&
      !(imuFresh(now_us) && imu_output_.seeded && feedback_.readyToHold(now_us))) {
    writeCommand(endpoint_, "DISABLE", now_us);
    command = endpoint_.command(now_us);
    Serial.println("Sensor feedback stale; software disable requested");
  }

  applyStartupLegPose(command);
  const auto result = control_loop_.step(state_, command, now_us);
  if (result == tap_fw::ControllerActuatorBridge::Result::TransportError &&
      control_loop_.last_command().enable_motors) {
    // Ride through a CAN bus-off (recovery ~20 ms); give up only if the
    // transport keeps failing past kLegRideThroughUs.
    if (transport_error_since_us_ == 0U) {
      transport_error_since_us_ = now_us;
      ++transport_error_events_;
    } else if (now_us - transport_error_since_us_ > tap_fw::board::kLegRideThroughUs) {
      writeCommand(endpoint_, "DISABLE", now_us);
      Serial.println("Actuator transport failed; software disable requested");
      transport_error_since_us_ = 0U;
    }
  } else if (result == tap_fw::ControllerActuatorBridge::Result::Sent) {
    transport_error_since_us_ = 0U;
  }

  // The legs are only held if every GDS68 actually entered CLOSED_LOOP.
  const bool legs_enabled = control_loop_.last_command().enable_motors;
  if (legs_enabled && !legs_enabled_) {
    legs_enabled_since_us_ = now_us;
    trace_start_us_ = now_us;
    trace_until_us_ = now_us + tap_fw::board::kBalanceTraceUs;
    next_trace_us_ = now_us;
  }
  const auto& motors = control_loop_.last_command();
  if (legs_enabled && now_us < trace_until_us_ && now_us >= next_trace_us_) {
    next_trace_us_ = now_us + tap_fw::board::kBalanceTracePeriodUs;
    const auto& lqr = control_loop_.controller().lqrDebugState();
    Serial.printf("TR t=%lu mode=%s pitch=%+.3f rate=%+.2f wcmd=[%+.2f %+.2f] "
                  "wmeas=[%+.2f %+.2f] axle_v=%+.3f axle_x=%+.3f vcmd=%+.3f "
                  "pref=%+.4f trim=%+.4f\n",
                  static_cast<unsigned long>((now_us - trace_start_us_) / 1000U),
                  robot_control::toString(control_loop_.controller().mode()),
                  state_.imu.pitch_rad, state_.imu.pitch_rate_rad_s,
                  motors.velocity_rad_s[2], motors.velocity_rad_s[5],
                  state_.joints.velocity_rad_s[2], state_.joints.velocity_rad_s[5],
                  state_.wheel.velocity_m_s, state_.wheel.position_m,
                  lqr.commanded_ground_velocity_m_s, lqr.lqr_pitch_ref_rad,
                  control_loop_.controller().balanceTrim());
  }
  // Leg trace while the legs are held (FixtureHold): command vs measured
  // angle, driver torque estimate (MIT Iq*Kt) and the firmware integral.
  if (legs_enabled &&
      control_loop_.controller().mode() == robot_control::ControlMode::FixtureHold &&
      now_us >= next_leg_trace_us_) {
    next_leg_trace_us_ = now_us + 100'000U;
    const auto& integral = control_loop_.legTargetIntegral();
    Serial.printf("LEG cmd=[%+.3f %+.3f %+.3f %+.3f] q=[%+.3f %+.3f %+.3f %+.3f] "
                  "tau=[%+.2f %+.2f %+.2f %+.2f] ff=[%+.2f %+.2f %+.2f %+.2f] "
                  "int=[%+.2f %+.2f %+.2f %+.2f]\n",
                  motors.position_rad[0], motors.position_rad[1], motors.position_rad[3],
                  motors.position_rad[4], state_.joints.position_rad[0],
                  state_.joints.position_rad[1], state_.joints.position_rad[3],
                  state_.joints.position_rad[4], state_.joints.effort_nm[0],
                  state_.joints.effort_nm[1], state_.joints.effort_nm[3],
                  state_.joints.effort_nm[4], motors.feedforward_nm[0],
                  motors.feedforward_nm[1], motors.feedforward_nm[3],
                  motors.feedforward_nm[4], integral[0], integral[1], integral[3],
                  integral[4]);
  }
  const bool wheels_fast =
      std::abs(state_.joints.velocity_rad_s[2]) > tap_fw::board::kWheelRunawayRadS &&
      std::abs(state_.joints.velocity_rad_s[5]) > tap_fw::board::kWheelRunawayRadS;
  if (!legs_enabled || !wheels_fast) {
    wheel_fast_since_us_ = 0U;
  } else if (wheel_fast_since_us_ == 0U) {
    wheel_fast_since_us_ = now_us;
  } else if (now_us - wheel_fast_since_us_ > tap_fw::board::kWheelRunawayUs) {
    writeCommand(endpoint_, "DISABLE", now_us);
    Serial.printf("Wheel runaway: |v| > %.0f rad/s for %lu ms; controller disabled\n",
                  tap_fw::board::kWheelRunawayRadS,
                  static_cast<unsigned long>(tap_fw::board::kWheelRunawayUs / 1000U));
    wheel_fast_since_us_ = 0U;
  }
  legs_enabled_ = legs_enabled;
  if (legs_enabled &&
      now_us - legs_enabled_since_us_ >= tap_fw::board::kClosedLoopGraceUs) {
    const std::size_t joint = feedback_.firstLegAxisFault(now_us, true);
    if (joint != robot_control::kJointCount) {
      static const char* const kNames[] = {"LH", "LK", "LW", "RH", "RK", "RW"};
      const auto& axis = feedback_.legAxis(joint);
      writeCommand(endpoint_, "DISABLE", now_us);
      Serial.printf("Leg %s not holding (axis_state=%u axis_error=0x%08lX "
                    "heartbeat=%s); controller disabled\n",
                    kNames[joint], static_cast<unsigned>(axis.state),
                    static_cast<unsigned long>(axis.error),
                    axis.valid ? "ok" : "none");
    }
  }
  const int mode = static_cast<int>(control_loop_.controller().mode());
  const bool in_fixture_hold =
      mode == static_cast<int>(robot_control::ControlMode::FixtureHold);
  if (in_fixture_hold && control_loop_.controller().fixtureHoldArrived() &&
      !fixture_arrived_reported_) {
    Serial.println("LEGS at stance: put the wheels on the floor at the balance "
                   "point, then Start #2");
    fixture_arrived_reported_ = true;
  } else if (!in_fixture_hold) {
    fixture_arrived_reported_ = false;
  }
  if (mode != last_mode_) {
    if (legs_enabled &&
        last_mode_ == static_cast<int>(robot_control::ControlMode::FixtureHold)) {
      // Fixture release: restart the trace so it covers the balance start.
      trace_start_us_ = now_us;
      trace_until_us_ = now_us + tap_fw::board::kBalanceTraceUs;
      next_trace_us_ = now_us;
      Serial.printf("RELEASE pitch=%+.4f seeded_trim=%+.4f\n", state_.imu.pitch_rad,
                    control_loop_.controller().balanceTrim());
    }
    last_mode_ = mode;
    Serial.printf("CONTROL mode=%s enable=%d result=%u\n",
                  robot_control::toString(control_loop_.controller().mode()),
                  control_loop_.last_command().enable_motors,
                  static_cast<unsigned>(result));
  }
}

bool FirmwareApp::imuFresh(std::uint64_t now_us) const noexcept {
  return last_imu_us_ != 0U && now_us >= last_imu_us_ &&
      now_us - last_imu_us_ <= tap_fw::board::kFeedbackMaxAgeUs;
}

bool FirmwareApp::sensorsReady(std::uint64_t now_us) const noexcept {
  return imuFresh(now_us) && imu_output_.seeded && feedback_.ready(now_us);
}

bool FirmwareApp::readyForEnable(std::uint64_t now_us) const noexcept {
  return can_ready_ && uart1_ready_ && uart2_ready_ && sensorsReady(now_us) &&
      control_loop_.controller().lqrGainTableReady() &&
      tap_fw::board::kWheelDirectionsVerified &&
      tap_fw::board::kWheelSpeedScaleVerified &&
      tap_fw::board::kImuFrameVerified &&
#if defined(TAP_IMPULSE_STYLE_V2)
      tap_fw::board::kManualExtendedPoseMappingVerified &&
      tap_fw::board::kImpulseExtendedLegBalanceVerified &&
#endif
      feedback_.legsHomed() &&
      feedback_.firstLegAxisFault(now_us, false) == robot_control::kJointCount &&
      !tap_fw::board::kCanListenOnly &&
      !tap_fw::board::kCanTxSingleFrameTest;
}

bool FirmwareApp::homeLegsNow(std::uint64_t now_us, const char** reason) noexcept {
  const bool gravity_pose = tap_fw::board::kLegHomingPose !=
      tap_fw::board::LegHomingPose::FixtureNatural;
  if (!imuFresh(now_us) || !imu_output_.seeded) {
    *reason = "IMU not ready";
    return false;
  }
  if (!gyro_quiet_) {
#if defined(TAP_IMPULSE_STYLE_V2)
    *reason = "body moving (steady the lifted body before homing)";
#else
    *reason = gravity_pose ? "body moving (hold the body still, legs hanging)"
                   : "body moving (robot must rest still on the fixture)";
#endif
    return false;
  }
  robot_control::JointArray pose{};
  if (gravity_pose) {
    pose = tap_fw::hangingLegPose(hang_pose_, imu_output_.pitch_rad);
  } else {
    pose[0] = pose[3] = robot_control::geometry::neutral_hip_rad;
    pose[1] = pose[4] = robot_control::geometry::neutral_knee_rad;
  }
  switch (feedback_.homeLegsAt(pose, now_us)) {
    case tap_fw::MotorFeedback::HomeResult::Homed: {
      const auto& offset = feedback_.legOffsets();
      startup_extended_pose_ = pose;
      startup_extended_pose_valid_ = true;
      Serial.printf("LEG ZERO homed %s (body pitch %+.4f): hip=%+.4f knee=%+.4f rad; "
                    "offsets LH=%+.4f LK=%+.4f RH=%+.4f RK=%+.4f\n",
                    gravity_pose ? "extended/hanging" : "on fixture", imu_output_.pitch_rad,
                    pose[0], pose[1], offset[0], offset[1], offset[3], offset[4]);
      digitalWrite(tap_fw::board::kLed2Gpio, HIGH);
      return true;
    }
    case tap_fw::MotorFeedback::HomeResult::StaleFeedback:
      *reason = "leg MIT feedback missing";
      return false;
    case tap_fw::MotorFeedback::HomeResult::Moving:
      *reason = "legs moving";
      return false;
  }
  *reason = "unknown";
  return false;
}

void FirmwareApp::teachNatural(std::uint64_t now_us, char side) noexcept {
  if (!feedback_.legsHomed()) {
    Serial.println("TEACH refused: legs not homed (HOME HANG first)");
    return;
  }
  if (control_loop_.last_command().enable_motors) {
    Serial.println("TEACH refused: motors enabled (press Select first)");
    return;
  }
  if (!feedback_.allLegsFresh(now_us)) {
    Serial.println("TEACH refused: leg feedback stale");
    return;
  }
  for (const std::size_t joint : {0U, 1U, 3U, 4U}) {
    const bool used = side == 'A' || (side == 'L' ? joint < 2U : joint >= 3U);
    if (used && std::abs(state_.joints.velocity_rad_s[joint]) > 0.05F) {
      Serial.println("TEACH refused: legs moving, hold them still");
      return;
    }
  }
  const auto& q = state_.joints.position_rad;
  // Both legs share one core convention (driver signs mirror them), so one
  // taught leg can stand for both.
  const float hip_l = side == 'R' ? q[3] : q[0];
  const float knee_l = side == 'R' ? q[4] : q[1];
  const float hip_r = side == 'L' ? q[0] : q[3];
  const float knee_r = side == 'L' ? q[1] : q[4];
  const float stored[4] = {hip_l, knee_l, hip_r, knee_r};
  Preferences prefs;
  bool saved = false;
  if (prefs.begin("tap", false)) {
    saved = prefs.putBytes("natural", stored, sizeof(stored)) == sizeof(stored);
    prefs.end();
  }
  taught_natural_[0] = stored[0];
  taught_natural_[1] = stored[1];
  taught_natural_[3] = stored[2];
  taught_natural_[4] = stored[3];
  taught_natural_valid_ = true;
  Serial.printf("NATURAL taught%s: LH=%+.4f LK=%+.4f RH=%+.4f RK=%+.4f rad "
                "(CAD hip %+.4f knee %+.4f)\n", saved ? "" : " (NOT saved to flash)",
                stored[0], stored[1], stored[2], stored[3],
                robot_control::geometry::neutral_hip_rad,
                robot_control::geometry::neutral_knee_rad);
}

void FirmwareApp::applyStartupLegPose(robot_control::ControlCommand& command) const noexcept {
#if defined(TAP_IMPULSE_STYLE_V2)
  const auto& target = startup_extended_pose_valid_
      ? startup_extended_pose_
      : tap_fw::hangingLegPose(hang_pose_, imu_output_.pitch_rad);
  for (const std::size_t joint : {0U, 1U, 3U, 4U}) {
    command.target_joint_position_rad[joint] = target[joint];
  }
#else
  if (tap_fw::board::kLegsLockedAtZeroPose) {
    // Two-wheel mode: legs locked at the captured extended pose.
    if (!startup_extended_pose_valid_) return;
    for (const std::size_t joint : {0U, 1U, 3U, 4U}) {
      command.target_joint_position_rad[joint] = startup_extended_pose_[joint];
    }
    return;
  }
  if (!taught_natural_valid_) return;
  for (const std::size_t joint : {0U, 1U, 3U, 4U}) {
    command.target_joint_position_rad[joint] = taught_natural_[joint];
  }
#endif
}

FirmwareApp::HomeGate FirmwareApp::homeLegsIfNeeded(std::uint64_t now_us) noexcept {
  if (feedback_.legsHomed()) return HomeGate::Ready;
  const char* reason = nullptr;
  bool settled = true;
  if (tap_fw::board::kLegHomingPose == tap_fw::board::LegHomingPose::GravityHang) {
    if (home_wait_since_us_ == 0U) {
      home_wait_since_us_ = now_us;
      hang_still_since_us_ = 0U;
      Serial.println("HOMING: motors limp. Hold the body up and let both legs "
                     "fall to hanging; the zero is taken after 2 s still.");
    }
    bool still = gyro_quiet_ && imuFresh(now_us) && imu_output_.seeded;
    for (const std::size_t joint : {0U, 1U, 3U, 4U}) {
      still = still && std::abs(state_.joints.velocity_rad_s[joint]) <=
          tap_fw::board::kHomingMaxJointSpeedRadS;
    }
    if (!still) {
      hang_still_since_us_ = 0U;
    } else if (hang_still_since_us_ == 0U) {
      hang_still_since_us_ = now_us;
    }
    settled = hang_still_since_us_ != 0U &&
        now_us - hang_still_since_us_ >= tap_fw::board::kHangSettleUs;
    if (!settled) {
      if (now_us - home_wait_since_us_ <= tap_fw::board::kHangSettleTimeoutUs) {
        return HomeGate::Waiting;
      }
      reason = "legs did not settle hanging within 30 s";
    }
  } else if (tap_fw::board::kLegHomingPose ==
             tap_fw::board::LegHomingPose::ManualExtended) {
    if (home_wait_since_us_ == 0U) {
      home_wait_since_us_ = now_us;
      hang_still_since_us_ = 0U;
      Serial.println("HOMING v2: lift the body, extend both legs straight down to "
                     "the knee stops, and hold still for 0.5 s.");
    }
    bool still = gyro_quiet_ && imuFresh(now_us) && imu_output_.seeded;
    for (const std::size_t joint : {0U, 1U, 3U, 4U}) {
      still = still && std::abs(state_.joints.velocity_rad_s[joint]) <=
          tap_fw::board::kHomingMaxJointSpeedRadS;
    }
    if (!still) {
      hang_still_since_us_ = 0U;
    } else if (hang_still_since_us_ == 0U) {
      hang_still_since_us_ = now_us;
    }
    settled = hang_still_since_us_ != 0U &&
        now_us - hang_still_since_us_ >= tap_fw::board::kManualExtendedSettleUs;
    if (!settled) {
      if (now_us - home_wait_since_us_ <= tap_fw::board::kManualExtendedTimeoutUs) {
        return HomeGate::Waiting;
      }
      reason = "extended pose did not settle within 30 s";
    }
  }
  if (settled && homeLegsNow(now_us, &reason)) {
    reported_home_block_ = false;
    home_wait_since_us_ = 0U;
    hang_still_since_us_ = 0U;
    return HomeGate::Ready;
  }
  if (!reported_home_block_) {
    Serial.printf("ENABLE refused: leg homing failed (%s)\n", reason);
    reported_home_block_ = true;
  }
  home_wait_since_us_ = 0U;
  hang_still_since_us_ = 0U;
  return HomeGate::Failed;
}

void FirmwareApp::report(std::uint64_t now_us) noexcept {
  if (now_us - last_report_us_ < kStatusPeriodUs) return;
  last_report_us_ = now_us;
  const bool enabled = control_loop_.last_command().enable_motors;
  // Bus diagnostics are long lines; keep them rare while the robot is live.
  if (now_us - last_can_diag_us_ >=
      (enabled ? 5U * kCanDiagPeriodUs : kCanDiagPeriodUs)) {
    last_can_diag_us_ = now_us;
    can_.printDiagnostics(Serial);
    if (ble_link_up_) {
      Serial.printf("BLE rx packets=%lu gap_max=%lums\n",
                    static_cast<unsigned long>(phone_packets_),
                    static_cast<unsigned long>(phone_gap_max_us_ / 1000U));
    }
    phone_packets_ = 0U;
    phone_gap_max_us_ = 0U;
  }
  const auto ageMs = [&](std::uint64_t stamp) {
    return stamp == 0U || now_us < stamp ? -1L :
        static_cast<long>((now_us - stamp) / 1000U);
  };
  Serial.printf(
      "SENS imu_age=%ldms seed=%d can_legs=%d uart_wheels=%d homed=%d "
      "joint_q=[%.3f %.3f %.3f %.3f %.3f %.3f] pitch=%+.3f "
      "pitch_rate=%+.3f yaw_rate=%+.3f acc=[%+.2f %+.2f %+.2f] "
      "chip_acc=[%+.2f %+.2f %+.2f] "
      "cycle_us max=%lu avg=%lu wheel_pos_travel=[%+.2f %+.2f] "
      "wheel_speed_integral=[%+.2f %+.2f] trim=%+.4f mit_wait=%lu vbus_min=%.2f "
      "can_dropouts=%lu\n",
      ageMs(last_imu_us_), imu_output_.seeded,
      feedback_.allLegsFresh(now_us), feedback_.allWheelsFresh(now_us),
      feedback_.legsHomed(),
      state_.joints.position_rad[0], state_.joints.position_rad[1],
      state_.joints.position_rad[2], state_.joints.position_rad[3],
      state_.joints.position_rad[4], state_.joints.position_rad[5],
      state_.imu.pitch_rad, state_.imu.pitch_rate_rad_s,
      state_.imu.yaw_rate_rad_s, imu_sample_.accel_m_s2[0],
      imu_sample_.accel_m_s2[1], imu_sample_.accel_m_s2[2],
      imu_chip_sample_.accel_m_s2[0], imu_chip_sample_.accel_m_s2[1],
      imu_chip_sample_.accel_m_s2[2],
      static_cast<unsigned long>(cycle_max_us_),
      static_cast<unsigned long>(cycle_count_ == 0U ? 0U
          : cycle_sum_us_ / cycle_count_),
      wheel_start_valid_ ? state_.joints.position_rad[2] - wheel_start_rad_[0] : 0.0F,
      wheel_start_valid_ ? state_.joints.position_rad[5] - wheel_start_rad_[1] : 0.0F,
      wheel_speed_integral_rad_[0], wheel_speed_integral_rad_[1],
      control_loop_.controller().balanceTrim(),
      static_cast<unsigned long>(control_loop_.mitDeferred()),
      feedback_.takeMinBusVoltage(),
      static_cast<unsigned long>(transport_error_events_));
  cycle_max_us_ = 0U;
  cycle_sum_us_ = 0U;
  cycle_count_ = 0U;
}

}  // namespace tap_hw
