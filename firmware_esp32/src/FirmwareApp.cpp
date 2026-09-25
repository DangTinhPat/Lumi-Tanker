#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE

#include "tap_hw/FirmwareApp.hpp"

#include <cmath>
#include <atomic>
#include <cstdio>

#include <DabbleESP32.h>
#include <BLEDevice.h>
#include <esp_timer.h>

#include "robot_control/WholeBodyModel.hpp"

namespace tap_hw {
namespace {

constexpr float kGravity = 9.80665F;
constexpr std::uint64_t kButtonDebounceUs = 30'000U;
constexpr std::uint64_t kStatusPeriodUs = 200'000U;
constexpr std::uint64_t kButtonGuardUs = 100'000U;
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

}  // namespace

FirmwareApp::FirmwareApp() noexcept
    : gamepad_(endpoint_),
      transport_(can_.port(), uart1_.port(), uart2_.port()) {}

bool FirmwareApp::begin() noexcept {
  Serial.begin(115200);
  delay(300);
  pinMode(tap_fw::board::kLed1Gpio, OUTPUT);
  pinMode(tap_fw::board::kLed2Gpio, OUTPUT);
  pinMode(tap_fw::board::kLed3Gpio, OUTPUT);
  pinMode(tap_fw::board::kButtonGpio, INPUT_PULLUP);
  digitalWrite(tap_fw::board::kLed1Gpio, LOW);
  digitalWrite(tap_fw::board::kLed2Gpio, LOW);
  digitalWrite(tap_fw::board::kLed3Gpio, LOW);

  Serial.println("TAP ESP32-S3 firmware: sensor + motor-bus bring-up");
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
  if (tap_fw::board::kCanBitrate == 1'000'000U) {
    Serial.println("CAN at 1 Mbit/s is the planned rate; harness validation recorded at 500 kbit/s");
  }

  Dabble.begin("TAP-RB");
  BLEDevice::setCustomGattsHandler(onGattsEvent);
  Serial.println("Dabble BLE advertising as TAP-RB");

  const std::uint64_t now_us = static_cast<std::uint64_t>(esp_timer_get_time());
  next_control_us_ = now_us;
  next_ble_poll_us_ = now_us;
  button_raw_ = digitalRead(tap_fw::board::kButtonGpio) != LOW;
  button_stable_ = button_raw_;
  button_candidate_since_us_ = now_us;
  return can_ready_ && uart1_ready_ && uart2_ready_ && imu_ready_;
}

void FirmwareApp::loop() noexcept {
  const auto now_us = static_cast<std::uint64_t>(esp_timer_get_time());
  pollBle(now_us);
  pollFixtureButton(now_us);
  drainMotorFeedback(now_us);

  if (now_us >= next_control_us_) {
    next_control_us_ = now_us + tap_fw::board::kControlPeriodUs;
    updateImu(now_us);
    controlTick(now_us);
    const bool poll_ok = feedback_.poll(
        transport_, now_us, control_loop_.last_command().enable_motors);
    if (!poll_ok && control_loop_.last_command().enable_motors) {
      writeCommand(endpoint_, "DISABLE", now_us);
      Serial.println("Motor feedback poll failed; controller disabled");
    }
    report(now_us);
  }
  delayMicroseconds(100U);
}

void FirmwareApp::pollBle(std::uint64_t now_us) noexcept {
  if (now_us < next_ble_poll_us_) return;
  next_ble_poll_us_ = now_us + tap_fw::board::kBlePollPeriodUs;
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
  }
  gamepad_.update(input, now_us);
}

void FirmwareApp::pollFixtureButton(std::uint64_t now_us) noexcept {
  const bool raw = digitalRead(tap_fw::board::kButtonGpio) != LOW;
  if (raw != button_raw_) {
    button_raw_ = raw;
    button_candidate_since_us_ = now_us;
  }
  if (raw == button_stable_ ||
      now_us - button_candidate_since_us_ < kButtonDebounceUs) return;

  button_stable_ = raw;
  if (button_stable_ || now_us - button_last_event_us_ < kButtonGuardUs) return;
  button_last_event_us_ = now_us;
  if (control_loop_.controller().mode() == robot_control::ControlMode::FixtureHold &&
      sensorsReady(now_us)) {
    fixture_release_pending_ = true;
    Serial.println("Fixture release confirmed by GPIO6 button");
  } else {
    Serial.println("GPIO6 press ignored: controller is not sensor-qualified in FixtureHold");
  }
}

void FirmwareApp::drainMotorFeedback(std::uint64_t now_us) noexcept {
  tap_fw::motor_transport::CanFrame can_frame;
  unsigned can_count = 0U;
  while (can_count < 64U && transport_.receiveCan(can_frame)) {
    (void)feedback_.ingestCan(can_frame, now_us);
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
  if (!imu_ready_ || !imu_.read(imu_sample_)) return;
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
  if (feedback_.ready(now_us)) {
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

  auto command = endpoint_.command(now_us);
  if (command.request_enable && !readyForEnable(now_us)) {
    writeCommand(endpoint_, "DISABLE", now_us);
    command = endpoint_.command(now_us);
    if (!reported_enable_block_) {
      Serial.printf("ENABLE refused: IMU=%d motors=%d wheel-directions=%d wheel-scale=%d imu-frame=%d buses=%d\n",
                    imuFresh(now_us) && imu_output_.seeded,
                    feedback_.ready(now_us), tap_fw::board::kWheelDirectionsVerified,
                    tap_fw::board::kWheelSpeedScaleVerified,
                    tap_fw::board::kImuFrameVerified,
                    can_ready_ && uart1_ready_ && uart2_ready_);
      reported_enable_block_ = true;
    }
  } else if (!command.request_enable) {
    reported_enable_block_ = false;
  }
  if (fixture_release_pending_) {
    command.request_fixture_release = true;
    fixture_release_pending_ = false;
  }

  if (control_loop_.last_command().enable_motors && !sensorsReady(now_us)) {
    writeCommand(endpoint_, "DISABLE", now_us);
    command = endpoint_.command(now_us);
    Serial.println("Sensor feedback stale; software disable requested");
  }

  const auto result = control_loop_.step(state_, command, now_us);
  if (result == tap_fw::ControllerActuatorBridge::Result::TransportError &&
      control_loop_.last_command().enable_motors) {
    writeCommand(endpoint_, "DISABLE", now_us);
    Serial.println("Actuator transport failed; software disable requested");
  }
  const int mode = static_cast<int>(control_loop_.controller().mode());
  if (mode != last_mode_) {
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
      tap_fw::board::kWheelDirectionsVerified &&
      tap_fw::board::kWheelSpeedScaleVerified &&
      tap_fw::board::kImuFrameVerified;
}

void FirmwareApp::report(std::uint64_t now_us) noexcept {
  if (now_us - last_report_us_ < kStatusPeriodUs) return;
  last_report_us_ = now_us;
  const auto ageMs = [&](std::uint64_t stamp) {
    return stamp == 0U || now_us < stamp ? -1L :
        static_cast<long>((now_us - stamp) / 1000U);
  };
  Serial.printf(
      "SENS imu_age=%ldms seed=%d can_legs=%d uart_wheels=%d "
      "joint_q=[%.3f %.3f %.3f %.3f %.3f %.3f] pitch=%+.3f\n",
      ageMs(last_imu_us_), imu_output_.seeded,
      feedback_.allLegsFresh(now_us), feedback_.allWheelsFresh(now_us),
      state_.joints.position_rad[0], state_.joints.position_rad[1],
      state_.joints.position_rad[2], state_.joints.position_rad[3],
      state_.joints.position_rad[4], state_.joints.position_rad[5],
      state_.imu.pitch_rad);
}

}  // namespace tap_hw
