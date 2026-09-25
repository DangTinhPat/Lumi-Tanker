#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE

#include <DabbleESP32.h>

#include <BLEDevice.h>

#include <Arduino.h>
#include <SPI.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <string>

#include "robot_control/ImuStateEstimator.hpp"
#include "robot_control/EmbeddedController.hpp"
#include "tap_fw/BoardConfig.hpp"
#include "tap_fw/DabbleGamepadMapper.hpp"

namespace {

constexpr float kGravity = 9.80665F;
constexpr float kDegToRad = 0.017453292519943295F;
constexpr std::uint32_t kSamplePeriodUs = 5'000U;  // 200 Hz

class Mpu6500 {
 public:
  struct Sample {
    std::array<float, 3> accel_m_s2{};
    std::array<float, 3> gyro_rad_s{};
  };

  bool begin() {
    pinMode(tap_fw::board::kImuCsGpio, OUTPUT);
    digitalWrite(tap_fw::board::kImuCsGpio, HIGH);
    pinMode(tap_fw::board::kImuIntGpio, INPUT);
    SPI.begin(tap_fw::board::kImuSclkGpio,
              tap_fw::board::kImuMisoGpio,
              tap_fw::board::kImuMosiGpio,
              tap_fw::board::kImuCsGpio);

    writeRegister(0x6B, 0x80);  // PWR_MGMT_1: reset
    delay(100);
    writeRegister(0x6A, 0x10);  // USER_CTRL: I2C_IF_DIS, SPI-only mode
    writeRegister(0x6B, 0x01);  // PLL with X gyro reference
    writeRegister(0x19, 0x04);  // 1 kHz / (1 + 4) = 200 Hz
    writeRegister(0x1A, 0x03);  // gyro/accel digital low-pass filter
    writeRegister(0x1B, 0x08);  // gyro +/-500 deg/s
    writeRegister(0x1C, 0x00);  // accel +/-2 g
    writeRegister(0x1D, 0x03);  // accel low-pass filter
    delay(10);

    who_am_i_ = readRegister(0x75);
    return who_am_i_ == 0x70 || who_am_i_ == 0x71;
  }

  bool read(Sample& sample) const {
    std::uint8_t bytes[14]{};
    if (!readRegisters(0x3B, bytes, sizeof(bytes))) return false;

    // 'word' is an Arduino.h macro (makeWord), so it cannot name this lambda.
    const auto be16 = [&](std::size_t index) {
      return static_cast<std::int16_t>(
          (static_cast<std::uint16_t>(bytes[index]) << 8U) |
          static_cast<std::uint16_t>(bytes[index + 1U]));
    };
    constexpr float kAccelScale = kGravity / 16384.0F;
    constexpr float kGyroScale = kDegToRad / 65.5F;
    sample.accel_m_s2 = {be16(0) * kAccelScale, be16(2) * kAccelScale,
                         be16(4) * kAccelScale};
    sample.gyro_rad_s = {be16(8) * kGyroScale, be16(10) * kGyroScale,
                         be16(12) * kGyroScale};
    return true;
  }

  [[nodiscard]] std::uint8_t whoAmI() const { return who_am_i_; }

  std::uint8_t reg(std::uint8_t address) const { return readRegister(address); }

 private:
  static inline const SPISettings kSpiSettings{1'000'000, MSBFIRST, SPI_MODE0};

  void writeRegister(std::uint8_t address, std::uint8_t value) const {
    SPI.beginTransaction(kSpiSettings);
    digitalWrite(tap_fw::board::kImuCsGpio, LOW);
    SPI.transfer(address & 0x7FU);
    SPI.transfer(value);
    digitalWrite(tap_fw::board::kImuCsGpio, HIGH);
    SPI.endTransaction();
  }

  std::uint8_t readRegister(std::uint8_t address) const {
    std::uint8_t value = 0U;
    readRegisters(address, &value, 1U);
    return value;
  }

  bool readRegisters(std::uint8_t address, std::uint8_t* data,
                     std::size_t length) const {
    if (data == nullptr || length == 0U) return false;
    SPI.beginTransaction(kSpiSettings);
    digitalWrite(tap_fw::board::kImuCsGpio, LOW);
    SPI.transfer(address | 0x80U);
    for (std::size_t i = 0U; i < length; ++i) data[i] = SPI.transfer(0U);
    digitalWrite(tap_fw::board::kImuCsGpio, HIGH);
    SPI.endTransaction();
    return true;
  }

  std::uint8_t who_am_i_ = 0U;
};

Mpu6500 imu;
robot_control::ImuStateEstimator estimator;
robot_control::EmbeddedController controller;
tap_fw::ble::CommandEndpoint ble_endpoint;
tap_fw::ble::DabbleGamepadMapper gamepad(ble_endpoint);
std::uint32_t next_imu_us = 0U;
std::uint32_t last_imu_us = 0U;
std::uint32_t last_report_us = 0U;
std::uint64_t imu_sequence = 0U;
bool imu_ok = false;

// The DabbleESP32 library never clears Dabble.isAppConnected() and never
// restarts advertising after a disconnect, so the real link state is tracked
// from the GATT server events instead (see onGattsEvent below).
std::atomic<int> ble_links{0};
std::atomic<bool> ble_needs_advertising{false};
bool ble_link_up = false;
// After a (re)connect the library may still hold the previous session's
// button state; inputs are ignored until every button has been seen released.
bool ble_inputs_armed = false;
std::uint32_t ble_advertise_at_ms = 0U;

void onGattsEvent(esp_gatts_cb_event_t event, esp_gatt_if_t,
                  esp_ble_gatts_cb_param_t*) {
  if (event == ESP_GATTS_CONNECT_EVT) {
    ble_links.fetch_add(1);
  } else if (event == ESP_GATTS_DISCONNECT_EVT) {
    int links = ble_links.load();
    while (links > 0 && !ble_links.compare_exchange_weak(links, links - 1)) {}
    ble_needs_advertising.store(true);
  }
}

void printImu(const Mpu6500::Sample& sample,
              const robot_control::ImuStateEstimator::Output& output,
              const robot_control::MotorCommand& controller_output) {
  Serial.printf(
      "IMU ax=%+.3f ay=%+.3f az=%+.3f gx=%+.3f gy=%+.3f gz=%+.3f "
      "seed=%d pitch=%+.4f rate=%+.3f accel_valid=%d trust=%.2f\n",
      sample.accel_m_s2[0], sample.accel_m_s2[1], sample.accel_m_s2[2],
      sample.gyro_rad_s[0], sample.gyro_rad_s[1], sample.gyro_rad_s[2],
      output.seeded, output.pitch_rad, output.pitch_rate_rad_s,
      output.accel_tilt_valid, output.accel_trust);
  Serial.printf("CTRL mode=%s enable=%d pitch=%.4f rate=%.3f\n",
                robot_control::toString(controller.mode()),
                controller_output.enable_motors, controller_output.position_rad[0],
                controller_output.velocity_rad_s[2]);
}

void pollBle(std::uint32_t now_us) {
  Dabble.processInput();

  const bool link_up = ble_links.load() > 0;
  if (link_up != ble_link_up) {
    ble_link_up = link_up;
    ble_inputs_armed = false;
    digitalWrite(tap_fw::board::kLed3Gpio, link_up ? HIGH : LOW);
    Serial.printf("BLE %s\n", link_up ? "connected" : "disconnected");
    if (!link_up) ble_endpoint.onDisconnect();
  }
  // Central advertising restart: a phone disconnect stops advertising and the
  // library does not resume it. Wait 300 ms so the stack can settle first.
  const std::uint32_t now_ms = millis();
  if (ble_needs_advertising.exchange(false)) ble_advertise_at_ms = now_ms + 300U;
  if (ble_advertise_at_ms != 0U &&
      static_cast<std::int32_t>(now_ms - ble_advertise_at_ms) >= 0) {
    ble_advertise_at_ms = 0U;
    BLEDevice::startAdvertising();
    Serial.println("BLE advertising restarted");
  }

  tap_fw::ble::GamepadState input{};
  if (link_up) {
    input = tap_fw::ble::GamepadState{
        GamePad.isUpPressed(),       GamePad.isDownPressed(),
        GamePad.isLeftPressed(),     GamePad.isRightPressed(),
        GamePad.isStartPressed(),    GamePad.isSelectPressed(),
        GamePad.isTrianglePressed(), GamePad.isCirclePressed(),
        GamePad.isCrossPressed(),    GamePad.isSquarePressed()};
    const bool any_pressed = input.up || input.down || input.left ||
        input.right || input.start || input.select || input.triangle ||
        input.circle || input.cross || input.square;
    if (!ble_inputs_armed) {
      if (!any_pressed) ble_inputs_armed = true;
      input = tap_fw::ble::GamepadState{};
    }
  }
  static tap_fw::ble::GamepadState previous_input{};
  static bool input_initialized = false;
  const bool input_changed = !input_initialized ||
      input.up != previous_input.up || input.down != previous_input.down ||
      input.left != previous_input.left || input.right != previous_input.right ||
      input.start != previous_input.start || input.select != previous_input.select ||
      input.triangle != previous_input.triangle || input.circle != previous_input.circle ||
      input.cross != previous_input.cross || input.square != previous_input.square;
  if (input_changed) {
    Serial.printf(
        "GAMEPAD up=%d down=%d left=%d right=%d start=%d select=%d "
        "triangle=%d circle=%d cross=%d square=%d\n",
        input.up, input.down, input.left, input.right, input.start,
        input.select, input.triangle, input.circle, input.cross, input.square);
    previous_input = input;
    input_initialized = true;
  }
  gamepad.update(input, now_us);

  static bool previous_enable = false;
  const auto command = ble_endpoint.command(now_us);
  if (command.request_enable != previous_enable) {
    previous_enable = command.request_enable;
    Serial.printf("BLE command: %s\n",
                  previous_enable ? "ENABLE" : "DISABLE/timeout");
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(tap_fw::board::kLed1Gpio, OUTPUT);
  pinMode(tap_fw::board::kLed2Gpio, OUTPUT);
  pinMode(tap_fw::board::kLed3Gpio, OUTPUT);
  digitalWrite(tap_fw::board::kLed1Gpio, LOW);
  digitalWrite(tap_fw::board::kLed2Gpio, LOW);
  digitalWrite(tap_fw::board::kLed3Gpio, LOW);

  Serial.println("TAP bring-up: MPU6500 SPI + estimator + Dabble BLE");
  Serial.printf("SPI CS=%d MOSI=%d SCLK=%d MISO=%d INT=%d\n",
                tap_fw::board::kImuCsGpio, tap_fw::board::kImuMosiGpio,
                tap_fw::board::kImuSclkGpio, tap_fw::board::kImuMisoGpio,
                tap_fw::board::kImuIntGpio);

  imu_ok = imu.begin();
  digitalWrite(tap_fw::board::kLed1Gpio, imu_ok ? HIGH : LOW);
  Serial.printf("MPU6500 WHO_AM_I=0x%02X (%s)\n", imu.whoAmI(),
                imu_ok ? "PASS" : "FAIL");

  Serial.printf("REG USER_CTRL=%02X PWR1=%02X PWR2=%02X CONFIG=%02X GYRO_CFG=%02X ACCEL_CFG=%02X ACCEL_CFG2=%02X SMPLRT=%02X\n",
                imu.reg(0x6A), imu.reg(0x6B), imu.reg(0x6C), imu.reg(0x1A),
                imu.reg(0x1B), imu.reg(0x1C), imu.reg(0x1D), imu.reg(0x19));
  {
    std::uint8_t raw[14]{};
    for (unsigned i = 0; i < 14; ++i) raw[i] = imu.reg(0x3B + i);
    Serial.printf("RAW:");
    for (unsigned i = 0; i < 14; ++i) Serial.printf(" %02X", raw[i]);
    Serial.println();
  }

  Dabble.begin(std::string("TAP-RB"));
  BLEDevice::setCustomGattsHandler(onGattsEvent);

  Serial.println("BLE advertising as TAP-RB; open Dabble GamePad");
  next_imu_us = micros();
}

void loop() {
  const std::uint32_t now_us = micros();
  pollBle(now_us);

  if (imu_ok && static_cast<std::int32_t>(now_us - next_imu_us) >= 0) {
    next_imu_us += kSamplePeriodUs;
    Mpu6500::Sample sample;
    if (imu.read(sample)) {
      const float dt_s = last_imu_us == 0U
          ? 1.0F / 200.0F
          : static_cast<float>(now_us - last_imu_us) * 1.0e-6F;
      last_imu_us = now_us;
      const float magnitude = std::sqrt(
          sample.accel_m_s2[0] * sample.accel_m_s2[0] +
          sample.accel_m_s2[1] * sample.accel_m_s2[1] +
          sample.accel_m_s2[2] * sample.accel_m_s2[2]);
      const bool quiet = std::abs(magnitude - kGravity) < 0.35F &&
          std::abs(sample.gyro_rad_s[1]) < 0.12F;
      const auto output = estimator.update(
          sample.accel_m_s2, sample.gyro_rad_s, dt_s,
          static_cast<float>(now_us) * 1.0e-6F, 0.0F, false, quiet, {});
      robot_control::RobotState controller_state{};
      controller_state.dt_s = dt_s;
      controller_state.imu_sample_sequence = ++imu_sequence;
      controller_state.floor_contact = true;
      controller_state.left_floor_contact = true;
      controller_state.right_floor_contact = true;
      controller_state.imu.pitch_rad = output.pitch_rad;
      controller_state.imu.pitch_rate_rad_s = output.pitch_rate_rad_s;
      controller_state.imu.pitch_rate_bias_corrected_rad_s =
          output.pitch_rate_rad_s;
      controller_state.imu.pitch_rate_bias_corrected_valid = output.seeded;
      controller_state.imu.gyro_bias_rad_s = output.gyro_bias_rad_s;
      controller_state.imu.accel_magnitude_m_s2 = output.accel_magnitude_m_s2;
      controller_state.imu.hard_impact_event = output.hard_impact_event;
      const auto controller_command = ble_endpoint.command(now_us);
      const auto controller_output =
          controller.update(controller_state, controller_command);
      digitalWrite(tap_fw::board::kLed2Gpio, output.seeded ? HIGH : LOW);
      static std::uint32_t last_report_us = 0U;
      if (static_cast<std::uint32_t>(now_us - last_report_us) >= 200'000U) {
        last_report_us = now_us;
        printImu(sample, output, controller_output);
      }
    } else {
      Serial.println("IMU burst read failed");
      digitalWrite(tap_fw::board::kLed1Gpio, LOW);
    }
  }
  delay(1);
}
