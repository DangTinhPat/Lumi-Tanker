#include "tap_hw/Mpu6500Spi.hpp"

#include <cmath>

#include <Arduino.h>
#include <SPI.h>

#include "tap_fw/BoardConfig.hpp"

namespace tap_hw {
namespace {

constexpr float kGravity = 9.80665F;
constexpr float kDegToRad = 0.017453292519943295F;
// MPU6500: 1 MHz for register writes/configuration, up to 20 MHz for reading
// the sensor registers. The 14-byte burst at 1 MHz cost ~150 us per cycle.
const SPISettings kSpiSettings{1'000'000U, MSBFIRST, SPI_MODE0};
const SPISettings kSpiReadSettings{8'000'000U, MSBFIRST, SPI_MODE0};

}  // namespace

bool Mpu6500Spi::begin() noexcept {
  pinMode(tap_fw::board::kImuCsGpio, OUTPUT);
  digitalWrite(tap_fw::board::kImuCsGpio, HIGH);
  pinMode(tap_fw::board::kImuIntGpio, INPUT);
  SPI.begin(tap_fw::board::kImuSclkGpio,
            tap_fw::board::kImuMisoGpio,
            tap_fw::board::kImuMosiGpio,
            tap_fw::board::kImuCsGpio);

  writeRegister(0x6B, 0x80);  // Reset.
  delay(100);
  writeRegister(0x6A, 0x10);  // Disable the I2C interface.
  writeRegister(0x6B, 0x01);  // PLL with X-gyro reference.
  writeRegister(0x19, 0x01);  // 1 kHz / (1 + 1) = 500 Hz.
  writeRegister(0x1A, 0x03);  // Gyro and accel digital low-pass filters.
  writeRegister(0x1B, 0x08);  // Gyro +/-500 deg/s.
  writeRegister(0x1C, 0x00);  // Accel +/-2 g.
  writeRegister(0x1D, 0x03);  // Accel low-pass filter.
  delay(10);

  who_am_i_ = readRegister(0x75);
  return who_am_i_ == 0x70U || who_am_i_ == 0x71U;
}

bool Mpu6500Spi::read(Sample& sample) const noexcept {
  // One hardware FIFO transaction instead of 15 SPI.transfer calls (each
  // taking the Arduino peripheral lock). Keep the same clock and filters.
  std::uint8_t request[15]{};
  std::uint8_t response[15]{};
  request[0] = 0x3BU | 0x80U;
  SPI.beginTransaction(kSpiReadSettings);
  digitalWrite(tap_fw::board::kImuCsGpio, LOW);
  SPI.transferBytes(request, response, sizeof request);
  digitalWrite(tap_fw::board::kImuCsGpio, HIGH);
  SPI.endTransaction();
  const auto* bytes = response + 1U;  // discard the address-phase byte
  const auto be16 = [&](std::size_t index) {
    return static_cast<std::int16_t>(
        (static_cast<std::uint16_t>(bytes[index]) << 8U) |
        static_cast<std::uint16_t>(bytes[index + 1U]));
  };
  constexpr float kAccelScale = kGravity / 16384.0F;
  constexpr float kGyroScale = kDegToRad / 65.5F;
  sample.accel_m_s2 = {be16(0U) * kAccelScale, be16(2U) * kAccelScale,
                       be16(4U) * kAccelScale};
  sample.gyro_rad_s = {be16(8U) * kGyroScale, be16(10U) * kGyroScale,
                       be16(12U) * kGyroScale};
  return std::isfinite(sample.accel_m_s2[0]) &&
      std::isfinite(sample.accel_m_s2[1]) &&
      std::isfinite(sample.accel_m_s2[2]) &&
      std::isfinite(sample.gyro_rad_s[0]) &&
      std::isfinite(sample.gyro_rad_s[1]) &&
      std::isfinite(sample.gyro_rad_s[2]);
}

void Mpu6500Spi::writeRegister(std::uint8_t address,
                               std::uint8_t value) const noexcept {
  SPI.beginTransaction(kSpiSettings);
  digitalWrite(tap_fw::board::kImuCsGpio, LOW);
  SPI.transfer(address & 0x7FU);
  SPI.transfer(value);
  digitalWrite(tap_fw::board::kImuCsGpio, HIGH);
  SPI.endTransaction();
}

std::uint8_t Mpu6500Spi::readRegister(std::uint8_t address) const noexcept {
  std::uint8_t value = 0U;
  (void)readRegisters(address, &value, 1U);
  return value;
}

bool Mpu6500Spi::readRegisters(std::uint8_t address, std::uint8_t* data,
                               std::size_t length) const noexcept {
  if (data == nullptr || length == 0U) return false;
  SPI.beginTransaction(kSpiSettings);
  digitalWrite(tap_fw::board::kImuCsGpio, LOW);
  SPI.transfer(address | 0x80U);
  for (std::size_t i = 0U; i < length; ++i) data[i] = SPI.transfer(0U);
  digitalWrite(tap_fw::board::kImuCsGpio, HIGH);
  SPI.endTransaction();
  return true;
}

}  // namespace tap_hw
