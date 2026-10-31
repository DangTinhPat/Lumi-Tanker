#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace gateway {

class Mpu6500Spi {
 public:
  struct Sample {
    std::array<float, 3> accel_m_s2{};
    std::array<float, 3> gyro_rad_s{};
  };

  bool begin() noexcept;
  bool read(Sample& sample) const noexcept;
  // Hybrid adapter: only consume a fresh sensor conversion. A floating SPI
  // bus reads 0xff, which must not count as data-ready.
  bool readFresh(Sample& sample) const noexcept;
  [[nodiscard]] std::uint8_t whoAmI() const noexcept { return who_am_i_; }
  // Accelerometer full scale +-2/4/8/16 g (boot: 2). Changes the scale of every later sample: call it only while the
  // controller is not using the IMU (Idle / Fault).
  bool setAccelRangeG(int g) noexcept;
  [[nodiscard]] int accelRangeG() const noexcept { return accel_range_g_; }

 private:
  void writeRegister(std::uint8_t address, std::uint8_t value) const noexcept;
  std::uint8_t readRegister(std::uint8_t address) const noexcept;
  bool readRegisters(std::uint8_t address, std::uint8_t* data,
                     std::size_t length) const noexcept;

  std::uint8_t who_am_i_ = 0U;
  int accel_range_g_ = 2;
  float accel_scale_ = 9.80665F / 16384.0F;
};

}  // namespace gateway
