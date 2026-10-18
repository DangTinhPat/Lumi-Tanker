#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace tap_hw {

class Mpu6500Spi {
 public:
  struct Sample {
    std::array<float, 3> accel_m_s2{};
    std::array<float, 3> gyro_rad_s{};
  };

  bool begin() noexcept;
  bool read(Sample& sample) const noexcept;
  [[nodiscard]] std::uint8_t whoAmI() const noexcept { return who_am_i_; }

 private:
  void writeRegister(std::uint8_t address, std::uint8_t value) const noexcept;
  std::uint8_t readRegister(std::uint8_t address) const noexcept;
  bool readRegisters(std::uint8_t address, std::uint8_t* data,
                     std::size_t length) const noexcept;

  std::uint8_t who_am_i_ = 0U;
};

}  // namespace tap_hw
