#pragma once

#include <array>
#include <cstdint>

#include "robot_control/Types.hpp"
#include "robot_hal/MotorTransport.hpp"
#include "tap_fw/BoardConfig.hpp"

namespace tap_fw {

namespace motor_frame = robot_control::motor_frame;
namespace motor_transport = robot_control::motor_transport;

class MotorFeedback {
 public:
  bool poll(motor_transport::Transport& transport, std::uint64_t now_us,
            bool legs_enabled) noexcept;
  bool ingestCan(const motor_transport::CanFrame& frame,
                 std::uint64_t now_us) noexcept;
  bool ingestUart(std::uint8_t bus, const motor_frame::DxlFrame& frame,
                  std::uint64_t now_us) noexcept;

  void writeTo(robot_control::RobotState& state) const noexcept;
  [[nodiscard]] bool ready(std::uint64_t now_us) const noexcept;
  [[nodiscard]] bool allLegsFresh(std::uint64_t now_us) const noexcept;
  [[nodiscard]] bool allWheelsFresh(std::uint64_t now_us) const noexcept;
  [[nodiscard]] std::uint64_t sequence() const noexcept { return sequence_; }

 private:
  [[nodiscard]] bool fresh(std::size_t joint,
                           std::uint64_t now_us) const noexcept;

  robot_control::JointArray position_rad_{};
  robot_control::JointArray velocity_rad_s_{};
  robot_control::JointArray effort_nm_{};
  std::array<std::uint64_t, robot_control::kJointCount> stamp_us_{};
  std::array<bool, robot_control::kJointCount> valid_{};
  std::array<std::uint16_t, 2> wheel_previous_raw_{};
  std::array<float, 2> wheel_unwrapped_rad_{};
  std::array<bool, 2> wheel_position_seeded_{};
  std::uint64_t last_can_probe_us_ = 0U;
  std::uint64_t last_wheel_poll_us_ = 0U;
  std::uint64_t sequence_ = 0U;
  bool can_probe_initialized_ = false;
  bool wheel_poll_initialized_ = false;
};

}  // namespace tap_fw
