#pragma once

#include <array>
#include <cstdint>

#include "robot_control/GeneratedGeometry.hpp"
#include "robot_control/Types.hpp"
#include "robot_control/WholeBodyModel.hpp"
#include "robot_hal/MotorTransport.hpp"
#include "tap_fw/BoardConfig.hpp"

namespace tap_fw {

namespace motor_frame = robot_control::motor_frame;
namespace motor_transport = robot_control::motor_transport;

class ControllerActuatorBridge {
 public:
  struct Config {
    board::MotorIds ids{};
    // 500 Hz is the hardware target from MOTOR_HAL_PLAN. The controller may
    // still be called faster; only this bridge is allowed to put frames on
    // the physical bus.
    std::uint32_t command_period_us = 2'000U;
    std::uint32_t wheel_enable_settle_us = 20'000U;
    float task_stiffness_fore_aft_n_per_m = 4618.0F;
    float task_stiffness_vertical_n_per_m = 3000.0F;
  };

  enum class Result {
    Sent,
    RateLimited,
    TransportError,
  };

  explicit ControllerActuatorBridge(motor_transport::Transport& transport) noexcept
      : transport_(transport) {}

  ControllerActuatorBridge(motor_transport::Transport& transport,
                           Config config) noexcept
      : transport_(transport), config_(config) {}

  [[nodiscard]] Result apply(const robot_control::MotorCommand& command,
                             const robot_control::RobotState& state,
                             std::uint64_t now_us) noexcept;

  [[nodiscard]] bool enabled() const noexcept { return enabled_; }

 private:
  struct LegStiffness {
    float hip_nm_per_rad = 0.0F;
    float knee_nm_per_rad = 0.0F;
    float cross_nm_per_rad = 0.0F;
  };

  static bool isHip(std::size_t joint) noexcept;
  [[nodiscard]] LegStiffness legStiffness(float hip_rad, float knee_rad) const noexcept;
  [[nodiscard]] bool sendDisabled() noexcept;
  [[nodiscard]] bool sendWheelStop(std::size_t wheel) noexcept;
  [[nodiscard]] bool sendWheelCommand(std::size_t wheel, float velocity_rad_s) noexcept;
  [[nodiscard]] bool due(std::uint64_t now_us) const noexcept;

  motor_transport::Transport& transport_;
  Config config_{};
  bool enabled_ = false;
  bool has_sent_ = false;
  std::uint64_t last_send_us_ = 0;
  std::uint64_t wheel_enable_us_ = 0;
};

}  // namespace tap_fw
