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
    // Leg MIT cadence; see board::kLegCommandPeriodUs for the CAN budget.
    std::uint32_t leg_command_period_us = board::kLegCommandPeriodUs;
    std::uint32_t disabled_repeat_period_us = board::kDisabledRepeatPeriodUs;
    // See board::kLegGainScale. Kp *= scale, Kd *= sqrt(scale).
    float leg_gain_scale = board::kLegGainScale;
    // See board::kLegMaxKpNmPerRad; Kd follows the capped Kp.
    float leg_max_kp_nm_per_rad = board::kLegMaxKpNmPerRad;
    // Used when a command carries no positive torque_nm for a leg.
    float leg_torque_limit_nm = 3.0F;
    std::uint32_t wheel_enable_settle_us = 20'000U;
    float task_stiffness_fore_aft_n_per_m = 4618.0F;
    float task_stiffness_vertical_n_per_m = 3000.0F;
    // One MIT frame in flight per GDS68: the driver answers on the SAME CAN
    // id as the command (node << 5 | 0x008). A new command queued while that
    // node's previous answer waits for the bus puts two different frames with
    // one id on the wire; arbitration cannot separate them, both retransmit
    // into each other and the ESP went bus-off within milliseconds (robot
    // 2026-09-26: 0 -> 62 bus errors in one burst, drivers healthy). A leg is
    // re-commanded only after its answer arrived or this timeout passed.
    std::uint32_t mit_reply_timeout_us = 3'000U;
    // Sent with Set_Limits on the enable edge (board::kLegCurrentLimitA).
    float leg_current_limit_a = board::kLegCurrentLimitA;
    float leg_vel_limit_turns_s = board::kLegVelLimitTurnsS;
    // See board::kLegTargetIntegralPerS; 0 disables.
    float leg_target_integral_per_s =
        board::kLegsLockedAtZeroPose ? 0.0F : board::kLegTargetIntegralPerS;
    // > 0: this Kp for every leg joint instead of the model stiffness
    // (two-wheel lock, board::kLegLockKpNmPerRad).
    float leg_fixed_kp_nm_per_rad =
        board::kLegsLockedAtZeroPose ? board::kLegLockKpNmPerRad : 0.0F;
    // > 0: this Kd with the fixed Kp (otherwise critically damped).
    float leg_fixed_kd_nm_s_per_rad =
        board::kLegsLockedAtZeroPose ? board::kLegLockKdNmSPerRad : 0.0F;
    float leg_target_integral_max_rad = board::kLegTargetIntegralMaxRad;
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

  // Every received CAN frame; a GDS68 MIT answer releases that node for its
  // next command (Config::mit_reply_timeout_us).
  void onCanReceived(const motor_transport::CanFrame& frame) noexcept;
  // Live tuning: new leg current limit, sent on the next command cycle.
  void setLegCurrentLimit(float amps) noexcept {
    config_.leg_current_limit_a = amps;
    limits_dirty_ = true;
  }
  [[nodiscard]] float legCurrentLimit() const noexcept { return config_.leg_current_limit_a; }
  // Live tuning of the fixed leg gains (0 = back to the model stiffness).
  void setLegFixedGains(float kp, float kd) noexcept {
    config_.leg_fixed_kp_nm_per_rad = kp;
    config_.leg_fixed_kd_nm_s_per_rad = kd;
  }
  [[nodiscard]] float legFixedKp() const noexcept { return config_.leg_fixed_kp_nm_per_rad; }
  [[nodiscard]] float legFixedKd() const noexcept { return config_.leg_fixed_kd_nm_s_per_rad; }
  [[nodiscard]] const robot_control::JointArray& legTargetIntegral() const noexcept {
    return leg_target_integral_rad_;
  }
  // Legs skipped because their previous MIT answer was still pending.
  [[nodiscard]] std::uint32_t mitDeferred() const noexcept { return mit_deferred_; }

  // Leg zero offsets (core = driver_sign * raw + offset). Null = no offset.
  void setLegOffsets(const robot_control::JointArray* offsets) noexcept {
    leg_offsets_ = offsets;
  }

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
  bool disabled_sent_ = false;
  std::uint64_t last_send_us_ = 0;
  std::uint64_t last_leg_us_ = 0;
  std::uint64_t last_disabled_us_ = 0;
  std::uint64_t wheel_enable_us_ = 0;
  const robot_control::JointArray* leg_offsets_ = nullptr;
  std::array<std::uint64_t, 4> mit_pending_since_us_{};  // 0 = no answer pending
  std::array<bool, 4> mit_pending_{};
  std::uint32_t mit_deferred_ = 0U;
  bool limits_dirty_ = false;
  robot_control::JointArray leg_target_integral_rad_{};
};

}  // namespace tap_fw
