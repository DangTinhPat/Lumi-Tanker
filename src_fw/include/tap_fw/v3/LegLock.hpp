#pragma once

#include <array>
#include <cstdint>

#include "robot_hal/MotorTransport.hpp"
#include "tap_fw/BoardConfig.hpp"

namespace tap_fw::v3 {

namespace motor_transport = robot_control::motor_transport;

// v3 leg driver: every GDS68 holds its own power-up position (encoder 0) plus
// the offsets the controller asks for. The operator arranges the legs before the
// motor supply is switched on, so "zero" is exactly that pose.
//
// Position mode (default, all four joints): the driver's internal
// position/velocity/integrator loop tracks Set_Input_Pos. At engage the target
// ramps smoothly from wherever the leg is to the zero pose; after that it moves
// by the height / tier-2 / roll offsets (setOffsets, setSideOffsets).
// MIT mode is a fallback: the same target with a fixed Kp/Kd spring (LEGMODE MIT).
class LegLock {
 public:
  enum class Mode { Position, Mit };

  struct Config {
    Mode mode = Mode::Position;
    float ramp_s = 1.0F;                   // current pose -> 0
    // 333 Hz, paced on the 500 Hz control clock (alternating 2/4 ms). Four
    // commands per cycle at 500 Hz would use ~88 % of the 500 kbit/s bus before
    // heartbeat/diagnostics (worst-case stuffing).
    std::uint32_t resend_period_us = 3'000U;
    float mit_kp_nm_per_rad = 15.0F;
    float mit_kd_nm_s_per_rad = 0.5F;
    float gear_ratio = 8.0F;               // rotor turns per joint turn
  };
  static constexpr std::size_t kLegs = 4U;  // board::MotorIds{}.leg_can order

  LegLock() noexcept = default;
  explicit LegLock(Config config) noexcept : config_(config) {}

  // Starts the lock from the measured rotor positions (driver turns).
  bool engage(const std::array<float, kLegs>& start_turns, std::uint64_t now_us,
              motor_transport::Transport& transport) noexcept;
  // Sends the (ramped) hold target at the resend period. Call every tick.
  bool update(std::uint64_t now_us, motor_transport::Transport& transport) noexcept;
  // All legs to IDLE.
  bool release(motor_transport::Transport& transport) noexcept;

  [[nodiscard]] bool engaged() const noexcept { return engaged_; }
  // Ramp finished: every leg is commanded to exactly 0.
  [[nodiscard]] bool atZero(std::uint64_t now_us) const noexcept;
  [[nodiscard]] float targetTurns(std::size_t leg, std::uint64_t now_us) const noexcept;
  [[nodiscard]] const Config& config() const noexcept { return config_; }
  void setMode(Mode mode) noexcept { config_.mode = mode; }
  // Height / tier-2 offsets: hip/knee (core frame, rad) added to every leg's
  // zero once the engage ramp is done; applied the same to both legs.
  void setOffsets(float hip_core_rad, float knee_core_rad) noexcept {
    hip_offset_rad_ = hip_core_rad;
    knee_offset_rad_ = knee_core_rad;
  }
  // Roll (V3Controller::Config::Roll): per-side offsets [left, right]
  // added to the common ones above.
  // Optional rates [core rad/s] go out as the position target's velocity
  // feed-forward (POS mode, Set_Input_Pos vel_ff).
  void setSideOffsets(const std::array<float, 2>& hip_core_rad,
                      const std::array<float, 2>& knee_core_rad,
                      const std::array<float, 2>& hip_rate = {},
                      const std::array<float, 2>& knee_rate = {}) noexcept {
    side_hip_rad_ = hip_core_rad;
    side_knee_rad_ = knee_core_rad;
    side_hip_rate_ = hip_rate;
    side_knee_rate_ = knee_rate;
  }
  // Velocity feed-forward this leg's target carries [driver turns/s].
  [[nodiscard]] float velFfTurns(std::size_t leg) const noexcept;
  void setResendPeriod(std::uint32_t period_us) noexcept {
    config_.resend_period_us = period_us;
    next_send_us_ = last_send_us_ + period_us;
  }
  void setMitGains(float kp, float kd) noexcept {
    config_.mit_kp_nm_per_rad = kp;
    config_.mit_kd_nm_s_per_rad = kd;
  }

 private:
  float hip_offset_rad_ = 0.0F;
  float knee_offset_rad_ = 0.0F;
  std::array<float, 2> side_hip_rad_{};
  std::array<float, 2> side_knee_rad_{};
  std::array<float, 2> side_hip_rate_{};
  std::array<float, 2> side_knee_rate_{};
  [[nodiscard]] bool jointMit(std::size_t leg) const noexcept;
  bool sendTargets(std::uint64_t now_us, motor_transport::Transport& transport) noexcept;

  Config config_{};
  bool engaged_ = false;
  std::uint64_t engage_us_ = 0U;
  std::uint64_t last_send_us_ = 0U;
  std::uint64_t next_send_us_ = 0U;
  std::array<float, kLegs> start_turns_{};
};

[[nodiscard]] const char* toString(LegLock::Mode mode) noexcept;

}  // namespace tap_fw::v3
