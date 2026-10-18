#pragma once

#include <algorithm>
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
// v3.5.2 defaults to GDS68 position + passthrough: the driver closes the
// position/velocity loops. No firmware Kt or Iq feedback is needed in this mode.
// The previous current-compensated torque path is an explicit source-only
// configuration for comparison; there is no live mode switch.
class LegLock {
 public:
  enum class Mode { Position, CurrentTorque };
  struct Config {
    Mode mode = Mode::Position;
    float ramp_s = 1.0F;                   // current pose -> 0
    // 333 Hz, paced on the 500 Hz control clock (alternating 2/4 ms). Four
    // commands per cycle at 500 Hz would use ~88 % of the 500 kbit/s bus before
    // heartbeat/diagnostics (worst-case stuffing).
    std::uint32_t resend_period_us = 3'000U;
    // Legacy CurrentTorque gains and Kt estimate; unused by POS.
    float joint_kp_nm_per_rad = board::kLegMaxKpNmPerRad;
    float joint_kd_nm_s_per_rad = 0.5F;
    float gear_ratio = 8.0F;               // rotor turns per joint turn
    float kt_nm_per_a = 0.47F;
    float current_limit_a = board::kLegCurrentLimitA;
    float current_comp_gain = 0.2F;
    float velocity_limit_turns_s = board::kLegVelLimitTurnsS;
  };
  static constexpr std::size_t kLegs = 4U;  // board::MotorIds{}.leg_can order

  LegLock() noexcept = default;
  explicit LegLock(Config config) noexcept : config_(config) {}

  // Starts the lock from the measured rotor positions (driver turns).
  // Stand-up passes ramp_to_zero=false: the caller supplies the measured pose as
  // the offsets and owns the trajectory, so no homing ramp is superimposed.
  bool engage(const std::array<float, kLegs>& start_turns, std::uint64_t now_us,
              motor_transport::Transport& transport, bool ramp_to_zero = true) noexcept;
  // Sends the (ramped) hold target at the resend period. Call every tick.
  bool update(std::uint64_t now_us, motor_transport::Transport& transport) noexcept;
  // All legs to IDLE.
  bool release(motor_transport::Transport& transport) noexcept;

  [[nodiscard]] bool engaged() const noexcept { return engaged_; }
  // Ramp finished: every leg is commanded to exactly 0.
  [[nodiscard]] bool atZero(std::uint64_t now_us) const noexcept;
  [[nodiscard]] float targetTurns(std::size_t leg, std::uint64_t now_us) const noexcept;
  [[nodiscard]] const Config& config() const noexcept { return config_; }
  [[nodiscard]] bool requiresCurrentFeedback() const noexcept {
    return config_.mode == Mode::CurrentTorque;
  }
  [[nodiscard]] const char* modeName() const noexcept {
    return requiresCurrentFeedback() ? "CURRENT_TORQUE" : "POS";
  }
  // Only used by the explicit CurrentTorque path; POS uses driver encoders.
  void setFeedback(const std::array<float, kLegs>& position_rad,
                   const std::array<float, kLegs>& velocity_rad_s,
                   const std::array<float, kLegs>& current_a,
                   const std::array<bool, kLegs>& current_valid) noexcept {
    feedback_position_rad_ = position_rad;
    feedback_velocity_rad_s_ = velocity_rad_s;
    feedback_current_a_ = current_a;
    feedback_valid_ = current_valid;
  }
  // Height / tier-2 offsets: hip/knee (core frame, rad) added to every leg's
  // zero once the engage ramp is done; applied the same to both legs.
  void setOffsets(float hip_core_rad, float knee_core_rad) noexcept {
    hip_offset_rad_ = hip_core_rad;
    knee_offset_rad_ = knee_core_rad;
  }
  // Roll (V3Controller::Config::Roll): per-side offsets [left, right]
  // added to the common ones above.
  // Optional rates [core rad/s] become driver velocity feed-forward in POS
  // or the velocity reference in CurrentTorque impedance.
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
  // Rate of the common hip/knee offsets [core rad/s]; POS sends it as the driver velocity feed-forward so
  // a ramped target (stand-up) is tracked without the pos_gain lag. Zero when the path is held.
  void setCommonRates(float hip_rate, float knee_rate) noexcept {
    common_hip_rate_ = hip_rate;
    common_knee_rate_ = knee_rate;
  }
  [[nodiscard]] float velFfTurns(std::size_t leg) const noexcept;
  void setResendPeriod(std::uint32_t period_us) noexcept {
    config_.resend_period_us = period_us;
    next_send_us_ = last_send_us_ + period_us;
  }
  // The following gains/scales only affect CurrentTorque, never driver POS gains.
  void setJointGains(float kp, float kd) noexcept {
    config_.joint_kp_nm_per_rad = kp;
    config_.joint_kd_nm_s_per_rad = kd;
  }
  // Joint impedance scale per joint type (v3.5 jump thrust / soft landing);
  // 1 = the configured gains. Leg order is MotorIds::leg_can: hips even, knees odd.
  void setImpedanceScale(float hip_kp, float hip_kd, float knee_kp, float knee_kd) noexcept {
    kp_scale_ = {hip_kp, knee_kp};
    kd_scale_ = {hip_kd, knee_kd};
  }
  void setCurrentCompGain(float gain) noexcept {
    config_.current_comp_gain = std::clamp(gain, 0.0F, 1.0F);
  }

 private:
  float hip_offset_rad_ = 0.0F;
  float knee_offset_rad_ = 0.0F;
  std::array<float, 2> side_hip_rad_{};
  std::array<float, 2> side_knee_rad_{};
  float common_hip_rate_ = 0.0F, common_knee_rate_ = 0.0F;
  std::array<float, 2> side_hip_rate_{};
  std::array<float, 2> side_knee_rate_{};
  std::array<float, kLegs> feedback_position_rad_{};
  std::array<float, kLegs> feedback_velocity_rad_s_{};
  std::array<float, kLegs> feedback_current_a_{};
  std::array<bool, kLegs> feedback_valid_{};
  std::array<float, 2> kp_scale_{1.0F, 1.0F};
  std::array<float, 2> kd_scale_{1.0F, 1.0F};
  bool sendTargets(std::uint64_t now_us, motor_transport::Transport& transport) noexcept;

  Config config_{};
  bool engaged_ = false;
  bool ramp_to_zero_ = true;
  std::uint64_t engage_us_ = 0U;
  std::uint64_t last_send_us_ = 0U;
  std::uint64_t next_send_us_ = 0U;
  std::array<float, kLegs> start_turns_{};
};

}  // namespace tap_fw::v3
