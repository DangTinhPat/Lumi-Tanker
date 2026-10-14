#pragma once
#include <algorithm>
#include <cmath>
#include <array>
#include "tap_fw/v3/BalanceCaptureGovernor.hpp"

namespace tap_fw::v3 {

// Supervises only posture transfer; wheel LQR and all safety checks remain
// in V3Controller. Angles are core hip/virtual-pitch coordinates, not CAD pose
// truth. Conditional integration in V3Controller prevents gimbal windup while
// this rate-limited actuator target is not yet reachable.
class PostStandUpBalance {
 public:
  enum class Phase { Inactive, CaptureHold, PoseTransfer, Normal };
  struct Input {
    float dt_s = 0.002F;
    float desired_hip_rad = 0.0F;
    float virtual_error_rad = 0.0F;
    float virtual_rate_rad_s = 0.0F;
    float body_rate_rad_s = 0.0F;
    float settle_s = 2.0F;
    float grace_s = 4.0F;
    float max_hip_rate_rad_s = 0.12F;
    float body_pitch_rad = 0.0F;
    bool governor_enabled = false;
    bool speed_qualified = false;
    float ground_speed_m_s = 0.0F;
    float command_speed_m_s = 0.0F;
    float quiet_speed_m_s = 0.10F;
    bool held = false;
    bool smooth_transfer = false; // bounded acceleration across quiet-gate pauses
    bool hold_normal_pose = false;
  };
  struct Output { float hip_rad; bool limited; Phase phase; bool failed = false; };
  void reset() noexcept { governor_.reset(); phase_ = Phase::Inactive; elapsed_s_ = quiet_s_ = hip_rate_ = 0.0F; }
  void begin(float hip_rad, float level_hip_rad) noexcept {
    hip_rad_ = hip_rad;
    level_hip_rad_ = level_hip_rad;
    elapsed_s_ = quiet_s_ = hip_rate_ = 0.0F;
    governor_.begin(hip_rad, BalanceCaptureGovernor::Config{});
    phase_ = Phase::CaptureHold;
  }
  Output update(const Input& in) noexcept {
    if (phase_ == Phase::Inactive) return {in.desired_hip_rad, false, phase_};
    if (in.governor_enabled) {
      BalanceCaptureGovernor::Input gi{};
      gi.dt_s=in.dt_s;gi.body_pitch_rad=in.body_pitch_rad;gi.body_rate_rad_s=in.body_rate_rad_s;
      gi.virtual_error_rad=in.virtual_error_rad;gi.virtual_rate_rad_s=in.virtual_rate_rad_s;
      gi.ground_speed_m_s=in.ground_speed_m_s;gi.command_speed_m_s=in.command_speed_m_s;gi.held=in.held;
      const auto go=governor_.update(gi);
      using GP=BalanceCaptureGovernor::Phase;
      phase_=go.phase==GP::Normal ? Phase::Normal :
          (go.phase==GP::Arrest || go.phase==GP::Verify) ? Phase::CaptureHold : Phase::PoseTransfer;
      hip_rad_=go.hip_rad;
      return {go.hip_rad,go.limited,phase_,go.phase==GP::Failed};
    }
    const float dt = std::max(in.dt_s, 0.0F);
    if (phase_ == Phase::Normal) {
      if (in.hold_normal_pose) return {hip_rad_, false, phase_};
      // Normal wheel/hip control owns the target, with the same 0.35 rad/s
      // bound as minbal. Keep supervising the delivered actuator command.
      const float step = std::clamp(in.max_hip_rate_rad_s, 0.0F, 0.35F) * dt;
      hip_rad_ += std::clamp(in.desired_hip_rad-hip_rad_, -step, step);
      return {hip_rad_, std::abs(in.desired_hip_rad-hip_rad_) > 1e-5F, phase_};
    }
    elapsed_s_ += dt;
    // A repeatedly disturbed catch must not shake forever in a paused
    // transfer (robot log: 55 s without reaching Normal). Caller stops wheels
    // and holds the last leg POS target through its existing Fault path.
    if (in.smooth_transfer && elapsed_s_ > 12.0F) {
      hip_rate_ = 0.0F;
      return {hip_rad_, true, phase_, true};
    }
    const bool speed_quiet = !in.speed_qualified ||
        (std::abs(in.ground_speed_m_s) < in.quiet_speed_m_s &&
         std::abs(in.command_speed_m_s) < in.quiet_speed_m_s);
    const float dwell = 0.15F;
    const bool quiet = speed_quiet && !in.held && elapsed_s_ >= std::max(in.settle_s, 0.0F) &&
        std::abs(in.virtual_error_rad) < 0.04F &&
        std::abs(in.virtual_rate_rad_s) < 0.40F && std::abs(in.body_rate_rad_s) < 0.35F;
    quiet_s_ = quiet ? std::min(quiet_s_ + dt, dwell) : 0.0F;
    const bool may_move = quiet_s_ >= dwell;
    if (may_move) phase_ = Phase::PoseTransfer;
    const float vmax = std::clamp(in.max_hip_rate_rad_s, 0.0F, 0.12F);
    if (in.smooth_transfer && phase_ == Phase::PoseTransfer && dt > 0.0F) {
      // Position servo: ramp transfer speed instead of toggling 0/0.12 at
      // every quiet-gate edge. Braking from full speed travels at most
      // vmax^2/(2*0.6) = 0.012 rad. Held or a large disturbance still stops
      // immediately; this is not a relaxation of the wheel/fall guards.
      const bool stop = !speed_quiet || in.held || std::abs(in.virtual_error_rad) >= 0.08F ||
          std::abs(in.virtual_rate_rad_s) >= 0.8F || std::abs(in.body_rate_rad_s) >= 0.8F;
      const float remaining = level_hip_rad_ - hip_rad_;
      const float brake_speed = std::sqrt(2.0F * 0.6F * std::abs(remaining));
      const float want_rate = may_move && !stop
          ? std::copysign(std::min(vmax, brake_speed), remaining) : 0.0F;
      if (stop) hip_rate_ = 0.0F;
      else hip_rate_ += std::clamp(want_rate-hip_rate_, -0.6F*dt, 0.6F*dt);
      hip_rad_ += std::copysign(std::min(std::abs(hip_rate_*dt), std::abs(remaining)), remaining);
      if (std::abs(level_hip_rad_-hip_rad_) <= 1e-5F) { hip_rad_=level_hip_rad_; hip_rate_=0.0F; }
    } else if (may_move) {
      const float step = vmax * dt;
      hip_rad_ += std::clamp(level_hip_rad_ - hip_rad_, -step, step);
    }
    const bool limited = std::abs(level_hip_rad_ - hip_rad_) > 1e-5F;
    // No timed jump to Normal. If motion disturbs balance, pause, requalify,
    // and complete only after reaching the latched level target while quiet.
    if (phase_ == Phase::PoseTransfer && may_move && !limited &&
        elapsed_s_ >= std::max(in.grace_s, 0.0F)) phase_ = Phase::Normal;
    return {hip_rad_, limited || !may_move, phase_};
  }
  [[nodiscard]] Phase phase() const noexcept { return phase_; }
  [[nodiscard]] BalanceCaptureGovernor::Phase governorPhase() const noexcept { return governor_.phase(); }
  [[nodiscard]] BalanceCaptureGovernor::Failure governorFailure() const noexcept { return governor_.failure(); }
  [[nodiscard]] float governorRateRms() const noexcept { return governor_.bodyRateRms(); }
  [[nodiscard]] unsigned governorBlockReason() const noexcept { return governor_.blockReason(); }
 private:
  BalanceCaptureGovernor governor_{};
  Phase phase_ = Phase::Inactive;
  float hip_rad_ = 0.0F, level_hip_rad_ = 0.0F, elapsed_s_ = 0.0F, quiet_s_ = 0.0F, hip_rate_ = 0.0F;
};

}  // namespace tap_fw::v3
