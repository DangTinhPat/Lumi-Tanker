#pragma once
#include <algorithm>
#include <cmath>

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
    bool held = false;
  };
  struct Output { float hip_rad; bool limited; Phase phase; };
  void reset() noexcept { phase_ = Phase::Inactive; elapsed_s_ = quiet_s_ = 0.0F; }
  void begin(float hip_rad, float level_hip_rad) noexcept {
    hip_rad_ = hip_rad;
    level_hip_rad_ = level_hip_rad;
    elapsed_s_ = quiet_s_ = 0.0F;
    phase_ = Phase::CaptureHold;
  }
  Output update(const Input& in) noexcept {
    if (phase_ == Phase::Inactive) return {in.desired_hip_rad, false, phase_};
    const float dt = std::max(in.dt_s, 0.0F);
    if (phase_ == Phase::Normal) {
      // Normal wheel/hip control owns the target, with the same 0.35 rad/s
      // bound as minbal. Keep supervising the delivered actuator command.
      const float step = std::clamp(in.max_hip_rate_rad_s, 0.0F, 0.35F) * dt;
      hip_rad_ += std::clamp(in.desired_hip_rad-hip_rad_, -step, step);
      return {hip_rad_, std::abs(in.desired_hip_rad-hip_rad_) > 1e-5F, phase_};
    }
    elapsed_s_ += dt;
    const bool quiet = !in.held && elapsed_s_ >= std::max(in.settle_s, 0.0F) &&
        std::abs(in.virtual_error_rad) < 0.04F &&
        std::abs(in.virtual_rate_rad_s) < 0.40F && std::abs(in.body_rate_rad_s) < 0.35F;
    quiet_s_ = quiet ? std::min(quiet_s_ + dt, 0.15F) : 0.0F;
    const bool may_move = quiet_s_ >= 0.15F;
    if (may_move) {
      phase_ = Phase::PoseTransfer;
      const float step = std::clamp(in.max_hip_rate_rad_s, 0.0F, 0.12F) * dt;
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
 private:
  Phase phase_ = Phase::Inactive;
  float hip_rad_ = 0.0F, level_hip_rad_ = 0.0F, elapsed_s_ = 0.0F, quiet_s_ = 0.0F;
};

}  // namespace tap_fw::v3
