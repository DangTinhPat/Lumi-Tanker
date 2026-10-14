#pragma once
#include <algorithm>
#include <cmath>

namespace tap_fw::v3 {

// Experimental posture governor from the 12:44 capture revision. It supervises
// hip geometry after the existing geometric StandUp catch; the wheel LQR,
// sensors and safety guards remain in V3Controller.
class BalanceCaptureGovernor {
 public:
  enum class Phase { Inactive, Arrest, Verify, Adjust, Observe, Normal, Failed };
  enum class Failure { None, Timeout, Travel, Response, Invalid };
  struct Config {
    float body_target_rad = 0.0F;
    float body_tolerance_rad = 0.08F;
    float rate_rms_max_rad_s = 0.20F;
    float lean_rms_max_rad = 0.025F;
    float speed_quiet_m_s = 0.10F;
    float command_rms_max_m_s = 0.15F;
    float speed_peak_max_m_s = 0.25F;
    float energy_tau_s = 0.4F;
    float arrest_s = 2.0F;
    float qualify_s = 0.5F;
    float observe_s = 0.5F;
    float timeout_s = 30.0F;
    float progress_grace_s = 3.0F;
    float progress_extension_max_s = 15.0F;
    float progress_min_rad = 0.01F;
    float hip_step_rad = 0.015F;
    float hip_rate_rad_s = 0.04F;
    float hip_accel_rad_s2 = 0.20F;
    float travel_max_rad = 0.35F;
    float response_worsen_rad = 0.01F;
  };
  struct Input {
    float dt_s = 0.002F;
    float body_pitch_rad = 0.0F;
    float body_rate_rad_s = 0.0F;
    float virtual_error_rad = 0.0F;
    float virtual_rate_rad_s = 0.0F;
    float ground_speed_m_s = 0.0F;
    float command_speed_m_s = 0.0F;
    bool held = false;
  };
  struct Output { float hip_rad; Phase phase; Failure failure; bool limited; };

  void reset() noexcept { *this = BalanceCaptureGovernor{}; }
  void begin(float hip_rad, Config config) noexcept {
    reset(); config_ = config;
    origin_ = hip_ = endpoint_ = hip_rad;
    deadline_ = config_.timeout_s;
    phase_ = Phase::Arrest;
  }
  Output update(const Input& in) noexcept {
    if (phase_ == Phase::Inactive || phase_ == Phase::Failed) return output();
    if (!std::isfinite(in.dt_s) || !std::isfinite(in.body_pitch_rad) ||
        !std::isfinite(in.body_rate_rad_s) || !std::isfinite(in.virtual_error_rad) ||
        !std::isfinite(in.virtual_rate_rad_s) || !std::isfinite(in.ground_speed_m_s) ||
        !std::isfinite(in.command_speed_m_s)) return fail(Failure::Invalid);
    if (phase_ == Phase::Normal) return output();
    const float dt = std::clamp(in.dt_s, 0.0F, 0.02F);
    elapsed_ += dt;
    if (elapsed_ > deadline_) return fail(Failure::Timeout);

    const float a = dt / (std::max(config_.energy_tau_s, 0.01F) + dt);
    body_energy_ += a * (in.body_rate_rad_s*in.body_rate_rad_s - body_energy_);
    lean_energy_ += a * (in.virtual_rate_rad_s*in.virtual_rate_rad_s - lean_energy_);
    error_energy_ += a * (in.virtual_error_rad*in.virtual_error_rad - error_energy_);
    speed_energy_ += a * (in.ground_speed_m_s*in.ground_speed_m_s - speed_energy_);
    command_energy_ += a * (in.command_speed_m_s*in.command_speed_m_s - command_energy_);

    // Qualification remains sensor/control based; quiet is not proof of floor contact.
    const bool quiet =
        body_energy_ < config_.rate_rms_max_rad_s*config_.rate_rms_max_rad_s &&
        lean_energy_ < config_.rate_rms_max_rad_s*config_.rate_rms_max_rad_s &&
        error_energy_ < config_.lean_rms_max_rad*config_.lean_rms_max_rad &&
        std::abs(in.body_rate_rad_s) < 0.35F && std::abs(in.virtual_rate_rad_s) < 0.40F &&
        speed_energy_ < config_.speed_quiet_m_s*config_.speed_quiet_m_s &&
        command_energy_ < config_.command_rms_max_m_s*config_.command_rms_max_m_s &&
        std::abs(in.ground_speed_m_s) < config_.speed_peak_max_m_s &&
        std::abs(in.command_speed_m_s) < config_.speed_peak_max_m_s && !in.held;

    // The 12:44 version compares the qualified endpoint response. The later
    // per-side encoder/stationarity window and rate-growth gate are withdrawn.
    if (phase_ == Phase::Arrest && elapsed_ >= config_.arrest_s) {
      phase_ = Phase::Verify;
      quiet_s_ = 0.0F;
    }
    if (phase_ == Phase::Verify) {
      quiet_s_ = quiet ? quiet_s_ + dt : 0.0F;
      if (quiet_s_ >= config_.qualify_s) {
        const float error = in.body_pitch_rad - config_.body_target_rad;
        if (std::abs(error) <= config_.body_tolerance_rad) {
          phase_ = Phase::Normal;
          return output();
        }
        before_error_ = std::abs(error);
        endpoint_ = hip_ + std::clamp(0.20F*error,
            -config_.hip_step_rad, config_.hip_step_rad);
        if (std::abs(endpoint_ - origin_) > config_.travel_max_rad)
          return fail(Failure::Travel);
        phase_ = Phase::Adjust;
        rate_ = 0.0F;
        return output();
      }
    } else if (phase_ == Phase::Adjust) {
      if (!quiet) { rate_ = 0.0F; return output(); }
      const float delta = endpoint_ - hip_;
      const float speed = std::min(config_.hip_rate_rad_s,
          std::sqrt(2.0F*config_.hip_accel_rad_s2*std::abs(delta)));
      const float want = std::copysign(speed, delta);
      rate_ += std::clamp(want-rate_, -config_.hip_accel_rad_s2*dt,
          config_.hip_accel_rad_s2*dt);
      hip_ += std::copysign(std::min(std::abs(rate_*dt), std::abs(delta)), delta);
      if (std::abs(endpoint_ - hip_) < 1e-5F) {
        hip_ = endpoint_;
        rate_ = 0.0F;
        phase_ = Phase::Observe;
        observe_s_ = quiet_s_ = 0.0F;
      }
    } else if (phase_ == Phase::Observe) {
      if (!quiet) { observe_s_ = quiet_s_ = 0.0F; return output(); }
      observe_s_ += dt;
      quiet_s_ += dt;
      if (observe_s_ >= config_.observe_s && quiet_s_ >= config_.qualify_s) {
        const float after_error = std::abs(in.body_pitch_rad-config_.body_target_rad);
        if (after_error > before_error_ + config_.response_worsen_rad)
          return fail(Failure::Response);
        if (before_error_ - after_error >= config_.progress_min_rad) {
          const float hard_deadline = config_.timeout_s +
              std::max(config_.progress_extension_max_s, 0.0F);
          deadline_ = std::max(deadline_, std::min(hard_deadline,
              elapsed_ + std::max(config_.progress_grace_s, 0.0F)));
        }
        phase_ = Phase::Verify;
        // Carry the completed quiet dwell into Verify. A new disturbance clears
        // it there before any next hip step is authorized.
      }
    }
    return output();
  }
  [[nodiscard]] Failure failure() const noexcept { return failure_; }
  [[nodiscard]] Phase phase() const noexcept { return phase_; }
  [[nodiscard]] float bodyRateRms() const noexcept { return std::sqrt(body_energy_); }
  [[nodiscard]] unsigned blockReason() const noexcept { return 0U; }

 private:
  Output output() const noexcept { return {hip_, phase_, failure_, phase_ != Phase::Normal}; }
  Output fail(Failure reason) noexcept {
    phase_ = Phase::Failed; failure_ = reason; rate_ = 0.0F; return output();
  }
  Config config_{};
  Phase phase_ = Phase::Inactive;
  Failure failure_ = Failure::None;
  float hip_ = 0.0F, origin_ = 0.0F, endpoint_ = 0.0F, rate_ = 0.0F;
  float elapsed_ = 0.0F, quiet_s_ = 0.0F, observe_s_ = 0.0F, before_error_ = 0.0F;
  float deadline_ = 0.0F;
  float body_energy_ = 0.0F, lean_energy_ = 0.0F, error_energy_ = 0.0F;
  float speed_energy_ = 0.0F, command_energy_ = 0.0F;
};

}  // namespace tap_fw::v3
