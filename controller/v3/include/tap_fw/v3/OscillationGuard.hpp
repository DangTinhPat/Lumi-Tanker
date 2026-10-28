#pragma once

#include <algorithm>
#include <cmath>

namespace tap_fw::v3 {

// Detects a growing balance oscillation before the pitch limit is reached.
//
// On the robot the wheel loop is held stable by a delay margin that shrinks with body height (a shorter
// pendulum is faster): at a low height the pitch sat still for ~17 s, then its swings doubled every half
// second until the robot fell. This class watches the pitch error (pitch - balance reference), measures the
// amplitude of every half swing and counts how many in a row grew; the resulting risk (0..1) is what the
// controller uses to cut loop gain, stretch the actuation prediction, hold a height descent and, last, raise the
// body. It does not need the dt of the caller to be constant and works for any control rate.
class OscillationGuard {
 public:
  struct Config {
    float hp_tau_s = 1.0F;            // high-pass time constant: removes the slow drift / trim from the error
    float hysteresis_rad = 0.002F;    // the error must pass this far beyond zero to end a half swing
    float amp_min_rad = 0.006F;       // half swings smaller than this are noise: they reset the streak
    float amp_alarm_rad = 0.12F;      // one half swing this large is risk 1 outright
    float growth_ratio = 1.12F;       // a half swing counts as growing when it is this much above the previous
    int growth_cycles = 3;            // consecutive growing half swings for full risk
    float min_half_period_s = 0.08F;  // shorter or longer half swings are not a balance oscillation
    float max_half_period_s = 1.5F;
    float rise_s = 0.20F;             // risk attack time constant
    float fall_s = 2.0F;              // risk release time constant (slow: the margin is still thin right after)
  };

  struct Output {
    float risk = 0.0F;       // 0 calm .. 1 growing oscillation
    float amplitude = 0.0F;  // peak of the current half swing [rad]
    float growth = 1.0F;     // ratio of the last two half swings
    int streak = 0;          // consecutive growing half swings
  };

  void reset() noexcept {
    initialised_ = false;
    hp_ = 0.0F;
    sign_ = 0;
    peak_ = 0.0F;
    prev_amp_ = 0.0F;
    growth_ = 1.0F;
    streak_ = 0;
    since_cross_s_ = 0.0F;
    risk_ = 0.0F;
  }

  Output update(float error_rad, float dt_s, const Config& cfg) noexcept {
    if (!std::isfinite(error_rad) || !std::isfinite(dt_s) || dt_s <= 0.0F) return output();
    const float dt = std::clamp(dt_s, 1.0e-4F, 0.1F);
    if (!initialised_) {
      initialised_ = true;
      hp_ = error_rad;
    }
    hp_ += std::clamp(dt / std::max(cfg.hp_tau_s, 1.0e-3F), 0.0F, 1.0F) * (error_rad - hp_);
    const float x = error_rad - hp_;
    since_cross_s_ += dt;

    if (sign_ == 0) {
      if (std::abs(x) > cfg.hysteresis_rad) {
        sign_ = x > 0.0F ? 1 : -1;
        peak_ = std::abs(x);
        since_cross_s_ = 0.0F;
      }
    } else if (static_cast<float>(sign_) * x > 0.0F) {
      peak_ = std::max(peak_, std::abs(x));
    } else if (-static_cast<float>(sign_) * x > cfg.hysteresis_rad) {
      // The error crossed to the other side: one half swing is complete.
      const bool period_ok = since_cross_s_ >= cfg.min_half_period_s && since_cross_s_ <= cfg.max_half_period_s;
      if (period_ok && peak_ >= cfg.amp_min_rad) {
        // The first swing after calm is only the baseline: growth needs a previous swing to compare with.
        growth_ = prev_amp_ > 0.0F ? peak_ / prev_amp_ : 1.0F;
        if (growth_ >= cfg.growth_ratio) {
          ++streak_;
        } else if (growth_ < 1.0F) {
          streak_ = 0;
        }
        prev_amp_ = peak_;
      } else {
        streak_ = 0;
        prev_amp_ = 0.0F;
        growth_ = 1.0F;
      }
      sign_ = -sign_;
      peak_ = std::abs(x);
      since_cross_s_ = 0.0F;
    }
    if (since_cross_s_ > cfg.max_half_period_s) {
      // No swing for a long time: whatever was growing has stopped.
      streak_ = 0;
      prev_amp_ = 0.0F;
      growth_ = 1.0F;
    }

    const int cycles = std::max(cfg.growth_cycles, 1);
    float target = 0.0F;
    if (streak_ >= cycles) {
      target = 1.0F;
    } else if (streak_ > 0) {
      target = 0.6F * static_cast<float>(streak_) / static_cast<float>(cycles);
    }
    if (peak_ >= cfg.amp_alarm_rad) target = 1.0F;
    const float tau = target > risk_ ? cfg.rise_s : cfg.fall_s;
    risk_ += std::clamp(dt / std::max(tau, 1.0e-3F), 0.0F, 1.0F) * (target - risk_);
    return output();
  }

  [[nodiscard]] float risk() const noexcept { return risk_; }

 private:
  [[nodiscard]] Output output() const noexcept { return {risk_, peak_, growth_, streak_}; }

  bool initialised_ = false;
  float hp_ = 0.0F;
  int sign_ = 0;
  float peak_ = 0.0F, prev_amp_ = 0.0F, growth_ = 1.0F;
  int streak_ = 0;
  float since_cross_s_ = 0.0F;
  float risk_ = 0.0F;
};

}  // namespace tap_fw::v3
