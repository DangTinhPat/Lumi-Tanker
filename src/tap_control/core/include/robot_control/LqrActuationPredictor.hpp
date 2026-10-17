#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace robot_control {

// Predict the pendulum until queued wheel commands reach its speed servo.
// Inputs are IMU/encoder state and previously sent commands, never plant pose.
class LqrActuationPredictor {
 public:
  struct Config {
    bool enabled = false;
    float delay_s = .05F;
    float lag_s = .075F;
  };
  using State = std::array<float, 4>;  // axle x, virtual pitch error, axle v, pitch rate
  static constexpr std::size_t kCapacity = 400;

  void reset() noexcept {
    history_.fill(0);
    head_ = ticks_ = 0;
    dt_s_ = 0;
  }

  State predict(State state, const Config& cfg, float dt_s, float length_m,
                float carrier_m_s, float max_accel_m_s2) noexcept {
    if (!configure(cfg, dt_s)) return state;
    const float dt = dt_s_;
    const float length = std::max(length_m, .02F);
    const float lag = std::max(cfg.lag_s, dt);
    const float limit = std::max(max_accel_m_s2, .1F);
    for (std::size_t i = 0; i < ticks_; ++i) {
      const float target = history_[(head_+i)%ticks_] + carrier_m_s;
      const float accel = std::clamp((target-state[2])/lag, -limit, limit);
      const float pitch_accel = (9.81F*state[1]-accel)/length;
      state[0] += dt*state[2] + .5F*dt*dt*accel;
      state[1] += dt*state[3] + .5F*dt*dt*pitch_accel;
      state[2] += dt*accel;
      state[3] += dt*pitch_accel;
    }
    return state;
  }

  void record(float sent_relative_speed_m_s) noexcept {
    if (ticks_ == 0) return;
    history_[head_] = std::isfinite(sent_relative_speed_m_s) ? sent_relative_speed_m_s : 0;
    head_ = (head_+1)%ticks_;
  }

  // Recovery also sends wheel commands before LQR takes ownership. Preserve
  // their real delay queue without invoking a free-pendulum prediction while
  // knees/body may still support the robot.
  void record(float sent_relative_speed_m_s, const Config& cfg, float dt_s) noexcept {
    if (configure(cfg, dt_s)) record(sent_relative_speed_m_s);
  }

 private:
  bool configure(const Config& cfg, float dt_s) noexcept {
    if (!cfg.enabled || !std::isfinite(dt_s) || dt_s <= 0 ||
        !std::isfinite(cfg.delay_s) || !std::isfinite(cfg.lag_s)) {
      reset();
      return false;
    }
    const float dt = std::clamp(dt_s, .00025F, .05F);
    const auto ticks = static_cast<std::size_t>(std::clamp(
        std::lround(std::clamp(cfg.delay_s, 0.0F, .1F) / dt), 0L, 400L));
    if (ticks != ticks_ || std::abs(dt-dt_s_) > 1e-6F) {
      reset();
      ticks_ = ticks;
      dt_s_ = dt;
    }
    return true;
  }
  std::array<float, kCapacity> history_{};
  std::size_t head_ = 0, ticks_ = 0;
  float dt_s_ = 0;
};

}  // namespace robot_control
