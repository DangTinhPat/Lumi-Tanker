#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace tap_fw::v3 {

// Short-horizon actuator prediction, following the queued-command structure of
// robot_control::LqrActuationPredictor. The local COM model uses an observed
// acceleration residual: it does NOT assume a free pendulum while recovering.
// Current hip motion is already in measured COM rate and carrier velocity.
// Residual/support/hip dynamics are frozen over the horizon, not certified.
class RecoveryWheelPredictor {
 public:
  struct Forecast {
    float lean = 0.0F, rate = 0.0F, speed = 0.0F;
    float measured_accel = 0.0F, angular_accel = 0.0F;
    bool valid = false;
  };

  void reset() noexcept { *this = RecoveryWheelPredictor{}; }

  void observe(float dt, float rate, float speed) noexcept {
    now_ += dt;
    if (!initialized_) {
      rate_filtered_ = rate;
      speed_filtered_ = speed;
      initialized_ = true;
    } else if (dt > 0.0F) {
      // Filtered differentiators, not raw encoder finite differences. Current
      // support forces and deliberate hip motion are included in this residual.
      constexpr float tau = 0.04F;
      angular_accel_ = std::clamp((rate - rate_filtered_) / (tau + dt), -40.0F, 40.0F);
      accel_ = std::clamp((speed - speed_filtered_) / (tau + dt), -6.0F, 6.0F);
      rate_filtered_ += dt * angular_accel_;
      speed_filtered_ += dt * accel_;
    }
  }

  void record(float relative_speed) noexcept {
    history_[head_] = {now_, relative_speed};
    head_ = (head_ + 1U) % kCapacity;
    count_ = std::min(count_ + 1U, kCapacity);
  }

  // Kinematic preview for recovery on changing supports. Do not assume a
  // queued speed request has produced braking before encoder/IMU evidence.
  // The caller bounds its correction and ramps authority. Constant measured
  // angular acceleration is a local extrapolation, not a contact model.
  Forecast predictMeasuredMotion(float lean, float rate, float speed,
                                float delay) const noexcept {
    Forecast out{lean, rate, speed, accel_, angular_accel_, false};
    if (!initialized_ || count_ < 2U) return out;
    const std::size_t oldest = (head_ + kCapacity - count_) % kCapacity;
    if (now_ - history_[oldest].time + 1.0e-5F < delay) return out;
    out.lean += delay * rate + 0.5F * delay * delay * angular_accel_;
    out.rate += delay * angular_accel_;
    // No motor response is invented; speed is held at its measured value.
    out.valid = std::isfinite(out.lean) && std::isfinite(out.rate) && std::isfinite(speed);
    return out;
  }

  Forecast predict(float lean, float rate, float speed, float carrier,
                   float omega, float delay, float lag, float accel_max) const noexcept {
    Forecast out{lean, rate, speed, accel_, angular_accel_, false};
    if (!initialized_ || count_ < 2U) return out;
    const std::size_t oldest = (head_ + kCapacity - count_) % kCapacity;
    // No invented prehistory: observe/record throughout Folding/Kneeling.
    if (now_ - history_[oldest].time + 1.0e-5F < delay) return out;
    constexpr unsigned steps = 32U;
    // Predict to the arrival of a newly issued command, integrating the lag
    // of commands already in flight. Servo lag lead is applied by the caller.
    const float dt = delay / static_cast<float>(steps);
    const float omega2 = omega * omega;
    const float inv_length = omega2 / 9.80665F;
    const float initial_sin = std::sin(lean);
    std::size_t cursor = 0U;
    for (unsigned step = 0U; step < steps; ++step) {
      const float issue_time = now_ + (static_cast<float>(step) + 0.5F)*dt - delay;
      while (cursor + 1U < count_ &&
             history_[(oldest + cursor + 1U) % kCapacity].time <= issue_time) ++cursor;
      // Beyond the known queue, hold the most recently issued command. A new
      // candidate command is not silently assumed to have reached the motor.
      const float target = history_[(oldest + cursor) % kCapacity].speed + carrier;
      const float accel = std::clamp((target - out.speed) / lag, -accel_max, accel_max);
      // theta_ddot = omega² sin(theta) - a/L + observed residual.
      // Difference form preserves measured motion at the initial state, even
      // when knees/body support the robot. Future support changes are unknown.
      const float angular = std::clamp(angular_accel_ +
          omega2*(std::sin(out.lean) - initial_sin) -
          inv_length*(accel - accel_), -40.0F, 40.0F);
      out.lean += dt*out.rate + 0.5F*dt*dt*angular;
      out.rate += dt*angular;
      out.speed += dt*accel;
    }
    out.valid = std::isfinite(out.lean) && std::isfinite(out.rate) && std::isfinite(out.speed);
    return out;
  }

 private:
  struct Sample { float time = 0.0F, speed = 0.0F; };
  static constexpr std::size_t kCapacity = 128U;
  std::array<Sample, kCapacity> history_{};
  std::size_t head_ = 0U, count_ = 0U;
  bool initialized_ = false;
  float now_ = 0.0F, rate_filtered_ = 0.0F, speed_filtered_ = 0.0F;
  float accel_ = 0.0F, angular_accel_ = 0.0F;
};

}  // namespace tap_fw::v3
