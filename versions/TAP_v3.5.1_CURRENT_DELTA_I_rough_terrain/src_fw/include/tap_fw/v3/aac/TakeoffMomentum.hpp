#pragma once

#include <algorithm>
#include <cmath>

namespace tap_fw::v3 {

// TakeoffMomentum: the ground half of the airborne attitude control (AAC), active from the dip start
// to the lift-off.
//
// In the air only the legs can turn the robot, and only by a fraction of what the angular momentum L
// at the lift-off does (FlightAttitude). So the quantity that decides the touchdown is fixed on the
// ground: the virtual pitch the robot would land with if it left the ground now,
//     e = theta_v + L T / I - (theta_ref + bias)              (T: planned flight time)
// and this block drives e to zero before the lift-off.
//
// Actuator: the CoM line (hip). The normal force on the wheels times the CoM offset from the axle
// changes L with no delay beyond the leg servo, so the lean is
//     lean = -gain * e - rate_gain * d(theta_v)/dt          (clamped, first-order smoothed)
// added to the CoM line the hip holds along the dip/thrust path. During the thrust the wheels hold the
// speed they had when it started (`hold_wheels`): the balance loop acts through the slow wheel servo
// (tens of ms of delay and lag against a ~0.1 s thrust) and would add momentum after it was needed.
// The dip keeps the balance loop (the CoM drifts relative to the axle there and the wheels follow it).
class TakeoffMomentum {
 public:
  struct Config {
    bool enabled = true;
    bool hold_wheels = true;
    float gain = 0.5F;               // CoM line lean per rad of predicted touchdown error
    float rate_gain = 0.0F;          // per rad/s of virtual pitch rate
    float lean_max_rad = 0.12F;
    float lean_tau_s = 0.02F;
    float sign = 1.0F;               // +1/-1: lean direction that reduces e (fixed by the model, tested in sim)
    // 0: hold the virtual pitch at its value when the dip started (the CoM stays where it was over
    // the axle, so the wheels do not chase it); 1: drive the full touchdown prediction e.
    float momentum_weight = 0.0F;
  };
  struct Input {
    float dt_s = 0.002F;
    float theta_v = 0.0F;
    float theta_v_rate = 0.0F;
    float theta_start = 0.0F;        // virtual pitch when the dip started
    float theta_goal = 0.0F;         // balance angle + the landing bias (FlightAttitude::Config::line_bias_rad)
    float angular_momentum = 0.0F;   // AirborneEstimator
    float inertia = 0.0F;            // at the planned flight shape (AirborneModel)
    float flight_time_s = 0.0F;      // planned
  };
  struct Output {
    float lean_rad = 0.0F;
    float predicted_error_rad = 0.0F;
  };

  TakeoffMomentum() noexcept = default;
  explicit TakeoffMomentum(Config config) noexcept : config_(config) {}
  void reset() noexcept { lean_ = 0.0F; }

  [[nodiscard]] Output update(const Input& in) noexcept {
    Output out{};
    out.predicted_error_rad = in.theta_v + in.angular_momentum * in.flight_time_s / std::max(in.inertia, 1.0e-4F) -
        in.theta_goal;
    const float hold_error = in.theta_v - in.theta_start;
    const float e = hold_error + config_.momentum_weight * (out.predicted_error_rad - hold_error);
    if (!config_.enabled) {
      lean_ = 0.0F;
      return out;
    }
    const float want = std::clamp(-config_.sign * (config_.gain * e +
                                                   config_.rate_gain * in.theta_v_rate),
                                  -config_.lean_max_rad, config_.lean_max_rad);
    lean_ += std::clamp(in.dt_s / std::max(config_.lean_tau_s, 1.0e-3F), 0.0F, 1.0F) * (want - lean_);
    out.lean_rad = lean_;
    return out;
  }
  [[nodiscard]] Config& config() noexcept { return config_; }

 private:
  Config config_{};
  float lean_ = 0.0F;
};

}  // namespace tap_fw::v3
