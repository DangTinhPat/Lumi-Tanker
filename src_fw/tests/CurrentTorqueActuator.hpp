#pragma once

// Simulation-only GDS68 current/torque path. The controller sees modeled Iq;
// the actuator applies torque from its delayed, lagged current state.
#include <algorithm>
#include <cmath>
#include <deque>
#include <random>
#include <utility>

#include "tap_fw/v3/CurrentTorque.hpp"

namespace tap_fw::sim {

class CurrentTorqueActuator {
 public:
  struct Config {
    double kt_nm_per_a = 0.47;
    double current_limit_a = 8.0;  // provisional simulator sweep value
    double current_lag_s = 0.02;   // not measured on this robot
    double correction_gain = 0.0;  // 0 keeps the uncompensated baseline
    unsigned command_delay_ticks = 0;
    unsigned feedback_sample_period_ticks = 1;
    unsigned feedback_delay_ticks = 0;
    unsigned feedback_max_age_ticks = 32;
    unsigned feedback_drop_period_ticks = 0;
    unsigned feedback_drop_ticks = 0;
    double feedback_noise_stddev_a = 0.0;
    unsigned noise_seed = 1;
    double plant_kt_nm_per_a = 0.47;  // sweep truth independently of controller Kt
  };

  struct Output {
    double target_current_a = 0.0;
    double measured_current_a = 0.0;
    double applied_current_a = 0.0;
    double delta_current_a = 0.0;
    double commanded_torque_nm = 0.0;
    double applied_torque_nm = 0.0;
    unsigned feedback_age_ticks = 0;
    bool feedback_valid = false;
  };

  CurrentTorqueActuator() = default;
  explicit CurrentTorqueActuator(Config config)
      : config_(config), noise_rng_(config.noise_seed) {}

  Output step(double requested_torque_nm, double dt_s) {
    if (!std::isfinite(requested_torque_nm)) requested_torque_nm = 0.0;
    if (!validConfig()) {
      reset();
      return output_;
    }
    if (!std::isfinite(dt_s) || dt_s <= 0.0) return output_;

    const double max_torque = config_.plant_kt_nm_per_a * config_.current_limit_a;
    output_.feedback_valid = feedback_seen_ &&
        tick_ - feedback_tick_ <= config_.feedback_max_age_ticks;
    output_.feedback_age_ticks = feedback_seen_ ? tick_ - feedback_tick_ : tick_;
    const auto command = tap_fw::v3::currentTorqueCommand(
        requested_torque_nm, output_.measured_current_a, output_.feedback_valid,
        config_.kt_nm_per_a, config_.current_limit_a, config_.correction_gain);
    output_.target_current_a = command.target_current_a;
    output_.delta_current_a = command.delta_current_a;
    output_.commanded_torque_nm = std::clamp(static_cast<double>(command.torque_nm),
                                             -max_torque, max_torque);

    command_delay_.push_back(output_.commanded_torque_nm);
    double delayed_torque = output_.commanded_torque_nm;
    if (command_delay_.size() > config_.command_delay_ticks) {
      delayed_torque = command_delay_.front();
      command_delay_.pop_front();
    } else {
      delayed_torque = 0.0;
    }
    const double target = clamp(delayed_torque / config_.plant_kt_nm_per_a,
                                config_.current_limit_a);
    const double alpha = config_.current_lag_s > 0.0
        ? std::clamp(dt_s / (config_.current_lag_s + dt_s), 0.0, 1.0) : 1.0;
    output_.applied_current_a += alpha * (target - output_.applied_current_a);
    output_.applied_torque_nm = config_.plant_kt_nm_per_a * output_.applied_current_a;
    ++tick_;
    sampleFeedback();
    return output_;
  }

  void reset() {
    output_ = {};
    command_delay_.clear();
    feedback_queue_.clear();
    feedback_tick_ = tick_ = 0;
    feedback_seen_ = false;
    noise_rng_.seed(config_.noise_seed);
  }

 private:
  bool validConfig() const {
    return std::isfinite(config_.kt_nm_per_a) && config_.kt_nm_per_a > 0.0 &&
           std::isfinite(config_.plant_kt_nm_per_a) && config_.plant_kt_nm_per_a > 0.0 &&
           std::isfinite(config_.current_limit_a) && config_.current_limit_a > 0.0 &&
           std::isfinite(config_.current_lag_s) && config_.current_lag_s >= 0.0 &&
           std::isfinite(config_.correction_gain) && config_.correction_gain >= 0.0 &&
           config_.correction_gain <= 1.0 &&
           config_.feedback_sample_period_ticks > 0 &&
           config_.feedback_noise_stddev_a >= 0.0 &&
           std::isfinite(config_.feedback_noise_stddev_a) &&
           (config_.feedback_drop_period_ticks == 0 ||
            config_.feedback_drop_ticks <= config_.feedback_drop_period_ticks);
  }

  void sampleFeedback() {
    if (tick_ % config_.feedback_sample_period_ticks == 0)
      feedback_queue_.emplace_back(tick_, output_.applied_current_a);
    if (feedback_queue_.empty()) return;
    if (tick_ - feedback_queue_.front().first < config_.feedback_delay_ticks) return;
    const auto sample = feedback_queue_.front();
    feedback_queue_.pop_front();
    const bool dropped = config_.feedback_drop_period_ticks != 0 &&
        sample.first % config_.feedback_drop_period_ticks < config_.feedback_drop_ticks;
    if (dropped) return;
    double noise_a = 0.0;
    if (config_.feedback_noise_stddev_a > 0.0) {
      std::normal_distribution<double> noise(0.0, config_.feedback_noise_stddev_a);
      noise_a = noise(noise_rng_);
    }
    output_.measured_current_a = sample.second + noise_a;
    feedback_tick_ = sample.first;
    feedback_seen_ = true;
  }

  static double clamp(double value, double limit) {
    return std::clamp(value, -limit, limit);
  }

  Config config_;
  Output output_{};
  std::deque<double> command_delay_;
  std::deque<std::pair<unsigned, double>> feedback_queue_;
  std::mt19937 noise_rng_;
  unsigned tick_ = 0, feedback_tick_ = 0;
  bool feedback_seen_ = false;
};

}  // namespace tap_fw::sim
