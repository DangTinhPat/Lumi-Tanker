#include "robot_control/ImuStateEstimator.hpp"

#include <algorithm>
#include <cmath>

namespace robot_control {
namespace {

bool finite3(const std::array<float, 3>& v) noexcept {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

}  // namespace

void ImuStateEstimator::reset() noexcept {
  pitch_rad_ = 0.0F;
  gyro_bias_ = 0.0F;
  p_[0][0] = p_[0][1] = p_[1][0] = p_[1][1] = 0.0F;
  seed_sum_ = {};
  seed_gyro_sum_ = 0.0F;
  seed_gyro_count_ = 0;
  seed_count_ = 0;
  seeded_ = false;
  impact_until_s_ = 0.0F;
  pitch_lagged_rad_ = 0.0F;
  accel_lagged_ = {};
  kinematic_velocity_ = {};
  kinematic_accel_ = {};
  kinematic_initialized_ = false;
}

ImuStateEstimator::Output ImuStateEstimator::update(
    const std::array<float, 3>& accel, const std::array<float, 3>& gyro,
    float dt_s, float now_s, float commanded_accel, bool pose_changing,
    bool quiet_hint, KinematicVelocity imu_velocity) noexcept {
  Output out;
  if (!finite3(accel) || !finite3(gyro) || !std::isfinite(dt_s) ||
      dt_s <= 0.0F || !std::isfinite(now_s)) {
    return out;
  }
  const float dt = std::clamp(dt_s, 1.0e-4F, 0.02F);
  const float mag = std::sqrt(accel[0] * accel[0] + accel[1] * accel[1] +
                              accel[2] * accel[2]);
  if (!std::isfinite(mag)) return out;
  out.valid = true;
  out.accel_magnitude_m_s2 = mag;
  // Keep the raw rate on the public dynamics channel during bring-up. The
  // bias-corrected rate is used internally for attitude integration; exposing
  // a poorly identified rate bias to the fast MPC would add a second failure
  // mode before calibration confidence is established.
  out.pitch_rate_rad_s = gyro[1];
  out.raw_pitch_rate_rad_s = gyro[1];
  out.roll_rate_rad_s = gyro[0];
  out.yaw_rate_rad_s = gyro[2];
  if (mag >= config_.impact_accel_m_s2 &&
      std::abs(gyro[1]) >= config_.impact_pitch_rate_rad_s) {
    impact_until_s_ = now_s + std::max(config_.impact_latch_s, 0.0F);
  }
  out.hard_impact_event = now_s <= impact_until_s_;

  if (!seeded_) {
    if (mag < config_.seed_min_accel_m_s2 || mag > config_.seed_max_accel_m_s2) {
      seed_count_ = 0;
      seed_sum_ = {};
      return out;
    }
    seed_sum_[0] += accel[0];
    seed_sum_[1] += accel[1];
    seed_sum_[2] += accel[2];
    if (std::abs(gyro[1]) <=
        std::max(config_.seed_max_gyro_rate_rad_s, 0.0F)) {
      seed_gyro_sum_ += gyro[1];
      ++seed_gyro_count_;
    }
    ++seed_count_;
    if (seed_count_ < std::max(config_.seed_window_samples, 1U)) return out;
    const float n = static_cast<float>(seed_count_);
    pitch_rad_ = std::atan2(-seed_sum_[0] / n, seed_sum_[2] / n);
    gyro_bias_ = seed_gyro_count_ > 0
        ? seed_gyro_sum_ / static_cast<float>(seed_gyro_count_)
        : 0.0F;
    p_[0][0] = std::max(config_.initial_angle_variance, 1.0e-8F);
    p_[0][1] = p_[1][0] = 0.0F;
    p_[1][1] = std::max(config_.initial_bias_variance, 1.0e-10F);
    pitch_lagged_rad_ = pitch_rad_;
    accel_lagged_ = {seed_sum_[0] / n, seed_sum_[2] / n};
    kinematic_initialized_ = false;
    seeded_ = true;
    out.seeded = true;
    out.pitch_rad = pitch_rad_;
    out.gyro_bias_rad_s = gyro_bias_;
    out.pitch_rate_rad_s = gyro[1] - gyro_bias_;
    out.raw_pitch_rate_rad_s = gyro[1];
    return out;
  }

  const float body_accel_limit =
      std::max(config_.body_accel_limit_m_s2, 0.0F);
  const float tau = std::max(config_.kinematic_velocity_filter_tau_s, 0.0F);
  const float lag_alpha = std::clamp(dt / (tau + dt), 0.0F, 1.0F);

  const bool measured_accel_valid = imu_velocity.valid &&
      std::isfinite(imu_velocity.horizontal_m_s) &&
      std::isfinite(imu_velocity.vertical_m_s);
  if (!measured_accel_valid) {
    // Decay with the accelerometer's matched lag: the lagged specific force
    // still contains the acceleration that was just being compensated.
    kinematic_initialized_ = false;
    for (auto& value : kinematic_accel_) value -= lag_alpha * value;
  } else {
    const std::array<float, 2> velocity{imu_velocity.horizontal_m_s,
                                        imu_velocity.vertical_m_s};
    if (!kinematic_initialized_) {
      // Start from the first delayed encoder packet. A startup difference
      // would turn the sensor latency edge into a false impact.
      kinematic_velocity_ = velocity;
      kinematic_accel_ = {};
      kinematic_initialized_ = true;
    } else {
      const auto previous_velocity = kinematic_velocity_;
      // Differentiating quantised encoder velocity directly amplifies its
      // 500 Hz noise by 1/dt. Filter velocity first; its derivative is then
      // the same one-pole lag as the accelerometer branch below.
      for (std::size_t i = 0; i < 2; ++i) {
        kinematic_velocity_[i] += lag_alpha *
            (velocity[i] - kinematic_velocity_[i]);
        kinematic_accel_[i] = std::clamp(
            (kinematic_velocity_[i] - previous_velocity[i]) / dt,
            -body_accel_limit, body_accel_limit);
      }
    }
  }
  // World (horizontal, vertical) IMU acceleration projected into body x/z.
  const float c = std::cos(pitch_rad_);
  const float s = std::sin(pitch_rad_);
  float body_accel = std::clamp(
      c * kinematic_accel_[0] - s * kinematic_accel_[1],
      -body_accel_limit, body_accel_limit);
  const float body_accel_z = std::clamp(
      s * kinematic_accel_[0] + c * kinematic_accel_[1],
      -body_accel_limit, body_accel_limit);
  // Keep the MPC command as a small model feed-forward term. If the encoder
  // path is valid it nudges the measured value; it never replaces it. This
  // prevents actuator delay/slip from closing an acceleration->tilt positive
  // feedback loop through the estimator.
  if (measured_accel_valid && std::isfinite(commanded_accel)) {
    const float command = std::clamp(
        commanded_accel, -body_accel_limit, body_accel_limit);
    const float blend = std::clamp(config_.commanded_accel_blend, 0.0F, 1.0F);
    body_accel += blend * (command - body_accel);
  } else if (!measured_accel_valid && std::isfinite(commanded_accel) &&
             std::abs(commanded_accel) <=
                 std::max(config_.accel_command_gate_m_s2, 0.0F)) {
    body_accel = std::clamp(body_accel + commanded_accel, -body_accel_limit,
                            body_accel_limit);
  }
  // Gate each raw sample before it enters the matched filter; otherwise the
  // filter would smear one implausible packet into a plausible-looking tilt.
  bool accel_sample_plausible = false;
  {
    const float raw_x = accel[0] - body_accel;
    const float raw_z = accel[2] - body_accel_z;
    const float raw_mag =
        std::sqrt(raw_x * raw_x + accel[1] * accel[1] + raw_z * raw_z);
    const float raw_innovation = std::atan2(-raw_x, raw_z) - pitch_rad_;
    accel_sample_plausible = std::isfinite(raw_innovation) &&
        std::abs(raw_innovation) <=
            std::max(config_.accel_max_innovation_rad, 0.0F) &&
        std::abs(raw_mag - config_.gravity_m_s2) <=
            std::max(config_.accel_max_deviation_m_s2, 0.0F);
    if (accel_sample_plausible) {
      accel_lagged_[0] += lag_alpha * (accel[0] - accel_lagged_[0]);
      accel_lagged_[1] += lag_alpha * (accel[2] - accel_lagged_[1]);
    }
  }
  out.body_accel_x_m_s2 = body_accel;
  out.accel_tilt_x_m_s2 = accel_lagged_[0] - body_accel;
  out.body_accel_compensation_valid = measured_accel_valid;

  // Both signals use the signed body-X/Z convention of the MPU6500 adapter.
  // Subtracting the kinematic acceleration of the IMU point removes the
  // specific force caused by wheel motion, body rotation about the axle and
  // leg articulation before extracting the gravity direction.
  const float corrected_accel_x = out.accel_tilt_x_m_s2;
  const float corrected_accel_z = accel_lagged_[1] - body_accel_z;
  const float measured_pitch = std::atan2(-corrected_accel_x, corrected_accel_z);
  const float corrected_mag = std::sqrt(
      corrected_accel_x * corrected_accel_x + accel[1] * accel[1] +
      corrected_accel_z * corrected_accel_z);
  const float deviation = std::abs(corrected_mag - config_.gravity_m_s2);
  const float trust_scale =
      std::max(config_.kinematic_accel_trust_scale_m_s2, 1.0e-3F);
  const float dynamic_ratio =
      std::hypot(body_accel, body_accel_z) / trust_scale;
  const float dynamic_trust = 1.0F / (1.0F + dynamic_ratio * dynamic_ratio);
  const bool force_valid = accel_sample_plausible &&
      corrected_mag >= config_.seed_min_accel_m_s2 &&
      corrected_mag <= config_.seed_max_accel_m_s2 &&
      deviation <= std::max(config_.accel_max_deviation_m_s2, 0.0F) &&
      std::abs(gyro[1]) <= std::max(config_.accel_max_pitch_rate_rad_s, 0.0F) &&
      (measured_accel_valid ||
       std::abs(commanded_accel) <=
           std::max(config_.accel_command_gate_m_s2, 0.0F));
  out.accel_tilt_valid = force_valid;
  const float innovation = measured_pitch - pitch_lagged_rad_;
  const bool innovation_valid = std::isfinite(innovation) &&
      std::abs(innovation) <=
          std::max(config_.accel_max_innovation_rad, 0.0F);
  const bool bias_quiet = quiet_hint && force_valid && !pose_changing &&
      deviation <= std::max(config_.bias_quiet_max_accel_deviation_m_s2, 0.0F) &&
      (measured_accel_valid ||
       std::abs(commanded_accel) <=
           std::max(config_.accel_command_gate_m_s2, 0.0F)) &&
      std::abs(gyro[1] - gyro_bias_) <=
          std::max(config_.bias_quiet_max_gyro_rate_rad_s, 0.0F);

  // Bias is observable directly from a stationary gyro.  Learn it with a
  // slow, gated low-pass instead of letting the attitude innovation absorb a
  // real body pitch transient as a sensor offset.
  if (bias_quiet) {
    const float bias_alpha = std::clamp(
        std::max(config_.bias_learning_gain, 0.0F) * dt, 0.0F, 1.0F);
    gyro_bias_ += bias_alpha * (gyro[1] - gyro_bias_);
  }

  const float rate = gyro[1] - gyro_bias_;
  pitch_rad_ += dt * rate;
  pitch_lagged_rad_ += lag_alpha * (pitch_rad_ - pitch_lagged_rad_);
  p_[0][0] += dt * (dt * p_[1][1] - p_[0][1] - p_[1][0] + config_.q_angle);
  p_[0][1] -= dt * p_[1][1];
  p_[1][0] -= dt * p_[1][1];
  p_[1][1] += config_.q_bias * dt;

  // Leg articulation is part of the kinematic model, so a leg transaction
  // only lowers accelerometer trust; the innovation gate still rejects a
  // gravity-like sample whose direction is implausible.
  out.accel_trust = force_valid && innovation_valid ? dynamic_trust : 0.0F;
  if (force_valid && innovation_valid) {
    const float deadband = 0.4F;
    const float excess = std::max(0.0F, deviation - deadband);
    const float trust = (pose_changing ||
                         std::abs(gyro[1]) > config_.accel_max_pitch_rate_rad_s)
                            ? config_.accel_trust_gain : 0.0F;
    const float r = config_.r_measure * (1.0F + trust * excess * excess) /
        std::max(dynamic_trust, 1.0e-3F);
    const float s = p_[0][0] + r;
    if (s > 1.0e-9F && std::isfinite(s)) {
      const float k0 = p_[0][0] / s;
      // The explicit stationary low-pass above owns bias adaptation. Keeping
      // the measurement innovation out of this state prevents a static
      // accelerometer-angle error from becoming a fictitious gyro bias.
      const float k1 = 0.0F;
      const float p00 = p_[0][0];
      const float p01 = p_[0][1];
      const float p10 = p_[1][0];
      const float p11 = p_[1][1];
      pitch_rad_ += k0 * innovation;
      pitch_lagged_rad_ += k0 * innovation;
      // The covariance path remains useful for uncertainty tracking, but its
      // default Q/R pair is intentionally conservative. A fixed complementary
      // correction gives a predictable phase budget at the plant's observed
      // oscillation frequency, while the gates above reject non-gravitational
      // samples. During a leg transaction reduce the gain, do not disable the
      // absolute reference completely.
      const float confidence = 1.0F / (1.0F + deviation * deviation);
      const float correction_gain =
          std::max(config_.accel_correction_gain_hz, 0.0F) *
          confidence * dynamic_trust;
      const float correction_alpha = std::clamp(
          1.0F - std::exp(-correction_gain * dt), 0.0F, 1.0F);
      pitch_rad_ += correction_alpha * innovation;
      pitch_lagged_rad_ += correction_alpha * innovation;
      gyro_bias_ += k1 * innovation;
      // A stationary accelerometer provides a weak absolute attitude
      // reference. Integrate only its low-frequency innovation into the gyro
      // bias while the explicit quiet gate is true. This is the fixed-cost
      // 1-D Mahony/Kalman bias observer: changing pitch or drive acceleration
      // cannot be learned as calibration.
      if (bias_quiet && std::isfinite(innovation)) {
        const float limit = std::max(config_.bias_innovation_limit_rad, 0.0F);
        const float bounded = std::clamp(innovation, -limit, limit);
        gyro_bias_ -= std::max(config_.bias_innovation_gain_hz, 0.0F) *
            bounded * dt;
      }
      const float bias_limit = std::max(config_.max_bias_rad_s, 0.0F);
      gyro_bias_ = std::clamp(gyro_bias_, -bias_limit, bias_limit);
      // Joseph-form algebra for this scalar measurement, using one
      // consistent prior covariance.  Reusing the already-updated p00/p01
      // for the second row makes P asymmetric and can create a false bias
      // walk under repeated quiet samples.
      p_[0][0] = p00 - k0 * p00;
      p_[0][1] = p01 - k0 * p01;
      p_[1][0] = p10 - k1 * p00;
      p_[1][1] = p11 - k1 * p01;
    }
  }
  out.seeded = true;
  out.pitch_rad = pitch_rad_;
  out.pitch_rate_rad_s = gyro[1] - gyro_bias_;
  out.raw_pitch_rate_rad_s = gyro[1];
  out.gyro_bias_rad_s = gyro_bias_;
  return out;
}

}  // namespace robot_control
