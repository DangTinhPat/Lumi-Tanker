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
}

ImuStateEstimator::Output ImuStateEstimator::update(
    const std::array<float, 3>& accel, const std::array<float, 3>& gyro,
    float dt_s, float now_s, float commanded_accel, bool pose_changing,
    bool quiet_hint, float measured_body_accel,
    bool measured_body_accel_valid) noexcept {
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
  const bool measured_accel_valid = measured_body_accel_valid &&
      std::isfinite(measured_body_accel);
  float body_accel = measured_accel_valid
      ? std::clamp(measured_body_accel, -body_accel_limit, body_accel_limit)
      : 0.0F;
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
    body_accel = std::clamp(commanded_accel, -body_accel_limit,
                            body_accel_limit);
  }
  out.body_accel_x_m_s2 = body_accel;
  out.accel_tilt_x_m_s2 = accel[0] - body_accel;
  out.body_accel_compensation_valid = measured_accel_valid;

  // Both signals use the signed body-X convention of the MPU6500 adapter.
  // Subtracting the measured translational acceleration removes specific
  // force caused by wheel motion before extracting the gravity direction.
  const float corrected_accel_x = out.accel_tilt_x_m_s2;
  const float measured_pitch = std::atan2(-corrected_accel_x, accel[2]);
  const float corrected_mag = std::sqrt(
      corrected_accel_x * corrected_accel_x + accel[1] * accel[1] +
      accel[2] * accel[2]);
  const float deviation = std::abs(corrected_mag - config_.gravity_m_s2);
  // A wheel acceleration estimate is useful to explain specific force only
  // while it is small enough for the reduced sensor model to be trustworthy.
  // During a wheel catch the wheel can accelerate underneath a nearly still
  // body; treating that actuator motion as IMU translational acceleration is
  // worse than temporarily relying on the gyro. This gate is deliberately
  // bounded and sensor-only, not a simulator contact shortcut.
  const bool dynamic_accel_gate = measured_accel_valid &&
      std::abs(body_accel) > 0.35F;
  const bool force_valid = corrected_mag >= config_.seed_min_accel_m_s2 &&
      corrected_mag <= config_.seed_max_accel_m_s2 &&
      deviation <= std::max(config_.accel_max_deviation_m_s2, 0.0F) &&
      std::abs(gyro[1]) <= std::max(config_.accel_max_pitch_rate_rad_s, 0.0F) &&
      !dynamic_accel_gate &&
      (measured_accel_valid ||
       std::abs(commanded_accel) <=
           std::max(config_.accel_command_gate_m_s2, 0.0F));
  out.accel_tilt_valid = force_valid;
  const float innovation = measured_pitch - pitch_rad_;
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
  p_[0][0] += dt * (dt * p_[1][1] - p_[0][1] - p_[1][0] + config_.q_angle);
  p_[0][1] -= dt * p_[1][1];
  p_[1][0] -= dt * p_[1][1];
  p_[1][1] += config_.q_bias * dt;

  // During a leg transaction the accelerometer is allowed to report its
  // diagnostic tilt channel, but it must not correct the integrated attitude.
  // A dynamic sample can have a gravity-like magnitude while its direction is
  // dominated by leg/wheel acceleration; applying the Kalman innovation here
  // creates a false multi-radian fall in one packet.
  if (force_valid && innovation_valid && !pose_changing) {
    const float deadband = 0.4F;
    const float excess = std::max(0.0F, deviation - deadband);
    const float trust = (pose_changing ||
                         std::abs(gyro[1]) > config_.accel_max_pitch_rate_rad_s)
                            ? config_.accel_trust_gain : 0.0F;
    const float r = config_.r_measure * (1.0F + trust * excess * excess);
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
      // The covariance path remains useful for uncertainty tracking, but its
      // default Q/R pair is intentionally conservative. A fixed complementary
      // correction gives a predictable phase budget at the plant's observed
      // oscillation frequency, while the gates above reject non-gravitational
      // samples. During a leg transaction reduce the gain, do not disable the
      // absolute reference completely.
      const float confidence = 1.0F / (1.0F + deviation * deviation);
      const float correction_gain =
          std::max(config_.accel_correction_gain_hz, 0.0F) *
          confidence;
      const float correction_alpha = std::clamp(
          1.0F - std::exp(-correction_gain * dt), 0.0F, 1.0F);
      pitch_rad_ += correction_alpha * innovation;
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
