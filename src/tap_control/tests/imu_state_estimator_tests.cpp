#include "robot_control/ImuStateEstimator.hpp"

#include <array>
#include <cassert>
#include <cmath>

int main() {
  robot_control::ImuStateEstimator estimator;
  const std::array<float, 3> accel{0.0F, 0.0F, 9.81F};
  const std::array<float, 3> gyro{0.0F, 0.0F, 0.0F};
  for (unsigned i = 0; i < 49; ++i) {
    const auto output = estimator.update(accel, gyro, 0.002F, i * 0.002F);
    assert(output.valid);
    assert(!output.seeded);
  }
  const auto seeded = estimator.update(accel, gyro, 0.002F, 0.1F);
  assert(seeded.valid && seeded.seeded);
  assert(std::abs(seeded.pitch_rad) < 1.0e-5F);
  assert(seeded.accel_tilt_valid == false);

  const std::array<float, 3> impact_accel{0.0F, 0.0F, 40.0F};
  const std::array<float, 3> impact_gyro{0.0F, 4.5F, 0.0F};
  const auto impact = estimator.update(impact_accel, impact_gyro, 0.002F, 0.102F);
  assert(impact.valid && impact.hard_impact_event);
  const auto held = estimator.update(accel, gyro, 0.002F, 0.12F);
  assert(held.hard_impact_event);
  const auto expired = estimator.update(accel, gyro, 0.002F, 0.20F);
  assert(!expired.hard_impact_event);

  // A slowly changing gyro bias must be learned only through the explicit
  // quiet gate; repeated covariance updates must not invent a larger bias.
  robot_control::ImuStateEstimator::Config bias_config;
  bias_config.seed_window_samples = 10;
  bias_config.initial_angle_variance = 1.0e-4F;
  bias_config.initial_bias_variance = 1.0e-4F;
  bias_config.bias_learning_gain = 0.25F;
  robot_control::ImuStateEstimator bias_estimator(bias_config);
  for (unsigned i = 0; i < 10; ++i) {
    const auto out = bias_estimator.update(
        accel, {0.0F, 0.02F, 0.0F}, 0.002F, i * 0.002F, 0.0F, false, true);
    assert(out.valid);
  }
  auto bias_output = bias_estimator.update(
      accel, {0.0F, 0.06F, 0.0F}, 0.002F, 0.022F, 0.0F, false, true);
  for (unsigned i = 1; i < 1000; ++i) {
    bias_output = bias_estimator.update(
        accel, {0.0F, 0.06F, 0.0F}, 0.002F, 0.022F + i * 0.002F,
        0.0F, false, true);
  }
  assert(std::abs(bias_output.pitch_rad) < 0.01F);
  assert(bias_output.gyro_bias_rad_s > 0.035F &&
         bias_output.gyro_bias_rad_s < 0.08F);
  assert(std::abs(bias_output.pitch_rate_rad_s) < 0.03F);
  assert(std::abs(bias_output.raw_pitch_rate_rad_s - 0.06F) < 0.01F);

  // A 1 m/s^2 body acceleration must not be interpreted as static tilt when
  // the encoder-derived acceleration is supplied. This checks both the
  // compensation path and its sign.
  robot_control::ImuStateEstimator::Config dynamic_config;
  dynamic_config.seed_window_samples = 4;
  dynamic_config.accel_correction_gain_hz = 55.0F;
  dynamic_config.commanded_accel_blend = 0.0F;
  robot_control::ImuStateEstimator dynamic_estimator(dynamic_config);
  const std::array<float, 3> accelerating_accel{1.0F, 0.0F, 9.81F};
  for (unsigned i = 0; i < 4; ++i) {
    (void)dynamic_estimator.update(
        accel, gyro, 0.002F, 0.002F * static_cast<float>(i));
  }
  const auto dynamic_output = dynamic_estimator.update(
      accelerating_accel, gyro, 0.002F, 0.010F, 0.0F, false, false, 1.0F,
      true);
  assert(dynamic_output.body_accel_compensation_valid);
  assert(std::abs(dynamic_output.accel_tilt_x_m_s2) < 1.0e-5F);
  assert(std::abs(dynamic_output.pitch_rad) < 0.002F);

  // A gravity-like but implausibly large one-sample direction change must not
  // teleport the integrated pitch during a wheel/body impulse.
  const auto rejected_innovation = dynamic_estimator.update(
      {9.81F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, 0.002F, 0.012F,
      0.0F, false, false, 0.0F, false);
  assert(std::abs(rejected_innovation.pitch_rad - dynamic_output.pitch_rad) <
         0.003F);

  // A gravity-like but directionally invalid leg transient must not create a
  // false fall while the posture transaction owns the attitude gate.
  const auto pose_transient = dynamic_estimator.update(
      {2.35F, 0.0F, -10.25F}, {0.0F, 0.39F, 0.0F}, 0.02F, 0.03F,
      0.0F, true, false, 0.07F, true);
  assert(pose_transient.accel_tilt_valid);
  assert(std::abs(pose_transient.pitch_rad) < 0.03F);

  return 0;
}
