#include "robot_control/ImuStateEstimator.hpp"
#include "robot_control/WholeBodyModel.hpp"

#include <array>
#include <cassert>
#include <cmath>

namespace {
using robot_control::WholeBodyModel;

struct Pose { float pitch, hip, knee, wheel; };

// World IMU position: rolling axle travel plus the rotated body-frame lever.
std::array<float, 2> imuPosition(const Pose& p) {
  const auto leg = WholeBodyModel::legHeight(p.hip, p.knee);
  const float lx = robot_control::geometry::imu_offset_x_m - leg.x_m;
  const float lz = robot_control::geometry::imu_offset_z_m - leg.height_m;
  const float c = std::cos(p.pitch), s = std::sin(p.pitch);
  return {robot_control::geometry::wheel_radius_m *
              (p.pitch + p.hip + p.knee + p.wheel) + c * lx + s * lz,
          -s * lx + c * lz};
}

Pose poseAt(double t) {
  return {static_cast<float>(0.05 * std::sin(3.0 * t)),
          static_cast<float>(0.6 + 0.3 * std::sin(1.3 * t)),
          static_cast<float>(-0.7 + 0.4 * std::sin(0.9 * t + 0.4)),
          static_cast<float>(2.0 * t)};
}

void imuVelocityMatchesFiniteDifference() {
  const double h = 1.0e-4;
  for (double t = 0.0; t < 3.0; t += 0.37) {
    const Pose a = poseAt(t - h), b = poseAt(t + h), p = poseAt(t);
    const auto pa = imuPosition(a), pb = imuPosition(b);
    const float inv = static_cast<float>(1.0 / (2.0 * h));
    const auto v = WholeBodyModel::imuVelocityThroughLeg(
        p.pitch, (b.pitch - a.pitch) * inv, p.hip, p.knee,
        (b.hip - a.hip) * inv, (b.knee - a.knee) * inv,
        (b.wheel - a.wheel) * inv);
    assert(std::abs(v.horizontal_m_s - (pb[0] - pa[0]) * inv) < 2.0e-3F);
    assert(std::abs(v.vertical_m_s - (pb[1] - pa[1]) * inv) < 2.0e-3F);
  }
}

// A tall stance rocking about a fixed axle: the accelerometer sees the lever
// arm's tangential/centripetal acceleration. Returns the worst pitch error.
float rockingPitchError(bool kinematics_valid) {
  robot_control::ImuStateEstimator::Config config;
  config.seed_window_samples = 10;
  config.accel_correction_gain_hz = 5.0F;
  config.commanded_accel_blend = 0.0F;
  robot_control::ImuStateEstimator estimator(config);
  constexpr float hip = 0.28F, knee = 0.29F;  // max-stand encoder pose
  constexpr float dt = 0.002F, g = 9.81F;
  const auto pitchAt = [](double t) {
    return static_cast<float>(t < 0.5 ? 0.0 : 0.02 * std::sin(15.0 * (t - 0.5)));
  };
  const auto positionAt = [&](double t) {
    const float pitch = pitchAt(t);
    // Wheel counter-rotates so the axle stays put.
    return imuPosition({pitch, hip, knee, -pitch - hip - knee});
  };
  float worst = 0.0F;
  for (int i = 0; i < 2500; ++i) {
    const double t = i * static_cast<double>(dt);
    const float pitch = pitchAt(t);
    const float rate = (pitchAt(t + 1e-4) - pitchAt(t - 1e-4)) / 2e-4F;
    const auto pm = positionAt(t - dt), p0 = positionAt(t), pp = positionAt(t + dt);
    const float ax = (pp[0] - 2.0F * p0[0] + pm[0]) / (dt * dt);
    const float az = (pp[1] - 2.0F * p0[1] + pm[1]) / (dt * dt) + g;
    const float c = std::cos(pitch), s = std::sin(pitch);
    const std::array<float, 3> accel{c * ax - s * az, 0.0F, s * ax + c * az};
    robot_control::ImuStateEstimator::KinematicVelocity kinematic;
    if (kinematics_valid) {
      const auto v = WholeBodyModel::imuVelocityThroughLeg(
          pitch, rate, hip, knee, 0.0F, 0.0F, -rate);
      kinematic = {v.horizontal_m_s, v.vertical_m_s, true};
    }
    const auto out = estimator.update(accel, {0.0F, rate, 0.0F}, dt,
                                      static_cast<float>(t), 0.0F, false,
                                      false, kinematic);
    if (t > 1.0) worst = std::max(worst, std::abs(out.pitch_rad - pitch));
  }
  return worst;
}
}  // namespace

int main() {
  imuVelocityMatchesFiniteDifference();
  const float compensated = rockingPitchError(true);
  const float uncompensated = rockingPitchError(false);
  assert(compensated < 0.004F);
  assert(compensated < 0.25F * uncompensated);

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

  // A 1 m/s^2 forward acceleration must not be interpreted as static tilt when
  // the encoder-derived IMU velocity is supplied. This checks both the
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
  robot_control::ImuStateEstimator::Output dynamic_output;
  for (unsigned i = 0; i < 100; ++i) {
    dynamic_output = dynamic_estimator.update(
        accelerating_accel, gyro, 0.002F, 0.010F + 0.002F * i, 0.0F, false,
        false, {0.002F * static_cast<float>(i), 0.0F, true});
  }
  assert(dynamic_output.body_accel_compensation_valid);
  assert(std::abs(dynamic_output.accel_tilt_x_m_s2) < 0.02F);
  assert(std::abs(dynamic_output.pitch_rad) < 0.002F);

  // A gravity-like but implausibly large one-sample direction change must not
  // teleport the integrated pitch during a wheel/body impulse.
  const auto rejected_innovation = dynamic_estimator.update(
      {9.81F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, 0.002F, 0.212F);
  assert(std::abs(rejected_innovation.pitch_rad - dynamic_output.pitch_rad) <
         0.003F);

  // A gravity-like but directionally invalid leg transient must not create a
  // false fall while the posture transaction owns the attitude gate.
  const auto pose_transient = dynamic_estimator.update(
      {2.35F, 0.0F, -10.25F}, {0.0F, 0.39F, 0.0F}, 0.02F, 0.23F,
      0.0F, true, false, {0.0F, 0.0F, true});
  assert(std::abs(pose_transient.pitch_rad - rejected_innovation.pitch_rad) <
         0.03F);

  return 0;
}
