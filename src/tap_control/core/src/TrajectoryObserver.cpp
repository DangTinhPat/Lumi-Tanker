#include "robot_control/TrajectoryObserver.hpp"

#include <algorithm>
#include <cmath>

namespace robot_control {

void TrajectoryObserver::reset() noexcept {
  output_ = {};
  output_.stationary = true;
  seeded_ = false;
  previous_left_position_m_ = 0.0F;
  previous_right_position_m_ = 0.0F;
}

void TrajectoryObserver::reset(const RobotState& state) noexcept {
  reset();
  if (std::isfinite(state.wheel.left_position_m) &&
      std::isfinite(state.wheel.right_position_m)) {
    previous_left_position_m_ = state.wheel.left_position_m;
    previous_right_position_m_ = state.wheel.right_position_m;
    seeded_ = true;
  }
}

float TrajectoryObserver::clamp01(float value) noexcept {
  return std::clamp(value, 0.0F, 1.0F);
}

bool TrajectoryObserver::finiteInput(const RobotState& state) const noexcept {
  return std::isfinite(state.dt_s) && state.dt_s > 0.0F &&
         std::isfinite(state.wheel.left_position_m) &&
         std::isfinite(state.wheel.right_position_m) &&
         std::isfinite(state.wheel.left_velocity_m_s) &&
         std::isfinite(state.wheel.right_velocity_m_s) &&
         std::isfinite(state.imu.yaw_rate_rad_s) &&
         std::isfinite(state.imu.accel_magnitude_m_s2);
}

TrajectoryObserver::Output TrajectoryObserver::update(
    const RobotState& state) noexcept {
  if (!finiteInput(state)) {
    output_.valid = false;
    output_.confidence = 0.0F;
    return output_;
  }

  const float dt = std::clamp(state.dt_s, 1.0e-5F,
                              std::max(config_.max_dt_s, 1.0e-5F));
  const float left = state.wheel.left_position_m;
  const float right = state.wheel.right_position_m;
  if (!seeded_) {
    previous_left_position_m_ = left;
    previous_right_position_m_ = right;
    seeded_ = true;
    output_.valid = false;
    output_.confidence = 0.0F;
    return output_;
  }

  const float delta_left = left - previous_left_position_m_;
  const float delta_right = right - previous_right_position_m_;
  previous_left_position_m_ = left;
  previous_right_position_m_ = right;

  const float left_velocity = delta_left / dt;
  const float right_velocity = delta_right / dt;
  const float max_speed = std::max(config_.max_wheel_speed_m_s, 0.01F);
  // A position discontinuity is much more likely to be an encoder reset or a
  // timestamp gap than robot motion.  Do not turn that into a false jump in
  // the relative trajectory; the next sample can seed a new increment.
  if (std::abs(left_velocity) > max_speed ||
      std::abs(right_velocity) > max_speed) {
    output_.valid = false;
    output_.confidence = 0.0F;
    output_.forward_velocity_m_s = 0.0F;
    output_.lateral_velocity_m_s = 0.0F;
    output_.yaw_rate_rad_s = 0.0F;
    return output_;
  }

  const float forward_delta = 0.5F * (delta_left + delta_right);
  const float yaw_delta = (delta_right - delta_left) /
                          std::max(2.0F * config_.half_track_m, 1.0e-4F);
  // Midpoint integration keeps the short local trajectory well behaved during
  // a turn without importing a world-frame pose or heading source.
  const float yaw_mid = output_.yaw_rad + 0.5F * yaw_delta;
  output_.forward_position_m += forward_delta * std::cos(yaw_mid);
  output_.lateral_position_m += forward_delta * std::sin(yaw_mid);
  output_.yaw_rad += yaw_delta;

  const float reported_forward = 0.5F *
      (state.wheel.left_velocity_m_s + state.wheel.right_velocity_m_s);
  const float reported_yaw_rate =
      (state.wheel.right_velocity_m_s - state.wheel.left_velocity_m_s) /
      std::max(2.0F * config_.half_track_m, 1.0e-4F);
  const float delta_forward_velocity = 0.5F * (left_velocity + right_velocity);
  const float delta_yaw_rate = yaw_delta / dt;
  // Use the encoder-reported instantaneous rates for the output.  The control
  // timer and joint-state callback can have different periods; dividing a
  // position increment by control dt would then create a false 2x (or larger)
  // velocity spike.  The position increment remains the correct odometry
  // quantity for the short-horizon trajectory.
  output_.forward_velocity_m_s = reported_forward;
  output_.lateral_velocity_m_s = 0.0F;
  output_.yaw_rate_rad_s = reported_yaw_rate;
  const float encoder_velocity_error =
      std::abs(delta_forward_velocity - reported_forward);
  const float encoder_yaw_error =
      std::abs(delta_yaw_rate - reported_yaw_rate);
  const float gyro_error =
      std::abs(output_.yaw_rate_rad_s - state.imu.yaw_rate_rad_s);
  const float encoder_quality = clamp01(1.0F - encoder_velocity_error /
                                                 std::max(max_speed, 0.1F));
  const float yaw_quality = clamp01(1.0F - std::max(encoder_yaw_error, gyro_error) /
                                             std::max(config_.gyro_consistency_scale_rad_s,
                                                      0.1F));
  // Specific-force magnitude is only a coarse sanity check.  During real
  // acceleration it can leave 1 g, so it is deliberately given low weight.
  const float accel_quality = clamp01(1.0F -
      std::abs(state.imu.accel_magnitude_m_s2 - 9.81F) / 9.81F);
  output_.confidence = clamp01(0.60F * encoder_quality +
                               0.25F * yaw_quality + 0.15F * accel_quality);
  output_.valid = true;
  output_.stationary = std::abs(output_.forward_velocity_m_s) < 0.01F &&
                       std::abs(output_.yaw_rate_rad_s) < 0.05F;
  return output_;
}

}  // namespace robot_control
