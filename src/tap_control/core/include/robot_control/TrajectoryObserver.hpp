#pragma once

#include "robot_control/Types.hpp"

namespace robot_control {

// Short-horizon, sensor-only trajectory observer.  It observes the robot's
// relative planar motion from the two wheel encoders and uses the MPU6500
// gyro/accelerometer only as consistency signals.  It is deliberately not a
// pose source: absolute x/y, terrain height and contact are not observable
// from this hardware contract alone.
class TrajectoryObserver {
 public:
  struct Config {
    float wheel_radius_m = 0.030F;
    float half_track_m = 0.1545F;
    float max_dt_s = 0.05F;
    float max_wheel_speed_m_s = 2.0F;
    float gyro_consistency_scale_rad_s = 1.5F;
  };

  struct Output {
    // Robot-local displacement from the last reset, +x in the forward wheel
    // direction and +y to the left.  These are relative estimates only.
    float forward_position_m = 0.0F;
    float lateral_position_m = 0.0F;
    float yaw_rad = 0.0F;
    float forward_velocity_m_s = 0.0F;
    float lateral_velocity_m_s = 0.0F;
    float yaw_rate_rad_s = 0.0F;
    // 0 means invalid/uninitialised, 1 means encoder motion and IMU checks
    // agree.  This is signal quality, not a ground-contact probability.
    float confidence = 0.0F;
    bool valid = false;
    bool stationary = true;
  };

  TrajectoryObserver() = default;
  explicit TrajectoryObserver(Config config) : config_(config) {}

  void configure(Config config) noexcept { config_ = config; }
  void reset() noexcept;
  void reset(const RobotState& state) noexcept;
  [[nodiscard]] Output update(const RobotState& state) noexcept;
  [[nodiscard]] const Output& output() const noexcept { return output_; }

 private:
  [[nodiscard]] bool finiteInput(const RobotState& state) const noexcept;
  [[nodiscard]] static float clamp01(float value) noexcept;

  Config config_{};
  Output output_{};
  bool seeded_ = false;
  float previous_left_position_m_ = 0.0F;
  float previous_right_position_m_ = 0.0F;
};

}  // namespace robot_control
