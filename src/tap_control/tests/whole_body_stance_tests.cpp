#include "robot_control/WholeBodyStanceController.hpp"

#include <cmath>
#include <iostream>

using namespace robot_control;

namespace {
int failures = 0;

void check(bool condition, const char* name) {
  std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
  if (!condition) ++failures;
}

bool finiteTarget(const JointArray& target) {
  for (float value : target) {
    if (!std::isfinite(value)) return false;
  }
  return true;
}
}  // namespace

int main() {
  WholeBodyStanceController::Config config;
  config.enabled = true;
  config.hip_min_rad = geometry::native_hip_min_rad;
  config.hip_max_rad = geometry::native_hip_max_rad;
  config.knee_min_rad = geometry::native_knee_min_rad;
  config.knee_max_rad = geometry::native_knee_max_rad;
  WholeBodyStanceController controller(config);

  RobotState state;
  state.joints.position_rad = {
      geometry::neutral_hip_rad, geometry::neutral_knee_rad, 0.0F,
      geometry::neutral_hip_rad, geometry::neutral_knee_rad, 0.0F};
  const JointArray base = state.joints.position_rad;
  const auto natural = WholeBodyModel{}.compute(
      base[0], base[1], base[3], base[4]);
  const auto analytic = WholeBodyModel::symmetricComOffsetJacobian(
      base[0], base[1]);
  constexpr float epsilon = 1.0e-4F;
  const auto plus_hip = WholeBodyModel{}.compute(
      base[0] + epsilon, base[1], base[3] + epsilon, base[4]);
  const auto minus_hip = WholeBodyModel{}.compute(
      base[0] - epsilon, base[1], base[3] - epsilon, base[4]);
  const auto plus_knee = WholeBodyModel{}.compute(
      base[0], base[1] + epsilon, base[3], base[4] + epsilon);
  const auto minus_knee = WholeBodyModel{}.compute(
      base[0], base[1] - epsilon, base[3], base[4] - epsilon);
  check(std::abs(analytic.d_com_offset_x_d_hip -
                 (plus_hip.com_offset_x_m - minus_hip.com_offset_x_m) /
                     (2.0F * epsilon)) < 2.0e-4F &&
            std::abs(analytic.d_com_offset_x_d_knee -
                     (plus_knee.com_offset_x_m - minus_knee.com_offset_x_m) /
                         (2.0F * epsilon)) < 2.0e-4F,
        "analytic COM Jacobian matches independent finite difference");

  auto output = controller.update(state, base, natural, true, 0.002F);
  check(output.active && finiteTarget(output.target),
        "enabled stance output is finite");
  check(std::abs(output.target[0] - base[0]) < 1.0e-4F &&
            std::abs(output.target[1] - base[1]) < 1.0e-4F &&
            std::abs(output.target[3] - base[3]) < 1.0e-4F &&
            std::abs(output.target[4] - base[4]) < 1.0e-4F,
        "neutral pitch and symmetric legs start with only a tiny trim");

  // The default profile keeps the optional leg pitch task disabled. This is
  // the pure COM-task contract used by the core unit fixture.
  WholeBodyModel::Result pitch_centered_body = natural;
  pitch_centered_body.com_offset_x_m = 0.0F;
  pitch_centered_body.com_line_angle_rad = 0.0F;
  RobotState body_pitch_state = state;
  body_pitch_state.imu.pitch_rad = 0.10F;
  const auto body_pitch_output = controller.update(
      body_pitch_state, base, pitch_centered_body, true, 0.01F);
  check(std::abs(body_pitch_output.com_target_pitch_rad) < 1.0e-6F &&
            std::abs(body_pitch_output.com_error_m) < config.com_deadband_m,
        "disabled leg pitch task leaves COM target unchanged");

  WholeBodyStanceController::Config pitch_task_config = config;
  pitch_task_config.body_pitch_task_enabled = true;
  WholeBodyStanceController pitch_task(pitch_task_config);
  const auto pitch_task_output = pitch_task.update(
      body_pitch_state, base, pitch_centered_body, true, 0.01F,
      true);
  check(finiteTarget(pitch_task_output.target) &&
            std::abs(pitch_task_output.target[0] - base[0]) > 1.0e-6F,
        "enabled leg pitch task produces a bounded axle correction");

  // The leg speed loop is an explicit Drive owner. A velocity error alone
  // must not make a stationary height/station update move the legs.
  WholeBodyStanceController::Config speed_config = config;
  speed_config.speed_loop_tau_s = 0.50F;
  speed_config.speed_loop_filter_hz = 100.0F;
  speed_config.speed_loop_max_rate_rad_s = 10.0F;
  WholeBodyStanceController speed_controller(speed_config);
  const auto speed_disabled = speed_controller.update(
      state, base, natural, true, 0.01F, true, 0.20F);
  check(std::abs(speed_disabled.speed_loop_hip_rad) < 1.0e-7F &&
            std::abs(speed_disabled.speed_loop_com_m) < 1.0e-7F,
        "leg speed loop stays disabled without explicit Drive ownership");
  const auto speed_enabled = speed_controller.update(
      state, base, natural, true, 0.01F, true, 0.20F, true);
  check(std::abs(speed_enabled.speed_loop_hip_rad) > 1.0e-6F &&
            std::abs(speed_enabled.speed_loop_com_m) > 1.0e-6F,
        "leg speed loop engages only when explicitly owned");
  const auto speed_released = speed_controller.update(
      state, base, natural, true, 0.01F, true, 0.20F, false);
  check(std::abs(speed_released.speed_loop_hip_rad) <
            std::abs(speed_enabled.speed_loop_hip_rad),
        "leg speed correction releases smoothly after Drive ownership ends");

  // Regression for the original COM-loop identity: a measured equilibrium
  // pitch must not be fed back as the stance target. With a non-zero COM
  // offset, the stance error remains a real error even when IMU pitch matches
  // the FK equilibrium that a wheel controller might use as feed-forward.
  WholeBodyModel::Result offset_body = natural;
  offset_body.com_offset_x_m = 0.010F;
  offset_body.com_above_axle_m = 0.200F;
  const float measured_equilibrium =
      WholeBodyModel::equilibriumPitch(offset_body);
  controller.reset();
  state.imu.pitch_rad = measured_equilibrium;
  output = controller.update(state, base, offset_body, true, 0.002F);
  check(output.com_error_m < -0.0085F && output.com_error_m > -0.0105F,
        "measured equilibrium pitch does not cancel the zero-pitch COM task");

  state.imu.pitch_rad = 0.12F;
  output = controller.update(state, base, natural, true, 0.002F);
  check(finiteTarget(output.target) &&
            std::abs(output.target[0] - base[0]) < 0.001F &&
            std::abs(output.target[1] - base[1]) < 0.001F,
        "COM trim starts rate-limited");
  for (int i = 0; i < 500; ++i) {
    output = controller.update(state, base, natural, true, 0.002F);
  }
  check(std::abs(output.target[0] - base[0]) <= config.max_correction_rad + 1e-6F &&
            std::abs(output.target[1] - base[1]) <= config.max_correction_rad + 1e-6F,
        "COM trim stays inside bounded joint correction");

  controller.reset();
  state.imu.pitch_rad = 0.0F;
  state.joints.position_rad[3] += 0.06F;
  const auto asymmetric = WholeBodyModel{}.compute(
      state.joints.position_rad[0], state.joints.position_rad[1],
      state.joints.position_rad[3], state.joints.position_rad[4]);
  output = controller.update(state, base, asymmetric, true, 0.002F);
  check(finiteTarget(output.target) &&
            std::abs(output.target[0] - output.target[3]) < 1.0e-6F,
        "asymmetric geometry produces finite differential correction");
  check(std::abs(output.axle_height_error_m) >= 0.0F,
        "axle height error is exposed for supervision");

  output = controller.update(state, base, asymmetric, false, 0.002F);
  check(finiteTarget(output.target), "disabled gate remains finite");
  std::cout << (failures == 0 ? "PASS" : "FAIL")
            << " whole-body stance controller\n";
  return failures == 0 ? 0 : 1;
}
