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
      true, false);
  check(finiteTarget(pitch_task_output.target) &&
            std::abs(pitch_task_output.target[0] - base[0]) > 1.0e-6F,
        "enabled leg pitch task produces a bounded axle correction");

  WholeBodyStanceController::Config vmc_config = config;
  vmc_config.vmc_enabled = true;
  vmc_config.vmc_max_torque_nm = 0.8F;
  WholeBodyStanceController vmc(vmc_config);
  WholeBodyModel::Result vmc_body = natural;
  vmc_body.com_line_angle_rad = 0.05F;
  RobotState vmc_state = state;
  vmc_state.imu.pitch_rad = 0.20F;
  vmc_state.imu.pitch_rate_rad_s = 0.10F;
  const auto vmc_output = vmc.update(vmc_state, base, vmc_body, true, 0.002F);
  check(vmc_output.vmc_pitch_torque_nm < 0.0F &&
            std::abs(vmc_output.vmc_pitch_torque_nm) <=
                vmc_config.vmc_max_torque_nm + 1.0e-6F,
        "VMC spring-damper opposes pitch error within its safety ceiling");
  bool finite_vmc_torque = true;
  for (const auto joint : {0U, 1U, 3U, 4U}) {
    finite_vmc_torque = finite_vmc_torque &&
        std::isfinite(vmc_output.vmc_torque_nm[joint]) &&
        std::abs(vmc_output.vmc_torque_nm[joint]) <=
            vmc_config.vmc_max_torque_nm + 1.0e-6F;
  }
  check(finite_vmc_torque, "VMC J-transpose torque allocation stays bounded");
  const auto neutral_leg = WholeBodyModel::legHeight(base[0], base[1]);
  const float expected_vmc_scale = 0.5F *
      vmc_output.vmc_pitch_torque_nm / vmc_body.com_above_axle_m;
  check(std::abs(vmc_output.vmc_torque_nm[0] -
                     (-expected_vmc_scale * neutral_leg.d_x_d_hip)) < 1.0e-5F &&
            std::abs(vmc_output.vmc_torque_nm[1] -
                     (-expected_vmc_scale * neutral_leg.d_x_d_knee)) < 1.0e-5F &&
            std::abs(vmc_output.vmc_torque_nm[3] -
                     (-expected_vmc_scale * neutral_leg.d_x_d_hip)) < 1.0e-5F &&
            std::abs(vmc_output.vmc_torque_nm[4] -
                     (-expected_vmc_scale * neutral_leg.d_x_d_knee)) < 1.0e-5F,
        "stance VMC uses contact-to-body -J-transpose sign");

  // The slow static-trim gate may close during a pitch-rate transient, but
  // that must not remove active suspension authority from the same sample.
  const auto gated_vmc_output = vmc.update(
      vmc_state, base, vmc_body, false, 0.002F, false, true);
  check(gated_vmc_output.active &&
            gated_vmc_output.vmc_pitch_torque_nm < 0.0F,
        "VMC remains active while static stance trim is gated");

  // A zero VMC spring ratio must retain a real damping term. A joint motion
  // changes the COM-line rate while body pitch remains outside this loop.
  WholeBodyStanceController::Config damper_config = vmc_config;
  damper_config.vmc_stiffness_ratio = 0.0F;
  WholeBodyStanceController damper(damper_config);
  WholeBodyModel::Result centered_body = natural;
  centered_body.com_offset_x_m = 0.0F;
  centered_body.com_line_angle_rad = 0.0F;
  RobotState damper_state = state;
  damper_state.joints.velocity_rad_s[0] = 0.10F;
  damper_state.joints.velocity_rad_s[3] = 0.10F;
  damper_state.imu.pitch_rate_rad_s = 0.10F;
  const auto damper_output = damper.update(
      damper_state, base, centered_body, true, 0.002F);
  check(std::abs(damper_output.vmc_pitch_torque_nm) > 1.0e-4F,
        "zero VMC stiffness retains COM-line damping");

  // Crossing the independent VMC safety latch must release torque gradually,
  // not inject a one-sample step into the leg servo.
  WholeBodyStanceController guarded_vmc(vmc_config);
  const auto guarded_start = guarded_vmc.update(
      vmc_state, base, vmc_body, true, 0.002F);
  vmc_state.imu.pitch_rate_rad_s = 2.0F;
  auto guarded_end = guarded_start;
  for (int i = 0; i < 40; ++i) {
    guarded_end = guarded_vmc.update(
        vmc_state, base, vmc_body, true, 0.002F);
  }
  check(std::abs(guarded_start.vmc_pitch_torque_nm) >
            std::abs(guarded_end.vmc_pitch_torque_nm) + 1.0e-4F &&
            std::abs(guarded_end.vmc_pitch_torque_nm) < 1.0e-4F,
        "VMC safety latch ramps torque to zero");

  // The leg VMC owns the small supported body-pitch band. A centered COM line
  // therefore still produces a bounded body-pitch restoring torque.
  WholeBodyModel::Result line_balanced = natural;
  line_balanced.com_above_axle_m = 0.200F;
  line_balanced.com_offset_x_m = 0.0F;
  line_balanced.com_line_angle_rad = 0.0F;
  RobotState line_state = state;
  line_state.imu.pitch_rad = 0.05F;
  line_state.imu.pitch_rate_rad_s = 0.0F;
  WholeBodyStanceController line_vmc(vmc_config);
  const auto line_output = line_vmc.update(
      line_state, base, line_balanced, true, 0.002F);
  check(line_output.vmc_pitch_torque_nm < 0.0F,
        "VMC opposes body pitch when the COM line is centered");

  // The leg-side damper also opposes supported body-pitch rate.
  RobotState rate_state = line_state;
  rate_state.imu.pitch_rate_rad_s = 0.40F;
  WholeBodyStanceController rate_vmc(vmc_config);
  const auto rate_output = rate_vmc.update(
      rate_state, base, line_balanced, true, 0.002F);
  check(rate_output.vmc_pitch_torque_nm < line_output.vmc_pitch_torque_nm,
        "VMC damps body-pitch rate when COM-line rate is zero");

  line_balanced.com_offset_x_m = -0.010F;
  line_balanced.com_line_angle_rad = std::atan2(
      line_balanced.com_offset_x_m, line_balanced.com_above_axle_m);
  RobotState line_error_state = line_state;
  line_error_state.imu.pitch_rad = 0.0F;
  line_error_state.imu.pitch_rate_rad_s = 0.0F;
  const auto line_error_output = line_vmc.update(
      line_error_state, base, line_balanced, true, 0.002F);
  check(std::abs(line_error_output.vmc_pitch_torque_nm) < 1.0e-4F,
        "body-pitch VMC stays quiet when body pitch is centered");

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
