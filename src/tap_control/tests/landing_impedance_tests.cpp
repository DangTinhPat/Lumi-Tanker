#include <cmath>
#include <iostream>

#include "robot_control/EmbeddedController.hpp"
#include "robot_control/LandingImpedance.hpp"
#include "robot_control/WholeBodyModel.hpp"

using namespace robot_control;
namespace {
int failures = 0;
void check(bool condition, const char* name) {
  std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
  if (!condition) ++failures;
}
bool near_equal(float a, float b, float eps = 1e-4F) {
  return std::abs(a - b) < eps;
}

void noOpAtDefaults() {
  LandingImpedance imp;  // Config{} - impact_kp/kd_scale == 1, velocity gate huge.
  const float hip = geometry::neutral_hip_rad, knee = 0.0F;
  bool ok = true;
  // Grounded, then a real liftoff/touchdown with a large, realistic
  // compression velocity - still must never engage Impact at defaults,
  // since the velocity threshold is impossibly high.
  (void)imp.update(true, hip, knee, hip, knee, 0.0F, 0.0F, 0.0F, 0.0F, 0.001F);
  for (float t = 0.0F; t < 0.2F; t += 0.001F) {
    const auto out =
        imp.update(false, hip, knee, hip, knee, 0.3F, -0.6F, 0.3F, -0.6F, 0.001F);
    ok = ok && out.leg_kp_scale == 1.0F && out.leg_kd_scale == 1.0F;
  }
  for (int i = 0; i < 500; ++i) {
    const auto out =
        imp.update(true, hip, knee, hip, knee, 0.3F, -0.6F, 0.3F, -0.6F, 0.001F);
    ok = ok && out.leg_kp_scale == 1.0F && out.leg_kd_scale == 1.0F;
  }
  check(ok, "LandingImpedance: default config is a no-op through a full "
            "liftoff/touchdown/settle sequence, even with real motion");
}

void legHeightJacobianMatchesFiniteDifference() {
  bool ok = true;
  constexpr float h = 1.0e-4F;
  for (float hip = -0.2F; hip <= 1.0F; hip += 0.2F) {
    for (float knee = -1.2F; knee <= 0.0F; knee += 0.2F) {
      const auto center = WholeBodyModel::legHeight(hip, knee);
      const float dh_hip =
          (WholeBodyModel::legHeight(hip + h, knee).height_m -
           WholeBodyModel::legHeight(hip - h, knee).height_m) /
          (2.0F * h);
      const float dh_knee =
          (WholeBodyModel::legHeight(hip, knee + h).height_m -
           WholeBodyModel::legHeight(hip, knee - h).height_m) /
          (2.0F * h);
      const float dx_hip =
          (WholeBodyModel::legHeight(hip + h, knee).x_m -
           WholeBodyModel::legHeight(hip - h, knee).x_m) /
          (2.0F * h);
      const float dx_knee =
          (WholeBodyModel::legHeight(hip, knee + h).x_m -
           WholeBodyModel::legHeight(hip, knee - h).x_m) /
          (2.0F * h);
      ok = ok && near_equal(center.d_height_d_hip, dh_hip, 1.0e-3F);
      ok = ok && near_equal(center.d_height_d_knee, dh_knee, 1.0e-3F);
      ok = ok && near_equal(center.d_x_d_hip, dx_hip, 1.0e-3F);
      ok = ok && near_equal(center.d_x_d_knee, dx_knee, 1.0e-3F);
    }
  }
  check(ok, "WholeBodyModel::legHeight: analytic height AND x gradients "
            "match central finite differences across the joint envelope");
}

void engagesOnlyOnGenuineImpactVelocity() {
  LandingImpedance::Config cfg;
  cfg.impact_kp_scale = 0.15F;
  cfg.impact_kd_scale = 3.0F;
  cfg.touchdown_velocity_threshold_m_s = 0.05F;
  cfg.min_airborne_before_touchdown_s = 0.1F;
  LandingImpedance imp(cfg);
  const float hip = geometry::neutral_hip_rad, knee = 0.0F;

  // Liftoff, real flight, then a GENTLE touchdown (near-zero compression
  // velocity) - must NOT engage Impact.
  (void)imp.update(true, hip, knee, hip, knee, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F);
  for (float t = 0.0F; t < 0.15F; t += 0.01F) {
    (void)imp.update(false, hip, knee, hip, knee, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F);
  }
  const auto gentle =
      imp.update(true, hip, knee, hip, knee, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F);
  check(!imp.active() && gentle.leg_kp_scale == 1.0F,
        "LandingImpedance: a gentle touchdown (near-zero compression "
        "velocity) does not engage the damper");

  imp.reset();
  // Same sequence, but now with a real compression velocity at the
  // touchdown instant (knee flexing fast, as it would under a genuine
  // impact) - MUST engage Impact.
  (void)imp.update(true, hip, knee, hip, knee, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F);
  for (float t = 0.0F; t < 0.15F; t += 0.01F) {
    (void)imp.update(false, hip, knee, hip, knee, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F);
  }
  const auto hard =
      imp.update(true, hip, knee, hip, knee, 0.0F, -2.0F, 0.0F, -2.0F, 0.01F);
  check(imp.active() && near_equal(hard.leg_kp_scale, cfg.impact_kp_scale) &&
            near_equal(hard.leg_kd_scale, cfg.impact_kd_scale),
        "LandingImpedance: a genuine impact (large compression velocity) "
        "engages the scheduled soft-Kp/high-Kd Impact scale");
}

void reboundsAndRecoversToGroundedAndInert() {
  LandingImpedance::Config cfg;
  cfg.impact_kp_scale = 0.2F;
  cfg.impact_kd_scale = 3.0F;
  cfg.touchdown_velocity_threshold_m_s = 0.05F;
  cfg.min_airborne_before_touchdown_s = 0.1F;
  cfg.min_impact_before_rebound_s = 0.0F;
  cfg.rebound_blend_s = 0.1F;
  LandingImpedance imp(cfg);
  const float hip = geometry::neutral_hip_rad, knee = 0.0F;
  (void)imp.update(true, hip, knee, hip, knee, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F);
  for (float t = 0.0F; t < 0.15F; t += 0.01F) {
    (void)imp.update(false, hip, knee, hip, knee, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F);
  }
  const auto touchdown =
      imp.update(true, hip, knee, hip, knee, 0.0F, -2.0F, 0.0F, -2.0F, 0.01F);
  check(imp.active() && near_equal(touchdown.leg_kp_scale, cfg.impact_kp_scale),
        "LandingImpedance: Impact engaged after a genuine touchdown");

  // Velocity settles to 0 (leg stopped moving, bottomed out): with
  // min_impact_before_rebound_s == 0, the very next tick transitions to
  // Rebound.
  (void)imp.update(true, hip, knee, hip, knee, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F);
  // Run out the rebound blend.
  LandingImpedance::Output settled{};
  for (int i = 0; i < 50; ++i) {
    settled =
        imp.update(true, hip, knee, hip, knee, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F);
  }
  check(near_equal(settled.leg_kp_scale, 1.0F) &&
            near_equal(settled.leg_kd_scale, 1.0F) && !imp.active(),
        "LandingImpedance: fully recovers to kp/kd scale 1.0 and Grounded "
        "well after touchdown");
}

void embeddedControllerNoOpAtDefaults() {
  EmbeddedController controller;
  RobotState state;
  state.dt_s = 0.001F;
  state.joints.position_rad = {geometry::neutral_hip_rad, 0.0F, 0.0F,
                               geometry::neutral_hip_rad, 0.0F, 0.0F};
  ControlCommand command;
  command.target_forward_velocity_m_s = 0.2F;
  command.target_joint_position_rad = state.joints.position_rad;

  controller.reset(state);
  command.request_enable = true;
  MotorCommand baseline{};
  for (int i = 0; i < 200; ++i) {
    state.floor_contact = true;
    baseline = controller.update(state, command);
  }
  check(controller.mode() == ControlMode::Balance ||
        controller.mode() == ControlMode::Drive,
        "EmbeddedController: reaches Balance/Drive before the LandingImpedance "
        "no-op comparison");

  EmbeddedController controller_b;
  controller_b.reset(state);
  MotorCommand toggled{};
  for (int i = 0; i < 200; ++i) {
    // Toggle floor_contact and knee velocity every cycle - with default
    // LandingImpedance config this must have ZERO effect on the commanded
    // position, kp/kd scale or feedforward.
    state.floor_contact = (i % 2 == 0);
    state.joints.velocity_rad_s[static_cast<std::size_t>(JointId::LeftKnee)] =
        (i % 2 == 0) ? 0.5F : -0.5F;
    toggled = controller_b.update(state, command);
  }
  bool same = baseline.enable_motors == toggled.enable_motors;
  for (std::size_t i = 0; i < kJointCount && same; ++i) {
    same = same && baseline.position_rad[i] == toggled.position_rad[i] &&
           baseline.kp_scale[i] == toggled.kp_scale[i] &&
           baseline.kd_scale[i] == toggled.kd_scale[i] &&
           baseline.feedforward_nm[i] == toggled.feedforward_nm[i];
  }
  check(same,
        "EmbeddedController: default LandingImpedance config is a no-op "
        "even under a toggling floor_contact/knee-velocity signal");
}

void embeddedControllerAppliesScaleToKneeOnlyNotHip() {
  EmbeddedController::Config cfg;
  cfg.landing_impedance.impact_kp_scale = 0.15F;
  cfg.landing_impedance.impact_kd_scale = 3.0F;
  cfg.landing_impedance.touchdown_velocity_threshold_m_s = 0.05F;
  cfg.landing_impedance.min_airborne_before_touchdown_s = 0.05F;
  EmbeddedController controller(cfg);
  RobotState state;
  state.dt_s = 0.005F;
  state.joints.position_rad = {geometry::neutral_hip_rad, 0.0F, 0.0F,
                               geometry::neutral_hip_rad, 0.0F, 0.0F};
  ControlCommand command;
  command.target_joint_position_rad = state.joints.position_rad;
  command.request_enable = true;

  controller.reset(state);
  MotorCommand out{};
  for (int i = 0; i < 200; ++i) {
    state.floor_contact = true;
    out = controller.update(state, command);
  }
  for (int i = 0; i < 20; ++i) {
    state.floor_contact = false;
    out = controller.update(state, command);
  }
  // Touchdown with a real compression velocity on the knee.
  state.joints.velocity_rad_s[static_cast<std::size_t>(JointId::LeftKnee)] = -2.0F;
  state.joints.velocity_rad_s[static_cast<std::size_t>(JointId::RightKnee)] = -2.0F;
  state.floor_contact = true;
  out = controller.update(state, command);

  constexpr auto lh = static_cast<std::size_t>(JointId::LeftHip);
  constexpr auto rh = static_cast<std::size_t>(JointId::RightHip);
  constexpr auto lk = static_cast<std::size_t>(JointId::LeftKnee);
  constexpr auto rk = static_cast<std::size_t>(JointId::RightKnee);
  check(out.kp_scale[lh] == 1.0F && out.kp_scale[rh] == 1.0F,
        "EmbeddedController: hip kp scale stays exactly 1.0 - every "
        "mechanism in this file's history that touched the hip measured "
        "worse on the real ledge test, so this component never does");
  check(out.kd_scale[lh] == 1.0F && out.kd_scale[rh] == 1.0F,
        "EmbeddedController: hip kd scale stays exactly 1.0");
  check(out.kp_scale[lk] < 0.9F && out.kp_scale[rk] < 0.9F,
        "EmbeddedController: the softened Kp scale DOES apply to the knee");
  check(out.kd_scale[lk] > 1.1F && out.kd_scale[rk] > 1.1F,
        "EmbeddedController: the raised Kd scale DOES apply to the knee");
  check(out.position_rad[lh] == command.target_joint_position_rad[lh] &&
        out.position_rad[lk] == command.target_joint_position_rad[lk],
        "EmbeddedController: LandingImpedance never changes the commanded "
        "position target itself, only the gains around it");
}
}  // namespace

int main() {
  noOpAtDefaults();
  legHeightJacobianMatchesFiniteDifference();
  engagesOnlyOnGenuineImpactVelocity();
  reboundsAndRecoversToGroundedAndInert();
  embeddedControllerNoOpAtDefaults();
  embeddedControllerAppliesScaleToKneeOnlyNotHip();
  std::cout << (failures == 0 ? "ALL PASS\n" : "FAILURES PRESENT\n");
  return failures == 0 ? 0 : 1;
}
