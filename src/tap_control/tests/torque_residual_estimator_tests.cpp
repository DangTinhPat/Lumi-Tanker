#include <cmath>
#include <array>
#include <iostream>
#include <limits>

#include "robot_control/GeneratedGeometry.hpp"
#include "robot_control/TorqueResidualEstimator.hpp"
#include "robot_control/WholeBodyModel.hpp"

using namespace robot_control;

namespace {
int failures = 0;

void check(bool condition, const char* name) {
  std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
  if (!condition) ++failures;
}

constexpr auto kLh = static_cast<std::size_t>(JointId::LeftHip);
constexpr auto kLk = static_cast<std::size_t>(JointId::LeftKnee);
constexpr auto kRh = static_cast<std::size_t>(JointId::RightHip);
constexpr auto kRk = static_cast<std::size_t>(JointId::RightKnee);

RobotState baseState() {
  RobotState state;
  state.dt_s = 0.01F;
  state.joints.position_rad[kLh] = 0.9F;
  state.joints.position_rad[kLk] = -1.3962634F;
  state.joints.position_rad[kRh] = 0.9F;
  state.joints.position_rad[kRk] = -1.3962634F;
  state.joints.effort_feedback_valid = true;
  return state;
}

// Independent distributed-link gravity calculation. It intentionally mirrors
// the CAD geometry contract rather than calling the estimator's private
// helper, so a sign or mass-allocation regression remains visible here.
struct Expected {
  float hip_nm;
  float knee_nm;
};

Expected expectedGravity(float hip_rad, float knee_rad, bool left) {
  (void)left;  // x/z CAD geometry is bilateral under native +Y axes.
  const auto rotate = [](float x, float z, float angle) {
    return std::array<float, 2>{std::cos(angle) * x + std::sin(angle) * z,
                                -std::sin(angle) * x + std::cos(angle) * z};
  };
  const auto dz = [&](float x, float z, float angle) {
    return -rotate(x, z, angle)[0];
  };
  Expected out{0.0F, 0.0F};
  const auto add = [&](float mass, float dhip, float dknee) {
    out.hip_nm -= dhip * (-mass * 9.81F);
    out.knee_nm -= dknee * (-mass * 9.81F);
  };
  const auto& knee_origin = geometry::origins[1];
  const auto& wheel_origin = geometry::origins[2];
  const auto& thigh = geometry::parts[1];
  const auto& shin = geometry::parts[2];
  const auto& wheel = geometry::parts[3];
  const float lower = hip_rad + knee_rad + geometry::knee_zero_rebase_rad;
  add(thigh.mass, dz(thigh.x, thigh.z, hip_rad), 0.0F);
  add(shin.mass,
      dz(knee_origin.x, knee_origin.z, hip_rad) +
          dz(shin.x, shin.z, lower),
      dz(shin.x, shin.z, lower));
  add(wheel.mass,
      dz(knee_origin.x, knee_origin.z, hip_rad) +
          dz(wheel_origin.x + wheel.x, wheel_origin.z + wheel.z, lower),
      dz(wheel_origin.x + wheel.x, wheel_origin.z + wheel.z, lower));
  return out;
}

void disabledConfigProducesInvalidZeroOutput() {
  TorqueResidualEstimator estimator;  // Config{}: enabled=false by default.
  auto state = baseState();
  state.joints.effort_nm[kLh] = 5.0F;  // Arbitrary nonzero, must be ignored.
  const auto out = estimator.update(state);
  check(!out.valid, "disabled estimator produces invalid output");
  check(out.left_hip_residual_nm == 0.0F && out.residual_confidence == 0.0F,
        "disabled estimator output is exactly zero, not partially computed");
}

void staticStanceResidualIsNearZero() {
  TorqueResidualEstimator::Config cfg;
  cfg.enabled = true;
  cfg.use_force_projection = false;
  TorqueResidualEstimator estimator(cfg);
  auto state = baseState();
  const auto left = expectedGravity(state.joints.position_rad[kLh],
                                    state.joints.position_rad[kLk], true);
  const auto right = expectedGravity(state.joints.position_rad[kRh],
                                     state.joints.position_rad[kRk], false);
  state.joints.effort_nm[kLh] = left.hip_nm;
  state.joints.effort_nm[kLk] = left.knee_nm;
  state.joints.effort_nm[kRh] = right.hip_nm;
  state.joints.effort_nm[kRk] = right.knee_nm;
  const auto out = estimator.update(state);
  check(out.valid, "enabled estimator with finite input is valid");
  check(std::fabs(out.left_hip_residual_nm) < 1.0e-3F &&
            std::fabs(out.left_knee_residual_nm) < 1.0e-3F &&
            std::fabs(out.right_hip_residual_nm) < 1.0e-3F &&
            std::fabs(out.right_knee_residual_nm) < 1.0e-3F,
        "measured torque equal to the model's own expected value gives ~zero residual");
}

void nonzeroMeasuredMinusExpectedProducesResidual() {
  TorqueResidualEstimator::Config cfg;
  cfg.enabled = true;
  cfg.use_force_projection = false;
  TorqueResidualEstimator estimator(cfg);
  auto state = baseState();
  const auto left = expectedGravity(state.joints.position_rad[kLh],
                                    state.joints.position_rad[kLk], true);
  constexpr float kDelta = 0.5F;
  state.joints.effort_nm[kLh] = left.hip_nm + kDelta;
  state.joints.effort_nm[kLk] = left.knee_nm;
  const auto out = estimator.update(state);
  check(std::fabs(out.left_hip_residual_nm - kDelta) < 1.0e-3F,
        "residual recovers a known offset from the expected torque model");
}

void unloadingDirectionRequiresBothLegsAgreement() {
  TorqueResidualEstimator::Config cfg;
  cfg.enabled = true;
  cfg.use_force_projection = false;
  cfg.residual_deadband_nm = 0.05F;
  TorqueResidualEstimator estimator(cfg);
  auto state = baseState();
  const auto left = expectedGravity(state.joints.position_rad[kLh],
                                    state.joints.position_rad[kLk], true);
  const auto right = expectedGravity(state.joints.position_rad[kRh],
                                     state.joints.position_rad[kRk], false);
  // Left leg reads well below expected (unloading-consistent); right leg
  // matches expected exactly (no evidence).
  state.joints.effort_nm[kLh] = left.hip_nm - 1.0F;
  state.joints.effort_nm[kLk] = left.knee_nm - 1.0F;
  state.joints.effort_nm[kRh] = right.hip_nm;
  state.joints.effort_nm[kRk] = right.knee_nm;
  const auto out = estimator.update(state);
  check(!out.direction_consistent_with_unloading,
        "one-sided unloading evidence alone does not satisfy both-legs agreement (Rule 2)");
}

void residualConfidenceDebouncesAndDecays() {
  TorqueResidualEstimator::Config cfg;
  cfg.enabled = true;
  cfg.use_force_projection = false;
  cfg.residual_deadband_nm = 0.05F;
  cfg.residual_debounce_s = 0.10F;
  cfg.residual_decay_rate = 2.0F;
  TorqueResidualEstimator estimator(cfg);
  auto state = baseState();
  state.dt_s = 0.01F;
  const auto left = expectedGravity(state.joints.position_rad[kLh],
                                    state.joints.position_rad[kLk], true);
  const auto right = expectedGravity(state.joints.position_rad[kRh],
                                     state.joints.position_rad[kRk], false);
  state.joints.effort_nm[kLh] = left.hip_nm - 1.0F;
  state.joints.effort_nm[kLk] = left.knee_nm - 1.0F;
  state.joints.effort_nm[kRh] = right.hip_nm - 1.0F;
  state.joints.effort_nm[kRk] = right.knee_nm - 1.0F;
  TorqueResidualEstimator::Output out;
  for (int i = 0; i < 10; ++i) out = estimator.update(state);
  check(out.direction_consistent_with_unloading &&
            out.residual_confidence >= 0.99F,
        "sustained both-legs disagreement ramps confidence to ~1 over the debounce window");

  // Now both legs match expected again; confidence must decay, not latch.
  state.joints.effort_nm[kLh] = left.hip_nm;
  state.joints.effort_nm[kLk] = left.knee_nm;
  state.joints.effort_nm[kRh] = right.hip_nm;
  state.joints.effort_nm[kRk] = right.knee_nm;
  out = estimator.update(state);
  const float after_one_decay_tick = out.residual_confidence;
  for (int i = 0; i < 20; ++i) out = estimator.update(state);
  check(out.residual_confidence < after_one_decay_tick,
        "confidence decays once disagreement evidence stops, never latches");
  check(out.residual_confidence <= 1.0e-3F,
        "confidence returns to ~0 after enough decay ticks");
}

void deadbandSuppressesSmallResidual() {
  TorqueResidualEstimator::Config cfg;
  cfg.enabled = true;
  cfg.use_force_projection = false;
  cfg.residual_deadband_nm = 0.5F;
  cfg.residual_debounce_s = 0.01F;
  TorqueResidualEstimator estimator(cfg);
  auto state = baseState();
  const auto left = expectedGravity(state.joints.position_rad[kLh],
                                    state.joints.position_rad[kLk], true);
  const auto right = expectedGravity(state.joints.position_rad[kRh],
                                     state.joints.position_rad[kRk], false);
  // Offset both legs by less than the deadband -- must never register.
  state.joints.effort_nm[kLh] = left.hip_nm - 0.1F;
  state.joints.effort_nm[kLk] = left.knee_nm - 0.1F;
  state.joints.effort_nm[kRh] = right.hip_nm - 0.1F;
  state.joints.effort_nm[kRk] = right.knee_nm - 0.1F;
  TorqueResidualEstimator::Output out;
  for (int i = 0; i < 20; ++i) out = estimator.update(state);
  check(!out.direction_consistent_with_unloading &&
            out.residual_confidence == 0.0F,
        "residual within the deadband never raises confidence -- exact no-op pre-tuning");
}

void nonFiniteJointDataProducesInvalidOutput() {
  TorqueResidualEstimator::Config cfg;
  cfg.enabled = true;
  cfg.use_force_projection = false;
  TorqueResidualEstimator estimator(cfg);
  auto state = baseState();
  state.joints.effort_nm[kLh] = std::numeric_limits<float>::quiet_NaN();
  const auto out = estimator.update(state);
  check(!out.valid, "non-finite joint effort produces an invalid output, never a fabricated residual");
}

void missingEffortFeedbackFailsClosed() {
  TorqueResidualEstimator::Config cfg;
  cfg.enabled = true;
  TorqueResidualEstimator estimator(cfg);
  auto state = baseState();
  state.joints.effort_feedback_valid = false;
  state.joints.effort_nm.fill(0.0F);
  const auto out = estimator.update(state);
  check(!out.valid && !out.direction_consistent_with_unloading &&
            !out.direction_consistent_with_impact,
        "missing GDS68 effort feedback cannot fabricate CMF evidence");
}

MotorCommand commandAtCurrentPose(const RobotState& state) {
  MotorCommand command;
  command.enable_motors = true;
  command.position_rad = state.joints.position_rad;
  command.velocity_rad_s.fill(0.0F);
  command.torque_nm.fill(50.0F);
  command.kp_scale.fill(1.0F);
  command.kd_scale.fill(1.0F);
  command.feedforward_nm.fill(0.0F);
  return command;
}

void forceProjectionRecoversExternalNormalForce() {
  TorqueResidualEstimator::Config cfg;
  cfg.enabled = true;
  cfg.commanded_kp_nm_rad = 40.0F;
  cfg.commanded_kd_nm_s_rad = 2.0F;
  cfg.unloading_force_threshold_n = 6.0F;
  cfg.impact_force_threshold_n = 30.0F;
  TorqueResidualEstimator estimator(cfg);
  auto state = baseState();
  const auto command = commandAtCurrentPose(state);
  estimator.setCommand(command);
  const auto left = expectedGravity(state.joints.position_rad[kLh],
                                    state.joints.position_rad[kLk], true);
  const auto right = expectedGravity(state.joints.position_rad[kRh],
                                     state.joints.position_rad[kRk], false);
  const auto left_leg = WholeBodyModel::legHeight(
      state.joints.position_rad[kLh], state.joints.position_rad[kLk]);
  const auto right_leg = WholeBodyModel::legHeight(
      state.joints.position_rad[kRh], state.joints.position_rad[kRk]);
  constexpr float kSupportForceN = 20.0F;
  state.joints.effort_nm[kLh] = left.hip_nm -
      left_leg.d_height_d_hip * kSupportForceN;
  state.joints.effort_nm[kLk] = left.knee_nm -
      left_leg.d_height_d_knee * kSupportForceN;
  state.joints.effort_nm[kRh] = right.hip_nm -
      right_leg.d_height_d_hip * kSupportForceN;
  state.joints.effort_nm[kRk] = right.knee_nm -
      right_leg.d_height_d_knee * kSupportForceN;
  const auto out = estimator.update(state);
  check(out.left_normal_force_valid && out.right_normal_force_valid,
        "Jacobian force projection is valid at the native landing pose");
  check(std::fabs(out.left_normal_force_n - kSupportForceN) < 1.0e-3F &&
            std::fabs(out.right_normal_force_n - kSupportForceN) < 1.0e-3F,
        "projected residual recovers the external normal force");
  check(!out.direction_consistent_with_unloading &&
            !out.direction_consistent_with_impact,
        "nominal support force is neither unloading nor impact");
}

void commandTorqueDoesNotCancelForceProjection() {
  TorqueResidualEstimator::Config cfg;
  cfg.enabled = true;
  cfg.commanded_kp_nm_rad = 40.0F;
  cfg.commanded_kd_nm_s_rad = 2.0F;
  TorqueResidualEstimator estimator(cfg);
  auto state = baseState();
  auto command = commandAtCurrentPose(state);
  command.position_rad[kLk] += 0.05F;
  command.position_rad[kRk] += 0.05F;
  estimator.setCommand(command);
  const auto left = expectedGravity(state.joints.position_rad[kLh],
                                    state.joints.position_rad[kLk], true);
  const auto right = expectedGravity(state.joints.position_rad[kRh],
                                     state.joints.position_rad[kRk], false);
  const auto left_leg = WholeBodyModel::legHeight(
      state.joints.position_rad[kLh], state.joints.position_rad[kLk]);
  const auto right_leg = WholeBodyModel::legHeight(
      state.joints.position_rad[kRh], state.joints.position_rad[kRk]);
  constexpr float kSupportForceN = 20.0F;
  // The command changes substantially, but the feedback sample still reports
  // the total actuator torque. CMF must not subtract that same request.
  state.joints.effort_nm[kLh] = left.hip_nm -
      left_leg.d_height_d_hip * kSupportForceN;
  state.joints.effort_nm[kLk] = left.knee_nm -
      left_leg.d_height_d_knee * kSupportForceN;
  state.joints.effort_nm[kRh] = right.hip_nm -
      right_leg.d_height_d_hip * kSupportForceN;
  state.joints.effort_nm[kRk] = right.knee_nm -
      right_leg.d_height_d_knee * kSupportForceN;
  const auto out = estimator.update(state);
  check(std::fabs(out.left_normal_force_n - kSupportForceN) < 1.0e-3F &&
            std::fabs(out.right_normal_force_n - kSupportForceN) < 1.0e-3F,
        "GDS68 MIT command changes do not cancel the estimated support force");
}

void supportedLegDoesNotLookUnloadedAndAirborneLegDoes() {
  TorqueResidualEstimator::Config cfg;
  cfg.enabled = true;
  cfg.unloading_force_threshold_n = 14.0F;
  cfg.residual_debounce_s = 0.02F;
  TorqueResidualEstimator estimator(cfg);
  auto state = baseState();
  estimator.setCommand(commandAtCurrentPose(state));
  const auto left = expectedGravity(state.joints.position_rad[kLh],
                                    state.joints.position_rad[kLk], true);
  const auto right = expectedGravity(state.joints.position_rad[kRh],
                                     state.joints.position_rad[kRk], false);
  const auto left_leg = WholeBodyModel::legHeight(
      state.joints.position_rad[kLh], state.joints.position_rad[kLk]);
  const auto right_leg = WholeBodyModel::legHeight(
      state.joints.position_rad[kRh], state.joints.position_rad[kRk]);
  const auto set_force = [&](float force_n) {
    state.joints.effort_nm[kLh] = left.hip_nm -
        left_leg.d_height_d_hip * force_n;
    state.joints.effort_nm[kLk] = left.knee_nm -
        left_leg.d_height_d_knee * force_n;
    state.joints.effort_nm[kRh] = right.hip_nm -
        right_leg.d_height_d_hip * force_n;
    state.joints.effort_nm[kRk] = right.knee_nm -
        right_leg.d_height_d_knee * force_n;
  };

  // A supported leg carries approximately 20 N in this synthetic fixture.
  // It must not be classified as unloading merely because the command model
  // and gravity model are different.
  set_force(20.0F);
  TorqueResidualEstimator::Output out;
  for (int i = 0; i < 10; ++i) out = estimator.update(state);
  check(out.left_normal_force_n > 19.0F && out.right_normal_force_n > 19.0F &&
            !out.direction_consistent_with_unloading,
        "supported 20 N/leg does not become unloading evidence");

  // Once the external support load disappears, the same estimator should
  // expose a low projected force after persistence.
  set_force(5.0F);
  for (int i = 0; i < 10; ++i) out = estimator.update(state);
  check(out.direction_consistent_with_unloading &&
            out.unloading_confidence >= 0.99F,
        "reduced airborne support load becomes unloading evidence");

  // During active leg motion the same torque magnitude is not observable as
  // contact force. CMF must invalidate the projection instead of turning an
  // airborne extension/braking torque into a touchdown/load claim.
  state.joints.velocity_rad_s[kLk] = 1.0F;
  state.joints.velocity_rad_s[kRk] = 1.0F;
  set_force(20.0F);
  out = estimator.update(state);
  check(!out.left_normal_force_valid && !out.right_normal_force_valid &&
            !out.direction_consistent_with_impact,
        "active leg motion invalidates torque-to-force projection");
}

void dynamicForceProjectionRemainsValidDuringFastLegMotion() {
  TorqueResidualEstimator::Config cfg;
  cfg.enabled = true;
  cfg.use_inertial_term = true;
  cfg.joint_accel_filter_time_constant_s = 1.0e-5F;
  cfg.max_force_projection_joint_speed_rad_s = 0.45F;
  cfg.unloading_force_threshold_n = 6.0F;
  TorqueResidualEstimator estimator(cfg);
  auto state = baseState();
  estimator.setCommand(commandAtCurrentPose(state));

  const auto left = expectedGravity(state.joints.position_rad[kLh],
                                    state.joints.position_rad[kLk], true);
  const auto right = expectedGravity(state.joints.position_rad[kRh],
                                     state.joints.position_rad[kRk], false);
  state.joints.effort_nm[kLh] = left.hip_nm;
  state.joints.effort_nm[kLk] = left.knee_nm;
  state.joints.effort_nm[kRh] = right.hip_nm;
  state.joints.effort_nm[kRk] = right.knee_nm;
  (void)estimator.update(state);

  // A fast joint interval is exactly where the former static gate rejected
  // the projection. Prime the acceleration model with a large but finite
  // velocity step, then apply a known external normal load on both legs.
  state.joints.velocity_rad_s[kLh] = 1.0F;
  state.joints.velocity_rad_s[kLk] = 1.0F;
  state.joints.velocity_rad_s[kRh] = 1.0F;
  state.joints.velocity_rad_s[kRk] = 1.0F;
  state.joints.effort_nm[kLh] = left.hip_nm;
  state.joints.effort_nm[kLk] = left.knee_nm;
  state.joints.effort_nm[kRh] = right.hip_nm;
  state.joints.effort_nm[kRk] = right.knee_nm;
  const auto primed = estimator.update(state);

  constexpr float kExternalForceN = 20.0F;
  // Keep the acceleration present in the next sample so the filtered
  // inverse-dynamics term remains aligned with the synthetic effort sample.
  state.joints.velocity_rad_s[kLh] = 2.0F;
  state.joints.velocity_rad_s[kLk] = 2.0F;
  state.joints.velocity_rad_s[kRh] = 2.0F;
  state.joints.velocity_rad_s[kRk] = 2.0F;
  const auto left_leg = WholeBodyModel::legHeight(
      state.joints.position_rad[kLh], state.joints.position_rad[kLk]);
  const auto right_leg = WholeBodyModel::legHeight(
      state.joints.position_rad[kRh], state.joints.position_rad[kRk]);
  state.joints.effort_nm[kLh] = primed.left_hip_expected_nm -
      left_leg.d_height_d_hip * kExternalForceN;
  state.joints.effort_nm[kLk] = primed.left_knee_expected_nm -
      left_leg.d_height_d_knee * kExternalForceN;
  state.joints.effort_nm[kRh] = primed.right_hip_expected_nm -
      right_leg.d_height_d_hip * kExternalForceN;
  state.joints.effort_nm[kRk] = primed.right_knee_expected_nm -
      right_leg.d_height_d_knee * kExternalForceN;
  const auto out = estimator.update(state);
  check(out.left_normal_force_valid && out.right_normal_force_valid,
        "dynamic torque projection remains valid above the static speed gate");
  check(std::fabs(out.left_normal_force_n - kExternalForceN) < 0.5F &&
            std::fabs(out.right_normal_force_n - kExternalForceN) < 0.5F,
        "dynamic torque projection removes inertial torque before estimating load");
}

void SupportedLoadNeedsShockForImpactEvidence() {
  TorqueResidualEstimator::Config cfg;
  cfg.enabled = true;
  cfg.impact_force_threshold_n = 30.0F;
  cfg.impact_accel_gate_m_s2 = 18.0F;
  TorqueResidualEstimator estimator(cfg);
  auto state = baseState();
  estimator.setCommand(commandAtCurrentPose(state));
  const auto left = expectedGravity(state.joints.position_rad[kLh],
                                    state.joints.position_rad[kLk], true);
  const auto right = expectedGravity(state.joints.position_rad[kRh],
                                     state.joints.position_rad[kRk], false);
  const auto left_leg = WholeBodyModel::legHeight(
      state.joints.position_rad[kLh], state.joints.position_rad[kLk]);
  const auto right_leg = WholeBodyModel::legHeight(
      state.joints.position_rad[kRh], state.joints.position_rad[kRk]);
  constexpr float kSupportedForceN = 35.0F;
  state.joints.effort_nm[kLh] = left.hip_nm -
      left_leg.d_height_d_hip * kSupportedForceN;
  state.joints.effort_nm[kLk] = left.knee_nm -
      left_leg.d_height_d_knee * kSupportedForceN;
  state.joints.effort_nm[kRh] = right.hip_nm -
      right_leg.d_height_d_hip * kSupportedForceN;
  state.joints.effort_nm[kRk] = right.knee_nm -
      right_leg.d_height_d_knee * kSupportedForceN;

  auto out = estimator.update(state);
  check(!out.direction_consistent_with_impact,
        "35 N supported load at stationary 1g is not impact evidence");
  state.imu.accel_magnitude_m_s2 = 30.0F;
  out = estimator.update(state);
  check(out.direction_consistent_with_impact,
        "supported-load torque becomes impact evidence only with an IMU shock");
}

void negativeProjectedForceIsNotUnloading() {
  TorqueResidualEstimator::Config cfg;
  cfg.enabled = true;
  cfg.unloading_force_threshold_n = 14.0F;
  TorqueResidualEstimator estimator(cfg);
  auto state = baseState();
  estimator.setCommand(commandAtCurrentPose(state));
  const auto left = expectedGravity(state.joints.position_rad[kLh],
                                    state.joints.position_rad[kLk], true);
  const auto right = expectedGravity(state.joints.position_rad[kRh],
                                     state.joints.position_rad[kRk], false);
  const auto left_leg = WholeBodyModel::legHeight(
      state.joints.position_rad[kLh], state.joints.position_rad[kLk]);
  const auto right_leg = WholeBodyModel::legHeight(
      state.joints.position_rad[kRh], state.joints.position_rad[kRk]);
  constexpr float kNegativeForceN = -20.0F;
  state.joints.effort_nm[kLh] = left.hip_nm -
      left_leg.d_height_d_hip * kNegativeForceN;
  state.joints.effort_nm[kLk] = left.knee_nm -
      left_leg.d_height_d_knee * kNegativeForceN;
  state.joints.effort_nm[kRh] = right.hip_nm -
      right_leg.d_height_d_hip * kNegativeForceN;
  state.joints.effort_nm[kRk] = right.knee_nm -
      right_leg.d_height_d_knee * kNegativeForceN;
  const auto out = estimator.update(state);
  check(out.left_normal_force_n < 0.0F && out.right_normal_force_n < 0.0F &&
            !out.direction_consistent_with_unloading,
        "negative projected force is invalid unloading evidence");
}

}  // namespace

int main() {
  disabledConfigProducesInvalidZeroOutput();
  staticStanceResidualIsNearZero();
  nonzeroMeasuredMinusExpectedProducesResidual();
  unloadingDirectionRequiresBothLegsAgreement();
  residualConfidenceDebouncesAndDecays();
  deadbandSuppressesSmallResidual();
  nonFiniteJointDataProducesInvalidOutput();
  missingEffortFeedbackFailsClosed();
  forceProjectionRecoversExternalNormalForce();
  commandTorqueDoesNotCancelForceProjection();
  supportedLegDoesNotLookUnloadedAndAirborneLegDoes();
  dynamicForceProjectionRemainsValidDuringFastLegMotion();
  SupportedLoadNeedsShockForImpactEvidence();
  negativeProjectedForceIsNotUnloading();
  return failures == 0 ? 0 : 1;
}
