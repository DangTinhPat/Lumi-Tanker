#include "robot_control/TorqueResidualEstimator.hpp"

#include <algorithm>
#include <cmath>

#include "robot_control/WholeBodyModel.hpp"

namespace robot_control {
namespace {

float clamp01(float value) {
  return std::clamp(value, 0.0F, 1.0F);
}

// Gravity torque of the leg links distal to the hip/knee. The MIT feedback
// field is an estimate of the total output torque produced by the actuator.
// It is compared with free-flight/passive torque, not with the same Kp/Kd/Tff
// command that caused the actuator to produce that torque. Subtracting that
// command would remove the external-load signal we are trying to observe.
struct ExpectedTorque {
  float hip_nm = 0.0F;
  float knee_nm = 0.0F;
};

ExpectedTorque expectedGravityTorque(float hip_rad, float knee_rad,
                                     float gravity_m_s2) {
  const auto torque = WholeBodyModel::legGravityTorque(
      hip_rad, knee_rad, gravity_m_s2);
  return {torque.hip_nm, torque.knee_nm};
}

}  // namespace

void TorqueResidualEstimator::reset() {
  previous_command_ = Command{};
  unloading_elapsed_s_ = 0.0F;
  impact_elapsed_s_ = 0.0F;
  previous_valid_ = false;
  previous_velocity_rad_s_ = JointArray{};
  filtered_accel_rad_s2_ = JointArray{};
}

void TorqueResidualEstimator::setCommand(const MotorCommand& command) noexcept {
  previous_command_.valid = true;
  previous_command_.enable_motors = command.enable_motors;
  previous_command_.position_rad = command.position_rad;
  previous_command_.velocity_rad_s = command.velocity_rad_s;
  previous_command_.torque_limit_nm = command.torque_nm;
  previous_command_.kp_scale = command.kp_scale;
  previous_command_.kd_scale = command.kd_scale;
  previous_command_.feedforward_nm = command.feedforward_nm;
}

bool TorqueResidualEstimator::finiteInput(const RobotState& state) const
    noexcept {
  if (!(std::isfinite(state.dt_s) && state.dt_s > 0.0F)) return false;
  if (!state.joints.effort_feedback_valid) return false;
  for (const auto joint : {JointId::LeftHip, JointId::LeftKnee,
                           JointId::RightHip, JointId::RightKnee}) {
    const auto index = static_cast<std::size_t>(joint);
    if (!std::isfinite(state.joints.position_rad[index]) ||
        !std::isfinite(state.joints.velocity_rad_s[index]) ||
        !std::isfinite(state.joints.effort_nm[index])) {
      return false;
    }
  }
  return true;
}

TorqueResidualEstimator::Output TorqueResidualEstimator::update(
    const RobotState& state) {
  Output output;
  if (!config_.enabled || !finiteInput(state)) {
    unloading_elapsed_s_ = 0.0F;
    impact_elapsed_s_ = 0.0F;
    previous_valid_ = false;
    return output;
  }

  const float dt = std::clamp(state.dt_s, 1.0e-5F, 0.05F);
  const auto lh = static_cast<std::size_t>(JointId::LeftHip);
  const auto lk = static_cast<std::size_t>(JointId::LeftKnee);
  const auto rh = static_cast<std::size_t>(JointId::RightHip);
  const auto rk = static_cast<std::size_t>(JointId::RightKnee);

  const auto left_expected = expectedGravityTorque(
      state.joints.position_rad[lh], state.joints.position_rad[lk],
      config_.gravity_m_s2);
  const auto right_expected = expectedGravityTorque(
      state.joints.position_rad[rh], state.joints.position_rad[rk],
      config_.gravity_m_s2);

  float left_hip_expected = left_expected.hip_nm;
  float left_knee_expected = left_expected.knee_nm;
  float right_hip_expected = right_expected.hip_nm;
  float right_knee_expected = right_expected.knee_nm;

  // Do not subtract previous_command_ here. In GDS68 MIT mode the feedback
  // torque is the actuator's total Iq-derived output torque, while the
  // command is only the request that generated it. Removing that request
  // would make a statically supported leg look unloaded. The command is
  // retained only for asynchronous validity and future diagnostics.

  if (config_.use_inertial_term) {
    // Low-pass-filtered finite difference of encoder velocity. Raw
    // double-differentiation of encoder velocity is a well-known noise
    // amplifier; this term is off by default (Config::use_inertial_term)
    // until validated.
    const float tau = std::max(config_.joint_accel_filter_time_constant_s,
                               1.0e-4F);
    const float alpha = dt / (tau + dt);
    for (const auto index : {lh, lk, rh, rk}) {
      const float raw_accel = previous_valid_
          ? (state.joints.velocity_rad_s[index] -
             previous_velocity_rad_s_[index]) / dt
          : 0.0F;
      filtered_accel_rad_s2_[index] +=
          alpha * (raw_accel - filtered_accel_rad_s2_[index]);
    }
    left_hip_expected += geometry::leg_hip_inertia_kgm2 *
        filtered_accel_rad_s2_[lh];
    left_knee_expected += geometry::leg_knee_inertia_kgm2 *
        filtered_accel_rad_s2_[lk];
    right_hip_expected += geometry::leg_hip_inertia_kgm2 *
        filtered_accel_rad_s2_[rh];
    right_knee_expected += geometry::leg_knee_inertia_kgm2 *
        filtered_accel_rad_s2_[rk];
  }
  previous_velocity_rad_s_ = state.joints.velocity_rad_s;
  previous_valid_ = true;

  output.valid = true;
  output.left_hip_expected_nm = left_hip_expected;
  output.left_knee_expected_nm = left_knee_expected;
  output.right_hip_expected_nm = right_hip_expected;
  output.right_knee_expected_nm = right_knee_expected;
  output.left_hip_residual_nm =
      state.joints.effort_nm[lh] - left_hip_expected;
  output.left_knee_residual_nm =
      state.joints.effort_nm[lk] - left_knee_expected;
  output.right_hip_residual_nm =
      state.joints.effort_nm[rh] - right_hip_expected;
  output.right_knee_residual_nm =
      state.joints.effort_nm[rk] - right_knee_expected;

  if (config_.use_force_projection && previous_command_.valid &&
      previous_command_.enable_motors) {
    const auto project_force = [&](float hip, float knee, float hip_residual,
                                   float knee_residual, float hip_velocity,
                                   float knee_velocity, float& force) {
      const float max_speed = std::max(
          config_.max_force_projection_joint_speed_rad_s, 0.0F);
      if (!config_.use_inertial_term &&
          std::max(std::abs(hip_velocity), std::abs(knee_velocity)) >
              max_speed) {
        // A gravity-only model cannot separate commanded/inertial torque from
        // an external leg load while the leg is moving. Keep the old
        // quasi-static guard for that intentionally incomplete model. When
        // use_inertial_term is enabled, the residual is instead formed after
        // subtracting the filtered joint acceleration term above, so the
        // touchdown sample is allowed to remain observable at high speed.
        return false;
      }
      const float jacobian_sq = hip * hip + knee * knee;
      if (!std::isfinite(jacobian_sq) ||
          jacobian_sq < std::max(config_.min_force_jacobian_sq_m2, 0.0F)) {
        return false;
      }
      // The residual is the motor torque needed to oppose an external force:
      // tau_external_motor = -J^T F. Recover F with the corresponding minus
      // sign, using the same native CAD joint-axis convention as WholeBodyModel.
      force = -(hip * hip_residual + knee * knee_residual) / jacobian_sq;
      return std::isfinite(force);
    };
    const auto left_leg = WholeBodyModel::legHeight(
        state.joints.position_rad[lh], state.joints.position_rad[lk]);
    const auto right_leg = WholeBodyModel::legHeight(
        state.joints.position_rad[rh], state.joints.position_rad[rk]);
    output.left_normal_force_valid = project_force(
        left_leg.d_height_d_hip, left_leg.d_height_d_knee,
        output.left_hip_residual_nm, output.left_knee_residual_nm,
        state.joints.velocity_rad_s[lh], state.joints.velocity_rad_s[lk],
        output.left_normal_force_n);
    output.right_normal_force_valid = project_force(
        right_leg.d_height_d_hip, right_leg.d_height_d_knee,
        output.right_hip_residual_nm, output.right_knee_residual_nm,
        state.joints.velocity_rad_s[rh], state.joints.velocity_rad_s[rk],
        output.right_normal_force_n);
  }

  const float deadband = std::max(config_.residual_deadband_nm, 0.0F);
  if (config_.use_force_projection) {
    // Both legs must agree on the *external normal-force hypothesis*.  The
    // individual hip/knee residuals are not required to share a sign: their
    // signs are determined by the live Jacobian and can legitimately differ.
    const float unloading_threshold = std::max(
        config_.unloading_force_threshold_n, 0.0F);
    const float impact_threshold = std::max(config_.impact_force_threshold_n,
                                            0.0F);
    output.direction_consistent_with_unloading =
        output.left_normal_force_valid && output.right_normal_force_valid &&
        output.left_normal_force_n >= 0.0F &&
        output.right_normal_force_n >= 0.0F &&
        output.left_normal_force_n <= unloading_threshold &&
        output.right_normal_force_n <= unloading_threshold;
    output.direction_consistent_with_impact =
        output.left_normal_force_valid && output.right_normal_force_valid &&
        state.imu.accel_magnitude_m_s2 >=
            std::max(config_.impact_accel_gate_m_s2, 0.0F) &&
        output.left_normal_force_n >= impact_threshold &&
        output.right_normal_force_n >= impact_threshold;
  } else {
    // Compatibility path for the pre-CMF unit contract.  Production profiles
    // keep use_force_projection=true; this fallback exists only so old callers
    // that have not yet supplied a command reference cannot silently change
    // their deterministic residual tests.
    const bool left_unloaded = output.left_hip_residual_nm <= -deadband &&
        output.left_knee_residual_nm <= -deadband;
    const bool right_unloaded = output.right_hip_residual_nm <= -deadband &&
        output.right_knee_residual_nm <= -deadband;
    output.direction_consistent_with_unloading = left_unloaded && right_unloaded;
    const float impact_threshold =
        std::max(config_.impact_residual_threshold_nm, 0.0F);
    const bool left_impact = output.left_hip_residual_nm >= impact_threshold &&
        output.left_knee_residual_nm >= impact_threshold;
    const bool right_impact = output.right_hip_residual_nm >= impact_threshold &&
        output.right_knee_residual_nm >= impact_threshold;
    output.direction_consistent_with_impact = left_impact && right_impact;
  }

  if (output.direction_consistent_with_unloading) {
    unloading_elapsed_s_ += dt;
  } else {
    unloading_elapsed_s_ = std::max(
        0.0F, unloading_elapsed_s_ - config_.residual_decay_rate * dt);
  }
  if (output.direction_consistent_with_impact) {
    impact_elapsed_s_ += dt;
  } else {
    impact_elapsed_s_ = std::max(
        0.0F, impact_elapsed_s_ - config_.residual_decay_rate * dt);
  }
  output.unloading_confidence = clamp01(
      unloading_elapsed_s_ / std::max(config_.residual_debounce_s, 1.0e-4F));
  output.impact_confidence = clamp01(
      impact_elapsed_s_ / std::max(config_.residual_debounce_s, 1.0e-4F));
  output.residual_confidence = std::max(output.unloading_confidence,
                                        output.impact_confidence);

  return output;
}

}  // namespace robot_control
