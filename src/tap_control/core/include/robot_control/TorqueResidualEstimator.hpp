#pragma once

#include "robot_control/Types.hpp"

namespace robot_control {

// Leg-motor current-based torque residual, per docs/RULES.md Rule 1's
// "Leg motor current-based torque exception (user, 2026-09-18)". This class
// reads RobotState::joints::effort_nm for the four leg (hip/knee) joints
// only -- never the wheel joints -- and compares it against an expected
// torque computed from FK/Jacobian and design-time CAD mass constants
// (GeneratedGeometry.hpp), never from simulator state.
//
// Binding conditions this class exists to satisfy (see docs/RULES.md Rule 1
// for the authoritative text):
//   1. Never the sole gate: this class only ever produces a *corroborating*
//      Output; it has no phase/state machine of its own and cannot, by
//      itself, declare support lost/gained. FlightObserver is the only
//      consumer, and only uses this Output to speed up a debounce that IMU
//      evidence has already started -- see FlightObserver::Config's
//      torque_corroboration_* fields.
//   2. Residual, not raw value: Output exposes measured-minus-expected
//      residuals, never the raw effort_nm value.
//   3. Leg-only: hip/knee CAN joints only. Wheel joints are never read here.
//   4. No new physical sensor: only RobotState::joints::effort_nm, already
//      populated end-to-end from the GDS68 driver's own Iq*Kt estimate
//      (docs/motor_drivers/GDS68_DRIVER.md), never a new topic/field.
//   5. Evidence-labeling: Output::valid plus the per-side direction flags
//      let a consumer/report state plainly whether this signal was used.
//
// KNOWN MODELING LIMITATIONS (documented, not hidden -- Rule 1 condition #2):
//   - Gravity is currently a distributed-link model using the provisional
//     thigh/shin/wheel COMs and origins in GeneratedGeometry.hpp. Body mass,
//     Coriolis terms and calibrated link friction are not yet included.
//   - No friction/gearbox model: no Kt, gear-ratio, or friction constant
//     exists anywhere in this repo for the leg CAN driver's 8:1 gearbox.
//     This is the dominant residual-noise source and cannot be compensated
//     analytically -- Config::residual_deadband_nm must be tuned empirically
//     against real trial data, and defaults to an "impossibly high" value so
//     the feature is an exact no-op until deliberately tuned.
//   - The previous MIT command is intentionally NOT subtracted from the
//     feedback torque. GDS68's reported Iq*Kt is the total actuator torque;
//     subtracting Kp/Kd/Tff would cancel the external support-load signal.
//   - Gravity/height-axis only: no fore/aft (x) force term, matching
//     LandingController.cpp's own "height is primary, x-force optional"
//     precedent.
class TorqueResidualEstimator {
 public:
  // The command that was sent during the previous control cycle.  The motor
  // feedback is asynchronous, so using the previous command is a bounded
  // approximation of the command that produced the returned effort sample.
  // It contains no simulator state; on hardware it corresponds to the MIT
  // position/velocity/gain/feedforward fields sent to the GDS68.
  struct Command {
    bool valid = false;
    bool enable_motors = false;
    JointArray position_rad{};
    JointArray velocity_rad_s{};
    JointArray torque_limit_nm{};
    JointArray kp_scale{};
    JointArray kd_scale{};
    JointArray feedforward_nm{};
  };

  struct Config {
    bool enabled = false;
    float gravity_m_s2 = 9.81F;
    // Retained for configuration/API compatibility with the first CMF draft.
    // A GDS68 MIT feedback sample is total Iq-derived actuator torque, so the
    // previous command is never subtracted from it.
    float commanded_kp_nm_rad = 0.0F;
    float commanded_kd_nm_s_rad = 0.0F;
    bool use_command_model = false;
    // Optional inertial term from a low-pass-filtered finite difference of
    // encoder velocity. Off by default: raw double-differentiation of
    // encoder velocity is a well-known noise amplifier and this term has not
    // been validated.
    bool use_inertial_term = false;
    float joint_accel_filter_time_constant_s = 0.02F;
    // Residual magnitude below this deadband never contributes to
    // confidence. Impossibly high by default (exact no-op) until tuned
    // against real trial data -- see the "no friction model" limitation
    // above for why this cannot be derived analytically.
    float residual_deadband_nm = 1.0e6F;
    // Elapsed-time debounce/decay for residual_confidence, mirroring
    // FlightObserver's own confidence idiom exactly (accumulate while
    // outside the deadband, decay at residual_decay_rate otherwise).
    float residual_debounce_s = 0.02F;
    float residual_decay_rate = 2.0F;
    // Residual magnitude (beyond the deadband) treated as impact-consistent.
    // Impossibly high by default (exact no-op) until tuned.
    float impact_residual_threshold_nm = 1.0e6F;
    // The external sagittal support reaction is estimated by projecting the
    // measured-minus-free-flight residual through the leg-height Jacobian.
    // These thresholds
    // are forces, not joint torques.  They are deliberately conservative and
    // must be calibrated from sensor-only trials.
    bool use_force_projection = true;
    float unloading_force_threshold_n = 6.0F;
    float impact_force_threshold_n = 30.0F;
    // An impact-force hypothesis is not allowed to accumulate while the IMU
    // still reports ballistic near-zero specific force.  This is a sensor
    // consistency gate, not a simulator contact gate.
    // Must exceed stationary 1g. This is a shock gate, not merely a
    // non-ballistic gate; otherwise ordinary supported load can look like an
    // impact whenever the projected force crosses impact_force_threshold_n.
    float impact_accel_gate_m_s2 = 18.0F;
    float min_force_jacobian_sq_m2 = 1.0e-4F;
    // Torque-to-force projection is only interpretable during a near-static
    // encoder interval. During active leg extension/braking, motor torque is
    // dominated by commanded/inertial motion and is not a contact estimate.
    float max_force_projection_joint_speed_rad_s = 0.5F;
  };

  struct Output {
    // False when inputs are non-finite, the GDS68 effort sample is missing,
    // or the estimator is disabled. A
    // disabled/invalid Output must never be read as "no external load" --
    // consumers must check this first.
    bool valid = false;
    float left_hip_residual_nm = 0.0F;
    float left_knee_residual_nm = 0.0F;
    float right_hip_residual_nm = 0.0F;
    float right_knee_residual_nm = 0.0F;
    float left_hip_expected_nm = 0.0F;
    float left_knee_expected_nm = 0.0F;
    float right_hip_expected_nm = 0.0F;
    float right_knee_expected_nm = 0.0F;
    bool left_normal_force_valid = false;
    bool right_normal_force_valid = false;
    // Estimated external force along the leg-height (+z) axis. The motor
    // residual is the opposing torque (-J^T F) in the native CAD convention;
    // this is a model-based hypothesis from encoder geometry and motor torque
    // feedback, not a force sensor and not simulator contact telemetry.
    float left_normal_force_n = 0.0F;
    float right_normal_force_n = 0.0F;
    // Both legs must independently show a residual consistent with reduced
    // load (measured below expected, beyond the deadband) before this is
    // true -- per docs/RULES.md Rule 2, never conclude a load-bearing state
    // from one leg alone.
    bool direction_consistent_with_unloading = false;
    // Both legs must independently show a residual consistent with an
    // unexpected extra load (measured above expected, beyond the deadband)
    // before this is true.
    bool direction_consistent_with_impact = false;
    // Debounced [0,1] confidence that the observed residual direction is
    // sustained, mirroring FlightObserver::Output::free_fall_confidence's
    // own clamp01(elapsed/debounce) idiom.
    float residual_confidence = 0.0F;
    float unloading_confidence = 0.0F;
    float impact_confidence = 0.0F;
  };

  TorqueResidualEstimator() = default;
  explicit TorqueResidualEstimator(Config config) : config_(config) {}

  void reset();

  void setCommand(const Command& command) noexcept { previous_command_ = command; }
  void setCommand(const MotorCommand& command) noexcept;

  [[nodiscard]] Output update(const RobotState& state);

 private:
  [[nodiscard]] bool finiteInput(const RobotState& state) const noexcept;

  Config config_;
  Command previous_command_{};
  float unloading_elapsed_s_ = 0.0F;
  float impact_elapsed_s_ = 0.0F;
  bool previous_valid_ = false;
  JointArray previous_velocity_rad_s_{};
  JointArray filtered_accel_rad_s2_{};
};

}  // namespace robot_control
