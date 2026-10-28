#include "robot_control/TunedConfig.hpp"

#include <algorithm>

#include "robot_control/GeneratedGeometry.hpp"

namespace robot_control {

EmbeddedController::Config tunedControllerConfig() {
  EmbeddedController::Config cfg;
  auto& lqr = cfg.lqr;
  lqr.wheel_radius_m = robot_control::geometry::wheel_radius_m;
  lqr.half_wheel_base_m = 0.1545F;
  lqr.left_wheel_sign = 1.0F;
  // The speed actuator is torque-limited at 0.13 N.m. Keep the operator
  // envelope wider than the validated cruise range so the feedback loop can
  // spend speed to catch pitch without violating the physical cap.
  // Stay below the native Moving Speed frame envelope while leaving enough
  // translational authority to catch a pitch transient after the wheel has
  // already crossed the quiet station speed.
  lqr.max_ground_velocity_m_s = 1.50F;
  cfg.wheel_max_torque_nm = 0.13F;
  const float wheel_torque_accel_m_s2 = 2.0F * cfg.wheel_max_torque_nm /
      (lqr.wheel_radius_m * robot_control::geometry::total_mass_kg);
  lqr.max_ground_accel_m_s2 = std::min(1.40F, wheel_torque_accel_m_s2);
  lqr.wheel_ground_mu = 0.8F;
  lqr.wheel_velocity_reference_from_measured = true;
  lqr.target_velocity_slew_rate_m_s2 = 0.30F;
  lqr.target_velocity_jerk_m_s3 = 1.0F;
  // Releasing DRIVE uses a faster but still torque-feasible profile. The
  // 0.65 factor leaves pitch-capture margin below the two-wheel torque cap;
  // both speed and acceleration remain continuous at the command edge.
  const float brake_capacity_m_s2 = std::min(
      wheel_torque_accel_m_s2, lqr.wheel_ground_mu * lqr.gravity_m_s2);
  lqr.stop_velocity_slew_rate_m_s2 = 0.65F * brake_capacity_m_s2;
  lqr.stop_velocity_jerk_m_s3 = 4.0F;
  lqr.target_yaw_slew_rate_rad_s2 = 1.0F;
  lqr.yaw_rate_feedback_kp = 1.0F;
  // Fail safe before the supported floor-side posture observed in the
  // native model (~0.69 rad).
  lqr.fall_pitch_rad = 0.55F;
  // Let the leg-side gimbal absorb the small supported pitch error. The
  // wheel LQR remains the fast catch controller for larger transients.
  lqr.wheel_pitch_feedback_scale = 0.70F;
  lqr.leg_length_m = 0.234F;
  lqr.pendulum_length_per_com_height = 1.077F;
  lqr.min_pendulum_length_m = 0.10F;
  lqr.max_pendulum_length_m = 0.27F;
  // LQR is solved for the fresh 500 Hz sensor interval, not the 3 kHz
  // actuator/cache loop. Gains are rebuilt only when the live COM height
  // changes enough to warrant a new reduced-order model.
  lqr.sampling_period_s = 0.002F;
  // Pitch capture is the primary safety objective.  A former MPC-shaped
  // velocity weight made the LQR brake odometry before it arrested pitch;
  // station return remains an adaptive outer target, not the fast state.
  lqr.position_cost_scale = 100.0F;
  lqr.pitch_cost_scale = 400.0F;
  lqr.velocity_cost_scale = 300.0F;
  lqr.pitch_rate_cost_scale = 40.0F;
  lqr.input_cost_scale = 10.0F;
  lqr.station_position_cost_scale = 200.0F;
  lqr.station_pitch_cost_scale = 400.0F;
  lqr.station_velocity_cost_scale = 300.0F;
  lqr.station_pitch_rate_cost_scale = 40.0F;
  lqr.station_input_cost_scale = 10.0F;
  lqr.station_gain_pitch_start_rad = 0.12F;
  lqr.station_gain_pitch_quiet_rad = 0.05F;
  lqr.station_gain_rate_start_rad_s = 0.60F;
  lqr.station_gain_rate_quiet_rad_s = 0.20F;
  lqr.station_gain_entry_dwell_s = 0.10F;
  lqr.station_gain_transition_s = 0.20F;
  lqr.station_pitch_return_gain_rad_per_m_s = 0.0F;
  lqr.station_pitch_return_max_rad = 0.0F;
  lqr.riccati_iterations = 80U;
  // A zero command is the natural ENABLE balance hold. Keep a local
  // odometry anchor in Balance so small IMU/encoder bias does not look like
  // continuous wheel slip. EmbeddedController releases this anchor whenever
  // a nonzero Drive/Yaw command or a pose transition is active.
  // Height transactions own a local support point. Re-anchor only at their
  // supported endpoint instead of pulling a long dance back to the initial
  // odometry sample after every shape change.
  lqr.position_anchor_hold = true;
  // Do not bleed the integrated wheel reference during a posture move: the
  // 0.13 N.m wheel servo needs the retained catch command to arrest pitch.
  // Position/velocity station feedback still owns the return target.
  lqr.station_velocity_decay_gain_hz = 0.0F;
  // The estimator's corrected rate remains exposed and calibrated, but the
  // current CAD plant profile keeps raw rate in the fast MPC until the
  // corrected-rate adoption trial clears all posture/stop envelopes.
  lqr.use_bias_corrected_pitch_rate = false;
  // The station return is an outer position loop around the same LQR.  A
  // 15 Hz return gain drove the bounded target straight into +/-0.10 m/s
  // on encoder quantisation, then reversed it every few hundred ms.  Keep
  // the return inside the measured wheel damping envelope so the body does
  // not trade a few millimetres of odometry for visible pitch chatter.
  lqr.station_hold_gain_hz = 5.0F;
  lqr.station_hold_yaw_return = true;
  lqr.station_hold_max_v = 0.06F;
  lqr.station_posture_hold_max_v = 0.25F;
  // Keep the integrated station command inside the posture envelope. The
  // live capture schedule still decides when this ceiling is approached;
  // it is distinct from the wider internal MPC catch acceleration limit.
  lqr.station_hold_command_max_v = 0.10F;
  // The station command starts quiet, then expands with the measured capture
  // distance in LqrBalancer. Do not cap a real posture catch at the quiet
  // 0.25 m/s envelope; that leaves the wheel without authority exactly when
  // pitch/velocity evidence says it must brake.
  lqr.station_hold_dynamic_max_v = lqr.station_posture_hold_max_v;
  // Once pitch/rate have been quiet, station return may move the LQR speed
  // reference back toward the local anchor.  Keep that outer reference
  // slower than the wheel pitch-capture loop so the two objectives cannot
  // fight during the first transient.
  lqr.station_hold_command_slew_rate_m_s2 = 0.10F;
  // station_target is fed into the LQR's own velocity_target, so this term
  // does not act as an independent outer damper: it scales the velocity
  // state the DARE already weights. At 5.0 it made x2 = 6*v + 15*pos_err,
  // i.e. ~6x the designed velocity feedback and ~48x K0's position gain,
  // which sat on the 0.25 m/s station clamp and limit-cycled. Keep this
  // small and let position/velocity regulation come from the costs below,
  // where the Riccati solve co-designs it against the pitch gains.
  lqr.station_hold_velocity_damping_gain = 1.00F;
  // Fade the slow station return to 25% inside a 100 mm capture envelope.
  // The fast pitch-capture MPC remains untouched, so this removes quiet
  // encoder chatter without sacrificing recovery authority.
  lqr.station_adaptive_full_capture_distance_m = 0.10F;
  lqr.station_quiet_gain_scale = 0.25F;
  // Capture-point scheduling supplies the slow, height-adaptive position
  // term; the separate damping term remains the high-frequency damper.
  // Keep the capture-point path available to callers, but do not enable it
  // in the native profile until it clears the closed-loop envelope; the
  // plant's provisional friction/torque model currently amplifies it.
  lqr.station_capture_gain = 0.0F;
  // Do not add a second velocity servo around the pitch LQR.  The first
  // closed-loop trial showed that this term uses the stale integrated
  // command, not the measured wheel state, and can reinforce a saturated
  // correction.  Keep it disabled until an actuator-aware reference
  // governor is proven in simulation.
  lqr.station_hold_velocity_servo_gain_hz = 0.0F;
  // Posture transitions must remain in place. The leg motion changes the
  // encoder-derived wheel odometry and COM geometry at the same time; if the
  // station anchor is released, the balance loop can turn that transient
  // into metres of unintended rolling. EmbeddedController re-anchors at the
  // actual endpoint, so this does not pull the robot back after a pose.
  lqr.hold_station_during_pose = true;
  // Station anchoring is adaptive: this is the allowable residual capture
  // distance, not a speed gate.  The core recomputes the required distance
  // from the live pendulum length and sensor state before resetting MPC.
  cfg.station_capture_margin_m = 0.025F;
  cfg.station_capture_filter_period_s = 0.08F;
  cfg.station_settle_dwell_s = 0.35F;
  cfg.posture_settle_dwell_s = 0.25F;
  // A posture path starts only after the wheel capture transient has mostly
  // decayed; otherwise the leg motion and the retained wheel velocity add
  // instead of sharing the same stationary anchor.
  cfg.posture_settle_wheel_velocity_m_s = 0.03F;
  cfg.height_pending_timeout_s = 2.0F;
  cfg.posture_reanchor_settle_s = 0.50F;
  // Riccati iterations are fixed and bounded; no online optimizer or
  // horizon storage remains in the native balance path.
  // Keep the provisional leg actuator envelope at the independently checked
  // 3 N.m value.  A larger clamp hides posture/geometry faults and has no
  // motor-spec evidence behind it.
  cfg.leg_max_torque_nm = 3.0F;
  cfg.leg_command_max_rate_rad_s = 12.0F;
  cfg.posture_abort_blend_s = 0.35F;
  cfg.hip_min_rad = robot_control::geometry::native_hip_min_rad;
  cfg.hip_max_rad = robot_control::geometry::native_hip_max_rad;
  cfg.knee_min_rad = robot_control::geometry::native_knee_min_rad;
  cfg.knee_max_rad = robot_control::geometry::native_knee_max_rad;
  // Height changes are task-space trajectories; the duration scales with
  // joint travel, while the balance loop catches the changing CoM.  Keep the
  // same measured rate in both directions so the GUI does not make one half
  // of the slider appear stuck.
  cfg.pose_change_duration_s = 2.5F;
  cfg.pose_max_joint_velocity_rad_s = 0.18F;
  cfg.pose_max_duration_s = 16.0F;
  cfg.height_min_duration_s = 5.0F;
  cfg.height_max_rate_m_s = 0.012F;
  cfg.height_lowering_rate_m_s = 0.012F;
  cfg.height_max_duration_s = 16.0F;
  cfg.height_capture_slowdown_min_scale = 0.02F;
  // Allow the final extension near the straight-leg branch to finish while
  // LQR is still inside its recoverable pitch envelope.  Pausing at 50 mm
  // froze the path around 0.237 m and then rejected a valid 0.245 m target.
  cfg.height_capture_pause_distance_m = 0.080F;
  cfg.height_capture_abort_s = 8.00F;
  // The fake driver has a steady-state load sag near the straight-leg and
  // natural branches; settle within 25 mm after the quiet-rate dwell.
  cfg.height_reached_tolerance_m = 0.025F;
  // LQR owns measured body pitch through the wheels. WBC owns only the
  // body-frame COM line through the leg Jacobian.
  cfg.whole_body_compensation = false;
  cfg.whole_body_pitch_gain = 1.0F;
  cfg.height_whole_body_compensation = true;
  cfg.height_whole_body_pitch_gain = cfg.whole_body_pitch_gain;
  // The legs are the fast sagittal body suspension. Its dimensionless gains
  // are scheduled from the live COM height inside the stance controller;
  // the wheel MPC remains the capture loop for translation.
  cfg.whole_body_stance.enabled = true;
  // Leg COM trim stays allowed through a fast rolling transient and the
  // bounded pre-fall capture window; the independent fall detector
  // still owns the final stop.
  cfg.whole_body_stance.max_pitch_rate_for_trim_rad_s = 10.0F;
  cfg.whole_body_stance.gravity_m_s2 = cfg.lqr.gravity_m_s2;
  cfg.whole_body_stance.com_position_gain = 3.00F;
  cfg.whole_body_stance.com_velocity_damping_s = 0.02F;
  // Give the legs a slow COM-line position task. The wheel LQR remains the
  // fast body-pitch capture loop; no IMU-pitch-to-leg path exists.
  cfg.whole_body_stance.max_pitch_for_trim_rad = 0.35F;
  cfg.whole_body_stance.roll_position_gain = 0.25F;
  cfg.whole_body_stance.height_position_weight = 1.0F;
  cfg.whole_body_stance.axle_position_weight = 0.10F;
  cfg.whole_body_stance.symmetric_task_damping = 1.0e-4F;
  cfg.whole_body_stance.correction_filter_hz = 4.0F;
  // Let the bounded correction_ set-point remove FK/model COM bias only in
  // settled stationary Balance; HEIGHT owns the leg target while active.
  cfg.whole_body_stance.com_integrator_gain_hz = 2.0F;
  cfg.whole_body_stance.max_correction_rad = 0.12F;
  // Leg COM motion is intentionally slower than the wheel capture loop;
  // otherwise the wheel spends its authority chasing the moving axle.
  cfg.whole_body_stance.max_correction_rate_rad_s = 0.30F;
  cfg.whole_body_stance.com_deadband_m = 0.0003F;
  cfg.whole_body_stance.body_pitch_task_enabled = true;
  cfg.whole_body_stance.body_pitch_task_gain = 0.80F;
  cfg.whole_body_stance.body_pitch_task_weight = 2.0F;
  cfg.whole_body_stance.body_pitch_task_max_axle_error_m = 0.020F;
  // Let the legs supply the accelerating lean geometrically. 1.0 = full
  // h*a/g compensation; the envelope below caps it at ~0.83 m/s2 worth of
  // COM travel, past which the wheels own the rest as before.
  // Legs own the outer speed loop; wheels own balance only. tau is the
  // speed-loop time constant the body shift alone would produce.
  cfg.whole_body_stance.speed_loop_tau_s = 0.55F;
  cfg.whole_body_stance.speed_loop_integral_hz = 0.0F;
  cfg.whole_body_stance.speed_loop_inertia_lead = 1.0F;
  cfg.whole_body_stance.speed_loop_filter_hz = 6.0F;
  cfg.whole_body_stance.speed_loop_max_rad = 0.18F;
  cfg.whole_body_stance.speed_loop_max_rate_rad_s = 1.50F;
  cfg.whole_body_stance.speed_loop_release_pitch_rad = 0.06F;
  cfg.whole_body_stance.speed_loop_release_span_rad = 0.09F;
  cfg.whole_body_stance_owns_static_com = true;
  cfg.whole_body_gravity_feedforward = true;
  cfg.whole_body_gravity_feedforward_scale = 1.0F;
  cfg.whole_body_stance.hip_min_rad = robot_control::geometry::native_hip_min_rad;
  cfg.whole_body_stance.hip_max_rad = robot_control::geometry::native_hip_max_rad;
  cfg.whole_body_stance.knee_min_rad = robot_control::geometry::native_knee_min_rad;
  cfg.whole_body_stance.knee_max_rad = robot_control::geometry::native_knee_max_rad;
  // No landing/recovery motion is compiled in this firmware profile.
  // A fall therefore stops every actuator instead of holding the legs live.
  cfg.keep_legs_active_when_fallen = false;
  cfg.flight_observer.free_fall_debounce_s = 0.025F;
  cfg.flight_observer.support_loss_debounce_s = 0.040F;
  // Use the observer's free-fall threshold instead of classifying ordinary
  // height-transition unloading as support loss. The time debounce still
  // protects genuine flight and keeps this sensor-only.
  cfg.flight_observer.free_fall_accel_threshold_m_s2 = 3.5F;
  cfg.flight_observer.collision_accel_threshold_m_s2 = 18.0F;
  cfg.flight_observer.collision_hold_s = 0.006F;
  cfg.flight_observer.touchdown_debounce_s = 0.006F;
  cfg.flight_observer.encoder_compression_rate_threshold_m_s = 0.08F;
  cfg.flight_observer.encoder_compression_reversal_threshold_m_s = 0.03F;
  cfg.flight_observer.encoder_touchdown_hold_s = 0.006F;
  // Effort frames may still be emitted by the fake GDS68 transport for
  // frame testing, but they are not part of the firmware balance contract.
  cfg.flight_observer.torque_corroboration_enabled = false;
  return cfg;
}

}  // namespace robot_control
