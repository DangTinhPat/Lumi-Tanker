#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "tap_fw/v3/RecoveryWheelPredictor.hpp"

namespace tap_fw::v3 {

// Recovery from the configured lying pose. Only IMU and encoder-derived
// kinematics enter this bounded, allocation-free state machine.
class StandUp {
 public:
  enum class Phase { Idle, Folding, Kneeling, Pitching, Flipped, Catching, Support, Failed };
  enum class Failure { None, InvalidInput, PitchLimit, Timeout, HipTracking, FloorConstraint };

  struct Config {
    bool enabled = false;
    // Adapter request: retain the seeded attitude/bias, but use the existing
    // gyro-dominant floor estimator during recovery. Core opt-in; no new
    // estimator or wheel/contact acceleration model is introduced here.
    bool gyro_dominant_imu = false;
    // Recovery task priority: capture the physical CoM before asking for a
    // level trunk. The adapter preserves the recovery knee/height after handoff;
    // support_hip_capture may keep the bounded CoM hip correction active.
    // Core opt-in; SET standcapture 0 restores the combined posture/catch law.
    bool capture_first = false;
    // Keep the existing CoM hip allocator alive after wheel LQR takes over.
    // Deferring trunk leveling must not also freeze the balance actuator.
    // Core opt-in; SET standhip 0 restores the fixed-hip recovery handoff.
    bool support_hip_capture = false;
    // Opt-in clearance velocity damper, not a body pitch/level target.
    // Geometry adapter supplies common-hip rate bounds from IMU + encoders.
    // SET standfloor 0 restores the previous recovery law on the next run.
    bool floor_guard = false;
    // Keep the floor guard (and its fault) alive after the wheel LQR owns balance (Support).
    // Robot log 20261019 (two Balancing entries): the guard faulted 0.6 s after handoff with
    // 19-29 mm of modelled clearance, predicted 4-6 mm, because it demanded 2.1-2.5 rad/s of hip
    // that the acceleration bound forbids. false = guard only during B4 (Catching).
    // Core default true (unchanged); tunedRecovery false. SET standfloorsup 1 restores it.
    bool floor_guard_in_support = true;
    // Opt-in: approach a feasible floor rate at bounded acceleration while
    // predicted clearance is still ABOVE the configured shell margin.
    bool floor_accel_grace = false;
    float floor_clearance_m = 0.006F;
    float floor_arm_clearance_m = 0.008F;
    float floor_arm_dwell_s = 0.02F;
    float floor_approach_time_s = 0.15F;
    float floor_prediction_lag_s = 0.08F;
    // Allocate stopping speed error to the hip's CoM target, rather than
    // adding same-sign velocity feedback to wheel acceleration while the hip
    // independently erases all lean. Core opt-in; SET standcoord 0 reverts.
    bool coordinated_speed = false;
    // With coordinated_speed, the wheel lean loop also regulates toward the
    // stopping lean target instead of zero. Otherwise the faster wheel loop
    // erases the hip's lean target and nothing can brake (logs 200417-201659:
    // lean held at -0.1..0 while B4 accelerates back to -0.7 m/s).
    // Core opt-in; SET standwheelref 0 restores the zero-lean wheel target.
    bool catch_wheel_speed_lean = false;
    // Robot logs 20261019: B4 holds lean +0.03..+0.05 (model) for 1.5-2 s while the speed keeps
    // growing, -0.44 -> -0.58 m/s. With the P-only speed->lean target the drift settles at
    // v = -g*bias/catch_speed_gain: bias 0.06 rad (model lean minus true lean, ~2.7 mm of COM x)
    // and gain 1/s give -0.59 m/s, the observed value. This bounded integral of ground speed is
    // added to the stopping lean target so any such offset (up to the bound) is absorbed.
    // 1/s^2; 0 = off. SET standspdi / standspdimax.
    float catch_speed_integral_gain = 0.0F;
    // After the handoff to the wheel LQR, keep its adaptive balance trim within +-this of the seed
    // (0 = no window). Robot log 20261019: in a 16 s balance the trim wandered -0.148 .. +0.10 rad
    // (the trim_acc estimate follows the pitch whenever acceleration is small) while the real
    // equilibrium sat near -0.05; at trim +0.098 the robot stood at pitch +0.03 and fell forward.
    // SET trimwin. Does not touch B4.
    float balance_trim_window_rad = 0.0F;
    // Balance-angle observer for the handoff: the trim_acc estimate is only updated while
    // |pitch rate| < 0.3 rad/s and |ground accel| < 0.8 m/s^2. During a large pitch oscillation that
    // keeps just 8-17 % of the samples, all of them near the turning points, so the gated estimate swings
    // (REC fits 19/10: gated -0.20 .. +0.06 per run) while the all-samples mean of theta_v - atan(a/g)
    // is stable (-0.03 .. +0.015, std of the signal 0.23). A positive tau uses every sample with this time
    // constant [s] (centre of the oscillation = equilibrium) instead of the gate. 0 = previous behaviour.
    // SET trimtau. Applies only after a stand-up handoff (together with balance_trim_window_rad).
    float balance_trim_ungated_tau_s = 0.0F;
    float catch_speed_integral_max_rad = 0.08F;
    float catch_speed_filter_s = 0.10F;
    float catch_speed_lean_max_rad = 0.12F;
    float fold_rate_rad_s = 0.35F;
    float fold_travel_rad = 2.05F;
    float fold_direction = -1.0F;
    float fold_pitch_limit_rad = 0.72F;
    float kneel_hold_s = 1.0F;
    // Skip the separate pitch-to-level flip: after the kneel hold go straight
    // to Catching, where hips and wheels cooperate on the physical CoM.
    bool skip_pitching = false;
    bool hold_at_kneel = false;    // diagnostic operator hold, FLIP advances
    bool hold_after_flip = false;  // diagnostic stop after step 3
    float flip_pitch_kp_rad_s_per_rad = 1.0F;
    float flip_pitch_kd = 0.08F;
    float flip_hip_rate_max_rad_s = 0.20F;
    float flip_travel_max_rad = 1.0F;
    float pitch_target_tol_rad = 0.04F;
    float pitch_rate_tol_rad_s = 0.10F;
    float pitch_settle_s = 0.15F;
    // Start capture while still pitching, once the body envelope clears the
    // floor. Core opt-in; the GIM runner enables this overlapping sequence.
    bool overlap_after_lift = false;
    float lift_clearance_m = 0.003F;
    float lift_progress_rad = 0.06F;
    float lift_confirm_s = 0.02F;
    float catch_blend_s = 0.15F;
    // Zero inherits the hip blend. A separate wheel ramp leaves hip lift
    // unchanged when requesting faster wheel assistance at B4 entry.
    float catch_wheel_blend_s = 0.0F;
    // Hip allocation: reduce CoM lean, damp its motion, and level the body.
    // Signed feedback permits correction in either direction; no oscillation
    // or timed sweep is injected to "search" for the centre of mass.
    float catch_hip_gain = 4.0F;       // (rad/s)/rad, before dividing by FK Jacobian
    float catch_hip_damping_s = 0.2F;
    // Opt-in divergent-mode hip error instead of angle + fixed damping.
    // SET standcp 0 restores angle/damping feedback before the next run.
    bool catch_capture_point_hip = false;
    float catch_capture_rate_filter_s = 0.03F;
    float catch_body_gain = 1.0F;
    float catch_body_damping_s = 0.0F;
    // Separate from capture_first: knees/height remain held after handoff.
    // SET standlevel 0 restores the preceding capture-only recovery.
    bool catch_body_leveling = false;
    // During B4's posture transfer, prevent a posture request from adding
    // hip motion away from a COM that is still outside the capture centre.
    // Core opt-in; floor/acceleration/travel constraints remain downstream.
    bool catch_posture_capture_guard = false;
    // v3.7 B4 lift-then-sweep (operator's plan). Hip rotation lifts the knees
    // off the floor and its reaction raises the head; when the hip stalls the
    // head drops back and the shell touches down again (logs 204830: pitch
    // .34 -> .47, clearance -2.5 mm at B4 0.37 s). While lifting, the hip keeps
    // a minimum rate (it is not braked by lean) and the knee opens gradually to
    // raise the whole body (about 100 mm/rad vs 11 mm/rad for the hip, CAD).
    // Once the modelled clearance is reached, or the capture point passes
    // lift_cp_max, the usual signed COM law sweeps the hip back. Knee never
    // refolds. Core opt-in; SET standlift 0 restores the fixed-knee B4.
    bool catch_lift = false;
    float lift_hip_rate_min_rad_s = 1.0F;
    float lift_hip_min_travel_rad = 0.0F;  // hip floor is kept at least until the hip rose this far (0 = off)
    float lift_hip_accel_rad_s2 = 12.0F;   // hip accel limit while the floor is active (sim: 20-30 rad/s^2 + 1.5-2 rad/s floor pitch the head past the limit by B4 0.4 s)
    float lift_hip_release_s = 0.15F;
    // Hip motion pitches the head up by angular-momentum exchange about the axle.
    // lift_head_rate_limit: scale the hip request down once the head rises faster
    // than this (rad/s, 0 = off). lift_hip_delay_s: hold the hip this long so the
    // wheels can spin up first (0 = off).
    float lift_head_rate_limit_rad_s = 0.0F;
    // Planted-leg kinematics (sim and log 204830): pitch ~ const - hip - knee, so hip
    // and knee opening both tilt the head up and the knee also pushes the COM behind
    // the axle (d lean/d knee ~ -0.7, d lean/d hip ~ -0.06). Pitch-neutral extension:
    // add -lift_pitch_neutral * knee_rate to the hip request (1 = hip cancels the knee).
    float lift_pitch_neutral = 0.0F;
    // Wheel assist (operator): while the hip lifts, accelerate the wheels BACKWARD by
    // lift_wheel_assist [m/s^2 per rad/s of hip rate]. A backward wheel torque reacts as a
    // nose-down torque on the body, opposing the head-up reaction of the hip. 0 = off.
    float lift_wheel_assist = 0.0F;
    // Signed posture (head level) authority already in B4: the leveling term is one-sided
    // (it may only lift the hip) until Support, so nothing opposes a head that goes up.
    // lift_signed_posture lets it also push the hip down while the head is above level.
    bool lift_signed_posture = false;
    float lift_hip_delay_s = 0.0F;      // the floor fades out over this time (no on/off jerk)
    float lift_hip_travel_rad = 0.6F;      // hip floor ends once the hip rose this far
    float lift_cp_max_rad = 0.10F;         // ... or the capture point passes this
    float lift_max_s = 0.8F;               // ... or after this long
    float lift_target_clearance_m = 0.020F;  // modelled clearance that completes the lift
    // Knee opening target: 0.643 rad = dashboard body height 130 mm (hip-to-axle 98.3 mm,
    // LegHeight::forHeight). With lift_knee_full the knee opens all the way regardless
    // of the clearance reached; otherwise it stops once lift_target_clearance_m is met.
    float lift_knee_max_rad = 0.643F;
    bool lift_knee_full = true;
    // Two-stage opening: past lift_knee_stage1_rad the knee only keeps opening while the
    // COM is near the axle (lean > lift_knee_lean_gate_rad). Knee extension pitches the
    // head up; doing it with the COM still far behind the axle flips the body (sim).
    // stage1 = 0 disables the gate.
    float lift_knee_stage1_rad = 0.0F;
    float lift_knee_lean_gate_rad = -0.06F;
    // Pause knee opening while the head is above this pitch (negative = nose up):
    // knee extension also pitches the head up (sim: flips past the limit). Off at -10.
    float lift_knee_pitch_stop_rad = -10.0F;
    float lift_knee_rate_rad_s = 1.5F;
    float lift_knee_accel_rad_s2 = 15.0F;
    float lift_knee_tracking_max_rad = 0.20F;
    // Posture (head level) term arms once the hip has risen this far AND the modelled
    // clearance is at least this much. Logs 204830: the hip stalls at -0.50 rad (rise
    // 0.247 < 0.25) with the posture term still disarmed. SET/--standup-set arm_hip, arm_clearance.
    float body_level_arm_hip_rad = 0.25F;
    float body_level_arm_clearance_m = 0.008F;
    float recovery_body_gain = 2.5F;
    float recovery_body_damping_s = 0.35F;
    float recovery_body_rate_max_rad_s = 1.0F;
    float catch_wheel_peak_s = 0.0F;    // opt-in actuator request, bounded window
    float catch_hip_rate_max_rad_s = 1.0F;
    float catch_hip_accel_rad_s2 = 12.0F;
    float catch_hip_travel_rad = 1.2F;
    float catch_hip_tracking_max_rad = 0.12F;
    // Wheel inner loop on physical CoM lean: acceleration -> speed, with
    // actuator-lag lead, tracking anti-windup and inverse carrier kinematics.
    float catch_accel_gain = 35.0F;     // (m/s^2)/rad
    float catch_accel_damping = 4.0F;   // (m/s^2)/(rad/s)
    float catch_speed_gain = 2.0F;      // 1/s, outer-loop stopping feedback
    float catch_accel_max = 2.0F;       // m/s^2, below the 0.13 Nm pair's ideal cap
    float catch_wheel_lead_s = 0.075F;
    // Experimental B4 wheel-only prediction; SET standpred 0 reverts. No hip
    // feedback or raw handoff/floor safety input is replaced by predictions.
    bool catch_wheel_prediction = false;
    // Capture-mode speed servo with quasi-steady equilibrium adaptation.
    // Core opt-in; SET standservo 0 restores the acceleration integrator.
    bool catch_capture_velocity = false;
    // SET standbrake: measured-motion preview and persistent command integral.
    // Core opt-in; avoids assuming braking from commands still in flight.
    bool catch_measured_braking = false;
    // Once a sustained capture region is reached, settle hip posture and
    // transfer rolling balance to LQR. Do not require standstill before the
    // controller responsible for stopping is allowed to take over.
    bool catch_rolling_transfer = false;
    // Allow the already captured, rolling robot to leave Catching even if
    // its wheel command touches the software cap. Measured tracking and
    // all physical-state gates still apply. SET standhandoff 0 reverts.
    bool catch_rolling_handoff = false;
    float rolling_handoff_settle_s = 0.08F;
    // Upper speed bound for the rolling handoff; 0 keeps the wheel cap +
    // tracking margin. Log 201659 handed over at -0.67 m/s and LQR sat at
    // the 20 rad/s cap from its first tick: it needs headroom to brake.
    float rolling_handoff_speed_m_s = 0.0F;
    float transfer_confirm_s = 0.04F;
    float transfer_ramp_s = 0.12F;
    float transfer_clearance_m = 0.012F;
    float transfer_floor_reserve_m = 0.008F;
    float transfer_capture_rate_rad_s = 0.60F;
    float catch_wheel_delay_s = 0.110F;
    float catch_wheel_lag_s = 0.030F;
    float catch_wheel_max_rad_s = 10.0F;
    float catch_tracking_margin_m_s = 0.15F;
    float catch_lean_tol_rad = 0.08F;
    float catch_capture_tol_rad = 0.06F;
    float catch_body_tol_rad = 0.25F;
    float catch_rate_tol_rad_s = 1.5F;
    float catch_speed_tol_m_s = 0.5F;
    float catch_settle_s = 0.04F;
    // Opt-in handoff guard, independent of the skipped Pitching phase. The
    // 18/10 robot log crossed the old capture gate with wheels still lagging.
    // Clearance is a model hypothesis, not a measured contact/knee-lift signal.
    bool guarded_handoff = false;
    float handoff_settle_s = 0.20F;
    float handoff_body_tol_rad = 0.12F;
    float handoff_body_rate_tol_rad_s = 0.40F;
    float handoff_hip_rate_tol_rad_s = 0.50F;
    float handoff_wheel_tracking_tol_rad_s = 4.0F;
    float pitch_limit_rad = 0.72F;
    // Optional larger nose-up/backward body envelope during catch/support.
    // Zero preserves the symmetric pitch_limit_rad. Positive pitch is forward.
    float back_pitch_limit_rad = 0.0F;
    float timeout_s = 20.0F;            // per moving phase
  };

  struct Input {
    float dt_s = 0.002F;
    float pitch_rad = 0.0F;             // +Y rotation, positive nose-down
    float pitch_rate_rad_s = 0.0F;
    float capture_lean_rad = 0.0F;      // physical axle->CoM angle, no hanging-pose rebase
    float capture_rate_rad_s = 0.0F;    // gyro + full measured FK Jacobian * joint rates
    float support_lean_ref_rad = 0.0F;  // physical CoM reference from wheel LQR in Support
    float hip_jacobian = 1.0F;
    float hip_measured_rad = 0.0F;
    float hip_measured_rate_rad_s = 0.0F;
    float knee_measured_rad = 0.0F;     // mean knee encoder (core offset, >= 0)
    std::array<float, 2> wheel_measured_rad_s{};  // native +Y on both sides
    float carrier_rate_rad_s = 0.0F;    // body + hip + knee angular velocity
    float ground_speed_m_s = 0.0F;      // encoders + carrier, assumes rolling support
    float wheel_radius_m = 0.04F;
    float capture_omega_rad_s = 10.0F;  // sqrt(m*g*l/I_axle), model constants + encoders
    float body_clearance_m = 0.0F;     // conservative CAD/IMU/encoder estimate
    bool body_clearance_valid = false;
    float floor_clearance_rate_m_s = 0.0F;
    float floor_predicted_clearance_m = 0.0F;
    float floor_hip_rate_lower_rad_s = 0.0F;
    float floor_hip_rate_upper_rad_s = 0.0F;
    bool floor_bounds_valid = false;
    bool floor_bounds_feasible = false;
    bool sensors_ok = true;
  };

  struct Output {
    Phase phase = Phase::Idle;
    float hip_target_rad = 0.0F;
    float knee_target_rad = 0.0F;
    float wheel_rad_s = 0.0F;
    bool wheels_on = true;
    float capture_point_rad = 0.0F;
    float capture_rate_rad_s = 0.0F;
    float hip_capture_rate_rad_s = 0.0F;
    float hip_capture_error_rad = 0.0F;
    float hip_feedback_rate_rad_s = 0.0F;
    bool posture_capture_limited = false;
    float body_level_weight = 0.0F;
    float body_level_rate_rad_s = 0.0F;
    float body_rate_filtered_rad_s = 0.0F;
    float hip_rate_rad_s = 0.0F;
    float knee_rate_rad_s = 0.0F;
    int lift_state = 0;                // 0 off, 1 lifting (hip floor), 2 knee/clearance only, 3 done
    float phase_elapsed_s = 0.0F;
    float body_clearance_m = 0.0F;
    bool body_clearance_valid = false;
    float catch_blend = 0.0F;
    bool wheel_peak_requested = false;
    Failure failure = Failure::None;
    float hip_measured_rad = 0.0F;
    float capture_lean_rad = 0.0F;
    float wheel_tracking_error_rad_s = 0.0F;  // max of the two wheels
    float settle_s = 0.0F;
    bool handoff_ready = false;
    bool rolling_handoff = false;  // candidate gate, not the actual mode switch
    float handoff_settle_required_s = 0.0F;
    bool hip_limited = false;
    bool wheel_limited = false;
    float wheel_accel_m_s2 = 0.0F;
    float wheel_lean_ref_rad = 0.0F;  // wheel lean target (0 unless catch_wheel_speed_lean)
    float speed_feedback_accel_m_s2 = 0.0F;
    float speed_filtered_m_s = 0.0F;
    float speed_integral_rad = 0.0F;   // integral part of the stopping lean target
    bool wheel_prediction_valid = false;
    bool wheel_measured_preview = false;
    bool wheel_prediction_capped = false;
    float wheel_prediction_weight = 0.0F;
    float wheel_predicted_lean_rad = 0.0F;
    float wheel_predicted_rate_rad_s = 0.0F;
    float wheel_predicted_speed_m_s = 0.0F;
    float wheel_observed_accel_m_s2 = 0.0F;
    float wheel_observed_angular_accel_rad_s2 = 0.0F;
    float wheel_raw_accel_m_s2 = 0.0F;
    float capture_servo_weight = 0.0F;
    float capture_servo_speed_m_s = 0.0F;
    float capture_integrated_speed_m_s = 0.0F;
    bool transfer_captured = false;
    float transfer_weight = 0.0F;
    float transfer_confirm_s = 0.0F;
    float capture_equilibrium_rad = 0.0F;
    bool capture_equilibrium_learning = false;
    float capture_lean_target_rad = 0.0F;
    bool floor_armed = false;
    bool floor_limited = false;
    bool floor_infeasible = false;
    bool floor_accel_saturated = false;
    bool floor_bounds_valid = false;
    float floor_clearance_rate_m_s = 0.0F;
    float floor_predicted_clearance_m = 0.0F;
    float floor_hip_rate_lower_rad_s = 0.0F;
    float floor_hip_rate_upper_rad_s = 0.0F;
  };

  // The recovery tuning the stand-up runs with on the robot and in the GIM simulator (18/10): the overlapping
  // sequence without the separate pitch flip (skip_pitching), a bounded catch (hip 2 rad/s, wheels 20 rad/s,
  // 5 m/s^2 requested). The peak request is simulator-only: the firmware wheel adapter
  // sends speed commands, not a higher torque/current limit.
  [[nodiscard]] static Config tunedRecovery(float wheel_lag_s = 0.075F) noexcept {
    Config su;
    su.enabled = false;
    su.gyro_dominant_imu = true;
    su.capture_first = true;
    su.support_hip_capture = true;
    su.catch_body_leveling = true;
    su.floor_guard = true;
    su.floor_accel_grace = true;
    // Robot log 174022: the 6 mm / 150 ms damper faults at 8.1 mm
    // estimated clearance (7.66 mm predicted), early in the capture phase.
    // Allow the capture allocator to use a closer approach to the floor;
    // this is a shell-clearance margin, not a backward-lean angle limit.
    // Smaller T permits faster closing away from the boundary. Keep the
    // 80 ms actuator-lag reserve and all hip/travel/fall limits unchanged.
    su.floor_clearance_m = 0.002F;
    su.floor_arm_clearance_m = 0.004F;
    su.floor_approach_time_s = 0.04F;
    // Operator-requested backward lean room, 50 degrees. No floor override.
    su.back_pitch_limit_rad = 0.8726646F;
    su.coordinated_speed = true;
    su.hold_at_kneel = false;
    su.hold_after_flip = false;
    su.skip_pitching = true;
    su.overlap_after_lift = true;
    su.fold_rate_rad_s = 0.80F;
    su.flip_hip_rate_max_rad_s = 3.00F;
    su.flip_pitch_kp_rad_s_per_rad = 10.0F;
    su.fold_pitch_limit_rad = 0.80F;
    su.catch_body_gain = 4.0F;
    su.catch_body_damping_s = 0.5F;
    // Logs 175749/180112: gains 4/6 ask ~0.7/1.2 rad/s initially,
    // still below the existing 2 rad/s cap; max encoder lag 0.027/0.034 rad.
    // Raise COM error response to 8, retaining damping and actuator bounds.
    // This is not increased driver current/torque. SET standhipgain 6 or 4
    // restores the corresponding run's allocator gain before the next run.
    su.catch_hip_gain = 8.0F;
    // Robot log 182625: stronger rate/omega feedback brakes the lift at
    // ~0.38 rad of hip travel, then the floor constraint faults. Restore the
    // preceding angle + 0.2 damping allocator; CP stays experimental/off.
    // The raw CP handoff gate remains active. Travel is a cap, not a target.
    su.catch_capture_point_hip = false;
    su.catch_wheel_prediction = true;
    su.catch_capture_velocity = true;
    su.catch_measured_braking = true;
    su.catch_rolling_transfer = true;
    su.catch_rolling_handoff = true;
    su.rolling_handoff_speed_m_s = 0.30F;
    su.catch_wheel_speed_lean = true;
    su.catch_posture_capture_guard = true;
    su.floor_guard_in_support = false;
    su.catch_speed_integral_gain = 1.0F;
    su.balance_trim_window_rad = 0.06F;
    su.balance_trim_ungated_tau_s = 2.0F;
    su.catch_speed_integral_max_rad = 0.08F;
    // Robot logs 20261019 (7 trials): the B4 hip rises only 0.19-0.25 rad and stops at about -0.5 rad
    // when lean crosses 0, because the posture (head-level) term arms only after 0.25 rad of hip rise.
    // The head then drops back (pitch .34 -> .48) and the floor guard faults at B4 ~0.6 s needing
    // 1.0-1.4 rad/s of hip. Arm the posture term from the start and let it ask for up to 1.5 rad/s
    // (4 rad/s per rad of pitch; one-sided: it only raises the hip while the head is down).
    // Rollback: SET standarmhip 0.25, standarmclr 8, standbodygain 2.5, standbodymax 1.
    su.body_level_arm_hip_rad = 0.0F;
    su.body_level_arm_clearance_m = 0.0F;
    su.recovery_body_gain = 4.0F;
    su.recovery_body_rate_max_rad_s = 1.5F;
    su.catch_hip_rate_max_rad_s = 2.0F;
    // Log 201044: encoders reach -18..-19 rad/s while B4 requests only -16;
    // COM remains behind the axle and falls back again. Operator requests
    // stronger response: 20 rad/s / 5 m/s^2 command ceilings, not measured
    // motor capabilities. Keep the preceding 60 ms wheel-only entry ramp.
    // Rollback this tuning: SET standwheel 16; SET standaccel 4.
    su.catch_wheel_max_rad_s = 20.0F;
    su.catch_accel_max = 5.0F;
    su.catch_wheel_blend_s = 0.06F;
    // In coordinated mode this is a stopping-acceleration gain for the hip
    // CoM target. Legacy mode retains the old wheel-only +gain*v term.
    // Slower outer speed loop than the 4/s hip lean loop and delayed wheels.
    // Robot log 170402: 0.5/s requested only ~0.035 rad of stopping lean
    // at -0.7 m/s; B4 rolled back 1.02--1.17 m before handoff. Increase the
    // existing stopping target, not the already saturated -3 m/s^2 wheel
    // request. SET standspeed 0.5 restores that session's tuning.
    // The old 2/s gain acted directly on wheel acceleration; it is not
    // carried unchanged into this hip allocation. SET standspeed tunes it.
    su.catch_speed_gain = 1.0F;
    su.catch_blend_s = 0.08F;
    su.catch_settle_s = 0.02F;
    su.guarded_handoff = true;
    su.catch_speed_tol_m_s = 0.15F;  // stop the drift before freezing the captured leg pose
    su.catch_wheel_peak_s = 0.6F;
    su.catch_wheel_lead_s = wheel_lag_s;
    return su;
  }

  StandUp() noexcept = default;
  explicit StandUp(Config config) noexcept : config_(config) {}

  void begin(float hip_rad, float knee_rad) noexcept {
    phase_ = std::isfinite(hip_rad) && std::isfinite(knee_rad) ? Phase::Folding : Phase::Failed;
    failure_ = phase_ == Phase::Failed ? Failure::InvalidInput : Failure::None;
    elapsed_s_ = pitch_settle_s_ = 0.0F;
    hip_start_rad_ = hip_cmd_ = std::isfinite(hip_rad) ? hip_rad : 0.0F;
    flip_start_hip_rad_ = catch_start_hip_rad_ = hip_cmd_;
    knee_cmd_ = std::isfinite(knee_rad) ? knee_rad : 0.0F;
    speed_filtered_m_s_ = speed_integral_rad_ = 0.0F;
    hip_rate_cmd_ = wheel_velocity_m_s_ = 0.0F;
    catch_leveling_ = true;
    catch_stall_s_ = 0.0F;
    body_level_armed_ = body_rate_initialized_ = false;
    body_level_weight_ = body_support_weight_ = body_rate_filtered_ = 0.0F;
    capture_equilibrium_rad_ = capture_equilibrium_dwell_s_ = 0.0F;
    transfer_captured_ = false;
    transfer_dwell_s_ = transfer_weight_ = 0.0F;
    hip_capture_rate_initialized_ = false;
    hip_capture_rate_filtered_ = 0.0F;
    wheel_predictor_.reset();
    wheel_prediction_armed_ = false;
    wheel_prediction_weight_ = 0.0F;
    lift_s_ = b4_clear_s_ = knee_rate_cmd_ = lift_hip_weight_ = 0.0F;
    lift_done_ = false;
    knee_start_rad_ = knee_cmd_;
    flip_pitch_initialized_ = catch_overlap_ = false;
    lift_clear_s_ = catch_entry_rate_ = 0.0F;
    floor_armed_ = false;
    floor_arm_s_ = 0.0F;
  }

  void reset() noexcept {
    phase_ = Phase::Idle;
    failure_ = Failure::None;
    body_level_armed_ = body_rate_initialized_ = false;
    body_level_weight_ = body_support_weight_ = body_rate_filtered_ = 0.0F;
    capture_equilibrium_rad_ = capture_equilibrium_dwell_s_ = 0.0F;
    transfer_captured_ = false;
    transfer_dwell_s_ = transfer_weight_ = 0.0F;
    floor_armed_ = false;
    floor_arm_s_ = 0.0F;
    hip_capture_rate_initialized_ = false;
    wheel_predictor_.reset();
    wheel_prediction_armed_ = false;
    wheel_prediction_weight_ = 0.0F;
    speed_integral_rad_ = 0.0F;
    lift_s_ = b4_clear_s_ = knee_rate_cmd_ = lift_hip_weight_ = 0.0F;
    lift_done_ = false;
  }

  bool advanceFromKneeling() noexcept {
    if (phase_ != Phase::Kneeling) return false;
    phase_ = Phase::Pitching;
    elapsed_s_ = pitch_settle_s_ = 0.0F;
    flip_start_hip_rad_ = hip_cmd_;
    flip_pitch_initialized_ = false;
    lift_clear_s_ = 0.0F;
    return true;
  }

  [[nodiscard]] Output update(const Input& in) noexcept {
    Output out{};
    out.hip_measured_rad = in.hip_measured_rad;
    out.capture_lean_rad = in.capture_lean_rad;
    out.capture_rate_rad_s = in.capture_rate_rad_s;
    out.body_clearance_m = in.body_clearance_m;
    out.body_clearance_valid = in.body_clearance_valid && std::isfinite(in.body_clearance_m);
    if (phase_ == Phase::Idle || phase_ == Phase::Failed) return output(out);
    if (!valid(in)) {
      phase_ = Phase::Failed;
      failure_ = Failure::InvalidInput;
      return output(out);
    }
    const float dt = std::clamp(in.dt_s, 0.0F, 0.01F);
    if (config_.catch_body_leveling) {
      if (!body_rate_initialized_) {
        body_rate_filtered_ = in.pitch_rate_rad_s;
        body_rate_initialized_ = true;
      } else {
        body_rate_filtered_ += dt / (0.04F + dt) *
            (in.pitch_rate_rad_s - body_rate_filtered_);
      }
    }
    wheel_radius_for_history_ = in.wheel_radius_m;
    if (config_.catch_wheel_prediction) {
      if (in.dt_s > 0.01F) {
        // A missed interval invalidates the time-stamped command queue. Raw
        // safety/legacy control still own this tick; rebuild preview history.
        wheel_predictor_.reset();
        wheel_prediction_armed_ = false;
        wheel_prediction_weight_ = 0.0F;
      } else {
        wheel_predictor_.observe(dt, in.capture_rate_rad_s, in.ground_speed_m_s);
      }
    }
    const bool supported_hip = phase_ == Phase::Support && config_.support_hip_capture;
    const bool rolling_transfer = config_.catch_rolling_transfer &&
        config_.capture_first && config_.support_hip_capture &&
        config_.catch_body_leveling && config_.guarded_handoff && config_.floor_guard;
    if (config_.catch_capture_point_hip && (phase_ == Phase::Catching || supported_hip)) {
      if (!hip_capture_rate_initialized_) {
        hip_capture_rate_filtered_ = in.capture_rate_rad_s;
        hip_capture_rate_initialized_ = true;
      } else {
        const float alpha = config_.catch_capture_rate_filter_s > 0.0F
            ? dt / (config_.catch_capture_rate_filter_s + dt) : 1.0F;
        hip_capture_rate_filtered_ += alpha * (in.capture_rate_rad_s - hip_capture_rate_filtered_);
      }
    } else {
      hip_capture_rate_initialized_ = false;
    }
    if (phase_ == Phase::Catching || supported_hip) {
      const float alpha = config_.catch_speed_filter_s > 0.0F
          ? std::clamp(dt / config_.catch_speed_filter_s, 0.0F, 1.0F) : 1.0F;
      speed_filtered_m_s_ += alpha * (in.ground_speed_m_s - speed_filtered_m_s_);
      if (phase_ == Phase::Catching && config_.coordinated_speed && config_.catch_speed_integral_gain > 0.0F)
        speed_integral_rad_ = std::clamp(speed_integral_rad_ -
            config_.catch_speed_integral_gain * speed_filtered_m_s_ / 9.80665F * dt,
            -config_.catch_speed_integral_max_rad, config_.catch_speed_integral_max_rad);
    } else {
      speed_filtered_m_s_ = in.ground_speed_m_s;
    }
    out.speed_filtered_m_s = speed_filtered_m_s_;
    out.speed_integral_rad = speed_integral_rad_;
    elapsed_s_ += dt;
    const bool catching_back = (phase_ == Phase::Catching || phase_ == Phase::Support) &&
        in.pitch_rad < 0.0F && config_.back_pitch_limit_rad > 0.0F;
    const float pitch_limit = phase_ == Phase::Folding ? config_.fold_pitch_limit_rad :
        catching_back ? std::max(config_.pitch_limit_rad, config_.back_pitch_limit_rad) :
        config_.pitch_limit_rad;
    const bool moving = phase_ == Phase::Folding || phase_ == Phase::Pitching || phase_ == Phase::Catching;
    if (std::abs(in.pitch_rad) > pitch_limit || (moving && elapsed_s_ > config_.timeout_s)) {
      phase_ = Phase::Failed;
      failure_ = std::abs(in.pitch_rad) > pitch_limit ? Failure::PitchLimit : Failure::Timeout;
      return output(out);
    }
    // Divergent-mode/capture-point gate, following Pratt et al., Humanoids 2006,
    // doi:10.1109/ICHR.2006.321385. The handoff gate uses a small-angle
    // capture region. Hip feedback below is a bounded angular approximation,
    // not a proof of capturability or ground contact with moving hip/wheels.
    out.capture_point_rad = in.capture_lean_rad + in.capture_rate_rad_s / in.capture_omega_rad_s;
    const float previous_hip = hip_cmd_;
    if (phase_ == Phase::Folding) {
      const float remaining = std::max(config_.fold_travel_rad - std::abs(hip_cmd_ - hip_start_rad_), 0.0F);
      const float step = std::min(config_.fold_rate_rad_s * dt, remaining);
      hip_cmd_ += (config_.fold_direction < 0.0F ? -1.0F : 1.0F) * step;
      if (step >= remaining) {
        phase_ = Phase::Kneeling;
        elapsed_s_ = 0.0F;
      }
    } else if (phase_ == Phase::Kneeling) {
      if (!config_.hold_at_kneel && elapsed_s_ >= config_.kneel_hold_s) {
        if (config_.skip_pitching) {
          phase_ = Phase::Catching;
          catch_overlap_ = true;  // blend the catch in (catch_blend_s) from the still hip
          catch_entry_rate_ = 0.0F;
          catch_start_hip_rad_ = hip_cmd_;
          elapsed_s_ = pitch_settle_s_ = 0.0F;
          wheel_velocity_m_s_ = std::clamp(in.ground_speed_m_s,
              -config_.catch_wheel_max_rad_s * in.wheel_radius_m,
              config_.catch_wheel_max_rad_s * in.wheel_radius_m);
        } else {
          advanceFromKneeling();
        }
      }
    } else if (phase_ == Phase::Pitching) {
      if (!flip_pitch_initialized_) {
        flip_start_pitch_rad_ = in.pitch_rad;
        flip_pitch_initialized_ = true;
      }
      const float rate = std::clamp(config_.flip_pitch_kp_rad_s_per_rad * in.pitch_rad +
          config_.flip_pitch_kd * in.pitch_rate_rad_s,
          -config_.flip_hip_rate_max_rad_s, config_.flip_hip_rate_max_rad_s);
      hip_cmd_ = std::clamp(hip_cmd_ + rate * dt,
          flip_start_hip_rad_ - config_.flip_travel_max_rad,
          flip_start_hip_rad_ + config_.flip_travel_max_rad);
      const bool level = std::abs(in.pitch_rad) <= config_.pitch_target_tol_rad &&
          std::abs(in.pitch_rate_rad_s) <= config_.pitch_rate_tol_rad_s;
      pitch_settle_s_ = level ? pitch_settle_s_ + dt : 0.0F;
      const bool overlap = config_.overlap_after_lift && !config_.hold_after_flip;
      const bool lifted = out.body_clearance_valid &&
          in.body_clearance_m >= config_.lift_clearance_m &&
          (std::abs(flip_start_pitch_rad_) - std::abs(in.pitch_rad) >= config_.lift_progress_rad ||
           std::abs(in.pitch_rad) <= config_.pitch_target_tol_rad) &&
          in.pitch_rad * in.pitch_rate_rad_s <= 0.0F;
      lift_clear_s_ = lifted ? lift_clear_s_ + dt : 0.0F;
      // Overlap deliberately does not wait for pitch zero or a stationary
      // body. Missing clearance never falls back to a timed blind launch.
      const bool ready = overlap ? (lifted && lift_clear_s_ >= config_.lift_confirm_s)
                                 : (pitch_settle_s_ >= config_.pitch_settle_s);
      if (ready) {
        phase_ = config_.hold_after_flip ? Phase::Flipped : Phase::Catching;
        catch_overlap_ = overlap;
        catch_entry_rate_ = dt > 0.0F ? (hip_cmd_ - previous_hip) / dt : 0.0F;
        catch_start_hip_rad_ = hip_cmd_;
        elapsed_s_ = pitch_settle_s_ = 0.0F;
        wheel_velocity_m_s_ = std::clamp(in.ground_speed_m_s,
            -config_.catch_wheel_max_rad_s * in.wheel_radius_m,
            config_.catch_wheel_max_rad_s * in.wheel_radius_m);
      }
    } else if (phase_ == Phase::Catching || supported_hip) {
      if (rolling_transfer) {
        // Pratt's divergent-mode gate plus a clearance hypothesis from the
        // existing IMU/encoder model. A zero-angle crossing alone cannot arm
        // this transfer. Robot log 193036 reaches this region while rolling,
        // but the old hip speed-to-lean task keeps lifting until a rear-floor
        // fault; the wheel stopping controller never receives ownership.
        const bool reserve = out.body_clearance_valid &&
            in.body_clearance_m >= config_.transfer_clearance_m &&
            in.floor_bounds_valid && in.floor_bounds_feasible && floor_armed_ &&
            in.floor_predicted_clearance_m >= config_.transfer_floor_reserve_m;
        const bool captured = in.dt_s <= 0.01F && reserve &&
            in.hip_measured_rad - catch_start_hip_rad_ >= 0.35F &&
            std::abs(in.capture_lean_rad) <= config_.catch_lean_tol_rad &&
            std::abs(out.capture_point_rad) <= config_.catch_capture_tol_rad &&
            std::abs(in.capture_rate_rad_s) <= config_.transfer_capture_rate_rad_s;
        if (!transfer_captured_) {
          transfer_dwell_s_ = captured ? transfer_dwell_s_ + dt : 0.0F;
          if (captured && transfer_dwell_s_ >= config_.transfer_confirm_s)
            transfer_captured_ = true;
        }
        // Hysteresis prevents task switching on encoder noise. Restore the
        // capture allocator if motion leaves the larger region; raw fall and
        // floor guards below retain priority throughout the ramp.
        if (in.dt_s > 0.01F || !out.body_clearance_valid ||
            in.body_clearance_m < config_.transfer_floor_reserve_m ||
            std::abs(in.capture_lean_rad) > 1.5F * config_.catch_lean_tol_rad ||
            std::abs(out.capture_point_rad) > 2.0F * config_.catch_capture_tol_rad ||
            std::abs(in.capture_rate_rad_s) > 2.0F * config_.transfer_capture_rate_rad_s) {
          transfer_captured_ = false;
          transfer_dwell_s_ = 0.0F;
        }
        transfer_weight_ += std::clamp((transfer_captured_ ? 1.0F : 0.0F) - transfer_weight_,
            -dt / config_.transfer_ramp_s, dt / config_.transfer_ramp_s);
      } else {
        transfer_captured_ = false;
        transfer_dwell_s_ = transfer_weight_ = 0.0F;
      }
      if (supported_hip) {
        // LQR now owns wheel speed/station regulation. Follow its physical
        // lean reference instead of running a second stopping-speed target.
        out.capture_lean_target_rad = in.support_lean_ref_rad;
      } else if (config_.coordinated_speed) {
        // Same speed-error -> desired lean allocation as the existing PID
        // cascade / LqrBalancer::Output::velocity_error_m_s contract, and
        // atan(a/g) used by V3's maneuver preview. This is a CoM target, not
        // body leveling or a wheel braking pulse. For v<0 the target is >0.
        out.capture_lean_target_rad = speedLeanRef();
      }
      // The learned angle is an effective balance reference, not a measured
      // displacement of the battery/COM. Support follows LQR's own trim only.
      if (config_.catch_capture_velocity && !supported_hip)
        out.capture_lean_target_rad += capture_equilibrium_rad_;
      // If body leveling cancels hip progress while CoM remains outside the
      // capture region, temporarily prioritize capture. This progress test
      // uses measured motion, not a simulator contact or a scripted sweep.
      if (std::abs(in.capture_lean_rad) < 0.08F) catch_leveling_ = true;
      const bool stalled = std::abs(in.capture_lean_rad) > 0.12F &&
          std::abs(in.capture_rate_rad_s) < 0.15F && std::abs(hip_rate_cmd_) < 0.25F;
      catch_stall_s_ = stalled ? catch_stall_s_ + dt : 0.0F;
      if (catch_stall_s_ > 0.15F) catch_leveling_ = false;
      // Task priority, not an extra reflex: retain the existing physical-CoM
      // feedback (Pratt et al., above), but remove BOTH body-posture terms.
      // The 18/10 log shows capture and leveling can oppose each other well
      // before support is established. Hard pitch/travel limits still apply.
      float body_rate = config_.capture_first ? 0.0F :
          (catch_leveling_ ? config_.catch_body_gain * in.pitch_rad : 0.0F) +
          config_.catch_body_damping_s * in.pitch_rate_rad_s;
      if (config_.catch_body_leveling) {
        // Hierarchical wheel balance / hip posture allocation as in V3's
        // existing gimbal layer (V3.3_HANDOFF section 2). Robot log 185251:
        // B4 dives before Support, so waiting for Support alone is too late.
        // Encoder progress + modeled clearance are NOT contact detection.
        if (out.body_clearance_valid && in.body_clearance_m >= config_.body_level_arm_clearance_m &&
            in.hip_measured_rad - catch_start_hip_rad_ >= config_.body_level_arm_hip_rad)
          body_level_armed_ = true;
        if (body_level_armed_)
          body_level_weight_ = std::min(1.0F, body_level_weight_ + dt / 0.08F);
        // During initial lifting, keep the one-sided anti-dive correction.
        // Once captured, signed posture control can brake backward body
        // rotation before it reaches the floor. The lift itself is unchanged.
        if (supported_hip)
          body_support_weight_ = std::min(1.0F, body_support_weight_ + dt / 0.25F);
        const float posture_rate = config_.recovery_body_gain * in.pitch_rad +
            config_.recovery_body_damping_s * body_rate_filtered_;
        const float signed_weight = (config_.catch_lift && config_.lift_signed_posture)
            ? 1.0F : std::max(body_support_weight_, transfer_weight_);
        body_rate = body_level_weight_ * std::clamp(posture_rate,
            -config_.recovery_body_rate_max_rad_s * signed_weight,
            config_.recovery_body_rate_max_rad_s);
        out.body_level_weight = body_level_weight_;
        out.body_level_rate_rad_s = body_rate;
        out.body_rate_filtered_rad_s = body_rate_filtered_;
      }
      // Use the existing angular capture-point approximation (Pratt et al.,
      // DOI above) as hip feedback, not only as a handoff gate. Log 181931
      // crosses lean=0 with >1.5 rad/s: angle zero is not captured motion.
      // Filter this stronger rate path; hard gates/floor still use raw rates.
      out.hip_capture_rate_rad_s = config_.catch_capture_point_hip
          ? hip_capture_rate_filtered_ : in.capture_rate_rad_s;
      out.hip_capture_error_rad = in.capture_lean_rad - out.capture_lean_target_rad;
      if (config_.catch_capture_point_hip)
        out.hip_capture_error_rad += out.hip_capture_rate_rad_s / in.capture_omega_rad_s;
      const float damping = config_.catch_capture_point_hip ? 0.0F :
          config_.catch_hip_damping_s * in.capture_rate_rad_s;
      float feedback_rate = std::clamp(
          -(1.0F - transfer_weight_) *
              (config_.catch_hip_gain * out.hip_capture_error_rad + damping) / in.hip_jacobian +
          body_rate,
          -config_.catch_hip_rate_max_rad_s, config_.catch_hip_rate_max_rad_s);
      // Preserve the existing capture-before-posture priority during the
      // transfer, too. Log 201044 at B4 .996 s: lean=-.058, CP=-.093,
      // posture requests hip=-.454 rad/s and pulls COM farther backwards.
      // Project only an outward total rate driven by outward posture. Use
      // the measured Jacobian (no hard-coded hip sign); do not touch initial
      // lifting or Support, where LQR owns the coordinated lean reference.
      if (config_.catch_posture_capture_guard && rolling_transfer && !supported_hip &&
          transfer_weight_ > 0.0F && in.capture_lean_rad * out.capture_point_rad > 0.0F &&
          in.capture_lean_rad * in.hip_jacobian * body_rate > 0.0F &&
          in.capture_lean_rad * in.hip_jacobian * feedback_rate > 0.0F) {
        const float weight = std::clamp((std::abs(in.capture_lean_rad) - 0.015F) / 0.015F, 0.0F, 1.0F);
        feedback_rate *= 1.0F - weight;
        out.posture_capture_limited = weight > 0.0F;
      }
      float hip_accel = config_.catch_hip_accel_rad_s2;
      if (config_.catch_lift && phase_ == Phase::Catching) {
        lift_s_ += dt;
        const bool cleared = out.body_clearance_valid &&
            in.body_clearance_m >= config_.lift_target_clearance_m;
        b4_clear_s_ = cleared ? b4_clear_s_ + dt : 0.0F;
        if (b4_clear_s_ >= 0.03F) lift_done_ = true;
        const bool timed = lift_s_ < config_.lift_max_s;
        const float hip_rise = in.hip_measured_rad - catch_start_hip_rad_;
        const bool hip_floor = timed && hip_rise < config_.lift_hip_travel_rad &&
            out.capture_point_rad < config_.lift_cp_max_rad &&
            !(lift_done_ && hip_rise >= config_.lift_hip_min_travel_rad);
        const float lift_blend = catchBlend();
        // Fade the floor in and out instead of switching it: no rate jerk.
        const float fade = config_.lift_hip_release_s > 0.0F ? dt / config_.lift_hip_release_s : 1.0F;
        lift_hip_weight_ = std::clamp(lift_hip_weight_ + (hip_floor ? fade : -fade), 0.0F, 1.0F);
        if (lift_hip_weight_ > 0.0F) {
          feedback_rate = std::max(feedback_rate,
              config_.lift_hip_rate_min_rad_s * lift_blend * lift_hip_weight_);
          hip_accel = config_.lift_hip_accel_rad_s2;
        }
        // Knee: open while the modelled clearance is short; never refold.
        const bool open_knee = (timed || config_.lift_knee_full) && (!cleared || config_.lift_knee_full) &&
            in.pitch_rad > config_.lift_knee_pitch_stop_rad &&
            (config_.lift_knee_stage1_rad <= 0.0F ||
             knee_cmd_ < knee_start_rad_ + config_.lift_knee_stage1_rad ||
             in.capture_lean_rad > config_.lift_knee_lean_gate_rad) &&
            knee_cmd_ < knee_start_rad_ + config_.lift_knee_max_rad - 1.0e-4F;
        const float knee_target = open_knee ? config_.lift_knee_rate_rad_s * lift_blend : 0.0F;
        knee_rate_cmd_ += std::clamp(knee_target - knee_rate_cmd_,
            -config_.lift_knee_accel_rad_s2 * dt, config_.lift_knee_accel_rad_s2 * dt);
        const float knee_lower = std::max(knee_start_rad_,
            in.knee_measured_rad - config_.lift_knee_tracking_max_rad);
        const float knee_upper = std::min(knee_start_rad_ + config_.lift_knee_max_rad,
            in.knee_measured_rad + config_.lift_knee_tracking_max_rad);
        const float knee_next = std::clamp(knee_cmd_ + knee_rate_cmd_ * dt,
            knee_lower, std::max(knee_lower, knee_upper));
        if (knee_next != knee_cmd_ + knee_rate_cmd_ * dt) knee_rate_cmd_ = 0.0F;  // no windup
        knee_cmd_ = knee_next;
        if (config_.lift_head_rate_limit_rad_s > 0.0F && in.pitch_rate_rad_s < -config_.lift_head_rate_limit_rad_s)
          feedback_rate *= std::max(0.0F, 1.0F - (-in.pitch_rate_rad_s - config_.lift_head_rate_limit_rad_s) /
              config_.lift_head_rate_limit_rad_s);
        if (lift_s_ < config_.lift_hip_delay_s) feedback_rate = 0.0F;
        feedback_rate -= config_.lift_pitch_neutral * knee_rate_cmd_;
        out.lift_state = hip_floor ? 1 : lift_done_ ? 3 : 2;
      }
      out.knee_rate_rad_s = knee_rate_cmd_;
      out.hip_feedback_rate_rad_s = feedback_rate;
      const float blend = catchBlend();
      // Continue the ongoing lift velocity while capture progressively takes
      // over. The acceleration bound brakes an inherited fast flip velocity;
      // the steady catch speed cap applies to the feedback target.
      const float rate = (1.0F - blend) * catch_entry_rate_ + blend * feedback_rate;
      const float previous_hip_rate = hip_rate_cmd_;
      hip_rate_cmd_ += std::clamp(rate - hip_rate_cmd_,
          -hip_accel * dt, hip_accel * dt);
      // Intersect travel and tracking bounds; never wind up against a stall
      // or silently extend the allowed travel when encoder tracking is lost.
      const float lower = std::max(catch_start_hip_rad_ - config_.catch_hip_travel_rad,
          in.hip_measured_rad - config_.catch_hip_tracking_max_rad);
      const float upper = std::min(catch_start_hip_rad_ + config_.catch_hip_travel_rad,
          in.hip_measured_rad + config_.catch_hip_tracking_max_rad);
      if (lower > upper) {
        phase_ = Phase::Failed;
        failure_ = Failure::HipTracking;
        return output(out);
      }
      const float requested_hip = hip_cmd_ + hip_rate_cmd_ * dt;
      float filtered_hip = requested_hip;
      if (config_.floor_guard && (!supported_hip || config_.floor_guard_in_support)) {
        out.floor_bounds_valid = in.floor_bounds_valid;
        out.floor_clearance_rate_m_s = in.floor_clearance_rate_m_s;
        out.floor_predicted_clearance_m = in.floor_predicted_clearance_m;
        out.floor_hip_rate_lower_rad_s = in.floor_hip_rate_lower_rad_s;
        out.floor_hip_rate_upper_rad_s = in.floor_hip_rate_upper_rad_s;
        // Do not demand positive clearance in the initial lying/contact pose.
        // Arm only after measured kinematics have cleared the shell for a
        // dwell; latch across Support so dropping below it cannot disarm us.
        const bool lifted = out.body_clearance_valid &&
            in.body_clearance_m >= config_.floor_arm_clearance_m;
        floor_arm_s_ = lifted ? floor_arm_s_ + dt : 0.0F;
        if (lifted && floor_arm_s_ >= config_.floor_arm_dwell_s) floor_armed_ = true;
        out.floor_armed = floor_armed_;
        if (floor_armed_ && dt > 0.0F) {
          // Intersect floor bounds with ALL existing velocity, acceleration,
          // tracking and travel bounds. Do not bypass an actuator limit to
          // satisfy geometry or jump the position target at guard activation.
          const float position_rate_lower = std::max(-config_.catch_hip_rate_max_rad_s,
              (lower - hip_cmd_) / dt);
          const float position_rate_upper = std::min(config_.catch_hip_rate_max_rad_s,
              (upper - hip_cmd_) / dt);
          const float actuator_lower = std::max(position_rate_lower,
              previous_hip_rate - hip_accel * dt);
          const float actuator_upper = std::min(position_rate_upper,
              previous_hip_rate + hip_accel * dt);
          const float rate_lower = std::max(actuator_lower, in.floor_hip_rate_lower_rad_s);
          const float rate_upper = std::min(actuator_upper, in.floor_hip_rate_upper_rad_s);
          out.floor_hip_rate_lower_rad_s = rate_lower;
          out.floor_hip_rate_upper_rad_s = rate_upper;
          float filtered_rate = hip_rate_cmd_;
          if (!in.floor_bounds_feasible || rate_lower > rate_upper) {
            out.floor_infeasible = true;
            // Log 175452: floor asks >=0.521 rad/s, the acceleration bound
            // permits <=0.452, with 3.19 mm predicted clearance (>2 mm).
            // Treat ONLY an acceleration conflict as saturation while there
            // is clearance reserve. Move toward the floor-feasible interval
            // using the closest reachable rate; never jump the servo target.
            const float floor_position_lower = std::max(position_rate_lower,
                in.floor_hip_rate_lower_rad_s);
            const float floor_position_upper = std::min(position_rate_upper,
                in.floor_hip_rate_upper_rad_s);
            const bool accel_only = config_.floor_accel_grace &&
                in.floor_bounds_feasible && actuator_lower <= actuator_upper &&
                floor_position_lower <= floor_position_upper &&
                in.floor_predicted_clearance_m > config_.floor_clearance_m;
            if (!accel_only) {
              phase_ = Phase::Failed;
              failure_ = Failure::FloorConstraint;
              // Retain the previous position target; Fault stops the wheels.
              // Neither stopping nor saturation guarantees no floor contact.
              hip_rate_cmd_ = 0.0F;
              return output(out);
            }
            const float floor_rate = std::clamp(hip_rate_cmd_,
                floor_position_lower, floor_position_upper);
            filtered_rate = std::clamp(floor_rate, actuator_lower, actuator_upper);
            out.floor_accel_saturated = true;
          } else {
            filtered_rate = std::clamp(hip_rate_cmd_, rate_lower, rate_upper);
          }
          out.floor_limited = out.floor_accel_saturated ||
              std::abs(filtered_rate - hip_rate_cmd_) > 1.0e-5F;
          filtered_hip = hip_cmd_ + filtered_rate * dt;
        }
      }
      hip_cmd_ = std::clamp(filtered_hip, lower, upper);
      out.hip_limited = requested_hip < lower || requested_hip > upper;
      out.hip_limited = out.hip_limited || out.floor_limited;
      const bool rolling_ready = rolling_transfer && transfer_captured_ && transfer_weight_ >= 0.999F &&
          std::abs(in.capture_rate_rad_s) <= config_.transfer_capture_rate_rad_s &&
          std::abs(in.pitch_rad) <= config_.handoff_body_tol_rad &&
          in.body_clearance_m >= config_.transfer_clearance_m &&
          in.floor_predicted_clearance_m >= config_.transfer_floor_reserve_m &&
          std::abs(in.ground_speed_m_s) <= rollingHandoffSpeed(in.wheel_radius_m);
      out.rolling_handoff = config_.catch_rolling_handoff && rolling_ready;
      out.handoff_ready = !supported_hip && blend >= 1.0F &&
          std::abs(in.capture_lean_rad) <= config_.catch_lean_tol_rad &&
          std::abs(out.capture_point_rad) <= config_.catch_capture_tol_rad &&
          std::abs(in.capture_rate_rad_s) <= config_.catch_rate_tol_rad_s &&
          (config_.capture_first || std::abs(in.pitch_rad) <= config_.catch_body_tol_rad) &&
          (std::abs(in.ground_speed_m_s) <= config_.catch_speed_tol_m_s || rolling_ready) &&
          std::abs(hip_cmd_ - in.hip_measured_rad) < 0.06F;
      if (config_.floor_guard)
        out.handoff_ready = out.handoff_ready && floor_armed_ && !out.floor_limited &&
            in.floor_predicted_clearance_m >= config_.floor_clearance_m;
    }
    if (dt > 0.0F) hip_rate_cmd_ = (hip_cmd_ - previous_hip) / dt;
    if (phase_ == Phase::Catching) {
      const float blend = catchBlend(true);
      out.catch_blend = blend;
      out.speed_feedback_accel_m_s2 = config_.coordinated_speed
          ? 0.0F : config_.catch_speed_gain * in.ground_speed_m_s;
      out.wheel_raw_accel_m_s2 = std::clamp(config_.catch_accel_gain * in.capture_lean_rad +
          config_.catch_accel_damping * in.capture_rate_rad_s +
          out.speed_feedback_accel_m_s2,
          -config_.catch_accel_max, config_.catch_accel_max);
      float wheel_lean = in.capture_lean_rad, wheel_rate = in.capture_rate_rad_s;
      float forecast_speed = in.ground_speed_m_s;
      if (config_.catch_wheel_prediction) {
        const auto future = config_.catch_measured_braking
            ? wheel_predictor_.predictMeasuredMotion(in.capture_lean_rad, in.capture_rate_rad_s,
                in.ground_speed_m_s, config_.catch_wheel_delay_s)
            : wheel_predictor_.predict(in.capture_lean_rad, in.capture_rate_rad_s,
            in.ground_speed_m_s, in.wheel_radius_m * in.carrier_rate_rad_s,
            in.capture_omega_rad_s, config_.catch_wheel_delay_s,
            config_.catch_wheel_lag_s, config_.catch_accel_max);
        out.wheel_measured_preview = config_.catch_measured_braking;
        if (!future.valid) capture_equilibrium_dwell_s_ = 0.0F;
        out.wheel_prediction_valid = future.valid;
        out.wheel_predicted_lean_rad = future.lean;
        out.wheel_predicted_rate_rad_s = future.rate;
        out.wheel_predicted_speed_m_s = future.speed;
        out.wheel_observed_accel_m_s2 = future.measured_accel;
        out.wheel_observed_angular_accel_rad_s2 = future.angular_accel;
        // Eligibility for a local extrapolation, NOT detection of contact.
        // Latch so approaching the floor cannot abruptly remove the preview.
        if (out.body_clearance_valid && in.body_clearance_m >= 0.008F &&
            std::abs(in.ground_speed_m_s) >= 0.08F &&
            std::abs(in.hip_measured_rad - catch_start_hip_rad_) >= 0.12F)
          wheel_prediction_armed_ = true;
        const float target_weight = wheel_prediction_armed_ && future.valid ? 1.0F : 0.0F;
        wheel_prediction_weight_ += std::clamp(target_weight - wheel_prediction_weight_,
            -dt / 0.06F, dt / 0.06F);
        out.wheel_prediction_weight = future.valid ? wheel_prediction_weight_ : 0.0F;
        // Bound extrapolation authority without clipping raw safety signals.
        const float lean_delta = std::clamp(future.lean - in.capture_lean_rad, -0.12F, 0.12F);
        const float rate_delta = std::clamp(future.rate - in.capture_rate_rad_s, -1.0F, 1.0F);
        out.wheel_prediction_capped = future.valid &&
            (std::abs(future.lean - in.capture_lean_rad) > 0.12F ||
             std::abs(future.rate - in.capture_rate_rad_s) > 1.0F);
        if (future.valid) {
          wheel_lean += out.wheel_prediction_weight * lean_delta;
          wheel_rate += out.wheel_prediction_weight * rate_delta;
          forecast_speed += std::clamp(future.speed - in.ground_speed_m_s,
              -config_.catch_tracking_margin_m_s, config_.catch_tracking_margin_m_s);
          if (config_.catch_capture_velocity) {
            // Quasi-steady residual observer, NOT COM identification during
            // lift/contact changes. theta_ddot=omega² sin(theta)-a/L+d;
            // equilibrium=asin(sin(theta)-a/g-theta_ddot/omega²).
            // Freeze during large hip/body motion, poor tracking or saturation.
            const bool quiet = out.body_clearance_valid && in.body_clearance_m >= 0.012F &&
                in.hip_measured_rad - catch_start_hip_rad_ >= 0.35F &&
                std::abs(in.hip_measured_rate_rad_s) <= 0.20F &&
                std::abs(in.pitch_rate_rad_s) <= 0.25F &&
                std::abs(in.capture_rate_rad_s) <= 0.25F &&
                std::abs(in.capture_lean_rad) <= 0.12F &&
                std::abs(in.ground_speed_m_s) <= 0.20F &&
                std::abs(future.measured_accel) <= 0.50F &&
                std::abs(future.angular_accel) <= 2.0F &&
                std::abs(wheel_velocity_m_s_ - in.ground_speed_m_s) <= 0.08F &&
                !out.hip_limited && !out.floor_limited;
            capture_equilibrium_dwell_s_ = quiet ? capture_equilibrium_dwell_s_ + dt : 0.0F;
            if (quiet && capture_equilibrium_dwell_s_ >= 0.25F) {
              const float omega2 = in.capture_omega_rad_s * in.capture_omega_rad_s;
              const float observed = std::clamp(std::asin(std::clamp(std::sin(in.capture_lean_rad) -
                  future.measured_accel / 9.80665F - future.angular_accel / omega2,
                  -0.06F, 0.06F)), -0.06F, 0.06F);
              // <=0.02 rad/s, <=0.06 rad (~3.4 degrees); no windup at a clamp.
              capture_equilibrium_rad_ += std::clamp(
                  dt / (1.0F + dt) * (observed - capture_equilibrium_rad_),
                  -0.02F * dt, 0.02F * dt);
              out.capture_equilibrium_learning = true;
            }
          }
        }
      }
      // Same stopping target the hip follows, so both actuators agree on it.
      // For v<0 the wheels first roll back faster to tip the CoM forward,
      // then gravity on the forward lean brakes (non-minimum phase).
      const float wheel_lean_ref = config_.catch_wheel_speed_lean && config_.coordinated_speed
          ? speedLeanRef() : 0.0F;
      out.wheel_lean_ref_rad = wheel_lean_ref;
      const float legacy_accel = std::clamp(config_.catch_accel_gain * (wheel_lean - wheel_lean_ref) +
          config_.catch_accel_damping * wheel_rate + out.speed_feedback_accel_m_s2,
          -config_.catch_accel_max, config_.catch_accel_max);
      const float servo_weight = config_.catch_capture_velocity ? out.wheel_prediction_weight : 0.0F;
      // Pratt et al., Humanoids 2006, DOI 10.1109/ICHR.2006.321385:
      // divergent mode e=theta+theta_dot/omega. With a lagged speed actuator,
      // v*=v_pred+tau*g*sin(theta)+L*(theta_dot+omega*theta), L=g/omega².
      // Prior direct-speed mode gives theta_ddot=-(theta_dot+omega*theta)/tau
      // only in the unsaturated ideal actuator model. Measured braking uses
      // this bounded acceleration demand with command integration below;
      // that ideal pole assignment does not apply to the integrated path.
      const float theta = wheel_lean - capture_equilibrium_rad_;
      const float omega = in.capture_omega_rad_s;
      const float length = 9.80665F / (omega * omega);
      float servo_accel = legacy_accel;
      if (servo_weight > 0.0F)
        servo_accel = std::clamp(9.80665F * std::sin(theta) +
            length / config_.catch_wheel_lag_s * (wheel_rate + omega * (theta - wheel_lean_ref)),
            -config_.catch_accel_max, config_.catch_accel_max);
      float accel = (1.0F - servo_weight) * legacy_accel + servo_weight * servo_accel;
      if (config_.catch_lift && config_.lift_wheel_assist != 0.0F && lift_hip_weight_ > 0.0F)
        accel = std::clamp(accel - config_.lift_wheel_assist * lift_hip_weight_ * std::max(hip_rate_cmd_, 0.0F),
            -config_.catch_accel_max, config_.catch_accel_max);
      out.wheel_accel_m_s2 = accel;
      const float vmax = config_.catch_wheel_max_rad_s * in.wheel_radius_m;
      // Bounds remain ordered even if the robot is rolling faster than vmax.
      wheel_velocity_m_s_ = std::clamp(wheel_velocity_m_s_ + blend * accel * dt,
          std::max(-vmax, std::min(in.ground_speed_m_s, 0.0F) - config_.catch_tracking_margin_m_s),
          std::min(vmax, std::max(in.ground_speed_m_s, 0.0F) + config_.catch_tracking_margin_m_s));
      // Preserve launch assist before preview eligibility. Measured braking
      // retains the launch lead rather than shrinking 75 ms to a modeled
      // 30 ms while the robot is already accelerating backwards.
      const float predicted_lead = config_.catch_measured_braking
          ? std::max(config_.catch_wheel_lead_s, config_.catch_wheel_lag_s)
          : config_.catch_wheel_lag_s;
      const float wheel_lead = config_.catch_wheel_prediction
          ? config_.catch_wheel_lead_s + out.wheel_prediction_weight *
              (predicted_lead - config_.catch_wheel_lead_s)
          : config_.catch_wheel_lead_s;
      float servo_speed = 0.0F;
      if (config_.catch_measured_braking)
        servo_speed = wheel_velocity_m_s_ + accel * wheel_lead;
      else if (servo_weight > 0.0F)
        servo_speed = std::clamp(forecast_speed + config_.catch_wheel_lag_s * servo_accel,
            std::max(-vmax, std::min(in.ground_speed_m_s, 0.0F) - config_.catch_tracking_margin_m_s),
            std::min(vmax, std::max(in.ground_speed_m_s, 0.0F) + config_.catch_tracking_margin_m_s));
      const float requested_ground_speed = (1.0F - servo_weight) *
          (wheel_velocity_m_s_ + accel * wheel_lead) + servo_weight * servo_speed;
      out.capture_servo_weight = servo_weight;
      out.capture_servo_speed_m_s = servo_speed;
      out.capture_integrated_speed_m_s = wheel_velocity_m_s_;
      // Prior mode tracks the forecast baseline while its servo owns speed.
      // With standbrake, integrate the requested acceleration persistently.
      // Resetting to a still-decreasing measured/forecast velocity each tick
      // can move the request backwards even while asking for +3 m/s² braking
      // (robot log 192223, B4 .417 -> .458 s). Existing bounds above provide
      // anti-windup; do not put the stale negative velocity back into the sum.
      if (!config_.catch_measured_braking)
        wheel_velocity_m_s_ += servo_weight * (forecast_speed - wheel_velocity_m_s_);
      const float requested_wheel = requested_ground_speed / in.wheel_radius_m -
              in.carrier_rate_rad_s;
      out.wheel_rad_s = blend * std::clamp(requested_wheel,
          -config_.catch_wheel_max_rad_s, config_.catch_wheel_max_rad_s);
      out.wheel_limited = std::abs(requested_wheel) >= config_.catch_wheel_max_rad_s;
      out.wheel_tracking_error_rad_s = std::max(
          std::abs(out.wheel_rad_s - in.wheel_measured_rad_s[0]),
          std::abs(out.wheel_rad_s - in.wheel_measured_rad_s[1]));
      if (config_.guarded_handoff) {
        out.handoff_ready = out.handoff_ready && out.body_clearance_valid &&
            in.body_clearance_m >= config_.lift_clearance_m &&
            (config_.capture_first || std::abs(in.pitch_rad) <= config_.handoff_body_tol_rad) &&
            std::abs(in.pitch_rate_rad_s) <= config_.handoff_body_rate_tol_rad_s &&
            std::abs(in.hip_measured_rate_rad_s) <= config_.handoff_hip_rate_tol_rad_s &&
            std::abs(hip_rate_cmd_) <= config_.handoff_hip_rate_tol_rad_s &&
            out.wheel_tracking_error_rad_s <= config_.handoff_wheel_tracking_tol_rad_s &&
            !out.hip_limited && (!out.wheel_limited || out.rolling_handoff);
      }
      pitch_settle_s_ = out.handoff_ready ? pitch_settle_s_ + dt : 0.0F;
      // Log 200417: posture transfer is complete, COM/rates are inside the
      // capture gates, but B4's -16 command repeatedly clears the 200 ms
      // timer. Its angle-only wheel loop keeps rolling; LQR never gets the
      // measured velocity/planner seed needed for stopping. After the prior
      // 40 ms capture confirmation + 120 ms posture ramp, require another
      // 80 ms of ALL raw state/tracking/floor gates. Cap contact alone is
      // not evidence of a lost capture. No shorter dwell on other paths.
      const float guarded_settle = out.rolling_handoff
          ? config_.rolling_handoff_settle_s : config_.handoff_settle_s;
      const float settle = config_.guarded_handoff
          ? std::max(config_.catch_settle_s, guarded_settle) : config_.catch_settle_s;
      out.handoff_settle_required_s = settle;
      if (out.handoff_ready && pitch_settle_s_ >= settle) {
        phase_ = Phase::Support;
        // Legacy path returned zero on the Support tick. The guarded path
        // retains its last catch command for consumers that blend the handoff.
        if (!config_.guarded_handoff) out.wheel_rad_s = 0.0F;
      }
    }
    return output(out);
  }

  [[nodiscard]] Phase phase() const noexcept { return phase_; }
  [[nodiscard]] float time() const noexcept { return elapsed_s_; }
  [[nodiscard]] Config& config() noexcept { return config_; }
  [[nodiscard]] const Config& config() const noexcept { return config_; }
  static const char* name(Phase phase) noexcept {
    switch (phase) {
      case Phase::Idle: return "Idle";
      case Phase::Folding: return "Folding";
      case Phase::Kneeling: return "Kneeling";
      case Phase::Pitching: return "Pitching";
      case Phase::Flipped: return "Flipped";
      case Phase::Catching: return "Catching";
      case Phase::Support: return "Support";
      case Phase::Failed: return "Failed";
    }
    return "?";
  }
  static const char* failureName(Failure failure) noexcept {
    switch (failure) {
      case Failure::None: return "none";
      case Failure::InvalidInput: return "stand-up invalid input/config";
      case Failure::PitchLimit: return "stand-up pitch limit";
      case Failure::Timeout: return "stand-up phase timeout";
      case Failure::HipTracking: return "stand-up hip tracking outside travel";
      case Failure::FloorConstraint: return "stand-up floor constraint infeasible";
    }
    return "stand-up unknown failure";
  }

 private:
  // Speed error -> CoM lean target, atan(a/g) with a = -catch_speed_gain*v.
  float speedLeanRef() const noexcept {
    return std::clamp(std::atan(-config_.catch_speed_gain * speed_filtered_m_s_ / 9.80665F) + speed_integral_rad_,
        -config_.catch_speed_lean_max_rad, config_.catch_speed_lean_max_rad);
  }

  float rollingHandoffSpeed(float wheel_radius_m) const noexcept {
    const float cap = config_.catch_wheel_max_rad_s * wheel_radius_m + config_.catch_tracking_margin_m_s;
    return config_.rolling_handoff_speed_m_s > 0.0F ? std::min(cap, config_.rolling_handoff_speed_m_s) : cap;
  }

  float catchBlend(bool wheel = false) const noexcept {
    const float duration = wheel && config_.catch_wheel_blend_s > 0.0F
        ? config_.catch_wheel_blend_s : config_.catch_blend_s;
    if (!catch_overlap_ || duration <= 0.0F) return 1.0F;
    const float t = std::clamp(elapsed_s_ / duration, 0.0F, 1.0F);
    return t * t * (3.0F - 2.0F * t);
  }

  bool valid(const Input& in) const noexcept {
    if (config_.catch_rolling_handoff &&
        (!std::isfinite(config_.rolling_handoff_settle_s) || config_.rolling_handoff_settle_s < 0.08F))
      return false;
    if (config_.catch_rolling_transfer) {
      for (float v : {config_.transfer_confirm_s, config_.transfer_ramp_s,
           config_.transfer_clearance_m, config_.transfer_floor_reserve_m,
           config_.transfer_capture_rate_rad_s})
        if (!std::isfinite(v) || v <= 0.0F) return false;
      if (config_.transfer_floor_reserve_m <= config_.floor_clearance_m ||
          config_.transfer_clearance_m <= config_.transfer_floor_reserve_m) return false;
    }
    if (config_.catch_measured_braking &&
        (!config_.catch_wheel_prediction || !config_.catch_capture_velocity)) return false;
    if (config_.catch_capture_velocity &&
        (!config_.catch_wheel_prediction || !std::isfinite(config_.catch_wheel_lag_s) ||
         config_.catch_wheel_lag_s < 0.015F || !std::isfinite(in.hip_measured_rate_rad_s))) return false;
    if (config_.catch_wheel_prediction &&
        (!std::isfinite(config_.catch_wheel_delay_s) || config_.catch_wheel_delay_s < 0.0F ||
         config_.catch_wheel_delay_s > 0.15F || !std::isfinite(config_.catch_wheel_lag_s) ||
         config_.catch_wheel_lag_s < 0.015F || config_.catch_wheel_lag_s > 0.12F ||
         config_.catch_wheel_delay_s + config_.catch_wheel_lag_s > 0.25F)) return false;
    if (config_.catch_lift &&
        (!std::isfinite(in.knee_measured_rad) || !std::isfinite(config_.lift_cp_max_rad) ||
         config_.lift_knee_max_rad > 0.8F || config_.lift_knee_rate_rad_s > 3.0F ||
         config_.lift_knee_accel_rad_s2 <= 0.0F || config_.lift_knee_accel_rad_s2 > 60.0F || config_.lift_hip_rate_min_rad_s >
             config_.catch_hip_rate_max_rad_s)) return false;
    if (config_.catch_capture_point_hip &&
        (!std::isfinite(config_.catch_capture_rate_filter_s) || config_.catch_capture_rate_filter_s < 0.0F))
      return false;
    if (!in.sensors_ok || !std::isfinite(in.dt_s) || in.dt_s < 0.0F ||
        !std::isfinite(in.pitch_rad) || !std::isfinite(in.pitch_rate_rad_s) ||
        !std::isfinite(in.capture_lean_rad) || !std::isfinite(in.capture_rate_rad_s) ||
        !std::isfinite(in.support_lean_ref_rad) ||
        !std::isfinite(in.hip_jacobian) || !std::isfinite(in.hip_measured_rad) ||
        !std::isfinite(in.carrier_rate_rad_s) || !std::isfinite(in.ground_speed_m_s) ||
        !std::isfinite(in.wheel_radius_m) || in.wheel_radius_m <= 0.0F ||
        !std::isfinite(in.capture_omega_rad_s) || in.capture_omega_rad_s <= 0.0F ||
        ((phase_ == Phase::Catching ||
          (phase_ == Phase::Support && config_.support_hip_capture)) &&
         std::abs(in.hip_jacobian) < 0.1F)) return false;
    if (config_.guarded_handoff &&
        (!std::isfinite(in.hip_measured_rate_rad_s) ||
         !std::isfinite(in.wheel_measured_rad_s[0]) ||
         !std::isfinite(in.wheel_measured_rad_s[1]))) return false;
    if (config_.floor_guard) {
      if (!std::isfinite(config_.floor_clearance_m) || config_.floor_clearance_m < 0.0F ||
          !std::isfinite(config_.floor_arm_clearance_m) ||
          config_.floor_arm_clearance_m <= config_.floor_clearance_m ||
          !std::isfinite(config_.floor_arm_dwell_s) || config_.floor_arm_dwell_s < 0.0F ||
          !std::isfinite(config_.floor_approach_time_s) || config_.floor_approach_time_s <= 0.0F ||
          !std::isfinite(config_.floor_prediction_lag_s) || config_.floor_prediction_lag_s < 0.0F)
        return false;
      if ((phase_ == Phase::Catching ||
           (phase_ == Phase::Support && config_.support_hip_capture)) &&
          (!in.body_clearance_valid || !std::isfinite(in.body_clearance_m) ||
           !in.floor_bounds_valid || !std::isfinite(in.floor_clearance_rate_m_s) ||
           !std::isfinite(in.floor_predicted_clearance_m) ||
           !std::isfinite(in.floor_hip_rate_lower_rad_s) ||
           !std::isfinite(in.floor_hip_rate_upper_rad_s))) return false;
    }
    for (float v : {config_.catch_speed_filter_s, config_.catch_speed_lean_max_rad}) {
      if (!std::isfinite(v) || v < 0.0F) return false;
    }
    if (config_.catch_speed_lean_max_rad > 0.12F) return false;
    for (float v : {config_.fold_rate_rad_s, config_.fold_travel_rad, config_.fold_pitch_limit_rad,
         config_.kneel_hold_s, config_.flip_pitch_kp_rad_s_per_rad, config_.flip_pitch_kd,
         config_.flip_hip_rate_max_rad_s, config_.flip_travel_max_rad, config_.pitch_target_tol_rad,
         config_.pitch_rate_tol_rad_s, config_.pitch_settle_s,
         config_.lift_clearance_m, config_.lift_progress_rad, config_.lift_confirm_s,
         config_.catch_blend_s, config_.catch_wheel_blend_s, config_.catch_hip_gain,
         config_.catch_hip_damping_s, config_.catch_body_gain, config_.catch_body_damping_s,
         config_.recovery_body_gain, config_.recovery_body_damping_s,
         config_.recovery_body_rate_max_rad_s,
         config_.catch_wheel_peak_s, config_.catch_hip_rate_max_rad_s,
         config_.catch_hip_accel_rad_s2, config_.catch_hip_travel_rad, config_.catch_hip_tracking_max_rad,
         config_.catch_accel_gain, config_.catch_accel_damping, config_.catch_speed_gain,
         config_.catch_accel_max, config_.catch_wheel_lead_s, config_.catch_wheel_max_rad_s,
         config_.catch_tracking_margin_m_s, config_.catch_lean_tol_rad, config_.catch_capture_tol_rad,
         config_.catch_body_tol_rad, config_.catch_rate_tol_rad_s, config_.catch_speed_tol_m_s,
         config_.catch_settle_s, config_.pitch_limit_rad, config_.back_pitch_limit_rad,
         config_.rolling_handoff_speed_m_s, config_.catch_speed_integral_gain,
         config_.catch_speed_integral_max_rad, config_.body_level_arm_hip_rad,
         config_.body_level_arm_clearance_m, config_.lift_hip_rate_min_rad_s,
         config_.lift_hip_min_travel_rad, config_.lift_hip_accel_rad_s2, config_.lift_hip_release_s,
         config_.lift_hip_travel_rad, config_.lift_max_s, config_.lift_target_clearance_m,
         config_.lift_knee_max_rad, config_.lift_knee_rate_rad_s,
         config_.lift_knee_accel_rad_s2, config_.lift_knee_tracking_max_rad,
         config_.timeout_s}) {
      if (!std::isfinite(v) || v < 0.0F) return false;
    }
    if (config_.guarded_handoff) {
      for (float v : {config_.handoff_settle_s, config_.handoff_body_tol_rad,
           config_.handoff_body_rate_tol_rad_s, config_.handoff_hip_rate_tol_rad_s,
           config_.handoff_wheel_tracking_tol_rad_s}) {
        if (!std::isfinite(v) || v < 0.0F) return false;
      }
    }
    // Software ceiling on one recovery request, not a motor thermal rating.
    return std::isfinite(config_.fold_direction) && config_.catch_wheel_peak_s <= 1.0F;
  }

  Output output(Output out) noexcept {
    out.phase = phase_;
    out.capture_equilibrium_rad = capture_equilibrium_rad_;
    out.transfer_captured = transfer_captured_;
    out.transfer_weight = transfer_weight_;
    out.transfer_confirm_s = transfer_dwell_s_;
    out.failure = failure_;
    out.floor_armed = floor_armed_;
    out.settle_s = pitch_settle_s_;
    out.hip_target_rad = hip_cmd_;
    out.knee_target_rad = knee_cmd_;
    out.wheels_on = phase_ != Phase::Failed && phase_ != Phase::Idle;
    out.hip_rate_rad_s = hip_rate_cmd_;
    out.phase_elapsed_s = elapsed_s_;
    out.wheel_peak_requested = phase_ == Phase::Catching && elapsed_s_ < config_.catch_wheel_peak_s;
    if (config_.catch_wheel_prediction && phase_ != Phase::Idle && phase_ != Phase::Failed)
      wheel_predictor_.record(out.wheel_rad_s * wheel_radius_for_history_);
    return out;
  }

  bool catch_leveling_ = true;
  bool transfer_captured_ = false;
  float transfer_dwell_s_ = 0.0F, transfer_weight_ = 0.0F;
  float capture_equilibrium_rad_ = 0.0F, capture_equilibrium_dwell_s_ = 0.0F;
  bool body_level_armed_ = false, body_rate_initialized_ = false;
  float body_level_weight_ = 0.0F, body_support_weight_ = 0.0F, body_rate_filtered_ = 0.0F;
  RecoveryWheelPredictor wheel_predictor_{};
  bool wheel_prediction_armed_ = false;
  float wheel_prediction_weight_ = 0.0F;
  float wheel_radius_for_history_ = 0.04F;
  bool hip_capture_rate_initialized_ = false;
  float hip_capture_rate_filtered_ = 0.0F;
  bool floor_armed_ = false;
  float floor_arm_s_ = 0.0F;
  bool lift_done_ = false;
  float lift_s_ = 0.0F, b4_clear_s_ = 0.0F, knee_rate_cmd_ = 0.0F, knee_start_rad_ = 0.0F;
  float lift_hip_weight_ = 0.0F;
  float speed_filtered_m_s_ = 0.0F, speed_integral_rad_ = 0.0F;
  bool flip_pitch_initialized_ = false, catch_overlap_ = false;
  float flip_start_pitch_rad_ = 0.0F, lift_clear_s_ = 0.0F, catch_entry_rate_ = 0.0F;
  float catch_stall_s_ = 0.0F;
  Config config_{};
  Phase phase_ = Phase::Idle;
  Failure failure_ = Failure::None;
  float elapsed_s_ = 0.0F, pitch_settle_s_ = 0.0F;
  float hip_start_rad_ = 0.0F, flip_start_hip_rad_ = 0.0F, catch_start_hip_rad_ = 0.0F;
  float hip_cmd_ = 0.0F, knee_cmd_ = 0.0F, hip_rate_cmd_ = 0.0F, wheel_velocity_m_s_ = 0.0F;
};

}  // namespace tap_fw::v3
