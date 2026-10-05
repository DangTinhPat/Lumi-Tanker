#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

#include "robot_control/ImuStateEstimator.hpp"
#include "robot_control/LqrBalancer.hpp"
#include "tap_fw/v3/LegHeight.hpp"
#include "tap_fw/v3/WholeBodyGains.hpp"

namespace tap_fw::v3 {

// v3 controller: sequences the start and runs the layered balance control.
//   Tier 1  wheels: LQR (or PID / whole-body) on the virtual pitch theta_v.
//   Tier 2  hips: reflex + preview + slow level loop (Config::Tier).
//   Knees:  height profile (updateHeight), suspension, knee hold, roll IK.
// All four leg joints are position-driven by LegLock; this class only produces
// their offsets from the power-up pose (Output::leg_*).
//
// Start flow (operator): START -> legs lock -> Ready (wheels hold still, no
// balance loop) -> the operator puts the robot on the floor and holds it still
// (ARMED) -> it engages after Engage::armed_go_s or at the first tip (or on GO).
class V3Controller {
 public:
  enum class State { Idle, Locking, Ready, Balancing, Fault };

  // Lqr (SET algo 0) with the tiers is the shipped balancer. Pid (SET algo 1) is the
  // classic two-loop fallback. WholeBody (SET algo 2) is an opt-in single LQR for
  // wheel + hip + knee (WholeBodyGains.hpp, src_fw/tools/wb_lqr_design.cpp).
  enum class Algo { Lqr, Pid, WholeBody };

  // Classic two-loop balancer: a fast angle PD whose output is wheel
  // acceleration, integrated into the speed command, and a slow speed PI whose
  // output is the angle to hold (its integral finds the balance point). The
  // held/lifted thresholds and deadband compensation below are shared by the
  // LQR and whole-body paths. Units: acceleration m/s^2, angle rad, speed m/s.
  struct Pid {
    float kp = 30.0F;          // m/s^2 per rad of (pitch - pitch_ref)
    // kp 30 / kd 4 / lead 0.12 s stood 318/324 plant variants (mass, inertia,
    // CoM height, wheel lag 0.05-0.2 s, delay 4-40 ms) in the physics tests.
    float kd = 4.0F;           // m/s^2 per rad/s of pitch rate
    float kv = 0.10F;          // rad of lean per m/s of speed error
    float kx = 0.10F;          // rad of lean per m of position error
    // Slow integral of position error into the balance angle theta_b [rad per
    // m*s]; it finds the true balance angle. 0.04 already rings.
    float kix = 0.02F;
    float corr_max_rad = 0.20F; // |kv*ev + kx*ex| clamp; integrator frozen there
    float ref_max_rad = 0.20F;  // |theta_b - seed| limit
    float accel_max_m_s2 = 8.0F;
    // Wheel-lag lead: the sent command leads the integrated speed by
    // accel * lead_s, so a lagging speed servo realises the acceleration sooner.
    float lead_s = 0.12F;
    // Operator drive speed ramp [m/s^2]: a 0.55 m/s command peaks at ~0.74 m/s
    // in the sim, under the 0.8 m/s cap.
    float drive_accel_m_s2 = 0.2F;
    // Wheel deadband compensation [rad/s]: the BL4820 wheels barely move below
    // ~1 rad/s. Added smoothly: w + comp * tanh(w / deadband_eps).
    float deadband_comp_rad_s = 0.6F;
    float deadband_eps_rad_s = 0.3F;
    float speed_lpf_hz = 5.0F; // speed-loop measurement filter
    // Command may lead the measured wheel speed by at most this (stalls).
    float tracking_margin_m_s = 0.30F;
    // Held/lifted detection: over each window, if the speed command changed by
    // more than held_dv but the body never rotated faster than held_rate (and
    // its horizontal acceleration stayed below held_accel), the body is not free
    // (hand, stand, wheels in the air). The loop then re-zeroes (heldRezero):
    // command 0, position here, balance angle = current pitch.
    float held_dv_m_s = 0.25F;
    // A free robot follows the wheels: ~dv/window >= 1.6 m/s^2.
    float held_accel_m_s2 = 0.6F;
    float held_rate_rad_s = 0.10F;
    std::uint64_t held_window_us = 150'000U;
  };

  // Roll (left/right leg length difference). The planar controllers hold pitch;
  // this keeps the body level across (cross slope, one-wheel bump, legs arranged
  // unevenly) and leans it into turns by atan(v w / g). Laterally the robot
  // stands on two wheels (no inverted pendulum), so the loop is a slow integral
  // on the side height difference through the leg IK. The roll estimate is
  // against gravity (gyro, pulled to the accelerometer with the turn's
  // centripetal pull removed). Boot: OFF (SET roll 1; AppV3.cpp).
  struct Roll {
    bool enabled = false;
    float est_tau_s = 0.5F;     // accelerometer correction of the roll estimate
    float track_m = 0.309F;     // wheel track (2 x half_wheel_base)
    // Integral time constant and proportional part. Slow and filtered on
    // purpose: fast gains made the side legs hunt +-2 mm every 40 ms on the
    // robot (roll estimate jitter +-0.015 rad; the sim legs follow ideally and
    // do not show it). Slopes, uneven legs and the turn lean-in are slow jobs.
    float tau_s = 0.40F;
    float err_lpf_hz = 2.0F;    // low-pass on the roll error the loop acts on
    float max_dh_m = 0.025F;    // |side height offset| (each side +-)
    // Kept below the full height while on (the straight knee can only fold):
    // +-reserve of roll at the top, ~3.7 deg with the 309 mm track.
    float reserve_m = 0.012F;
    float rate_m_s = 0.08F;     // side height offset slew
    float turn_ff = 1.0F;       // lean-in weight (target atan(v w / g))
    float kp = 0.2F;            // proportional part (1 = the whole tilt at once)
    // Servo velocity feed-forward (SET rollvff): the side offsets' rate goes to
    // the GDS68 with each target (Set_Input_Pos vel_ff), so the knee follows a
    // ramp without its position-loop lag and the loop runs with kp_vff/tau_vff.
    // The fast gains without the feed-forward are worse, so both switch together.
    // Off: it made the side legs hunt on the robot.
    bool vel_ff = false;
    float kp_vff = 0.3F;
    float tau_vff = 0.25F;
    // Roll the IMU reads with the robot level (its mount; measured -0.0405 on
    // the floor). SET rolltgt re-trims it.
    float target_rad = -0.040F;
    // Turn command slew while on [rad/s^2]: the lean needs ~0.2-0.3 s (side
    // height slew + knee servo); a stepped yaw would pull the body sideways first.
    float yaw_accel_rad_s2 = 4.0F;
    // Side mapping of the correction: -1 if the IMU's body +y is the robot's
    // right (SET rollsign).
    float sign = 1.0F;
  };

  // Wheel speed predictor (Smith style). The measured wheel speed trails the
  // command by ~75 ms: ~20 ms dead time + a ~35 ms wheel lag, the rest is the
  // reading's age (driver estimate + UART read). The predictor runs a model of
  // the wheel servo on the commands sent and adds what it moved over the
  // reading's age: speed(t) = read + w_model(t) - w_model(t - meas_delay),
  // position likewise.
  struct WheelPredict {
    // On with the whole-body LQR (its gains assume a fresh reading); opt-in for
    // the tiers, whose wheel lead was tuned on the delayed reading.
    bool enabled = false;          // SET wpred (tiers)
    bool with_wb = true;           // SET wbpred
    float act_delay_s = 0.020F;    // command -> wheel dead time
    // Model wheel lag, set longer than the measured 0.035 s on purpose: the
    // whole-body loop stays calm for a true lag of 0.035..0.12 s with 0.06.
    float tau_s = 0.06F;           // SET wptau
    float meas_delay_s = 0.055F;   // age of the speed/position reading
  };

  struct Engage {
    // Engage as soon as the legs are locked. The struct default is on (the host
    // tests assume it); the firmware boots with it off (SET imm 1 re-enables):
    // held in a moving hand the held test reads "free", and the wheels, hips
    // and level loop wound up in the air before the robot met the floor.
    // Ready keeps the wheels still and the tiers off until the robot is held
    // still on the floor (armed) and let go.
    bool immediate = true;
    bool automatic = true;
    // Algo::WholeBody engages only through the armed sequence (immediate and GO
    // ignored): engaged at once from the stand it slammed the wheels +-20 rad/s
    // at 1.5 Hz because the swinging legs defeated the held test.
    bool wb_armed_only = true;
    float move_rate_rad_s = 0.40F;    // carrying the robot off the stand
    float move_pitch_rad = 0.08F;
    float still_rate_rad_s = 0.08F;   // bias-corrected IMU body rate, not joint rate
    // Quiet time required, with hysteresis and a bounded short-glitch
    // allowance. Sustained movement or body-angle drift restarts the window.
    std::uint64_t still_us = 700'000U;
    // Body pitch span allowed inside the quiet window. The IMU pitch of a robot
    // held still on the floor reads 0.010-0.023 rad pp.
    float still_span_rad = 0.015F;
    float tip_rate_rad_s = 0.12F;     // released: starts to tip
    float tip_pitch_rad = 0.02F;
    // Armed: engage this long after ARMED even if it has not tipped (0 = wait
    // for the tip). Armed means held still on the floor, so the wheels cannot
    // race in the air.
    float armed_go_s = 0.0F;
  };

  // v3.1 height change while balancing (both algorithms).
  struct Height {
    float rate_m_s = 0.025F;          // hip-axle height change speed
    // Let the wheel balance loop engage before reserving roll travel.
    float startup_delay_s = 0.8F;
    float startup_rate_m_s = 0.008F;
    float startup_accel_m_s2 = 0.02F;
    float startup_jerk_m_s3 = 0.25F;
    // Peak height acceleration; jerk_m_s3 limits how quickly this reaches it.
    float accel_m_s2 = 0.07F;
    // Bound the change in height acceleration so the commanded joint path
    // starts, brakes and reverses with an S-curve instead of an accel step.
    float jerk_m_s3 = 0.50F;
    // Speed scales down smoothly with the balance quality and reaches zero at
    // these: (1 - |pitch - ref| / err) * (1 - |pitch_rate| / rate). A hard gate
    // stopped and restarted the legs, and each kick fed the wobble that tripped
    // the gate again.
    float max_pitch_error_rad = 0.10F;
    float max_pitch_rate_rad_s = 1.0F;
    std::uint64_t held_suppress_us = 400'000U;  // after the legs stop
    // Scale of the CAD balance-angle feed-forward (PID and tiers-off paths only:
    // ffPitch() is 0 with the virtual pitch). The CAD mass model has the wrong
    // sign on this robot (measured shifts -0.025 rad at 110 mm, -0.03 at 78 mm
    // against CAD +0.054 / +0.097); -0.38 sits between the two ratios. SET hff.
    float balance_ff_scale = -0.38F;
  };

  // Two-tier control. virtual_pitch and trunk both off = the plain wheel LQR.
  //  - Tier 1 (wheels) balances the axle under the whole-robot CoM line:
  //    theta_v = body pitch + com_line(measured hip/knee) - com_line(hang).
  //  - Tier 2 (hips, dhip) has three parts: the reflex ("skateboard": residual
  //    lean + its lead + body damping), the preview (planned lean atan(a/g)) and
  //    the slow level loop ("gimbal": IMU body pitch -> level_pitch_rad). Fast
  //    body/leg counter-rotation cancels in theta_v instead of being amplified
  //    into hip commands by body-only feedback.
  // Values marked "boot" are what AppV3.cpp ships; the host tests use the struct
  // defaults unless a case overrides them.
  struct Tier {
    // On by default. SET vpitch 0 (outside balancing) returns to the plain wheel
    // LQR; the tiers also stay inert without measured legs (legs_measured) or
    // with the PID algorithm.
    bool virtual_pitch = true;   // SET vpitch 0/1 (not while balancing)
    bool trunk = true;           // SET trunk 0/1; needs virtual_pitch (LQR only)
    // The LQR solves ground wheel speed; the UART drives wheel speed relative to
    // the moving shin. A bounded feed-forward uses the inverse of odometry's
    // carrier transform; full correction worsened the delayed wheel plant, so
    // only a small coordination term is applied.
    bool wheel_carrier_compensation = true;
    float wheel_carrier_gain = 0.10F;
    // Body CoM fore/aft correction of the CAD mass model [m, body frame], fitted
    // to the measured balance-angle shifts with height (CAD alone has the wrong
    // sign; HANDOFF 5.37).
    float body_com_dx_m = 0.021F;
    float lean_share = 1.0F;   // full virtual-lean isolation inside the existing travel limits
    // Residual whole-system lean authority, separate from maneuver preview.
    float disturbance_share = 1.0F;
    // Long-lived CoM lean belongs to the wheel trim (high-pass time constant on
    // the residual lean); body level is handled by the separate slow IMU loop.
    float hpf_s = 3.0F;
    // The hip share of the residual lean falls with the pendulum length as
    // (l / l_hang)^exponent; the exponent blends from height_exp_tall at ratio
    // >= ratio_tall to height_exp at ratio <= ratio_low. Struct default 0 = the
    // hips take the whole lean at every height; boot: height_exp 0.15 (share
    // 0.88 at 78 mm). A larger exponent (1.5 was the old crouch cut) leaves most
    // of a push to the body when crouched (HANDOFF 5.46).
    float height_exp = 0.0F;
    float height_exp_tall = 0.0F;
    float ratio_tall = 0.80F;
    float ratio_low = 0.55F;
    // Body-rate lead against the hip servo lag, scaled by the pendulum ratio as
    // ratio^lead_exp (0.5 keeps it a fixed fraction of sqrt(l / g): 0.06 s at the
    // hang pose, ~0.04 at 84 mm). Simulation evidence, not a hardware
    // calibration of the loaded hip/knee servos.
    float lead_s = 0.06F;
    float lead_exp = 0.5F;
    // Hip rate limit and level-loop time scaled by sqrt(l / l_hang) with the
    // measured CoM height: crouched that cuts the hip rate to ~2.1 rad/s and the
    // level time to ~0.53 s. false = fixed rate_rad_s / level_tau_s (SET tadapt 0).
    bool adaptive_schedule = true;
    // Bounded gyro damping of body rotation through the hip [s of hip
    // counter-rotation per rad/s], scheduled by sqrt(l/l_hang) and gated by the
    // disturbance weight (ungated it rings). Struct default 0.04; boot 0.08.
    float body_damping_s = 0.04F;
    // Soft deadband on the fast feedback part of the tier-2 hip (residual lean +
    // lead), not on the planned lean or the slow level loop [rad of lean,
    // SET tdb; 0 = off]. Small leans stay with the wheels; the hips act on real
    // disturbances. 0.003 cut rest hip pp 17-21 % in the sim; 0.005 gave the body
    // too much of a push, 0.002 lost the 8 N catch at 189 mm.
    float hip_deadband_rad = 0.0F;
    // Soft event gate on the reflex ("skateboard") hip path: the lean feedback,
    // its lead and the body damping. The planned lean (preview) and the slow
    // level ("gimbal") loop are not gated. It opens on the body gyro and the
    // whole-system lean residual, NOT on the hips' encoder rate (the signal the
    // chatter pollutes): weight 0 below *_lo, 1 above *_hi, fast attack, slow
    // release so the reflex stays on through a recovery. Off at boot (SET tgate).
    bool reflex_gate = false;
    // Thresholds: ~0.8 standing hip reversals/s with the body close to the
    // ungated one (HANDOFF 5.42).
    float reflex_rate_lo = 0.06F, reflex_rate_hi = 0.20F;   // |gyro| low-passed at 5 Hz [rad/s]
    float reflex_lean_lo = 0.012F, reflex_lean_hi = 0.03F;  // |fast residual| [rad]
    float reflex_attack_s = 0.03F, reflex_release_s = 0.40F;
    // Static level loop, the "gimbal" (SET tlvl): slow feedback on the body IMU
    // angle, independent of the dynamic virtual-lean term and the wheel trim.
    bool level = true;
    float level_pitch_rad = 0.0F;  // SET tlevel: target body pitch (IMU frame)
    // Tall-pose integral time. Scheduled as max(base*sqrt(l/l_hang), 0.5 s) so
    // the body loop stays slow; the 0.5 Hz noise filter below is separate.
    float level_tau_s = 0.75F;
    // SET tlvmax: |static hip offset| allowed. It also absorbs legs arranged off
    // by hand at power-up (the encoder zero).
    float level_max_rad = 0.20F;
    float level_lpf_hz = 0.5F;     // body pitch low-pass feeding the integrator
    float max_rad = 0.15F;     // |dynamic hip offset| from the static height pose
    float rate_rad_s = 3.0F;   // hip command slew
    float ramp_s = 1.0F;       // tier-2 authority ramp 0 <-> 1
    // Tier 2 waits this long after balance engages (and after every held spell)
    // so the wheel loop handles the release catch alone; then the reflex and the
    // level loop ramp in.
    float settle_s = 2.0F;
    // Plausibility of the virtual pitch: the com_line of the MEASURED legs must
    // stay within this of the commanded pose's (plus J * tier-2 hip) for the
    // whole check window, or it faults (a wrong hip/knee sign or zero, a joint
    // that let go). Hip servo lag under a push reaches ~0.07 rad in the sim; a
    // wrong hip sign at 110 mm reads ~1 rad.
    float model_check_rad = 0.12F;
    float model_check_s = 0.3F;
    // Weight of the joint-velocity term in theta_v_rate (the balancer's D
    // channel): 1 = full CoM-line rate, 0 = gyro only (as plain LQR).
    float joint_rate_gain = 1.0F;
    // The same weight crouched (blending in below ratio_tall, all of it at
    // ratio_low, by the measured CoM height; < 0 = joint_rate_gain). The driver
    // rates are ROTOR-side: through a compliant hip gear under the crouched load
    // they lead the link, and the error rides into the balancer's D channel
    // (HANDOFF 5.37). SET tvglow. Boot 0.7; struct default -1 = off.
    float joint_rate_gain_low = -1.0F;
    // Maneuver preview (SET tprev, LQR path; independent of vpitch/trunk). A
    // balancing robot answering a speed step reacts late and backwards (to brake
    // it first speeds up), and the BL4820 adds ~125 ms on top. The operator speed
    // becomes a jerk-limited plan; the balancer tracks it preview_s late with the
    // planned lean atan(a/g) as its pitch reference and the plan's acceleration
    // fed forward; the wheels get the plan preview_s early plus the h*jerk/g
    // contact shift that builds the lean (LIP / ZMP preview).
    // Needs virtual_pitch: the planned lean is only as good as the pitch estimate
    // during a sustained acceleration. Plan 0.3 m/s^2 / 1 m/s^3 matches the
    // command slew of the plain LQR; 0.6 / 3 doubled body acceleration peaks.
    bool preview = true;
    float preview_s = 0.12F;
    float plan_accel_m_s2 = 0.3F;
    float plan_jerk_m_s3 = 1.0F;
    // Planned speed cap (0.55 m/s = the BLE full stick). Cruising within
    // 0.05 m/s of the 0.8 m/s wheel cap leaves no headroom to stand the lean back
    // up; 0.55 held every plant variant (v3_tier_tests V15).
    float plan_vmax_m_s = 0.55F;
    // Motion envelope from the pendulum at the current height (SET tenv): the
    // planned speed cap becomes the wheel cap minus env_catch_rad * sqrt(g l)
    // (capture point: catching a lean theta needs about theta * sqrt(g l) of wheel
    // speed beyond the cruise), and the plan's jerk scales with omega / omega_hang.
    // Off: plan_vmax / plan_jerk as fixed numbers. On with the whole-body LQR
    // (envelope_wb), opt-in for the tiers.
    bool envelope = false;
    bool envelope_wb = true;
    // Whole-body LQR: the legs take the planned lean, so the planned acceleration
    // is the lean budget g * tan(env_lean_rad) (0.10 -> ~1 m/s^2).
    float env_lean_rad = 0.10F;
    // Knee-height compensation (SET khold). A hip swing dq moves the wheel
    // fore/aft at first order but drops the body by ~r (1 - cos dq), which a
    // linear controller cannot see. Paired hip/knee IK (coordinateTrunkAxle)
    // keeps the axle on the nominal horizontal plane while the trunk swings; at
    // the workspace limit it projects X and blocks the level integral windup.
    // At full height the knee is straight and can only fold, so the drop stays.
    // knee_hold_wb: the whole-body path keeps its own knee height solve.
    bool knee_hold = true;
    bool knee_hold_wb = true;
    float env_catch_rad = 0.20F;   // see envelope
    // Tier 2 takes the planned lean from the plan this much ahead of the tracked
    // reference (the GDS68 hip servo lags ~60-90 ms).
    float hip_advance_s = 0.08F;
    // The planned lean (atan(a/g) of a commanded speed-up or braking) goes to the
    // legs in full at every height; the height share cuts only the feedback on
    // the residual lean. The planned lean is open loop and cannot ring; cutting it
    // by the share would leave the body (1 - share) of every commanded
    // acceleration. SET tplanfull 0: scaled by the share.
    bool plan_full = true;
    // Low-pass on the lean rate feeding the tier-2 lead [Hz, 0 = off]. Every
    // cut-off tried (10-30 Hz) cost robustness (crouched push wobble or the
    // hard-stop margin), so it is off; the planned-lean split removes most of the
    // lead's work during commanded maneuvers instead.
    float lead_lpf_hz = 0.0F;
    // Source of the lean rate feeding the lead. false: the virtual-pitch rate
    // (gyro + weighted joint rates). The hips' own encoder rate then closes a
    // loop around the very joint being commanded: the standing hip command
    // reversed 20-30 times/s on the robot (the 2026-10-03 "lộc cộc", HANDOFF
    // 5.41; VMC removal ended it). true: body gyro only (SET tlsrc 1): quieter in
    // the sim, but it rings if the hip lags ~40 ms more than modelled, so it is
    // off at boot.
    bool lead_gyro_only = false;
    // Early hand-back to the body when the WHOLE system is out of headroom. The
    // authority target falls off linearly once the full-authority desired hip
    // offset passes sat_margin of max_rad, but only while the wheel command is
    // also within wheel_sat_margin of its cap (a demanding but recoverable push
    // can ask a lot of the hip without the wheel saturating; fading there costs
    // smoothness for no safety benefit). Applied every tick with no hold timer.
    // Down uses sat_fade_s (fast), up uses ramp_s (slow).
    float sat_margin = 0.6F;
    float wheel_sat_margin = 0.9F;
    float sat_fade_s = 0.2F;
    // Not worth doing (measured in the two-body sim, TIER_AB): a GDS68 velocity
    // feed-forward of the tier-2 hip rate fed the delayed hip speed back
    // positively (standing wobble 0.02 -> 0.34 rad/s); extrapolating the ~96 Hz
    // joint samples by velocity x age gave no gain and more velocity noise.
    //
    // Feed the IMU estimator the kinematic velocity of the IMU point (axle
    // odometry + legs + rotation) while balancing free on the floor, so a
    // sustained acceleration no longer tilts the pitch estimate. Gated off while
    // held/lifted (a held body with spinning wheels reads as a fake
    // acceleration). Needs virtual_pitch. SET timu.
    bool imu_kinematic = true;
    // Knee suspension (SET tsus, needs vpitch): position-driven "sky-hook"
    // damping of the body's vertical speed (IMU specific force along gravity,
    // leaky-integrated, so pitch errors enter only to second order); the height
    // offset eases back to the pose. The hips stay the fore/aft (lean) actuator:
    // at every height a hip moves the body < 1.1 mm per 0.1 rad, the knee 5-10 cm
    // per rad (v3_tier_tests TIER_RATIO). Over a 5 mm bump it is about as good as
    // no suspension (HANDOFF 5.47); real compliance would need GDS68 current.
    bool suspension = true;
    float sus_gain = 3.0F;        // height rate per body vertical speed (SET tsusg)
    float sus_return_s = 0.6F;    // offset back to the pose (0.3 s releases slow ramps too early)
    float sus_startup_settle_s = 0.4F;
    float sus_startup_ramp_s = 1.0F;
    float sus_max_m = 0.008F;     // |height offset| (SET tsusmax)
    // Vertical speed estimate: the IMU above this time constant, the legs' own
    // height rate (encoders; equal to it on a flat floor) below. An IMU-only
    // estimate drifts.
    float sus_leak_s = 0.3F;
    // Soft fixed deadband on that speed [m/s]: sensor noise in the sim reads
    // ~1-3 mm/s, a 5 mm bump at 0.3 m/s ~50.
    float sus_deadband_m_s = 0.005F;
    // Noise-adaptive suspension (SET tsusad 0/1; boot on). On the robot the
    // vertical-speed estimate has sd 0.02-0.05 m/s against the fixed 5 mm/s
    // deadband and a gain of 3, so the suspension turned noise into a +-8 mm
    // height random walk (HANDOFF 5.47). Adaptive: the deadband follows
    // k * rms(vz error) over sus_noise_tau_s (a bump is far above it, noise and
    // drift below), the suspension waits sus_engage_delay_s after engage (the
    // start-up lurch is not a bump) and does nothing while the robot is not calm
    // (|ground speed| / |gyro| above the limits).
    bool sus_adaptive = false;
    float sus_noise_k = 3.0F;
    float sus_noise_tau_s = 4.0F;
    float sus_engage_delay_s = 6.0F;
    float sus_calm_speed_m_s = 0.30F;
    float sus_calm_rate_rad_s = 0.50F;
  };

  #ifdef TAP_SPLIT_HOST_PROTOTYPE
  // Experimental host prototype. Default-off keeps the existing path intact.
  struct Split {
    bool enabled = false;
    float posture_tau_s = 1.5F;
    float posture_rate_rad_s = 0.1F;
    float lean_threshold_rad = 0.04F;
    float rate_threshold_rad_s = 0.6F;
    float accel_threshold_m_s2 = 1.5F;
    float wheel_threshold = 0.6F;
    float attack_s = 0.03F;
    float hold_s = 0.3F;
    float release_s = 0.75F;
    float rearm_s = 0.3F;
    float crouch_scale = 1.0F;
  };
  #endif
  struct Config {
    Algo algo = Algo::Pid;
    #ifdef TAP_SPLIT_HOST_PROTOTYPE
    Split split{};
    #endif
    Tier tier{};
    Roll roll;
    WheelPredict wheel_pred;
    Height height{};
    Engage engage{};
    Pid pid{};
    robot_control::LqrBalancer::Config lqr{};
    float com_height_m = 0.2145F;         // locked extended legs (CAD FK)
    // How many pendulum-length points of the LQR gain table to pre-solve per
    // tick once the controller runs (LqrBalancer::buildGainTableStep), so a live
    // height change only interpolates instead of re-solving the Riccati equation
    // (~50 ms per point on the ESP32-S3).
    unsigned lqr_gain_table_points = 12U;
    // Wheel deadband push on the LQR path (the PID path keeps Pid's 0.6).
    float lqr_deadband_comp_rad_s = 0.3F;
    // Fixed wheel-loop feedback lead [s] on the LQR path: the sent wheel speed
    // leads the LQR's filtered solved acceleration by this much, compensating the
    // wheel servo lag. Used when lqr_lead_adaptive is off (SET lqrlead <s> pins
    // it and turns the schedule off). 0.03 stands tall but falls crouched; 0.08
    // stands both (HANDOFF 5.46).
    float lqr_lead_s = 0.03F;
    // Height-adaptive lead (SET lqradapt 0/1; boot on). The wheel loop's phase
    // lag matters more the faster the pendulum is, so the lead grows as the CoM
    // comes down:
    //   lead = lqr_lead_tall_s + (lqr_lead_low_s - lqr_lead_tall_s) * clamp((hi - r) / (hi - lo))
    // with r = measured CoM height / hang CoM height (adaptive_ratio_). Fitted in
    // the sim on wheel models consistent with the robot (dead time 44-52 ms, lag
    // 0.09 s, reading 54-80 ms old): optimum 0.05-0.065 at 177-150 mm, 0.08 at
    // 120 mm, 0.085-0.095 at <= 100 mm (HANDOFF 5.46).
    bool lqr_lead_adaptive = false;
    float lqr_lead_tall_s = 0.05F;
    float lqr_lead_low_s = 0.09F;
    float lqr_lead_ratio_hi = 0.95F;
    float lqr_lead_ratio_lo = 0.50F;
    // LQR position-error trim integral [rad per m*s] (half the PID kix).
    float lqr_trim_gain = 0.01F;
    // Balance-angle estimate from the measured ground acceleration (SET ttrimacc
    // 0/1; boot on). The position integral above is slow (after a height change
    // the robot drifts ~0.2 m for ~25 s). A free robot at rest has zero
    // acceleration at the balance angle, a = g tan(theta_v - theta_eq), so
    // theta_eq = theta_v - atan(a/g) with a from the wheels (not the
    // accelerometer: its tilt frame carries the IMU pitch bias). The trim follows
    // it with trim_acc_tau_s. Off: only the position integral (HANDOFF 5.46).
    bool trim_acc = false;
    float trim_acc_tau_s = 3.0F;
    float trim_acc_max_a_m_s2 = 0.8F;   // ignore the estimate while accelerating harder
    // LQR pendulum model length (host tests only: TR_LQRLEFF; no SET key). false: the point-mass model,
    // pendulum_length_per_com_height x CoM height. true: the rigid body's
    // effective length I_axle / (m l) from the CAD inertia at the current pose.
    // The true length is 1.22x the point-mass one at 189 mm and 2.03x at 78 mm
    // (HANDOFF 5.37). Off by default.
    bool lqr_inertia_length = false;
    // Algo::WholeBody: rate [rad per m s] at which the position error teaches
    // the measured natural body pitch (IMU mount + CoM model error).
    float wb_bias_gain = 0.10F;
    // Algo::WholeBody limit-cycle guard: 4 cap-to-cap reversals of the wheel
    // command within this window fault (see wbUpdate). A push saturates one way.
    std::uint64_t wb_ring_window_us = 2'000'000U;
    // Algo::WholeBody: above this fraction of the wheel speed cap the
    // position anchor gives way toward the robot and the integral holds
    // (balance before position) - SET wbgive (1 = off).
    float wb_give_from = 0.7F;
    // Sign of the leg-Jacobian axle speed taken off in the WB held test.
    float wb_held_leg_sign = 1.0F;
    // Algo::WholeBody stiff/soft blend (see wbUpdate): starts at this
    // fraction of the wheel speed cap, full 0.15 + below the cap; returns
    // over wb_soft_return_s - SET wbsoft (1 = stiff table only).
    float wb_soft_from = 0.5F;
    float wb_soft_return_s = 1.0F;
    // Algo::WholeBody: the hips take the planned lean atan(a/g) (body stays at
    // its target) - SET wbff 0/1.
    bool wb_lean_ff = true;
    // Algo::WholeBody: how far the position anchor may lead/lag the robot while
    // driving [m]; 0 = speed only (SET wbslack).
    float wb_drive_slack_m = 0.0F;
    std::uint64_t lock_settle_us = 1'200'000U;  // leg ramp + settle before wheels
    float fall_pitch_rad = 0.60F;
    // Auto-learned balance trim (theta_v-referenced, height-independent while
    // virtual_pitch is on). Once the robot has held its ground position within
    // learn_position_tol_m and rotated less than learn_rate_tol_rad_s for
    // learn_hold_s, the current trim is remembered. Every later release this
    // power-up (held/lifted -> free) walks the trim from the hold angle to the
    // learned one over learn_release_s instead of balancing around whatever angle
    // the operator put it down at. While still held the trim tracks the hold
    // angle (a learned trim there makes the wheels fight the hand). Needs
    // virtual_pitch; inert otherwise.
    float learn_position_tol_m = 0.03F;
    // Standing has continuous small pitch-rate noise (up to ~0.1-0.2 rad/s even
    // when calm); a tighter tolerance keeps resetting the hold timer.
    float learn_rate_tol_rad_s = 0.20F;
    float learn_hold_s = 2.5F;
    float learn_tau_s = 4.0F;  // slow re-tracking once learned (adapts, no jumps)
    float learn_release_s = 0.3F;
    // Wheel ground-speed cap while held/lifted (after a held re-zero, until the
    // body turns on its own again). In the air the balancer's speed feedback is
    // positive feedback, so the wheels would race to the cap and the robot would
    // meet the floor with them spinning. A free robot on the floor lifts the cap
    // within ~0.1 s: it tips, and the body rate leaves the held state.
    float held_wheel_cap_m_s = 0.10F;
    // Runaway guard: wheel speed [rad/s] sustained for runaway_us. Below the
    // 0.8 m/s ground-speed cap (20 rad/s) so a lifted or held robot, which
    // saturates the wheels at the cap, still trips it.
    float runaway_wheel_rad_s = 15.0F;
    std::uint64_t runaway_us = 500'000U;
    // ...only once the wheel ground speed is this far from the speed the
    // accelerometer integrates (leaking to the wheels over runaway_v_tau_s).
    float runaway_dv_m_s = 0.25F;
    // LQR drive anchor at most this far from the robot (0 = off, SET leash): a
    // climb or stall must not bank position error for a catch-up sprint.
    float drive_leash_m = 0.10F;
    // Standing leash [m] (0 = off, SET sleash; see lqrUpdate): once the wheels
    // pass stand_leash_from of their cap, the LQR anchor gives way to within this
    // of the robot until it has stopped, then walks back to the station at
    // stand_return_m_s. It is off by default: a fixed leash (no gate, no walk
    // back) made hand presses roll further and lost the 8 N catch (sim TIER_PRESS).
    float stand_leash_m = 0.0F;
    float stand_leash_from = 0.7F;  // same give point as wb_give_from
    float stand_return_m_s = 0.10F;  // anchor walk back to the station
    float stand_leash_release_s = 5.0F;  // leash off this long after engage/release
    float runaway_v_tau_s = 1.5F;
    // The runaway guard stays off while the operator drives and this long
    // after the last drive command (covers LqrBalancer's stop ramp).
    std::uint64_t drive_guard_hold_us = 1'500'000U;
    float wheel_radius_m = 0.04F;
  };

  struct Input {
    std::uint64_t now_us = 0U;
    float dt_s = 0.002F;
    float pitch_rad = 0.0F;
    float pitch_rate_rad_s = 0.0F;
    float yaw_rate_rad_s = 0.0F;
    float roll_rate_rad_s = 0.0F;  // body x (forward) axis, + = left side up
    float accel_y_m_s2 = 0.0F;     // body y (left) specific force
    // Body-frame accelerometer specific force (x forward, z up) [m/s^2]:
    // tells a free robot (body accelerates with the wheels) from a held one.
    float accel_x_m_s2 = 0.0F;
    float accel_z_m_s2 = 9.80665F;
    std::array<float, 2> wheel_rad_s{};    // left, right (core sign: + forward)
    std::array<float, 2> wheel_pos_rad{};
    // Mean over both legs of (hip + knee) measured offsets from their zero,
    // core frame [rad]: the wheel encoder turns with the shin, so axle
    // odometry needs it once the legs move.
    float leg_chain_rad = 0.0F;
    // measured leg joints per side (left, right), core frame, offsets
    // from each joint's zero [rad] and driver velocity estimates [rad/s].
    // legs_measured = filled and fresh (the tiers stay off otherwise).
    std::array<float, 2> hip_rad{};
    std::array<float, 2> knee_rad{};
    std::array<float, 2> hip_rate_rad_s{};
    std::array<float, 2> knee_rate_rad_s{};
    bool legs_measured = false;
    bool sensors_ok = true;                 // IMU, wheels, legs fresh; no axis fault
    bool legs_ready = false;                // leg lock has reached its target
  };

  struct Command {
    bool start = false;       // edge
    bool go = false;          // edge: engage balancing now (from Ready)
    bool disable = false;     // level: stop everything
    float forward_m_s = 0.0F;
    float yaw_rad_s = 0.0F;
    // Requested hip-axle height [m]; <= 0 keeps the current request.
    float height_m = 0.0F;
  };

  struct Output {
    State state = State::Idle;
    bool engage_legs = false;   // edge: begin the leg lock now
    bool legs_locked = false;   // level: keep the legs held (also through a fault)
    bool wheels_enabled = false;
    std::array<float, 2> wheel_cmd_rad_s{};
    // hip/knee offsets for both legs (core rad, from their zero).
    float leg_hip_offset_rad = 0.0F;
    float leg_knee_offset_rad = 0.0F;
    // Roll (Config::Roll): per-side joint offsets added to the common ones
    // above, [left, right]; opposite heights, so the mean is unchanged.
    std::array<float, 2> leg_roll_hip_rad{};
    std::array<float, 2> leg_roll_knee_rad{};
    // ...and their rates for the servo velocity feed-forward (0 unless
    // Roll::vel_ff) [rad/s].
    std::array<float, 2> leg_roll_hip_rate{};
    std::array<float, 2> leg_roll_knee_rate{};
    bool legs_moving = false;
    const char* fault = nullptr;  // set on the tick a fault is raised
  };

  V3Controller() : V3Controller(Config{}) {}
  explicit V3Controller(Config config);

  [[nodiscard]] Output update(const Input& input, const Command& command);
  [[nodiscard]] float runawayImuSpeed() const noexcept { return runaway_v_imu_; }
  [[nodiscard]] State state() const noexcept { return state_; }
  [[nodiscard]] robot_control::LqrBalancer& balancer() noexcept { return lqr_; }
  [[nodiscard]] const robot_control::LqrBalancer& balancer() const noexcept { return lqr_; }
  [[nodiscard]] const char* lastFault() const noexcept { return last_fault_; }
  [[nodiscard]] Config& config() noexcept { return config_; }
  // Measured balance angle (IMU pitch with the COM over the axle). When set,
  // it seeds the balance reference at release instead of the release pitch.
  void setBalanceAngle(bool valid, float pitch_rad) noexcept {
    balance_set_ = valid;
    balance_rad_ = pitch_rad;
  }
  [[nodiscard]] bool balanceAngleSet() const noexcept { return balance_set_; }
  // KNEETEST FLOOR: knee offset [rad, |.| <= 0.1] added to the height pose's
  // knee while balancing; the hip follows through LegHeight::forKnee as for a
  // height change. Dropped as soon as the robot leaves Balancing.
  void setKneeExcitation(float dk_rad) noexcept {
    knee_excite_rad_ = std::clamp(dk_rad, -0.1F, 0.1F);
  }
  // Back to Ready (legs stay locked) so the next GO engages the real balance
  // loop, e.g. from the hand-held TILTTEST check.
  void returnToReady() noexcept {
    if (state_ == State::Balancing) {
      state_ = State::Ready;
      ready_phase_ = 0;
      ready_entry_pitch_ = theta_v_;
      resetReadyStill();
    }
  }
  [[nodiscard]] float balanceAngle() const noexcept { return balance_rad_; }
  // Balance angle in use: LQR trim, or the PID's learned theta_b.
  [[nodiscard]] float balanceRef() const noexcept {
    return config_.algo == Algo::Pid ? pid_theta_b_ : lqr_.balanceTrim();
  }
  // Auto-learned balance trim (see Config::learn_hold_s); for GET/telemetry.
  [[nodiscard]] bool learnedTrimValid() const noexcept { return learned_trim_valid_; }
  [[nodiscard]] float learnedTrim() const noexcept { return learned_trim_rad_; }
  [[nodiscard]] float pitchRef() const noexcept { return pid_ref_; }
  [[nodiscard]] float positionError() const noexcept { return pid_x_ - pid_x_target_; }
  // LQR anchor relative to the robot, and the standing-leash give in it
  // (Config::stand_leash_m); for telemetry.
  [[nodiscard]] float lqrAnchorOffset() const noexcept {
    return lqr_x_target_m_ + stand_give_m_ - ground_x_;
  }
  [[nodiscard]] float standGive() const noexcept { return stand_give_m_; }
  // Ready sub-phase for the log: 0 wait move, 1 wait still, 2 armed.
  [[nodiscard]] int readyPhase() const noexcept { return ready_phase_; }
  // Number of held/lifted re-zeroes since the balance loop engaged.
  [[nodiscard]] int heldResets() const noexcept { return held_resets_; }
  // Held/lifted state: outer loop off (wheels capped), and still quiet.
  [[nodiscard]] bool heldOuterOff() const noexcept { return held_outer_off_; }
  [[nodiscard]] bool heldQuiet() const noexcept { return held_quiet_; }
  [[nodiscard]] float heightTarget() const noexcept { return height_target_m_; }
  // Roll and coordinated trunk motion both need knee extension in reserve.
  // START/Ready still hold the exact power-up pose; this ceiling applies
  // only to the subsequent supported height profile.
  [[nodiscard]] float supportedMaxHeight() const noexcept {
    const bool reserve = config_.roll.enabled ||
        (tierVirtual() && config_.tier.trunk && kneeHoldOn());
    return LegHeight::maxHeight() - (reserve
        ? std::clamp(config_.roll.reserve_m, 0.0F, 0.05F) : 0.0F);
  }
  [[nodiscard]] float heightSpeed() const noexcept { return height_speed_m_s_; }
  [[nodiscard]] float heightAcceleration() const noexcept { return height_accel_m_s2_; }
  // Nominal hip-to-axle target that brings the current height speed to rest.
  // Used by HMOVE release so the requested stop uses the same jerk-limited
  // profile as updateHeight().
  [[nodiscard]] float heightStopTarget() const noexcept;
  [[nodiscard]] bool legsHeld() const noexcept { return legs_held_; }
  [[nodiscard]] const LegPose& legPose() const noexcept { return pose_; }
  [[nodiscard]] bool lqrGainTableReady() const noexcept { return lqr_.gainTableReady(); }
  // Ground velocity/position used by the balancer (axle, rolling contact).
  [[nodiscard]] float groundVelocity() const noexcept { return ground_v_; }
  [[nodiscard]] float groundPosition() const noexcept { return ground_x_; }
  // virtual pitch (equals the body pitch while virtual_pitch is off),
  // tier-2 hip offset on top of the height pose, and its authority 0..1.
  [[nodiscard]] float virtualPitch() const noexcept { return theta_v_; }
  [[nodiscard]] float virtualPitchRate() const noexcept { return theta_v_rate_; }
  [[nodiscard]] float trunkHip() const noexcept { return dhip_ + axle_hip_correction_; }
  [[nodiscard]] float trunkSwingRequest() const noexcept { return dhip_; }
  [[nodiscard]] float trunkAuthority() const noexcept { return trunk_alpha_; }
  // Knee suspension (Tier::suspension): height offset [m], vertical speed.
  [[nodiscard]] float suspensionOffset() const noexcept { return sus_dz_; }
  [[nodiscard]] float bodyVerticalSpeed() const noexcept { return sus_vz_; }
  // Static level loop's share of trunkHip() (Tier::level).
  [[nodiscard]] float wbLevelHip() const noexcept { return wb_bias_; }
  [[nodiscard]] float rollEstimate() const noexcept { return roll_est_; }  // vs gravity
  [[nodiscard]] float rollTarget() const noexcept { return roll_target_; }
  [[nodiscard]] float rollSideHeight() const noexcept { return roll_dh_; }  // left +, right -
  [[nodiscard]] float yawCommand() const noexcept { return yaw_cmd_; }
  [[nodiscard]] float levelHip() const noexcept { return level_beta_ * level_i_; }
  [[nodiscard]] float reflexGate() const noexcept { return reflex_w_; }
  [[nodiscard]] float lqrLeadEffective() const noexcept { return lqr_lead_eff_; }
  [[nodiscard]] float suspensionDeadband() const noexcept { return sus_db_eff_; }
  #ifdef TAP_SPLIT_HOST_PROTOTYPE
  struct SplitTelemetry { float hip_a, hip_b, alpha_b, preview, model_target; bool frozen; };
  [[nodiscard]] SplitTelemetry splitTelemetry() const noexcept {
    return {split_a_, split_b_, split_alpha_b_, split_preview_, split_model_target_, split_frozen_};
  }
  #endif

  // Hip offset to send to the legs (height pose + tier 2), every state.
  [[nodiscard]] float hipCommand() const noexcept {
    return pose_.hip_offset_rad + dhip_ + axle_hip_correction_;
  }
  [[nodiscard]] bool trunkWorkspaceLimited() const noexcept { return axle_limited_; }
  struct TrunkSchedule {
    float com_ratio = 1.0F;
    float lead_s = 0.06F;
    float rate_rad_s = 3.0F;
    float max_rad = 0.15F;
    float hip_to_lean_gain = 1.0F;
    float level_tau_s = 0.75F;
  };
  [[nodiscard]] const TrunkSchedule& trunkSchedule() const noexcept { return trunk_schedule_; }
  [[nodiscard]] const std::array<float, 2>& wheelCarrierCorrection() const noexcept {
    return wheel_carrier_rad_s_;
  }
  // Knee offset to send to the legs (height pose + whole-body offset).
  [[nodiscard]] float kneeCommand() const noexcept { return pose_.knee_offset_rad + dknee_; }
  [[nodiscard]] bool wholeBody() const noexcept { return config_.algo == Algo::WholeBody; }
  // Gain table in use (default: the generated WholeBodyGains.hpp); tests and
  // the weight search swap in their own designs.
  using WbTable = std::array<wb::Point, wb::kHeights>;
  void setWholeBodyTable(const WbTable* table, const WbTable* soft = nullptr) noexcept {
    wb_table_ = table ? table : &wb::kTable;
    wb_table_soft_ = soft ? soft : (table ? table : &wb::kTableSoft);
  }
  [[nodiscard]] float wbSoftBlend() const noexcept { return wb_soft_; }
  [[nodiscard]] const std::array<float, 10>& wbTerms() const noexcept { return wb_dbg_terms_; }
  struct WbDebug { float x_ref, xi, theta_ref, v_ref, bias; bool driving; };
  [[nodiscard]] WbDebug wbDebug() const noexcept {
    return {wb_x_ref_, wb_xi_, wb_theta_ref_, plan_v_, wb_bias_, pid_driving_};
  }
  // World-frame velocity of the IMU point for the pitch estimator's
  // specific-force compensation; valid only while balancing free (Tier::
  // imu_kinematic).
  struct ImuVelocity {
    float horizontal_m_s = 0.0F;
    float vertical_m_s = 0.0F;
    bool valid = false;
  };
  [[nodiscard]] ImuVelocity imuVelocity() const noexcept { return imu_velocity_; }
  // d com_line / d hip at the current height pose (CAD + body_com_dx).
  [[nodiscard]] float comLineHipGain() const noexcept { return j_hip_; }
  // Measured minus exact commanded com_line, including paired knee motion.
  [[nodiscard]] float legModelError() const noexcept;

 private:
  Output fault(const char* reason, const Output& base);

  Config config_{};
  robot_control::LqrBalancer lqr_;
  State state_ = State::Idle;
  std::uint64_t state_since_us_ = 0U;
  std::uint64_t runaway_since_us_ = 0U;
  // Pitch when wheel speed first sustained above runaway_wheel_rad_s: a held or
  // lifted robot's body stays near this while the wheels race, an unrecovered
  // fall moves steadily away from it. Net displacement tells them apart (a peak
  // rate gate disqualified normal catch transients and let slow drifts through).
  float runaway_start_pitch_ = 0.0F;
  float runaway_v_imu_ = 0.0F;  // accelerometer ground speed (runaway guard)
  std::uint64_t drive_seen_us_ = 0U;  // last tick with a nonzero drive command
  const char* last_fault_ = "";
  // The legs are structure in v3: once locked they stay locked through a
  // fault; only DISABLE (command.disable) releases them.
  bool legs_held_ = false;
  float ground_v_ = 0.0F;
  float ground_x_ = 0.0F;
  // Height state: the knee fold is ramped per tick, the requested height is
  // converted to a knee target once per request.
  float height_target_m_ = 0.0F;
  float knee_target_rad_ = 0.0F;
  float knee_cmd_rad_ = 0.0F;
  float height_speed_m_s_ = 0.0F;  // signed hip-axle speed, + = up
  float height_accel_m_s2_ = 0.0F; // signed acceleration of the height profile
  LegPose pose_{};
  float chain_prev_rad_ = 0.0F;
  float chain_rate_f_ = 0.0F;
  bool chain_init_ = false;
  std::uint64_t legs_moved_us_ = 0U;
  void updateHeight(const Input& in, const Command& cmd);
  [[nodiscard]] static float heightStoppingDistance(float speed_m_s,
                                                     float accel_m_s2,
                                                     float max_accel_m_s2,
                                                     float max_jerk_m_s3) noexcept;
  void applyPoseOffsets(const Input& in);
  void updateSuspension(const Input& in);
  float knee_excite_rad_ = 0.0F;
  float applied_dk_ = 0.0F, applied_base_ = 0.0F;
  float sus_g_ = 9.80665F, sus_vz_ = 0.0F;
  float sus_dz_ = 0.0F, sus_dk_ = 0.0F;
  bool sus_init_ = false;
  bool height_startup_ = false;
  float startup_height_m_ = 0.0F;
  std::uint64_t height_settled_since_us_ = 0U;
  float sus_alpha_ = 0.0F;
  bool excite_applied_ = false;
  // The virtual pitch already carries the balance shift with height (from
  // the measured legs), so the CAD feed-forward is off with it.
  [[nodiscard]] float ffPitch() const noexcept {
    return tierVirtual() ? 0.0F
                         : config_.height.balance_ff_scale * pose_.balance_pitch_delta_rad;
  }
  // The tiers run on the LQR path only: the PID path with the virtual pitch
  // overshot a 0.55 m/s drive past the speed cap in the two-body sim.
  [[nodiscard]] bool tierVirtual() const noexcept {
    return config_.tier.virtual_pitch && legs_measured_ && config_.algo == Algo::Lqr;
  }
  // com_line angle [rad] and its partials by hip/knee at core leg offsets.
  struct ComLine { float angle, d_hip, d_knee, height_m; };
  void resetReadyStill() noexcept;
  [[nodiscard]] bool readyStill(const Input& in, float body_pitch, float body_rate) noexcept;
  [[nodiscard]] ComLine comLine(float hip_offset, float knee_offset) const noexcept;
  void updateVirtualPitch(const Input& in);
  void updateTrunk(const Input& in, bool active, std::array<float, 2> wheel_cmd_rad_s);
  void updateLegLayers(const Input& in, bool active, std::array<float, 2> wheel_cmd_rad_s);
  #ifdef TAP_SPLIT_HOST_PROTOTYPE
  void updatePosture(const Input& in, bool on, bool saturated);
  void updateShock(const Input& in, bool on, float wheel_ratio, float plan_lean);
  [[nodiscard]] float splitModelPosture() const noexcept;
  float split_a_ = 0.0F, split_b_ = 0.0F, split_alpha_b_ = 0.0F;
  float split_preview_ = 0.0F, split_model_target_ = 0.0F, split_integral_ = 0.0F;
  float split_hold_s_ = 0.0F, split_quiet_s_ = 0.0F;
  bool split_was_enabled_ = false, split_frozen_ = false, split_armed_ = true;
  #endif

  void updateWheelLegCoordination(const Input& in) noexcept;
  void updateImuVelocity(const Input& raw, bool free);
  // Maneuver preview (Tier::preview): jerk-limited speed plan and its history.
  struct PlanSample { float v, a; };
  void updatePlan(float target_v, float dt, bool reset);
  [[nodiscard]] float planCap(float vmax) const noexcept;
  // Knee offset from the pose that keeps the pose's hip-axle height with the
  // hip at pose + dhip (Tier::knee_hold), 0 when off.
  [[nodiscard]] float kneeHold(float dhip) noexcept;
  void coordinateTrunkAxle() noexcept;
  [[nodiscard]] bool kneeHoldOn() const noexcept {
    return config_.tier.knee_hold || (wholeBody() && config_.tier.knee_hold_wb);
  }
  float knee_hold_ = 0.0F;  // last solution (warm start)
  // Wheel predictor (Config::WheelPredict).
  void predictWheels(const Input& raw, float* rate, float* pos) noexcept;
  static constexpr std::size_t kWpBuf = 128U;  // 0.25 s at 500 Hz
  std::array<float, kWpBuf> wp_cmd_{}, wp_w_{}, wp_p_{};
  std::size_t wp_head_ = 0U;
  float wp_w_model_ = 0.0F, wp_p_model_ = 0.0F, wp_last_cmd_ = 0.0F;
  // Roll (Config::Roll).
  void updateRoll(const Input& in, bool active);
  float roll_est_ = 0.0F;      // vs the apparent vertical, + = left side up
  float roll_bias_ = 0.0F;     // learned roll gyro bias [rad/s]
  bool roll_init_ = false;
  float roll_dh_ = 0.0F;       // side height offset: left +roll_dh_, right -roll_dh_ [m]
  float roll_v_lp_ = 0.0F, roll_w_lp_ = 0.0F, roll_i_ = 0.0F, roll_e_lp_ = 0.0F;
  float yaw_cmd_ = 0.0F;  // the turn command as used (slewed with roll on)
  float roll_target_ = 0.0F;
  std::array<float, 2> roll_hip_{}, roll_knee_{}, roll_hip_rate_{}, roll_knee_rate_{};
  [[nodiscard]] bool envelopeOn() const noexcept {
    return config_.tier.envelope || (wholeBody() && config_.tier.envelope_wb);
  }
  [[nodiscard]] PlanSample planDelayed() const noexcept;
  [[nodiscard]] PlanSample planAt(float delay_s) const noexcept;  // plan delay_s ago
  float lean_lp_ = 0.0F, lean_rate_f_ = 0.0F; // whole-system residual / rate
  float sus_vz2_ = 0.0F;  // running mean square of the vertical-speed error [m2/s2]
  float ga_v1_ = 0.0F, ga_prev_ = 0.0F, ga_f_ = 0.0F;  // ground-acceleration estimate stages
  bool ga_init_ = false;
  float reflex_w_ = 0.0F, gate_rate_lp_ = 0.0F; // reflex gate weight, |gyro| low-pass
  float plan_v_ = 0.0F, plan_a_ = 0.0F, plan_j_ = 0.0F, plan_dt_ = 0.002F;
  bool plan_init_ = false;
  std::array<PlanSample, 128> plan_buf_{};
  std::size_t plan_head_ = 0U, plan_n_ = 0U;
  bool legs_measured_ = false;
  float theta_v_ = 0.0F;
  float theta_v_rate_ = 0.0F;
  float com_line_hang_ = 0.0F;
  float com_line_dx_ = 0.0F;  // body_com_dx the hang reference was computed for
  float j_hip_ = 1.0F;
  float j_hip_knee_ = 1.0e9F;  // knee pose j_hip_ was computed for
  float c_pose_ = 0.0F;         // com_line of the commanded height pose
  float c_meas_ = 0.0F;         // com_line of the measured legs
  float com_meas_height_m_ = 0.0F;
  float adaptive_ratio_ = 1.0F;
  float sus_db_eff_ = 0.0F;  // suspension deadband in use [m/s]
  float lqr_lead_eff_ = 0.0F;  // wheel-loop lead in use (fixed or height-adaptive)
  bool adaptive_init_ = false;
  TrunkSchedule trunk_schedule_{};
  std::array<float, 2> wheel_carrier_rad_s_{};
  float wheel_leg_alpha_ = 0.0F;
  std::uint64_t wheel_leg_quiet_since_us_ = 0U;
  std::uint64_t model_bad_since_us_ = 0U;
  float dhip_ = 0.0F;
  // dhip is the requested swing; IK may alter the actual hip AND knee.
  float axle_hip_correction_ = 0.0F;
  bool axle_limited_ = false;
  float axle_limit_direction_ = 0.0F;
  float trunk_alpha_ = 0.0F;
  // Static level loop (Tier::level): integrated hip offset, its authority,
  // the filtered body pitch it integrates, and whether it has been started.
  float level_i_ = 0.0F;
  float level_beta_ = 0.0F;
  float body_lp_ = 0.0F;
  bool level_init_ = false;
  float com_hang_m_ = 0.0F;  // CoM above the axle at the hang pose
  float chain_rate_meas_ = 0.0F;
  float body_pitch_ = 0.0F;  // IMU body pitch (the input pitch may be theta_v)
  float body_rate_ = 0.0F;   // IMU body pitch rate
  std::uint64_t trunk_block_us_ = 0U;  // last tick tier 2 was not allowed (held)
  ImuVelocity imu_velocity_{};
  float hip_meas_ = 0.0F, knee_meas_ = 0.0F, hip_rate_meas_ = 0.0F, knee_rate_meas_ = 0.0F;
  bool balance_set_ = false;
  float balance_rad_ = 0.0F;
  // PID state.
  float pid_seed_ = 0.0F;     // balance angle at engage
  float pid_theta_b_ = 0.0F;  // learned balance angle
  float pid_ref_ = 0.0F;      // pitch to hold
  float pid_v_filt_ = 0.0F;
  float pid_cmd_m_s_ = 0.0F;  // integrated wheel speed command
  float pid_x0_ = 0.0F;
  float pid_x_ = 0.0F;
  float pid_x_target_ = 0.0F;
  bool pid_driving_ = false;
  float pid_drive_v_ = 0.0F;  // ramped operator speed (PID path)
  // LQR absolute ground-position anchor (position_anchor_hold), moved by
  // cmd.forward_m_s while driving and frozen at the current position on release.
  // Same job as pid_x_target_, but LqrBalancer takes it as an absolute anchor
  // (reanchorPosition).
  float lqr_x_target_m_ = 0.0F;
  // Config::lqr_inertia_length: CAD effective pendulum length, cached per pose.
  float lqr_leff_hip_ = 1.0e9F, lqr_leff_knee_ = 1.0e9F, lqr_leff_m_ = 0.15F;
  [[nodiscard]] float lqrInertiaLength() noexcept;
  // Standing leash give: the LQR anchor is lqr_x_target_m_ + this (see
  // Config::stand_leash_m), back to 0 at stand_return_m_s.
  float stand_give_m_ = 0.0F;
  float stand_leash_wait_s_ = 0.0F;  // leash off after engage/release
  bool stand_leash_on_ = false;      // giving way until the robot stops
  // 20 Hz low-pass of the LQR's solved acceleration for the wheel-lag lead term;
  // unfiltered, gyro/encoder noise rode into the sent wheel command and read as
  // jerky chatter on the robot.
  float lqr_lead_filt_ = 0.0F;
  std::uint64_t held_window_start_us_ = 0U;
  float held_v0_ = 0.0F;
  float held_max_rate_ = 0.0F;
  float held_accel_sum_ = 0.0F;
  int held_accel_n_ = 0;
  float body_accel_f_ = 0.0F;
  float ax_f_ = 0.0F;         // 5 Hz body-frame accel_x
  float held_ax_ref_ = 0.0F;  // ax_f_ at the last held re-zero
  int held_free_ticks_ = 0;
  int held_resets_ = 0;
  // After a held re-zero: command 0 until the wheels have stopped (or the
  // operator tilts it), so their run-down cannot restart the loop.
  bool held_quiet_ = false;
  // After a held re-zero the speed/position loop stays off (angle loop only)
  // until the body rotates on its own again (released on the floor): with
  // the body held, the outer loop is what regrows the wheel speed.
  bool held_outer_off_ = false;
  std::uint64_t held_quiet_since_us_ = 0U;
  // Ready state.
  int ready_phase_ = 0;
  float ready_entry_pitch_ = 0.0F;
  std::uint64_t still_since_us_ = 0U;
  float still_sum_ = 0.0F;
  int still_n_ = 0;
  std::uint64_t ready_last_us_ = 0U;
  std::uint64_t still_quiet_us_ = 0U, still_bad_us_ = 0U, still_bad_run_us_ = 0U;
  std::uint64_t ready_tip_us_ = 0U;
  std::uint64_t ready_armed_us_ = 0U;  // when ARMED (Engage::armed_go_s)
  float still_body_min_ = 0.0F, still_body_max_ = 0.0F;
  bool still_quiet_ = false;
  float armed_pitch_ = 0.0F;

  void engageBalance(const Input& in, float seed);
  void heldRezero(const Input& in);
  [[nodiscard]] float releaseBalance(const Input& in) const;
  void updateLearnedTrim(const Input& in, float ex, float current_trim_rad);
  float learned_trim_rad_ = 0.0F;
  bool learned_trim_valid_ = false;
  std::uint64_t learn_since_us_ = 0U;
  bool trim_blend_ = false;       // walking the trim to the learned one after a release
  float trim_blend_rate_ = 0.0F;  // [rad/s]
  std::array<float, 2> pidUpdate(const Input& in, const Command& cmd);
  std::array<float, 2> lqrUpdate(const Input& in, const Command& cmd);
  // Whole-body LQR (Algo::WholeBody).
  std::array<float, 2> wbUpdate(const Input& in, const Command& cmd);
  void wbReset(const Input& in);
  float dknee_ = 0.0F;
  const WbTable* wb_table_ = &wb::kTable;
  const WbTable* wb_table_soft_ = &wb::kTableSoft;
  float wb_soft_ = 0.0F;  // blend toward the soft table near the wheel cap (0..1)
  float wb_x_ref_ = 0.0F;
  float wb_xi_ = 0.0F;
  float wb_theta_ref_ = 0.0F;   // body pitch reference (tracks the hand while held)
  float wb_theta_rate_ = 0.0F;  // its walk back to the table equilibrium after release [rad/s]
  float wb_cmd_v_ = 0.0F;       // last wheel command as ground speed (held test)
  float wb_cmd_f_ = 0.0F;       // ...as the wheels follow it (0.09 s lag)
  float wb_drive_ex0_ = 0.0F;   // position error kept through a drive
  std::array<float, 10> wb_dbg_terms_{};  // diagnostics: wheel-row K*X terms
  float wb_held_v_ = 0.0F;      // ...minus the legs' own axle speed (held test)
  // Measured-minus-model natural body pitch (IMU mount + CoM model error),
  // learned from the position error like the tiers' balance trim; kept
  // across re-zeroes.
  float wb_bias_ = 0.0F;
  // Algo::WholeBody with Tier::level: equilibrium lean measured from the
  // IMU pitch + the CoM line of the measured legs while standing still.
  float wb_lean_lp_ = 0.0F;
  bool wb_lean_init_ = false;
  std::array<float, wb::kWheelDelay> wb_wheel_hist_{};
  std::array<float, wb::kJointDelay> wb_hip_hist_{};
  std::array<float, wb::kJointDelay> wb_knee_hist_{};
  int wb_sat_sign_ = 0;                         // wheel command at a cap: +1/-1, 0 none yet
  std::array<std::uint64_t, 4> wb_sat_flips_{};  // times of the last cap reversals
  std::size_t wb_sat_head_ = 0U;
  bool wb_ring_fault_ = false;
  struct WbGain { float theta0; const std::array<std::array<float, wb::kStates>, 3>* lo;
                  const std::array<std::array<float, wb::kStates>, 3>* hi; float a;
                  const std::array<std::array<float, wb::kStates>, 3>* soft_lo;
                  const std::array<std::array<float, wb::kStates>, 3>* soft_hi; };
  [[nodiscard]] WbGain wbGain() const noexcept;
};

[[nodiscard]] const char* toString(V3Controller::State state) noexcept;

// The profile: the simulator-tuned LQR plus the hardware-proven deltas
// (adaptive balance angle seeded at start, anti-windup, full pitch feedback)
// and a conservative ground speed cap for bring-up.
[[nodiscard]] V3Controller::Config defaultV3Config();

// Pitch estimator while balancing on the floor. The shipped estimator pulls
// pitch toward the accelerometer at 55 /s; with the wheels pushing on the
// floor the specific force follows the contact force, and the accelerometer
// sees only ~17 % of the real tilt (IMU near the centre of percussion), so
// the loop goes blind and falls. Balancing uses a gyro-dominant estimate:
// slow accelerometer correction, no fast Kalman branch, no quiet-time gyro
// bias learning (it acts as a high-pass on the pitch rate). Idle/Locking/
// Ready keep the base config (accelerometer is right when held or still).
[[nodiscard]] robot_control::ImuStateEstimator::Config balancingEstimatorConfig(
    robot_control::ImuStateEstimator::Config base);

}  // namespace tap_fw::v3
