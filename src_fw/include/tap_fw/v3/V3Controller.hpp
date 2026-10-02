#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

#include "robot_control/ImuStateEstimator.hpp"
#include "robot_control/LqrBalancer.hpp"
#include "tap_fw/v3/LegHeight.hpp"
#include "tap_fw/v3/WholeBodyGains.hpp"

namespace tap_fw::v3 {

// v3: a plain two-wheel inverted pendulum. The four leg joints are locked by
// LegLock; this class only sequences the start and runs the wheel balancer.
//
// Start flow (operator): robot lifted on the stand -> START -> legs lock ->
// Ready (wheels hold still, no balance loop) -> operator carries the robot to
// the floor and holds it at its best standing point -> releases it -> the
// balance loop engages (auto: a move, then >= still_us of stillness arms it,
// then the first tip engages; or the GO command).
class V3Controller {
 public:
  enum class State { Idle, Locking, Ready, Balancing, Fault };

  // WholeBody (SET algo 2): one LQR for wheel + hip + knee (WholeBodyGains.hpp,
  // src_fw/tools/wb_lqr_design.cpp); Lqr with the tiers stays the default.
  enum class Algo { Lqr, Pid, WholeBody };

  // Classic two-loop balancer of hobby two-wheel robots (jjrobots B-ROBOT,
  // Balanduino, ESP32 balancing bots): a fast angle PD whose output is wheel
  // acceleration, integrated into the speed command sent to the drivers, and
  // a slow speed PI whose output is the angle to hold (its integral finds the
  // balance point). Units: acceleration m/s^2, angle rad, speed m/s.
  struct Pid {
    float kp = 30.0F;          // m/s^2 per rad of (pitch - pitch_ref)
    // kp 30 / kd 4 / lead 0.12 s: robustness sweep in the physics plant
    // (mass x0.7-1.3, inertia x0.7-1.4, COM height x0.8-1.2, wheel lag
    // 0.05-0.2 s, command delay 4-40 ms): 318/324 stand, no falls; without
    // the lead the old kp 30 / kd 5 stood 53/108 (wheel lag 0.15 s rings).
    float kd = 4.0F;           // m/s^2 per rad/s of pitch rate
    float kv = 0.10F;          // rad of lean per m/s of speed error
    float kx = 0.10F;          // rad of lean per m of position error
    // Slow integral of position error into the balance angle theta_b: it is
    // what finds the true balance angle (rad per m*s). 0.04 already rings.
    float kix = 0.02F;
    float corr_max_rad = 0.20F; // |kv*ev + kx*ex| clamp; integrator frozen there
    float ref_max_rad = 0.20F;  // |theta_b - seed| limit
    float accel_max_m_s2 = 8.0F;
    // Wheel-lag lead: the drivers are speed servos lagging ~0.1-0.15 s, so
    // the sent command leads the integrated speed by accel * lead_s (a
    // first-order servo then realises the commanded acceleration sooner).
    float lead_s = 0.12F;
    // Operator drive speed ramp [m/s^2] (was a step). 0.2: a 0.55 m/s
    // command peaks at ~0.74 m/s in the sim, under the 0.8 cap (0.5 -> 0.80).
    float drive_accel_m_s2 = 0.2F;
    // Wheel deadband compensation [rad/s]: the BL4820 wheels barely move
    // below ~1 rad/s (2026-09-27 floor log; a 1 rad/s deadband in the plant
    // reproduces the measured 0.19 deg / 1.6 cm sway at 0.46 Hz). Added
    // smoothly: w + comp * tanh(w / deadband_eps). 0.6/0.3 cuts the plant's
    // sway 0.0032 -> 0.0013 rad; 0.8 over-drives a light plant without a
    // deadband (physics tests T7/T9).
    float deadband_comp_rad_s = 0.6F;
    float deadband_eps_rad_s = 0.3F;
    float speed_lpf_hz = 5.0F; // speed-loop measurement filter
    // Command may lead the measured wheel speed by at most this (stalls).
    float tracking_margin_m_s = 0.30F;
    // Held/lifted detection: over each window, if the speed command changed
    // by more than held_dv but the body never rotated faster than held_rate,
    // the body is not free (hand, stand, wheels in the air). A free pendulum
    // under 1 m/s^2 would reach well over 0.1 rad/s in that time. Instead of a
    // runaway, the loop re-zeroes: command 0, position here, balance angle =
    // current pitch. So wheels answer a tilt on the stand and stop again.
    float held_dv_m_s = 0.25F;
    // ... and the body's horizontal acceleration stayed below this (a free
    // robot follows the wheels: ~dv/window >= 1.6 m/s^2).
    float held_accel_m_s2 = 0.6F;
    float held_rate_rad_s = 0.10F;
    std::uint64_t held_window_us = 150'000U;
  };

  // Ready-state auto engage (IMU pitch rate / pitch only).
  // Roll (left/right leg length difference). The planar controllers hold
  // pitch; this keeps the body level across: on a cross slope, over a
  // one-wheel bump, with legs arranged unevenly by hand - and leans it into
  // turns. The roll estimate is the attitude against the APPARENT vertical
  // (gyro + accelerometer, gravity plus the turn's centripetal pull), so
  // holding it at 0 levels the body on a slope and leans it by
  // atan(v w / g) in a turn, where the resultant of gravity and the
  // centripetal force passes between the wheels. Laterally the robot stands
  // on two wheels (no inverted pendulum), so the loop is a slow integral on
  // the side height difference through the leg IK. Off by default (SET roll).
  struct Roll {
    bool enabled = false;
    float est_tau_s = 0.5F;     // accelerometer correction of the roll estimate
    float track_m = 0.309F;     // wheel track (2 x half_wheel_base)
    // Integral time constant and proportional part. With the estimate on
    // gravity (gyro-fast, the turn's lean as the target) the loop can be
    // quick: sim, a 10 mm step under one wheel 0.030 -> 0.019 rad peak and
    // back under 0.005 in 0.33 s (was 0.52), turn entry 0.041 -> 0.012.
    // Hardware 2026-10-01 (rollvff on: kp 0.75 / tau 0.08): the side legs
    // hunted +-2 mm every 40 ms with the roll estimate jittering +-0.015
    // rad - the robot visibly rocked side to side; the sim's legs follow
    // ideally and have no such vibration. Slow and filtered by default: the
    // roll jobs that matter (slopes, uneven legs, the lean-in through its
    // feed-forward) are slow anyway.
    float tau_s = 0.40F;
    float err_lpf_hz = 2.0F;    // low-pass on the roll error the loop acts on
    float max_dh_m = 0.025F;    // |side height offset| (each side +-)
    // Kept below the full height while on (the straight knee can only fold):
    // +-reserve of roll at the top, ~3.7 deg with the 309 mm track.
    float reserve_m = 0.012F;
    float rate_m_s = 0.08F;     // side height offset slew
    float turn_ff = 1.0F;       // lean-in weight (target atan(v w / g))
    float kp = 0.2F;            // proportional part (1 = the whole tilt at once)
    // Servo velocity feed-forward (SET rollvff): the side offsets' rate goes
    // to the GDS68 with each target (Set_Input_Pos vel_ff), so the knee
    // follows a ramp without its ~67 ms position-loop lag, and the loop
    // runs faster (kp_vff / tau_vff). Sim, 10 mm under one wheel: peak
    // 0.019 -> 0.013 rad, back under 0.005 in 0.33 -> 0.20 s; turn entry
    // 0.012 -> 0.006. The fast gains WITHOUT the feed-forward (the driver
    // ignoring it) are worse in turns (0.057), so both switch together.
    bool vel_ff = false;
    float kp_vff = 0.3F;
    float tau_vff = 0.25F;
    // Roll the IMU reads with the robot level (its mount): standing on the
    // floor before engage it read -0.0405 (2026-10-01), so leveling to 0
    // tilted the body ~2.3 deg and parked the legs at -10 of +-12 mm.
    // SET rolltgt re-trims it.
    float target_rad = -0.040F;
    // Turn command slew while on [rad/s^2]: the lean needs ~0.2-0.3 s
    // (side height slew + knee servo); a stepped yaw pulled the body
    // sideways by the full atan(v w / g) before it could lean.
    float yaw_accel_rad_s2 = 4.0F;
    // Side mapping of the correction: -1 if the IMU's body +y turns out to
    // be the robot's right (roll=+ in TR when the RIGHT side is lifted) -
    // checked on the stand before the first floor run (SET rollsign).
    float sign = 1.0F;
  };

  // Wheel speed predictor (Smith style). Hardware 2026-10-01: the measured
  // wheel speed trails the command by ~75 ms, but the robot behaves as if
  // the wheels themselves answer in ~20 ms + a 35 ms lag and the rest is
  // the speed READING's age (driver estimate + UART read): only that split
  // reproduces both the tiers' calm stand (pitch rate rms 0.03-0.07 rad/s)
  // and the whole-body LQR's 2-4 Hz ringing in the sim. The balancers then
  // act on a 55 ms old axle speed/position. The predictor runs a model of
  // the wheel servo on the commands sent and adds what it moved over the
  // reading's age to the reading: speed(t) = read + w_model(t) -
  // w_model(t - meas_delay), position likewise.
  // Knee VMC (v3.4, SET vmc, default off): the knees run the GDS68 MIT law
  // T = Kp (q* - q) + Kd (qd* - qd) + T_ff in the driver, so each leg is a
  // vertical spring-damper the body rides on - a sharp bump or a hand on
  // the body is taken by the driver's own PD at its rate, not after a CAN
  // round trip. The hips stay position servos (fore/aft: the tiers).
  //  - Kp, Kd: a body-on-knee vertical spring of freq_hz / zeta mapped to
  //    the joint by the leg Jacobian, (dh/dk)^2 - the same feel at every
  //    height;
  //  - T_ff: gravity of the leg's share standing on its wheel (x grav_scale),
  //    sky-hook -sky * v_body (replaces Tier::suspension's height offsets)
  //    and an integral of the knee error (model error, bumpless hand-over);
  //  - stiff (kp_stiff) until balancing on the floor, and again while held
  //    or lifted (gravity compensation would push the legs out).
  // q* is the knee command as before (pose, knee hold, roll).
  // A knee turn moves the axle along (dx/dk, dh/dk): at the hang pose
  // (-0.097, 0.053) m/rad - more fore/aft than vertical, so a soft knee
  // there is a wobbling wheel, not a suspension - crouched (81-96 mm)
  // (-0.05, 0.10). The softness fades in with that direction's vertical
  // share between vert_stiff and vert_soft (TIER_VMCGRAV); a vertical
  // spring at the tall poses needs the hips too (later).
  struct Vmc {
    bool enabled = false;
    float freq_hz = 5.0F;        // SET vfreq
    float zeta = 0.6F;           // SET vzeta
    float sky_n_s_m = 80.0F;     // SET vsky: sky-hook per leg [N s/m]
    float ki_s = 0.5F;           // SET vki: integral time of the knee error
    float i_max_nm = 3.0F;
    float grav_scale = 1.0F;     // SET vgrav
    float kp_stiff = 40.0F;      // [N m/rad] not balancing / held
    float kd_stiff = 1.0F;
    float kp_min = 2.0F, kp_max = 60.0F;
    float kd_min = 0.05F, kd_max = 3.0F;
    float tau_max_nm = 8.0F;     // |T_ff| (GDS68 stall 11)
    float blend_s = 0.5F;        // stiff -> soft after the release
    float stiffen_s = 0.1F;      // soft -> stiff when held / lifted
    float vel_lpf_hz = 20.0F;    // q* rate feed-forward
    float vert_stiff = 0.60F;    // dh / |(dx, dh)| at or below: stiff
    float vert_soft = 0.85F;     // at or above: the full spring
    // Adaptive impedance, from what the knees feel (encoders only):
    //  - progressive spring (SET vprog, default off): stiffer with the
    //    leg's vertical give |dh/dk x (q* - q)| between prog_lo_m and
    //    prog_hi_m, up to prog_ratio x the base spring. Off: no measurable
    //    effect in the sim (bumps, presses, rough roads identical; the knee
    //    hardly gives under a 5 N press - the extra tilt there is not sag)
    //    and +0.3 N m knee torque on rough roads. For trials on the robot;
    //  - roughness damping (SET vadapt, default off): Kd towards kd_hard as
    //    the knees get pounded (rms of their vertical give speed between
    //    rough_lo and rough_hi). Off: with the tyre-filtered dense rough
    //    road (TIER_ROUGH) it cost the comfort (vertical accel rms 0.91 ->
    //    1.51 m/s^2 at 0.5 m/s) without saving a fall - the falls on the
    //    1.5x road are wheel traction, the stiff knee fell there too.
    bool progressive = false;
    float prog_lo_m = 0.003F;      // SET vplo
    float prog_hi_m = 0.010F;      // SET vphi
    float prog_ratio = 4.0F;       // stiffness x at prog_hi (SET vpratio)
    bool adapt = false;
    float rough_attack_s = 0.05F;
    float rough_release_s = 0.8F;
    float rough_lo_m_s = 0.02F;    // SET vrlo
    float rough_hi_m_s = 0.06F;    // SET vrhi
    float kp_hard = 40.0F;
    float kd_hard = 4.5F;
  };

  struct WheelPredict {
    // On with the whole-body LQR (its gains are designed for a fresh
    // reading); opt-in for the tiers (their wheel lead was tuned on the
    // delayed one - with the predictor too they lose margin on other wheel
    // models in the sim).
    bool enabled = false;          // SET wpred (tiers)
    bool with_wb = true;           // SET wbpred
    float act_delay_s = 0.020F;    // command -> wheel dead time
    // Wheel speed lag of the model. Measured 0.035 s, set long on purpose:
    // with 0.06 the WB stays calm for a true lag of 0.035..0.12 s (hard
    // stop at 0.75 m/s, TIER_WB); matched at 0.035 it rings from 0.06 up,
    // at 0.08 it rings at the measured lag.
    float tau_s = 0.06F;           // SET wptau
    float meas_delay_s = 0.055F;   // age of the speed/position reading
  };

  struct Engage {
    // Engage the balance loop as soon as the legs are locked (the held/
    // lifted detection keeps it from running away on the stand). When false,
    // Ready waits for the auto sequence below or GO NOW. Off (SET imm): held
    // in a moving hand the held test kept reading "free" (the hand turns the
    // body), and for ~100 s in the air the wheels raced to 0.8 m/s, the hips
    // swung +-0.2 rad, the knees +-8 mm and the level loop wound to 0.16 rad
    // (hardware 2026-10-01) - the robot then met the floor in that state.
    // Ready keeps the wheels still and the tiers off until it is held still
    // on the floor (armed) and let go - SET imm 0 tries that; default stays
    // on until it is tried on the robot (v3_tests assume it).
    bool immediate = true;
    bool automatic = true;
    // Algo::WholeBody engages only through the armed sequence (immediate and
    // GO ignored): its first hardware run (2026-10-01, 386 s) engaged at once
    // from the stand/hand, the held test never fired (the swinging legs shook
    // the body) and the wheels slammed +-20 rad/s at 1.5 Hz with the hips
    // +-0.2 rad - in the sim the same start is calm only because the stand
    // there holds the body still.
    bool wb_armed_only = true;
    float move_rate_rad_s = 0.40F;    // carrying the robot off the stand
    float move_pitch_rad = 0.08F;
    float still_rate_rad_s = 0.08F;   // held still at the standing point
    std::uint64_t still_us = 700'000U;
    float tip_rate_rad_s = 0.12F;     // released: starts to tip
    float tip_pitch_rad = 0.02F;
  };

  // v3.1 height change while balancing (both algorithms).
  struct Height {
    float rate_m_s = 0.015F;          // hip-axle height change speed
    // Height speed changes at most this fast (start, stop, slow-down): the
    // speed used to step 0 <-> 15 mm/s in one tick.
    float accel_m_s2 = 0.05F;
    // Speed scales down smoothly with the balance quality and reaches zero
    // at these: (1 - |pitch - ref| / err) * (1 - |pitch_rate| / rate). A hard
    // on/off gate here (0.05 rad / 0.5 rad/s) stopped and restarted the legs
    // 14 % of a 188 -> 78 mm lowering (2026-09-30), and each stop/start kick
    // fed the ~5 Hz pitch wobble that tripped the gate again.
    float max_pitch_error_rad = 0.10F;
    float max_pitch_rate_rad_s = 1.0F;
    std::uint64_t held_suppress_us = 400'000U;  // after the legs stop
    // Scale of the CAD balance-angle feed-forward. The CAD mass model has the
    // wrong sign on this robot: measured shifts were -0.025 rad at 110 mm
    // (2026-09-27, CAD +0.054 -> ratio -0.46) and -0.03 rad at 78 mm
    // (2026-09-30, CAD +0.097 -> ratio -0.31). Without it the trim relearns
    // too slowly and the robot parked 12-17 cm away after a height change;
    // -0.38 (between the two) cut the sim drift over 188->78 mm from 24-35 cm
    // to 3-8 cm for both algorithms. Runtime: SET hff.
    float balance_ff_scale = -0.38F;
  };

  // v3.3 two-tier control, Ascento style. Both flags off = v3.2 exactly.
  //  - Tier 1 (wheels, LQR or PID unchanged) balances the virtual pendulum,
  //    the axle -> whole-robot CoM line: theta_v = body pitch + com_line(
  //    measured hip/knee) - com_line(hang pose). At the hang pose theta_v is
  //    the body pitch, so every v3.2 threshold, trim and seed keeps its units.
  //  - Tier 2 (hips) takes the tier-1 lean: the hips turn by lean / J, J =
  //    d com_line / d hip, so the legs lean under the CoM and the body stays at
  //    its balance pitch. theta_v includes the measured hip, so this is the
  //    body-pitch feedback theta_b -> target written through the geometry
  //    (q_cmd = q_meas + (theta_b - target) / J): zero steady error, and no
  //    loop through the hip servo lag beyond the encoder age.
  struct Tier {
    // Default ON (operator, 2026-10-01: all algorithms always on, no GUI
    // toggles). SET vpitch 0 (outside balancing) returns to v3.2 exactly;
    // the tiers also stay inert without measured legs (legs_measured) or
    // with the PID algorithm.
    bool virtual_pitch = true;   // SET vpitch 0/1 (not while balancing)
    bool trunk = true;           // SET trunk 0/1; needs virtual_pitch (LQR only)
    // Body CoM fore/aft correction of the CAD mass model [m, body frame].
    // Fitted to the measured balance-angle shifts with height (188 -> 110 mm
    // -0.025 rad, 188 -> 78 mm -0.03 rad): +21 mm predicts -0.014 / -0.036,
    // the CAD alone +0.054 / +0.097 (wrong sign).
    float body_com_dx_m = 0.021F;
    float lean_share = 1.0F;   // share of the lean the hips take (1 = body level)
    // The share may fall with the pendulum length as (l / l_hang)^exponent,
    // the exponent blending from height_exp_tall at pendulum ratio >=
    // ratio_tall to height_exp at <= ratio_low. Both 0 now: the hips take
    // the whole lean at every height.
    //
    // History: 1.5 crouched / 0.5 tall was chosen on the earlier wheel model
    // (50 ms + 0.09 s lag, fresh reading - wrong, 2026-10-01 logs): there a
    // full share rang at <= 110 mm, and on the robot at 84 mm (share 0.30)
    // the body took ~90 % of a push. On the measured wheel (TIER_EXP) the
    // ringing is the lean-rate lead's, not the share's: with a fixed 0.06 s
    // lead a full share rings at 78-84 mm (rest hip rate 0.47-0.64 rad/s),
    // with the lead scaled to the pendulum (lead_exp) it stays calm. Push at
    // 84 mm, 8 plant variants: body share 0.73 -> 0.19, worst body 0.135 ->
    // 0.034 rad; 189 mm unchanged.
    float height_exp = 0.0F;
    float height_exp_tall = 0.0F;
    float ratio_tall = 0.80F;
    float ratio_low = 0.55F;
    // Lean-rate lead against the hip servo lag, scaled by the pendulum
    // ratio as ratio^lead_exp: 0.5 keeps it a fixed fraction of the
    // pendulum's time constant sqrt(l / g) (0.06 s at the hang pose, ~0.04
    // at 84 mm). Fixed 0.06 rang crouched; fixed 0.03 is calm crouched but
    // gives up ~25 % of the tall push body (TIER_EXP, 78-189 mm).
    float lead_s = 0.06F;
    float lead_exp = 0.5F;
    // The hips take the lean's changes; a lean held longer than about this
    // [s] (a balance-trim error while it still learns, a slope) goes back to
    // the body, as in v3.2. Without it the body copied every trim error
    // (seed 0.08 rad off: body 0.08 off for > 10 s) and spent hip range on it.
    // The static body attitude is the level loop's job (below), which
    // measures the body itself instead of trusting the trim.
    float hpf_s = 3.0F;
    // Static "gimbal" (SET tlvl, needs trunk): a slow integrator on the IMU
    // body pitch adds its own hip offset until the body sits at
    // level_pitch_rad, whatever the release angle, trim quality, height pose,
    // hand-arranged leg zero or CoM model error. Tier 1 keeps the CoM over
    // the axle through theta_v, so turning the hip only re-orients the body
    // against the legs; it does not unbalance the robot. The high-passed
    // dynamic law above stays the fast part. Hardware 2026-09-30 logs: the
    // free-standing body sat at -0.085 rad with the legs at the pose (one
    // session +0.022), so level 0 costs ~0.085 rad of hip at J ~= 1.
    bool level = true;
    float level_pitch_rad = 0.0F;  // SET tlevel: target body pitch (IMU frame)
    float level_tau_s = 1.5F;      // SET tltau: integrator time constant
    // SET tlvmax: |static hip offset| allowed. It also absorbs legs arranged
    // off by hand at power-up (the encoder zero): hip 0.10 + knee 0.08 rad
    // off needed 0.135 and left the body ~1 deg tilted at 0.12 (sim V25).
    float level_max_rad = 0.20F;
    float level_lpf_hz = 0.5F;     // body pitch low-pass feeding the integrator
    float max_rad = 0.15F;     // |dynamic hip offset| from the static height pose
    float rate_rad_s = 3.0F;   // hip command slew
    float ramp_s = 1.0F;       // tier-2 authority ramp 0 <-> 1
    // Tier 2 waits this long after the balance loop engages: the release
    // catch (a seed error of up to 0.08 rad) is left to tier 1 as in v3.2,
    // and the high-pass starts from the settled lean.
    float settle_s = 2.0F;
    // Plausibility of the virtual pitch: the com_line of the MEASURED legs
    // must stay within this of the commanded pose's (plus J * tier-2 hip)
    // for the whole check window, or it faults (a hip/knee sign or zero
    // wrong, a joint that let go). Sim worst case (hip servo lag under a
    // push) is ~0.07 rad; a wrong hip sign at 110 mm reads ~1 rad.
    float model_check_rad = 0.12F;
    float model_check_s = 0.3F;
    // Weight of the joint-velocity term in theta_v_rate (the balancer's D
    // channel): 1 = full CoM-line rate, 0 = gyro only (as v3.2).
    float joint_rate_gain = 1.0F;
    // Early hand-back to the body when the WHOLE system, not just the hip,
    // is out of headroom. A hard clamp at max_rad alone lets tier 2 hold
    // full authority (body at zero lean) right up to the wall and then hand
    // the WHOLE shortfall to the body in one step - exactly when a hard
    // stop from near vmax also pins the wheel command at its cap, with no
    // time left for the body to catch up (two-body sim, 0.12 s wheel lag:
    // v3.2 alone falls at 0.72 m/s, tier 1 alone survives to 0.78, trunk
    // with a hard clamp fell at 0.75).
    //
    // The fix gates on BOTH signals: the authority target falls off
    // linearly once the full-authority desired hip offset passes sat_margin
    // of max_rad, but only while the wheel command is also within
    // wheel_sat_margin of its own cap (derived from max_ground_velocity_m_s
    // and the wheel radius). A demanding but recoverable disturbance (a
    // push under wheel lag/delay) can ask a lot of the hip for a while
    // without the wheel ever saturating - tier 1 is still winning - and
    // fading there only costs body-level smoothness for no safety benefit
    // (found in the robustness sweep: gating on hip range alone regressed
    // the "wheel delay 70ms + push" case from 0.06 to 0.15 rad of body
    // pitch). The wheel actually pinned at its cap is the signal that
    // matters: at that point tier 1 alone cannot help further and whatever
    // lean the hip was withholding needs to reach the body now. Applied
    // every tick with no hold timer (the desired offset oscillates through
    // a catch transient; a timer that resets on every dip below the margin
    // never accumulates in time). Down uses sat_fade_s (fast: give the lean
    // back before it is needed elsewhere); up uses the normal ramp_s (slow:
    // only take it back once the stress is well clear).
    // Maneuver preview (SET tprev, LQR path; independent of vpitch/trunk).
    // A balancing robot answering a speed step reacts late and backwards:
    // to brake it first speeds up to get the wheels ahead of the CoM
    // (two-body sim: +0.10 m/s at 0.55, +0.20 at 0.70 before braking),
    // and the BL4820 adds ~125 ms on top. Commanded maneuvers are known in
    // advance, so: the operator speed becomes a jerk-limited plan; the
    // balancer tracks it preview_s late with the planned lean atan(a/g) as
    // its pitch reference and the plan's acceleration fed forward; the
    // wheels get the plan preview_s early plus the h*jerk/g contact shift
    // that builds the lean (LIP / ZMP preview, as legged robots do).
    //
    // Needs virtual_pitch (and its kinematic IMU compensation): the planned
    // lean is only as good as the pitch estimate during a sustained
    // acceleration, and the gyro-dominant estimator alone drifts ~0.02 rad
    // there - v3.2 + preview over-leaned and ran into vmax at 0.70 m/s.
    // Plan 0.3 m/s^2 / 1 m/s^3 matches v3.2's own command slew: 0.6 / 3
    // doubled the body acceleration peaks and ran out of wheel headroom.
    bool preview = true;
    float preview_s = 0.12F;
    float plan_accel_m_s2 = 0.3F;
    float plan_jerk_m_s3 = 1.0F;
    // Planned speed cap. With preview the robot really reaches the command
    // (v3.2's own station slew held a 4 s drive at ~0.4 m/s), and cruising
    // within 0.05 m/s of the 0.8 m/s wheel cap left no headroom to stand the
    // lean back up: 0.75 fell with a 0.12 s wheel lag, 0.70 held; with a
    // 70 ms wheel dead time 0.60 still fell. 0.55 (= the BLE full stick)
    // held every plant variant (75/75, v3_tier_tests V15).
    float plan_vmax_m_s = 0.55F;
    // Motion envelope from the pendulum at the current height (SET tenv):
    // crouched is how the robot is driven fast. Catching a lean theta needs
    // about theta * sqrt(g l) of wheel speed beyond the cruise (capture
    // point), so the planned speed cap is the wheel cap minus
    // env_catch_rad * sqrt(g l) - 0.55 m/s at the hang pose (the validated
    // plan_vmax above), ~0.63 at 84 mm. A lower CoM is also a faster
    // pendulum (omega = sqrt(g / l)), so the plan's jerk scales with
    // omega / omega_hang; its acceleration (a lean of atan(a/g)) does not
    // depend on the height. Off: plan_vmax / plan_jerk as fixed numbers.
    // Sim (TIER_SPEED, crouched 0.75 m/s command + 3 N push mid-cruise):
    // Algo::WholeBody held every slow-wheel variant (cruise 0.70 at 84 mm,
    // body <= 0.045 rad); the tiers fell with a 0.12 s lag or 70 ms dead
    // time - so it is on with WB (envelope_wb) and opt-in for the tiers.
    bool envelope = false;
    bool envelope_wb = true;
    // With the whole-body LQR the legs take the planned lean and pin the
    // body, so the planned acceleration is the lean budget given to them:
    // g * tan(env_lean_rad) (0.10 -> ~1 m/s^2, 3x plan_accel). Sim: body
    // <= 0.039 rad at 84-189 mm with the slow-wheel variants, CoM lean up to
    // 0.17 (the legs swing, the body stays).
    float env_lean_rad = 0.10F;
    // Knee keeps the hip-axle height while the hip swings (exact leg IK,
    // SET khold): the pose puts the axle straight under the hip, so a hip
    // swing dq moves the wheel fore/aft at first order but drops the body by
    // ~r (1 - cos dq) - a linear controller (the WB LQR included) cannot
    // see that. The knee folds/extends by the IK solution for the hip it is
    // being sent. At the full height the knee is already straight and can
    // only fold: there the drop stays. On for both (the tiers since the
    // hips take the whole lean crouched, up to ~0.16 rad: TIER_ACCEL at
    // 84 mm, body height change while driving 1.3 -> 1.0 mm, body pitch and
    // the random episodes unchanged). SET khold / wbkhold 0: off.
    bool knee_hold = true;
    bool knee_hold_wb = true;
    float env_catch_rad = 0.20F;
    // Tier 2 takes the planned lean from the plan this much ahead of the
    // tracked reference (the GDS68 hip servo lags ~60-90 ms).
    float hip_advance_s = 0.08F;
    // The planned lean (the plan's atan(a/g), a commanded speed-up or
    // braking) goes to the legs in full at every height; the height share
    // above cuts only the feedback on the residual lean (pushes, errors).
    // The share exists because that feedback loop rings with a crouched,
    // faster pendulum; the planned lean is open loop - it cannot ring - and
    // cut by the share the body took (1 - share) of every commanded
    // acceleration (70 % at 84 mm). SET tplanfull 0: scaled by the share.
    bool plan_full = true;
    // Low-pass on the lean rate feeding the tier-2 lead [Hz, 0 = off]: the
    // lead's derivative gain was the main source of body translational jerk
    // (two-body sim, drive 0.55 m/s: jerk rms 0.9 without tier 2, 3.8 with
    // the unfiltered 0.06 s lead). Off by default: every cut-off tried
    // (10/15/20/30 Hz) cost robustness somewhere - crouched push wobble
    // (<= 20 Hz) or the hard-stop margin with a 0.12 s wheel lag (even
    // 30 Hz fell at 0.75 m/s). The planned-lean split above removes most of
    // the lead's work during commanded maneuvers instead.
    float lead_lpf_hz = 0.0F;
    float sat_margin = 0.6F;
    float wheel_sat_margin = 0.9F;
    float sat_fade_s = 0.2F;
    // Not done, measured in the two-body sim (v3_tier_tests TIER_AB):
    //  - GDS68 velocity feed-forward of the tier-2 hip rate: that rate holds
    //    the measured hip (through theta_v), so vel_ff fed the delayed hip
    //    speed back positively - standing wobble 0.02 -> 0.34 rad/s;
    //  - extrapolating the ~96 Hz joint samples by velocity x age: no gain,
    //    more velocity noise in theta_v.
    // Feed the IMU estimator the kinematic velocity of the IMU point (axle
    // odometry + legs + rotation) while balancing free on the floor, so a
    // sustained acceleration no longer tilts the pitch estimate. v3.0 turned
    // this off because a held body with spinning wheels read as a fake
    // acceleration: here it is gated off while held/lifted. Sim: pitch
    // estimate error while driving 0.018 -> 0.004 rad, stop creep 5-15 cm ->
    // ~1-5 cm. Needs virtual_pitch (measured legs). SET timu.
    bool imu_kinematic = true;
    // Knee suspension (SET tsus, needs vpitch): the knees keep the body's
    // height in space instead of following every floor bump, like a
    // skater's knees - "sky-hook" damping on the body's vertical speed from
    // the IMU (specific force along gravity, leaky-integrated, so pitch
    // errors only enter to second order), the height offset easing back to
    // the pose. The hips stay the fore/aft (lean) actuator: at every height
    // the pose's hip moves the body < 1.1 mm per 0.1 rad while the knee
    // moves it 5-10 cm per rad (v3_tier_tests TIER_RATIO).
    bool suspension = true;
    float sus_gain = 3.0F;        // height rate per body vertical speed (SET tsusg)
    float sus_return_s = 0.3F;    // offset back to the pose
    float sus_max_m = 0.008F;     // |height offset| (SET tsusmax)
    // Vertical speed estimate: IMU above this time constant, the legs' own
    // height rate (encoders; equal to it on a flat floor) below - an IMU-only
    // estimate drifted and the knees wandered 5-11 mm standing (sim).
    float sus_leak_s = 0.3F;
    // Soft deadband on that speed: standing, sensor noise reads ~1-3 mm/s
    // and moved the knees 1.4-3.6 mm; a 5 mm bump at 0.3 m/s reads ~50.
    float sus_deadband_m_s = 0.005F;
  };

  struct Config {
    Algo algo = Algo::Pid;
    Tier tier{};
    Roll roll;
    WheelPredict wheel_pred;
    Vmc vmc;
    Height height{};
    Engage engage{};
    Pid pid{};
    robot_control::LqrBalancer::Config lqr{};
    float com_height_m = 0.2145F;         // locked extended legs (CAD FK)
    // v3.2: how many pendulum-length points to pre-solve, one per tick, once
    // Locking starts (LqrBalancer::buildGainTableStep) so a live height
    // change only interpolates (cheap) instead of re-solving the Riccati
    // equation (~50 ms on the ESP32-S3's single-precision FPU).
    unsigned lqr_gain_table_points = 12U;
    // Wheel deadband push on the LQR path (the PID path keeps Pid's 0.6).
    float lqr_deadband_comp_rad_s = 0.3F;
    // Wheel-lag lead on the LQR path. The real BL4820 answers after ~50 ms
    // dead time + ~75 ms lag (fitted on the 2026-09-29/30 logs); with that
    // delay the PID's 0.12 s lead rang at ~4 Hz, 0 fell - 0.08 is the sim
    // optimum (see input_cost_scale in defaultV3Config).
    // 2026-10-01: that fit took the speed READING's 55 ms age for wheel lag.
    // On the wheel as measured (20 ms + 0.035 s, reading 55 ms old) 0.08
    // fell on the +20 ms / slower-lag variants (sim) - and the robot fell
    // twice that day from a growing ~1.2 Hz wheel oscillation; 0.03 held
    // every variant (worst rest pitch rate 0.06 rad/s), 0.02 fell again
    // (TIER_LEADSWEEP). SET lqrlead restores the old value.
    float lqr_lead_s = 0.03F;
    // LQR position-error trim integral [rad per m*s] (half the PID kix).
    float lqr_trim_gain = 0.01F;
    // Algo::WholeBody: rate [rad per m s] at which the position error teaches
    // the measured natural body pitch (IMU mount + CoM model error).
    float wb_bias_gain = 0.10F;
    // Algo::WholeBody limit-cycle guard: 4 cap-to-cap reversals of the wheel
    // command within this window fault (see wbUpdate). A push saturates one
    // way; the 2026-10-01 ringing flipped every ~0.33 s.
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
    // Algo::WholeBody: how far the position anchor may lead/lag the robot
    // while driving [m]; 0 = speed only. Leading by up to 8 cm did not get it
    // over an 8 mm / 4 cm bump at 0.15 m/s (stalls; the tiers climb it) and
    // added body tilt while driving (sim TIER_WB) - SET wbslack.
    float wb_drive_slack_m = 0.0F;
    std::uint64_t lock_settle_us = 1'200'000U;  // leg ramp + settle before wheels
    float fall_pitch_rad = 0.60F;
    // Auto-learned balance trim (theta_v-referenced, height-independent
    // while virtual_pitch is on). Once the robot has held its ground
    // position within learn_position_tol_m and rotated less than
    // learn_rate_tol_rad_s for learn_hold_s, the current trim is
    // remembered. Every later release this power-up (held/lifted -> free)
    // then walks the trim from the hold angle to the learned one over
    // learn_release_s, instead of balancing around whatever angle the
    // operator happened to put it down at and creeping back on the slow
    // position integral for 15-20 s (hardware 2026-10-01: nose-down or
    // nose-up before settling, depending on the release angle). While
    // still held the trim keeps tracking the hold angle: a learned trim
    // there made the wheels fight the hand and wind up (two-body sim: fell
    // after a 3 s hold at >= 0.075 rad). Needs virtual_pitch; inert
    // otherwise.
    float learn_position_tol_m = 0.03F;
    // Ordinary standing has continuous small pitch-rate noise (LQR/wheel
    // deadband micro-corrections, seen up to ~0.1-0.2 rad/s even while
    // genuinely calm) - too tight a tolerance here means the hold timer
    // resets before ever accumulating learn_hold_s.
    float learn_rate_tol_rad_s = 0.20F;
    float learn_hold_s = 2.5F;
    float learn_tau_s = 4.0F;  // slow re-tracking once learned (adapts, no jumps)
    float learn_release_s = 0.3F;
    // Wheel ground-speed cap while held/lifted (after a held re-zero, until
    // the body turns on its own again). In the air a balancer's own speed
    // feedback is positive feedback (it accelerates towards its motion to
    // lean back), so the wheels raced to the 0.8 m/s cap and re-zeroed every
    // 0.6-1 s on the stand (hardware 2026-09-30: axle_v +-0.6-0.9 m/s at
    // every re-zero) - and the robot was put on the floor with its wheels
    // spinning at whatever phase that cycle was in (two-body sim: fell
    // after a 3 s hold put down at 0.05 rad with the wheels at -10..-20
    // rad/s). A free robot on the floor lifts the cap within ~0.1 s: it
    // tips, and the body rate leaves the held state.
    float held_wheel_cap_m_s = 0.10F;
    // Below the 0.8 m/s ground-speed cap (20 rad/s): a lifted or held robot
    // saturates the wheels at the cap and must still trip this guard.
    float runaway_wheel_rad_s = 15.0F;
    std::uint64_t runaway_us = 500'000U;
    // ...only once the wheel ground speed is this far from the speed the
    // accelerometer integrates (leaking to the wheels over runaway_v_tau_s).
    float runaway_dv_m_s = 0.25F;
    // LQR drive anchor at most this far from the robot (0 = off, SET leash).
    float drive_leash_m = 0.10F;
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
    // odometry needs it once the legs move (v3.1).
    float leg_chain_rad = 0.0F;
    // v3.3: measured leg joints per side (left, right), core frame, offsets
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
    // v3.1: hip/knee offsets for both legs (core rad, from their zero).
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
    // Knee VMC (Config::Vmc): MIT gains and feed-forwards per side [left,
    // right], core frame; valid only with Vmc::enabled.
    struct KneeMit {
      float kp = 0.0F, kd = 0.0F;      // [N m/rad], [N m s/rad]
      float tff_nm = 0.0F;             // torque feed-forward
      float vel_rad_s = 0.0F;          // q* rate
    };
    std::array<KneeMit, 2> knee_mit{};
    bool knee_mit_valid = false;
    bool legs_moving = false;
    const char* fault = nullptr;  // set on the tick a fault is raised
  };

  V3Controller() : V3Controller(Config{}) {}
  explicit V3Controller(Config config);

  [[nodiscard]] Output update(const Input& input, const Command& command);
  // Knee torque holding one leg's share of the robot standing on its wheel
  // (CAD hip angle, core knee, body pitch) - the knee VMC's gravity term.
  [[nodiscard]] static float stanceKneeTorque(float hip, float knee, float pitch, float g) noexcept;
  [[nodiscard]] float runawayImuSpeed() const noexcept { return runaway_v_imu_; }
  // Knee VMC adaptation: roughness rms [m/s] and the blend to hard (0..1).
  [[nodiscard]] float vmcRoughness() const noexcept { return std::sqrt(vmc_rough_ms_); }
  [[nodiscard]] float vmcAdapt() const noexcept { return vmc_adapt_; }
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
  // Back to Ready (legs stay locked) so the next GO engages the real balance
  // loop, e.g. from the hand-held TILTTEST check.
  // KNEETEST FLOOR: knee offset [rad, |.| <= 0.1] added to the height pose's
  // knee while balancing; the hip follows through LegHeight::forKnee as for a
  // height change. Dropped as soon as the robot leaves Balancing.
  void setKneeExcitation(float dk_rad) noexcept {
    knee_excite_rad_ = std::clamp(dk_rad, -0.1F, 0.1F);
  }
  void returnToReady() noexcept {
    if (state_ == State::Balancing) {
      state_ = State::Ready;
      ready_phase_ = 0;
      still_since_us_ = 0U;
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
  // Ready sub-phase for the log: 0 wait move, 1 wait still, 2 armed.
  [[nodiscard]] int readyPhase() const noexcept { return ready_phase_; }
  // Number of held/lifted re-zeroes since the balance loop engaged.
  [[nodiscard]] int heldResets() const noexcept { return held_resets_; }
  // Held/lifted state: outer loop off (wheels capped), and still quiet.
  [[nodiscard]] bool heldOuterOff() const noexcept { return held_outer_off_; }
  [[nodiscard]] bool heldQuiet() const noexcept { return held_quiet_; }
  [[nodiscard]] float heightTarget() const noexcept { return height_target_m_; }
  [[nodiscard]] float heightSpeed() const noexcept { return height_speed_m_s_; }
  [[nodiscard]] bool legsHeld() const noexcept { return legs_held_; }
  [[nodiscard]] const LegPose& legPose() const noexcept { return pose_; }
  [[nodiscard]] bool lqrGainTableReady() const noexcept { return lqr_.gainTableReady(); }
  // Ground velocity/position used by the balancer (axle, rolling contact).
  [[nodiscard]] float groundVelocity() const noexcept { return ground_v_; }
  [[nodiscard]] float groundPosition() const noexcept { return ground_x_; }
  // v3.3: virtual pitch (equals the body pitch while virtual_pitch is off),
  // tier-2 hip offset on top of the height pose, and its authority 0..1.
  [[nodiscard]] float virtualPitch() const noexcept { return theta_v_; }
  [[nodiscard]] float virtualPitchRate() const noexcept { return theta_v_rate_; }
  [[nodiscard]] float trunkHip() const noexcept { return dhip_; }
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
  // Hip offset to send to the legs (height pose + tier 2), every state.
  [[nodiscard]] float hipCommand() const noexcept { return pose_.hip_offset_rad + dhip_; }
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
  // Measured minus expected com_line [rad] (the model check's signal).
  [[nodiscard]] float legModelError() const noexcept {
    return c_meas_ - c_pose_ - j_hip_ * dhip_;
  }

 private:
  Output fault(const char* reason, const Output& base);

  Config config_{};
  robot_control::LqrBalancer lqr_;
  State state_ = State::Idle;
  std::uint64_t state_since_us_ = 0U;
  std::uint64_t runaway_since_us_ = 0U;
  // Pitch when wheel speed first sustained above runaway_wheel_rad_s: a
  // genuinely held/lifted robot's body stays near this (low net pitch
  // travel) while the wheels race; a real, unrecovered fall has the body
  // moving steadily further from it throughout. A peak-rate gate here was
  // too strict (disqualified normal-catch transients, e.g. physics-sim T8)
  // and too permissive at once (a slow monotonic drift can stay under any
  // single-tick rate threshold); net displacement catches both correctly.
  // Without this the guard could not tell "held" from "falling" at all and
  // zeroed the command mid-fall on real hardware (2026-09-29), guaranteeing
  // the fall it should have let fall_pitch_rad catch cleanly instead.
  float runaway_start_pitch_ = 0.0F;
  float runaway_v_imu_ = 0.0F;  // accelerometer ground speed (runaway guard)
  std::uint64_t drive_seen_us_ = 0U;  // last tick with a nonzero drive command
  const char* last_fault_ = "";
  // The legs are structure in v3: once locked they stay locked through a
  // fault; only DISABLE (command.disable) releases them.
  bool legs_held_ = false;
  float ground_v_ = 0.0F;
  float ground_x_ = 0.0F;
  // v3.1 height state: the knee fold is ramped (cheap per tick), the
  // requested height is converted to a knee target once per request.
  float height_target_m_ = 0.0F;
  float knee_target_rad_ = 0.0F;
  float knee_cmd_rad_ = 0.0F;
  float height_speed_m_s_ = 0.0F;  // signed hip-axle speed, + = up
  LegPose pose_{};
  float chain_prev_rad_ = 0.0F;
  float chain_rate_f_ = 0.0F;
  bool chain_init_ = false;
  std::uint64_t legs_moved_us_ = 0U;
  void updateHeight(const Input& in, const Command& cmd);
  void applyPoseOffsets(const Input& in);
  void updateSuspension(const Input& in);
  float knee_excite_rad_ = 0.0F;
  float applied_dk_ = 0.0F, applied_base_ = 0.0F;
  float sus_g_ = 9.80665F, sus_vz_ = 0.0F, sus_dz_ = 0.0F, sus_dk_ = 0.0F;
  float sus_vz_eff_ = 0.0F;  // body vertical speed off the plan, past the deadband
  bool sus_init_ = false;
  bool excite_applied_ = false;
  // The virtual pitch already carries the balance shift with height (from
  // the measured legs), so the CAD feed-forward is off with it.
  [[nodiscard]] float ffPitch() const noexcept {
    return tierVirtual() ? 0.0F
                         : config_.height.balance_ff_scale * pose_.balance_pitch_delta_rad;
  }
  // v3.3 tiers.
  // LQR only: the PID path with the virtual pitch overshot a 0.55 m/s drive
  // past the speed cap and fell in the two-body sim (v3_tier_tests); the
  // PID stays the v3.2 fallback.
  [[nodiscard]] bool tierVirtual() const noexcept {
    return config_.tier.virtual_pitch && legs_measured_ && config_.algo == Algo::Lqr;
  }
  // com_line angle [rad] and its partials by hip/knee at core leg offsets.
  struct ComLine { float angle, d_hip, d_knee; };
  [[nodiscard]] ComLine comLine(float hip_offset, float knee_offset) const noexcept;
  void updateVirtualPitch(const Input& in);
  void updateTrunk(const Input& in, bool active, std::array<float, 2> wheel_cmd_rad_s);
  void updateImuVelocity(const Input& raw, bool free);
  // v3.3 maneuver preview: jerk-limited speed plan and its history.
  struct PlanSample { float v, a; };
  void updatePlan(float target_v, float dt, bool reset);
  [[nodiscard]] float planCap(float vmax) const noexcept;
  // Knee offset from the pose that keeps the pose's hip-axle height with the
  // hip at pose + dhip (Tier::knee_hold), 0 when off.
  [[nodiscard]] float kneeHold(float dhip) noexcept;
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
  // Knee VMC (Config::Vmc): fills out.knee_mit. soft: balancing free on
  // the floor (else stiff).
  void updateVmc(const Input& in, bool locked, bool soft, Output& out);
  [[nodiscard]] bool vmcOn() const noexcept { return config_.vmc.enabled; }
  float roll_est_ = 0.0F;      // vs the apparent vertical, + = left side up
  float roll_bias_ = 0.0F;     // learned roll gyro bias [rad/s]
  bool roll_init_ = false;
  float roll_dh_ = 0.0F;       // side height offset: left +roll_dh_, right -roll_dh_ [m]
  float roll_v_lp_ = 0.0F, roll_w_lp_ = 0.0F, roll_i_ = 0.0F, roll_e_lp_ = 0.0F;
  float yaw_cmd_ = 0.0F;  // the turn command as used (slewed with roll on)
  float roll_target_ = 0.0F;
  float roll_dk_dh_ = 0.0F, roll_dq_dk_ = 0.0F, roll_deriv_key_ = 1.0F;
  std::array<float, 2> roll_hip_{}, roll_knee_{}, roll_hip_rate_{}, roll_knee_rate_{};
  // Knee VMC state per side: integral [N m], soft blend 0..1, last q*, its rate.
  std::array<float, 2> vmc_i_{}, vmc_qprev_{}, vmc_qrate_{};
  float vmc_beta_ = 0.0F;
  bool vmc_init_ = false, vmc_armed_ = false;
  float vmc_rough_ms_ = 0.0F, vmc_adapt_ = 0.0F;  // roughness (mean square), blend to hard
  [[nodiscard]] bool envelopeOn() const noexcept {
    return config_.tier.envelope || (wholeBody() && config_.tier.envelope_wb);
  }
  [[nodiscard]] PlanSample planDelayed() const noexcept;
  [[nodiscard]] PlanSample planAt(float delay_s) const noexcept;  // plan delay_s ago
  float lean_rate_f_ = 0.0F;  // filtered lean rate for the tier-2 lead
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
  std::uint64_t model_bad_since_us_ = 0U;
  float dhip_ = 0.0F;
  float trunk_alpha_ = 0.0F;
  float lean_lp_ = 0.0F;  // slow part of the lean (tier-2 high-pass)
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
  // v3.2 LQR: absolute ground-position anchor (position_anchor_hold), moved
  // by cmd.forward_m_s while driving, frozen at the current position on
  // release - same job as pid_x_target_ but LqrBalancer takes it directly as
  // an absolute anchor (reanchorPosition), not a relative target.
  float lqr_x_target_m_ = 0.0F;
  // v3.2 LQR: low-pass filtered ground_accel for the wheel-lag lead term
  // (unfiltered, the raw instantaneous LQR solve - itself sensitive to gyro
  // and encoder noise via the pitch_rate/velocity state terms - rode straight
  // into the sent wheel command every tick and read as jerky/"vồ vập" on
  // real hardware even after filtering the velocity state term alone).
  float lqr_lead_filt_ = 0.0F;
  std::uint64_t held_window_start_us_ = 0U;
  float held_v0_ = 0.0F;
  float held_max_rate_ = 0.0F;
  float held_accel_sum_ = 0.0F;
  int held_accel_n_ = 0;
  float body_accel_f_ = 0.0F;
  float ax_f_ = 0.0F;         // 5 Hz body-frame accel_x
  float held_ax_ref_ = 0.0F;  // ax_f_ at the last held re-zero  // filtered horizontal body acceleration
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
