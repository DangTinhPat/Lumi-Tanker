#pragma once

#include <array>
#include <cstdint>

#include "robot_control/ImuStateEstimator.hpp"
#include "robot_control/LqrBalancer.hpp"
#include "tap_fw/v3/LegHeight.hpp"

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

  enum class Algo { Lqr, Pid };

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
  struct Engage {
    // Engage the balance loop as soon as the legs are locked (the held/
    // lifted detection keeps it from running away on the stand). When false,
    // Ready waits for the auto sequence below or GO.
    bool immediate = true;
    bool automatic = true;
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
    // The share falls with the pendulum length as (l / l_hang)^height_exp:
    // a crouched robot is a faster pendulum (1.25 Hz hang, 1.9 Hz at 78 mm)
    // while the hips stay a ~3-4 Hz servo. Two-body sim: the full share
    // rang at <= 110 mm (1 Hz body/leg wobble); 1.5 keeps 78-150 mm calm.
    float height_exp = 1.5F;
    // ...but only where the ringing lives: the exponent blends from
    // height_exp_tall at pendulum ratio >= ratio_tall to height_exp at
    // <= ratio_low. Hardware 2026-09-30 at 136 mm (ratio 0.72): the body
    // still took ~40 % of a push (1.5 -> share 0.61); 1.5 everywhere is only
    // needed <= 110 mm (ratio 0.58), and 1.0 everywhere rang at 78 mm.
    float height_exp_tall = 0.5F;
    float ratio_tall = 0.80F;
    float ratio_low = 0.55F;
    float lead_s = 0.06F;      // lean-rate lead against the hip servo lag
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
    float level_max_rad = 0.12F;   // SET tlvmax: |static hip offset| allowed
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
    // Tier 2 takes the planned lean from the plan this much ahead of the
    // tracked reference (the GDS68 hip servo lags ~60-90 ms).
    float hip_advance_s = 0.08F;
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
  };

  struct Config {
    Algo algo = Algo::Pid;
    Tier tier{};
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
    float lqr_lead_s = 0.08F;
    // LQR position-error trim integral [rad per m*s] (half the PID kix).
    float lqr_trim_gain = 0.01F;
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
    bool legs_moving = false;
    const char* fault = nullptr;  // set on the tick a fault is raised
  };

  V3Controller() : V3Controller(Config{}) {}
  explicit V3Controller(Config config);

  [[nodiscard]] Output update(const Input& input, const Command& command);
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
  // Static level loop's share of trunkHip() (Tier::level).
  [[nodiscard]] float levelHip() const noexcept { return level_beta_ * level_i_; }
  // Hip offset to send to the legs (height pose + tier 2), every state.
  [[nodiscard]] float hipCommand() const noexcept { return pose_.hip_offset_rad + dhip_; }
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
