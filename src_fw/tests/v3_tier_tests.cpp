// Tiered control on a planar multi-body plant (src_fw/tests/LegPlant.hpp). The
// v3_physics_tests plant is one rigid pendulum (legs locked); here the body
// (IMU), the thighs and the shins are separate bodies joined at the hips and
// knees, s = (x axle, theta body pitch, q hip offset, k knee offset):
//  - hips and knees: the GDS68 position servo fitted to the 2026-09-30
//    HIPTEST log (on the stand: position gain 16 /s, velocity loop 48 /s,
//    ~3 ms delay; rms fit error 3 mrad of 28 mrad), realised as a torque
//    servo on the real inertia so the floor load (the body) slows it as it
//    would; the knees use the same fit until KNEETEST data replaces it;
//    250 Hz command, ~96 Hz position/velocity feedback;
//  - wheels: speed servo on the wheel-to-shin angle with dead time, lag or a
//    resonant second-order response, whole-rpm command, deadband, torque cap;
//  - the IMU specific force at its body point through the real
//    ImuStateEstimator (balancing config), as AppV3 does;
//  - optional ground bumps/steps under the axle.
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <deque>
#include <random>
#include <string>
#include <vector>

#include "robot_control/GeneratedGeometry.hpp"
#include "robot_control/ImuStateEstimator.hpp"
#include "robot_control/WholeBodyModel.hpp"
#include "LegPlant.hpp"
#ifdef TAP_HAVE_EIGEN
#include "WbDesign.hpp"
#endif
#include "tap_fw/v3/JointTestSchedule.hpp"
#include "tap_fw/v3/LegLock.hpp"
#include "tap_fw/v3/LegHeight.hpp"
#include "tap_fw/v3/V3Controller.hpp"
#include "tap_fw/v3/AddWifiBalanceProfile.hpp"

namespace {

namespace geo = robot_control::geometry;
using robot_control::ImuStateEstimator;
using tap_fw::v3::LegHeight;
using tap_fw::v3::V3Controller;

constexpr double kG = 9.80665;
constexpr double kPi = 3.14159265358979323846;

struct Case {
  const char* name = "";
  V3Controller::Algo algo = V3Controller::Algo::Lqr;
  bool vpitch = false, trunk = false;
  double lean_share = -1.0, tier_lead = -1.0, tier_max = -1.0;
  double tier_exp = -1.0;         // < 0: firmware default (Tier::height_exp)
  int imu_kin = -1;              // -1: firmware default
  int preview = -1;              // -1: firmware default (Tier::preview)
  double plan_accel = -1.0, plan_jerk = -1.0, preview_s = -1.0, lead_lpf = -1.0;
  double joint_rate_gain = -1.0;  // < 0: firmware default
  // Plant truth vs. the controller's model.
  double body_dx = 0.021;       // true body CoM shift [m] (controller: 0.021)
  double imu_mount = 0.03;      // IMU pitch reading minus body pitch [rad]
  double seed_error = 0.02;     // released this far from the balance point [rad]
  double mass_scale = 1.0;      // body mass
  // Wheels.
  // 0: dead time + first-order lag (the fitted model). 1: resonant second
  // order - kept for the WHEELID follow-up, but as tried (2.5-5 Hz, zeta
  // 0.3-0.5) it rings plain LQR at rest too, which the robot does not do.
  int wheel_mode = 0;
  // Measured 2026-10-01 (TR frequency response + which split reproduces the
  // robot): ~20 ms dead time and a 35 ms lag, the speed/position READING
  // 55 ms old. Was 50 ms + 0.09 s with a fresh reading, which hid the
  // whole-body LQR's ringing.
  double wheel_tau = 0.035, wheel_wn_hz = 3.0, wheel_zeta = 0.35;
  std::size_t wheel_delay_ticks = 10;      // 20 ms
  std::size_t wheel_meas_delay_ticks = 27; // 55 ms wheel speed/position feedback latency
  int wheel_pred = -1;                     // Config::WheelPredict::enabled (-1: default)
  double deadband = 1.0, wheel_tau_max = 0.6;
  // Hips (per joint, doubled for two legs).
  double hip_kp = 16.0, hip_vel_bw = 48.0, hip_rotor = 0.004, hip_tau_max = 4.0;
  std::size_t hip_delay_ticks = 2;
  // Knees: no KNEETEST yet. Position gain as the hips' fit (16 /s); the
  // velocity gain calibrated to the one loaded fact we have - plain LQR, which
  // reads every knee motion as body pitch, stood calm on the robot
  // (2026-09-29 TR logs: pitch-rate rms 0.09-0.32 rad/s). The hip fit's
  // bandwidth on the knee's own inertia (bw 48 -> 0.35 N m s/rad) rang plain LQR
  // at 0.69 rad/s, the hips' absolute gain (0.84) at 0.66; bw 350 on the knee
  // inertia (2.6 N m s/rad, 41 N m/rad per knee) gives 0.28. The tiers barely
  // care (rest 0.02-0.04 rad/s, push body 0.015 rad over the whole sweep,
  // TIER_KNEECAL); kVariants carries a soft and a stiff knee. Torque cap at
  // the GDS68 rating (5 N m). knee_vel_bw < 0: the hips' absolute gain.
  double knee_kp = 16.0, knee_vel_bw = 350.0, knee_rotor = 0.004, knee_tau_max = 5.0;
  // Ground: a step of bump_h [m] at x = bump_x0, smoothed over bump_ramp; with
  // bump_width > 0 it comes back down that far on (a bump, not a step).
  double bump_h = 0.0, bump_x0 = 0.3, bump_ramp = 0.02, bump_width = 0.0;
  // Rough road (roughRoad): random small bumps, dips and gentle ramps from
  // x = 0.3 m over rough_len_m, seeded (0 = flat). Edges no steeper than
  // ~15 deg (the wheel climbs ~20-25 deg). rough_scale scales the heights.
  int rough_seed = 0;
  double rough_len_m = 4.0, rough_scale = 1.0;
  // Dense: a bump or dip (-6..+8 mm) every 4-10 cm, overlapping, on top of
  // long gentle waves (~15 mm, 0.8-1.5 m) - a continuously rough surface.
  bool rough_dense = false;
  int suspension = -1;          // Tier::suspension (-1: firmware default)
  // Legs arranged by hand before power-up: the encoder zero (the pose the
  // controller calls hip/knee 0) is this far [rad] from the intended hang
  // pose. The body then balances tilted until something corrects it.
  double hip_zero_err = 0.0, knee_zero_err = 0.0;
  double acc_noise_scale = 1.0;  // accelerometer noise (0.05 m/s^2 per axis per sample) scale
  double leg_fb_period_s = 0.0104;  // 0x009 estimate period (~96 Hz)
  double hip_vel_noise = 0.03;      // driver velocity estimate noise [rad/s]
  double knee_vel_noise = -1.0;     // knee's, < 0: same as the hip
  // Control loop period in plant ticks (2 ms each): the ESP32 loop overran
  // to ~3.6 ms per tick on 2026-10-02 (SENS ctrl_us, late).
  int ctrl_every = 1;
  double gyro_noise = 0.005;  // pitch gyro white noise per sample [rad/s]
  // Gear backlash (total free play, link side [rad]) between each GDS68
  // rotor and its link, 0 = rigid (the old path, bit-identical). With it
  // the rotor is its own body (its inertia leaves the link), the servo
  // and the encoder act on the ROTOR side, and torque reaches the link only
  // through the gear: stiffness gear_k [N m/rad per joint] with damping
  // ratio gear_zeta once the play is taken up.
  double hip_backlash = 0.0, knee_backlash = 0.0, gear_k = 150.0, gear_zeta = 0.3;
  double gear_k_knee = -1.0;  // knee gear stiffness, < 0: gear_k
  double wheel_noise = 0.3;   // wheel speed reading noise per sample [rad/s]
  // Scenario.
  double drive_m_s = 0.0, drive_start_s = 6.0, drive_end_s = 10.0;
  double push_n = 0.0, push_start_s = 6.0, push_end_s = 6.3;  // at the body CoM
  // Crouched pushes: hold the push (and the run's end) back until the
  // commanded height has been reached and held for 3 s - the height ramp
  // slows while the body rocks, so a fixed time can push half way down.
  bool push_wait_height = false;
  // Episodes (TIER_GENERAL): extra pushes [N] and drive segments [m/s] as
  // (t0, t1, value), and a height schedule (time, hip-axle height [m]).
  struct Pulse { double t0, t1, v; };
  std::vector<Pulse> pushes, drives;
  // Hand presses on the body: from t0 to t1 a vertical force v [N, + = up]
  // at body-frame x (+ = front) - the operator's "ấn đầu / ấn đuôi" test.
  struct Press { double t0, t1, x, v; };
  std::vector<Press> presses;
  std::vector<std::pair<double, double>> heights;
  double height_brake_s = -1.0;  // TIER_STOP: jerk-limited stop, controller state only
  int stop_mode = -1;           // -1 unchanged, 0 current, 1 wheel-only, 2 legacy

  double slope = 0.0;
  double height_low_m = 0.0, height_low_s = 6.0, height_up_s = 16.0;
  bool trace_position = false;
  std::string animation_file;
  double duration_s = 20.0;
  bool trace = false;
  bool ideal_imu = false;  // controller gets the true body pitch/rate (isolates the estimator)
  // Operator flow: engaged on the stand (body held at stand_pitch, wheels in
  // the air, legs dangling) for held_s, carried to the floor, let go.
  double held_s = 0.0, stand_pitch = -0.02;
  double trunk_on_s = -1.0, trunk_off_s = -1.0;  // runtime SET trunk 1 / 0
  // Grabbed mid-balance: body held still in the air from grab_s for
  // grab_len_s (wheels spin free, legs dangle), then put back down.
  double grab_s = -1.0, grab_len_s = 3.0;
  // Release-placement bias [rad]: the "put back down" pose after a grab is
  // offset from the true resting angle by this much, simulating the
  // operator placing the robot at an arbitrary pitch. Default 0.02 matches
  // the old behaviour (a small, incidental offset); a deliberately large
  // value tests the learned-trim fix.
  double grab_release_bias_rad = 0.02;
  double learn_hold = -1.0;  // < 0: firmware default (Config::learn_hold_s)
  double held_cap = -1.0;    // < 0: firmware default (Config::held_wheel_cap_m_s)
  // KNEETEST FLOOR: knee excitation amplitude [rad] from excite_start_s.
  double excite_amp = 0.0, excite_start_s = 10.0;
  bool no_rezero = false;    // diagnostics: held and runaway re-zeroes disabled
  const V3Controller::WbTable* wb_table = nullptr;  // Algo::WholeBody gains (null: generated)
  // Static level loop (Tier::level): -1 firmware default. The target is
  // the true resting body pitch at the hang pose (as the IMU reads it) plus
  // level_rel, so level_rel = 0 leaves the regression cases' posture as it
  // was; the body deviation metrics are taken from that target.
  int level = -1;
  double level_rel = 0.0;
  int envelope = -1;  // Tier::envelope (-1: firmware default)
  // Push / standing calm (< 0: firmware default): Config::stand_leash_m,
  // Tier::hip_deadband_rad, Tier::sat_margin.
  double stand_leash = -1.0, hip_db = -1.0, sat_margin = -1.0;
  int lead_gyro = -1;  // Tier::lead_gyro_only (-1: firmware default)
  int reflex_gate = -1;  // Tier::reflex_gate (-1: firmware default)
  // Roll (Config::Roll) on a lateral side model next to the planar plant:
  // left/right hip heights follow the commanded side offset through the
  // joint servo lag; the body rolls by their difference over the track plus
  // the ground's (cross slope, a bump under the left wheel only); a turn
  // adds the centripetal v * yaw to the accelerometer and the wheel loads.
  int roll = -1;                     // Config::Roll::enabled (-1: default)
  double lqr_lead = -1.0;            // Config::lqr_lead_s (-1: default)
  double height_exp_low = -1.0;      // Tier::height_exp (-1: default)
  int lqr_adapt = -1;                // Config::lqr_lead_adaptive (-1: default)
  int sus_adaptive = -1;             // Tier::sus_adaptive (-1: default)
  double cross_slope = 0.0;          // + = left wheel higher [rad]
  double yaw_rad_s = 0.0, yaw_t0 = 0.0, yaw_t1 = 0.0;  // commanded turn window
  bool bump_left_only = false;       // bump_h under the left wheel only
  double leg_side_err_m = 0.0;       // left hip-axle height minus right (hand-arranged)
  double roll_gyro_bias = 0.0;       // raw roll gyro bias [rad/s]
  // IMU roll reading minus the true body roll (its mount): the robot's
  // measured -0.040, which Config::Roll::target_rad carries.
  double imu_roll_mount = -0.040;
  int roll_vff = -1;                 // Config::Roll::vel_ff (-1: default)
  // Layered roll (docs/ROLL_REDESIGN_20261005.md); -1 / negative: struct default.
  int roll_learn = -1, roll_pre = -1, roll_latch = -1, roll_gimbal = -1;
  double roll_damp = -1.0;
  double slope_t0 = 0.0;             // > 0: the cross slope ramps in over 2 s from here
  double roll_noise_scale = 1.0;     // roll gyro / accelerometer noise
  double roll_servo_scale = 1.0;     // side-leg servo bandwidth (x knee_kp)
  int knee_hold = -1, knee_hold_wb = -1;  // Tier::knee_hold / knee_hold_wb
  double knee_follow_hz = -1.0, knee_follow_rate = -1.0;  // Tier::knee_follow_* (-1: default)
  double body_damping = -1.0;       // Tier::body_damping_s (-1: default)
  double body_damp_lpf = -1.0;      // Tier::body_damp_lpf_hz (-1: default)
  int boot_like = 0;                // 1: the AppV3 boot tier/LQR overrides (HANDOFF section 3)
};

struct Result {
  bool fell = false;
  const char* fault = "";
  int held_resets = 0;
  double body_dev_drive = 0.0;    // max |body pitch - rest| from drive start to end + 1 s
  double body_dev_all = 0.0;      // max |body pitch - rest| after 2 s
  double capture_body_peak = 0.0; // first 3 s after placement, no 2 s exclusion
  double capture_height_m = 0.0;  // measured leg height at placement
  double lean_max = 0.0;          // max |CoM line angle from vertical| after 2 s
  double max_dhip = 0.0, max_hip_rate = 0.0, max_hip_err = 0.0;
  double max_wheel_cmd = 0.0, max_speed = 0.0;
  double rate_rms_drive = 0.0;    // body pitch-rate rms while driving (pecking)
  double body_rms_end = 0.0;      // body pitch - rest, rms over the last 4 s
  double rest_rate_rms = 0.0;     // body pitch-rate rms over the last 4 s (standing calm)
  double rest_hip_rms = 0.0;      // hip rate rms over the last 4 s
  double x_end = 0.0, max_x = 0.0, x_stop_travel = 0.0;
  double max_tau_hip = 0.0, max_tau_knee = 0.0, max_knee_err = 0.0;
  double body_vacc_peak = 0.0;     // |vertical body CoM acceleration| peak after 2 s (20 Hz)
  double body_vacc_bump_peak = 0.0;  // filtered peak while the wheel crosses the test bump
  double body_vacc_rms = 0.0;      // its rms after 2 s
  double body_z_range = 0.0;       // body CoM height max - min after 2 s [m]
  double body_z_range_late = 0.0;  // ...after 11 s (crouch done in the sweeps that lower at 1 s)
  double held_max_dhip = 0.0, held_max_wheel = 0.0;  // while on the stand
  double est_err_max = 0.0, est_err_rms = 0.0;  // IMU pitch estimate - truth (after 2 s)
  double grab_detect_s = -1.0;     // grab start -> first held re-zero
  double grab_max_dhip_end = 0.0;  // |tier-2 hip| over the last second of the grab
  double grab_max_wheel = 0.0;     // wheel speed while grabbed [rad/s]
  double excite_lean = 0.0, excite_body = 0.0;  // peak change during the excitation
  double excite_knee_err = 0.0;                 // |knee cmd - knee| peak during it
  double putdown_wheel = 0.0;      // wheel speed (rel. to shin) at the grab's put-down [rad/s]
  double putdown_cmd = 0.0;        // wheel command at put-down
  double level_hip_end = 0.0;      // static level loop's hip offset at the end [rad]
  double start_dev = 0.0;          // body pitch - target at the start [rad]
  double push_body = 0.0, push_lean = 0.0, push_hip = 0.0;  // peak change in the 2 s after push start
  double push_knee = 0.0, push_dz = 0.0;  // knee and body CoM height, same window
  double push_height = 0.0;        // hip-to-axle height at the push [m]
  double push_opposite = 0.0;      // body tipped against the push, 0.3-6 s after its start [rad]
  double rest_trunk_pp = 0.0;      // tier-2 hip offset max - min over the last 15 s [rad]
  double lqr_lead_end = 0.0;       // wheel-loop lead in use at the end [s]
  double rest_hip_reversals_s = 0.0;  // hip command direction changes > 5 mrad per second, last 15 s
  double rest_x_pp = 0.0;          // wheel ground position max - min over the last 15 s [m]
  double push_sequence_body_peak = 0.0;  // body error during a scheduled pulse sequence [rad]
  double push_sequence_height_m = 0.0;   // plant hip-to-axle height at sequence start [m]
  double press_rebound_rad = 0.0;       // opposite body pitch between release and next press
  double body_rms_all = 0.0, lean_rms_all = 0.0;  // dynamic rms after 2 s (share = body / lean)
  double drive_dz = 0.0;           // body CoM height change from the drive start, drive + 1 s [m]
  double drive_vacc_rms = 0.0, drive_vacc_peak = 0.0;  // body vertical accel while driving (+1 s)
  double drive_pitch_rms = 0.0;    // body pitch off rest while driving (+1 s)
  double body_mean_end = 0.0;      // mean body pitch - target over the last 4 s (static tilt left)
  double roll_app_max = 0.0;       // max |roll vs the apparent vertical| after 2 s [rad]
  double roll_app_end = 0.0;       // its mean over the last 4 s
  double roll_world_max = 0.0;     // max |body roll vs gravity| after 2 s
  double wheel_load_min = 0.5;     // smaller wheel's share of the load (0 = lifts)
  double roll_dh_max = 0.0;        // max |side height offset| commanded [m]
  double knee_cmd_rate_rms = 0.0;  // IK knee-term (dknee) rate rms after 12 s [rad/s]
  double knee_cmd_rate_peak = 0.0; // its peak [rad/s]
  double knee_cmd_range = 0.0;     // dknee max - min after 12 s [rad]
  double axle_depth_err_max = 0.0; // max |hip-axle depth - nominal pose| from the commanded joints [m]
  double gyro_hf_rms = 0.0;        // controller gyro input above 5 Hz, rms after 2 s [rad/s]
  double dhip_hf_rms = 0.0;        // tier-2 hip command above 5 Hz, rms after 2 s [rad]
  double roll_dh_end = 0.0;        // mean |side height offset| over the last 4 s [m]
  double roll_phi_end = 0.0;       // mean body roll vs gravity over the last 4 s [rad]
  double roll_ref_end = 0.0;       // learned level reference at the end
  int roll_mode_end = 0;           // 0 off/learning, 1 active, 2 latched
  double roll_dh_rev_s = 0.0;      // side offset direction reversals per second (> 0.3 mm)
  double roll_t_first = -1.0, roll_t_last = -1.0;  // first / last time |roll vs apparent| > 0.005 (after 2 s)
  double roll_t_half = -1.0;       // first time back under half its peak after the peak
  double update_us_sum = 0.0, update_us_max = 0.0;  // host time in V3Controller::update
  long update_n = 0;
  double settle01_s = -1.0;        // last time |body - target| > 0.01 rad (after 2 s)
  bool learned_valid = false;      // Config::learn_hold_s trim learned by the end
  double level_settle_s = -1.0;    // last time |body - target| was above 0.015 rad (after 2 s)
  double post_release_dev = 0.0;   // max |body pitch - rest| in the 2 s after put-down
  double settled_dev = 0.0;        // |body pitch - rest| ~8 s after put-down (has it gone home?)
  // Body as a stabilised platform (drive/push event window, see simulate):
  // horizontal acceleration and jerk of the BODY CoM, and the speed bumps
  // of a balancing robot - the body speeding up before it brakes, or
  // backing off before it accelerates (non-minimum-phase catch).
  double body_acc_peak = 0.0, body_acc_rms = 0.0, body_jerk_rms = 0.0;
  double brake_bump = 0.0;   // max body speed gain after the drive release [m/s]
  double start_dip = 0.0;    // max body speed loss (against the command) after drive start
  double height_down_s = -1.0, height_up_s = -1.0;
  std::vector<double> x_trace;
};

using tap_fw::sim::LegPlant;
using tap_fw::sim::Pt;
using State = LegPlant::Vec;
double wb_bias_gain_override = -1.0;  // TR_WBB diagnostics

Result simulate(const Case& c) {
  const double dt_c = 0.002;
  const int sub = 8;
  const double dt = dt_c / sub;
  const double R = geo::wheel_radius_m;
  const bool backlash = c.hip_backlash > 0.0 || c.knee_backlash > 0.0;
  LegPlant plant(c.body_dx, c.mass_scale, backlash ? 0.0 : c.hip_rotor, backlash ? 0.0 : c.knee_rotor,
                 LegHeight::hangHipCad());
  std::function<double(double)> c_rough_surface;  // rough road: the floor (plant.ground is the axle path)
  auto bump_profile = [c](double x) {
    auto edge = [&](double x0) {
      const double a = std::clamp((x - x0) / std::max(c.bump_ramp, 1e-4), 0.0, 1.0);
      return 0.5 * (1.0 - std::cos(kPi * a));
    };
    double h = c.bump_h * edge(c.bump_x0);
    if (c.bump_width > 0.0) h -= c.bump_h * edge(c.bump_x0 + c.bump_ramp + c.bump_width);
    return h;
  };
  if (c.rough_seed > 0 && c.rough_dense) {
    struct Hump { double xc, w, h; };
    std::vector<Hump> humps;
    std::mt19937 rg(static_cast<unsigned>(c.rough_seed));
    auto U = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(rg); };
    const double tan15 = std::tan(15.0 * kPi / 180.0);
    for (double x = 0.3; x < 0.3 + c.rough_len_m; x += U(0.04, 0.10)) {
      const double h = U(-0.006, 0.008) * c.rough_scale;
      const double w = std::max(U(0.04, 0.12), std::abs(h) * kPi / tan15);  // raised cosine, edge <= 15 deg
      humps.push_back({x + 0.5 * w, w, h});
    }
    const double x0 = 0.3, x1 = 0.3 + c.rough_len_m;
    const double a1 = 0.008 * c.rough_scale, l1 = U(0.8, 1.5), p1 = U(0.0, 2.0 * kPi);
    const double a2 = 0.006 * c.rough_scale, l2 = U(0.4, 0.7), p2 = U(0.0, 2.0 * kPi);
    plant.ground = [humps, x0, x1, a1, l1, p1, a2, l2, p2](double x) {
      double h = 0.0;
      for (const auto& b : humps) {
        const double u = (x - b.xc) / b.w;
        if (std::abs(u) < 0.5) h += b.h * 0.5 * (1.0 + std::cos(2.0 * kPi * u));
      }
      // Long waves, faded in and out over 0.3 m at the ends of the stretch.
      const double fade = std::clamp(std::min(x - x0, x1 - x) / 0.3, 0.0, 1.0);
      const double ww = a1 * std::sin(2.0 * kPi * (x - x0) / l1 + p1) - a1 * std::sin(p1) +
                        a2 * std::sin(2.0 * kPi * (x - x0) / l2 + p2) - a2 * std::sin(p2);
      return h + fade * ww;
    };
  } else if (c.rough_seed > 0) {
    struct Feat { double x0, ramp, width, h; };
    std::vector<Feat> feats;
    std::mt19937 rg(static_cast<unsigned>(c.rough_seed));
    auto U = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(rg); };
    const double tan15 = std::tan(15.0 * kPi / 180.0);
    for (double x = 0.3; x < 0.3 + c.rough_len_m;) {
      const double kind = U(0.0, 1.0);
      Feat f{};
      if (kind < 0.45) {         // bump 3-8 mm
        f.h = U(0.003, 0.008) * c.rough_scale;
        f.width = U(0.02, 0.10);
      } else if (kind < 0.70) {  // dip 3-6 mm
        f.h = -U(0.003, 0.006) * c.rough_scale;
        f.width = U(0.03, 0.12);
      } else {                   // gentle ramp up or down 1-3 cm over 0.3-0.6 m, held a while
        f.h = (U(0.0, 1.0) < 0.5 ? 1.0 : -1.0) * U(0.010, 0.030) * c.rough_scale;
        f.width = U(0.20, 0.50);
      }
      const double min_ramp = std::abs(f.h) * kPi / (2.0 * tan15);
      f.ramp = kind < 0.70 ? std::max(U(0.03, 0.06), min_ramp) : std::max(U(0.30, 0.60), min_ramp);
      f.x0 = x;
      feats.push_back(f);
      x += 2.0 * f.ramp + f.width + U(0.10, 0.40);
    }
    plant.ground = [feats](double x) {
      double h = 0.0;
      for (const auto& f : feats) {
        auto edge = [&](double x0) {
          const double a = std::clamp((x - x0) / f.ramp, 0.0, 1.0);
          return 0.5 * (1.0 - std::cos(kPi * a));
        };
        h += f.h * (edge(f.x0) - edge(f.x0 + f.ramp + f.width));
      }
      return h;
    };
  }
  if (c.rough_seed > 0 && plant.ground) {
    // A rigid wheel of radius R rolls over the surface: its axle path is the
    // surface's envelope under a circle, axle(x) = max_u [h(x+u) + sqrt(R^2 -
    // u^2)] - R, not the surface under the axle - short steep features meet
    // the rim ahead of the axle and are far gentler on the axle than on the
    // floor (an unfiltered dense road had 28 deg walls the wheel "stalled"
    // on). Tabulated at 1 mm.
    const auto surf = plant.ground;
    const double xa = -0.5, xb = 0.3 + c.rough_len_m + 1.0, step = 0.001;
    const int n = static_cast<int>((xb - xa) / step) + 1;
    std::vector<double> hs(n), axle(n);
    for (int i = 0; i < n; ++i) hs[i] = surf(xa + i * step);
    const int ru = static_cast<int>(R / step);
    for (int i = 0; i < n; ++i) {
      double best = -1e9;
      for (int j = -ru; j <= ru; ++j) {
        const int k = std::clamp(i + j, 0, n - 1);
        const double u = j * step;
        best = std::max(best, hs[k] + std::sqrt(std::max(R * R - u * u, 0.0)) - R);
      }
      axle[i] = best;
    }
    // Smoothed over the rubber tyre's contact patch (three 9 mm box passes,
    // ~1 cm wide in all) - the rim-contact corners of a rigid wheel are
    // instant impacts the tyre spreads - and read back with C1 cubic
    // (Catmull-Rom) interpolation: the plant takes the path's curvature by
    // finite differences, and a piecewise-linear table turned every 1 mm
    // node into a spurious jolt.
    for (int pass = 0; pass < 3; ++pass) {
      std::vector<double> sm(n);
      for (int i = 0; i < n; ++i) {
        double acc = 0.0;
        for (int j = -4; j <= 4; ++j) acc += axle[std::clamp(i + j, 0, n - 1)];
        sm[i] = acc / 9.0;
      }
      axle.swap(sm);
    }
    c_rough_surface = surf;  // the floor itself, for the viewer
    plant.ground = [axle, xa, step, n](double x) {
      const double f = std::clamp((x - xa) / step, 1.0, double(n - 3) - 1e-9);
      const int i = static_cast<int>(f);
      const double t = f - i;
      const double p0 = axle[i - 1], p1 = axle[i], p2 = axle[i + 1], p3 = axle[i + 2];
      return p1 + 0.5 * t * (p2 - p0 + t * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3 + t * (3.0 * (p1 - p2) + p3 - p0)));
    };
  }
  if (c.rough_seed > 0) {
    // (rough road set above)
  } else if (c.bump_h != 0.0 && c.bump_left_only) {
    plant.ground = [bump_profile](double x) { return 0.5 * bump_profile(x); };  // the axle midpoint
  } else if (c.bump_h != 0.0) {
    plant.ground = [c](double x) {
      auto edge = [&](double x0) {
        const double a = std::clamp((x - x0) / std::max(c.bump_ramp, 1e-4), 0.0, 1.0);
        return 0.5 * (1.0 - std::cos(kPi * a));
      };
      double h = c.bump_h * edge(c.bump_x0);
      if (c.bump_width > 0.0) h -= c.bump_h * edge(c.bump_x0 + c.bump_ramp + c.bump_width);
      return h;
    };
  }
  std::mt19937 rng(7);
  std::normal_distribution<double> n_acc(0, 0.05 * c.acc_noise_scale), n_gyr(0, c.gyro_noise), n_w(0, c.wheel_noise),
      n_q(0, 0.0005), n_qd(0, 0.03);

  // Stand at the hang pose, CoM over the axle plus the seed error.
  State s{}, sd{};
  for (int i = 0; i < 40; ++i) s[1] -= plant.leanAngle(s);
  // Body pitch at balance with correctly arranged legs (the posture target).
  const double theta_rest0 = s[1];
  std::FILE* anim = nullptr;
  const char* animation_file = c.animation_file.empty() ? std::getenv("TR_ANIM")
                                                         : c.animation_file.c_str();
  if (const char* ap = animation_file) {
    if (plant.ground) {  // the floor profile for tools/tier_sim_view.py
      const std::string gp = std::string(ap) + ".ground";
      if (std::FILE* gf = std::fopen(gp.c_str(), "w")) {
        std::fprintf(gf, "x,h\n");
        const auto& floor_h = c_rough_surface ? c_rough_surface : plant.ground;
        for (double x = -1.0; x < 8.0; x += 0.002) std::fprintf(gf, "%.3f,%.5f\n", x, floor_h(x));
        std::fclose(gf);
      }
    }
    anim = std::fopen(ap, "w");
    if (anim) std::fprintf(anim, "t,hip_x,hip_z,knee_x,knee_z,axle_x,axle_z,com_x,com_z,v,body,"
                                 "b0x,b0z,b1x,b1z,b2x,b2z,b3x,b3z,lean,hip,knee,cmd_v,push,ground,press,press_x,"
                                 "trunk_hip,hip_authority,rate,tvr,gx,hA,hB,aB,preview,modelA,plan_h,plan_v,plan_a\n");
  }
  // The legs as actually arranged; released this far from their own balance.
  s[2] = c.hip_zero_err;
  s[3] = c.knee_zero_err;
  for (int i = 0; i < 40; ++i) s[1] -= plant.leanAngle(s) - c.seed_error;

  const ImuStateEstimator::Config base{};
  ImuStateEstimator est(base);
  ImuStateEstimator::Output imu{};
  std::array<float, 3> last_accel{0.0F, 0.0F, 9.80665F};
  float t_s = 0.0F;
  ImuStateEstimator::KinematicVelocity kin{};
  auto feed = [&](double ax, double az, double gy) {
    const std::array<float, 3> a{static_cast<float>(ax + n_acc(rng)), 0.0F,
                                 static_cast<float>(az + n_acc(rng))};
    const std::array<float, 3> w{0.0F, static_cast<float>(gy - 0.05 + n_gyr(rng)), 0.0F};
    float as = 0, ws = 0;
    for (int i = 0; i < 3; ++i) { as += a[i] * a[i]; ws += w[i] * w[i]; }
    const bool quiet = std::abs(std::sqrt(as) - 9.80665F) < 0.35F && std::sqrt(ws) < 0.12F;
    last_accel = a;
    t_s += static_cast<float>(dt_c);
    const auto out = est.update(a, w, static_cast<float>(dt_c), t_s, 0.0F, false, quiet, kin);
    if (out.valid) imu = out;
  };
  const double pitch_read0 = s[1] + c.imu_mount;
  for (int i = 0; i < 1500; ++i) feed(-kG * std::sin(pitch_read0), kG * std::cos(pitch_read0), 0.0);

  auto cfg = tap_fw::v3::defaultV3Config();
  cfg.algo = c.algo;
  cfg.engage.wb_armed_only = false;  // the harness engages with GO (its stand holds the body still)
  if (const char* v = std::getenv("TR_LQRTRIM")) cfg.lqr_trim_gain = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_HLS")) cfg.wb_held_leg_sign = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_WBSOFT")) cfg.wb_soft_from = static_cast<float>(std::atof(v));
  cfg.tier.virtual_pitch = c.vpitch;
  cfg.tier.trunk = c.trunk;
  // Host-only A/B playback: hold every startup, plant and disturbance
  // parameter fixed while comparing tier-2 hip compensation enabled/off.
  if (const char* v = std::getenv("TR_TRUNK")) cfg.tier.trunk = std::atoi(v) != 0;
  if (c.stop_mode == 1) cfg.tier.trunk = false;
  if (c.stop_mode == 2) {
    cfg.tier.height_exp = 1.5F; cfg.tier.height_exp_tall = 0.5F;
    cfg.tier.lead_exp = 0.0F; cfg.tier.level_tau_s = 1.5F;
    cfg.tier.knee_hold = false; cfg.tier.adaptive_schedule = false;
    cfg.tier.wheel_carrier_compensation = false; cfg.tier.body_damping_s = 0.0F;
  }

  if (c.lean_share >= 0) cfg.tier.lean_share = static_cast<float>(c.lean_share);
  if (c.tier_lead >= 0) cfg.tier.lead_s = static_cast<float>(c.tier_lead);
  if (c.boot_like != 0) {
    cfg.tier.height_exp = 0.15F;
    cfg.lqr_lead_adaptive = true; cfg.lqr_lead_tall_s = 0.05F; cfg.lqr_lead_low_s = 0.09F;
    cfg.trim_acc = true;
    cfg.tier.body_damping_s = 0.08F;
    cfg.tier.joint_rate_gain_low = 0.7F;
    cfg.tier.sus_adaptive = true;
    cfg.tier.knee_follow_hz = 1.0F; cfg.tier.knee_follow_rate_rad_s = 0.6F;
    cfg.height.rate_m_s = 0.015F; cfg.height.accel_m_s2 = 0.05F; cfg.height.jerk_m_s3 = 0.20F;
  }
  if (c.body_damping >= 0) cfg.tier.body_damping_s = static_cast<float>(c.body_damping);
  if (c.body_damp_lpf >= 0) cfg.tier.body_damp_lpf_hz = static_cast<float>(c.body_damp_lpf);
  if (const char* v = std::getenv("TR_TBDLPF")) cfg.tier.body_damp_lpf_hz = static_cast<float>(std::atof(v));
  if (c.tier_max >= 0) cfg.tier.max_rad = static_cast<float>(c.tier_max);
  if (c.tier_exp >= 0) cfg.tier.height_exp = cfg.tier.height_exp_tall = static_cast<float>(c.tier_exp);
  if (const char* v = std::getenv("TIER_EXP_TALL")) cfg.tier.height_exp_tall = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_HEXP")) cfg.tier.height_exp = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_TVG")) cfg.tier.joint_rate_gain = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_TVGLOW")) cfg.tier.joint_rate_gain_low = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_SPLIT")) cfg.split.enabled = std::atoi(v) != 0 && c.stop_mode <= 0;
  if (const char* v = std::getenv("TR_SPLIT_TAU")) cfg.split.posture_tau_s = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_SPLIT_RATE")) cfg.split.posture_rate_rad_s = static_cast<float>(std::atof(v));

  if (const char* v = std::getenv("TR_TRATE")) cfg.tier.rate_rad_s = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_TMAX")) cfg.tier.max_rad = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_SLEASH")) cfg.stand_leash_m = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_TRIMACC")) cfg.trim_acc = std::atoi(v) != 0;
  if (const char* v = std::getenv("TR_TRIMACCTAU")) cfg.trim_acc_tau_s = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_TRIMGAIN")) cfg.lqr_trim_gain = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_LQRADAPT")) cfg.lqr_lead_adaptive = std::atoi(v) != 0;
  if (const char* v = std::getenv("TR_LQRLEADTALL")) cfg.lqr_lead_tall_s = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_LQRLEADLOW")) cfg.lqr_lead_low_s = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_SUSAD")) cfg.tier.sus_adaptive = std::atoi(v) != 0;
  if (const char* v = std::getenv("TR_KFHZ")) cfg.tier.knee_follow_hz = float(std::atof(v));
  if (const char* v = std::getenv("TR_SUSK")) cfg.tier.sus_noise_k = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_SUSDELAY")) cfg.tier.sus_engage_delay_s = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_RGATE")) cfg.tier.reflex_gate = std::atoi(v) != 0;
  if (const char* v = std::getenv("TR_RGRLO")) cfg.tier.reflex_rate_lo = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_RGRHI")) cfg.tier.reflex_rate_hi = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_RGLLO")) cfg.tier.reflex_lean_lo = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_RGLHI")) cfg.tier.reflex_lean_hi = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_RGREL")) cfg.tier.reflex_release_s = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_LEADGYRO")) cfg.tier.lead_gyro_only = std::atoi(v) != 0;
  if (const char* v = std::getenv("TR_TDB")) cfg.tier.hip_deadband_rad = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_SLFROM")) cfg.stand_leash_from = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_SRET")) cfg.stand_return_m_s = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_SLREL")) cfg.stand_leash_release_s = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_THPF")) cfg.tier.hpf_s = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_TLEADLPF")) cfg.tier.lead_lpf_hz = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_SATFADE")) cfg.tier.sat_fade_s = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_WSATM")) cfg.tier.wheel_sat_margin = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_SATM")) cfg.tier.sat_margin = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_TLEAD")) cfg.tier.lead_s = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_DSHARE")) cfg.tier.disturbance_share = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_BODYDAMP")) cfg.tier.body_damping_s = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_CARRIER")) cfg.tier.wheel_carrier_compensation = std::atoi(v) != 0;
  if (const char* v = std::getenv("TR_CARRIER_GAIN")) cfg.tier.wheel_carrier_gain = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_START_RATE")) cfg.height.startup_rate_m_s = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_START_ACCEL")) cfg.height.startup_accel_m_s2 = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_START_DELAY")) cfg.height.startup_delay_s = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_HRATE")) cfg.height.rate_m_s = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_HACCEL")) cfg.height.accel_m_s2 = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_HJERK")) cfg.height.jerk_m_s3 = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_TLEADEXP")) cfg.tier.lead_exp = static_cast<float>(std::atof(v));
  if (c.imu_kin >= 0) cfg.tier.imu_kinematic = c.imu_kin != 0;
  if (c.preview >= 0) cfg.tier.preview = c.preview != 0;
  if (c.plan_accel > 0) cfg.tier.plan_accel_m_s2 = static_cast<float>(c.plan_accel);
  if (c.plan_jerk > 0) cfg.tier.plan_jerk_m_s3 = static_cast<float>(c.plan_jerk);
  if (c.preview_s >= 0) cfg.tier.preview_s = static_cast<float>(c.preview_s);
  if (c.lead_lpf >= 0) cfg.tier.lead_lpf_hz = static_cast<float>(c.lead_lpf);
  if (c.learn_hold >= 0) cfg.learn_hold_s = static_cast<float>(c.learn_hold);
  if (c.held_cap >= 0) cfg.held_wheel_cap_m_s = static_cast<float>(c.held_cap);
  if (c.no_rezero) { cfg.pid.held_dv_m_s = 1.0e9F; cfg.runaway_us = 1'000'000'000U; }
  if (c.level >= 0) cfg.tier.level = c.level != 0;
  if (c.stand_leash >= 0.0) cfg.stand_leash_m = static_cast<float>(c.stand_leash);
  if (c.hip_db >= 0.0) cfg.tier.hip_deadband_rad = static_cast<float>(c.hip_db);
  if (c.lead_gyro >= 0) cfg.tier.lead_gyro_only = c.lead_gyro != 0;
  if (c.reflex_gate >= 0) cfg.tier.reflex_gate = c.reflex_gate != 0;
  if (c.sat_margin >= 0.0) cfg.tier.sat_margin = static_cast<float>(c.sat_margin);
  if (const char* v = std::getenv("TR_LEVELTAU")) cfg.tier.level_tau_s = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_LEVELLFP")) cfg.tier.level_lpf_hz = static_cast<float>(std::atof(v));
  if (c.envelope >= 0) cfg.tier.envelope = c.envelope != 0;
  if (c.roll >= 0) cfg.roll.enabled = c.roll != 0;
  if (const char* v = std::getenv("TR_ROLL")) cfg.roll.enabled = std::atoi(v) != 0;
  if (const char* v = std::getenv("TR_ROLLTAU")) cfg.roll.tau_s = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_ROLLFF")) cfg.roll.turn_ff = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_ROLLRATE")) cfg.roll.rate_m_s = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_ROLLKP")) cfg.roll.kp = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_LQRLEAD")) cfg.lqr_lead_s = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_LQRLEFF")) cfg.lqr_inertia_length = std::atoi(v) != 0;
  if (const char* v = std::getenv("TR_LQRLMAX")) cfg.lqr.max_pendulum_length_m = static_cast<float>(std::atof(v));
  if (c.lqr_lead >= 0) cfg.lqr_lead_s = static_cast<float>(c.lqr_lead);
  if (c.height_exp_low >= 0) cfg.tier.height_exp = static_cast<float>(c.height_exp_low);
  if (c.lqr_adapt >= 0) cfg.lqr_lead_adaptive = c.lqr_adapt != 0;
  if (c.sus_adaptive >= 0) cfg.tier.sus_adaptive = c.sus_adaptive != 0;
  if (c.wheel_pred >= 0) cfg.wheel_pred.enabled = cfg.wheel_pred.with_wb = c.wheel_pred != 0;
  if (std::getenv("TR_WPRED")) cfg.wheel_pred.enabled = cfg.wheel_pred.with_wb = std::atoi(std::getenv("TR_WPRED")) != 0;
  if (std::getenv("TR_WPTAU")) cfg.wheel_pred.tau_s = static_cast<float>(std::atof(std::getenv("TR_WPTAU")));
  if (std::getenv("TR_WPDELAY")) cfg.wheel_pred.act_delay_s = static_cast<float>(std::atof(std::getenv("TR_WPDELAY")));
  if (std::getenv("TR_WPMEAS")) cfg.wheel_pred.meas_delay_s = static_cast<float>(std::atof(std::getenv("TR_WPMEAS")));
  if (c.roll_vff >= 0) cfg.roll.vel_ff = c.roll_vff != 0;
  if (c.roll_learn >= 0) cfg.roll.learn_ref = c.roll_learn != 0;
  if (c.roll_pre >= 0) cfg.roll.cmd_preview = c.roll_pre != 0;
  if (c.roll_latch >= 0) cfg.roll.latch = c.roll_latch != 0;
  if (c.roll_gimbal >= 0) cfg.roll.gimbal_link = c.roll_gimbal != 0;
  if (std::getenv("TR_ROLLLAYERED")) {  // the firmware's layered roll settings for any suite
    cfg.roll.learn_ref = cfg.roll.cmd_preview = cfg.roll.latch = cfg.roll.gimbal_link = true;
  }
  if (std::getenv("TR_ROLLLAYERED")) {  // the firmware's layered roll settings for any suite
    cfg.roll.learn_ref = cfg.roll.cmd_preview = cfg.roll.latch = cfg.roll.gimbal_link = true;
  }
  if (c.roll_damp >= 0.0) cfg.roll.damp_s = static_cast<float>(c.roll_damp);
  if (std::getenv("RS_VFF")) cfg.roll.vel_ff = true;
  if (const char* v = std::getenv("TR_ROLLEST")) cfg.roll.est_tau_s = static_cast<float>(std::atof(v));
  if (c.knee_hold >= 0) cfg.tier.knee_hold = c.knee_hold != 0;
  if (c.knee_hold_wb >= 0) cfg.tier.knee_hold_wb = c.knee_hold_wb != 0;
  if (c.knee_follow_hz >= 0.0) cfg.tier.knee_follow_hz = float(c.knee_follow_hz);
  if (c.knee_follow_rate >= 0.0) cfg.tier.knee_follow_rate_rad_s = float(c.knee_follow_rate);
  if (const char* v = std::getenv("TR_PLANFULL")) cfg.tier.plan_full = std::atoi(v) != 0;
  if (const char* v = std::getenv("TR_KHOLD")) cfg.tier.knee_hold = cfg.tier.knee_hold_wb = std::atoi(v) != 0;
  if (c.suspension >= 0) cfg.tier.suspension = c.suspension != 0;
  if (const char* v = std::getenv("TR_SUSPENSION")) cfg.tier.suspension = std::atoi(v) != 0;
  if (const char* v = std::getenv("TR_LEASH")) cfg.drive_leash_m = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_SUSG")) cfg.tier.sus_gain = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_SUSR")) cfg.tier.sus_return_s = static_cast<float>(std::atof(v));
  if (const char* v = std::getenv("TR_SUSDB")) cfg.tier.sus_deadband_m_s = static_cast<float>(std::atof(v));
  cfg.tier.level_pitch_rad = static_cast<float>(theta_rest0 + c.imu_mount + c.level_rel);
  const bool level_active = c.vpitch && c.trunk && cfg.tier.level;
  if (const char* v = std::getenv("TIER_PVMAX")) cfg.tier.plan_vmax_m_s = static_cast<float>(std::atof(v));
  if (c.joint_rate_gain >= 0) cfg.tier.joint_rate_gain = static_cast<float>(c.joint_rate_gain);
  if (wb_bias_gain_override >= 0) cfg.wb_bias_gain = static_cast<float>(wb_bias_gain_override);
  if (std::getenv("TR_WBFF")) cfg.wb_lean_ff = std::atoi(std::getenv("TR_WBFF")) != 0;
  if (std::getenv("TR_WBSLACK")) cfg.wb_drive_slack_m = static_cast<float>(std::atof(std::getenv("TR_WBSLACK")));
  // Optional offline comparison of actual boot balance settings. No UART/Wi-Fi model.
  if (const char* profile = std::getenv("TIER_ADDWIFI_PROFILE")) {
    cfg.tier.height_exp = .15F;
    cfg.lqr_lead_adaptive = true;
    cfg.lqr_lead_tall_s = cfg.lqr_lead_low_s = .09F;
    cfg.trim_acc = true;
    cfg.tier.body_damping_s = .08F; cfg.tier.body_damp_lpf_hz = 4.5F;
    cfg.tier.joint_rate_gain_low = .7F;
    cfg.tier.knee_follow_hz = 1; cfg.tier.knee_follow_rate_rad_s = .6F;
    cfg.roll.learn_ref = cfg.roll.cmd_preview = cfg.roll.latch = cfg.roll.gimbal_link = true;
    tap_fw::v3::applyAddWifiBalanceProfile(cfg, std::atoi(profile) != 0);
  }
  V3Controller ctrl(cfg);
  ctrl.setWholeBodyTable(c.wb_table);
  V3Controller::Input in{};
  V3Controller::Command cmd{};
  V3Controller::Output out_hold{};
  in.legs_ready = true;
  in.legs_measured = true;
  in.pitch_rad = imu.pitch_rad;
  cmd.start = true;
  (void)ctrl.update(in, cmd);
  cmd.start = false;
  in.now_us = 1'300'000U;
  (void)ctrl.update(in, cmd);
  assert(ctrl.state() == V3Controller::State::Ready);
  cmd.go = true;
  {
    auto bal = tap_fw::v3::balancingEstimatorConfig(base);
    // Estimator sweeps (docs/archive/V3.3_HANDOFF_HISTORY_20261005.md 5.34): accelerometer trust while balancing.
    if (const char* v = std::getenv("TR_IMUACC")) bal.accel_correction_gain_hz = static_cast<float>(std::atof(v));
    if (const char* v = std::getenv("TR_IMUKT")) bal.kinematic_accel_trust_scale_m_s2 = static_cast<float>(std::atof(v));
    est.setConfig(bal);
  }

  Result r;
  if (c.trace_position) r.x_trace.reserve(static_cast<std::size_t>(c.duration_s / dt_c));
  r.capture_height_m = -plant.points(s[2], s[3]).axle.z;
  std::uint64_t t0_us = 1'302'000U;
  double held_wheel_pos = 0.0;
  if (c.held_s > 0.0) {
    // Stand: the body is held, the wheels spin in the air (lag only), the
    // legs dangle (the fitted stand servo). Carried to the floor over the
    // last 0.5 s (the gyro sees it).
    double w = 0.0, wp = 0.0, q = 0.0, qd = 0.0;
    const double floor_pitch = s[1] + c.imu_mount, stand = c.stand_pitch + c.imu_mount;
    const int n = static_cast<int>(c.held_s / dt_c);
    for (int k = 0; k < n; ++k, t0_us += 2000U) {
      const double tk = k * dt_c, ramp = 0.5;
      const double a = std::clamp((tk - (c.held_s - ramp)) / ramp, 0.0, 1.0);
      const double pitch = stand + a * (floor_pitch - stand);
      const double rate = (a > 0.0 && a < 1.0) ? (floor_pitch - stand) / ramp : 0.0;
      feed(-kG * std::sin(pitch), kG * std::cos(pitch), rate);
      in.now_us = t0_us;
      in.pitch_rad = imu.pitch_rad;
      in.pitch_rate_rad_s = imu.pitch_rate_rad_s;
      in.accel_x_m_s2 = last_accel[0];
      in.accel_z_m_s2 = last_accel[2];
      in.wheel_rad_s = {static_cast<float>(w), static_cast<float>(w)};
      in.wheel_pos_rad = {static_cast<float>(wp), static_cast<float>(wp)};
      in.hip_rad = {static_cast<float>(q), static_cast<float>(q)};
      in.knee_rad = {0.0F, 0.0F};
      in.hip_rate_rad_s = {static_cast<float>(qd), static_cast<float>(qd)};
      in.leg_chain_rad = static_cast<float>(q);
      const auto out = ctrl.update(in, cmd);
      cmd.go = false;
      if (out.state == V3Controller::State::Fault) {
        r.fell = true;
        r.fault = ctrl.lastFault();
        if (anim) std::fclose(anim);
        return r;
      }
      r.held_max_dhip = std::max(r.held_max_dhip, std::abs(double(ctrl.trunkHip())));
      r.held_max_wheel = std::max(r.held_max_wheel, std::abs(w));
      w += (0.5 * (out.wheel_cmd_rad_s[0] + out.wheel_cmd_rad_s[1]) - w) * dt_c / c.wheel_tau;
      wp += w * dt_c;
      for (int m = 0; m < 4; ++m) {
        const double acc = c.hip_vel_bw * (c.hip_kp * (ctrl.hipCommand() - q) - qd);
        qd += acc * dt_c / 4;
        q += qd * dt_c / 4;
      }
    }
    s[2] = q + c.hip_zero_err;
    s[3] = ctrl.legPose().knee_offset_rad + c.knee_zero_err;
    held_wheel_pos = wp - s[2] - s[3];  // encoder = x / R - theta - q - k (x = 0 here)
    for (int i = 0; i < 40; ++i) s[1] -= plant.leanAngle(s) - c.seed_error;
  }
  std::deque<double> wheel_q, hip_q, knee_q;
  std::deque<std::pair<double, double>> wm_q;
  double wheel_applied = 0.0, hip_applied = 0.0, knee_applied = s[3] - c.knee_zero_err;
  // Servo targets and readings are in the encoder frame; the plant is not.
  const double hz = c.hip_zero_err, kz = c.knee_zero_err;
  double u = held_wheel_pos, ud = 0.0;  // wheel-to-shin angle and rate
  double ws_acc = 0.0;              // resonant servo acceleration state
  // The GDS68s are already holding the robot up when the balance loop
  // starts: their integrators carry the static joint torques.
  const auto hold = plant.staticJointTorques(s);
  double hip_integ = std::clamp(hold[0], -2.0, 2.0), knee_integ = hold[1];
  double q_meas = s[2] - c.hip_zero_err, qd_meas = 0.0, k_meas = s[3] - c.knee_zero_err,
         kd_meas = 0.0, next_leg_fb = 0.0;
  // Rotor-side angle/rate of the hip and knee pairs (Case::hip_backlash).
  // Start loaded: the play already taken up in the direction of the static
  // joint torque.
  double mh = s[2], mhd = 0.0, mk = s[3], mkd = 0.0;
  auto gear = [&](double m, double q, double md, double qd, double play, double rotor, double kj) {
    // Torque the pair of gears passes to the links (two joints).
    const double h = 0.5 * play, e = m - q;
    const double k = 2.0 * kj;
    const double cg = 2.0 * c.gear_zeta * std::sqrt(k * 2.0 * std::max(rotor, 1.0e-4));
    if (e > h) return k * (e - h) + cg * (md - qd);
    if (e < -h) return k * (e + h) + cg * (md - qd);
    return 0.0;
  };
  if (backlash) {
    const double kp = 2.0 * c.gear_k;
    if (hold[0] != 0.0) mh += std::copysign(0.5 * c.hip_backlash + std::abs(hold[0]) / kp, hold[0]);
    const double kpk = 2.0 * (c.gear_k_knee > 0.0 ? c.gear_k_knee : c.gear_k);
    if (hold[1] != 0.0) mk += std::copysign(0.5 * c.knee_backlash + std::abs(hold[1]) / kpk, hold[1]);
  }
  double hip_zoh = 0.0, knee_zoh = s[3] - c.knee_zero_err, next_hip_cmd = 0.0;
  double knee_cmd_prev = 0.0, knee_rate_sq = 0.0, knee_cmd_lo = 1.0e9, knee_cmd_hi = -1.0e9;
  double hf_g_lp = 0.0, hf_d_lp = 0.0, hf_g_sq = 0.0, hf_d_sq = 0.0;
  long hf_n = 0;
  bool knee_cmd_prev_valid = false;
  long knee_rate_n = 0;
  double s_acc[LegPlant::kDof] = {};
  double dv_sq = 0.0, dp_sq = 0.0;
  int dv_n = 0;
  double abz_f = 0.0, vacc_sq = 0.0, zb_min = 1e9, zb_max = -1e9, zl_min = 1e9, zl_max = -1e9;
  int vacc_n = 0;
  const double kv_joint = c.hip_vel_bw * (geo::leg_hip_inertia_kgm2 + c.hip_rotor);
  const double kv_knee = c.knee_vel_bw > 0.0
      ? c.knee_vel_bw * (geo::leg_knee_inertia_kgm2 + c.knee_rotor) : kv_joint;
  double rate_sq = 0.0, body_sq = 0.0, x_stop = 0.0, rest_rate_sq = 0.0, rest_hip_sq = 0.0;
  double est_sq = 0.0, est_bias0 = 0.0;
  double ab_f = 0.0, ev_acc_sq = 0.0, ev_jerk_sq = 0.0, v_release = 0.0, v_start = 0.0;
  int ev_n = 0;
  int est_n = 0, held_before_grab = 0;
  bool put_down = false, grab_biased = false;
  double grab_from = 0.0, grab_to = 0.0;
  double push_b0 = 0.0, push_l0 = 0.0, push_q0 = 0.0, push_k0 = 0.0, push_z0 = 0.0;
  double excite_center = 0.0, excite_b0 = 0.0, excite_l0 = 0.0;
  double put_down_t = 0.0;
  int rate_n = 0, body_n = 0;
  double push_start_s = c.push_start_s, push_end_s = c.push_end_s, duration_s = c.duration_s;
  double all_body_sq = 0.0, all_lean_sq = 0.0, all_body_lp = 0.0, all_lean_lp = 0.0, drive_z0 = 0.0;
  std::deque<double> roll_cmd_q;
  std::mt19937 roll_rng(11);
  std::normal_distribution<double> roll_n_gyr(0, 0.005 * c.roll_noise_scale),
      roll_n_acc(0, 0.05 * c.acc_noise_scale * c.roll_noise_scale);
  double roll_dh_act = 0.0, roll_phi_prev = 0.0, roll_app_sum = 0.0, yaw_act = 0.0, roll_t_peak = 0.0;
  double roll_dh_abs_sum = 0.0, roll_phi_sum = 0.0, roll_dh_ext = 0.0, roll_dh_dir = 0.0;
  long roll_dh_n = 0, roll_dh_revs = 0;
  double roll_cmd_prev = 0.0;
  long roll_app_n = 0;
  double body_mean_sum = 0.0;
  double rest_trunk_lo = 1e9, rest_trunk_hi = -1e9, rest_x_lo = 1e9, rest_x_hi = -1e9;
  double rev_ext = 0.0, rev_dir = 0.0;
  int rev_n = 0;
  bool rev_init = false;
  long all_n = 0;
  double height_held_s = 0.0;
  bool push_sequence_measured = false;
  float height_brake_target = 0.0F;
  for (int k = 0; k < static_cast<int>(duration_s / dt_c); ++k) {
    const double t = k * dt_c;
    if (c.push_wait_height && c.height_low_m > 0.0 && t < push_start_s && t + dt_c >= push_start_s) {
      const bool there = std::abs(double(ctrl.legPose().axle_below_hip_m) - c.height_low_m) < 1.0e-3;
      height_held_s = there ? height_held_s + dt_c : 0.0;
      if (height_held_s < 3.0 && t < 120.0) {
        push_start_s += dt_c; push_end_s += dt_c; duration_s += dt_c;
      }
    }
    double fwd = (t >= c.drive_start_s && t < c.drive_end_s) ? c.drive_m_s : 0.0;
    for (const auto& d : c.drives) if (t >= d.t0 && t < d.t1) fwd += d.v;
    cmd.forward_m_s = static_cast<float>(fwd);
    for (const auto& hs : c.heights) if (t >= hs.first) cmd.height_m = static_cast<float>(hs.second);
    if (c.height_brake_s >= 0.0 && t >= c.height_brake_s) {
      if (height_brake_target == 0.0F) height_brake_target = ctrl.heightStopTarget();
      cmd.height_m = height_brake_target;
    }

    if (c.trunk_on_s >= 0.0 && std::abs(t - c.trunk_on_s) < 1e-9) ctrl.config().tier.trunk = true;
    if (c.trunk_off_s >= 0.0 && std::abs(t - c.trunk_off_s) < 1e-9) ctrl.config().tier.trunk = false;
    if (c.height_low_m > 0.0) {
      cmd.height_m = static_cast<float>(t < c.height_low_s ? 0.0
          : t < c.height_up_s ? c.height_low_m : LegHeight::maxHeight());
    }
    // Leg feedback (0x009) at ~96 Hz.
    if (t >= next_leg_fb) {
      next_leg_fb = t + c.leg_fb_period_s;
      q_meas = (backlash ? mh : s[2]) - hz + n_q(rng);
      qd_meas = (backlash ? mhd : sd[2]) + n_qd(rng) * (c.hip_vel_noise / 0.03);
      k_meas = (backlash ? mk : s[3]) - kz + n_q(rng);
      kd_meas = (backlash ? mkd : sd[3]) + n_qd(rng) * ((c.knee_vel_noise >= 0.0 ? c.knee_vel_noise : c.hip_vel_noise) / 0.03);
    }
    if (c.excite_amp > 0.0 && t >= c.excite_start_s) {
      // Same centring as AppV3: keep the whole swing inside the knee range.
      if (t - c.excite_start_s < 1e-9) {
        const double k = ctrl.legPose().knee_offset_rad;
        excite_center = k + c.excite_amp > 0.0 ? -c.excite_amp
            : k - c.excite_amp < LegHeight::kKneeMinRad ? c.excite_amp : 0.0;
      }
      const double te = t - c.excite_start_s;
      const double dz = tap_fw::v3::floorTestOffset(static_cast<float>(te),
                                                    static_cast<float>(c.excite_amp),
                                                    static_cast<float>(excite_center));
      ctrl.setKneeExcitation(static_cast<float>(dz));
    }
    in.now_us = t0_us + static_cast<std::uint64_t>(k) * 2000U;
    in.pitch_rad = c.ideal_imu ? static_cast<float>(s[1] + c.imu_mount) : imu.pitch_rad;
    in.pitch_rate_rad_s = c.ideal_imu ? static_cast<float>(sd[1]) : imu.pitch_rate_rad_s;
    in.accel_x_m_s2 = last_accel[0];
    in.accel_z_m_s2 = last_accel[2];
    // Wheel feedback latency (driver speed estimate + UART read):
    // c.wheel_meas_delay_ticks behind the true wheel.
    wm_q.push_back({ud, u});
    std::pair<double, double> wfb = wm_q.front();
    if (wm_q.size() > c.wheel_meas_delay_ticks) wm_q.pop_front();
    const double wm = wfb.first + n_w(rng);
    in.wheel_rad_s = {static_cast<float>(wm), static_cast<float>(wm)};
    in.wheel_pos_rad = {static_cast<float>(wfb.second), static_cast<float>(wfb.second)};
    in.hip_rad = {static_cast<float>(q_meas + 0.004), static_cast<float>(q_meas - 0.004)};
    in.knee_rad = {static_cast<float>(k_meas), static_cast<float>(k_meas)};
    in.hip_rate_rad_s = {static_cast<float>(qd_meas), static_cast<float>(qd_meas)};
    in.knee_rate_rad_s = {static_cast<float>(kd_meas), static_cast<float>(kd_meas)};
    in.leg_chain_rad = static_cast<float>(q_meas + k_meas);
    // Lateral side model (see Case::roll).
    cmd.yaw_rad_s = static_cast<float>((t >= c.yaw_t0 && t < c.yaw_t1) ? c.yaw_rad_s : 0.0);
    // The robot turns as the wheels follow the (shaped) differential command.
    yaw_act += dt_c / c.wheel_tau * (double(ctrl.yawCommand()) - yaw_act);
    const double yaw = yaw_act;
    in.yaw_rate_rad_s = static_cast<float>(yaw);
    {
      const double half = 0.5 * 0.309;
      roll_cmd_q.push_back(double(ctrl.rollSideHeight()));
      double dh_cmd = roll_cmd_q.front();
      if (roll_cmd_q.size() > c.hip_delay_ticks + 1U) roll_cmd_q.pop_front();
      // Optional servo velocity feed-forward (Set_Input_Pos vel_ff): the
      // driver adds the commanded rate, so a ramp is followed without the
      // position loop's lag.
      const double vff = ctrl.config().roll.vel_ff ? (dh_cmd - roll_cmd_prev) / dt_c : 0.0;
      roll_cmd_prev = dh_cmd;
      roll_dh_act += dt_c * (c.knee_kp * c.roll_servo_scale * (dh_cmd - roll_dh_act) + vff);
      const double slope = c.slope_t0 > 0.0
          ? c.cross_slope * std::clamp((t - c.slope_t0) / 2.0, 0.0, 1.0) : c.cross_slope;
      const double zl = half * std::tan(slope) + (c.bump_left_only ? bump_profile(s[0]) : 0.0);
      const double zr = -half * std::tan(slope);
      const double phi = std::atan(((zl + roll_dh_act + 0.5 * c.leg_side_err_m) -
                                    (zr - roll_dh_act - 0.5 * c.leg_side_err_m)) / (2.0 * half));
      const double phi_rate = k > 0 ? (phi - roll_phi_prev) / dt_c : 0.0;
      roll_phi_prev = phi;
      const double a_lat = sd[0] * yaw;
      const double fy = a_lat * std::cos(phi) + kG * std::sin(phi);
      const double fz = -a_lat * std::sin(phi) + kG * std::cos(phi);
      // What the (mounted) IMU reads: rotated by the mount offset.
      const double mphi = c.imu_roll_mount;
      const double fy_imu = fy * std::cos(mphi) + fz * std::sin(mphi);
      // Own noise stream: the planar runs keep their random sequence.
      in.roll_rate_rad_s = static_cast<float>(phi_rate + c.roll_gyro_bias + roll_n_gyr(roll_rng));
      in.accel_y_m_s2 = static_cast<float>(fy_imu + roll_n_acc(roll_rng));
      if (t > 2.0) {
        const double app = std::atan2(fy, fz);
        if (std::abs(app) > 0.005) { if (r.roll_t_first < 0) r.roll_t_first = t; r.roll_t_last = t; }
        if (std::abs(app) > r.roll_app_max) { r.roll_app_max = std::abs(app); roll_t_peak = t; r.roll_t_half = -1.0; }
        else if (r.roll_t_half < 0 && std::abs(app) < 0.5 * r.roll_app_max) r.roll_t_half = t;
        r.roll_world_max = std::max(r.roll_world_max, std::abs(phi));
        const double H = ctrl.legPose().com_above_axle_m + R;
        const double yc = -H * std::sin(phi);
        const double lam_l = 0.5 + (yc - H * a_lat / kG) / (2.0 * half);
        r.wheel_load_min = std::min({r.wheel_load_min, lam_l, 1.0 - lam_l});
        r.roll_dh_max = std::max(r.roll_dh_max, std::abs(double(ctrl.rollSideHeight())));
        if (t > duration_s - 4.0) {
          roll_app_sum += app; ++roll_app_n;
          roll_dh_abs_sum += std::abs(double(ctrl.rollSideHeight())); roll_phi_sum += phi; ++roll_dh_n;
        }
        {  // direction reversals of the commanded side offset (hunting)
          const double dhc = double(ctrl.rollSideHeight());
          if (roll_dh_dir == 0.0) { roll_dh_dir = 1.0; roll_dh_ext = dhc; }
          else if (roll_dh_dir > 0.0) {
            if (dhc > roll_dh_ext) roll_dh_ext = dhc;
            else if (roll_dh_ext - dhc > 3.0e-4) { roll_dh_dir = -1.0; roll_dh_ext = dhc; ++roll_dh_revs; }
          } else {
            if (dhc < roll_dh_ext) roll_dh_ext = dhc;
            else if (dhc - roll_dh_ext > 3.0e-4) { roll_dh_dir = 1.0; roll_dh_ext = dhc; ++roll_dh_revs; }
          }
        }
        if (std::getenv("GV_TRACE") && k % 50 == 0) {
          std::printf("GV t=%.2f plant_v=%+.4f ctrl_v=%+.4f wheel_meas=%+.3f R*(w+th'+q'+k')=%+.4f\n", t, sd[0],
                      double(ctrl.groundVelocity()), wm, R * (wm + sd[1] + sd[2] + sd[3]));
        }
        if (std::getenv("RS_TRACE") && k % 25 == 0) {
          std::printf("RS t=%.2f v=%.3f yaw=%.3f app=%+.4f phi=%+.4f est=%+.4f dh=%+.1f\n", t, sd[0], yaw, app, phi,
                      double(ctrl.rollEstimate()), double(ctrl.rollSideHeight()) * 1000);
        }
      }
    }
    const auto prof_t0 = std::chrono::steady_clock::now();
    if (k % std::max(c.ctrl_every, 1) == 0) {
      in.dt_s = static_cast<float>(dt_c * std::max(c.ctrl_every, 1));
      out_hold = ctrl.update(in, cmd);
    }
    const auto& out = out_hold;
    {
      const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - prof_t0).count();
      r.update_us_sum += us; ++r.update_n;
      r.update_us_max = std::max(r.update_us_max, us);
    }
    {
      const auto v = ctrl.imuVelocity();
      kin = {v.horizontal_m_s, v.vertical_m_s, v.valid};
    }
    if (std::getenv("RW_TRACE") && k % 50 == 0) {
      std::printf("RW t=%.2f plant_v=%+.3f wheel_v=%+.3f imu_v=%+.3f\n", t, sd[0],
                  double(ctrl.groundVelocity()), double(ctrl.runawayImuSpeed()));
    }
    cmd.go = false;
    if (t < 3.0) {
      r.capture_body_peak = std::max(r.capture_body_peak, std::abs(s[1] - theta_rest0));
    }
    if (out.state == V3Controller::State::Fault) {
      r.fell = true;
      r.fault = ctrl.lastFault();
      break;
    }
    r.max_wheel_cmd = std::max({r.max_wheel_cmd, std::abs(double(out.wheel_cmd_rad_s[0])),
                                std::abs(double(out.wheel_cmd_rad_s[1]))});
    wheel_q.push_back(0.5 * (out.wheel_cmd_rad_s[0] + out.wheel_cmd_rad_s[1]));
    if (wheel_q.size() > c.wheel_delay_ticks) { wheel_applied = wheel_q.front(); wheel_q.pop_front(); }
    // Paired targets use the same deadline pacing as the firmware, then
    // CAN + driver delay. Do not turn 3 ms into 4 ms by using t+period.
    if (t >= next_hip_cmd) {
      next_hip_cmd += tap_fw::v3::LegLock::Config{}.resend_period_us * 1.0e-6;
      hip_zoh = ctrl.hipCommand();
      knee_zoh = ctrl.kneeCommand();
    }
    hip_q.push_back(hip_zoh);
    if (hip_q.size() > c.hip_delay_ticks) { hip_applied = hip_q.front(); hip_q.pop_front(); }
    knee_q.push_back(knee_zoh);
    if (knee_q.size() > c.hip_delay_ticks) { knee_applied = knee_q.front(); knee_q.pop_front(); }
    r.max_dhip = std::max(r.max_dhip, std::abs(double(ctrl.trunkHip())));
    if (t > 2.0) {
      // One-pole 5 Hz high-pass of what the hip loop is fed (gyro) and what it commands.
      const double a_hf = 2.0 * 3.14159265358979 * 5.0 * dt_c;
      const double g_in = in.pitch_rate_rad_s, d_in = ctrl.trunkHip();
      if (hf_n == 0) { hf_g_lp = g_in; hf_d_lp = d_in; }
      hf_g_lp += a_hf * (g_in - hf_g_lp); hf_d_lp += a_hf * (d_in - hf_d_lp);
      hf_g_sq += (g_in - hf_g_lp) * (g_in - hf_g_lp);
      hf_d_sq += (d_in - hf_d_lp) * (d_in - hf_d_lp);
      ++hf_n;
    }
    if (t > 12.0) {  // after every test's height profile and settling
      const double kc = ctrl.kneeCommand();
      // The IK coupling term only (what the hip swing adds), not height/suspension.
      const double kd = kc - ctrl.legPose().knee_offset_rad;
      if (knee_cmd_prev_valid) {
        const double kr = (kd - knee_cmd_prev) / dt_c;
        knee_rate_sq += kr * kr; ++knee_rate_n;
        r.knee_cmd_rate_peak = std::max(r.knee_cmd_rate_peak, std::abs(kr));
      }
      knee_cmd_prev = kd; knee_cmd_prev_valid = true;
      knee_cmd_lo = std::min(knee_cmd_lo, kd); knee_cmd_hi = std::max(knee_cmd_hi, kd);
      // Axle depth the COMMANDED joints give (body pitch from the plant) against
      // the nominal pose's depth: what the knee low-pass costs in height.
      const auto leg = robot_control::WholeBodyModel::legHeight(
          LegHeight::hangHipCad() + ctrl.hipCommand(), float(kc));
      const double depth = -(-std::sin(s[1]) * leg.x_m + std::cos(s[1]) * leg.height_m);
      r.axle_depth_err_max = std::max(r.axle_depth_err_max,
          std::abs(depth - ctrl.legPose().axle_below_hip_m * std::cos(s[1])));
    }

    double fx = (t >= push_start_s && t < push_end_s) ? c.push_n : 0.0;
    for (const auto& pp : c.pushes) if (t >= pp.t0 && t < pp.t1) fx += pp.v;
    plant.ext = {};
    for (const auto& pr : c.presses) {
      if (t >= pr.t0 && t < pr.t1) { plant.ext.local = {pr.x, 0.065}; plant.ext.fz += pr.v; }
    }
    const bool grabbed = c.grab_s >= 0.0 && t >= c.grab_s && t < c.grab_s + c.grab_len_s;
    if (grabbed) {
      // Held in the air at an arbitrary pitch (operator's hand), body still,
      // wheels free (servo lag only), legs dangle. The hold angle is set
      // ONCE at grab onset and held fixed for the whole grab - as it would
      // be in a hand, not corrected tick by tick - so whatever the
      // controller records as its seed while held genuinely reflects that
      // (possibly wrong) angle, not the angle before the grab.
      // Tilted there by hand over 0.3 s, which the gyro sees: a one-tick
      // jump with a zero gyro rate put 0.08 rad of error into the
      // gyro-dominant estimate (and a wrong gyro bias after it) that no
      // real grab has.
      if (!grab_biased) {
        grab_biased = true;
        grab_from = s[1];
        auto target = s;
        for (int i = 0; i < 40; ++i) target[1] -= plant.leanAngle(target) - c.grab_release_bias_rad;
        grab_to = target[1];
      }
      const double tilt_s = 0.3;
      const double tilt_a = std::min((t - c.grab_s) / tilt_s, 1.0);
      s[1] = grab_from + tilt_a * (grab_to - grab_from);
      const double tilt_rate = tilt_a < 1.0 ? (grab_to - grab_from) / tilt_s : 0.0;
      if (r.grab_detect_s < 0.0 && ctrl.heldResets() > held_before_grab) {
        r.grab_detect_s = t - c.grab_s;
      }
      if (t > c.grab_s + c.grab_len_s - 1.0) {
        r.grab_max_dhip_end = std::max(r.grab_max_dhip_end, std::abs(double(ctrl.trunkHip())));
      }
      sd = {0.0, 0.0, sd[2], sd[3]};
      ud += (wheel_applied - ud) * dt_c / c.wheel_tau;
      u += ud * dt_c;
      r.grab_max_wheel = std::max(r.grab_max_wheel, std::abs(ud));
      if (c.trace && k % 10 == 0) {
        std::printf("GRB t=%.2f ud=%+.2f wcmd=%+.2f held=%d outer_off=%d quiet=%d ref=%+.4f tv=%+.4f\n",
                    t, ud, 0.5 * (out.wheel_cmd_rad_s[0] + out.wheel_cmd_rad_s[1]), ctrl.heldResets(),
                    ctrl.heldOuterOff(), ctrl.heldQuiet(), double(ctrl.balanceRef()),
                    double(ctrl.virtualPitch()));
      }
      for (int n = 0; n < sub; ++n) {
        const double acc = c.hip_vel_bw * (c.hip_kp * (hip_applied + hz - s[2]) - sd[2]);
        sd[2] += acc * dt;
        s[2] += sd[2] * dt;
        const double acck = kv_knee / (geo::leg_knee_inertia_kgm2 + c.knee_rotor) *
            (c.knee_kp * (knee_applied + kz - s[3]) - sd[3]);
        sd[3] += acck * dt;
        s[3] += sd[3] * dt;
      }
      mh = s[2]; mhd = sd[2]; mk = s[3]; mkd = sd[3];
      for (double& a : s_acc) a = 0.0;
      const double th = s[1] + c.imu_mount;
      feed(-kG * std::sin(th), kG * std::cos(th), tilt_rate);
      continue;
    }
    if (c.grab_s >= 0.0 && t < c.grab_s) held_before_grab = ctrl.heldResets();
    if (c.grab_s >= 0.0 && !put_down && t >= c.grab_s + c.grab_len_s) {
      // Let go, physically, at whatever angle it was being held (set at
      // grab onset above) - no further correction here.
      put_down = true;
      put_down_t = t;
      r.putdown_wheel = ud;
      r.putdown_cmd = 0.5 * (out.wheel_cmd_rad_s[0] + out.wheel_cmd_rad_s[1]);
      sd = {0.0, 0.0, 0.0, 0.0};
      mh = s[2]; mhd = 0.0; mk = s[3]; mkd = 0.0;
    }
    if (put_down && t >= put_down_t && t < put_down_t + 2.0) {
      r.post_release_dev = std::max(r.post_release_dev, std::abs(s[1] - theta_rest0));
    }
    if (put_down && t >= put_down_t + 7.0 && t < put_down_t + 8.0) {
      r.settled_dev = std::max(r.settled_dev, std::abs(s[1] - theta_rest0));
    }
    for (int n = 0; n < sub; ++n) {
      // Wheel speed servo (whole rpm, deadband) -> desired u''.
      double target = std::round(wheel_applied * 60.0 / (2.0 * kPi)) * 2.0 * kPi / 60.0;
      if (std::abs(target) < c.deadband) target = 0.0;
      double udd_des;
      if (c.wheel_mode == 0) {
        udd_des = (target - ud) / c.wheel_tau;
      } else {
        const double wn = 2.0 * kPi * c.wheel_wn_hz;
        ws_acc += (wn * wn * (target - ud) - 2.0 * c.wheel_zeta * wn * ws_acc) * dt;
        udd_des = ws_acc;
      }
      // Hip position servo (P position -> PI velocity -> torque), two joints.
      const double hq = backlash ? mh : s[2], hqd = backlash ? mhd : sd[2];
      const double kq = backlash ? mk : s[3], kqd = backlash ? mkd : sd[3];
      const double vel_sp = c.hip_kp * (hip_applied + hz - hq);
      const double vel_err = vel_sp - hqd;
      hip_integ = std::clamp(hip_integ + 2.0 * kv_joint * 2.0 * vel_err * dt, -2.0, 2.0);
      const double tau_h = std::clamp(2.0 * kv_joint * vel_err + hip_integ,
                                      -2.0 * c.hip_tau_max, 2.0 * c.hip_tau_max);
      r.max_tau_hip = std::max(r.max_tau_hip, std::abs(tau_h));
      // Knee position servo, same structure; its integrator carries the
      // body's weight, so it may use the whole torque range.
      const double vel_err_k = c.knee_kp * (knee_applied + kz - kq) - kqd;
      const double cap_k = 2.0 * c.knee_tau_max;
      knee_integ = std::clamp(knee_integ + 2.0 * kv_knee * 2.0 * vel_err_k * dt, -cap_k, cap_k);
      const double tau_k = std::clamp(2.0 * kv_knee * vel_err_k + knee_integ, -cap_k, cap_k);
      if (t > 2.0) r.max_tau_knee = std::max(r.max_tau_knee, std::abs(tau_k) / 2.0);
      // Through the gears (backlash): the links get the gear torque, the
      // rotors the servo torque minus it.
      const double tau_hl = backlash ? gear(mh, s[2], mhd, sd[2], c.hip_backlash, c.hip_rotor, c.gear_k) : tau_h;
      const double tau_kl = backlash ? gear(mk, s[3], mkd, sd[3], c.knee_backlash, c.knee_rotor, c.gear_k_knee > 0.0 ? c.gear_k_knee : c.gear_k) : tau_k;
      if (backlash) {
        mhd += (tau_h - tau_hl) / (2.0 * std::max(c.hip_rotor, 1.0e-4)) * dt;
        mh += mhd * dt;
        mkd += (tau_k - tau_kl) / (2.0 * std::max(c.knee_rotor, 1.0e-4)) * dt;
        mk += mkd * dt;
      }
      // Wheel torque that realises udd_des (linear in the torque), capped.
      const auto a0 = plant.accel(s, sd, 0.0, tau_hl, tau_kl, fx, c.slope);
      const auto a1 = plant.accel(s, sd, 1.0, tau_hl, tau_kl, fx, c.slope);
      const double u0 = a0[0] / R - a0[1] - a0[2] - a0[3];
      const double u1 = a1[0] / R - a1[1] - a1[2] - a1[3];
      const double tw = std::clamp((udd_des - u0) / (u1 - u0), -c.wheel_tau_max, c.wheel_tau_max);
      State a{};
      for (int j = 0; j < LegPlant::kDof; ++j) a[j] = a0[j] + tw * (a1[j] - a0[j]);
      for (int j = 0; j < LegPlant::kDof; ++j) {
        sd[j] += a[j] * dt; s[j] += sd[j] * dt; s_acc[j] = a[j];
      }
      const double udd = a[0] / R - a[1] - a[2] - a[3];
      if (c.wheel_mode == 1) ws_acc = udd;  // the servo state follows the real wheel
      ud += udd * dt;
      u += ud * dt;
    }
    // IMU specific force at its body point.
    {
      LegPlant::Jac J;
      Pt jd;
      plant.kin(s, sd, LegPlant::kImu, J, jd);
      double ax = jd.x, az = jd.z;
      for (int j = 0; j < LegPlant::kDof; ++j) { ax += J[j].x * s_acc[j]; az += J[j].z * s_acc[j]; }
      const double fxw = ax + kG * std::sin(c.slope), fzw = az + kG * std::cos(c.slope);
      const double th = s[1] + c.imu_mount;
      const double cth = std::cos(th), sth = std::sin(th);
      feed(fxw * cth - fzw * sth, fxw * sth + fzw * cth, sd[1]);
    }
    if (t > 2.0) {
      const double e = imu.pitch_rad - (s[1] + c.imu_mount) - est_bias0;
      r.est_err_max = std::max(r.est_err_max, std::abs(e));
      est_sq += e * e;
      ++est_n;
    } else {
      est_bias0 = imu.pitch_rad - (s[1] + c.imu_mount);
    }
    // Body CoM horizontal velocity / acceleration (exact, from the plant).
    double vb = 0.0, ab = 0.0;
    {
      LegPlant::Jac J;
      Pt jd;
      plant.kin(s, sd, 0, J, jd);
      ab = jd.x;
      double abz = jd.z;
      for (int j = 0; j < LegPlant::kDof; ++j) {
        vb += J[j].x * sd[j]; ab += J[j].x * s_acc[j]; abz += J[j].z * s_acc[j];
      }
      abz_f += std::min(2.0 * kPi * 20.0 * dt_c, 1.0) * (abz - abz_f);
      if (t > 2.0) {
        r.body_vacc_peak = std::max(r.body_vacc_peak, std::abs(abz_f));
        vacc_sq += abz_f * abz_f;
        ++vacc_n;
        const double zb = plant.world(s, 0).z;
        zb_min = std::min(zb_min, zb);
        zb_max = std::max(zb_max, zb);
        if (t > 11.0) { zl_min = std::min(zl_min, zb); zl_max = std::max(zl_max, zb); }
      }
      if (c.bump_h != 0.0 && c.drive_m_s != 0.0 && t >= c.drive_start_s &&
          t <= c.drive_end_s + 0.5) {
        const double axle_x = plant.world(s, LegPlant::kAxle).x;
        const double bump_lo = c.bump_x0 - 0.06;
        const double bump_hi = c.bump_x0 + 2.0 * c.bump_ramp + c.bump_width + 0.06;
        if (axle_x >= bump_lo && axle_x <= bump_hi) {
          r.body_vacc_bump_peak = std::max(r.body_vacc_bump_peak, std::abs(abz_f));
        }
      }
    }
    {
      const double a_prev = ab_f;
      ab_f += std::min(2.0 * kPi * 20.0 * dt_c, 1.0) * (ab - ab_f);  // 20 Hz: drop substep noise
      const double jerk = (ab_f - a_prev) / dt_c;
      const bool drive_ev = c.drive_m_s != 0.0 && t >= c.drive_start_s && t < c.drive_end_s + 3.0;
      const bool push_ev = c.push_n != 0.0 && t >= push_start_s && t < push_start_s + 2.0;
      if (drive_ev || push_ev) {
        r.body_acc_peak = std::max(r.body_acc_peak, std::abs(ab_f));
        ev_acc_sq += ab_f * ab_f;
        ev_jerk_sq += jerk * jerk;
        ++ev_n;
      }
      const double dir = c.drive_m_s >= 0.0 ? 1.0 : -1.0;
      if (c.drive_m_s != 0.0) {
        if (std::abs(t - c.drive_end_s) < 0.5 * dt_c) v_release = vb;
        if (t >= c.drive_end_s && t < c.drive_end_s + 1.5) {
          r.brake_bump = std::max(r.brake_bump, dir * (vb - v_release));
        }
        if (t >= c.drive_start_s && t < c.drive_start_s + 1.0) {
          r.start_dip = std::max(r.start_dip, -dir * (vb - v_start));
        } else if (t < c.drive_start_s) {
          v_start = vb;
        }
      }
    }
    const double lean = plant.leanAngle(s);
    if (std::abs(lean) > 0.6 || std::abs(s[1]) > 0.8) { r.fell = true; r.fault = "plant"; break; }
    // Body pitch at rest for the current height pose (CoM over the axle,
    // hips at the height pose), relative to which the body deviation counts.
    const double rest = theta_rest0 + (level_active ? c.level_rel : 0.0);
    const double dev = s[1] - rest;
    for (std::size_t pi = 0; pi < c.presses.size(); ++pi) {
      const auto& pr = c.presses[pi];
      const double end = pi+1 < c.presses.size() ? c.presses[pi+1].t0 : duration_s;
      if (t >= pr.t1 && t < end)
        r.press_rebound_rad = std::max(r.press_rebound_rad,
            std::max(-std::copysign(1.0, pr.x)*dev, 0.0));
    }
    if (k == 0) r.start_dev = dev;
    if (!c.pushes.empty() && !push_sequence_measured && t >= c.pushes.front().t0) {
      const Pt hip_pt = plant.worldOf(s, {0.0, 0.0});
      const Pt axle_pt = plant.world(s, LegPlant::kAxle);
      r.push_sequence_height_m = hip_pt.z - axle_pt.z;
      push_sequence_measured = true;
    }
    if (push_sequence_measured && !c.pushes.empty() &&
        t <= c.pushes.back().t1 + 0.5) {
      r.push_sequence_body_peak = std::max(r.push_sequence_body_peak, std::abs(dev));
    }
    if (c.excite_amp > 0.0) {
      if (t < c.excite_start_s) { excite_b0 = s[1]; excite_l0 = lean; }
      else if (t < c.excite_start_s + tap_fw::v3::kFloorTestDurationS + 1.0) {
        r.excite_body = std::max(r.excite_body, std::abs(s[1] - excite_b0));
        r.excite_lean = std::max(r.excite_lean, std::abs(lean - excite_l0));
        r.excite_knee_err = std::max(r.excite_knee_err,
                                     std::abs(double(ctrl.legPose().knee_offset_rad) - s[3]));
      }
    }
    if (c.push_n != 0.0) {
      if (t < push_start_s) { r.push_height = ctrl.legPose().axle_below_hip_m; push_b0 = s[1]; push_l0 = lean; push_q0 = s[2]; push_k0 = s[3]; push_z0 = plant.world(s, 0).z; }
      else if (t < push_start_s + 2.0) {
        r.push_body = std::max(r.push_body, std::abs(s[1] - push_b0));
        r.push_lean = std::max(r.push_lean, std::abs(lean - push_l0));
        r.push_hip = std::max(r.push_hip, std::abs(s[2] - push_q0));
        r.push_knee = std::max(r.push_knee, std::abs(s[3] - push_k0));
        r.push_dz = std::max(r.push_dz, std::abs(plant.world(s, 0).z - push_z0));
      }
      if (t >= push_start_s + 0.3 && t < push_start_s + 6.0) {
        r.push_opposite = std::max(r.push_opposite, (c.push_n > 0.0 ? -1.0 : 1.0) * dev);
      }
    }
    if (t > duration_s - 15.0) {
      const double th = ctrl.trunkHip();
      rest_trunk_lo = std::min(rest_trunk_lo, th); rest_trunk_hi = std::max(rest_trunk_hi, th);
      // Zigzag count with 5 mrad hysteresis: a direction change of the
      // hip command that is bigger than gear play, not sample-to-sample noise.
      if (!rev_init) { rev_ext = th; rev_init = true; }
      else if (rev_dir >= 0.0 && th < rev_ext - 0.005) { ++rev_n; rev_dir = -1.0; rev_ext = th; }
      else if (rev_dir <= 0.0 && th > rev_ext + 0.005) { ++rev_n; rev_dir = 1.0; rev_ext = th; }
      else if ((rev_dir >= 0.0 && th > rev_ext) || (rev_dir <= 0.0 && th < rev_ext)) rev_ext = th;
      rest_x_lo = std::min(rest_x_lo, s[0]); rest_x_hi = std::max(rest_x_hi, s[0]);
    }
    if (t > 2.0) {
      if (std::abs(dev) > 0.015) r.level_settle_s = t;
      r.body_dev_all = std::max(r.body_dev_all, std::abs(dev));
      r.lean_max = std::max(r.lean_max, std::abs(lean));
      // Dynamic part only (minus a 1 s low-pass): slopes, model error and
      // the level loop's slow settling are not what "share" asks about.
      const double a_hp = std::min(dt_c / 1.0, 1.0);
      all_body_lp += a_hp * (dev - all_body_lp);
      all_lean_lp += a_hp * (lean - all_lean_lp);
      all_body_sq += (dev - all_body_lp) * (dev - all_body_lp);
      all_lean_sq += (lean - all_lean_lp) * (lean - all_lean_lp);
      ++all_n;
      r.max_hip_rate = std::max(r.max_hip_rate, std::abs(sd[2]));
      r.max_hip_err = std::max(r.max_hip_err, std::abs(hip_applied + hz - s[2]));
      r.max_knee_err = std::max(r.max_knee_err, std::abs(knee_applied + kz - s[3]));
    }
    if (c.drive_m_s != 0.0 && t < c.drive_start_s) drive_z0 = plant.world(s, 0).z;
    if (c.drive_m_s != 0.0 && t >= c.drive_start_s && t < c.drive_end_s + 1.0) {
      r.drive_dz = std::max(r.drive_dz, std::abs(plant.world(s, 0).z - drive_z0));
      dv_sq += abz_f * abz_f; dp_sq += dev * dev; ++dv_n;
      r.drive_vacc_peak = std::max(r.drive_vacc_peak, std::abs(abz_f));
      r.body_dev_drive = std::max(r.body_dev_drive, std::abs(dev));
      rate_sq += sd[1] * sd[1];
      ++rate_n;
    }
    if (t > 2.0 && std::abs(dev) > 0.01) r.settle01_s = t;
    if (t > duration_s - 4.0) {
      body_mean_sum += dev;
      body_sq += dev * dev;
      rest_rate_sq += sd[1] * sd[1];
      rest_hip_sq += sd[2] * sd[2];
      ++body_n;
    }
    if (anim != nullptr && k % 5 == 0) {
      // Animation frame every 10 ms (tools/tier_sim_view.py): hip, knee,
      // axle, a schematic body box (body frame, hip origin) and signals.
      const auto pts = plant.points(s[2], s[3]);
      const Pt hip = plant.worldOf(s, {0.0, 0.0}), knee = plant.worldOf(s, pts.knee);
      const Pt ax = plant.world(s, LegPlant::kAxle), com = plant.world(s, 0);
      const Pt b[4] = {plant.worldOf(s, {-0.11, -0.035}), plant.worldOf(s, {0.11, -0.035}),
                       plant.worldOf(s, {0.11, 0.065}), plant.worldOf(s, {-0.11, 0.065})};
      std::fprintf(anim, "%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f", t, hip.x, hip.z,
                   knee.x, knee.z, ax.x, ax.z, com.x, com.z, sd[0], dev);
      for (const Pt& q : b) std::fprintf(anim, ",%.5f,%.5f", q.x, q.z);
      std::fprintf(anim, ",%.4f,%.4f,%.4f,%.3f,%.2f,%.5f,%.2f,%.3f,%.4f,%.2f", lean, s[2], s[3],
                   double(cmd.forward_m_s), fx, plant.ground ? plant.ground(s[0]) : 0.0, plant.ext.fz,
                   plant.ext.local.x, double(ctrl.trunkHip()), double(ctrl.trunkAuthority()));
      // As the firmware TR line logs them (tools/tr_rest_stats.py --sim):
      // the IMU pitch rate the controller got, theta_v rate, encoder axle_x.
      const auto split = ctrl.splitTelemetry();
      std::fprintf(anim, ",%.4f,%.4f,%.5f,%.6f,%.6f,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                   double(in.pitch_rate_rad_s), double(ctrl.virtualPitchRate()), double(ctrl.groundPosition()),
                   double(split.hip_a), double(split.hip_b), double(split.alpha_b), double(split.preview),
                   double(split.model_target), double(ctrl.legPose().axle_below_hip_m),
                   double(ctrl.heightSpeed()), double(ctrl.heightAcceleration()));
    }
    const int trace_every = std::getenv("TR_FINE") ? 1 : 25;
    if (c.trace && k % trace_every == 0 && std::getenv("TR_WBDBG")) {
      const auto d = ctrl.wbDebug();
      std::printf("WBD t=%.2f x=%+.3f xref=%+.3f xi=%+.4f thref=%+.4f vref=%+.3f drv=%d bias=%+.4f body=%+.4f\n",
                  t, s[0], d.x_ref, d.xi, d.theta_ref, d.v_ref, d.driving, d.bias, s[1] - theta_rest0);
      const auto& tm = ctrl.wbTerms();
      std::printf("WBT t=%.2f x %+.2f th %+.2f q %+.2f k %+.2f v %+.2f thd %+.2f qd %+.2f kd %+.2f xi %+.2f hist %+.2f\n",
                  t, tm[0], tm[1], tm[2], tm[3], tm[4], tm[5], tm[6], tm[7], tm[8], tm[9]);
    }
    if (c.trace && k % trace_every == 0) {
      std::printf("TRC t=%.2f x=%+.3f v=%+.3f body=%+.4f lean=%+.4f q=%+.4f qcmd=%+.4f tv=%+.4f "
                  "ref=%+.4f wcmd=%+.2f ta=%.2f lvl=%+.4f/%+.4f learn=%d/%+.4f est=%+.4f k=%+.4f kcmd=%+.4f az=%+.2f gz=%.4f\n", t, s[0], sd[0], dev,
                  lean, s[2], double(ctrl.hipCommand()), double(ctrl.virtualPitch()),
                  double(ctrl.balanceRef()), 0.5 * (out.wheel_cmd_rad_s[0] + out.wheel_cmd_rad_s[1]),
                  double(ctrl.trunkAuthority()), double(ctrl.levelHip()), double(ctrl.wbLevelHip()), ctrl.learnedTrimValid(),
                  double(ctrl.learnedTrim()),
                  double(imu.pitch_rad) - (s[1] + c.imu_mount) - est_bias0, s[3],
                  double(ctrl.kneeCommand()), abz_f, plant.ground ? plant.ground(s[0]) : 0.0);
    }
    r.max_speed = std::max(r.max_speed, std::abs(sd[0]));
    r.max_x = std::max(r.max_x, std::abs(s[0]));
    if (c.trace_position) r.x_trace.push_back(s[0]);
    if (c.height_low_m > 0.0) {
      const float height = ctrl.legPose().axle_below_hip_m;
      if (r.height_down_s < 0.0 && t >= c.height_low_s && t < c.height_up_s &&
          std::abs(height - c.height_low_m) < 0.002F) {
        r.height_down_s = t - c.height_low_s;
      }
      const float hmax = ctrl.supportedMaxHeight();
      if (r.height_up_s < 0.0 && t >= c.height_up_s && std::abs(height - hmax) < 0.002F) {
        r.height_up_s = t - c.height_up_s;
      }
    }
    if (k == static_cast<int>((c.drive_end_s + 3.0) / dt_c)) x_stop = s[0];
  }
  if (anim) std::fclose(anim);
  r.level_hip_end = ctrl.levelHip();
  r.body_vacc_rms = vacc_n ? std::sqrt(vacc_sq / vacc_n) : 0.0;
  r.body_z_range = zb_max > zb_min ? zb_max - zb_min : 0.0;
  r.body_z_range_late = zl_max > zl_min ? zl_max - zl_min : 0.0;
  r.drive_vacc_rms = dv_n ? std::sqrt(dv_sq / dv_n) : 0.0;
  r.drive_pitch_rms = dv_n ? std::sqrt(dp_sq / dv_n) : 0.0;
  r.learned_valid = ctrl.learnedTrimValid();
  r.x_end = s[0];
  r.x_stop_travel = s[0] - x_stop;
  r.rate_rms_drive = rate_n ? std::sqrt(rate_sq / rate_n) : 0.0;
  r.body_rms_end = body_n ? std::sqrt(body_sq / body_n) : 0.0;
  r.gyro_hf_rms = hf_n ? std::sqrt(hf_g_sq / hf_n) : 0.0;
  r.dhip_hf_rms = hf_n ? std::sqrt(hf_d_sq / hf_n) : 0.0;
  r.knee_cmd_rate_rms = knee_rate_n ? std::sqrt(knee_rate_sq / knee_rate_n) : 0.0;
  r.knee_cmd_range = knee_cmd_hi > knee_cmd_lo ? knee_cmd_hi - knee_cmd_lo : 0.0;
  r.body_mean_end = body_n ? body_mean_sum / body_n : 0.0;
  r.roll_app_end = roll_app_n ? roll_app_sum / roll_app_n : 0.0;
  r.roll_dh_end = roll_dh_n ? roll_dh_abs_sum / roll_dh_n : 0.0;
  r.roll_phi_end = roll_dh_n ? roll_phi_sum / roll_dh_n : 0.0;
  r.roll_ref_end = ctrl.rollReference();
  r.roll_mode_end = ctrl.rollMode();
  r.roll_dh_rev_s = duration_s > 2.0 ? roll_dh_revs / (duration_s - 2.0) : 0.0;
  r.body_rms_all = all_n ? std::sqrt(all_body_sq / all_n) : 0.0;
  r.lean_rms_all = all_n ? std::sqrt(all_lean_sq / all_n) : 0.0;
  r.rest_rate_rms = body_n ? std::sqrt(rest_rate_sq / body_n) : 0.0;
  r.rest_hip_rms = body_n ? std::sqrt(rest_hip_sq / body_n) : 0.0;
  r.rest_trunk_pp = rest_trunk_hi > rest_trunk_lo ? rest_trunk_hi - rest_trunk_lo : 0.0;
  r.rest_hip_reversals_s = rev_n / 15.0;
  r.lqr_lead_end = ctrl.lqrLeadEffective();
  r.rest_x_pp = rest_x_hi > rest_x_lo ? rest_x_hi - rest_x_lo : 0.0;
  r.held_resets = ctrl.heldResets();
  r.est_err_rms = est_n ? std::sqrt(est_sq / est_n) : 0.0;
  r.body_acc_rms = ev_n ? std::sqrt(ev_acc_sq / ev_n) : 0.0;
  r.body_jerk_rms = ev_n ? std::sqrt(ev_jerk_sq / ev_n) : 0.0;
  return r;
}

void report(const Case& c, const Result& r) {
  std::printf("%-34s fell=%d%s%s held=%d body_drive=%.3f body_all=%.3f lean=%.3f "
              "dhip=%.3f hip_rate=%.2f hip_err=%.3f tau_h=%.2f wcmd=%.1f v=%.2f "
              "rate_rms=%.2f rest=%.3f/%.3f x_end=%+.3f stop=%+.3f\n",
              c.name, r.fell, r.fell ? " " : "", r.fell ? r.fault : "", r.held_resets,
              r.body_dev_drive, r.body_dev_all, r.lean_max, r.max_dhip, r.max_hip_rate,
              r.max_hip_err, r.max_tau_hip, r.max_wheel_cmd, r.max_speed, r.rate_rms_drive,
              r.rest_rate_rms, r.rest_hip_rms, r.x_end, r.x_stop_travel);
}


const char* modeName(int mode) {
  return mode == 0 ? "v3.2" : mode == 1 ? "vpitch" : "vpitch+trunk";
}

Case withMode(Case c, int mode) {
  c.vpitch = mode >= 1;
  c.trunk = mode == 2;
  return c;
}

void runSuspensionPushRegression() {
  // Layered body hold and knee suspension under alternating fore/aft hand
  // pushes. Positive force is forward; every second pulse reverses direction.
  // Wait for a crouch target to settle before starting the pulse train, then
  // assert the plant's measured hip-to-axle height at the first impulse.
  struct Support { const char* name; int suspension; bool trunk; };
  const Support modes[] = {{"no trunk", 1, false}, {"position", 1, true}};
  for (double h : {double(LegHeight::maxHeight()), 0.110}) {
    double no_trunk_body = 0.0;
    for (const auto& mode : modes) {
      Case c = withMode(Case{}, 2);
      c.trunk = mode.trunk;
      c.name = "V31 alternating fore/aft pushes";
      c.duration_s = 23.0;
      if (h < LegHeight::maxHeight() - 1.0e-4) c.heights.push_back({1.0, h});
      c.suspension = mode.suspension;
      // Match the active app: roll is enabled, so the existing
      // startup reserves 12 mm before trunk sway. Exact max extension has
      // no horizontal workspace at unchanged height (covered by IK tests
      // and MAXSTART); do not claim body isolation there.
      c.roll = 1;
      for (int i = 0; i < 8; ++i) {
        const double t = 13.0 + 0.55 * i;
        c.pushes.push_back({t, t + 0.25, (i % 2 == 0) ? 4.0 : -4.0});
      }
      const Result r = simulate(c);
      std::printf("V31 h %.0f mm %-9s: fell=%d held=%d body=%.4f rad pushBody=%.4f rad "
                  "hPush=%.1f mm trunkHip=%.4f rad kneeTau=%.2f N m bodyZ(after11s)=%.1f mm\n",
                  h * 1000.0, mode.name, r.fell, r.held_resets, r.body_dev_all,
                  r.push_sequence_body_peak, r.push_sequence_height_m * 1000.0,
                  r.max_dhip, r.max_tau_knee, r.body_z_range_late * 1000.0);
      std::fflush(stdout);
      assert(!r.fell && r.held_resets == 0);
      assert(r.body_dev_all < 0.15);
      const double expected_height = std::min(h, double(LegHeight::maxHeight()) -
          double(tap_fw::v3::defaultV3Config().roll.reserve_m));
      assert(std::abs(r.push_sequence_height_m - expected_height) < 0.006);
      assert(r.push_sequence_body_peak < 0.15);
      if (!mode.trunk) {
        no_trunk_body = r.push_sequence_body_peak;
        assert(r.max_dhip < 1.0e-3);
      } else {
        assert(r.max_dhip > 0.01);
        assert(r.push_sequence_body_peak < 0.8 * no_trunk_body);
      }
      assert(r.max_tau_knee < 5.0);
    }
  }
}

void runHeightTransitionRegression() {
  const char* selected_height = std::getenv("TIER_HEIGHT_MM");
  for (double height : {0.110, 0.0784}) {
    if (selected_height &&
        std::abs(height * 1000.0 - std::atof(selected_height)) > 0.2) continue;
    Case hold = withMode(Case{}, 2);
    hold.name = "height transition reference";
    hold.duration_s = 30.0;
    hold.roll = 0;
    hold.trace_position = true;
    Case move = hold;
    move.name = "jerk-limited height transition";
    move.height_low_m = height;
    move.height_low_s = 5.0;
    move.height_up_s = 17.0;
    if (const char* animation = std::getenv("TR_HEIGHT_ANIM")) {
      move.animation_file = animation;
    }
    // Repeat equal and opposite fore/aft taps during both directions of
    // travel. The no-height run uses identical pulses as the station-hold
    // reference, so the measured X difference isolates the pose transition.
    if (std::getenv("TIER_HEIGHT_NOPUSH") == nullptr) {
      for (int i = 0; i < 4; ++i) {
        const double down_t = 6.0 + 0.55 * i;
        const double up_t = 18.0 + 0.55 * i;
        const double force = (i % 2 == 0) ? 4.0 : -4.0;
        hold.pushes.push_back({down_t, down_t + 0.25, force});
        hold.pushes.push_back({up_t, up_t + 0.25, force});
      }
    }
    move.pushes = hold.pushes;

    const Result reference = simulate(hold);
    const Result result = simulate(move);
    double peak_x_error = 0.0;
    assert(reference.x_trace.size() == result.x_trace.size());
    for (std::size_t i = 0; i < result.x_trace.size(); ++i) {
      peak_x_error = std::max(peak_x_error,
          std::abs(result.x_trace[i] - reference.x_trace[i]));
    }
    const double final_x_error = result.x_trace.empty() ? 1.0
        : result.x_trace.back() - reference.x_trace.back();
    std::printf("HEIGHT PROFILE h=%.1f mm %s: down=%.2f s up=%.2f s fell=%d body=%.4f rad "
                "peak station error=%.1f mm final=%.1f mm xRef=%.1f mm xMove=%.1f mm\n",
                height * 1000.0, hold.pushes.empty() ? "no push" : "alternating pushes",
                result.height_down_s, result.height_up_s,
                result.fell ? 1 : 0, result.body_dev_all, peak_x_error * 1000.0,
                final_x_error * 1000.0, reference.x_end * 1000.0, result.x_end * 1000.0);
    std::fflush(stdout);
    assert(!result.fell && result.held_resets == 0);
    assert(result.height_down_s >= 0.0 && result.height_down_s < 8.0);
    assert(result.height_up_s >= 0.0 && result.height_up_s < 7.0);
    assert(result.body_dev_all < 0.05);
    assert(peak_x_error < 0.05);
    assert(std::abs(final_x_error) < 0.03);
  }
}


// V33 / V34: the calm settings (Config::stand_leash_m, Tier::hip_deadband_rad,
// Tier::sat_margin) against the same nominal cases with them off.
Case calmCase(double h, bool calm) {
  Case c = withMode(Case{}, 2);
  if (h < 0.188) c.heights.push_back({1.0, h});
  if (calm) { c.stand_leash = 0.10; c.hip_db = 0.003; c.sat_margin = 0.8; }
  return c;
}

void runCalmRegression() {
  // V33: a hard push (wheels into their cap) must not throw the body the
  // other way on the stop - the spring back to the old spot and the hips
  // handing the braking lean to the body did - yet the robot still walks
  // home, and the catch itself holds.
  for (double h : {0.189, 0.110}) {
    for (double f : {6.0, -8.0, 8.0}) {
      Result r[2];
      for (int calm = 0; calm < 2; ++calm) {
        Case c = calmCase(h, calm);
        c.push_n = f; c.push_start_s = 14.0; c.push_end_s = 14.3; c.duration_s = 30.0;
        r[calm] = simulate(c);
      }
      std::printf("V33 %3.0f mm push %+.0f N off/on: fell %d/%d held %d/%d opposite %.3f/%.3f rad "
                  "x_end %+.3f/%+.3f m\n", h * 1000, f, r[0].fell, r[1].fell, r[0].held_resets,
                  r[1].held_resets, r[0].push_opposite, r[1].push_opposite, r[0].x_end, r[1].x_end);
      std::fflush(stdout);
      assert(!r[1].fell && r[1].held_resets == 0);
      // 8 N: at least a quarter less (sim: 0.07-0.08 -> 0.013-0.034 rad).
      // 6 N never reaches the wheel cap; the deadband leaves the body a
      // little more of it (110 mm: 0.019 -> 0.023), allowed up to 5 mrad.
      if (std::abs(f) > 7.0) assert(r[1].push_opposite <= 0.75 * r[0].push_opposite);
      else assert(r[1].push_opposite <= r[0].push_opposite + 0.005);
      assert(std::abs(r[1].x_end) < 0.10);
    }
  }
  // V34: standing, the hips stop twitching on noise and stick-slip and the
  // wheels stop wandering (at 84 mm the position trim wound into a +-0.2 m,
  // ~16 s cycle).
  for (double h : {0.189, 0.110, 0.084}) {
    Result r[2];
    for (int calm = 0; calm < 2; ++calm) {
      Case c = calmCase(h, calm);
      c.duration_s = 40.0;
      r[calm] = simulate(c);
    }
    std::printf("V34 %3.0f mm standing off/on: hip pp %.4f/%.4f rad wheel pp %.1f/%.1f mm "
                "rest %.3f/%.3f\n", h * 1000, r[0].rest_trunk_pp, r[1].rest_trunk_pp,
                r[0].rest_x_pp * 1000, r[1].rest_x_pp * 1000, r[0].rest_rate_rms, r[1].rest_rate_rms);
    std::fflush(stdout);
    assert(!r[1].fell && r[1].held_resets == 0);
    assert(r[1].rest_trunk_pp < 0.9 * r[0].rest_trunk_pp);
    assert(r[1].rest_x_pp < std::max(r[0].rest_x_pp, 0.06));
    assert(r[1].rest_rate_rms < r[0].rest_rate_rms + 0.01);
  }
}

// Plant variants around the fitted/CAD values the tiers must tolerate.
struct Variant { const char* name; void (*apply)(Case&); };
const Variant kVariants[] = {
    {"nominal", [](Case&) {}},
    // Around the measured wheel (20 ms + 0.035 s lag, reading 55 ms old).
    {"wheel lag .06", [](Case& c) { c.wheel_tau = 0.06; }},
    {"wheel lag .10", [](Case& c) { c.wheel_tau = 0.10; }},
    {"wheel delay +20ms", [](Case& c) { c.wheel_delay_ticks += 10; c.wheel_meas_delay_ticks += 5; }},
    {"hip slow kp10 bw30", [](Case& c) { c.hip_kp = 10; c.hip_vel_bw = 30; }},
    {"hip rotor x3", [](Case& c) { c.hip_rotor = 0.012; }},
    {"hip delay +10ms", [](Case& c) { c.hip_delay_ticks = 7; }},
    {"hip torque 1.5", [](Case& c) { c.hip_tau_max = 1.5; }},
    {"body dx 0 (model err)", [](Case& c) { c.body_dx = 0.0; }},
    {"body dx .045", [](Case& c) { c.body_dx = 0.045; }},
    {"imu mount -.08", [](Case& c) { c.imu_mount = -0.08; }},
    {"body mass x1.2", [](Case& c) { c.mass_scale = 1.2; }},
    {"fast wheel no db", [](Case& c) { c.wheel_delay_ticks = 4; c.deadband = 0; c.wheel_tau = 0.02; c.wheel_meas_delay_ticks = 20; }},
    {"hip vel noise 0.2", [](Case& c) { c.hip_vel_noise = 0.2; }},
    {"leg feedback 50 Hz", [](Case& c) { c.leg_fb_period_s = 0.02; }},
    {"knee soft (hip gains)", [](Case& c) { c.knee_vel_bw = -1.0; }},
    {"knee stiff", [](Case& c) { c.knee_kp = 50.0; }},
};

// V35: standing hip chatter. The tier-2 lead must not take its rate from the
// hips' own encoder (Tier::lead_gyro_only): reversals of the standing hip
// command drop by >= 4x (robot logs 2026-10-03 showed 57-71 % of 40 ms samples
// stepping > 0.01 rad). The deadband stays off: it added little and cost body rate.
// full: also bound what it costs - a 4 N push and a 0.55 m/s drive at three
// heights over seven plant variants, no new fall and body deviation within
// +0.03 rad of the old lead (TIER_HIPLEAD=1 runs it).
void runHipLeadRegression(bool full) {
  auto fixCase = [](Case& c, bool fix) {
    c.lead_gyro = fix ? 1 : 0;
    c.hip_vel_noise = 0.06;  // SENS hipv_rms on the robot at rest
  };
  for (double h : {0.1766, 0.110, 0.084}) {
    Result r[2];
    for (int fix = 0; fix < 2; ++fix) {
      Case c = withMode(Case{}, 2);
      if (h < 0.188) c.heights.push_back({1.0, h});
      c.roll = 1; c.suspension = 1;
      c.duration_s = 30.0;
      fixCase(c, fix != 0);
      r[fix] = simulate(c);
    }
    std::printf("V35 %3.0f mm standing old/new lead: hip reversals/s %.1f/%.1f hip pp %.4f/%.4f rad\n",
                h * 1000, r[0].rest_hip_reversals_s, r[1].rest_hip_reversals_s,
                r[0].rest_trunk_pp, r[1].rest_trunk_pp);
    std::fflush(stdout);
    assert(!r[0].fell && !r[1].fell && r[1].held_resets == 0);
    assert(r[1].rest_hip_reversals_s * 4.0 <= r[0].rest_hip_reversals_s);
    assert(r[1].rest_trunk_pp < r[0].rest_trunk_pp);
  }
  if (!full) return;
  int cases = 0, new_falls = 0;
  double worst = 0.0;
  for (double h : {0.084, 0.12, 0.1766}) {
    for (int scenario = 0; scenario < 2; ++scenario) {
      for (int v = 0; v < 7; ++v) {
        Result r[2];
        for (int fix = 0; fix < 2; ++fix) {
          Case c = withMode(Case{}, 2);
          c.heights.push_back({1.0, h});
          c.roll = 1; c.suspension = 1;
          c.duration_s = 24.0;
          if (scenario == 0) { c.push_n = 4.0; c.push_start_s = 14.0; c.push_end_s = 14.3; }
          else { c.drive_m_s = 0.55; c.drive_start_s = 12.0; c.drive_end_s = 16.0; }
          fixCase(c, fix != 0);
          kVariants[v].apply(c);  // once: some variants add to the delay ticks
          r[fix] = simulate(c);
        }
        ++cases;
        if (r[1].fell && !r[0].fell) {
          ++new_falls;
          std::printf("V35 NEW FALL h %.0f mm %s %s\n", h * 1000, scenario ? "drive" : "push 4 N", kVariants[v].name);
        }
        if (!r[0].fell && !r[1].fell) worst = std::max(worst, r[1].body_dev_all - r[0].body_dev_all);
      }
    }
  }
  std::printf("V35 %d push/drive cases: new falls %d, worst body deviation increase %.4f rad\n",
              cases, new_falls, worst);
  std::fflush(stdout);
  assert(new_falls == 0);
  assert(worst < 0.03);
}

// V36: soft event gate on the reflex hip path (Tier::reflex_gate). Standing the
// reflex no longer chatters the hip (>= 10x fewer reversals of the hip command),
// and it opens for disturbances: a 4 N push and a 0.55 m/s drive at three heights
// over seven plant variants have no new fall and the body stays within +0.04 rad
// of the ungated one; it must also stay calm with 40 ms more hip lag.
void runReflexGateRegression() {
  auto gate = [](Case& c, bool on) { c.reflex_gate = on ? 1 : 0; c.hip_vel_noise = 0.06; };
  for (double h : {0.1766, 0.110, 0.084}) {
    Result r[2];
    for (int on = 0; on < 2; ++on) {
      Case c = withMode(Case{}, 2);
      if (h < 0.188) c.heights.push_back({1.0, h});
      c.roll = 1; c.suspension = 1; c.duration_s = 30.0;
      gate(c, on != 0);
      r[on] = simulate(c);
    }
    std::printf("V36 %3.0f mm standing ungated/gated: hip reversals/s %.1f/%.1f hip pp %.4f/%.4f rad\n",
                h * 1000, r[0].rest_hip_reversals_s, r[1].rest_hip_reversals_s,
                r[0].rest_trunk_pp, r[1].rest_trunk_pp);
    std::fflush(stdout);
    assert(!r[1].fell && r[1].held_resets == 0);
    assert(r[1].rest_hip_reversals_s * 10.0 <= r[0].rest_hip_reversals_s + 1.0e-9);
    assert(r[1].rest_trunk_pp < r[0].rest_trunk_pp);
  }
  {
    Result r[2];
    for (int on = 0; on < 2; ++on) {
      Case c = withMode(Case{}, 2);
      c.heights.push_back({1.0, 0.1766});
      c.roll = 1; c.suspension = 1; c.duration_s = 30.0;
      c.hip_delay_ticks = 22;  // +40 ms
      gate(c, on != 0);
      r[on] = simulate(c);
    }
    std::printf("V36 177 mm +40 ms hip lag ungated/gated: hip pp %.4f/%.4f rad\n", r[0].rest_trunk_pp, r[1].rest_trunk_pp);
    assert(!r[1].fell && r[1].rest_trunk_pp < 0.03);
  }
  int cases = 0, new_falls = 0;
  double worst = 0.0;
  for (double h : {0.084, 0.12, 0.1766}) {
    for (int scenario = 0; scenario < 2; ++scenario) {
      for (int v = 0; v < 7; ++v) {
        Result r[2];
        for (int on = 0; on < 2; ++on) {
          Case c = withMode(Case{}, 2);
          c.heights.push_back({1.0, h});
          c.roll = 1; c.suspension = 1; c.duration_s = 24.0;
          if (scenario == 0) { c.push_n = 4.0; c.push_start_s = 14.0; c.push_end_s = 14.3; }
          else { c.drive_m_s = 0.55; c.drive_start_s = 12.0; c.drive_end_s = 16.0; }
          gate(c, on != 0);
          kVariants[v].apply(c);
          r[on] = simulate(c);
        }
        ++cases;
        if (r[1].fell && !r[0].fell) {
          ++new_falls;
          std::printf("V36 NEW FALL h %.0f mm %s %s\n", h * 1000, scenario ? "drive" : "push 4 N", kVariants[v].name);
        }
        if (!r[0].fell && !r[1].fell) worst = std::max(worst, r[1].body_dev_all - r[0].body_dev_all);
      }
    }
  }
  std::printf("V36 %d push/drive cases: new falls %d, worst body deviation increase %.4f rad\n", cases, new_falls, worst);
  std::fflush(stdout);
  assert(new_falls == 0);
  assert(worst < 0.04);
}

// V37: the sim's wheel model, calibrated to the robot's crouch behaviour (2026-10-05). The robot
// (b4b6d4a, 95f17a4: lqr_lead_s 0.08) stands at 78 mm; with lqr_lead_s 0.03 it falls ~2.7 s after
// the legs stop at ~100 mm. The default sim wheel (20 ms dead time, lag 0.035 s, reading 55 ms old)
// cannot show that (it even prefers 0.03); dead time 52 ms, lag 0.11 s, reading 54-80 ms old does:
// lead 0.03 falls at both crouch heights, lead 0.08 stands, for every hip lag up to +40 ms. And with
// lead 0.08 a larger hip share at crouch (height_exp 0.5 against 1.5) damps a 4 N push better.
void runCalibratedWheelRegression() {
  auto crouch = [](Case& c, double h, double lead, double exp_low, std::size_t meas, std::size_t hip_delay) {
    c.heights.push_back({1.0, h});
    c.roll = 0; c.suspension = 1;
    c.wheel_delay_ticks = 26; c.wheel_tau = 0.11; c.wheel_meas_delay_ticks = meas;
    c.hip_delay_ticks = hip_delay;
    c.hip_vel_noise = 0.06;
    c.lqr_lead = lead;
    c.height_exp_low = exp_low;
    c.push_n = 4.0; c.push_start_s = 14.0; c.push_end_s = 14.3;
    c.duration_s = 24.0;
  };
  int falls_03 = 0, falls_08 = 0, n = 0;
  double dev_15 = 0.0, dev_05 = 0.0;
  for (double h : {0.0997, 0.084}) {
    for (std::size_t meas : {27U, 40U}) {
      for (std::size_t hd : {2U, 22U}) {
        Case a = withMode(Case{}, 2), b = withMode(Case{}, 2), c05 = withMode(Case{}, 2);
        crouch(a, h, 0.03, 1.5, meas, hd);
        crouch(b, h, 0.08, 1.5, meas, hd);
        crouch(c05, h, 0.08, 0.5, meas, hd);
        const Result ra = simulate(a), rb = simulate(b), rc = simulate(c05);
        ++n;
        if (ra.fell) ++falls_03;
        if (rb.fell || rc.fell) ++falls_08;
        dev_15 += rb.body_dev_all; dev_05 += rc.body_dev_all;
      }
    }
  }
  std::printf("V37 calibrated wheel, %d crouch cases: lead .03 falls %d, lead .08 falls %d; 4 N push body "
              "deviation texp 1.5 / 0.5: %.3f / %.3f rad (mean)\n", n, falls_03, falls_08, dev_15 / n, dev_05 / n);
  std::fflush(stdout);
  assert(falls_03 >= n - 1);   // reproduces the robot's crouch fall with the old lead
  assert(falls_08 == 0);       // and its standing with 0.08, at any hip lag up to +40 ms
  assert(dev_05 < 0.9 * dev_15);
}

// V38: height-adaptive wheel-loop lead (Config::lqr_lead_adaptive). On the robot-consistent wheel
// model (V37) the lead in use grows as the CoM comes down, and the schedule is never worse than the
// fixed 0.08 that stood on the robot nor than the fixed 0.05 that is right only tall: no fall at any
// height, push deviation within +0.005 rad of the fixed 0.08 and < 0.9x the fixed 0.05 at 100 mm (the fixed 0.05 is only slightly worse at 84 mm in this single condition).
void runAdaptiveLeadRegression() {
  auto setup = [](Case& c, double h, int adapt, double fixed_lead) {
    c.heights.push_back({1.0, h});
    c.roll = 0; c.suspension = 1;
    c.wheel_delay_ticks = 22; c.wheel_tau = 0.09; c.wheel_meas_delay_ticks = 27;
    c.hip_vel_noise = 0.06;
    c.lqr_adapt = adapt; if (fixed_lead > 0) c.lqr_lead = fixed_lead;
    c.push_n = 4.0; c.push_start_s = 14.0; c.push_end_s = 14.3;
    c.duration_s = 24.0;
  };
  double lead_tall = 0.0, lead_low = 0.0;
  for (double h : {0.1766, 0.12, 0.0997, 0.084}) {
    Case a = withMode(Case{}, 2), b = withMode(Case{}, 2), c05 = withMode(Case{}, 2);
    setup(a, h, 1, 0.0); setup(b, h, 0, 0.08); setup(c05, h, 0, 0.05);
    const Result ra = simulate(a), rb = simulate(b), rc = simulate(c05);
    std::printf("V38 %5.1f mm: adaptive lead %.3f s, push body dev adaptive/fixed .08/fixed .05 = %.3f/%.3f/%.3f rad\n",
                h * 1000, ra.lqr_lead_end, ra.body_dev_all, rb.body_dev_all, rc.body_dev_all);
    std::fflush(stdout);
    assert(!ra.fell);
    assert(ra.body_dev_all <= rb.body_dev_all + 0.005);
    if (h < 0.105 && h > 0.09) assert(ra.body_dev_all < 0.9 * rc.body_dev_all);
    if (h > 0.17) lead_tall = ra.lqr_lead_end;
    if (h < 0.09) lead_low = ra.lqr_lead_end;
  }
  assert(lead_tall < 0.06 && lead_low > 0.085);  // the schedule spans its range
}

// V39: the suspension must not turn vertical-speed noise into height wander. Standing with the
// accelerometer noise x10 (robot-like vibration), the fixed-deadband suspension wanders the CoM
// height over a range several times larger than the noise-adaptive one, which stays as quiet as
// no suspension; with normal noise the adaptive one still damps a 5 mm bump (body vertical accel
// peak no worse than the fixed one + 5 %).
void runAdaptiveSuspensionRegression() {
  auto stand = [](Case& c, int adaptive, double acc) {
    c.suspension = 1; c.roll = 0; c.sus_adaptive = adaptive;
    c.acc_noise_scale = acc; c.hip_vel_noise = 0.06;
    c.wheel_delay_ticks = 22; c.wheel_tau = 0.09; c.wheel_meas_delay_ticks = 27;
    c.duration_s = 40.0;
  };
  Case a = withMode(Case{}, 2), b = withMode(Case{}, 2), c0 = withMode(Case{}, 2);
  stand(a, 0, 10.0); stand(b, 1, 10.0); stand(c0, 0, 10.0); c0.suspension = 0;
  const Result ra = simulate(a), rb = simulate(b), r0 = simulate(c0);
  std::printf("V39 standing, accelerometer noise x10: CoM height range late fixed/adaptive/no suspension = %.1f/%.1f/%.1f mm\n",
              ra.body_z_range_late * 1000, rb.body_z_range_late * 1000, r0.body_z_range_late * 1000);
  std::fflush(stdout);
  assert(!ra.fell && !rb.fell);
  assert(rb.body_z_range_late * 3.0 < ra.body_z_range_late);
  assert(rb.body_z_range_late < r0.body_z_range_late + 0.002);
  auto bump = [](Case& c, int adaptive) {
    c.suspension = 1; c.roll = 0; c.sus_adaptive = adaptive;
    c.bump_h = 0.005; c.bump_ramp = 0.015; c.bump_x0 = 0.25; c.bump_width = 0.03;
    c.drive_m_s = 0.30; c.drive_start_s = 12.0; c.drive_end_s = 16.0; c.duration_s = 21.0;
    c.wheel_delay_ticks = 22; c.wheel_tau = 0.09; c.wheel_meas_delay_ticks = 27;
  };
  Case fa = withMode(Case{}, 2), fb = withMode(Case{}, 2);
  bump(fa, 0); bump(fb, 1);
  const Result bf = simulate(fa), ba = simulate(fb);
  std::printf("V39 5 mm bump at 0.30 m/s: body vertical accel peak fixed/adaptive = %.2f/%.2f m/s2\n",
              bf.body_vacc_bump_peak, ba.body_vacc_bump_peak);
  std::fflush(stdout);
  assert(!bf.fell && !ba.fell);
  assert(ba.body_vacc_bump_peak <= 1.05 * bf.body_vacc_bump_peak);
}

// V41: the knee must not copy the hip's reflex swing. coordinateTrunkAxle's knee term grows as
// dhip^2, so without the follow filter the loaded knee is driven by the skateboard reflex
// (robot knee grind, 07/10). Alternating pushes and a 0.55 m/s drive at three heights over the
// plant variants: knee command rate rms/peak must drop a lot, the axle depth must stay within a
// few mm, and the body must not get worse or fall.
void runKneeFollowRegression() {
  const double hz = std::getenv("KF_HZ") ? std::atof(std::getenv("KF_HZ")) : 1.0;
  const double rate = std::getenv("KF_RATE") ? std::atof(std::getenv("KF_RATE")) : 0.6;
  const bool sweep = std::getenv("KF_SWEEP") != nullptr;
  // hipForKneeAtX with the IK's own knee must reproduce the IK's hip exactly.
  double ik_gap = 0.0;
  for (double depth : {0.185, 0.1766, 0.15, 0.11, 0.084}) {
    for (double x : {-0.05, -0.02, 0.0, 0.01, 0.03, 0.06}) {
      for (double pitch : {-0.05, 0.0, 0.08}) {
        const auto g = LegHeight::forAxleOnPlane(float(x), float(depth), float(pitch));
        if (!g.ok || g.limited) continue;
        const float hip = LegHeight::hipForKneeAtX(g.knee_offset_rad, g.x_world_m, float(pitch));
        ik_gap = std::max(ik_gap, std::abs(double(hip - g.hip_offset_rad)));
      }
    }
  }
  std::printf("V41 hipForKneeAtX vs forAxleOnPlane hip: max gap %.2e rad\n", ik_gap);
  assert(ik_gap < 2.0e-4);
  int episodes = 0, failures = 0;
  double ratio_sum = 0, worst_body = 0, worst_depth = 0, rms_off = 0, rms_on = 0, pk_off = 0, pk_on = 0;
  for (double h : {0.1766, 0.110, 0.084}) {
    for (int scen = 0; scen < 2; ++scen) {
      for (int vi : {0, 2, 3, 4, 6, 8}) {
        Result r[2];
        for (int on = 0; on < 2; ++on) {
          Case c = withMode(Case{}, 2);
          c.roll = c.suspension = 1;
          c.hip_vel_noise = 0.06;
          if (h < 0.188) c.heights.push_back({1.0, h});
          c.duration_s = 26;
          kVariants[vi].apply(c);
          if (scen == 0) {
            for (int i = 0; i < 8; ++i) {
              const double t = 13 + 0.7 * i;
              c.pushes.push_back({t, t + 0.3, i % 2 == 0 ? 4.5 : -4.5});
            }
          } else {
            c.drive_m_s = 0.55; c.drive_start_s = 13.0; c.drive_end_s = 17.0;
          }
          c.knee_follow_hz = on ? hz : 0.0; c.knee_follow_rate = rate;
          r[on] = simulate(c);
        }
        ++episodes;
        std::printf("V41 h=%3.0f %s %-24s | off: rate rms %.3f peak %.2f range %.3f dhip %.3f body %.4f%s | on: rms %.3f peak %.2f range %.3f dhip %.3f body %.4f depth %.1f mm%s\n",
                    h * 1000, scen ? "drive" : "push ", kVariants[vi].name,
                    r[0].knee_cmd_rate_rms, r[0].knee_cmd_rate_peak, r[0].knee_cmd_range, r[0].max_dhip,
                    r[0].body_dev_all, r[0].fell ? " FELL" : "",
                    r[1].knee_cmd_rate_rms, r[1].knee_cmd_rate_peak, r[1].knee_cmd_range, r[1].max_dhip,
                    r[1].body_dev_all, r[1].axle_depth_err_max * 1000, r[1].fell ? " FELL" : "");
        std::fflush(stdout);
        rms_off = std::max(rms_off, r[0].knee_cmd_rate_rms); rms_on = std::max(rms_on, r[1].knee_cmd_rate_rms);
        pk_off = std::max(pk_off, r[0].knee_cmd_rate_peak); pk_on = std::max(pk_on, r[1].knee_cmd_rate_peak);
        if (!r[1].fell) worst_depth = std::max(worst_depth, r[1].axle_depth_err_max);
        ratio_sum += r[0].knee_cmd_rate_rms > 1e-9 ? r[1].knee_cmd_rate_rms / r[0].knee_cmd_rate_rms : 1.0;
        worst_body = std::max(worst_body, r[1].body_dev_all - r[0].body_dev_all);
        const bool either_fell = r[0].fell || r[1].fell;  // rates mean nothing during a fall
        if (!sweep && ((r[1].fell && !r[0].fell) || r[1].body_dev_all > r[0].body_dev_all + 0.02 ||
                       (!either_fell && (r[1].knee_cmd_rate_peak > 0.5 * r[0].knee_cmd_rate_peak + 0.02 ||
                                         r[1].knee_cmd_rate_rms > 0.5 * r[0].knee_cmd_rate_rms + 0.005)))) ++failures;
      }
    }
  }
  std::printf("V41 knee follow %.2f Hz / %.2f rad/s: %d episodes, worst-case knee rate rms %.3f -> %.3f rad/s, peak %.2f -> %.2f rad/s, "
              "mean rms ratio %.2f, worst axle depth error %.1f mm, worst body change %+.4f rad, %d failures\n",
              hz, rate, episodes, rms_off, rms_on, pk_off, pk_on, ratio_sum / episodes, worst_depth * 1000, worst_body, failures);
  assert(failures == 0 || sweep);
  assert(worst_depth < 0.004);
}

// V42: the tier-2 hip must not buzz at 6-8 Hz after a push-and-brake. The body damping and the
// lead feed the raw gyro to the hip through ~70 ms of lag: near 7 Hz the loop phase reaches -180 deg
// and, once the disturbance gate opens the damping, a small limit cycle starts (robot logs 07/10:
// gyro >5 Hz 0.02 at rest -> 0.17-0.20 rad/s pushed, hip command >5 Hz 0.011 rad, in phase at
// 6-8 Hz). With the boot damping 0.08 the sim shows it from +30 ms of extra hip lag at 177 mm and
// the 4.5 Hz damping low-pass (Tier::body_damp_lpf_hz) removes it up to +60 ms at every height.
// Exploration: HF_FIX=1 (wide delays), HF_DELAYS="27,32", HF_DAMPS="0,0.08", HF_GYRN, HF_QUIET,
// HF_LPF (corner Hz of the 'on' column, default 4.5), TR_TBDLPF/TR_TLEADLPF; the default run asserts.
void runHfVibrationRegression() {
  const bool explore = std::getenv("HF_FIX") || std::getenv("HF_DELAYS") || std::getenv("HF_DAMPS") ||
      std::getenv("HF_QUIET") || std::getenv("HF_GYRN");
  std::vector<double> heights{0.1766, 0.110, 0.084};
  std::vector<std::size_t> delays{2u, 12u, 22u, 27u};  // hip lag +0 / +20 / +40 / +50 ms
  std::vector<double> dampings{0.08};
  if (const char* env = std::getenv("HF_DELAYS")) {
    delays.clear();
    for (const char* q = env; *q;) { delays.push_back(static_cast<std::size_t>(std::atoi(q))); while (*q && *q != ',') ++q; if (*q) ++q; }
  }
  if (const char* env = std::getenv("HF_DAMPS")) {
    dampings.clear();
    for (const char* q = env; *q;) { dampings.push_back(std::atof(q)); while (*q && *q != ',') ++q; if (*q) ++q; }
  }
  auto runCase = [&](double h, std::size_t delay, double bd, double lpf_hz) {
    Case c = withMode(Case{}, 2);
    c.boot_like = 1; c.roll = 0; c.suspension = 1;
    c.hip_vel_noise = 0.06;
    c.wheel_delay_ticks = 22; c.wheel_tau = 0.09; c.wheel_meas_delay_ticks = 27;
    if (h < 0.188) c.heights.push_back({1.0, h});
    c.duration_s = 26;
    c.hip_delay_ticks = delay; c.body_damping = bd; c.body_damp_lpf = lpf_hz;
    if (const char* g = std::getenv("HF_GYRN")) c.gyro_noise = std::atof(g);
    if (!std::getenv("HF_QUIET")) for (int i = 0; i < 6; ++i) {
      const double t = 13 + 0.9 * i;
      c.pushes.push_back({t, t + 0.3, i % 2 == 0 ? 4.0 : -4.0});
    }
    return simulate(c);
  };
  int failures = 0;
  double control_tall_50 = 0.0, worst_fix_hf = 0.0, worst_body_cost = -1.0;
  std::printf("V42 gyro/hip command above 5 Hz after pushes (boot-like tier), lpf off / 4.5 Hz\n");
  for (double h : heights) for (std::size_t delay : delays) for (double bd : dampings) {
    const double lpf_on = std::getenv("HF_LPF") ? std::atof(std::getenv("HF_LPF")) : 4.5;
    const Result off = runCase(h, delay, bd, 0.0), on = runCase(h, delay, bd, lpf_on);
    const double lag_ms = (double(delay) - 2.0) * 2.0;
    std::printf("V42 h=%3.0f hip lag +%2.0f ms damping %.2f | off: gyro %.3f dhip %.4f body %.4f%s | lpf: gyro %.3f dhip %.4f body %.4f%s\n",
                h * 1000, lag_ms, bd, off.gyro_hf_rms, off.dhip_hf_rms, off.body_dev_all, off.fell ? " FELL" : "",
                on.gyro_hf_rms, on.dhip_hf_rms, on.body_dev_all, on.fell ? " FELL" : "");
    std::fflush(stdout);
    if (h > 0.17 && lag_ms >= 49.0 && bd == 0.08) control_tall_50 = off.gyro_hf_rms;
    if (on.fell && !off.fell) ++failures;
    worst_body_cost = std::max(worst_body_cost, on.body_dev_all - off.body_dev_all);
    if (!explore && bd == 0.08 && lag_ms <= 50.0) {
      worst_fix_hf = std::max(worst_fix_hf, on.gyro_hf_rms);
      if (on.dhip_hf_rms > 0.011) ++failures;
      if (lag_ms >= 39.0 && h > 0.17 && on.gyro_hf_rms > 0.6 * off.gyro_hf_rms) ++failures;
    }
  }
  std::printf("V42 summary: control (177 mm, +50 ms, no low-pass) gyro>5Hz %.3f rad/s; with the low-pass worst %.3f; "
              "worst body cost %+.4f rad; %d failures\n", control_tall_50, worst_fix_hf, worst_body_cost, failures);
  if (!explore) {
    assert(control_tall_50 > 0.12);   // the test sees the oscillation the low-pass removes
    assert(worst_fix_hf < 0.085);
    assert(worst_body_cost < 0.012);
    assert(failures == 0);
  }
}

void runCoordinatedAdaptiveRegression() {
  int episodes = 0;
  int failures = 0;
  double worst_body = 0;
  for (double height : {double(LegHeight::maxHeight())-0.012, 0.110, 0.084}) {
    for (const auto& variant : kVariants) {
      Case c = withMode(Case{}, 2);
      c.name = "paired adaptive alternating pushes";
      c.roll = c.suspension = 1;
      c.duration_s = 23;
      c.heights.push_back({1.0, height});
      variant.apply(c);
      for (int i = 0; i < 8; ++i) {
        const double t = 13 + 0.55*i;
        c.pushes.push_back({t, t+0.25, i%2 == 0 ? 4.0 : -4.0});
      }
      const auto r = simulate(c);
      std::printf("PAIRED ADAPT h=%.1f mm %-24s fell=%d held=%d body=%.4f "
                  "hPush=%.1f mm rest=%.4f\n", height*1000, variant.name, r.fell,
                  r.held_resets, r.body_dev_all, r.push_sequence_height_m*1000, r.rest_rate_rms);
      std::fflush(stdout);
      const bool pass = !r.fell && r.held_resets == 0 && r.body_dev_all < 0.15 &&
          std::abs(r.push_sequence_height_m-height) < 0.006;
      if (!pass) ++failures;
      worst_body = std::max(worst_body, r.body_dev_all);
      ++episodes;
    }
  }
  std::printf("PAIRED ADAPT %d/%d pass, worst body %.4f rad\n", episodes-failures, episodes, worst_body);
  std::fflush(stdout);
  assert(failures == 0);
}

void runAlternatingPressRegression() {
  int total = 0, failures = 0;
  double worst_body = 0, worst_rebound = 0, worst_rest = 0;
  for (double h : {double(LegHeight::maxHeight())-0.012, 0.136, 0.110, 0.084}) {
    for (int vi : {0, 1, 2, 3, 12, 4, 6}) {
      Case c = withMode(Case{}, 2);
      c.name = "alternating nose/tail presses";
      c.roll = c.suspension = 1;
      c.duration_s = 30;
      c.heights.push_back({1.0, h});
      kVariants[vi].apply(c);
      for (int i = 0; i < 8; ++i) {
        const double start = 13+1.5*i;
        c.presses.push_back({start, start+0.5, i%2 ? -0.11 : 0.11, -3.0});
      }
      const auto r = simulate(c);
      const bool pass = !r.fell && r.held_resets == 0 && r.body_dev_all < 0.15 &&
                        r.rest_rate_rms < 0.15;
      std::printf("ALT PRESS h=%.1f mm %-22s fell=%d held=%d body=%.4f rebound=%.4f rest=%.4f pass=%d\n",
                  h*1000, kVariants[vi].name, r.fell, r.held_resets, r.body_dev_all,
                  r.press_rebound_rad, r.rest_rate_rms, pass);
      std::fflush(stdout);
      ++total;
      failures += !pass;
      worst_body = std::max(worst_body, r.body_dev_all);
      worst_rebound = std::max(worst_rebound, r.press_rebound_rad);
      worst_rest = std::max(worst_rest, r.rest_rate_rms);
    }
  }
  std::printf("ALT PRESS %d/%d pass body=%.4f rebound=%.4f rest=%.4f\n",
              total-failures, total, worst_body, worst_rebound, worst_rest);
  std::fflush(stdout);
  assert(failures == 0);
}

}  // namespace

// The operator's hand test (TIER_PRESS / ANIM_PRESS), on the body's front
// and back edges (+-0.11 m from the hip, top face): f [N] down on the nose
// for hold_s at 13 s and on the tail at 17 s (hold_s 0: none), then three
// 0.25 s pokes on the nose from 21 s and three on the tail from 25.5 s.
void addPresses(Case& c, double f, double hold_s) {
  if (hold_s > 0.0) {
    c.presses.push_back({13.0, 13.0 + hold_s, 0.11, -f});
    c.presses.push_back({17.0, 17.0 + hold_s, -0.11, -f});
  }
  const double poke = std::getenv("PRESS_POKE") ? std::atof(std::getenv("PRESS_POKE")) : 0.25;
  for (int i = 0; i < 3; ++i) c.presses.push_back({21.0 + 1.0 * i, 21.0 + poke + 1.0 * i, 0.11, -f});
  for (int i = 0; i < 3; ++i) c.presses.push_back({25.5 + 1.0 * i, 25.5 + poke + 1.0 * i, -0.11, -f});
}

// Random 30 s episode (TIER_GENERAL, V26): see there.
Case randomEpisode(int seed, bool wb, int* variant) {
  const int nv = static_cast<int>(sizeof(kVariants) / sizeof(kVariants[0]));
  std::mt19937 g(static_cast<unsigned>(seed));
  auto U = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(g); };
  Case c = withMode(Case{}, 2);
  if (wb) c.algo = V3Controller::Algo::WholeBody;
  const int vi = static_cast<int>(U(0, nv - 1e-9));
  if (vi > 0) kVariants[vi].apply(c);
  if (variant) *variant = vi;
  c.body_dx += U(-0.01, 0.01);
  c.mass_scale *= U(0.9, 1.15);
  c.imu_mount += U(-0.03, 0.03);
  c.slope = U(-0.03, 0.03);
  c.duration_s = 30.0;
  double t = 2.0;
  for (int k = 0; k < 3; ++k) {
    c.heights.push_back({t, U(0.080, 0.189)});
    t += U(6.0, 10.0);
  }
  const int np = 2 + static_cast<int>(U(0, 3));
  for (int k = 0; k < np; ++k) {
    const double t0 = U(5.0, 26.0), len = U(0.1, 0.8);
    const double f = (U(0, 1) < 0.5 ? -1 : 1) * U(1.5, 4.5) * std::min(1.0, 0.4 / len + 0.3);
    c.pushes.push_back({t0, t0 + len, f});
  }
  const double d0 = U(8.0, 20.0);
  c.drives.push_back({d0, d0 + U(1.5, 4.0), (U(0, 1) < 0.5 ? -1 : 1) * U(0.1, 0.35)});
  return c;
}

#ifdef TAP_HAVE_EIGEN
V3Controller::WbTable makeWbTable(const tap_fw::sim::WbWeights& w,
                                  const tap_fw::sim::WbActuators& a = {}) {
  const tap_fw::sim::WbDesign d(a, w);
  V3Controller::WbTable t{};
  const double lo = LegHeight::minHeight(), hi = LegHeight::maxHeight();
  for (int i = 0; i < tap_fw::v3::wb::kHeights; ++i) {
    const auto p = d.design(lo + (hi - lo) * i / (tap_fw::v3::wb::kHeights - 1));
    auto& e = t[static_cast<std::size_t>(i)];
    e.height_m = static_cast<float>(p.height_m);
    e.theta0 = static_cast<float>(p.theta0);
    e.q0 = static_cast<float>(p.q0);
    e.k0 = static_cast<float>(p.k0);
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < tap_fw::v3::wb::kStates; ++c) {
        e.K[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] = static_cast<float>(p.K(r, c));
      }
    }
  }
  return t;
}

// Whole-body score over the scenarios that matter (lower is better).
struct WbScore {
  double total = 0.0;
  int falls = 0;
  double lean_push = 0, body_push = 0, az_bump = 0, rest_rate = 0, rest_hip = 0, body_drive = 0;
  double overshoot = 0, body_after = 0;
  int held = 0;
};
WbScore scoreWb(const V3Controller::WbTable& t, bool verbose = false) {
  WbScore sc;
  auto base = [&]() {
    Case c = withMode(Case{}, 2);
    c.algo = V3Controller::Algo::WholeBody;
    c.wb_table = &t;
    if (std::getenv("WB_REALWHEEL")) {  // the measured wheel + its reading age, predictor on
      c.wheel_delay_ticks = 10; c.wheel_tau = 0.035; c.wheel_meas_delay_ticks = 27; c.wheel_pred = 1;
    }
    return c;
  };
  auto runc = [&](Case c, const char* name) {
    const Result r = simulate(c);
    if (r.fell) ++sc.falls;
    if (std::getenv("WB_ROBUST") && c.held_s <= 0.0) sc.held += r.held_resets;
    if (verbose) {
      std::printf("  %-16s fell=%d held=%d body=%.3f lean=%.3f rest=%.3f/%.3f az=%.2f x_end=%+.3f\n",
                  name, r.fell, r.held_resets, r.body_dev_all, r.lean_max, r.rest_rate_rms,
                  r.rest_hip_rms, r.body_vacc_peak, r.x_end);
    }
    return r;
  };
  {
    const Result r = runc(base(), "stand");
    sc.rest_rate += r.rest_rate_rms;
    sc.rest_hip += r.rest_hip_rms;
  }
  for (double f : {4.0, -4.0}) {
    Case c = base();
    c.push_n = f;
    const Result r = runc(c, f > 0 ? "push 4" : "push -4");
    sc.lean_push = std::max(sc.lean_push, r.lean_max);
    sc.body_push = std::max(sc.body_push, r.body_dev_all);
  }
  { Case c = base(); c.push_n = 6.0; runc(c, "push 6x0.3"); }
  {
    Case c = base();
    c.height_low_m = 0.136; c.height_low_s = 3; c.height_up_s = 60;
    c.push_n = 4.0; c.push_start_s = 12; c.push_end_s = 13; c.duration_s = 20;
    const Result r = runc(c, "136 push 4x1");
    sc.lean_push = std::max(sc.lean_push, r.lean_max);
  }
  {
    Case c = base();
    c.bump_h = 0.005; c.bump_x0 = 0.25; c.bump_width = 0.03;
    c.drive_m_s = 0.30; c.drive_end_s = 9.0; c.duration_s = 14;
    const Result r = runc(c, "bump 5mm");
    sc.az_bump = r.body_vacc_peak;
  }
  for (double v : {0.55, -0.55}) {
    Case c = base();
    c.drive_m_s = v;
    const Result r = runc(c, v > 0 ? "drive 0.55" : "drive -0.55");
    sc.body_drive = std::max(sc.body_drive, r.body_dev_drive);
    sc.overshoot = std::max(sc.overshoot, r.max_speed - 0.55);
    sc.body_after = std::max(sc.body_after, r.body_rms_end);
  }
  { Case c = base(); c.wheel_tau = 0.12; c.drive_m_s = 0.75; runc(c, "stop 0.75"); }
  { Case c = base(); c.held_s = 8.0; runc(c, "stand flow"); }
  if (std::getenv("WB_REALWHEEL")) {
    // The wheel model is a fit (TR at 25 Hz): score the gains around it -
    // dead time 10-30 ms, lag 0.02-0.06 s, reading age 40-70 ms.
    struct WV { std::size_t d; double tau; std::size_t m; };
    const WV wvs[] = {{5, 0.035, 27}, {15, 0.035, 27}, {10, 0.02, 27}, {10, 0.06, 27}, {10, 0.035, 20}, {10, 0.035, 35}};
    for (const auto& wv : wvs) {
      for (double h : {0.0, 0.110}) {
        Case c = base();
        c.wheel_delay_ticks = wv.d; c.wheel_tau = wv.tau; c.wheel_meas_delay_ticks = wv.m;
        if (h > 0) { c.height_low_m = h; c.height_low_s = 2.0; c.height_up_s = 1e9; c.push_wait_height = true; }
        c.push_n = 3.0; c.push_start_s = 8.0; c.push_end_s = 8.3; c.duration_s = 16.0;
        const Result r = runc(c, "wheel model variant");
        sc.held += r.held_resets;
        sc.rest_rate += r.rest_rate_rms / 12.0;
      }
    }
  }
  if (std::getenv("WB_ROBUST")) {
    // Hardware 2026-10-01: the generated gains rang cap to cap on the robot;
    // in the sim the same happens with encoder-rate noise or a stiffer knee
    // servo. Score the gains on those plants too (crouched and tall), and
    // count every false held re-zero.
    for (int vi : {13, 14, 16}) {
      for (double h : {0.0, 0.110}) {
        Case c = base();
        kVariants[vi].apply(c);
        if (h > 0) { c.height_low_m = h; c.height_low_s = 2.0; c.height_up_s = 1e9; c.push_wait_height = true; }
        c.push_n = 3.0; c.push_start_s = 8.0; c.push_end_s = 8.3; c.duration_s = 16.0;
        const Result r = runc(c, kVariants[vi].name);
        sc.held += r.held_resets;
        sc.rest_rate += r.rest_rate_rms / 6.0;
      }
    }
  }
  if (std::getenv("WB_SAT")) {
    // Wheel at its cap (hard stop, 6 N push, crouched 0.75 m/s cruise) on
    // the wheel variants: with the predictor these rang cap to cap where
    // the 3 N pushes above held (TIER_WPGRID).
    for (int vi : {1, 2, 3, 12}) {
      for (int k = 0; k < 3; ++k) {
        Case c = base();
        kVariants[vi].apply(c);
        if (k == 0) c.drive_m_s = 0.75;
        if (k == 1) c.push_n = 6.0;
        if (k == 2) { c.heights.push_back({1.0, 0.084}); c.drive_m_s = 0.75; c.drive_end_s = 12.0; c.duration_s = 18.0; }
        const Result r = runc(c, kVariants[vi].name);
        sc.held += r.held_resets;
      }
    }
  }
  double gen_share = 0.0;
  if (const char* ng = std::getenv("WB_GEN")) {
    // Random episodes (seeds 101.., disjoint from TIER_GENERAL's 1..48
    // check): a weight set that only wins the fixed scenarios overfits -
    // a body-weighted pass cut the push body 0.014 -> 0.008 rad and then
    // fell 3 / false-held 12 of 48 random episodes.
    const int n = std::atoi(ng);
    for (int e = 0; e < n; ++e) {
      Case c = randomEpisode(101 + e, true, nullptr);
      c.wb_table = &t;
      if (std::getenv("WB_REALWHEEL")) {
        c.wheel_delay_ticks = 10; c.wheel_tau = 0.035; c.wheel_meas_delay_ticks = 27; c.wheel_pred = 1;
      }
      const Result r = simulate(c);
      if (r.fell) ++sc.falls;
      sc.held += r.held_resets;
      gen_share += (r.lean_rms_all > 1e-4 ? r.body_rms_all / r.lean_rms_all : 0.0) / n;
    }
  }
  const double body_w = std::getenv("WB_BODYW") ? std::atof(std::getenv("WB_BODYW")) : 1.0;
  sc.total = 100.0 * sc.falls + 20.0 * sc.held + gen_share / 0.05 + sc.lean_push / 0.1 + body_w * sc.body_push / 0.02 + sc.az_bump / 3.0 +
      sc.rest_rate / 0.03 + sc.rest_hip / 0.1 + sc.body_drive / 0.02 +
      std::max(sc.overshoot, 0.0) / 0.03 + sc.body_after / 0.005;
  return sc;
}
#endif

// V40: layered roll = the lateral axis of the gimbal (docs/ROLL_REDESIGN_20261005.md). The plant's IMU
// mount and leg zeros are NOT the controller's constants (the old test shared the -0.040 with the
// controller and hid a fixed target that held the robot tilted with a permanent 5-11 mm side offset).
// all_modes prints off / old single loop / layered; the regression checks the layered mode only.
void runLayeredRoll(bool all_modes) {
  struct Sc {
    const char* name;
    void (*apply)(Case&);
    double max_dh_end_mm;   // mean |side offset| over the last 4 s
    double max_phi_end;     // |body roll vs gravity| mean over the last 4 s (< 0: not checked)
    double max_app;         // peak roll vs the apparent vertical after 2 s
    double min_load;        // smaller wheel's load share
    int mode;               // expected final roll mode (0 off/learning, 1 active, 2 latched)
    double t_settle_s;      // roll_t_last must be below this (< 0: not checked)
  };
  const Sc scs[] = {
      {"flat, mount -0.040", [](Case& c) { c.imu_roll_mount = -0.040; }, 1.0, 0.004, 0.008, 0.49, 1, -1},
      {"flat, mount -0.013", [](Case& c) { c.imu_roll_mount = -0.013; }, 1.0, 0.004, 0.008, 0.49, 1, -1},
      {"flat, mount 0", [](Case& c) { c.imu_roll_mount = 0.0; }, 1.0, 0.004, 0.008, 0.49, 1, -1},
      {"flat, mount +0.02", [](Case& c) { c.imu_roll_mount = 0.02; }, 1.0, 0.004, 0.008, 0.49, 1, -1},
      // Hand-arranged leg zeros are indistinguishable from the IMU mount: the body stays as tilted as
      // with roll off (no correction), but without a permanent side offset.
      {"flat, legs 8mm, mount -0.013", [](Case& c) { c.imu_roll_mount = -0.013; c.leg_side_err_m = 0.008; },
       1.0, -1.0, 0.032, 0.47, 1, -1},
      {"flat, noise x4, mount -0.013", [](Case& c) { c.imu_roll_mount = -0.013; c.roll_noise_scale = 4.0; },
       1.0, 0.005, 0.010, 0.49, 1, -1},
      {"flat, servo x0.5", [](Case& c) { c.imu_roll_mount = -0.013; c.roll_servo_scale = 0.5; },
       1.0, 0.004, 0.008, 0.49, 1, -1},
      {"flat, servo x2", [](Case& c) { c.imu_roll_mount = -0.013; c.roll_servo_scale = 2.0; },
       1.0, 0.004, 0.008, 0.49, 1, -1},
      {"slope +0.06 at 12 s", [](Case& c) { c.imu_roll_mount = -0.013; c.cross_slope = 0.06; c.slope_t0 = 12.0; },
       11.0, 0.004, 0.020, 0.47, 1, 17.5},
      {"slope -0.05 at 12 s", [](Case& c) { c.imu_roll_mount = -0.013; c.cross_slope = -0.05; c.slope_t0 = 12.0; },
       11.0, 0.004, 0.020, 0.47, 1, 17.5},
      // Beyond the legs' travel: the latch fades the loop out instead of pinning the legs at the stop.
      {"slope +0.15 at 12 s (too steep)", [](Case& c) { c.imu_roll_mount = -0.013; c.cross_slope = 0.15;
                                                          c.slope_t0 = 12.0; }, 1.0, -1.0, 0.16, 0.40, 2, -1},
      {"turn 0.55 m/s 1.5 rad/s", [](Case& c) { c.imu_roll_mount = -0.013; c.drive_m_s = 0.55; c.drive_start_s = 8;
                                                 c.drive_end_s = 16; c.yaw_rad_s = 1.5; c.yaw_t0 = 11;
                                                 c.yaw_t1 = 14; }, 1.0, 0.004, 0.030, 0.46, 1, -1},
      {"left bump 8 mm 0.3", [](Case& c) { c.imu_roll_mount = -0.013; c.bump_h = 0.008; c.bump_ramp = 0.04;
                                            c.bump_x0 = 0.25; c.bump_width = 0.05; c.bump_left_only = true;
                                            c.drive_start_s = 6; c.drive_m_s = 0.3; c.drive_end_s = 12; },
       1.0, 0.004, 0.026, 0.46, 1, -1},
  };
  for (const auto& sc : scs) {
    for (int mode = all_modes ? 0 : 2; mode < 3; ++mode) {  // off, old single loop, layered
      Case c = withMode(Case{}, 2);
      c.duration_s = 28.0;
      c.roll = mode > 0;
      if (mode == 1) { c.roll_learn = 0; c.roll_pre = 0; c.roll_damp = 0.0; c.roll_latch = 0; }
      if (mode == 2) { c.roll_learn = 1; c.roll_pre = 1; c.roll_damp = 0.0; c.roll_latch = 1; c.roll_gimbal = 1; }
      if (mode == 2 && std::getenv("RL_DAMP")) c.roll_damp = std::atof(std::getenv("RL_DAMP"));
      if (mode == 2 && std::getenv("RL_PRE")) c.roll_pre = std::atoi(std::getenv("RL_PRE"));
      if (std::getenv("RL_ONLY") && mode != std::atoi(std::getenv("RL_ONLY"))) continue;
      sc.apply(c);
      const Result r = simulate(c);
      std::printf("V40 %-32s %-7s fell=%d held=%d | dh_end %4.1f mm phi_end %+.4f ref %+.4f mode=%d rev/s %.1f | "
                  "app max %.3f (over 0.005 until %.1f s) vs gravity max %.3f load min %.2f dh max %4.1f mm body %.3f\n",
                  sc.name, mode == 0 ? "off" : mode == 1 ? "old" : "layered", r.fell, r.held_resets,
                  r.roll_dh_end * 1000, r.roll_phi_end, r.roll_ref_end, r.roll_mode_end, r.roll_dh_rev_s,
                  r.roll_app_max, r.roll_t_last, r.roll_world_max, r.wheel_load_min, r.roll_dh_max * 1000,
                  r.body_dev_all);
      std::fflush(stdout);
      if (mode != 2) continue;
      assert(!r.fell && r.held_resets == 0);
      assert(r.roll_dh_end * 1000 <= sc.max_dh_end_mm || sc.mode == 2);
      if (sc.max_phi_end >= 0.0) assert(std::abs(r.roll_phi_end) <= sc.max_phi_end);
      assert(r.roll_app_max <= sc.max_app && r.wheel_load_min >= sc.min_load);
      assert(r.roll_mode_end == sc.mode && r.roll_dh_rev_s < 1.5);
      if (sc.t_settle_s > 0.0) assert(r.roll_t_last < sc.t_settle_s);
    }
  }
  // Roll belongs to the gimbal: without the pitch level loop it does not act.
  {
    Case c = withMode(Case{}, 2);
    c.duration_s = 24.0;
    c.roll = 1; c.roll_learn = 1; c.roll_pre = 1; c.roll_latch = 1; c.roll_gimbal = 1; c.level = 0;
    c.imu_roll_mount = -0.013; c.cross_slope = 0.06; c.slope_t0 = 12.0;
    const Result r = simulate(c);
    std::printf("V40 roll without the pitch gimbal: dh_end %.1f mm mode=%d\n", r.roll_dh_end * 1000, r.roll_mode_end);
    assert(!r.fell && r.roll_mode_end == 0 && r.roll_dh_end < 1.0e-3);
  }
}

int main() {
  if (std::getenv("TIER_ADDWIFI_COMPARE")) {
    // Released standing balance/height changes, not the floor stand-up sequence.
    for (double h : {.12, .15, .18}) for (double e : {-.02, .02})
      for (std::size_t hd : {2U, 22U}) for (double wt : {.035, .070}) {
        Case c = withMode(Case{}, 2);
        c.name = "1840/addwifi balance diagnostic";
        c.duration_s = 8; c.seed_error = e; c.hip_delay_ticks = hd; c.wheel_tau = wt;
        c.heights.push_back({1.0, h});
        const auto r = simulate(c);
        std::printf("PROFILE h=%.3f seed=%+.3f hip_ticks=%zu wheel_tau=%.3f fell=%d pitch_rms=%.6f gyro_hf=%.6f hip_hf=%.6f\n",
                    h,e,hd,wt,r.fell?1:0,r.drive_pitch_rms,r.gyro_hf_rms,r.dhip_hf_rms);
      }
    return 0;
  }

  // V1: plant self-check - passive motion (no torques, free hip and knee) conserves
  // energy, so the numeric Kane equations are consistent.
  {
    LegPlant p(0.021, 1.0, 0.004, 0.004, LegHeight::hangHipCad());
    State s{0.0, 0.3, -0.2, -0.6}, sd{0.2, 0.5, 1.5, -1.0};
    const double e0 = p.energy(s, sd);
    const double dt = 1.0e-5;
    for (int i = 0; i < 50000; ++i) {  // 0.5 s, midpoint
      const auto a = p.accel(s, sd, 0.0, 0.0, 0.0, 0.0, 0.0);
      State sm, sdm;
      for (int j = 0; j < LegPlant::kDof; ++j) {
        sm[j] = s[j] + 0.5 * dt * sd[j]; sdm[j] = sd[j] + 0.5 * dt * a[j];
      }
      const auto am = p.accel(sm, sdm, 0.0, 0.0, 0.0, 0.0, 0.0);
      for (int j = 0; j < LegPlant::kDof; ++j) { s[j] += dt * sdm[j]; sd[j] += dt * am[j]; }
    }
    const double e1 = p.energy(s, sd);
    std::printf("V1 plant energy: %.6f -> %.6f J\n", e0, e1);
    assert(std::abs(e1 - e0) < 1.0e-3 * std::abs(e0) + 1.0e-4);
  }
  // V2: theta_v follows the measured hip with the gain the controller uses
  // (finite difference), and theta_v_rate is that gain times the hip rate.
  {
    auto cfg = tap_fw::v3::defaultV3Config();
    cfg.tier.virtual_pitch = true;
    V3Controller ctrl(cfg);
    V3Controller::Input in{};
    in.legs_measured = true;
    V3Controller::Command cmd{};
    auto tv = [&](float hip) {
      in.hip_rad = {hip, hip};
      (void)ctrl.update(in, cmd);
      return ctrl.virtualPitch();
    };
    const float fd = (tv(0.02F) - tv(-0.02F)) / 0.04F;
    in.hip_rad = {0.0F, 0.0F};
    in.hip_rate_rad_s = {1.0F, 1.0F};
    (void)ctrl.update(in, cmd);
    std::printf("V2 d com_line / d hip: fd %.4f analytic %.4f rate %.4f; theta_v(hang) %+.5f\n",
                fd, ctrl.comLineHipGain(), ctrl.virtualPitchRate(), tv(0.0F));
    assert(std::abs(fd - ctrl.comLineHipGain()) < 0.01F);
    assert(std::abs(ctrl.virtualPitchRate() - ctrl.comLineHipGain()) < 1.0e-4F);
    assert(std::abs(tv(0.0F)) < 1.0e-6F);  // at the hang pose theta_v = body pitch
  }
  // V2b: the model check. Balancing crouched at 110 mm on a stiff stand-in
  // (legs at the commanded pose): fine; with the hip reading sign-flipped
  // (a wiring/sign error) it faults within the check window.
  for (int flip = 0; flip < 2; ++flip) {
    auto cfg = tap_fw::v3::defaultV3Config();
    cfg.tier.virtual_pitch = true;
    // Isolate the sensor/model sign guard; tier-2 body leveling is tested in
    // V25 and must not move the expected pose while this fixture stays rigid.
    cfg.tier.trunk = false;
    cfg.tier.level = false;
    cfg.engage.immediate = true;
    V3Controller ctrl(cfg);
    V3Controller::Input in{};
    in.legs_measured = true;
    in.legs_ready = true;
    V3Controller::Command cmd{};
    cmd.start = true;
    (void)ctrl.update(in, cmd);
    cmd.start = false;
    const char* fault = nullptr;
    for (int k = 1; k < 4000 && fault == nullptr; ++k) {
      in.now_us = 2000U * static_cast<std::uint64_t>(k) + 1'300'000U;
      cmd.height_m = 0.110F;
      const auto& pose = ctrl.legPose();
      const float hip = (flip ? -1.0F : 1.0F) * pose.hip_offset_rad;
      in.hip_rad = {hip, hip};
      in.knee_rad = {pose.knee_offset_rad, pose.knee_offset_rad};
      in.leg_chain_rad = hip + pose.knee_offset_rad;
      // Keep the virtual pitch at the balance point so only the check acts.
      in.pitch_rad = -(ctrl.virtualPitch() - in.pitch_rad);
      const auto out = ctrl.update(in, cmd);
      fault = out.fault;
    }
    std::printf("V2b model check, hip sign %s: %s (h %.0f mm)\n", flip ? "flipped" : "right",
                fault ? fault : "no fault", ctrl.legPose().axle_below_hip_m * 1000.0F);
    assert((fault != nullptr) == (flip == 1));
  }
  auto run = [](Case c) { const Result r = simulate(c); report(c, r); return r; };
  if (std::getenv("TIER_ALT_PRESS") != nullptr) {
    runAlternatingPressRegression();
    return 0;
  }
  if (std::getenv("TIER_PAIRED_ADAPT") != nullptr) {
    runCoordinatedAdaptiveRegression();
    return 0;
  }
  if (std::getenv("TIER_HEIGHT_PROFILE") != nullptr) {
    runHeightTransitionRegression();
    return 0;
  }
  if (std::getenv("TIER_SUSPENSION_PUSH") != nullptr) {
    runSuspensionPushRegression();
    return 0;
  }
  if (std::getenv("TIER_HFVIB") != nullptr) {
    runHfVibrationRegression();
    return 0;
  }
  if (std::getenv("TIER_KNEEFOLLOW") != nullptr) {
    runKneeFollowRegression();
    return 0;
  }
  if (std::getenv("TIER_ADAPTSUS") != nullptr) {
    runAdaptiveSuspensionRegression();
    return 0;
  }
  if (std::getenv("TIER_ADAPTLEAD") != nullptr) {
    runAdaptiveLeadRegression();
    return 0;
  }
  if (std::getenv("TIER_WHEELCAL") != nullptr) {
    runCalibratedWheelRegression();
    return 0;
  }
  if (std::getenv("TIER_REFLEXGATE") != nullptr) {
    runReflexGateRegression();
    return 0;
  }
  if (std::getenv("TIER_HIPLEAD") != nullptr) {
    runHipLeadRegression(true);
    return 0;
  }
  if (std::getenv("TIER_CALM") != nullptr) {
    runCalmRegression();
    return 0;
  }
  if (std::getenv("TIER_MAXSTART") != nullptr) {
    // Max-extension start with the boot profile (knees are position servos).
    // Immediate placement at engagement: measured legs start fully extended.
    // No use of held_s: that older fixture pins the measured knee to zero
    // then replaces it with the commanded knee at placement. It cannot prove
    // continuous joint motion for a long spell on the stand.
    // This fixture covers the floor catch and roll-reserve transition only;
    // it does not model touchdown impact or the preceding hand-held motion.
    const bool animation = std::getenv("MS_ANIM") != nullptr;
    const double selected_force = animation && std::getenv("MS_FORCE")
        ? std::atof(std::getenv("MS_FORCE")) : 3.0;
    const double selected_bias = animation && std::getenv("MS_BIAS")
        ? std::atof(std::getenv("MS_BIAS")) : 0.02;
    int episodes = 0;
    double peak = 0, rest = 0;
    for (int roll : {0, 1}) for (double bias : {-0.06, -0.02, 0.0, 0.02, 0.06}) {
      for (double force : {0.0, 3.0}) for (int vi : {0, 1, 2, 3, 12, 4, 6}) {
        const double run_force = animation && std::getenv("MS_FORCE") ? selected_force : force;
        if (animation && (roll != (std::getenv("MS_ROLL") ? std::atoi(std::getenv("MS_ROLL")) : 1)
                          || bias != selected_bias || force != (selected_force == 0.0 ? 0.0 : 3.0) || vi != 0)) continue;
        Case c = withMode(Case{}, 2);
        c.roll = roll;
        c.seed_error = bias;
        c.duration_s = 18;
        kVariants[vi].apply(c);
        if (run_force > 0) {
          const bool late_only = animation && std::getenv("MS_LATE_ONLY") != nullptr;
          if (late_only) {
            c.presses = {{10.0, 10.5, 0.11, -run_force}, {13.0, 13.5, -0.11, -run_force}};
          } else {
            c.presses = {{0.25, 0.5, 0.11, -run_force}, {1.25, 1.5, -0.11, -run_force},
                         {10.0, 10.5, 0.11, -run_force}, {13.0, 13.5, -0.11, -run_force}};
          }
        }
        const Result r = simulate(c);
        std::printf("MAXSTART roll=%d bias=%+.3f force=%.0f variant=%d height=%.1f mm "
                    "fell=%d capture=%.4f body=%.4f rest=%.4f%s%s\n",
                    roll, bias, run_force, vi, r.capture_height_m * 1000, r.fell,
                    r.capture_body_peak, r.body_dev_all, r.rest_rate_rms,
                    r.fell ? " fault=" : "", r.fell ? r.fault : "");
        std::fflush(stdout);
        assert(std::abs(r.capture_height_m - LegHeight::maxHeight()) < 0.0001);
        assert(!r.fell && r.capture_body_peak <= 0.15 && r.body_dev_all <= 0.15);
        peak = std::max(peak, std::max(r.capture_body_peak, r.body_dev_all));
        rest = std::max(rest, r.rest_rate_rms);
        ++episodes;
      }
    }
    std::printf("MAXSTART PASS %d episodes peak %.4f rad rest %.4f rad/s\n", episodes, peak, rest);
    return 0;
  }
  if (const char* tr = std::getenv("TIER_TRACE")) {
    // Diagnostics: TIER_TRACE=<drive m/s> [TR_MODE=0|1|2] [TR_H=<height m>] [TR_IDEAL=1]
    Case c = withMode(Case{}, std::getenv("TR_MODE") ? std::atoi(std::getenv("TR_MODE")) : 2);
    c.drive_m_s = std::atof(tr);
    c.trace = true;
    c.ideal_imu = std::getenv("TR_IDEAL") != nullptr;
    if (const char* v = std::getenv("TR_KIN")) c.imu_kin = std::atoi(v);
    if (const char* v = std::getenv("TR_PREV")) c.preview = std::atoi(v);
    if (const char* v = std::getenv("TR_LPF")) c.lead_lpf = std::atof(v);
    if (const char* v = std::getenv("TR_TAU")) c.wheel_tau = std::atof(v);
    if (const char* v = std::getenv("TR_PUSH")) c.push_n = std::atof(v);
    if (const char* v = std::getenv("TR_IMUMOUNT")) c.imu_mount = std::atof(v);
    if (const char* v = std::getenv("TR_BODYDX")) c.body_dx = std::atof(v);
    if (const char* v = std::getenv("TR_GRAB")) { c.grab_s = std::atof(v); c.duration_s = c.grab_s + 8.0; }
    if (const char* v = std::getenv("TR_BIAS")) c.grab_release_bias_rad = std::atof(v);
    if (const char* v = std::getenv("TR_LEVEL")) c.level_rel = std::atof(v);
    if (const char* v = std::getenv("TR_HVN")) c.hip_vel_noise = std::atof(v);
    if (const char* v = std::getenv("TR_LEARN")) c.learn_hold = std::atof(v);
    if (const char* v = std::getenv("TR_KKP")) c.knee_kp = std::atof(v);
    if (const char* v = std::getenv("TR_ALGO")) c.algo = static_cast<V3Controller::Algo>(std::atoi(v));
    if (const char* v = std::getenv("TR_WBB")) wb_bias_gain_override = std::atof(v);
    if (const char* v = std::getenv("TR_BUMP")) {
      c.bump_h = std::atof(v); c.bump_x0 = 0.25; c.bump_width = 0.03; c.drive_end_s = 9.0; c.duration_s = 12.0;
    }
    if (const char* v = std::getenv("TR_KBW")) c.knee_vel_bw = std::atof(v);
    if (const char* v = std::getenv("TR_EXC")) { c.excite_amp = std::atof(v); c.excite_start_s = 10.0; c.duration_s = 20.0; }
    if (const char* v = std::getenv("TR_LVL")) c.level = std::atoi(v);
    if (const char* v = std::getenv("TR_DUR")) c.duration_s = std::atof(v);
    if (const char* v = std::getenv("TR_SEED")) c.seed_error = std::atof(v);
    if (const char* v = std::getenv("TR_ROLL")) c.roll = std::atoi(v);
    if (const char* h = std::getenv("TR_H")) {
      c.height_low_m = std::atof(h); c.height_low_s = 5; c.height_up_s = 17; c.duration_s = 30;
    }
    c.name = "trace";
    run(c);
    return 0;
  }
  if (std::getenv("TIER_BUMP") != nullptr) {
    // Which floor bumps can the wheels climb (BL4820 torque cap)?
    for (double h : {0.003, 0.005, 0.008, 0.012}) {
      for (double ramp : {0.02, 0.04, 0.06}) {
        std::printf("h %2.0f mm ramp %2.0f mm (max slope %2.0f deg):", h * 1000, ramp * 1000,
                    std::atan(h * kPi / (2 * ramp)) * 180 / kPi);
        for (double v : {0.15, 0.30}) {
          Case c = withMode(Case{}, 2);
          c.bump_h = h; c.bump_x0 = 0.25; c.bump_ramp = ramp; c.bump_width = 0.03;
          c.drive_m_s = v; c.drive_start_s = 6.0; c.drive_end_s = 6.0 + 0.6 / v + 1.0;
          c.duration_s = c.drive_end_s + 5.0;
          const Result r = simulate(c);
          std::printf("  %.2f m/s: %s az %.1f body %.3f x_end %.2f", v, r.fell ? "FALL" : "ok  ",
                      r.body_vacc_peak, r.body_dev_all, r.x_end);
        }
        std::printf("\n");
      }
    }
    return 0;
  }
  if (std::getenv("TIER_KNEECAL") != nullptr) {
    // Knee servo stiffness vs the hardware: plain LQR (no theta_v) standing calm at
    // a pitch-rate rms of 0.09-0.32 rad/s on 2026-09-29 (TR logs).
    for (double kp : {16.0, 30.0, 50.0}) {
      for (double bw : {115.0, 200.0, 350.0}) {
        double rr[3] = {};
        for (int mode : {0, 2}) {
          Case c = withMode(Case{}, mode);
          c.knee_kp = kp; c.knee_vel_bw = bw;
          rr[mode] = simulate(c).rest_rate_rms;
        }
        Case p = withMode(Case{}, 2);
        p.knee_kp = kp; p.knee_vel_bw = bw; p.push_n = 4.0;
        const Result rp = simulate(p);
        std::printf("knee kp %4.0f bw %4.0f (stiffness %5.1f N m/rad/knee): v3.2 rest rate %.3f, "
                    "tiers rest %.3f, tiers push body %.3f lean %.3f\n", kp, bw,
                    kp * bw * (geo::leg_knee_inertia_kgm2 + 0.004), rr[0], rr[2], rp.body_dev_all,
                    rp.lean_max);
      }
    }
    return 0;
  }
  if (std::getenv("TIER_RATIO") != nullptr) {
    const double hang = LegHeight::forHeight(LegHeight::maxHeight()).com_above_axle_m;
    LegPlant lp(0.021, 1.0, 0.004, 0.004, LegHeight::hangHipCad());
    for (double h : {0.189, 0.170, 0.150, 0.136, 0.120, 0.110, 0.095, 0.078}) {
      const auto p = LegHeight::forHeight(static_cast<float>(h));
      {
        // Leg Jacobian (hip-to-axle height) and CoM-line partials at the pose.
        const float hip_cad = LegHeight::hangHipCad() + p.hip_offset_rad;
        const auto L = robot_control::WholeBodyModel::legHeight(hip_cad, p.knee_offset_rad);
        State s0{0.0, 0.0, p.hip_offset_rad, p.knee_offset_rad};
        for (int i = 0; i < 40; ++i) s0[1] -= lp.leanAngle(s0);
        auto dl = [&](int j) { State a = s0, b = s0; a[j] += 1e-5; b[j] -= 1e-5;
                               return (lp.leanAngle(a) - lp.leanAngle(b)) / 2e-5; };
        auto dzb = [&](int j) { State a = s0, b = s0; a[j] += 1e-5; b[j] -= 1e-5;
                                return (lp.world(a, 0).z - lp.world(b, 0).z) / 2e-5; };
        std::printf("   h %3.0f: dx/dhip %+.4f dx/dknee %+.4f (IK, wheel fore/aft at constant height: hip %+.3f knee %+.3f rad/cm)\n",
                    h * 1000, L.d_x_d_hip, L.d_x_d_knee,
                    0.01 * L.d_height_d_knee / (L.d_x_d_hip * L.d_height_d_knee - L.d_x_d_knee * L.d_height_d_hip),
                    -0.01 * L.d_height_d_hip / (L.d_x_d_hip * L.d_height_d_knee - L.d_x_d_knee * L.d_height_d_hip));
        std::printf("   h %3.0f: dh/dhip %+.4f dh/dknee %+.4f | lean/dhip %+.3f lean/dknee %+.3f | "
                    "body z/dhip %+.4f /dknee %+.4f\n", h * 1000, L.d_height_d_hip, L.d_height_d_knee,
                    dl(2), dl(3), dzb(2), dzb(3));
      }
      const double r = p.com_above_axle_m / hang;
      const auto lo = LegHeight::forHeight(static_cast<float>(h - 0.008));
      std::printf("h %3.0f mm: com %.1f mm ratio %.3f  share^1.5 %.2f  ^1.0 %.2f  ^0.5 %.2f  knee %+.3f hip %+.3f"
                  "  (-8 mm: dknee %+.3f dhip %+.3f)\n", h * 1000,
                  p.com_above_axle_m * 1000, r, std::pow(r, 1.5), r, std::sqrt(r), p.knee_offset_rad,
                  p.hip_offset_rad, lo.knee_offset_rad - p.knee_offset_rad, lo.hip_offset_rad - p.hip_offset_rad);
    }
    return 0;
  }
#ifdef TAP_HAVE_EIGEN
  if (std::getenv("TIER_SERVO") != nullptr) {
    // Faster joint servos (GDS68 position gain): WB redesigned for each,
    // plant to match. Push at 189/110/84 mm, crouched fast drive + push.
    for (double kp : {16.0, 25.0, 35.0}) {
      tap_fw::sim::WbActuators a;
      a.hip_kp = a.knee_kp = kp;
      const auto t = makeWbTable(tap_fw::sim::WbWeights{}, a);
      for (double h : {0.0, 0.110, 0.084}) {
        Case c = withMode(Case{}, 2);
        c.algo = V3Controller::Algo::WholeBody;
        c.wb_table = &t;
        c.hip_kp = c.knee_kp = kp;
        if (h > 0) { c.height_low_m = h; c.height_low_s = 2.0; c.height_up_s = 1e9; c.push_wait_height = true; }
        c.push_n = 4.0; c.push_start_s = 8.0; c.push_end_s = 8.3; c.duration_s = 16.0;
        const Result r = simulate(c);
        std::printf("kp %2.0f push h %3.0f: fell=%d held=%d body %.3f lean %.3f share %.2f hip %.3f rest %.3f max hip rate %.2f\n",
                    kp, h > 0 ? h * 1000 : 189.0, r.fell, r.held_resets, r.push_body, r.push_lean,
                    r.push_lean > 1e-4 ? r.push_body / r.push_lean : 0.0, r.push_hip, r.rest_rate_rms, r.max_hip_rate);
      }
      Case c = withMode(Case{}, 2);
      c.algo = V3Controller::Algo::WholeBody;
      c.wb_table = &t;
      c.hip_kp = c.knee_kp = kp;
      c.heights.push_back({1.0, 0.084});
      c.drive_m_s = 0.75; c.drive_start_s = 12.0; c.drive_end_s = 18.0; c.duration_s = 26.0;
      c.pushes.push_back({15.0, 15.3, -3.0});
      const Result r = simulate(c);
      std::printf("kp %2.0f drive 84 mm: fell=%d body %.3f lean %.3f vmax %.2f\n", kp, r.fell, r.body_dev_drive,
                  r.lean_max, r.max_speed);
      std::fflush(stdout);
    }
    return 0;
  }
  if (std::getenv("TIER_WBTUNE") != nullptr) {
    // Coordinate search on the whole-body cost weights (log scale, x4 / /4,
    // then x2 / /2), scored over the scenarios in scoreWb.
    tap_fw::sim::WbWeights w;
    struct P { const char* name; double* v; };
    std::vector<P> ps = {{"pitch", &w.pitch}, {"pitch_rate", &w.pitch_rate}, {"lean", &w.lean},
                         {"pos", &w.pos}, {"vel", &w.vel}, {"pos_int", &w.pos_int},
                         {"hip", &w.hip}, {"knee", &w.knee}, {"hip_rate", &w.hip_rate},
                         {"knee_rate", &w.knee_rate}, {"wheel_cmd", &w.wheel_cmd},
                         {"wheel_step", &w.wheel_step}, {"hip_step", &w.hip_step},
                         {"knee_step", &w.knee_step}};
    ps.push_back({"body_vacc", &w.body_vacc});
    ps.push_back({"hip_cmd", &w.hip_cmd});
    ps.push_back({"knee_cmd", &w.knee_cmd});
    auto eval = [&]() { const auto t = makeWbTable(w); return scoreWb(t); };
    WbScore best = eval();
    std::printf("start score %.2f (falls %d) over %.3f after %.4f\n", best.total, best.falls,
                best.overshoot, best.body_after);
    for (double f : {4.0, 2.0}) {
      for (int round = 0; round < 2; ++round) {
        for (auto& p : ps) {
          const double v0 = *p.v;
          for (double m : {f, 1.0 / f}) {
            *p.v = v0 * m;
            const WbScore sc = eval();
            if (sc.total < best.total - 1e-6) {
              best = sc;
              std::printf("  %-10s x%-5.2f -> score %.2f falls %d held %d lean %.3f body %.3f az %.2f rest %.3f/%.3f drive %.3f over %.3f after %.4f\n",
                          p.name, m, sc.total, sc.falls, sc.held, sc.lean_push, sc.body_push, sc.az_bump,
                          sc.rest_rate, sc.rest_hip, sc.body_drive, sc.overshoot, sc.body_after);
              std::fflush(stdout);
              break;
            }
            *p.v = v0;
          }
        }
      }
    }
    std::printf("best weights (1/sqrt = acceptable size):\n");
    for (const auto& p : ps) std::printf("  %-10s %.6g  (%.4g)\n", p.name, *p.v, 1.0 / std::sqrt(*p.v));
    const auto t = makeWbTable(w);
    scoreWb(t, true);
    return 0;
  }
#endif
  if (std::getenv("TIER_STOP") != nullptr) {
    Case c = withMode(Case{}, 2);
    c.duration_s = 26.0;
    c.heights = {{1.0, 0.150}, {10.0, std::getenv("STOP_H") ? std::atof(std::getenv("STOP_H")) : 0.095}};
    c.stop_mode = std::getenv("STOP_MODE") ? std::atoi(std::getenv("STOP_MODE")) : 0;
    if (const char* v = std::getenv("STOP_BRAKE")) c.height_brake_s = std::atof(v);
    const int variant = std::getenv("STOP_VARIANT") ? std::atoi(std::getenv("STOP_VARIANT")) : 0;
    if (variant == 1 || variant == 2) {
      c.hip_backlash = 0.001; c.gear_k = 20.0; c.gear_k_knee = 150.0;
    }
    if (variant == 2 || variant == 3) c.wheel_delay_ticks += 10;
    if (variant == 4) c.hip_vel_noise = c.knee_vel_noise = 0.12;
    if (variant == 5) { c.hip_zero_err = 0.05; c.knee_zero_err = -0.04; }
    c.roll = 1;
    const Result r = simulate(c);
    std::printf("STOP h=%.0f mode=%d variant=%d fell=%d held=%d body=%.5f rest=%.5f fault=%s\n",
                c.heights.back().second*1000, c.stop_mode, variant, r.fell, r.held_resets,
                r.body_dev_all, r.rest_rate_rms, r.fault);
    return 0;  // This is measurement, not a passing assertion.
  }
  if (std::getenv("TIER_ZERO") != nullptr) {
    // Legs arranged off by hand: N-P aligned only with the body tilted.
    // Does the level loop straighten the body with the legs?
    for (double hz : {-0.10, -0.05, 0.05, 0.10}) {
      for (double kz : {0.0, -0.08}) {
        for (int lvl = 0; lvl < 2; ++lvl) {
          Case c = withMode(Case{}, 2);
          c.hip_zero_err = hz;
          c.knee_zero_err = kz;
          c.level = lvl;
          c.duration_s = 15.0;
          c.trace = std::getenv("ZR_TRACE") != nullptr;
          const Result r = simulate(c);
          std::printf("hip zero %+.2f knee zero %+.2f level %d: fell=%d body vs level at start %+.3f, "
                      "end rms %.4f | lean %.3f | hip offset %+.3f | body at target after %.1f s\n",
                      hz, kz, lvl, r.fell, r.start_dev, r.body_rms_end, r.lean_max, r.level_hip_end,
                      r.level_settle_s);
        }
      }
    }
    return 0;
  }
  if (std::getenv("TIER_SUS") != nullptr) {
    // Suspension gain / return sweep: bump vs standing.
    const double accn = std::getenv("SK_ACCN") ? std::atof(std::getenv("SK_ACCN")) : 1.0;
    for (double g : {0.0, 1.0, 2.0, 3.0}) {
      for (double ret : {0.3, 0.6, 1.2}) {
        if (g == 0.0 && ret != 0.6) continue;
        char gs[16], rs[16];
        std::snprintf(gs, sizeof gs, "%g", g);
        std::snprintf(rs, sizeof rs, "%g", ret);
        setenv("TR_SUSG", gs, 1);
        setenv("TR_SUSR", rs, 1);
        Case b = withMode(Case{}, 2);
        b.bump_h = 0.005; b.bump_x0 = 0.25; b.bump_width = 0.03;
        b.drive_m_s = 0.30; b.drive_end_s = 9.0; b.duration_s = 14; b.acc_noise_scale = accn;
        if (std::getenv("SK_OLDWHEEL")) { b.wheel_tau = 0.09; b.wheel_delay_ticks = 25; b.wheel_meas_delay_ticks = 0; }
        const Result rb = simulate(b);
        Case b8 = b;
        b8.bump_h = 0.008; b8.bump_ramp = 0.04; b8.drive_m_s = 0.15; b8.drive_end_s = 11.0; b8.duration_s = 16;
        const Result r8 = simulate(b8);
        Case st = withMode(Case{}, 2);
        st.acc_noise_scale = accn;
        const Result rs_ = simulate(st);
        Case pu = st;
        pu.push_n = 4.0;
        const Result rp = simulate(pu);
        std::printf("gain %.1f ret %.1f: bump5 bob %.1f mm vacc %.2f | bump8 bob %.1f vacc %.2f | stand bob %.1f mm "
                    "rate %.3f | push bob %.1f pitch %.3f | falls %d\n", g, ret, rb.body_z_range * 1000,
                    rb.body_vacc_peak, r8.body_z_range * 1000, r8.body_vacc_peak, rs_.body_z_range * 1000,
                    rs_.rest_rate_rms, rp.body_z_range * 1000, rp.body_dev_all,
                    rb.fell + r8.fell + rs_.fell + rp.fell);
      }
    }
    unsetenv("TR_SUSG");
    unsetenv("TR_SUSR");
    return 0;
  }
  if (std::getenv("TIER_SKATE") != nullptr) {
    // Where does the body still move (default tiers)? Pitch, pitch rate,
    // vertical bob and vertical acceleration per scenario, and what the
    // standing jitter comes from (sensor noise vs dynamics).
    struct Sc { const char* name; void (*apply)(Case&); };
    const Sc scs[] = {
        {"stand", [](Case&) {}},
        {"stand ideal IMU+joints", [](Case& c) { c.ideal_imu = true; c.hip_vel_noise = 0.0; }},
        {"stand 136", [](Case& c) { c.height_low_m = 0.136; c.height_low_s = 3; c.height_up_s = 60; c.duration_s = 20; }},
        {"stand 90", [](Case& c) { c.height_low_m = 0.090; c.height_low_s = 3; c.height_up_s = 60; c.duration_s = 20; }},
        {"push 4 N", [](Case& c) { c.push_n = 4.0; }},
        {"push 4 N at 136", [](Case& c) { c.height_low_m = 0.136; c.height_low_s = 3; c.height_up_s = 60;
                                          c.push_n = 4.0; c.push_start_s = 12; c.push_end_s = 12.3; c.duration_s = 20; }},
        {"drive 0.55", [](Case& c) { c.drive_m_s = 0.55; }},
        {"bump 5mm 0.30", [](Case& c) { c.bump_h = 0.005; c.bump_x0 = 0.25; c.bump_width = 0.03;
                                        c.drive_m_s = 0.30; c.drive_end_s = 9.0; c.duration_s = 14; }},
        {"bump 5mm soft knee", [](Case& c) { c.bump_h = 0.005; c.bump_x0 = 0.25; c.bump_width = 0.03;
                                             c.drive_m_s = 0.30; c.drive_end_s = 9.0; c.duration_s = 14;
                                             c.knee_kp = 6.0; c.knee_vel_bw = 120.0; }},
        {"stand soft knee", [](Case& c) { c.knee_kp = 6.0; c.knee_vel_bw = 120.0; }},
        {"push 4 N soft knee", [](Case& c) { c.push_n = 4.0; c.knee_kp = 6.0; c.knee_vel_bw = 120.0; }},
    };
    const double accn = std::getenv("SK_ACCN") ? std::atof(std::getenv("SK_ACCN")) : 1.0;
    for (const auto& sc : scs) {
      for (int sus = 0; sus < 2; ++sus) {
        Case c = withMode(Case{}, 2);
        sc.apply(c);
        c.suspension = sus;
        c.acc_noise_scale = accn;
        const Result r = simulate(c);
        std::printf("%-22s sus=%d fell=%d pitch max %.3f end %.4f | rate rest %.3f | hip %.3f | "
                    "bob %.1f mm | vacc rms %.2f peak %.2f | lean %.3f | knee err %.3f\n",
                    sc.name, sus, r.fell, r.body_dev_all, r.body_rms_end, r.rest_rate_rms,
                    r.rest_hip_rms, r.body_z_range * 1000, r.body_vacc_rms, r.body_vacc_peak,
                    r.lean_max, r.max_knee_err);
      }
    }
    return 0;
  }
  if (std::getenv("TIER_WB") != nullptr) {
    // Whole-body LQR (Algo::WholeBody) next to the default tiers.
    struct Sc { const char* name; void (*apply)(Case&); };
    const Sc scs[] = {
        {"stand", [](Case&) {}},
        {"drive +0.55", [](Case& c) { c.drive_m_s = 0.55; }},
        {"drive -0.55", [](Case& c) { c.drive_m_s = -0.55; }},
        {"push 4 N", [](Case& c) { c.push_n = 4.0; }},
        {"push -4 N", [](Case& c) { c.push_n = -4.0; }},
        {"push 6N x0.3", [](Case& c) { c.push_n = 6.0; }},
        {"136 push 4Nx1s", [](Case& c) { c.height_low_m = 0.136; c.height_low_s = 3; c.height_up_s = 60;
                                        c.push_n = 4.0; c.push_start_s = 12; c.push_end_s = 13; c.duration_s = 20; }},
        {"height 136", [](Case& c) { c.height_low_m = 0.136; c.duration_s = 30; c.height_up_s = 17; }},
        {"height 78", [](Case& c) { c.height_low_m = 0.078; c.duration_s = 30; c.height_up_s = 17; }},
        {"stop .75 tau.12", [](Case& c) { c.wheel_tau = 0.12; c.drive_m_s = 0.75; }},
        {"stop .75", [](Case& c) { c.drive_m_s = 0.75; }},
        {"stop .75 tau.06", [](Case& c) { c.wheel_tau = 0.06; c.drive_m_s = 0.75; }},
        {"stop .65 tau.06", [](Case& c) { c.wheel_tau = 0.06; c.drive_m_s = 0.65; }},
        {"stand flow 8 s", [](Case& c) { c.held_s = 8.0; }},
        {"bump 5mm .30", [](Case& c) { c.bump_h = 0.005; c.bump_x0 = 0.25; c.bump_width = 0.03;
                                      c.drive_m_s = 0.30; c.drive_end_s = 9.0; c.duration_s = 14; }},
        {"bump 8mm/4cm .15", [](Case& c) { c.bump_h = 0.008; c.bump_ramp = 0.04; c.bump_x0 = 0.25;
                                          c.bump_width = 0.03; c.drive_m_s = 0.15; c.drive_end_s = 11.0; c.duration_s = 16; }},
    };
    for (const auto& sc : scs) {
      for (int wbm = 0; wbm < 2; ++wbm) {
        Case c = withMode(Case{}, 2);
        if (wbm) c.algo = V3Controller::Algo::WholeBody;
        sc.apply(c);
        const Result r = simulate(c);
        std::printf("%-17s %-5s fell=%d%s held=%d body=%.3f lean=%.3f rest=%.3f/%.3f az=%.2f "
                    "x_end=%+.3f wcmd=%.1f tau_k=%.2f dhip=%.3f\n", sc.name, wbm ? "WB" : "tiers",
                    r.fell, r.fell ? r.fault : "", r.held_resets, r.body_dev_all, r.lean_max,
                    r.rest_rate_rms, r.rest_hip_rms, r.body_vacc_peak, r.x_end, r.max_wheel_cmd,
                    r.max_tau_knee, r.max_dhip);
      }
    }
    return 0;
  }
  if (std::getenv("TIER_PUSHSWEEP") != nullptr) {
    // Hard pushes (wheels saturated, hardware 2026-09-30 182 s: 1.1 m/s, held
    // re-zero mid-catch) and hand pushes (force held on for up to 1 s).
    const double cap = std::getenv("PS_CAP") ? std::atof(std::getenv("PS_CAP")) : -1.0;
    const bool norz = std::getenv("PS_NOREZERO") != nullptr;
    const bool wbm = std::getenv("PS_WB") != nullptr;
    for (double h : {0.0, 0.136}) {
      for (int mode : {0, 2}) {
        for (double len : {0.3, 0.6, 1.0}) {
          std::printf("h %s %-13s %.1f s:", h > 0 ? "136" : "189", modeName(mode), len);
          for (double f : {4.0, 6.0, 8.0, 10.0, 12.0, 14.0}) {
            Case c = withMode(Case{}, mode);
            c.held_cap = cap;
            c.no_rezero = norz;
            if (wbm) c.algo = V3Controller::Algo::WholeBody;
            if (h > 0) { c.height_low_m = h; c.height_low_s = 3.0; c.height_up_s = 60.0; }
            c.push_n = f; c.push_start_s = 12.0; c.push_end_s = 12.0 + len; c.duration_s = 22.0;
            const Result r = simulate(c);
            std::printf("  %2.0fN:%s h%d v%.2f", f, r.fell ? (std::strstr(r.fault, "ringing") ? "RING" : "FALL") : "ok  ",
                        r.held_resets, r.max_speed);
          }
          std::printf("\n");
        }
      }
    }
    return 0;
  }
#ifdef TAP_HAVE_EIGEN
  if (std::getenv("TIER_WBKNEE") != nullptr) {
    // How much the knee joins the hip in a push under Algo::WholeBody, per
    // knee cost (acceptable knee offset / step) - body, legs, CoM bob, and
    // the robustness variants at the crouch.
    struct W { double knee, knee_step, knee_rate; };
    std::vector<W> ws = {{0.10, 0.04, 4.0}, {0.30, 0.08, 6.0}};
    if (std::getenv("WK_ONE")) ws.resize(1);
    for (const W& wk : ws) {
      tap_fw::sim::WbWeights w;
      w.knee = 1.0 / (wk.knee * wk.knee);
      w.knee_step = 1.0 / (wk.knee_step * wk.knee_step);
      w.knee_rate = 1.0 / (wk.knee_rate * wk.knee_rate);
      if (const char* v = std::getenv("WK_VACC")) w.body_vacc = 1.0 / std::pow(std::atof(v), 2);
      if (const char* v = std::getenv("WK_Z")) w.body_z = 1.0 / std::pow(std::atof(v), 2);
      const auto t = makeWbTable(w);
      for (double h : {0.0, 0.136, 0.0784}) {
        int bad = 0; double wbody = 0, wrest = 0;
        Result nom;
        for (int vi = 0; vi < 17; ++vi) {
          if (vi > 0 && std::getenv("WK_NOVAR")) break;
          for (double f : {4.0, -4.0}) {
            Case c = withMode(Case{}, 2);
            if (vi > 0) kVariants[vi].apply(c);
            c.algo = V3Controller::Algo::WholeBody;
            c.wb_table = &t;
            if (h > 0) { c.height_low_m = h; c.height_low_s = 3.0; c.height_up_s = 60.0; }
            c.push_n = f; c.push_start_s = 12.0; c.push_end_s = 12.5; c.duration_s = 20.0;
            const Result r = simulate(c);
            if (r.fell || r.held_resets) ++bad;
            wbody = std::max(wbody, r.push_body); wrest = std::max(wrest, r.rest_rate_rms);
            if (vi == 0 && f > 0) nom = r;
          }
        }
        std::printf("knee %.2f step %.2f rate %.0f | h %3.0f: body %.3f lean %.3f hip %.3f knee %.3f dz %.1f mm "
                    "az %.2f tau_k %.2f | variants bad %d worst body %.3f rest %.3f\n",
                    wk.knee, wk.knee_step, wk.knee_rate, h > 0 ? h * 1000 : 189.0, nom.push_body,
                    nom.push_lean, nom.push_hip, nom.push_knee, nom.push_dz * 1000, nom.body_vacc_peak,
                    nom.max_tau_knee, bad, wbody, wrest);
        std::fflush(stdout);
      }
    }
    return 0;
  }
#endif
  if (std::getenv("TIER_WPGRID") != nullptr) {
    // Whole-body wheel predictor model (TR_WPTAU / TR_WPDELAY / TR_WPMEAS
    // from the environment) on every wheel variant: hard stop, 6 N push and
    // 84 mm crouched cruise, plus the V26 random episodes.
    const int wv[] = {0, 1, 2, 3, 12};
    int falls = 0; double worst = 0, sum = 0; int n = 0;
    auto take = [&](const Result& r, const char* what, const char* var) {
      if (r.fell) { ++falls; std::printf("  FALL %s / %s: %s\n", what, var, r.fault); }
      const double b = r.fell ? 1.0 : r.body_dev_all;
      worst = std::max(worst, b); sum += b; ++n;
    };
    for (int vi : wv) {
      for (int sc = 0; sc < 4; ++sc) {
        Case c = withMode(Case{}, 2);
        c.algo = V3Controller::Algo::WholeBody;
        kVariants[vi].apply(c);
        if (sc == 0) { c.drive_m_s = 0.75; }
        if (sc == 1) { c.drive_m_s = 0.75; c.wheel_tau = std::max(c.wheel_tau, 0.12); }
        if (sc == 2) { c.push_n = 6.0; }
        if (sc == 3) { c.heights.push_back({1.0, 0.084}); c.drive_m_s = 0.75; c.drive_end_s = 12.0; c.duration_s = 18.0; }
        const char* scn[] = {"stop .75", "stop .75 lag>=.12", "push 6N", "84mm cruise .75"};
        take(simulate(c), scn[sc], kVariants[vi].name);
      }
    }
    for (int seed = 1; seed <= 16; ++seed) {
      int vi = 0;
      take(simulate(randomEpisode(seed, true, &vi)), "episode", kVariants[vi].name);
    }
    std::printf("WPGRID tau %s delay %s meas %s: falls %d worst body %.3f mean %.4f\n",
                std::getenv("TR_WPTAU") ? std::getenv("TR_WPTAU") : "def",
                std::getenv("TR_WPDELAY") ? std::getenv("TR_WPDELAY") : "def",
                std::getenv("TR_WPMEAS") ? std::getenv("TR_WPMEAS") : "def", falls, worst, sum / n);
    return 0;
  }
  if (std::getenv("TIER_LEADSWEEP") != nullptr) {
    // Tiers' wheel lead (Config::lqr_lead_s) on the measured wheel and its
    // variants: stand + 4 N push at 189 / 110 mm.
    for (double lead : {0.04, 0.03, 0.025, 0.02, 0.01, 0.0}) {
      int falls = 0; double worst = 0, sum = 0; int n = 0;
      for (int vi : {0, 1, 2, 3, 12}) {
        for (double h : {0.0, 0.110}) {
          Case c = withMode(Case{}, 2);
          if (vi > 0) kVariants[vi].apply(c);
          if (h > 0) c.heights.push_back({1.0, h});
          c.push_n = 4.0; c.push_start_s = 10.0; c.push_end_s = 10.3; c.duration_s = 18.0;
          c.lqr_lead = lead;
          const Result r = simulate(c);
          if (r.fell) ++falls;
          worst = std::max(worst, r.fell ? 9.0 : r.rest_rate_rms); sum += r.rest_rate_rms; ++n;
        }
      }
      std::printf("lead %.2f: falls %d worst rest %.3f mean rest %.3f\n", lead, falls, worst, sum / n);
      std::fflush(stdout);
    }
    return 0;
  }
  if (std::getenv("TIER_WHEELFIT") != nullptr) {
    // WB against wheel models: the design one and faster ones (the robot's
    // wheel, measured at the 2-4 Hz ringing: ~40 ms, gain ~1).
    struct W { int delay; double tau; double db; const char* name; int meas = 0; };
    const W ws[] = {{25, 0.09, 1.0, "design 50ms+0.09"}, {37, 0.035, 1.0, "act 75 meas 0", 0},
                    {25, 0.035, 1.0, "act 50 meas 25", 12}, {20, 0.035, 1.0, "act 40 meas 35", 17},
                    {15, 0.035, 1.0, "act 30 meas 45", 22}, {10, 0.035, 1.0, "act 20 meas 55", 27}};
    for (const auto& w : ws) {
      for (double h : {0.0, 0.110}) {
        Case c = withMode(Case{}, 2);
        c.algo = V3Controller::Algo::WholeBody;
        c.wheel_delay_ticks = static_cast<std::size_t>(w.delay); c.wheel_tau = w.tau; c.deadband = w.db;
        c.wheel_meas_delay_ticks = static_cast<std::size_t>(w.meas);
        if (h > 0) c.heights.push_back({1.0, h});
        c.push_n = 3.0; c.push_start_s = 10.0; c.push_end_s = 10.3; c.duration_s = 18.0;
        if (const char* alg = std::getenv("WF_ALGO")) c.algo = static_cast<V3Controller::Algo>(std::atoi(alg));
        const Result r = simulate(c);
        std::printf("%-18s h %3.0f: fell=%d%s held=%d rest rate %.3f body %.3f wcmd max %.1f\n", w.name,
                    h > 0 ? h * 1000 : 189.0, r.fell, r.fell ? r.fault : "", r.held_resets, r.rest_rate_rms,
                    r.body_dev_all, r.max_wheel_cmd);
        std::fflush(stdout);
      }
    }
    return 0;
  }
  if (std::getenv("TIER_PROF") != nullptr) {
    // Host cost of V3Controller::update (relative, not the ESP32's) and of
    // the leg-model calls it makes.
    auto bench = [](const char* name, auto&& f) {
      const int n = 20000;
      volatile float sink = 0.0F;
      const auto t0 = std::chrono::steady_clock::now();
      for (int i = 0; i < n; ++i) sink = sink + f(i);
      const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / n;
      std::printf("  %-34s %7.3f us\n", name, us);
    };
    bench("LegHeight::forKnee", [](int i) { return LegHeight::forKnee(-0.001F * float(i % 1000)).hip_offset_rad; });
    bench("LegHeight::maxHeight", [](int) { return LegHeight::maxHeight(); });
    bench("LegHeight::forHeight", [](int i) { return LegHeight::forHeight(0.08F + 0.0001F * float(i % 1000)).knee_offset_rad; });
    bench("WholeBodyModel::legHeight", [](int i) {
      return robot_control::WholeBodyModel::legHeight(0.4F + 0.0001F * float(i % 1000), -0.5F).height_m; });
    bench("WholeBodyModel::compute", [](int i) {
      const float h = 0.4F + 0.0001F * float(i % 1000);
      return robot_control::WholeBodyModel{}.compute(h, -0.5F, h, -0.5F).com_above_axle_m; });
    struct Sc { const char* name; double h; int algo; int roll; double drive; };
    const Sc scs[] = {{"tiers 189", 0.0, 0, 0, 0.0}, {"tiers 84", 0.084, 0, 0, 0.0}, {"tiers 84 drive", 0.084, 0, 0, 0.3},
                      {"tiers 84 roll", 0.084, 0, 1, 0.3}, {"WB 189", 0.0, 2, 0, 0.0}, {"WB 84", 0.084, 2, 0, 0.0},
                      {"WB 84 drive roll", 0.084, 2, 1, 0.3}};
    for (const auto& sc : scs) {
      Case c = withMode(Case{}, 2);
      c.algo = static_cast<V3Controller::Algo>(sc.algo);
      c.roll = sc.roll;
      if (sc.h > 0) c.heights.push_back({1.0, sc.h});
      c.drive_m_s = sc.drive; c.drive_start_s = 12; c.drive_end_s = 18; c.duration_s = 24;
      const Result r = simulate(c);
      std::printf("  update %-18s mean %6.2f us max %7.2f us\n", sc.name, r.update_us_sum / std::max(r.update_n, 1L),
                  r.update_us_max);
    }
    return 0;
  }
  if (std::getenv("TIER_ROLLSTEP") != nullptr) {
    // Roll reaction time: the left wheel climbs a 10 mm step (2 cm ramp at
    // 0.15 m/s: ~0.13 s), stays up; and a 1.5 rad/s turn at 0.55 m/s, 136 mm.
    for (int sc = 0; sc < 2; ++sc) {
      Case c = withMode(Case{}, 2);
      c.algo = V3Controller::Algo::WholeBody;
      c.roll = 1;
      if (sc == 0) {
        c.bump_h = 0.010; c.bump_ramp = 0.02; c.bump_x0 = 0.25; c.bump_left_only = true;
        c.drive_m_s = 0.15; c.drive_end_s = 9.0; c.duration_s = 14.0;
      } else {
        c.heights.push_back({1.0, 0.136});
        c.drive_m_s = 0.55; c.drive_start_s = 10; c.drive_end_s = 18; c.yaw_rad_s = 1.5;
        c.yaw_t0 = 13; c.yaw_t1 = 16; c.duration_s = 22.0;
      }
      const Result r = simulate(c);
      std::printf("%s: fell=%d peak %.3f | over 0.005 from %.2f to %.2f s (%.2f s) | under half peak at +%.2f s | dh max %.1f mm\n",
                  sc ? "turn 0.55/1.5 " : "left step 10mm", r.fell, r.roll_app_max, r.roll_t_first, r.roll_t_last,
                  r.roll_t_last - r.roll_t_first, r.roll_t_half - r.roll_t_first, r.roll_dh_max * 1000);
      std::fflush(stdout);
    }
    return 0;
  }
  if (std::getenv("TIER_ROLL2") != nullptr) {
    runLayeredRoll(true);
    return 0;
  }
  if (std::getenv("TIER_ROLL") != nullptr) {
    // Roll (Config::Roll): off vs on, tiers and whole-body.
    struct Sc { const char* name; void (*apply)(Case&); };
    const Sc scs[] = {
        {"cross slope +0.06", [](Case& c) { c.cross_slope = 0.06; }},
        {"legs uneven 8 mm", [](Case& c) { c.leg_side_err_m = 0.008; }},
        {"turn 0.4 m/s 1.5 rad/s", [](Case& c) { c.drive_m_s = 0.4; c.drive_start_s = 6; c.drive_end_s = 14;
                                                  c.yaw_rad_s = 1.5; c.yaw_t0 = 8; c.yaw_t1 = 12; }},
        {"turn 0.55 m/s 2.0 rad/s", [](Case& c) { c.drive_m_s = 0.55; c.drive_start_s = 6; c.drive_end_s = 14;
                                                   c.yaw_rad_s = 2.0; c.yaw_t0 = 9; c.yaw_t1 = 12; }},
        {"110 mm turn 0.55/2.0", [](Case& c) { c.heights.push_back({1.0, 0.110}); c.drive_m_s = 0.55;
                                                c.drive_start_s = 10; c.drive_end_s = 18; c.yaw_rad_s = 2.0;
                                                c.yaw_t0 = 13; c.yaw_t1 = 16; c.duration_s = 26; }},
        {"left wheel onto 10 mm mat", [](Case& c) { c.bump_h = 0.010; c.bump_ramp = 0.05; c.bump_x0 = 0.25;
                                                     c.bump_width = 0.0; c.bump_left_only = true; c.drive_m_s = 0.2;
                                                     c.drive_end_s = 9.0; c.duration_s = 16; }},
        {"left bump 8 mm 0.3", [](Case& c) { c.bump_h = 0.008; c.bump_ramp = 0.04; c.bump_x0 = 0.25;
                                              c.bump_width = 0.05; c.bump_left_only = true; c.drive_m_s = 0.3;
                                              c.drive_end_s = 9.0; c.duration_s = 14; }},
    };
    for (int wbm = 0; wbm < 2; ++wbm) {
      for (const auto& sc : scs) {
        for (int on = 0; on < 2; ++on) {
          Case c = withMode(Case{}, 2);
          if (wbm) c.algo = V3Controller::Algo::WholeBody;
          c.duration_s = 20.0;
          c.roll = on;
          sc.apply(c);
          if (const char* v = std::getenv("RL_VAR")) kVariants[std::atoi(v)].apply(c);
          const Result r = simulate(c);
          std::printf("%-5s %-24s roll %-3s fell=%d held=%d | apparent max %.3f end %+.4f | vs gravity max %.3f | "
                      "wheel load min %.2f | side dh max %4.1f mm | body %.3f\n",
                      wbm ? "WB" : "tiers", sc.name, on ? "on" : "off", r.fell, r.held_resets,
                      r.roll_app_max, r.roll_app_end, r.roll_world_max, r.wheel_load_min,
                      r.roll_dh_max * 1000, r.body_dev_all);
          std::fflush(stdout);
        }
      }
    }
    return 0;
  }
  if (std::getenv("TIER_LEVELREC") != nullptr) {
    // Back to level: how fast and how cleanly the body returns to parallel
    // after what knocks it off - model error, hand-arranged legs, a tilted
    // put-down, a push, a height change - tiers vs whole-body.
    struct Sc { const char* name; void (*apply)(Case&); double t_event; };
    const Sc scs[] = {
        {"CoM +15 mm", [](Case& c) { c.body_dx = 0.036; }, 0.0},
        {"CoM -15 mm", [](Case& c) { c.body_dx = 0.006; }, 0.0},
        {"legs hip+.10 knee-.08", [](Case& c) { c.hip_zero_err = 0.10; c.knee_zero_err = -0.08; }, 0.0},
        {"legs hip-.10", [](Case& c) { c.hip_zero_err = -0.10; }, 0.0},
        {"seed 0.08", [](Case& c) { c.seed_error = 0.08; }, 0.0},
        {"push 4 N", [](Case& c) { c.push_n = 4.0; c.push_start_s = 8.0; c.push_end_s = 8.3; }, 8.0},
        {"to 110 mm", [](Case& c) { c.heights.push_back({6.0, 0.110}); }, 6.0},
        {"110 push 4 N", [](Case& c) { c.heights.push_back({1.0, 0.110}); c.push_n = 4.0;
                                       c.push_start_s = 12.0; c.push_end_s = 12.3; }, 12.0},
        {"slope +0.05", [](Case& c) { c.slope = 0.05; }, 0.0},
    };
    for (int wbm = 0; wbm < 2; ++wbm) {
      for (const auto& sc : scs) {
        Case c = withMode(Case{}, 2);
        if (wbm) c.algo = V3Controller::Algo::WholeBody;
        c.duration_s = 24.0;
        sc.apply(c);
        if (const char* v = std::getenv("LR_VAR")) kVariants[std::atoi(v)].apply(c);
        const Result r = simulate(c);
        std::printf("%-5s %-22s fell=%d held=%d start %+.3f peak %.3f | |dev|>0.01 until %5.1f s (%4.1f s after) | end mean %+.4f rms %.4f | max hip rate %.2f\n",
                    wbm ? "WB" : "tiers", sc.name, r.fell, r.held_resets, r.start_dev, r.body_dev_all,
                    r.settle01_s, r.settle01_s - std::max(sc.t_event, 2.0), r.body_mean_end, r.body_rms_end,
                    r.max_hip_rate);
        std::fflush(stdout);
      }
    }
    return 0;
  }
  if (std::getenv("TIER_ANIM") != nullptr) {
    // One scripted run for tools/tier_sim_view.py (frames to TR_ANIM):
    // crouch to ANIM_H, speed up / brake forward, a push standing, then
    // backward. ANIM_ALGO=2: whole-body LQR.
    Case c = withMode(Case{}, 2);
    if (const char* a = std::getenv("ANIM_ALGO")) c.algo = static_cast<V3Controller::Algo>(std::atoi(a));
    const double h = std::getenv("ANIM_H") ? std::atof(std::getenv("ANIM_H")) : 0.084;
    if (h < 0.188) c.heights.push_back({1.0, h});
    const double v = std::getenv("ANIM_V") ? std::atof(std::getenv("ANIM_V")) : 0.55;
    // The crouch to 84 mm takes ~10 s; the maneuvers start after it.
    c.drives.push_back({13.0, 17.0, v});
    c.pushes.push_back({20.0, 20.3, std::getenv("ANIM_PUSH") ? std::atof(std::getenv("ANIM_PUSH")) : 4.0});
    c.drives.push_back({23.0, 27.0, -v});
    c.duration_s = 31.0;
    if (const char* b = std::getenv("ANIM_BUMP")) {  // a floor bump on the forward run
      c.bump_h = std::atof(b); c.bump_x0 = 0.9; c.bump_width = 0.03; c.bump_ramp = 0.02;
    }
    if (const char* v2 = std::getenv("ANIM_SUS")) c.suspension = std::atoi(v2);
    if (const char* rs = std::getenv("ANIM_ROUGH")) {  // rough road on the forward run only
      c.rough_seed = std::atoi(rs); c.rough_len_m = 4.0;
      c.rough_dense = std::getenv("ANIM_DENSE") != nullptr;
      if (const char* av = std::getenv("ANIM_VAR")) kVariants[std::atoi(av)].apply(c);
      if (const char* sc2 = std::getenv("ANIM_SCALE")) c.rough_scale = std::atof(sc2);
      c.drives.clear(); c.pushes.clear();
      c.drives.push_back({13.0, 13.0 + 4.5 / v, v});
      c.duration_s = 13.0 + 4.5 / v + 4.0;
    }
    if (const char* pf = std::getenv("ANIM_PRESS")) {
      // Hand on the body instead of driving: press the nose down 1 s, the
      // tail down 1 s, then quick pokes on the nose and on the tail.
      c.drives.clear();
      c.pushes.clear();
      addPresses(c, std::atof(pf), std::getenv("ANIM_HOLD") ? std::atof(std::getenv("ANIM_HOLD")) : 0.0);
      c.duration_s = 31.0;
    }
    const Result r = simulate(c);
    std::printf("anim h %.0f mm: fell=%d%s body max %.3f lean max %.3f dhip max %.3f\n", h * 1000, r.fell,
                r.fell ? r.fault : "", r.body_dev_all, r.lean_max, r.max_dhip);
    return 0;
  }
  if (std::getenv("TIER_PRESS") != nullptr) {
    // Hand presses on the nose / tail (addPresses) at each height on the
    // wheel variants: body tilt, how far the robot rolls, rest wobble.
    const double f = std::getenv("PRESS_F") ? std::atof(std::getenv("PRESS_F")) : 8.0;
    const double hold = std::getenv("PRESS_HOLD") ? std::atof(std::getenv("PRESS_HOLD")) : 0.0;
    for (double h : {0.189, 0.136, 0.110, 0.084}) {
      int falls = 0, held = 0, n = 0;
      double worst = 0, sum = 0, dhip = 0, xr = 0, rest = 0;
      for (int vi : {0, 1, 2, 3, 12, 4, 6}) {
        Case c = withMode(Case{}, 2);
        kVariants[vi].apply(c);
        if (h < 0.188) c.heights.push_back({1.0, h});
        addPresses(c, f, hold);
        c.duration_s = 31.0;
        const Result r = simulate(c);
        if (r.fell) { ++falls; std::printf("  FALL h %.0f %s: %s\n", h * 1000, kVariants[vi].name, r.fault); }
        if (std::getenv("PRESS_V")) {
          std::printf("  h %.0f %-18s body %.3f dhip %.3f max|x| %.3f rest %.3f held %d\n", h * 1000,
                      kVariants[vi].name, r.body_dev_all, r.max_dhip, r.max_x, r.rest_rate_rms, r.held_resets);
        }
        held += r.held_resets;
        worst = std::max(worst, r.fell ? 1.0 : r.body_dev_all); sum += r.body_dev_all; ++n;
        dhip = std::max(dhip, r.max_dhip); xr = std::max(xr, r.max_x); rest = std::max(rest, r.rest_rate_rms);
      }
      std::printf("PRESS %.0f N hold %.1f h %3.0f: falls %d held %d | body worst %.3f mean %.3f | dhip %.3f | roll-away %.3f m | rest %.3f\n",
                  f, hold, h * 1000, falls, held, worst, sum / n, dhip, xr, rest);
      std::fflush(stdout);
    }
    return 0;
  }
  if (std::getenv("TIER_DIAG") != nullptr) {
    // One push (or none) for tools/tier_push_metrics.py: the CSV goes to
    // TR_ANIM. DIAG_H height [m, 0 = full], DIAG_F force [N] at 14 s for
    // DIAG_LEN [s], DIAG_VAR plant variant, DIAG_DUR run length [s].
    Case c = withMode(Case{}, 2);
    const double h = std::getenv("DIAG_H") ? std::atof(std::getenv("DIAG_H")) : 0.0;
    if (h > 0.0) c.heights.push_back({1.0, h});
    const double f = std::getenv("DIAG_F") ? std::atof(std::getenv("DIAG_F")) : 0.0;
    const double len = std::getenv("DIAG_LEN") ? std::atof(std::getenv("DIAG_LEN")) : 0.3;
    c.push_n = f; c.push_start_s = 14.0; c.push_end_s = 14.0 + len;
    c.duration_s = std::getenv("DIAG_DUR") ? std::atof(std::getenv("DIAG_DUR")) : 40.0;
    if (const char* v = std::getenv("DIAG_VAR")) kVariants[std::atoi(v)].apply(c);
    if (const char* v = std::getenv("DIAG_LVL")) c.level = std::atoi(v);
    if (const char* v = std::getenv("DIAG_HELD")) c.held_s = std::atof(v);
    if (const char* v = std::getenv("DIAG_GRAB")) c.grab_s = std::atof(v);
    if (const char* v = std::getenv("DIAG_GRAB_BIAS")) c.grab_release_bias_rad = std::atof(v);
    if (std::getenv("DIAG_NOLEARN")) c.learn_hold = 1000.0;
    if (const char* v = std::getenv("DIAG_SEED")) c.seed_error = std::atof(v);
    if (const char* v = std::getenv("DIAG_HVN")) c.hip_vel_noise = std::atof(v);
    if (const char* v = std::getenv("DIAG_KVN")) c.knee_vel_noise = std::atof(v);
    if (const char* v = std::getenv("DIAG_CTRL_EVERY")) c.ctrl_every = std::atoi(v);
    if (const char* v = std::getenv("DIAG_GYRN")) c.gyro_noise = std::atof(v);
    if (const char* v = std::getenv("DIAG_ACCN")) c.acc_noise_scale = std::atof(v);
    if (const char* v = std::getenv("DIAG_DRIVE")) {
      c.drive_m_s = std::atof(v); c.drive_start_s = 12.0; c.drive_end_s = 16.0;
    }
    if (const char* v = std::getenv("DIAG_IDEALIMU")) c.ideal_imu = std::atoi(v) != 0;
    if (const char* v = std::getenv("DIAG_WNOISE")) c.wheel_noise = std::atof(v);
    if (const char* v = std::getenv("DIAG_WDELAY")) c.wheel_delay_ticks = static_cast<std::size_t>(std::atoi(v));
    if (const char* v = std::getenv("DIAG_WMEAS")) c.wheel_meas_delay_ticks = static_cast<std::size_t>(std::atoi(v));
    if (const char* v = std::getenv("DIAG_WTAU")) c.wheel_tau = std::atof(v);
    if (const char* v = std::getenv("DIAG_WTQ")) c.wheel_tau_max = std::atof(v);
    if (const char* v = std::getenv("DIAG_BLH")) c.hip_backlash = std::atof(v);
    if (const char* v = std::getenv("DIAG_BLK")) c.knee_backlash = std::atof(v);
    if (const char* v = std::getenv("DIAG_GK")) c.gear_k = std::atof(v);
    if (const char* v = std::getenv("DIAG_GZ")) c.gear_zeta = std::atof(v);
    if (const char* v = std::getenv("DIAG_GKK")) c.gear_k_knee = std::atof(v);
    if (const char* v = std::getenv("DIAG_WDB")) c.deadband = std::atof(v);
    const Result r = simulate(c);
    std::printf("DIAG fell=%d%s held=%d body_all=%.4f est_max=%.4f est_rms=%.4f body_drive=%.4f\n", r.fell,
                r.fell ? r.fault : "", r.held_resets, r.body_dev_all, r.est_err_max, r.est_err_rms,
                r.body_dev_drive);
    return 0;
  }
  if (std::getenv("TIER_ACCEL") != nullptr) {
    // Commanded speed-up / braking at each height on the wheel variants
    // (TR_PLANFULL / TR_KHOLD from the environment): does the body stay
    // level while the legs take the lean, and does it lose height?
    std::vector<double> hs = {0.189, 0.136, 0.110, 0.084};
    if (const char* hv = std::getenv("ACC_H")) hs = {std::atof(hv)};
    for (double h : hs) {
      int falls = 0, held = 0, n = 0;
      double worst = 0, sum = 0, dhip = 0, dz = 0, rest = 0, lean = 0;
      for (int vi : {0, 1, 2, 3, 12}) {
        for (double v : {0.55, -0.55}) {
          for (int push = 0; push < 2; ++push) {
            Case c = withMode(Case{}, 2);
            kVariants[vi].apply(c);
            if (h < 0.188) c.heights.push_back({1.0, h});
            c.drive_m_s = v; c.drive_start_s = 12.0; c.drive_end_s = 16.0; c.duration_s = 24.0;
            if (push) c.pushes.push_back({14.0, 14.3, v > 0 ? -3.0 : 3.0});
            const Result r = simulate(c);
            if (r.fell) { ++falls; std::printf("  FALL h %.0f %s v %+.2f push %d: %s\n", h * 1000, kVariants[vi].name, v, push, r.fault); }
            held += r.held_resets;
            if (std::getenv("ACC_V")) {
              std::printf("  h %.0f %-18s v %+.2f push %d: body %.3f lean %.3f dhip %.3f stop %+.2f rest %.3f\n",
                          h * 1000, kVariants[vi].name, v, push, r.body_dev_drive, r.lean_max, r.max_dhip,
                          r.x_stop_travel, r.rest_rate_rms);
            }
            worst = std::max(worst, r.fell ? 1.0 : r.body_dev_drive); sum += r.body_dev_drive; ++n;
            dhip = std::max(dhip, r.max_dhip); dz = std::max(dz, std::abs(r.drive_dz));
            rest = std::max(rest, r.rest_rate_rms); lean = std::max(lean, r.lean_max);
          }
        }
      }
      std::printf("ACCEL h %3.0f: falls %d held %d | body drive worst %.3f mean %.3f | lean %.3f | dhip %.3f | dz %.1f mm | rest %.3f\n",
                  h * 1000, falls, held, worst, sum / n, lean, dhip, dz * 1000, rest);
      std::fflush(stdout);
    }
    return 0;
  }
  if (std::getenv("TIER_SPEED") != nullptr) {
    // Crouch to drive fast (operator practice): lowered first, then a speed
    // step and a stop, optionally a push mid-cruise. Body pinned level?
    for (int wbm = 0; wbm < 2; ++wbm) {
      for (double h : {0.189, 0.136, 0.110, 0.084}) {
        for (double v : {0.55, 0.75}) {
          for (int push = 0; push < 2; ++push) {
            Case c = withMode(Case{}, 2);
            if (wbm) c.algo = V3Controller::Algo::WholeBody;
            if (const char* vs = std::getenv("SP_VAR")) kVariants[std::atoi(vs)].apply(c);
            if (std::getenv("SP_ENV")) c.envelope = 1;
            if (const char* a = std::getenv("SP_ACC")) c.plan_accel = std::atof(a);
            if (h < 0.188) c.heights.push_back({1.0, h});
            c.drive_m_s = v; c.drive_start_s = 12.0; c.drive_end_s = 18.0; c.duration_s = 26.0;
            if (push) c.pushes.push_back({15.0, 15.3, -3.0});
            const Result r = simulate(c);
            std::printf("%-5s h %3.0f v %.2f %s: fell=%d%s held=%d body_drive %.3f lean %.3f dz %.1f mm vmax %.2f stop %+.2f rest %.3f\n",
                        wbm ? "WB" : "tiers", h * 1000, v, push ? "push" : "    ", r.fell,
                        r.fell ? r.fault : "", r.held_resets, r.body_dev_drive, r.lean_max, r.drive_dz * 1000,
                        r.max_speed, r.x_stop_travel, r.rest_rate_rms);
            std::fflush(stdout);
          }
        }
      }
    }
    return 0;
  }
  if (const char* cfgname = std::getenv("TIER_GENERAL")) {
    // Generalisation check: random 30 s episodes nobody tuned for - a
    // height schedule (2-3 targets, changed while disturbed), 2-4 pushes of
    // random size/length/sign at random times, a drive segment, a floor
    // slope, and a random plant (variant + body CoM / mass / IMU error).
    //   TIER_GENERAL=tiers|wb  GEN_N=<episodes> GEN_SEED=<first seed>
    const std::string name = cfgname;
    const int n = std::getenv("GEN_N") ? std::atoi(std::getenv("GEN_N")) : 24;
    const int seed0 = std::getenv("GEN_SEED") ? std::atoi(std::getenv("GEN_SEED")) : 1;
    const int nv = static_cast<int>(sizeof(kVariants) / sizeof(kVariants[0]));
    int falls = 0, held = 0;
    std::vector<double> shares;
    double worst_body = 0, sum_rest = 0;
    for (int e = 0; e < n; ++e) {
      int vi = 0;
      Case c = randomEpisode(seed0 + e, name == "wb", &vi);
      if (std::getenv("GEN_TRACE")) c.trace = true;
#ifdef TAP_HAVE_EIGEN
      static V3Controller::WbTable gen_table;
      static bool gen_table_built = false;
      if (const char* wp = std::getenv("GEN_WPITCH")) {
        if (!gen_table_built) {
          tap_fw::sim::WbWeights w;
          if (std::getenv("GEN_R1")) {  // the first robust set (before the WB_GEN pass)
            w.pitch_rate = 0.25; w.lean = 25.0; w.vel = 1.0 / (0.15 * 0.15); w.hip = 0.6944;
            w.knee = 1600.0; w.knee_rate = 0.015625; w.knee_cmd = 2844.4; w.hip_step = 2500.0;
            w.knee_step = 2500.0;
          }
          w.pitch = std::atof(wp);
          if (const char* wh = std::getenv("GEN_WHIPSTEP")) w.hip_step = std::atof(wh);
          gen_table = makeWbTable(w);
          gen_table_built = true;
        }
        c.wb_table = &gen_table;
      }
#endif
      if (std::getenv("GEN_PRINT")) {
        std::printf("  ep %d spec: slope %+.3f dx %.3f mass %.2f |", seed0 + e, c.slope, c.body_dx, c.mass_scale);
        for (const auto& hh : c.heights) std::printf(" h@%.1f=%.0f", hh.first, hh.second * 1000);
        for (const auto& pp : c.pushes) std::printf(" push %.1f-%.1f %+.1fN", pp.t0, pp.t1, pp.v);
        for (const auto& d : c.drives) std::printf(" drive %.1f-%.1f %+.2f", d.t0, d.t1, d.v);
        std::printf("\n");
      }
      const Result r = simulate(c);
      if (r.fell) ++falls;
      held += r.held_resets;
      const double share = r.lean_rms_all > 1e-4 ? r.body_rms_all / r.lean_rms_all : 0.0;
      shares.push_back(share);
      worst_body = std::max(worst_body, r.body_dev_all);
      sum_rest += r.rest_rate_rms;
      if (std::getenv("GEN_VERBOSE") || r.fell || r.held_resets) {
        std::printf("  ep %2d %-22s fell=%d%s%s held=%d share %.2f body %.3f lean %.3f rest %.3f\n", seed0 + e,
                    kVariants[vi].name, r.fell, r.fell ? " " : "", r.fell ? r.fault : "", r.held_resets,
                    share, r.body_dev_all, r.lean_max, r.rest_rate_rms);
      }
    }
    std::sort(shares.begin(), shares.end());
    std::printf("%-6s %d episodes: falls %d, held %d | body share median %.2f p90 %.2f | worst body %.3f | mean rest %.3f\n",
                name.c_str(), n, falls, held, shares[shares.size() / 2],
                shares[std::min(shares.size() - 1, shares.size() * 9 / 10)], worst_body, sum_rest / n);
    return 0;
  }
  if (const char* cfgname = std::getenv("TIER_CROUCH")) {
    // The height-independent scorecard (adaptive tiers plan): a 3 N push
    // at each height once the height is reached, nominal and every plant
    // variant. Body share = body pitch change / CoM lean change.
    //   TIER_CROUCH=tiers|full|wb|<anything, with CR_* env overrides>
    const std::string name = cfgname;
    const bool novar = std::getenv("CR_NOVAR") != nullptr;
    for (double h : {0.0, 0.150, 0.136, 0.110, 0.095, 0.0784}) {
      int bad = 0; double wshare = 0, wrest = 0, wbody = 0;
      Result nom;
      const int nv = novar ? 1 : static_cast<int>(sizeof(kVariants) / sizeof(kVariants[0]));
      for (int vi = 0; vi < nv; ++vi) {
        if (const char* vs = std::getenv("CR_VARS")) {
          if (std::string(",") .append(vs).append(",").find("," + std::to_string(vi) + ",") == std::string::npos) continue;
        }
        for (double f : {3.0, -3.0}) {
          if (novar && f < 0) continue;
          Case c = withMode(Case{}, 2);
          if (vi > 0) kVariants[vi].apply(c);
          if (name == "wb") c.algo = V3Controller::Algo::WholeBody;
          if (name == "full") c.tier_exp = 0.0;
          if (const char* v = std::getenv("CR_EXP")) c.tier_exp = std::atof(v);
          if (const char* v = std::getenv("CR_LEAD")) c.tier_lead = std::atof(v);
          if (h > 0) { c.height_low_m = h; c.height_low_s = 2.0; c.height_up_s = 1e9; c.push_wait_height = true; }
          c.push_n = f; c.push_start_s = 8.0; c.push_end_s = 8.3; c.duration_s = 16.0;
          const Result r = simulate(c);
          if (r.fell || r.held_resets) ++bad;
          if ((r.fell || r.held_resets) && std::getenv("CR_VERBOSE")) {
            std::printf("   bad: %s h %.0f f %+.0f fell=%d (%s) held=%d\n", kVariants[vi].name,
                        h * 1000, f, r.fell, r.fault, r.held_resets);
          }
          const double share = r.push_lean > 1e-4 ? r.push_body / r.push_lean : 0.0;
          wshare = std::max(wshare, share); wrest = std::max(wrest, r.rest_rate_rms);
          wbody = std::max(wbody, r.push_body);
          if (vi == 0 && f > 0) nom = r;
        }
      }
      std::printf("%-6s h %3.0f (at push %3.0f): body %.3f lean %.3f share %.2f hip %.3f knee %.3f dz %.1f rest %.3f | "
                  "%d runs: bad %d worst share %.2f body %.3f rest %.3f\n",
                  name.c_str(), h > 0 ? h * 1000 : 189.0, nom.push_height * 1000, nom.push_body, nom.push_lean,
                  nom.push_lean > 1e-4 ? nom.push_body / nom.push_lean : 0.0, nom.push_hip, nom.push_knee,
                  nom.push_dz * 1000, nom.rest_rate_rms, novar ? 1 : 2 * nv, bad, wshare, wbody, wrest);
      std::fflush(stdout);
    }
    return 0;
  }
  if (std::getenv("TIER_LEAD") != nullptr) {
    // Full tier-2 share crouched: does more lean-rate lead (hip servo lag
    // vs the faster crouched pendulum) stop the ringing?
    for (double h : {0.110, 0.0784}) {
      for (double e : {0.0, 1.5}) {
        for (double lead : {0.0, 0.03, 0.06, 0.10, 0.15, 0.20}) {
          for (int vi : {0, 4, 6}) {
            Case c = withMode(Case{}, 2);
            if (vi > 0) kVariants[vi].apply(c);
            c.tier_exp = e; c.tier_lead = lead;
            if (const char* v = std::getenv("LD_MAX")) c.tier_max = std::atof(v);
            if (std::getenv("LD_ONE") && (vi != 0 || std::abs(lead - std::atof(std::getenv("LD_ONE"))) > 1e-6 ||
                                          std::abs(h - std::atof(std::getenv("LD_H"))) > 1e-4 || e != std::atof(std::getenv("LD_E")))) continue;
            c.trace = std::getenv("LD_ONE") != nullptr;
            c.height_low_m = h; c.height_low_s = 3.0; c.height_up_s = 99.0;
            c.duration_s = 25.0; c.push_n = 3.0; c.push_start_s = 15.0; c.push_end_s = 15.3;
            const Result r = simulate(c);
            std::printf("h %3.0f exp %.1f lead %.2f %-18s fell=%d held=%d rest_rate=%.3f rest_hip=%.3f push body %.3f lean %.3f hip %.3f\n",
                        h * 1000, e, lead, kVariants[vi].name, r.fell, r.held_resets, r.rest_rate_rms, r.rest_hip_rms,
                        r.push_body, r.push_lean, r.push_hip);
            std::fflush(stdout);
          }
        }
      }
    }
    return 0;
  }
  if (std::getenv("TIER_V6") != nullptr) {
    // V6 (crouched, pushed 3 N) per height and tier-2 share exponent.
    for (double h : {0.150, 0.110, 0.0784}) {
      for (double e : {-2.0, 0.0, 0.5, 1.0, 1.5}) {  // -2: Algo::WholeBody
        if (std::getenv("V6_E") && std::atof(std::getenv("V6_E")) != e) continue;
        for (int vi : {0, 4, 6}) {
          Case c = withMode(Case{}, 2);
          if (vi > 0) kVariants[vi].apply(c);
          if (e < -1.5) c.algo = V3Controller::Algo::WholeBody; else c.tier_exp = e;
          c.height_low_m = h; c.height_low_s = 3.0; c.height_up_s = 99.0;
          c.duration_s = 25.0; c.push_n = 3.0; c.push_start_s = 15.0; c.push_end_s = 15.3;
          const Result r = simulate(c);
          std::printf("h %3.0f exp %.1f %-18s fell=%d held=%d rest_rate=%.3f rest_hip=%.3f push body %.3f lean %.3f hip %.3f end_rms %.4f\n",
                      h * 1000, e, kVariants[vi].name, r.fell, r.held_resets, r.rest_rate_rms, r.rest_hip_rms,
                      r.push_body, r.push_lean, r.push_hip, r.body_rms_end);
        }
      }
    }
    return 0;
  }
  if (std::getenv("TIER_EXP") != nullptr) {
    // Tier-2 share vs height (Tier::height_exp): pushed at a crouch height.
    std::vector<double> hs = {0.136, 0.110, 0.078}, es = {-1.0, 1.5};
    if (const char* v = std::getenv("EXP_H")) { hs.clear(); for (const char* q = v; *q; ) { char* e; hs.push_back(std::strtod(q, &e)); q = *e ? e + 1 : e; } }
    if (const char* v = std::getenv("EXP_E")) { es.clear(); for (const char* q = v; *q; ) { char* e; es.push_back(std::strtod(q, &e)); q = *e ? e + 1 : e; } }
    for (double h : hs) {
      for (double e : es) {
        int bad = 0; double wb = 0, wr = 0, wh = 0, sumb = 0, suml = 0, sumq = 0; int n = 0;
        for (int vi = 0; vi < 8; ++vi) {
          for (double f : {4.0, -4.0}) {
            Case c = withMode(Case{}, 2);
            if (vi > 0) kVariants[vi].apply(c);
            c.tier_exp = e;
            c.height_low_m = h; c.height_low_s = 3.0; c.height_up_s = 60.0;
            c.push_n = f; c.push_start_s = 12.0; c.push_end_s = 12.3; c.duration_s = 20.0;
            const Result r = simulate(c);
            if (r.fell || r.held_resets) ++bad;
            wr = std::max(wr, r.rest_rate_rms); wh = std::max(wh, r.rest_hip_rms);
            wb = std::max(wb, r.push_body);
            if (vi == 0) { sumb += r.push_body; suml += r.push_lean; sumq += r.push_hip; ++n; }
          }
        }
        std::printf("h %3.0f mm exp %.2f: nominal push body %.3f lean %.3f hip %.3f (body share %.2f) | "
                    "8 variants x2: bad %d, worst push body %.3f, rest rate %.2f, rest hip %.2f\n",
                    h * 1000, e, sumb / n, suml / n, sumq / n, sumb / suml, bad, wb, wr, wh);
      }
    }
    return 0;
  }
  if (std::getenv("TIER_GRAB") != nullptr) {
    // Held tilted in the air for 3 s after the trim was learned, then let go
    // at that angle; learned (default) vs never learned.
    for (int mode = 1; mode < 3; ++mode) {
      for (double bias : {-0.10, -0.08, -0.05, 0.05, 0.08, 0.10}) {
        for (int learn = 0; learn < 2; ++learn) {
          Case c = withMode(Case{}, mode);
          c.grab_s = 30.0;
          c.duration_s = 42.0;
          c.grab_release_bias_rad = bias;
          if (!learn) c.learn_hold = 1000.0;
          if (std::getenv("TL_REL")) c.level_rel = std::atof(std::getenv("TL_REL"));
          const Result r = simulate(c);
          std::printf("grab %-13s bias %+.2f %-9s fell=%d%s held=%d post_peak=%.3f settled=%.4f "
                      "end_rms=%.4f x_end=%+.3f wheel_grab=%.1f putdown_w=%+.1f cmd=%+.1f\n", modeName(mode), bias,
                      learn ? "learned" : "unlearned", r.fell, r.fell ? r.fault : "", r.held_resets,
                      r.post_release_dev, r.settled_dev, r.body_rms_end, r.x_end, r.grab_max_wheel,
                      r.putdown_wheel, r.putdown_cmd);
        }
      }
    }
    return 0;
  }
  if (std::getenv("TIER_LEVEL") != nullptr) {
    const bool robust_only = std::getenv("TL_ROBUST") != nullptr;
    // Static level loop sweep: target offset from the natural resting pitch
    // (hardware: natural -0.085 rad, target 0 -> +0.085) x scenario.
    for (double rel : {-0.10, -0.05, 0.05, 0.085, 0.12, 0.16}) {
      if (robust_only) break;
      for (int sc = 0; sc < 11; ++sc) {
        Case c = withMode(Case{}, 2);
        c.level_rel = rel;
        const char* sn = "stand";
        if (sc == 1) { c.seed_error = 0.06; sn = "seed .06"; }
        if (sc == 2) { c.seed_error = -0.06; sn = "seed -.06"; }
        if (sc == 3) { c.drive_m_s = 0.55; sn = "drive +.55"; }
        if (sc == 4) { c.drive_m_s = -0.55; sn = "drive -.55"; }
        if (sc == 5) { c.push_n = 4.0; sn = "push +4"; }
        if (sc == 6) { c.push_n = -4.0; sn = "push -4"; }
        if (sc == 7) { c.height_low_m = 0.110; c.duration_s = 30; c.height_up_s = 40; sn = "110 mm"; }
        if (sc == 8) { c.height_low_m = 0.078; c.duration_s = 30; c.height_up_s = 40; sn = "78 mm"; }
        if (sc == 9) { c.wheel_tau = 0.12; c.drive_m_s = 0.75; sn = "hard stop .75"; }
        if (sc == 10) { c.held_s = 8.0; sn = "stand flow 8 s"; }
        const Result r = simulate(c);
        std::printf("level %+.3f %-15s fell=%d%s held=%d body_all=%.3f end_rms=%.4f settle=%.1f "
                    "lvl_hip=%+.3f dhip=%.3f x_end=%+.3f wcmd=%.1f\n", rel, sn, r.fell,
                    r.fell ? r.fault : "", r.held_resets, r.body_dev_all, r.body_rms_end,
                    r.level_settle_s, r.level_hip_end, r.max_dhip, r.x_end, r.max_wheel_cmd);
      }
    }
    int n = 0, bad = 0;
    double worst = 0.0;
    for (const auto& v : kVariants) {
      for (int sc = 0; sc < 3; ++sc) {
        Case c = withMode(Case{}, 2);
        v.apply(c);
        c.level_rel = std::getenv("TL_REL") ? std::atof(std::getenv("TL_REL")) : 0.085;
        if (const char* lv = std::getenv("TL_LVL")) c.level = std::atoi(lv);
        if (sc == 1) c.drive_m_s = 0.55;
        if (sc == 2) c.push_n = 4.0;
        const Result r = simulate(c);
        ++n;
        if (r.fell || r.held_resets != 0 || r.max_wheel_cmd >= 25.0 || r.body_rms_end > 0.01) {
          ++bad;
          std::printf("  variant %-22s sc=%d fell=%d%s held=%d end_rms=%.4f wcmd=%.1f\n", v.name, sc,
                      r.fell, r.fell ? r.fault : "", r.held_resets, r.body_rms_end, r.max_wheel_cmd);
        }
        worst = std::max(worst, r.body_rms_end);
      }
    }
    std::printf("level +0.085 robustness: %d runs, %d bad, worst end rms %.4f\n", n, bad, worst);
    return 0;
  }
  if (std::getenv("TIER_NOISE") != nullptr) {
    // Hip velocity-estimate noise vs the joint-rate weight in theta_v_rate.
    // Measured 2026-10-01: a low-pass on the rates destabilised tier 2 (lag
    // breaks the body/hip cancellation) and weights <= 0.4 fell with trunk;
    // the static HIPTEST jitter (~0.05-0.1 rad/s) keeps weight 1 ahead of plain LQR.
    for (double noise : {0.03, 0.1, 0.2, 0.4}) for (double fc : {1.0, 0.7, 0.4, 0.0}) {
      double rest[3] = {}, body = 0.0, wc = 0.0, hipr = 0.0;
      int falls = 0;
      for (int mode = 1; mode < 3; ++mode) {
        for (int sc = 0; sc < 3; ++sc) {
          Case c = withMode(Case{}, mode);
          c.hip_vel_noise = noise; c.joint_rate_gain = fc;
          if (sc == 1) c.drive_m_s = 0.55;
          if (sc == 2) c.push_n = 4.0;
          const Result r = simulate(c);
          falls += r.fell;
          if (sc == 0) rest[mode] = r.rest_rate_rms;
          if (mode == 2) { body = std::max(body, r.body_dev_all); hipr = std::max(hipr, r.rest_hip_rms); }
          wc = std::max(wc, r.max_wheel_cmd);
        }
      }
      std::printf("noise %.2f gain %.1f: falls %d rest vpitch %.3f trunk %.3f (hip %.3f) trunk body %.3f wcmd %.1f\n",
                  noise, fc, falls, rest[1], rest[2], hipr, body, wc);
    }
    return 0;
  }
  if (std::getenv("TIER_PREVIEW_SWEEP") != nullptr) {
    int n = 0, falls = 0;
    for (const auto& v : kVariants) for (int sc = 0; sc < 5; ++sc) {
      Case c = withMode(Case{}, 2);
      c.preview = 1;
      v.apply(c);
      if (sc == 1) c.drive_m_s = 0.55;
      if (sc == 2) c.drive_m_s = -0.55;
      if (sc == 3) c.drive_m_s = 0.65;
      if (sc == 4) c.push_n = 4.0;
      const Result r = simulate(c);
      ++n;
      falls += r.fell;
      std::printf("%-22s sc%d fell=%d held=%d pitch=%.3f acc_pk=%.2f jerk=%.2f bump=%.3f dhip=%.3f wcmd=%.1f\n",
                  v.name, sc, r.fell, r.held_resets, r.body_dev_all, r.body_acc_peak, r.body_jerk_rms,
                  r.brake_bump, r.max_dhip, r.max_wheel_cmd);
    }
    std::printf("preview sweep: %d runs, %d falls\n", n, falls);
    return 0;
  }
  if (std::getenv("TIER_GIMBAL") != nullptr) {
    // Tier-2 gimbal quality: body pitch vs body translational jerk, over
    // lead and share, preview on (0.3 m/s^2, 1 m/s^3). Standing / drive / push.
    for (double lead : {0.06}) for (double share : {0.0, 30.0, 20.0, 15.0}) {
      double pitch = 0, jerk = 0, rest = 0, acc = 0, push_pitch = 0;
      int falls = 0;
      for (int sc = 0; sc < 3; ++sc) {
        Case c = withMode(Case{}, 2);
        c.preview = 1; c.plan_accel = 0.3; c.plan_jerk = 1.0;
        c.tier_lead = lead; c.lead_lpf = share;  // (share loop var = lead LPF Hz here)
        if (sc == 1) c.drive_m_s = 0.55;
        if (sc == 2) c.push_n = 4.0;
        if (sc == 0) { c.drive_m_s = 0.0; }
        const Result r = simulate(c);
        falls += r.fell;
        if (sc == 1) { pitch = r.body_dev_all; jerk = r.body_jerk_rms; acc = r.body_acc_peak; }
        if (sc == 2) push_pitch = r.body_dev_all;
        if (sc == 0) rest = r.rest_rate_rms;
      }
      double crouch_rest = 0.0;
      for (double h : {0.110, 0.0784}) {
        Case c = withMode(Case{}, 2);
        c.tier_lead = lead; c.lead_lpf = share;
        c.height_low_m = h; c.height_low_s = 3.0; c.height_up_s = 99.0; c.duration_s = 25.0;
        c.push_n = 3.0; c.push_start_s = 15.0; c.push_end_s = 15.3;
        const Result r = simulate(c);
        falls += r.fell;
        crouch_rest = std::max(crouch_rest, r.rest_rate_rms);
      }
      std::printf("   crouched push rest rate max %.3f\n", crouch_rest);
      std::printf("lead %.2f lpf %4.1f Hz: falls %d | drive: pitch %.3f jerk_rms %4.2f acc_pk %.2f | push pitch %.3f | rest rate %.3f\n",
                  lead, share, falls, pitch, jerk, acc, push_pitch, rest);
    }
    return 0;
  }
  if (std::getenv("TIER_PLATFORM") != nullptr) {
    // Body-as-platform metrics per mode: body pitch, body CoM acceleration
    // peak/rms, jerk rms, speed bumps. TIER_PLATFORM_TAU overrides wheel lag.
    const double tau = std::getenv("TIER_PLATFORM_TAU") ? std::atof(std::getenv("TIER_PLATFORM_TAU")) : 0.09;
    const std::pair<const char*, void (*)(Case&)> scens[] = {
        {"drive 0.30", [](Case& c) { c.drive_m_s = 0.30; }},
        {"drive 0.55", [](Case& c) { c.drive_m_s = 0.55; }},
        {"drive 0.70", [](Case& c) { c.drive_m_s = 0.70; }},
        {"push 4 N", [](Case& c) { c.push_n = 4.0; }},
    };
    for (const auto& sc : scens) for (int mode = 0; mode < 3; ++mode) {
      Case c = withMode(Case{}, mode);
      c.wheel_tau = tau;
      sc.second(c);
      if (const char* pv = std::getenv("TIER_PREVIEW")) c.preview = std::atoi(pv);
      if (const char* v = std::getenv("TIER_PACC")) c.plan_accel = std::atof(v);
      if (const char* v = std::getenv("TIER_PJERK")) c.plan_jerk = std::atof(v);
      if (const char* v = std::getenv("TIER_PT")) c.preview_s = std::atof(v);
      const Result r = simulate(c);
      std::printf("%-11s %-13s fell=%d pitch=%.3f acc_pk=%.2f acc_rms=%.2f jerk_rms=%5.1f "
                  "brake_bump=%.3f start_dip=%.3f dhip=%.3f\n", sc.first, modeName(mode), r.fell,
                  r.body_dev_all, r.body_acc_peak, r.body_acc_rms, r.body_jerk_rms, r.brake_bump,
                  r.start_dip, r.max_dhip);
    }
    return 0;
  }
  if (std::getenv("TIER_SWEEP") != nullptr) {
    // Tuning table: every variant x scenario x mode (~1 min).
    const std::pair<const char*, void (*)(Case&)> scens[] = {
        {"stand", [](Case&) {}},
        {"seed.08", [](Case& c) { c.seed_error = 0.08; }},
        {"drive+", [](Case& c) { c.drive_m_s = 0.55; }},
        {"drive-", [](Case& c) { c.drive_m_s = -0.55; }},
        {"push4", [](Case& c) { c.push_n = 4.0; }},
    };
    for (const auto& v : kVariants) for (const auto& sc : scens) for (int mode = 0; mode < 3; ++mode) {
      Case c = withMode(Case{}, mode);
      v.apply(c);
      sc.second(c);
      const Result r = simulate(c);
      std::printf("%-22s %-8s %-13s fell=%d held=%d body=%.3f lean=%.3f rest=%.3f dhip=%.3f "
                  "hipr=%.2f wcmd=%.1f stop=%+.3f\n", v.name, sc.first, modeName(mode), r.fell,
                  r.held_resets, r.body_dev_all, r.lean_max, r.rest_rate_rms, r.max_dhip,
                  r.max_hip_rate, r.max_wheel_cmd, r.x_stop_travel);
    }
    return 0;
  }

  // V3: the three modes on the nominal plant - standing, demo drive +-0.55
  // m/s, a 4 N push. None falls or looks held; the tiers never do worse than
  // plain LQR and tier 2 at least halves the body pitch excursion of driving and
  // pushing (plain LQR: ~0.09 rad; the legs take the lean).
  {
    const std::pair<const char*, void (*)(Case&)> scens[] = {
        {"stand", [](Case&) {}},
        {"drive +0.55", [](Case& c) { c.drive_m_s = 0.55; }},
        {"drive -0.55", [](Case& c) { c.drive_m_s = -0.55; }},
        {"push 4 N", [](Case& c) { c.push_n = 4.0; }},
    };
    for (const auto& sc : scens) {
      Result r[3];
      for (int mode = 0; mode < 3; ++mode) {
        Case c = withMode(Case{}, mode);
        sc.second(c);
        static char name[64];
        std::snprintf(name, sizeof name, "V3 %s %s", modeName(mode), sc.first);
        c.name = name;
        r[mode] = run(c);
        assert(!r[mode].fell && r[mode].held_resets == 0);
        assert(r[mode].max_wheel_cmd < 25.0);
        assert(r[mode].max_dhip <= 0.15 + 1e-6);
      }
      if (sc.first[0] != 's') assert(r[2].body_dev_all < 0.5 * r[0].body_dev_all);
      // On the measured wheel (fast actuator, 55 ms old reading) the
      // virtual pendulum alone (mode 1, not used on the robot) rests less
      // calmly than plain LQR after a reverse drive; the default tiers (mode 2)
      // must still be at least as calm as plain LQR.
      assert(r[1].rest_rate_rms <= std::max(r[0].rest_rate_rms, 0.15) + 0.01);
      assert(r[2].rest_rate_rms <= r[0].rest_rate_rms + 0.01);
    }
  }
  // V4: operator flow - engaged on the stand (held 8 s, wheels in the air,
  // legs dangling), carried to the floor, let go. Tier 2 must stay out while
  // held (a held body cannot close the hip loop), the held re-zero must still
  // fire with the hips moving, and it must stand.
  for (int mode = 0; mode < 3; ++mode) {
    Case c = withMode(Case{}, mode);
    c.held_s = 8.0;
    static char name[64];
    std::snprintf(name, sizeof name, "V4 %s held 8 s", modeName(mode));
    c.name = name;
    const Result r = run(c);
    std::printf("   on the stand: max dhip %.4f, max wheel %.1f rad/s\n", r.held_max_dhip,
                r.held_max_wheel);
    assert(!r.fell && r.held_resets >= 1);
    assert(r.held_max_dhip < 0.005);
  }
  // V5: height 188 -> 110 / 78 mm and back while balancing. Plain LQR on the
  // two-body plant wobbles the body on the compliant hip servos at low
  // height (the ~5 Hz wobble seen on the robot); the virtual pitch leaves
  // that mode out of the balancer. Tier 2 stays on, its share scaled by the
  // pendulum length (full share rang at <= 110 mm).
  for (double h : {0.110, 0.0784}) {
    Result r[3];
    for (int mode = 0; mode < 3; ++mode) {
      Case c = withMode(Case{}, mode);
      c.height_low_m = h;
      c.height_low_s = 5.0;
      c.height_up_s = 17.0;
      c.duration_s = 30.0;
      static char name[64];
      std::snprintf(name, sizeof name, "V5 %s height %.0f mm", modeName(mode), h * 1000);
      c.name = name;
      r[mode] = run(c);
      assert(!r[mode].fell && r[mode].held_resets == 0);
    }
    // Measured wheel: mode 1 (unused) 0.215 at 78 mm; the robot's tiers
    // rested at 0.117 there (2026-10-01).
    assert(r[1].rest_rate_rms < 0.25 && r[2].rest_rate_rms < 0.2);
  }
  // V6: tier 2 standing crouched, pushed 3 N: calm at every height.
  for (double h : {0.150, 0.110, 0.0784}) {
    Case c = withMode(Case{}, 2);
    c.height_low_m = h;
    c.height_low_s = 3.0;
    c.height_up_s = 99.0;
    c.duration_s = 25.0;
    c.push_n = 3.0;
    c.push_start_s = 15.0;
    c.push_end_s = 15.3;
    static char name[64];
    std::snprintf(name, sizeof name, "V6 trunk at %.0f mm, push", h * 1000);
    c.name = name;
    const Result r = run(c);
    assert(!r.fell && r.held_resets == 0 && r.rest_rate_rms < 0.25);
  }
  // V7: robustness - every plant variant, standing / driving / pushed, with
  // both tiers on: never falls, never looks held, keeps wheel headroom.
  {
    int n = 0;
    double worst_body = 0.0, worst_rest = 0.0;
    for (const auto& v : kVariants) {
      for (int sc = 0; sc < 3; ++sc) {
        Case c = withMode(Case{}, 2);
        v.apply(c);
        if (sc == 1) c.drive_m_s = 0.55;
        if (sc == 2) c.push_n = 4.0;
        const Result r = simulate(c);
        if (r.fell || r.held_resets != 0 || r.max_wheel_cmd >= 25.0) {
          c.name = v.name;
          report(c, r);
        }
        assert(!r.fell && r.held_resets == 0 && r.max_wheel_cmd < 25.0);
        worst_body = std::max(worst_body, r.body_dev_all);
        worst_rest = std::max(worst_rest, r.rest_rate_rms);
        ++n;
      }
    }
    std::printf("V7 robustness: %d runs, none fell; worst body dev %.3f rad, worst rest rate %.2f rad/s\n",
                n, worst_body, worst_rest);
    assert(worst_body < 0.08);
  }
  // V8: seed error 0.08 rad and a 0.03 rad slope both ways with the tiers.
  for (int sc = 0; sc < 3; ++sc) {
    Case c = withMode(Case{}, 2);
    if (sc == 0) c.seed_error = 0.08;
    if (sc == 1) c.slope = 0.03;
    if (sc == 2) c.slope = -0.03;
    c.name = sc == 0 ? "V8 trunk seed 0.08" : sc == 1 ? "V8 trunk slope +0.03" : "V8 trunk slope -0.03";
    const Result r = run(c);
    assert(!r.fell && r.held_resets == 0);
  }
  // V9: SET trunk 1 at 8 s and 0 at 14 s while driving 0.3 m/s (9-12 s):
  // authority ramps, no kick.
  {
    Case c = withMode(Case{}, 1);
    c.trunk_on_s = 8.0;
    c.trunk_off_s = 14.0;
    c.drive_m_s = 0.3;
    c.drive_start_s = 9.0;
    c.drive_end_s = 12.0;
    c.name = "V9 trunk on 8 s, off 14 s";
    const Result r = run(c);
    assert(!r.fell && r.held_resets == 0 && r.body_dev_all < 0.05);
  }
  // V10: the tiers are LQR only - with the PID the flags change nothing.
  {
    Case a;
    a.algo = V3Controller::Algo::Pid;
    a.drive_m_s = 0.3;
    Case b = withMode(a, 2);
    const Result ra = simulate(a), rb = simulate(b);
    std::printf("V10 PID with tier flags: x_end %+.6f vs %+.6f, dhip %.4f\n", ra.x_end, rb.x_end,
                rb.max_dhip);
    assert(ra.x_end == rb.x_end && rb.max_dhip == 0.0);
  }
  // V11: grabbed mid-balance for 3 s (body held in the air, wheels free)
  // and put back down, tier 2 active before the grab. The held re-zero must
  // fire (a held body is still whatever the hips do), tier 2 must be out
  // before the put-down, and it must balance again.
  for (int mode = 0; mode < 3; ++mode) {
    Case c = withMode(Case{}, mode);
    c.grab_s = 8.0;
    static char name[64];
    std::snprintf(name, sizeof name, "V11 %s grabbed 3 s", modeName(mode));
    c.name = name;
    const Result r = run(c);
    std::printf("   grab: held re-zero after %.2f s, tier-2 hip in the last 1 s %.4f rad\n",
                r.grab_detect_s, r.grab_max_dhip_end);
    assert(!r.fell && r.grab_detect_s >= 0.0 && r.grab_detect_s < 1.5);
    assert(r.grab_max_dhip_end < 0.005);
  }
  // V12: a 4 N backward push while driving 0.45 m/s. Regression for the
  // held false positive: with tier 2 the body stays level through the
  // catch, and a held test on the body rate alone re-zeroed the robot
  // mid-catch (fell, 2026-10-01). Tier 2 still halves the body excursion.
  {
    Result r[3];
    for (int mode = 0; mode < 3; ++mode) {
      Case c = withMode(Case{}, mode);
      c.drive_m_s = 0.45;
      c.push_n = -4.0;
      c.push_start_s = 8.0;
      c.push_end_s = 8.3;
      static char name[64];
      std::snprintf(name, sizeof name, "V12 %s push while driving", modeName(mode));
      c.name = name;
      r[mode] = run(c);
      assert(!r[mode].fell && r[mode].held_resets == 0);
    }
    assert(r[2].body_dev_all < 0.5 * r[0].body_dev_all);
    // The case that fell (tau 0.06 under the old 50 ms + 0.09 model) and its
    // neighbour, around the measured 0.035 s lag.
    for (double tau : {0.06, 0.10}) {
      Case c = withMode(Case{}, 2);
      c.wheel_tau = tau;
      c.push_n = 4.0;
      const Result rr = simulate(c);
      assert(!rr.fell && rr.held_resets == 0);
    }
  }
  // V14: hard stop from near-vmax. Releasing a 0.75 m/s drive command with a
  // realistic wheel lag (0.12 s, fitted BL4820 range) makes the LQR's own
  // catch transient ask more of the lean than tier 2's hip range can supply
  // while the wheel command is simultaneously near its own cap; a hard hip
  // clamp with no early release held the body at zero lean the whole time
  // and then dumped the full shortfall on it in one step right at that
  // moment - found to fall in this exact case (2026-10-01) where plain LQR alone
  // falls earlier (0.72 m/s) and tier 1 alone survives further (0.78 m/s).
  // The early hand-back (Tier::sat_margin/wheel_sat_margin) must close that
  // gap back to at least tier 1 alone's margin without regressing the
  // ordinary push-under-wheel-lag case (V7).
  for (double v : {0.55, 0.65, 0.70, 0.75}) {
    Case c = withMode(Case{}, 2);
    c.wheel_tau = 0.12;
    c.drive_m_s = v;
    static char name[64];
    std::snprintf(name, sizeof name, "V14 trunk hard stop from %.2f m/s", v);
    c.name = name;
    const Result r = run(c);
    assert(!r.fell && r.held_resets == 0);
  }
  // V15: maneuver preview (Tier::preview) - the body as a stabilised
  // platform. A balancing robot answering a speed step first speeds up to
  // get the wheels ahead of its CoM before it brakes (the non-minimum-phase
  // catch); with the command planned (jerk-limited) and previewed 0.12 s
  // ahead, the lean is built on purpose, the wheels get the plan early, and
  // the legs take the planned lean from the plan itself.
  {
    for (double v : {0.30, 0.55, -0.55}) {
      Result r[2];
      for (int prev = 0; prev < 2; ++prev) {
        Case c = withMode(Case{}, 2);
        c.preview = prev;
        c.drive_m_s = v;
        static char name[64];
        std::snprintf(name, sizeof name, "V15 trunk %s drive %+.2f", prev ? "preview" : "reactive", v);
        c.name = name;
        r[prev] = run(c);
        std::printf("   body acc peak %.2f m/s^2, jerk rms %.2f, brake bump %+.3f m/s\n",
                    r[prev].body_acc_peak, r[prev].body_jerk_rms, r[prev].brake_bump);
        assert(!r[prev].fell && r[prev].held_resets == 0);
      }
      assert(r[1].brake_bump < 0.01);           // no speed-up before braking
      assert(r[1].body_acc_peak < 0.5);
      assert(r[1].body_dev_all < 0.03);
      if (std::abs(v) > 0.5) assert(r[0].brake_bump > 0.05);  // the catch it removes
    }
    // Robustness: every plant variant, standing / +-0.55 / 0.8 commanded
    // (planned at the 0.55 cap) / pushed, preview on.
    int n = 0;
    for (const auto& v : kVariants) for (int sc = 0; sc < 5; ++sc) {
      Case c = withMode(Case{}, 2);
      c.preview = 1;
      v.apply(c);
      if (sc == 1) c.drive_m_s = 0.55;
      if (sc == 2) c.drive_m_s = -0.55;
      if (sc == 3) c.drive_m_s = 0.8;
      if (sc == 4) c.push_n = 4.0;
      const Result r = simulate(c);
      if (r.fell || r.held_resets != 0) { c.name = v.name; report(c, r); }
      assert(!r.fell && r.held_resets == 0);
      ++n;
    }
    std::printf("V15 preview robustness: %d runs, none fell\n", n);
    // Operator flow and a grab with preview on; and preview is ignored
    // without the virtual pitch (plain LQR unchanged).
    Case h = withMode(Case{}, 2);
    h.preview = 1;
    h.held_s = 8.0;
    h.drive_m_s = 0.3;
    h.drive_start_s = 4.0;
    h.drive_end_s = 7.0;
    const Result rh = simulate(h);
    assert(!rh.fell && rh.held_resets >= 1 && rh.held_max_dhip < 0.005);
    Case g = withMode(Case{}, 2);
    g.preview = 1;
    g.grab_s = 8.0;
    const Result rg = simulate(g);
    assert(!rg.fell && rg.grab_detect_s >= 0.0 && rg.grab_detect_s < 1.5);
    Case a = withMode(Case{}, 0);
    a.drive_m_s = 0.3;
    Case b = a;
    b.preview = 1;
    const Result ra = simulate(a), rb = simulate(b);
    assert(ra.x_end == rb.x_end);
  }
  // V13: the kinematic IMU compensation (Tier::imu_kinematic) keeps the
  // pitch estimate within a few mrad through a 0.55 m/s drive, where the
  // gyro-dominant estimator alone drifts toward the apparent gravity.
  {
    Result r[2];
    for (int kin = 0; kin < 2; ++kin) {
      Case c = withMode(Case{}, 2);
      c.imu_kin = kin;
      c.drive_m_s = 0.55;
      r[kin] = simulate(c);
    }
    std::printf("V13 pitch estimate error driving: max %.4f rad without, %.4f with kinematics\n",
                r[0].est_err_max, r[1].est_err_max);
    assert(!r[1].fell && r[1].est_err_max < 0.5 * r[0].est_err_max);
  }
  // V16: the auto-learned balance trim (Config::learn_hold_s) becomes valid
  // standing, however badly the first engage was seeded, and the body ends
  // level (the level loop holds it from ~3 s after release).
  {
    Case c = withMode(Case{}, 2);
    c.seed_error = 0.06;  // a deliberately bad first-engage placement
    c.duration_s = 34.0;  // past the ~27 s it takes with tier 2's hip noise
    c.name = "V16 learned trim, seed 0.06";
    const Result r = run(c);
    std::printf("V16 learned trim: settled body rms (last 4 s) %.4f rad, worst %.4f\n",
                r.body_rms_end, r.body_dev_all);
    assert(!r.fell && r.held_resets == 0 && r.learned_valid);
    assert(r.body_rms_end < 0.006 && r.body_dev_all < 0.03);
  }
  // V17: static level loop (Tier::level). The hardware body rests at -0.085
  // rad with the legs at the pose and the target is 0, so the loop holds
  // ~0.085 rad of hip for good; targets both ways of the natural pose must
  // be reached (end rms), cost nothing in robustness (no fall, no held
  // re-zero, wheel headroom), and a target past level_max_rad must clamp
  // cleanly instead of winding up.
  {
    for (double rel : {-0.10, 0.085}) {
      for (int sc = 0; sc < 5; ++sc) {
        Case c = withMode(Case{}, 2);
        c.level_rel = rel;
        const char* sn = "stand";
        if (sc == 1) { c.drive_m_s = 0.55; sn = "drive 0.55"; }
        if (sc == 2) { c.push_n = -4.0; sn = "push 4 N"; }
        if (sc == 3) { c.height_low_m = 0.110; c.duration_s = 30; c.height_up_s = 40; sn = "110 mm"; }
        if (sc == 4) { c.wheel_tau = 0.12; c.drive_m_s = 0.75; sn = "hard stop 0.75"; }
        static char name[64];
        std::snprintf(name, sizeof name, "V17 level %+.3f %s", rel, sn);
        c.name = name;
        const Result r = run(c);
        std::printf("   level: hip %+.3f, body at target after %.1f s, end rms %.4f\n",
                    r.level_hip_end, r.level_settle_s, r.body_rms_end);
        assert(!r.fell && r.held_resets == 0 && r.max_wheel_cmd < 25.0);
        assert(r.body_rms_end < 0.008);
        if (sc == 0) assert(r.level_settle_s < 4.0);  // release + settle_s + ramp_s
      }
    }
    Case c = withMode(Case{}, 2);
    c.level_rel = 0.25;
    c.push_n = 4.0;
    c.name = "V17 level +0.250 (past the limit)";
    const Result r = run(c);
    assert(!r.fell && std::abs(r.level_hip_end) <= tap_fw::v3::defaultV3Config().tier.level_max_rad + 1e-4);
  }
  // V18: lifted and set down again (the operator's stand -> floor move):
  // held in the air for 3 s tilted by hand, let go on the floor at that
  // angle. Before 2026-10-01 the wheels raced in the air (balancer speed
  // feedback is positive feedback there, and the held state ended within
  // 0.24 s on a drifting pitch estimate), and the robot was set down with
  // them at -10..-20 rad/s: fell at 0.05 rad in every mode. Now: wheels
  // capped while held, held exit on raw accel_x, capture-point seed on the
  // first release, learned trim after that.
  {
    for (int mode = 1; mode < 3; ++mode) {
      for (double bias : {-0.08, 0.08}) {
        for (int learn = 0; learn < 2; ++learn) {
          Case c = withMode(Case{}, mode);
          c.grab_s = 30.0;
          c.duration_s = 42.0;
          c.grab_release_bias_rad = bias;
          if (!learn) c.learn_hold = 1000.0;
          static char name[80];
          std::snprintf(name, sizeof name, "V18 %s set down %+.2f %s", modeName(mode), bias,
                        learn ? "learned" : "first");
          c.name = name;
          const Result r = run(c);
          std::printf("   set down: wheels %+.2f rad/s, travel %+.3f m, end rms %.4f\n",
                      r.putdown_wheel, r.x_end, r.body_rms_end);
          assert(!r.fell && r.held_resets == 1 && std::abs(r.putdown_wheel) < 1.0);
          assert(r.body_rms_end < 0.006);
          assert(std::abs(r.x_end) < (learn ? 0.12 : 0.7));
        }
      }
    }
  }
  // V19: pushes the robot can take (wheel speed stays under its cap), short
  // and held on like a hand, must never look held/lifted: a re-zero turns
  // tier 2 off for settle_s + ramp_s. Hardware 2026-09-30: 31 of 33 re-zeros
  // were in the air (stand, hands), none in a recoverable floor push.
  {
    for (double h : {0.0, 0.136}) {
      for (double len : {0.3, 0.6, 1.0}) {
        Case c = withMode(Case{}, 2);
        if (h > 0) { c.height_low_m = h; c.height_low_s = 3.0; c.height_up_s = 60.0; }
        // 4 N held 1 s needs ~0.87 m/s of wheel - over the 0.8 cap on the
        // measured wheel (every lead fell, TIER_PUSHSWEEP): 3 N for 1 s.
        c.push_n = len > 0.9 ? 3.0 : 4.0; c.push_start_s = 12.0; c.push_end_s = 12.0 + len; c.duration_s = 20.0;
        static char name[64];
        std::snprintf(name, sizeof name, "V19 push %.0f N x %.1f s at %s", c.push_n, len, h > 0 ? "136" : "189");
        c.name = name;
        const Result r = run(c);
        assert(!r.fell && r.held_resets == 0);
      }
    }
  }
  // V21: KNEETEST FLOOR (knee +-0.03 rad around the height pose while
  // balancing, hips following as for a height change, 0.5/1/2/3 Hz) must be
  // safe to run on the robot: no fall, never looks held/lifted, the CoM line
  // and body barely move. At the top height the knee can only fold, so the
  // swing is centred 0.03 rad folded (AppV3 does the same). Default tiers
  // only: the plain LQR fallback has no theta_v and sees every leg motion as a
  // body tilt. AppV3 refuses it below 120 mm (90 mm: body 0.028 rad, the
  // crouched robot already wobbles more).
  {
    for (double h : {0.0, 0.136}) {
      Case c = withMode(Case{}, 2);
      if (h > 0) { c.height_low_m = h; c.height_low_s = 3.0; c.height_up_s = 60.0; }
      c.excite_amp = 0.03;
      c.excite_start_s = 10.0;
      c.duration_s = 22.0;
      static char name[64];
      std::snprintf(name, sizeof name, "V21 knee floor test at %s", h > 0 ? "136" : "189");
      c.name = name;
      const Result r = run(c);
      std::printf("   floor test: lean %.4f rad, body %.4f rad, knee lag %.4f rad\n",
                  r.excite_lean, r.excite_body, r.excite_knee_err);
      assert(!r.fell && r.held_resets == 0);
      assert(r.excite_lean < 0.05 && r.excite_body < 0.02);
    }
  }
  // V22: rolling over floor bumps the wheels can climb (default tiers): 5 mm
  // with a 2 cm edge (21 deg at its steepest) and 8 mm with a 4 cm edge (17
  // deg), 3 cm on top, at 0.15 and 0.30 m/s. No fall, never looks held; the
  // body's vertical acceleration peak is the baseline the whole-body
  // controller's suspension has to beat. Known limit (TIER_BUMP sweep): past
  // ~20-25 deg the BL4820 torque cap (0.6 N m, ~15 N at the floor for a
  // 45 N robot) cannot lift it - a square 8 mm step or 12 mm over 4 cm falls,
  // 8-12 mm over 6 cm stalls at 0.15 m/s.
  {
    for (int b = 0; b < 2; ++b) {
      for (double v : {0.15, 0.30}) {
        Case c = withMode(Case{}, 2);
        c.bump_h = b ? 0.008 : 0.005; c.bump_ramp = b ? 0.04 : 0.02;
        c.bump_x0 = 0.25; c.bump_width = 0.03;
        c.drive_m_s = v; c.drive_start_s = 6.0; c.drive_end_s = 6.0 + 0.6 / v + 1.0;
        c.duration_s = c.drive_end_s + 5.0;
        static char name[64];
        std::snprintf(name, sizeof name, "V22 bump %.0f mm at %.2f m/s", c.bump_h * 1000, v);
        c.name = name;
        const Result r = run(c);
        std::printf("   bump: body vertical accel peak %.2f m/s^2, body %.3f rad, lean %.3f rad\n",
                    r.body_vacc_peak, r.body_dev_all, r.lean_max);
        assert(!r.fell && r.held_resets == 0 && r.x_end > 0.5);
      }
    }
  }
  // V23: the whole-body LQR (Algo::WholeBody, SET algo 2, generated gains)
  // on the core scenarios: never falls, never looks held on the floor, keeps
  // the body level standing and at every height, and holds its own against
  // the tiers on pushes. Known limits (TIER_WB, docs/archive/V3.3_HANDOFF_HISTORY_20261005.md 5.10):
  // stalls at an 8 mm / 4 cm bump the tiers climb, ~20 % speed overshoot.
  {
    struct Sc { const char* name; void (*apply)(Case&); double body_max, lean_max; };
    const Sc scs[] = {
        {"stand", [](Case&) {}, 0.01, 0.05},
        // CoM lean up to 0.18: the measured-wheel gains (WbDesign.hpp) let
        // the legs take more of a push - body 0.005 rad, CoM 0.16.
        {"push 4 N", [](Case& c) { c.push_n = 4.0; }, 0.03, 0.18},
        {"push -4 N", [](Case& c) { c.push_n = -4.0; }, 0.03, 0.18},
        {"push 6 N x 0.3 s", [](Case& c) { c.push_n = 6.0; }, 0.04, 0.25},
        {"136 mm push 4 N x 1 s", [](Case& c) { c.height_low_m = 0.136; c.height_low_s = 3;
            c.height_up_s = 60; c.push_n = 4.0; c.push_start_s = 12; c.push_end_s = 13;
            c.duration_s = 20; }, 0.04, 0.26},
        {"height 136", [](Case& c) { c.height_low_m = 0.136; c.duration_s = 30; c.height_up_s = 17; },
         0.015, 0.05},
        {"height 78", [](Case& c) { c.height_low_m = 0.078; c.duration_s = 30; c.height_up_s = 17; },
         0.015, 0.05},
        // Drives: the envelope plans ~1 m/s^2 (Tier::env_lean_rad), so the
        // CoM leans ~0.10 rad - the legs' job; the body limit is the test.
        {"drive +0.55", [](Case& c) { c.drive_m_s = 0.55; }, 0.04, 0.16},
        {"drive -0.55", [](Case& c) { c.drive_m_s = -0.55; }, 0.04, 0.16},
        {"hard stop 0.75, lag 0.12", [](Case& c) { c.wheel_tau = 0.12; c.drive_m_s = 0.75; }, 0.04, 0.16},
        {"hard stop 0.75, lag 0.06", [](Case& c) { c.wheel_tau = 0.06; c.drive_m_s = 0.75; }, 0.04, 0.16},
        {"bump 5 mm at 0.30", [](Case& c) { c.bump_h = 0.005; c.bump_x0 = 0.25; c.bump_width = 0.03;
            c.drive_m_s = 0.30; c.drive_end_s = 9.0; c.duration_s = 14; }, 0.04, 0.10},
    };
    for (const auto& sc : scs) {
      Case c = withMode(Case{}, 2);
      c.algo = V3Controller::Algo::WholeBody;
      sc.apply(c);
      static char name[64];
      std::snprintf(name, sizeof name, "V23 WB %s", sc.name);
      c.name = name;
      const Result r = run(c);
      std::fflush(stdout);
      assert(!r.fell && r.held_resets == 0);
      assert(r.body_dev_all < sc.body_max && r.lean_max < sc.lean_max);
    }
    Case c = withMode(Case{}, 2);
    c.algo = V3Controller::Algo::WholeBody;
    c.held_s = 8.0;
    c.name = "V23 WB stand flow 8 s";
    const Result r = run(c);
    assert(!r.fell && r.held_resets >= 1);
  }
  // V24: knee suspension (Tier::suspension, sky-hook on the IMU/leg vertical
  // speed). Over a 5 mm bump at 0.30 m/s and an 8 mm / 4 cm bump at
  // 0.15 m/s the body rises less and accelerates less than with the knees
  // only holding the pose; standing it adds no motion (deadband) even with
  // 3x accelerometer noise; pushes are unchanged.
  {
    auto bump = [](double h, double ramp, double v) {
      Case c = withMode(Case{}, 2);
      c.bump_h = h; c.bump_ramp = ramp; c.bump_x0 = 0.25; c.bump_width = 0.03;
      c.drive_m_s = v; c.drive_end_s = 6.0 + 0.6 / v + 1.0; c.duration_s = c.drive_end_s + 5.0;
      return c;
    };
    for (int b = 0; b < 2; ++b) {
      Result r[2];
      for (int sus = 0; sus < 2; ++sus) {
        Case c = b ? bump(0.008, 0.04, 0.15) : bump(0.005, 0.02, 0.30);
        c.suspension = sus;
        static char name[64];
        std::snprintf(name, sizeof name, "V24 bump %d mm suspension %s", b ? 8 : 5, sus ? "on" : "off");
        c.name = name;
        r[sus] = run(c);
        assert(!r[sus].fell && r[sus].held_resets == 0);
      }
      std::printf("   bump %d mm: bob %.1f -> %.1f mm, vertical accel peak %.2f -> %.2f m/s^2\n",
                  b ? 8 : 5, r[0].body_z_range * 1000, r[1].body_z_range * 1000,
                  r[0].body_vacc_peak, r[1].body_vacc_peak);
      std::fflush(stdout);
      // The 8 mm bump is a slow 4 cm ramp the body has to follow. On the
      // measured wheel (2026-10-01) it already rides it softer without the
      // suspension (9.1 mm / 1.96 m/s^2 -> 8.6 / 1.71), leaving ~10 % to take
      // (7.8 mm with the knee also holding the height against the hip
      // swing, Tier::knee_hold); on the earlier wheel model the same code
      // took 20 % (7.3 / 1.47).
      const double k = b ? 0.95 : 0.8;
      assert(r[1].body_z_range < k * r[0].body_z_range || r[1].body_vacc_peak < k * r[0].body_vacc_peak);
      assert(r[1].body_vacc_peak <= r[0].body_vacc_peak * 1.02);
    }
    for (double accn : {1.0, 3.0}) {
      Result r[2];
      for (int sus = 0; sus < 2; ++sus) {
        Case c = withMode(Case{}, 2);
        c.suspension = sus;
        c.acc_noise_scale = accn;
        r[sus] = simulate(c);
      }
      std::printf("V24 standing, accel noise x%.0f: bob %.2f -> %.2f mm, pitch rate %.3f -> %.3f\n",
                  accn, r[0].body_z_range * 1000, r[1].body_z_range * 1000, r[0].rest_rate_rms,
                  r[1].rest_rate_rms);
      assert(r[1].body_z_range < r[0].body_z_range + 0.0006);
      assert(r[1].rest_rate_rms < r[0].rest_rate_rms * 1.2 + 0.005);
    }
  }
  // V25: legs arranged off by hand before power-up (the encoder zero is not
  // the intended hang pose), so N-P (CoM over the wheel contact) is only
  // aligned with the body tilted. The level loop must straighten the body
  // with the hips while tier 1 keeps N-P aligned - hip and knee errors both
  // ways, combined up to hip 0.10 + knee 0.08 rad.
  {
    for (double hz : {-0.10, 0.10}) {
      for (double kz : {0.0, -0.08}) {
        Case c = withMode(Case{}, 2);
        c.hip_zero_err = hz;
        c.knee_zero_err = kz;
        c.duration_s = 15.0;
        static char name[64];
        std::snprintf(name, sizeof name, "V25 legs arranged hip %+.2f knee %+.2f", hz, kz);
        c.name = name;
        const Result r = run(c);
        std::printf("   body tilted %+.3f at start -> rms %.4f at the end, hip took %+.3f, level after %.1f s\n",
                    r.start_dev, r.body_rms_end, r.level_hip_end, r.level_settle_s);
        assert(!r.fell && r.held_resets == 0);
        assert(r.body_rms_end < 0.005 && r.level_settle_s < 5.0 && r.lean_max < 0.03);
      }
    }
  }
  // V26: episodes nobody tuned for (randomEpisode: height schedule changed
  // while pushed, 2-4 random pushes, a drive, a slope, a random plant and
  // model error). The whole-body LQR must never fall or false-hold, and the
  // body must take well under half of the dynamic lean. The tiers are only
  // reported: they fall in ~2 of 48 such episodes (slow-wheel variants) and
  // their share is larger crouched (archive 5.13).
  {
    std::vector<double> shares[2];
    for (int wb = 0; wb < 2; ++wb) {
      for (int seed = 1; seed <= 8; ++seed) {
        int vi = 0;
        Case c = randomEpisode(seed, wb == 1, &vi);
        const Result r = simulate(c);
        const double share = r.lean_rms_all > 1e-4 ? r.body_rms_all / r.lean_rms_all : 0.0;
        shares[wb].push_back(share);
        std::printf("V26 %-5s episode %d (%s): fell=%d held=%d share %.2f body %.3f\n", wb ? "WB" : "tiers",
                    seed, kVariants[vi].name, r.fell, r.held_resets, share, r.body_dev_all);
        // Body bound 0.16: near the wheel cap the gains blend to the soft
        // table and the body helps the catch. Known limit: the "wheel delay
        // +20 ms" plant (40 ms dead + reading 65 ms old) can still ring cap
        // to cap at the wheel cap (episode 2; 2 of 48 in TIER_GENERAL=wb,
        // both that variant) - reported, not asserted. The tiers (default
        // algo) hold it.
        const bool known_limit = std::string(kVariants[vi].name) == "wheel delay +20ms";
        if (wb && !known_limit) assert(!r.fell && r.held_resets == 0 && r.body_dev_all < 0.16);
        if (wb && known_limit && r.fell) std::printf("    (known WB limit: %s)\n", kVariants[vi].name);
      }
      std::sort(shares[wb].begin(), shares[wb].end());
    }
    assert(shares[1][4] < 0.40 && shares[1][4] < shares[0][4]);
  }
  // V27: crouched to drive fast (operator practice). Algo::WholeBody at
  // 84 mm, 0.75 m/s asked, 3 N push mid-cruise, nominal and the slow-wheel
  // plants: the envelope lets it cruise above the fixed 0.55 cap, the legs
  // take the ~1 m/s^2 lean, the body stays pinned.
  for (int vi : {0, 2, 3}) {
    Case c = withMode(Case{}, 2);
    c.algo = V3Controller::Algo::WholeBody;
    if (vi > 0) kVariants[vi].apply(c);
    c.heights.push_back({1.0, 0.084});
    c.drive_m_s = 0.75; c.drive_start_s = 12.0; c.drive_end_s = 18.0; c.duration_s = 26.0;
    c.pushes.push_back({15.0, 15.3, -3.0});
    static char name[64];
    std::snprintf(name, sizeof name, "V27 WB 84 mm 0.75 m/s + push (%s)", kVariants[vi].name);
    c.name = name;
    const Result r = run(c);
    std::fflush(stdout);
    // Known WB limit (see V26): the +20 ms wheel plant rings at the cap.
    if (vi == 3) { if (r.fell) std::printf("    (known WB limit: %s)\n", kVariants[vi].name); continue; }
    assert(!r.fell && r.held_resets == 0 && r.body_dev_drive < 0.05 && r.max_speed > 0.65);
  }
  // V28: back to level with the whole-body LQR whatever the model error -
  // CoM off by 15 mm either way, legs arranged 0.1 rad off, a tilted
  // release: the equilibrium is read from the measured pose, so the body
  // is within 0.01 rad of level from 2 s on and within 0.003 at the end.
  {
    struct Sc { const char* name; void (*apply)(Case&); };
    const Sc scs[] = {
        {"CoM +15 mm", [](Case& c) { c.body_dx = 0.036; }},
        {"CoM -15 mm", [](Case& c) { c.body_dx = 0.006; }},
        {"legs hip +0.10 knee -0.08", [](Case& c) { c.hip_zero_err = 0.10; c.knee_zero_err = -0.08; }},
        {"legs hip -0.10", [](Case& c) { c.hip_zero_err = -0.10; }},
        {"seed 0.08", [](Case& c) { c.seed_error = 0.08; }},
    };
    for (const auto& sc : scs) {
      Case c = withMode(Case{}, 2);
      c.algo = V3Controller::Algo::WholeBody;
      c.duration_s = 16.0;
      sc.apply(c);
      static char name[64];
      std::snprintf(name, sizeof name, "V28 WB level, %s", sc.name);
      c.name = name;
      const Result r = run(c);
      std::printf("   above 0.01 rad until %.1f s, end mean %+.4f\n", r.settle01_s, r.body_mean_end);
      std::fflush(stdout);
      assert(!r.fell && r.held_resets == 0 && r.settle01_s < 3.0 && std::abs(r.body_mean_end) < 0.003);
    }
  }
  // V29: roll (Config::Roll) - cross slope, legs arranged unevenly, turns
  // at full and crouched height, tiers and whole-body: the body ends level
  // against the apparent vertical (leans into the turn), never falls, the
  // pitch side unchanged; off, the same cases are left tilted.
  {
    struct Sc { const char* name; void (*apply)(Case&); double app_max; };
    const Sc scs[] = {
        {"cross slope +0.06", [](Case& c) { c.cross_slope = 0.06; }, 0.03},
        {"legs uneven 8 mm", [](Case& c) { c.leg_side_err_m = 0.008; }, 0.03},
        {"slope + gyro bias 0.05", [](Case& c) { c.cross_slope = 0.04; c.roll_gyro_bias = 0.05; }, 0.03},
        {"turn 0.55 m/s 1.5 rad/s", [](Case& c) { c.drive_m_s = 0.55; c.drive_start_s = 6; c.drive_end_s = 14;
                                                   c.yaw_rad_s = 1.5; c.yaw_t0 = 9; c.yaw_t1 = 12; }, 0.06},
        {"110 mm turn", [](Case& c) { c.heights.push_back({1.0, 0.110}); c.drive_m_s = 0.55;
                                       c.drive_start_s = 10; c.drive_end_s = 18; c.yaw_rad_s = 1.5;
                                       c.yaw_t0 = 13; c.yaw_t1 = 16; c.duration_s = 26; }, 0.06},
    };
    for (int wbm = 0; wbm < 3; ++wbm) {  // tiers, WB, WB + servo vel_ff
      for (const auto& sc : scs) {
        Case c = withMode(Case{}, 2);
        if (wbm) c.algo = V3Controller::Algo::WholeBody;
        if (wbm == 2) c.roll_vff = 1;
        c.duration_s = 20.0;
        c.roll = 1;
        sc.apply(c);
        static char name[64];
        std::snprintf(name, sizeof name, "V29 %s roll, %s", wbm == 2 ? "WB+vff" : wbm ? "WB" : "tiers", sc.name);
        c.name = name;
        const Result r = run(c);
        std::printf("   roll vs apparent: max %.3f end %+.4f, vs gravity max %.3f, wheel load min %.2f\n",
                    r.roll_app_max, r.roll_app_end, r.roll_world_max, r.wheel_load_min);
        std::fflush(stdout);
        assert(!r.fell && r.held_resets == 0 && r.roll_app_max < sc.app_max &&
               std::abs(r.roll_app_end) < 0.005);
      }
    }
  }
  // V40: layered roll as the lateral axis of the gimbal.
  runLayeredRoll(false);
  // V31: the same alternating push regression can be run in isolation with
  // TIER_SUSPENSION_PUSH=1 and remains part of the complete suite below.
  runSuspensionPushRegression();
  // V32: height profile transitions with alternating fore/aft pushes and an
  // identical no-transition position reference.
  runHeightTransitionRegression();
  // V33 / V34: push and standing calm (SET sleash / tdb / tsatm) against the
  // same cases with them off.
  runCalmRegression();
  runHipLeadRegression(false);
  std::puts("v3_tier_tests: all checks passed");
  return 0;
}
