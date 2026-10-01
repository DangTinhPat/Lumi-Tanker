// v3.3 two-tier control on a two-body plant. The v3_physics_tests plant is
// one rigid pendulum (legs locked); here the body (IMU) and the legs are
// separate bodies joined at the hips:
//  - generalized coordinates s = (x axle, theta body pitch, q hip offset);
//    the knee is held at the height pose (prescribed, slow);
//  - masses, CoMs and link geometry from the CAD model (GeneratedGeometry),
//    body CoM shifted by a "true" fore/aft offset (the calibrated one or not);
//  - hips: the GDS68 position servo fitted to the 2026-09-30 HIPTEST log
//    (on the stand: position gain 16 /s, velocity loop 48 /s, ~3 ms delay;
//    rms fit error 3 mrad of 28 mrad), realised here as a torque servo on
//    the real inertia so the floor load (the body) slows it as it would;
//    250 Hz command, ~96 Hz position/velocity feedback;
//  - wheels: speed servo on the wheel-to-shin angle with dead time, lag or a
//    resonant second-order response, whole-rpm command, deadband, torque cap;
//  - the IMU specific force at its body point through the real
//    ImuStateEstimator (balancing config), as AppV3 does.
// Kane's method on numeric Jacobians of the forward kinematics: M(s) s'' +
// J^T (J' s') = Q. Numeric but exact to O(eps^2) for this holonomic chain.
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <random>

#include "robot_control/GeneratedGeometry.hpp"
#include "robot_control/ImuStateEstimator.hpp"
#include "robot_control/WholeBodyModel.hpp"
#include "tap_fw/v3/LegHeight.hpp"
#include "tap_fw/v3/V3Controller.hpp"

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
  double lean_share = -1.0, tier_lead = -1.0, tier_max = -1.0, tier_hpf = -1.0;
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
  // 0.3-0.5) it rings v3.2 at rest too, which the robot does not do.
  int wheel_mode = 0;
  double wheel_tau = 0.09, wheel_wn_hz = 3.0, wheel_zeta = 0.35;
  std::size_t wheel_delay_ticks = 25;  // 50 ms
  double deadband = 1.0, wheel_tau_max = 0.6;
  // Hips (per joint, doubled for two legs).
  double hip_kp = 16.0, hip_vel_bw = 48.0, hip_rotor = 0.004, hip_tau_max = 4.0;
  std::size_t hip_delay_ticks = 2;
  double leg_fb_period_s = 0.0104;  // 0x009 estimate period (~96 Hz)
  double hip_vel_noise = 0.03;      // driver velocity estimate noise [rad/s]
  // Scenario.
  double drive_m_s = 0.0, drive_start_s = 6.0, drive_end_s = 10.0;
  double push_n = 0.0, push_start_s = 6.0, push_end_s = 6.3;  // at the body CoM
  double slope = 0.0;
  double height_low_m = 0.0, height_low_s = 6.0, height_up_s = 16.0;
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
  bool no_rezero = false;    // diagnostics: held and runaway re-zeroes disabled
  // Static level loop (Tier::level): -1 firmware default. The target is
  // the true resting body pitch at the hang pose (as the IMU reads it) plus
  // level_rel, so level_rel = 0 leaves the regression cases' posture as it
  // was; the body deviation metrics are taken from that target.
  int level = -1;
  double level_rel = 0.0;
};

struct Result {
  bool fell = false;
  const char* fault = "";
  int held_resets = 0;
  double body_dev_drive = 0.0;    // max |body pitch - rest| from drive start to end + 1 s
  double body_dev_all = 0.0;      // max |body pitch - rest| after 2 s
  double lean_max = 0.0;          // max |CoM line angle from vertical| after 2 s
  double max_dhip = 0.0, max_hip_rate = 0.0, max_hip_err = 0.0;
  double max_wheel_cmd = 0.0, max_speed = 0.0;
  double rate_rms_drive = 0.0;    // body pitch-rate rms while driving (pecking)
  double body_rms_end = 0.0;      // body pitch - rest, rms over the last 4 s
  double rest_rate_rms = 0.0;     // body pitch-rate rms over the last 4 s (standing calm)
  double rest_hip_rms = 0.0;      // hip rate rms over the last 4 s
  double x_end = 0.0, max_x = 0.0, x_stop_travel = 0.0;
  double max_tau_hip = 0.0;
  double held_max_dhip = 0.0, held_max_wheel = 0.0;  // while on the stand
  double est_err_max = 0.0, est_err_rms = 0.0;  // IMU pitch estimate - truth (after 2 s)
  double grab_detect_s = -1.0;     // grab start -> first held re-zero
  double grab_max_dhip_end = 0.0;  // |tier-2 hip| over the last second of the grab
  double grab_max_wheel = 0.0;     // wheel speed while grabbed [rad/s]
  double putdown_wheel = 0.0;      // wheel speed (rel. to shin) at the grab's put-down [rad/s]
  double putdown_cmd = 0.0;        // wheel command at put-down
  double level_hip_end = 0.0;      // static level loop's hip offset at the end [rad]
  double push_body = 0.0, push_lean = 0.0, push_hip = 0.0;  // peak change in the 2 s after push start
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
};

struct Pt { double x, z; };
Pt rot(Pt p, double a) {
  const double c = std::cos(a), s = std::sin(a);
  return {c * p.x + s * p.z, -s * p.x + c * p.z};
}
Pt add(Pt a, Pt b) { return {a.x + b.x, a.z + b.z}; }
Pt sub(Pt a, Pt b) { return {a.x - b.x, a.z - b.z}; }

// Points in the body frame (hip origin) for a hip/knee offset (core).
struct Points {
  std::array<Pt, 4> com;  // body, thigh, shin, wheel (one leg; legs identical)
  Pt axle, imu;
};

class TwoBody {
 public:
  TwoBody(double body_dx, double mass_scale, double rotor, double hip_cad0)
      : dx_(body_dx), hip_cad0_(hip_cad0), rotor_(rotor) {
    m_[0] = geo::parts[0].mass * mass_scale;
    for (int i = 1; i < 4; ++i) m_[i] = 2.0 * geo::parts[i].mass;
    i_body_ = geo::parts[0].iyy * mass_scale;
    i_legs_ = 2.0 * (geo::parts[1].iyy + geo::parts[2].iyy);
    i_wheel_ = 2.0 * geo::parts[3].iyy;
  }
  double knee = 0.0;

  Points points(double q) const {
    Points p{};
    const double hip = hip_cad0_ + q;
    const double upper = hip, lower = hip + knee + geo::knee_zero_rebase_rad;
    p.com[0] = {geo::parts[0].x + dx_, geo::parts[0].z};
    const Pt knee_pt = rot({geo::origins[1].x, geo::origins[1].z}, upper);
    p.com[1] = rot({geo::parts[1].x, geo::parts[1].z}, upper);
    p.com[2] = add(knee_pt, rot({geo::parts[2].x, geo::parts[2].z}, lower));
    p.axle = add(knee_pt, rot({geo::origins[2].x, geo::origins[2].z}, lower));
    p.com[3] = add(p.axle, rot({geo::parts[3].x, geo::parts[3].z}, lower));
    p.imu = {robot_control::geometry::imu_offset_x_m, robot_control::geometry::imu_offset_z_m};
    return p;
  }
  // World position of body-frame point k (0..3 CoMs, 4 IMU) at state s.
  Pt world(const std::array<double, 3>& s, int k) const {
    const Points p = points(s[2]);
    const Pt local = k < 4 ? p.com[k] : p.imu;
    const Pt w = rot(sub(local, p.axle), s[1]);
    return {s[0] + w.x, geo::wheel_radius_m + w.z};
  }
  // Jacobian columns d world / d s_j and the velocity-product term.
  void kin(const std::array<double, 3>& s, const std::array<double, 3>& sd, int k,
           std::array<Pt, 3>& jac, Pt& jdot_sd) const {
    const double e = 1.0e-6;
    for (int j = 0; j < 3; ++j) {
      auto a = s, b = s;
      a[j] += e;
      b[j] -= e;
      const Pt pa = world(a, k), pb = world(b, k);
      jac[j] = {(pa.x - pb.x) / (2 * e), (pa.z - pb.z) / (2 * e)};
    }
    const double h = 1.0e-4;
    auto a = s, b = s;
    for (int j = 0; j < 3; ++j) { a[j] += h * sd[j]; b[j] -= h * sd[j]; }
    const Pt p0 = world(s, k), pa = world(a, k), pb = world(b, k);
    jdot_sd = {(pa.x - 2 * p0.x + pb.x) / (h * h), (pa.z - 2 * p0.z + pb.z) / (h * h)};
  }
  // Accelerations for wheel torque tw (on the wheel-to-shin angle), hip
  // torque th (on q) and a horizontal push fx at the body CoM.
  std::array<double, 3> accel(const std::array<double, 3>& s, const std::array<double, 3>& sd,
                              double tw, double th, double fx, double slope) const {
    double M[3][3] = {};
    double Q[3] = {};
    const double gx = -kG * std::sin(slope), gz = -kG * std::cos(slope);
    for (int k = 0; k < 4; ++k) {
      std::array<Pt, 3> J;
      Pt jd;
      kin(s, sd, k, J, jd);
      for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) M[a][b] += m_[k] * (J[a].x * J[b].x + J[a].z * J[b].z);
        Q[a] += m_[k] * (J[a].x * (gx - jd.x) + J[a].z * (gz - jd.z));
        if (k == 0) Q[a] += J[a].x * fx;
      }
    }
    const double R = geo::wheel_radius_m;
    // Rotational inertia: body (theta), leg links (theta + q), wheel rotors
    // (x / R), hip rotors through the gear (q).
    M[1][1] += i_body_;
    for (int a : {1, 2}) for (int b : {1, 2}) M[a][b] += i_legs_;
    M[0][0] += i_wheel_ / (R * R);
    M[2][2] += 2.0 * rotor_;
    // Wheel torque does work on u = x / R - theta - q (knee fixed).
    Q[0] += tw / R; Q[1] -= tw; Q[2] -= tw;
    Q[2] += th;
    // Solve 3x3 (Cramer).
    const double det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
        M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
        M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    std::array<double, 3> out{};
    for (int c = 0; c < 3; ++c) {
      double A[3][3];
      for (int r = 0; r < 3; ++r) for (int k = 0; k < 3; ++k) A[r][k] = k == c ? Q[r] : M[r][k];
      out[c] = (A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0])) / det;
    }
    return out;
  }
  // Kinetic + potential energy (plant self-check).
  double energy(const std::array<double, 3>& s, const std::array<double, 3>& sd) const {
    double e = 0.0;
    for (int k = 0; k < 4; ++k) {
      std::array<Pt, 3> J;
      Pt jd;
      kin(s, sd, k, J, jd);
      double vx = 0, vz = 0;
      for (int j = 0; j < 3; ++j) { vx += J[j].x * sd[j]; vz += J[j].z * sd[j]; }
      e += 0.5 * m_[k] * (vx * vx + vz * vz) + m_[k] * kG * world(s, k).z;
    }
    const double R = geo::wheel_radius_m;
    e += 0.5 * i_body_ * sd[1] * sd[1] + 0.5 * i_legs_ * (sd[1] + sd[2]) * (sd[1] + sd[2]) +
        0.5 * i_wheel_ * sd[0] * sd[0] / (R * R) + 0.5 * 2.0 * rotor_ * sd[2] * sd[2];
    return e;
  }
  // Angle of the axle -> CoM line from vertical (world), true masses.
  double leanAngle(const std::array<double, 3>& s) const {
    double mx = 0, mz = 0, mt = 0;
    for (int k = 0; k < 4; ++k) {
      const Pt w = world(s, k);
      mx += m_[k] * (w.x - s[0]);
      mz += m_[k] * (w.z - geo::wheel_radius_m);
      mt += m_[k];
    }
    return std::atan2(mx / mt, mz / mt);
  }

 private:
  double dx_, hip_cad0_, rotor_;
  std::array<double, 4> m_{};
  double i_body_ = 0, i_legs_ = 0, i_wheel_ = 0;
};

Result simulate(const Case& c) {
  const double dt_c = 0.002;
  const int sub = 8;
  const double dt = dt_c / sub;
  const double R = geo::wheel_radius_m;
  TwoBody plant(c.body_dx, c.mass_scale, c.hip_rotor, LegHeight::hangHipCad());
  std::mt19937 rng(7);
  std::normal_distribution<double> n_acc(0, 0.05), n_gyr(0, 0.005), n_w(0, 0.3),
      n_q(0, 0.0005), n_qd(0, 0.03);

  // Stand at the hang pose, CoM over the axle plus the seed error.
  std::array<double, 3> s{0.0, 0.0, 0.0}, sd{0.0, 0.0, 0.0};
  for (int i = 0; i < 40; ++i) s[1] -= plant.leanAngle(s) - c.seed_error;
  const double theta_rest0 = s[1] - c.seed_error;  // body pitch at balance, hang pose

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
  cfg.tier.virtual_pitch = c.vpitch;
  cfg.tier.trunk = c.trunk;
  if (c.lean_share >= 0) cfg.tier.lean_share = static_cast<float>(c.lean_share);
  if (c.tier_lead >= 0) cfg.tier.lead_s = static_cast<float>(c.tier_lead);
  if (c.tier_max >= 0) cfg.tier.max_rad = static_cast<float>(c.tier_max);
  if (c.tier_hpf >= 0) cfg.tier.hpf_s = static_cast<float>(c.tier_hpf);
  if (c.tier_exp >= 0) cfg.tier.height_exp = cfg.tier.height_exp_tall = static_cast<float>(c.tier_exp);
  if (const char* v = std::getenv("TIER_EXP_TALL")) cfg.tier.height_exp_tall = static_cast<float>(std::atof(v));
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
  cfg.tier.level_pitch_rad = static_cast<float>(theta_rest0 + c.imu_mount + c.level_rel);
  const bool level_active = c.vpitch && c.trunk && cfg.tier.level;
  if (const char* v = std::getenv("TIER_PVMAX")) cfg.tier.plan_vmax_m_s = static_cast<float>(std::atof(v));
  if (c.joint_rate_gain >= 0) cfg.tier.joint_rate_gain = static_cast<float>(c.joint_rate_gain);
  V3Controller ctrl(cfg);
  V3Controller::Input in{};
  V3Controller::Command cmd{};
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
  est.setConfig(tap_fw::v3::balancingEstimatorConfig(base));

  Result r;
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
    s[2] = q;
    held_wheel_pos = wp - q;  // encoder = x / R - theta - q (x = 0 here)
    for (int i = 0; i < 40; ++i) s[1] -= plant.leanAngle(s) - c.seed_error;
  }
  std::deque<double> wheel_q, hip_q;
  double wheel_applied = 0.0, hip_applied = 0.0;
  double u = held_wheel_pos, ud = 0.0;  // wheel-to-shin angle and rate
  double ws_acc = 0.0;              // resonant servo acceleration state
  double hip_integ = 0.0;
  double q_meas = 0.0, qd_meas = 0.0, next_leg_fb = 0.0;
  double hip_zoh = 0.0, next_hip_cmd = 0.0;
  double s_acc[3] = {0, 0, 0};
  const double kv_joint = c.hip_vel_bw * (geo::leg_hip_inertia_kgm2 + c.hip_rotor);
  double rate_sq = 0.0, body_sq = 0.0, x_stop = 0.0, rest_rate_sq = 0.0, rest_hip_sq = 0.0;
  double est_sq = 0.0, est_bias0 = 0.0;
  double ab_f = 0.0, ev_acc_sq = 0.0, ev_jerk_sq = 0.0, v_release = 0.0, v_start = 0.0;
  int ev_n = 0;
  int est_n = 0, held_before_grab = 0;
  bool put_down = false, grab_biased = false;
  double grab_from = 0.0, grab_to = 0.0;
  double push_b0 = 0.0, push_l0 = 0.0, push_q0 = 0.0;
  double put_down_t = 0.0;
  int rate_n = 0, body_n = 0;
  for (int k = 0; k < static_cast<int>(c.duration_s / dt_c); ++k) {
    const double t = k * dt_c;
    cmd.forward_m_s = static_cast<float>(
        (t >= c.drive_start_s && t < c.drive_end_s) ? c.drive_m_s : 0.0);
    if (c.trunk_on_s >= 0.0 && std::abs(t - c.trunk_on_s) < 1e-9) ctrl.config().tier.trunk = true;
    if (c.trunk_off_s >= 0.0 && std::abs(t - c.trunk_off_s) < 1e-9) ctrl.config().tier.trunk = false;
    if (c.height_low_m > 0.0) {
      cmd.height_m = static_cast<float>(t < c.height_low_s ? 0.0
          : t < c.height_up_s ? c.height_low_m : LegHeight::maxHeight());
    }
    // Knee follows the height pose (slow, prescribed).
    plant.knee += (ctrl.legPose().knee_offset_rad - plant.knee) * std::min(dt_c / 0.03, 1.0);
    // Leg feedback (0x009) at ~96 Hz.
    if (t >= next_leg_fb) {
      next_leg_fb = t + c.leg_fb_period_s;
      q_meas = s[2] + n_q(rng);
      qd_meas = sd[2] + n_qd(rng) * (c.hip_vel_noise / 0.03);
    }
    in.now_us = t0_us + static_cast<std::uint64_t>(k) * 2000U;
    in.pitch_rad = c.ideal_imu ? static_cast<float>(s[1] + c.imu_mount) : imu.pitch_rad;
    in.pitch_rate_rad_s = c.ideal_imu ? static_cast<float>(sd[1]) : imu.pitch_rate_rad_s;
    in.accel_x_m_s2 = last_accel[0];
    in.accel_z_m_s2 = last_accel[2];
    const double wm = ud + n_w(rng);
    in.wheel_rad_s = {static_cast<float>(wm), static_cast<float>(wm)};
    in.wheel_pos_rad = {static_cast<float>(u), static_cast<float>(u)};
    const float kq = static_cast<float>(plant.knee);
    in.hip_rad = {static_cast<float>(q_meas + 0.004), static_cast<float>(q_meas - 0.004)};
    in.knee_rad = {kq, kq};
    in.hip_rate_rad_s = {static_cast<float>(qd_meas), static_cast<float>(qd_meas)};
    in.knee_rate_rad_s = {0.0F, 0.0F};
    in.leg_chain_rad = static_cast<float>(q_meas + plant.knee);
    const auto out = ctrl.update(in, cmd);
    {
      const auto v = ctrl.imuVelocity();
      kin = {v.horizontal_m_s, v.vertical_m_s, v.valid};
    }
    cmd.go = false;
    if (out.state == V3Controller::State::Fault) {
      r.fell = true;
      r.fault = ctrl.lastFault();
      break;
    }
    r.max_wheel_cmd = std::max({r.max_wheel_cmd, std::abs(double(out.wheel_cmd_rad_s[0])),
                                std::abs(double(out.wheel_cmd_rad_s[1]))});
    wheel_q.push_back(0.5 * (out.wheel_cmd_rad_s[0] + out.wheel_cmd_rad_s[1]));
    if (wheel_q.size() > c.wheel_delay_ticks) { wheel_applied = wheel_q.front(); wheel_q.pop_front(); }
    // Hip command: LegLock resends at 250 Hz, then CAN + driver delay.
    if (t >= next_hip_cmd) {
      next_hip_cmd = t + 0.004;
      hip_zoh = ctrl.hipCommand();
    }
    hip_q.push_back(hip_zoh);
    if (hip_q.size() > c.hip_delay_ticks) { hip_applied = hip_q.front(); hip_q.pop_front(); }
    r.max_dhip = std::max(r.max_dhip, std::abs(double(ctrl.trunkHip())));

    const double fx = (t >= c.push_start_s && t < c.push_end_s) ? c.push_n : 0.0;
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
      sd = {0.0, 0.0, sd[2]};
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
        const double acc = c.hip_vel_bw * (c.hip_kp * (hip_applied - s[2]) - sd[2]);
        sd[2] += acc * dt;
        s[2] += sd[2] * dt;
      }
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
      sd = {0.0, 0.0, 0.0};
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
      const double vel_sp = c.hip_kp * (hip_applied - s[2]);
      const double vel_err = vel_sp - sd[2];
      hip_integ = std::clamp(hip_integ + 2.0 * kv_joint * 2.0 * vel_err * dt, -2.0, 2.0);
      const double tau_h = std::clamp(2.0 * kv_joint * vel_err + hip_integ,
                                      -2.0 * c.hip_tau_max, 2.0 * c.hip_tau_max);
      r.max_tau_hip = std::max(r.max_tau_hip, std::abs(tau_h));
      // Wheel torque that realises udd_des (linear in the torque), capped.
      const auto a0 = plant.accel(s, sd, 0.0, tau_h, fx, c.slope);
      const auto a1 = plant.accel(s, sd, 1.0, tau_h, fx, c.slope);
      const double u0 = a0[0] / R - a0[1] - a0[2], u1 = a1[0] / R - a1[1] - a1[2];
      const double tw = std::clamp((udd_des - u0) / (u1 - u0), -c.wheel_tau_max, c.wheel_tau_max);
      std::array<double, 3> a{};
      for (int j = 0; j < 3; ++j) a[j] = a0[j] + tw * (a1[j] - a0[j]);
      for (int j = 0; j < 3; ++j) { sd[j] += a[j] * dt; s[j] += sd[j] * dt; s_acc[j] = a[j]; }
      const double udd = a[0] / R - a[1] - a[2];
      if (c.wheel_mode == 1) ws_acc = udd;  // the servo state follows the real wheel
      ud += udd * dt;
      u += ud * dt;
    }
    // IMU specific force at its body point.
    {
      std::array<Pt, 3> J;
      Pt jd;
      plant.kin(s, sd, 4, J, jd);
      double ax = jd.x, az = jd.z;
      for (int j = 0; j < 3; ++j) { ax += J[j].x * s_acc[j]; az += J[j].z * s_acc[j]; }
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
      std::array<Pt, 3> J;
      Pt jd;
      plant.kin(s, sd, 0, J, jd);
      ab = jd.x;
      for (int j = 0; j < 3; ++j) { vb += J[j].x * sd[j]; ab += J[j].x * s_acc[j]; }
    }
    {
      const double a_prev = ab_f;
      ab_f += std::min(2.0 * kPi * 20.0 * dt_c, 1.0) * (ab - ab_f);  // 20 Hz: drop substep noise
      const double jerk = (ab_f - a_prev) / dt_c;
      const bool drive_ev = c.drive_m_s != 0.0 && t >= c.drive_start_s && t < c.drive_end_s + 3.0;
      const bool push_ev = c.push_n != 0.0 && t >= c.push_start_s && t < c.push_start_s + 2.0;
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
    if (c.push_n != 0.0) {
      if (t < c.push_start_s) { push_b0 = s[1]; push_l0 = lean; push_q0 = s[2]; }
      else if (t < c.push_start_s + 2.0) {
        r.push_body = std::max(r.push_body, std::abs(s[1] - push_b0));
        r.push_lean = std::max(r.push_lean, std::abs(lean - push_l0));
        r.push_hip = std::max(r.push_hip, std::abs(s[2] - push_q0));
      }
    }
    if (t > 2.0) {
      if (std::abs(dev) > 0.015) r.level_settle_s = t;
      r.body_dev_all = std::max(r.body_dev_all, std::abs(dev));
      r.lean_max = std::max(r.lean_max, std::abs(lean));
      r.max_hip_rate = std::max(r.max_hip_rate, std::abs(sd[2]));
      r.max_hip_err = std::max(r.max_hip_err, std::abs(hip_applied - s[2]));
    }
    if (c.drive_m_s != 0.0 && t >= c.drive_start_s && t < c.drive_end_s + 1.0) {
      r.body_dev_drive = std::max(r.body_dev_drive, std::abs(dev));
      rate_sq += sd[1] * sd[1];
      ++rate_n;
    }
    if (t > c.duration_s - 4.0) {
      body_sq += dev * dev;
      rest_rate_sq += sd[1] * sd[1];
      rest_hip_sq += sd[2] * sd[2];
      ++body_n;
    }
    const int trace_every = std::getenv("TR_FINE") ? 1 : 25;
    if (c.trace && k % trace_every == 0) {
      std::printf("TRC t=%.2f x=%+.3f v=%+.3f body=%+.4f lean=%+.4f q=%+.4f qcmd=%+.4f tv=%+.4f "
                  "ref=%+.4f wcmd=%+.2f ta=%.2f lvl=%+.4f learn=%d/%+.4f est=%+.4f\n", t, s[0], sd[0], dev,
                  lean, s[2], double(ctrl.hipCommand()), double(ctrl.virtualPitch()),
                  double(ctrl.balanceRef()), 0.5 * (out.wheel_cmd_rad_s[0] + out.wheel_cmd_rad_s[1]),
                  double(ctrl.trunkAuthority()), double(ctrl.levelHip()), ctrl.learnedTrimValid(),
                  double(ctrl.learnedTrim()),
                  double(imu.pitch_rad) - (s[1] + c.imu_mount) - est_bias0);
    }
    r.max_speed = std::max(r.max_speed, std::abs(sd[0]));
    r.max_x = std::max(r.max_x, std::abs(s[0]));
    if (k == static_cast<int>((c.drive_end_s + 3.0) / dt_c)) x_stop = s[0];
  }
  r.level_hip_end = ctrl.levelHip();
  r.learned_valid = ctrl.learnedTrimValid();
  r.x_end = s[0];
  r.x_stop_travel = s[0] - x_stop;
  r.rate_rms_drive = rate_n ? std::sqrt(rate_sq / rate_n) : 0.0;
  r.body_rms_end = body_n ? std::sqrt(body_sq / body_n) : 0.0;
  r.rest_rate_rms = body_n ? std::sqrt(rest_rate_sq / body_n) : 0.0;
  r.rest_hip_rms = body_n ? std::sqrt(rest_hip_sq / body_n) : 0.0;
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

// Plant variants around the fitted/CAD values the tiers must tolerate.
struct Variant { const char* name; void (*apply)(Case&); };
const Variant kVariants[] = {
    {"nominal", [](Case&) {}},
    {"wheel lag .06", [](Case& c) { c.wheel_tau = 0.06; }},
    {"wheel lag .12", [](Case& c) { c.wheel_tau = 0.12; }},
    {"wheel delay 70ms", [](Case& c) { c.wheel_delay_ticks = 35; }},
    {"hip slow kp10 bw30", [](Case& c) { c.hip_kp = 10; c.hip_vel_bw = 30; }},
    {"hip rotor x3", [](Case& c) { c.hip_rotor = 0.012; }},
    {"hip delay +10ms", [](Case& c) { c.hip_delay_ticks = 7; }},
    {"hip torque 1.5", [](Case& c) { c.hip_tau_max = 1.5; }},
    {"body dx 0 (model err)", [](Case& c) { c.body_dx = 0.0; }},
    {"body dx .045", [](Case& c) { c.body_dx = 0.045; }},
    {"imu mount -.08", [](Case& c) { c.imu_mount = -0.08; }},
    {"body mass x1.2", [](Case& c) { c.mass_scale = 1.2; }},
    {"fast wheel no db", [](Case& c) { c.wheel_delay_ticks = 3; c.deadband = 0; c.wheel_tau = 0.05; }},
    {"hip vel noise 0.2", [](Case& c) { c.hip_vel_noise = 0.2; }},
    {"leg feedback 50 Hz", [](Case& c) { c.leg_fb_period_s = 0.02; }},
};

}  // namespace

int main() {
  // V1: plant self-check - passive motion (no torques, free hip) conserves
  // energy, so the numeric Kane equations are consistent.
  {
    TwoBody p(0.021, 1.0, 0.004, LegHeight::hangHipCad());
    std::array<double, 3> s{0.0, 0.3, -0.2}, sd{0.2, 0.5, 1.5};
    const double e0 = p.energy(s, sd);
    const double dt = 1.0e-5;
    for (int i = 0; i < 50000; ++i) {  // 0.5 s, midpoint
      const auto a = p.accel(s, sd, 0.0, 0.0, 0.0, 0.0);
      std::array<double, 3> sm, sdm;
      for (int j = 0; j < 3; ++j) { sm[j] = s[j] + 0.5 * dt * sd[j]; sdm[j] = sd[j] + 0.5 * dt * a[j]; }
      const auto am = p.accel(sm, sdm, 0.0, 0.0, 0.0, 0.0);
      for (int j = 0; j < 3; ++j) { s[j] += dt * sdm[j]; sd[j] += dt * am[j]; }
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
    if (const char* v = std::getenv("TR_LVL")) c.level = std::atoi(v);
    if (const char* v = std::getenv("TR_DUR")) c.duration_s = std::atof(v);
    if (const char* v = std::getenv("TR_SEED")) c.seed_error = std::atof(v);
    if (const char* h = std::getenv("TR_H")) {
      c.height_low_m = std::atof(h); c.height_low_s = 5; c.height_up_s = 17; c.duration_s = 30;
    }
    c.name = "trace";
    run(c);
    return 0;
  }
  if (std::getenv("TIER_RATIO") != nullptr) {
    const double hang = LegHeight::forHeight(LegHeight::maxHeight()).com_above_axle_m;
    for (double h : {0.189, 0.170, 0.150, 0.136, 0.120, 0.110, 0.095, 0.078}) {
      const auto p = LegHeight::forHeight(static_cast<float>(h));
      const double r = p.com_above_axle_m / hang;
      std::printf("h %3.0f mm: com %.1f mm ratio %.3f  share^1.5 %.2f  ^1.0 %.2f  ^0.5 %.2f\n", h * 1000,
                  p.com_above_axle_m * 1000, r, std::pow(r, 1.5), r, std::sqrt(r));
    }
    return 0;
  }
  if (std::getenv("TIER_PUSHSWEEP") != nullptr) {
    // Hard pushes (wheels saturated, hardware 2026-09-30 182 s: 1.1 m/s, held
    // re-zero mid-catch) and hand pushes (force held on for up to 1 s).
    const double cap = std::getenv("PS_CAP") ? std::atof(std::getenv("PS_CAP")) : -1.0;
    const bool norz = std::getenv("PS_NOREZERO") != nullptr;
    for (double h : {0.0, 0.136}) {
      for (int mode : {0, 2}) {
        for (double len : {0.3, 0.6, 1.0}) {
          std::printf("h %s %-13s %.1f s:", h > 0 ? "136" : "189", modeName(mode), len);
          for (double f : {4.0, 6.0, 8.0, 10.0, 12.0, 14.0}) {
            Case c = withMode(Case{}, mode);
            c.held_cap = cap;
            c.no_rezero = norz;
            if (h > 0) { c.height_low_m = h; c.height_low_s = 3.0; c.height_up_s = 60.0; }
            c.push_n = f; c.push_start_s = 12.0; c.push_end_s = 12.0 + len; c.duration_s = 22.0;
            const Result r = simulate(c);
            std::printf("  %2.0fN:%s h%d v%.2f", f, r.fell ? "FALL" : "ok  ", r.held_resets, r.max_speed);
          }
          std::printf("\n");
        }
      }
    }
    return 0;
  }
  if (std::getenv("TIER_EXP") != nullptr) {
    // Tier-2 share vs height (Tier::height_exp): pushed at a crouch height.
    for (double h : {0.136, 0.110, 0.078}) {
      for (double e : {-1.0, 1.5}) {
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
    // the static HIPTEST jitter (~0.05-0.1 rad/s) keeps weight 1 ahead of v3.2.
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
  // v3.2 and tier 2 at least halves the body pitch excursion of driving and
  // pushing (v3.2: ~0.09 rad; the legs take the lean).
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
      assert(r[1].rest_rate_rms <= r[0].rest_rate_rms + 0.01);
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
  // V5: height 188 -> 110 / 78 mm and back while balancing. v3.2 on the
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
    assert(r[1].rest_rate_rms < 0.2 && r[2].rest_rate_rms < 0.2);
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
    for (double tau : {0.06, 0.12}) {  // the case that fell, and its neighbour
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
  // moment - found to fall in this exact case (2026-10-01) where v3.2 alone
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
    // without the virtual pitch (v3.2 unchanged).
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
    c.level_rel = 0.16;
    c.push_n = 4.0;
    c.name = "V17 level +0.160 (past the limit)";
    const Result r = run(c);
    assert(!r.fell && std::abs(r.level_hip_end) <= 0.1201);
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
        c.push_n = 4.0; c.push_start_s = 12.0; c.push_end_s = 12.0 + len; c.duration_s = 20.0;
        static char name[64];
        std::snprintf(name, sizeof name, "V19 push 4 N x %.1f s at %s", len, h > 0 ? "136" : "189");
        c.name = name;
        const Result r = run(c);
        assert(!r.fell && r.held_resets == 0);
      }
    }
  }
  std::puts("v3_tier_tests: all checks passed");
  return 0;
}
