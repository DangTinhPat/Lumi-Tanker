// v3 balance on a physical plant: planar wheeled pendulum (Lagrange, CAD mass
// estimates), velocity-servo wheels with lag/torque cap/command delay, and the
// accelerometer specific force at the IMU point fed through the real
// ImuStateEstimator exactly as AppV3 does. Guards the finding that the shipped
// estimator config goes blind on the floor (falls) and that the balancing
// config + PID with a position integrator stands and finds the balance angle.
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <deque>
#include <random>

#include "robot_control/ImuStateEstimator.hpp"
#include "tap_fw/v3/V3Controller.hpp"

namespace {

using robot_control::ImuStateEstimator;
using tap_fw::v3::V3Controller;

struct Case {
  // Plant uncertainty around the CAD estimates, and the wheel command delay.
  double mass_scale = 1.0, inertia_scale = 1.0, l_scale = 1.0;
  std::size_t delay_ticks = 2;  // 2 ms control ticks
  double eq = 0.10;          // body pitch with the COM over the axle
  double seed_error = 0.035; // pitch held at engage minus eq
  double tau_s = 0.08;       // wheel speed servo lag
  double slope = 0.0;        // floor slope, + = uphill forward
  double push_n = 0.0;       // horizontal push at 0.25 m, 10..40 s
  double bias_error = 0.0;   // gyro bias change after the estimator seeded
  bool balancing_estimator = true;
  // Operator flow: balance loop engaged on the stand (body held at
  // stand_pitch, wheels in the air) for held_s, then held on the floor at the
  // release pitch for 1 s, then let go.
  double held_s = 0.0;
  double stand_pitch = -0.02;
  double duration_s = 40.0;
};

struct Result {
  bool fell = false;
  double x_end = 0.0, max_x = 0.0, th_rms = 0.0, theta_b = 0.0, theta_b_15s = 0.0;
  int held_resets = 0;  // a free robot must never look held
};

Result simulate(const Case& c) {
  const double g = 9.80665, m = 4.181 * c.mass_scale, l = 0.191 * c.l_scale,
               J = 0.178 * c.mass_scale * c.inertia_scale * c.l_scale * c.l_scale,
               Mw = 0.44, Iw = 0.00029,
               R = 0.04, h_imu = 0.224, tau_max = 0.6, gyro_bias = -0.05;
  const double Mp = Mw + Iw / (R * R) + m;
  const double dt_c = 0.002;
  const int sub = 4;
  const double dt = dt_c / sub;
  double x = 0, xd = 0, th = c.eq + c.seed_error, thd = 0, phid = 0, phi = 0, xdd = 0, thdd = 0;
  std::mt19937 rng(11);
  std::normal_distribution<double> n_acc(0, 0.05), n_gyr(0, 0.005), n_w(0, 0.3);

  const ImuStateEstimator::Config base{};
  ImuStateEstimator est(base);
  ImuStateEstimator::Output imu{};
  std::array<float, 3> last_accel{0.0F, 0.0F, 9.80665F};
  bool seeded = false;
  float t_s = 0.0F;
  auto feed = [&](double ax, double az, double gy) {
    const std::array<float, 3> a{static_cast<float>(ax + n_acc(rng)), 0.0F,
                                 static_cast<float>(az + n_acc(rng))};
    const std::array<float, 3> w{0.0F, static_cast<float>(gy + gyro_bias +
                                     (seeded ? c.bias_error : 0.0) + n_gyr(rng)), 0.0F};
    float as = 0, ws = 0;
    for (int i = 0; i < 3; ++i) { as += a[i] * a[i]; ws += w[i] * w[i]; }
    const bool quiet = std::abs(std::sqrt(as) - 9.80665F) < 0.35F && std::sqrt(ws) < 0.12F;
    last_accel = a;
    t_s += static_cast<float>(dt_c);
    const auto out = est.update(a, w, static_cast<float>(dt_c), t_s, 0.0F, false, quiet, {});
    if (out.valid) imu = out;
  };
  // Held still at the standing point (Ready): the estimator settles.
  for (int i = 0; i < 1500; ++i) feed(-g * std::sin(th), g * std::cos(th), 0.0);
  seeded = true;

  V3Controller ctrl(tap_fw::v3::defaultV3Config());
  V3Controller::Input in{};
  V3Controller::Command cmd{};
  in.legs_ready = true;
  if (c.held_s > 0.0) {
    // Re-seed the estimator at the stand pitch.
    for (int i = 0; i < 1500; ++i) {
      feed(-g * std::sin(c.stand_pitch), g * std::cos(c.stand_pitch), 0.0);
    }
  }
  in.pitch_rad = imu.pitch_rad;
  cmd.start = true;
  (void)ctrl.update(in, cmd);
  cmd.start = false;
  in.now_us = 1'300'000U;
  (void)ctrl.update(in, cmd);  // -> Ready
  assert(ctrl.state() == V3Controller::State::Ready);
  cmd.go = true;               // released at the held pitch
  if (c.balancing_estimator) est.setConfig(tap_fw::v3::balancingEstimatorConfig(base));
  std::uint64_t t_us = 1'302'000U;
  if (c.held_s > 0.0) {
    // Body held (stand, then hands on the floor): wheels spin freely.
    double wheel = 0.0, pos = 0.0;
    const int n_held = static_cast<int>((c.held_s + 1.0) / dt_c);
    for (int k = 0; k < n_held; ++k, t_us += 2000U) {
      // Carried from the stand pitch to the floor pitch over 0.5 s (the gyro
      // sees the rotation), then held still.
      const double tk = k * dt_c, ramp = 0.5;
      const double a = std::clamp((tk - (c.held_s - ramp)) / ramp, 0.0, 1.0);
      const double held_pitch = c.stand_pitch + a * (th - c.stand_pitch);
      const double held_rate = (a > 0.0 && a < 1.0) ? (th - c.stand_pitch) / ramp : 0.0;
      feed(-g * std::sin(held_pitch), g * std::cos(held_pitch), held_rate);
      in.now_us = t_us;
      in.pitch_rad = imu.pitch_rad;
      in.pitch_rate_rad_s = imu.pitch_rate_rad_s;
      in.accel_x_m_s2 = last_accel[0];
      in.accel_z_m_s2 = last_accel[2];
      in.wheel_rad_s = {static_cast<float>(wheel), static_cast<float>(wheel)};
      in.wheel_pos_rad = {static_cast<float>(pos), static_cast<float>(pos)};
      const auto out = ctrl.update(in, cmd);
      cmd.go = false;
      if (out.state == V3Controller::State::Fault) {
        std::printf("  held-phase fault at %.2f s: %s\n", k * dt_c, ctrl.lastFault());
        Result r; r.fell = true; return r;
      }
      const double target = 0.5 * (out.wheel_cmd_rad_s[0] + out.wheel_cmd_rad_s[1]);
      wheel += (target - wheel) * dt_c / c.tau_s;
      pos += wheel * dt_c;
    }
    phi = pos;
  }

  std::deque<double> queue;
  double applied = 0.0, sum_th2 = 0.0;
  int n_rms = 0;
  Result r;
  for (int k = 0; k < static_cast<int>(c.duration_s / dt_c); ++k) {
    const double t = k * dt_c;
    in.now_us = t_us + static_cast<std::uint64_t>(k) * 2000U;
    in.pitch_rad = imu.pitch_rad;
    in.pitch_rate_rad_s = imu.pitch_rate_rad_s;
    in.accel_x_m_s2 = last_accel[0];
    in.accel_z_m_s2 = last_accel[2];
    const double wm = phid + n_w(rng);
    in.wheel_rad_s = {static_cast<float>(wm), static_cast<float>(wm)};
    in.wheel_pos_rad = {static_cast<float>(phi), static_cast<float>(phi)};
    const auto out = ctrl.update(in, cmd);
    cmd.go = false;
    if (out.state == V3Controller::State::Fault) { r.fell = true; break; }
    queue.push_back(0.5 * (out.wheel_cmd_rad_s[0] + out.wheel_cmd_rad_s[1]));
    if (queue.size() > c.delay_ticks) { applied = queue.front(); queue.pop_front(); }

    for (int s = 0; s < sub; ++s) {
      const double psi = th - c.eq - c.slope, cs = std::cos(th - c.eq), sn = std::sin(th - c.eq);
      const double fx = (t >= 10.0 && t < 40.0) ? c.push_n : 0.0;
      const double gx = -g * std::sin(c.slope);
      auto solve = [&](double tau) {
        const double a11 = Mp, a12 = m * l * cs;
        const double b1 = tau / R + m * l * sn * thd * thd + fx + (m + Mw) * gx;
        const double a21 = m * l * cs, a22 = J;
        const double b2 = m * g * l * std::sin(psi) - tau + fx * 0.25 * std::cos(th);
        const double det = a11 * a22 - a12 * a21;
        xdd = (b1 * a22 - a12 * b2) / det;
        thdd = (a11 * b2 - a21 * b1) / det;
      };
      double phidd = (applied - phid) / c.tau_s;
      solve(0.0);
      const double x0 = xdd, t0 = thdd;
      solve(1.0);
      const double dx = xdd - x0, dth = thdd - t0;
      const double tau = std::clamp((phidd - (x0 / R - t0)) / (dx / R - dth), -tau_max, tau_max);
      solve(tau);
      phidd = xdd / R - thdd;
      xd += xdd * dt; x += xd * dt; thd += thdd * dt; th += thd * dt;
      phid += phidd * dt; phi += phid * dt;
    }
    const double cth = std::cos(th), sth = std::sin(th);
    const double axw = xdd + h_imu * (cth * thdd - sth * thd * thd);
    const double azw = -h_imu * (sth * thdd + cth * thd * thd);
    const double fxw = axw + g * std::sin(c.slope), fzw = azw + g * std::cos(c.slope);
    feed(fxw * cth - fzw * sth, fxw * sth + fzw * cth, thd);

    if (std::abs(th - c.eq) > 0.6) { r.fell = true; break; }
    if (t > c.duration_s - 10.0) { sum_th2 += (th - c.eq) * (th - c.eq); ++n_rms; }
    if (k == static_cast<int>(15.0 / dt_c)) r.theta_b_15s = ctrl.balanceRef();
    r.max_x = std::max(r.max_x, std::abs(x));
  }
  r.x_end = x;
  r.th_rms = n_rms > 0 ? std::sqrt(sum_th2 / n_rms) : -1.0;
  r.theta_b = ctrl.balanceRef();
  r.held_resets = ctrl.heldResets();
  return r;
}

void report(const char* name, const Case& c, const Result& r) {
  std::printf("%-28s eq %+.3f tau %.2f: fell=%d x_end=%+.3f max|x|=%.2f th_rms=%.4f "
              "theta_b=%+.4f (15 s %+.4f) held=%d\n", name, c.eq, c.tau_s, r.fell, r.x_end,
              r.max_x, r.th_rms, r.theta_b, r.theta_b_15s, r.held_resets);
}

}  // namespace

int main() {
  // T1: the shipped estimator config on the floor goes blind and falls.
  {
    Case c;
    c.balancing_estimator = false;
    c.duration_s = 10.0;
    const auto r = simulate(c);
    report("T1 shipped estimator", c, r);
    assert(r.fell);
  }
  // T2: balance angles +0.1/0/-0.1, held 2 deg off, wheel lag 80/120 ms.
  for (double tau : {0.08, 0.12}) {
    for (double eq : {0.10, 0.0, -0.10}) {
      for (double err : {0.035, -0.035}) {
        Case c;
        c.eq = eq;
        c.seed_error = err;
        c.tau_s = tau;
        const auto r = simulate(c);
        report("T2 balance", c, r);
        assert(!r.fell);
        assert(std::abs(r.x_end) < 0.02);
        assert(r.th_rms < 0.002);
        assert(std::abs(r.theta_b - eq) < 0.0087);
        assert(r.held_resets == 0);
      }
    }
  }
  // T3: held 4.6 deg off the balance angle.
  for (double err : {0.08, -0.08}) {
    Case c;
    c.seed_error = err;
    const auto r = simulate(c);
    report("T3 large seed error", c, r);
    assert(!r.fell && r.max_x < 1.5);
  }
  // T4: gyro bias drifts 0.01 rad/s after release.
  {
    Case c;
    c.bias_error = 0.01;
    const auto r = simulate(c);
    report("T4 gyro bias drift", c, r);
    assert(!r.fell && std::abs(r.x_end) < 0.02);
  }
  // T5: 1 N push for 30 s, 20 s to recover.
  {
    Case c;
    c.push_n = 1.0;
    c.duration_s = 60.0;
    const auto r = simulate(c);
    report("T5 push 1 N", c, r);
    assert(!r.fell && std::abs(r.x_end) < 0.05);
  }
  // T6: 0.03 rad floor slope.
  {
    Case c;
    c.slope = 0.03;
    const auto r = simulate(c);
    report("T6 slope", c, r);
    assert(!r.fell && std::abs(r.x_end) < 0.02);
  }
  // T7: robustness of the default gains to the unknowns: mass +-30 %,
  // inertia -30/+40 %, COM height +-20 %, wheel lag 0.1/0.15 s, command
  // delay 4/20 ms, held 3 deg off. Every plant must stand without ringing.
  {
    int n = 0;
    for (double ms : {0.7, 1.3}) for (double is : {0.7, 1.4}) for (double ls : {0.8, 1.2})
    for (double tau : {0.10, 0.15}) for (std::size_t dly : {std::size_t{2}, std::size_t{10}}) {
      Case c;
      c.mass_scale = ms; c.inertia_scale = is; c.l_scale = ls;
      c.tau_s = tau; c.delay_ticks = dly; c.seed_error = 0.05; c.duration_s = 20.0;
      const auto r = simulate(c);
      if (r.fell || r.th_rms >= 0.01 || r.held_resets) {
        std::printf("m%.1f J%.1f l%.1f dly%zu ", ms, is, ls, dly);
        report("T7 FAIL", c, r);
      }
      assert(!r.fell && r.th_rms < 0.01);
      assert(r.held_resets == 0);
      ++n;
    }
    std::printf("T7 robustness: %d/%d plants stand\n", n, n);
  }
  // T8: the operator flow. Engaged on the stand (held 3 s at -0.02 rad,
  // wheels in the air), carried and held on the floor 1 s, let go.
  for (double eq : {0.10, 0.0}) {
    for (double err : {0.02, -0.02}) {
      Case c;
      c.eq = eq;
      c.seed_error = err;
      c.held_s = 3.0;
      c.tau_s = 0.12;
      const auto r = simulate(c);
      report("T8 stand -> floor", c, r);
      assert(!r.fell && std::abs(r.x_end) < 0.2 && r.th_rms < 0.003);
    }
  }
  std::puts("v3_physics_tests: all checks passed");
  return 0;
}
