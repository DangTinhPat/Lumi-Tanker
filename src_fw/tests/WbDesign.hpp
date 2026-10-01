#pragma once
// Whole-body LQR design (host only, Eigen): wheel + hip + knee on the
// LegPlant model, one gain per height. Used by src_fw/tools/wb_lqr_design.cpp
// (writes the firmware gain table) and by the tier tests.
//
// Model. The wheels are a speed servo (u'' = (w_cmd - u') / tau, u the wheel
// angle to the shin), the hips and knees GDS68 position servos realised as
// torques on the real inertia (as the sim does: tau = 2 kv (kp (cmd - q) -
// q') + the static torque the servo integrator already holds). With the
// wheel acceleration prescribed, the remaining dynamics live in the
// directions that keep u fixed - theta, q and k, each moving x by R - so
//   D^T M D [th'' q'' k'']^T = D^T (Q - M c_u) + [0 tau_h tau_k]^T,
//   D = [(R,1,0,0) (R,0,1,0) (R,0,0,1)], c_u = (R u'', 0, 0, 0),
// and x'' = R (u'' + th'' + q'' + k''). States z = (x, th, q, k) and rates,
// all measured on the robot (wheel and joint encoders, IMU).
//
// Discrete design at the control period with the wheel dead time and the
// joint command delay as shift registers of past commands, an integral of
// the position error, and costs on outputs (body pitch, CoM lean, position,
// joints, command steps). DARE by the structure-preserving doubling
// algorithm (as LqrBalancer: plain value iteration crawls on the slow
// position modes).
#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "LegPlant.hpp"
#include "tap_fw/v3/LegHeight.hpp"

namespace tap_fw::sim {

struct WbActuators {
  double dt = 0.002;
  // The wheel as the robot behaves (2026-10-01 logs, see
  // V3Controller::WheelPredict): ~20 ms dead time + 35 ms lag; the 55 ms age
  // of the speed reading is taken out by the predictor, so the design sees
  // a fresh reading. The old 50 ms + 0.09 s lag model rolled the high
  // frequencies off that the real wheel passes - the gains rang at 2-4 Hz.
  double wheel_tau = 0.035;
  int wheel_delay_ticks = 10;
  int joint_delay_ticks = 3;  // 2 ticks CAN/driver + half the 250 Hz resend
  // GDS68 position servo, per joint (see v3_tier_tests Case).
  double hip_kp = 16.0, hip_kv = 48.0 * (geo::leg_hip_inertia_kgm2 + 0.004);
  double knee_kp = 16.0, knee_kv = 350.0 * (geo::leg_knee_inertia_kgm2 + 0.004);
  double hip_rotor = 0.004, knee_rotor = 0.004;
  double body_dx = 0.021;  // model body CoM offset (= the controller's)
};

// Output weights are 1 / (acceptable size)^2. Values from the coordinate
// search over the two-body sim scenarios (v3_tier_tests TIER_WBTUNE).
struct WbWeights {
  // Robust search (TIER_WBTUNE, 2026-10-01): the first table rang cap to
  // cap on the robot; in the sim it did so with encoder-rate noise or a
  // stiffer knee servo. Scored on those plants (WB_ROBUST), on 8 random
  // episodes disjoint from the TIER_GENERAL check (WB_GEN) - a pass on the
  // fixed scenarios alone overfit (push body 0.008 rad, but 3 falls / 12
  // false holds in 48 random episodes) - with the body steadiness weighted
  // 2x (WB_BODYW). Re-searched on the measured wheel (20 ms + 0.035 s,
  // reading 55 ms old, predictor tau 0.06) with the wheel at its cap on
  // the wheel variants too (WB_SAT: hard stop, 6 N push, 84 mm cruise):
  // falls in the score 4 -> 1. The comments give the weight as 1/size^2
  // of an acceptable value.
  double pitch = 1250.0;          // 0.028 rad
  double pitch_rate = 0.125;      // 2.8 rad/s
  double lean = 6.25;             // 0.4 rad
  double pos = 6.25;              // 0.4 m
  double vel = 2.7778;            // 0.6 m/s
  double pos_int = 2.7778;        // 0.6
  double hip = 2.7778;            // 0.6 rad
  double knee = 6400.0;           // 0.0125 rad
  double hip_rate = 0.0625;       // 4 rad/s
  double knee_rate = 0.0019531;   // 23 rad/s
  double wheel_cmd = 0.0011111;   // 30 rad/s
  double hip_cmd = 12.5;          // 0.28 rad
  double knee_cmd = 11377.8;      // 0.0094 rad
  double wheel_step = 16.0;       // 0.25 rad/s
  double hip_step = 78.125;       // 0.11 rad
  double knee_step = 40000.0;     // 0.005 rad
  double body_vacc = 0.0069445;   // 12 m/s^2: body CoM vertical acceleration
  double body_hacc = 0.0;                // body CoM horizontal acceleration
  double body_z = 0.0;                   // body CoM height off the pose [m] (knee holds it)
};

// The softer set (first robust pass): the body is held looser, so near the
// wheel limit it can lean to help the catch. V3Controller blends toward the
// table built from it as the wheel command approaches its cap (the stiff
// set alone fell where this one held: a 70 ms wheel dead time, driving
// backwards into a push, sim episode 2).
inline WbWeights softWeights() {
  WbWeights w;
  w.pitch = 156.25;
  w.pitch_rate = 0.25;
  w.lean = 25.0;
  w.vel = 1.0 / (0.15 * 0.15);
  w.hip = 0.6944;
  w.knee = 1600.0;
  w.knee_rate = 0.015625;
  w.knee_cmd = 2844.4;
  w.hip_step = 2500.0;
  w.knee_step = 2500.0;
  return w;
}

struct WbPoint {
  double height_m = 0.0;
  double theta0 = 0.0, q0 = 0.0, k0 = 0.0;  // equilibrium (body pitch, hip, knee offsets)
  double tau_h0 = 0.0, tau_k0 = 0.0;        // static joint torques (both legs)
  Eigen::MatrixXd K;                        // 3 x n: v = v0 - K X
  double max_closed_loop_eig = 0.0;         // spectral radius of A - B K
};

class WbDesign {
 public:
  static constexpr int kNz = 8;  // x, th, q, k, rates
  static constexpr int kNv = 3;  // wheel speed cmd [rad/s], hip offset, knee offset
  // Augmented state layout.
  int nw() const { return act_.wheel_delay_ticks; }
  int nj() const { return act_.joint_delay_ticks; }
  int n() const { return kNz + 1 + nw() + 2 * nj(); }
  int idxInt() const { return kNz; }
  int idxWheel() const { return kNz + 1; }
  int idxHip() const { return kNz + 1 + nw(); }
  int idxKnee() const { return kNz + 1 + nw() + nj(); }

  WbDesign(const WbActuators& act, const WbWeights& w)
      : act_(act), w_(w),
        plant_(act.body_dx, 1.0, act.hip_rotor, act.knee_rotor, v3::LegHeight::hangHipCad()) {}

  // Equilibrium at a height pose: CoM over the axle, joints at the pose.
  WbPoint operatingPoint(double height_m) const {
    const auto pose = v3::LegHeight::forHeight(static_cast<float>(height_m));
    WbPoint p;
    p.height_m = height_m;
    LegPlant::Vec s{0.0, 0.0, pose.hip_offset_rad, pose.knee_offset_rad};
    for (int i = 0; i < 60; ++i) s[1] -= plant_.leanAngle(s);
    p.theta0 = s[1];
    p.q0 = s[2];
    p.k0 = s[3];
    const auto hold = plant_.staticJointTorques(s);
    p.tau_h0 = hold[0];
    p.tau_k0 = hold[1];
    return p;
  }

  // Continuous dynamics z' = f(z, v).
  Eigen::VectorXd f(const WbPoint& p, const Eigen::VectorXd& z, const Eigen::Vector3d& v) const {
    const double R = geo::wheel_radius_m;
    LegPlant::Vec s{z[0], z[1], z[2], z[3]}, sd{z[4], z[5], z[6], z[7]};
    double M[LegPlant::kDof][LegPlant::kDof];
    double Q[LegPlant::kDof];
    plant_.generalized(s, sd, 0.0, 0.0, M, Q);
    Eigen::Matrix4d Me;
    Eigen::Vector4d Qe;
    for (int a = 0; a < 4; ++a) {
      Qe[a] = Q[a];
      for (int b = 0; b < 4; ++b) Me(a, b) = M[a][b];
    }
    const double ud = z[4] / R - z[5] - z[6] - z[7];
    const double udd = (v[0] - ud) / act_.wheel_tau;
    const double tau_h = 2.0 * act_.hip_kv * (act_.hip_kp * (v[1] - z[2]) - z[6]) + p.tau_h0;
    const double tau_k = 2.0 * act_.knee_kv * (act_.knee_kp * (v[2] - z[3]) - z[7]) + p.tau_k0;
    Eigen::Matrix<double, 4, 3> D;
    D << R, R, R, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    const Eigen::Vector4d cu(R * udd, 0.0, 0.0, 0.0);
    const Eigen::Vector3d rhs = D.transpose() * (Qe - Me * cu) + Eigen::Vector3d(0.0, tau_h, tau_k);
    const Eigen::Vector3d y = (D.transpose() * Me * D).ldlt().solve(rhs);
    Eigen::VectorXd zd(kNz);
    for (int i = 0; i < 4; ++i) zd[i] = z[4 + i];
    zd[5] = y[0];
    zd[6] = y[1];
    zd[7] = y[2];
    zd[4] = R * (udd + y[0] + y[1] + y[2]);
    return zd;
  }

  // CoM line angle from the axle (linearised output).
  double lean(const LegPlant::Vec& s) const { return plant_.leanAngle(s); }

  // Gain at one height. Returns the operating point with K filled in.
  WbPoint design(double height_m) const {
    WbPoint p = operatingPoint(height_m);
    Eigen::VectorXd z0 = Eigen::VectorXd::Zero(kNz);
    z0[1] = p.theta0;
    z0[2] = p.q0;
    z0[3] = p.k0;
    const Eigen::Vector3d v0(0.0, p.q0, p.k0);
    // Linearise.
    Eigen::MatrixXd A(kNz, kNz), B(kNz, kNv);
    const double e = 1e-6;
    for (int j = 0; j < kNz; ++j) {
      Eigen::VectorXd zp = z0, zm = z0;
      zp[j] += e;
      zm[j] -= e;
      A.col(j) = (f(p, zp, v0) - f(p, zm, v0)) / (2 * e);
    }
    for (int j = 0; j < kNv; ++j) {
      Eigen::Vector3d vp = v0, vm = v0;
      vp[j] += e;
      vm[j] -= e;
      B.col(j) = (f(p, z0, vp) - f(p, z0, vm)) / (2 * e);
    }
    // Discretise (zero-order hold) by the block exponential.
    Eigen::MatrixXd blk = Eigen::MatrixXd::Zero(kNz + kNv, kNz + kNv);
    blk.topLeftCorner(kNz, kNz) = A * act_.dt;
    blk.topRightCorner(kNz, kNv) = B * act_.dt;
    const Eigen::MatrixXd ex = blk.exp();
    const Eigen::MatrixXd Ad = ex.topLeftCorner(kNz, kNz);
    const Eigen::MatrixXd Bd = ex.topRightCorner(kNz, kNv);
    // Augment: position integral and command delay registers.
    const int N = n();
    Eigen::MatrixXd Aa = Eigen::MatrixXd::Zero(N, N), Ba = Eigen::MatrixXd::Zero(N, kNv);
    Aa.topLeftCorner(kNz, kNz) = Ad;
    auto feed = [&](int input, int first, int len) {
      if (len == 0) {
        Ba.block(0, input, kNz, 1) = Bd.col(input);
        return;
      }
      Aa.block(0, first + len - 1, kNz, 1) = Bd.col(input);  // the oldest command acts
      Ba(first, input) = 1.0;
      for (int i = 1; i < len; ++i) Aa(first + i, first + i - 1) = 1.0;
    };
    feed(0, idxWheel(), nw());
    feed(1, idxHip(), nj());
    feed(2, idxKnee(), nj());
    Aa(idxInt(), idxInt()) = 1.0;
    Aa(idxInt(), 0) = act_.dt;
    // Outputs y = C X + D v with weights W.
    std::vector<Eigen::RowVectorXd> Cy;
    std::vector<Eigen::RowVectorXd> Dy;
    std::vector<double> Wy;
    auto out = [&](double w, int state, int input = -1, double input_gain = 1.0) {
      Eigen::RowVectorXd c = Eigen::RowVectorXd::Zero(N), d = Eigen::RowVectorXd::Zero(kNv);
      if (state >= 0) c[state] = 1.0;
      if (input >= 0) d[input] = input_gain;
      Cy.push_back(c);
      Dy.push_back(d);
      Wy.push_back(w);
    };
    out(w_.pitch, 1);
    out(w_.pitch_rate, 5);
    out(w_.pos, 0);
    out(w_.vel, 4);
    out(w_.pos_int, idxInt());
    out(w_.hip, 2);
    out(w_.knee, 3);
    out(w_.hip_rate, 6);
    out(w_.knee_rate, 7);
    {
      Eigen::RowVectorXd c = Eigen::RowVectorXd::Zero(N);
      LegPlant::Vec s{0.0, p.theta0, p.q0, p.k0};
      for (int j = 1; j < 4; ++j) {
        LegPlant::Vec a = s, b = s;
        a[j] += e;
        b[j] -= e;
        c[j] = (lean(a) - lean(b)) / (2 * e);
      }
      Cy.push_back(c);
      Dy.push_back(Eigen::RowVectorXd::Zero(kNv));
      Wy.push_back(w_.lean);
    }
    // Body CoM height (linearised): the knee keeps the body at its height
    // while the hip swings the legs.
    if (w_.body_z > 0.0) {
      Eigen::RowVectorXd c = Eigen::RowVectorXd::Zero(N);
      LegPlant::Vec s{0.0, p.theta0, p.q0, p.k0};
      for (int j = 1; j < 4; ++j) {
        LegPlant::Vec a = s, b = s;
        a[j] += e;
        b[j] -= e;
        c[j] = (plant_.world(a, 0).z - plant_.world(b, 0).z) / (2 * e);
      }
      Cy.push_back(c);
      Dy.push_back(Eigen::RowVectorXd::Zero(kNv));
      Wy.push_back(w_.body_z);
    }
    // Body CoM accelerations (linearised: d pos / d s times s'', s'' from
    // the continuous model with the commands that act now - the oldest in
    // each delay register).
    if (w_.body_vacc > 0.0 || w_.body_hacc > 0.0) {
      LegPlant::Vec s{0.0, p.theta0, p.q0, p.k0};
      double dz[4], dxb[4];
      for (int j = 0; j < 4; ++j) {
        LegPlant::Vec a = s, b = s;
        a[j] += e;
        b[j] -= e;
        const Pt pa = plant_.world(a, 0), pb = plant_.world(b, 0);
        dz[j] = (pa.z - pb.z) / (2 * e);
        dxb[j] = (pa.x - pb.x) / (2 * e);
      }
      for (int which = 0; which < 2; ++which) {
        const double w = which ? w_.body_hacc : w_.body_vacc;
        if (w <= 0.0) continue;
        const double* g = which ? dxb : dz;
        Eigen::RowVectorXd c = Eigen::RowVectorXd::Zero(N), d = Eigen::RowVectorXd::Zero(kNv);
        for (int j = 0; j < 4; ++j) {
          c.head(kNz) += g[j] * A.row(4 + j);
          const int src[3] = {nw() ? idxWheel() + nw() - 1 : -1, nj() ? idxHip() + nj() - 1 : -1,
                              nj() ? idxKnee() + nj() - 1 : -1};
          for (int in = 0; in < kNv; ++in) {
            if (src[in] >= 0) c[src[in]] += g[j] * B(4 + j, in);
            else d[in] += g[j] * B(4 + j, in);
          }
        }
        Cy.push_back(c);
        Dy.push_back(d);
        Wy.push_back(w);
      }
    }
    // Command size and command steps (v - previous command).
    out(w_.wheel_cmd, -1, 0);
    out(w_.hip_cmd, -1, 1);
    out(w_.knee_cmd, -1, 2);
    auto step = [&](double w, int input, int first, int len) {
      if (len == 0) return;
      Eigen::RowVectorXd c = Eigen::RowVectorXd::Zero(N), d = Eigen::RowVectorXd::Zero(kNv);
      c[first] = -1.0;
      d[input] = 1.0;
      Cy.push_back(c);
      Dy.push_back(d);
      Wy.push_back(w);
    };
    step(w_.wheel_step, 0, idxWheel(), nw());
    step(w_.hip_step, 1, idxHip(), nj());
    step(w_.knee_step, 2, idxKnee(), nj());
    Eigen::MatrixXd Qm = Eigen::MatrixXd::Zero(N, N), Nm = Eigen::MatrixXd::Zero(N, kNv),
                    Rm = Eigen::MatrixXd::Zero(kNv, kNv);
    for (std::size_t i = 0; i < Cy.size(); ++i) {
      Qm += Wy[i] * Cy[i].transpose() * Cy[i];
      Nm += Wy[i] * Cy[i].transpose() * Dy[i];
      Rm += Wy[i] * Dy[i].transpose() * Dy[i];
    }
    // Per-step costs: scale by dt so the weights mean the same at any rate.
    Qm *= act_.dt;
    Nm *= act_.dt;
    Rm *= act_.dt;
    p.K = solveGain(Aa, Ba, Qm, Nm, Rm);
    const Eigen::MatrixXd Acl = Aa - Ba * p.K;
    p.max_closed_loop_eig = Acl.eigenvalues().cwiseAbs().maxCoeff();
    return p;
  }

  const WbActuators& actuators() const { return act_; }
  const LegPlant& plant() const { return plant_; }

 private:
  // DARE with a cross term by SDA; K = (R + B'PB)^-1 (B'PA + N').
  static Eigen::MatrixXd solveGain(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                                   const Eigen::MatrixXd& Q, const Eigen::MatrixXd& Nc,
                                   const Eigen::MatrixXd& R) {
    const int n = static_cast<int>(A.rows());
    const Eigen::MatrixXd Ri = R.inverse();
    Eigen::MatrixXd Ak = A - B * Ri * Nc.transpose();
    Eigen::MatrixXd Gk = B * Ri * B.transpose();
    Eigen::MatrixXd Hk = Q - Nc * Ri * Nc.transpose();
    Hk = 0.5 * (Hk + Hk.transpose());
    const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    for (int it = 0; it < 80; ++it) {
      const Eigen::MatrixXd W = (I + Gk * Hk).partialPivLu().inverse();
      const Eigen::MatrixXd An = Ak * W * Ak;
      const Eigen::MatrixXd Gn = Gk + Ak * W * Gk * Ak.transpose();
      const Eigen::MatrixXd Hn = Hk + Ak.transpose() * Hk * W * Ak;
      const double change = (Hn - Hk).norm() / std::max(Hn.norm(), 1e-12);
      Ak = An;
      Gk = 0.5 * (Gn + Gn.transpose());
      Hk = 0.5 * (Hn + Hn.transpose());
      if (change < 1e-12) break;
    }
    const Eigen::MatrixXd& P = Hk;
    return (R + B.transpose() * P * B).ldlt().solve(B.transpose() * P * A + Nc.transpose());
  }

  WbActuators act_;
  WbWeights w_;
  LegPlant plant_;
};

}  // namespace tap_fw::sim
