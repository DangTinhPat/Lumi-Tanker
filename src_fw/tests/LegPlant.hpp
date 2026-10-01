#pragma once
// Planar body + legs + wheels plant for the v3 host sims, both legs moving
// together (the planar model has no roll):
//  - generalized coordinates s = (x axle, theta body pitch, q hip offset,
//    k knee offset), hip/knee in the core frame (offsets from the hang pose);
//  - masses, CoMs and link geometry from the CAD model (GeneratedGeometry),
//    body CoM shifted by a "true" fore/aft offset;
//  - thigh turns with theta + q, shin and wheel axle frame with
//    theta + q + k; the wheel spins with x / R (rolling), and the wheel motor
//    works on its angle to the shin, u = x / R - theta - q - k;
//  - optional ground profile under the axle (smooth bumps/steps): every
//    point is lifted by ground(x), a holonomic stand-in for rolling over it.
// Kane's method on numeric Jacobians of the forward kinematics: M(s) s'' +
// J^T (J' s') = Q. Numeric but exact to O(eps^2) for this holonomic chain.
#include <array>
#include <cmath>
#include <functional>

#include "robot_control/GeneratedGeometry.hpp"
#include "robot_control/WholeBodyModel.hpp"  // imu_offset_x_m / _z_m

namespace tap_fw::sim {

namespace geo = robot_control::geometry;
inline constexpr double kG = 9.80665;

struct Pt { double x, z; };
inline Pt rot(Pt p, double a) {
  const double c = std::cos(a), s = std::sin(a);
  return {c * p.x + s * p.z, -s * p.x + c * p.z};
}
inline Pt add(Pt a, Pt b) { return {a.x + b.x, a.z + b.z}; }
inline Pt sub(Pt a, Pt b) { return {a.x - b.x, a.z - b.z}; }

class LegPlant {
 public:
  static constexpr int kDof = 4;
  using Vec = std::array<double, kDof>;
  using Jac = std::array<Pt, kDof>;
  // Body-frame points (hip origin): CoMs of body, thigh, shin, wheel (one
  // leg; legs identical), the axle and the IMU.
  struct Points {
    std::array<Pt, 4> com;
    Pt axle, imu;
  };
  static constexpr int kImu = 4;
  static constexpr int kAxle = 5;

  LegPlant(double body_dx, double mass_scale, double hip_rotor, double knee_rotor,
           double hip_cad0)
      : dx_(body_dx), hip_cad0_(hip_cad0), hip_rotor_(hip_rotor), knee_rotor_(knee_rotor) {
    m_[0] = geo::parts[0].mass * mass_scale;
    for (int i = 1; i < 4; ++i) m_[i] = 2.0 * geo::parts[i].mass;
    i_body_ = geo::parts[0].iyy * mass_scale;
    i_thigh_ = 2.0 * geo::parts[1].iyy;
    i_shin_ = 2.0 * geo::parts[2].iyy;
    i_wheel_ = 2.0 * geo::parts[3].iyy;
  }

  // Ground height [m] under axle position x (default flat).
  std::function<double(double)> ground;

  Points points(double q, double k) const {
    Points p{};
    const double hip = hip_cad0_ + q;
    const double upper = hip, lower = hip + k + geo::knee_zero_rebase_rad;
    p.com[0] = {geo::parts[0].x + dx_, geo::parts[0].z};
    const Pt knee_pt = rot({geo::origins[1].x, geo::origins[1].z}, upper);
    p.com[1] = rot({geo::parts[1].x, geo::parts[1].z}, upper);
    p.com[2] = add(knee_pt, rot({geo::parts[2].x, geo::parts[2].z}, lower));
    p.axle = add(knee_pt, rot({geo::origins[2].x, geo::origins[2].z}, lower));
    p.com[3] = add(p.axle, rot({geo::parts[3].x, geo::parts[3].z}, lower));
    p.imu = {geo::imu_offset_x_m, geo::imu_offset_z_m};
    return p;
  }
  // World position of point k (0..3 CoMs, kImu, kAxle) at state s.
  Pt world(const Vec& s, int k) const {
    const Points p = points(s[2], s[3]);
    const Pt local = k < 4 ? p.com[k] : k == kImu ? p.imu : p.axle;
    const Pt w = rot(sub(local, p.axle), s[1]);
    const double z0 = geo::wheel_radius_m + (ground ? ground(s[0]) : 0.0);
    return {s[0] + w.x, z0 + w.z};
  }
  // Jacobian columns d world / d s_j and the velocity-product term J' s'.
  void kin(const Vec& s, const Vec& sd, int k, Jac& jac, Pt& jdot_sd) const {
    const double e = 1.0e-6;
    for (int j = 0; j < kDof; ++j) {
      Vec a = s, b = s;
      a[j] += e;
      b[j] -= e;
      const Pt pa = world(a, k), pb = world(b, k);
      jac[j] = {(pa.x - pb.x) / (2 * e), (pa.z - pb.z) / (2 * e)};
    }
    const double h = 1.0e-4;
    Vec a = s, b = s;
    for (int j = 0; j < kDof; ++j) { a[j] += h * sd[j]; b[j] -= h * sd[j]; }
    const Pt p0 = world(s, k), pa = world(a, k), pb = world(b, k);
    jdot_sd = {(pa.x - 2 * p0.x + pb.x) / (h * h), (pa.z - 2 * p0.z + pb.z) / (h * h)};
  }
  // Mass matrix and generalized forces (gravity, velocity terms, a push fx
  // at the body CoM; no actuator torques): M(s) s'' = Q + actuator terms.
  void generalized(const Vec& s, const Vec& sd, double fx, double slope,
                   double (&M)[kDof][kDof], double (&Q)[kDof]) const {
    for (int a = 0; a < kDof; ++a) {
      Q[a] = 0.0;
      for (int b = 0; b < kDof; ++b) M[a][b] = 0.0;
    }
    const double gx = -kG * std::sin(slope), gz = -kG * std::cos(slope);
    for (int k = 0; k < 4; ++k) {
      Jac J;
      Pt jd;
      kin(s, sd, k, J, jd);
      for (int a = 0; a < kDof; ++a) {
        for (int b = 0; b < kDof; ++b) M[a][b] += m_[k] * (J[a].x * J[b].x + J[a].z * J[b].z);
        Q[a] += m_[k] * (J[a].x * (gx - jd.x) + J[a].z * (gz - jd.z));
        if (k == 0) Q[a] += J[a].x * fx;
      }
    }
    const double R = geo::wheel_radius_m;
    // Rotational inertia: body (theta), thigh (theta + q), shin (theta + q +
    // k), wheel rotors (x / R), hip and knee rotors through the gear.
    M[1][1] += i_body_;
    for (int a : {1, 2}) for (int b : {1, 2}) M[a][b] += i_thigh_;
    for (int a : {1, 2, 3}) for (int b : {1, 2, 3}) M[a][b] += i_shin_;
    M[0][0] += i_wheel_ / (R * R);
    M[2][2] += 2.0 * hip_rotor_;
    M[3][3] += 2.0 * knee_rotor_;
  }
  // Accelerations for wheel torque tw (on u), hip torque th (on q), knee
  // torque tk (on k) and a horizontal push fx at the body CoM.
  Vec accel(const Vec& s, const Vec& sd, double tw, double th, double tk, double fx,
            double slope) const {
    double M[kDof][kDof];
    double Q[kDof];
    generalized(s, sd, fx, slope, M, Q);
    const double R = geo::wheel_radius_m;
    Q[0] += tw / R; Q[1] -= tw; Q[2] -= tw; Q[3] -= tw;
    Q[2] += th;
    Q[3] += tk;
    return solve(M, Q);
  }
  // Hip and knee torques (both legs together) that hold pose s still against
  // gravity on flat ground: with no velocity the x row needs no wheel torque,
  // and each joint row must cancel its gravity generalized force.
  std::array<double, 2> staticJointTorques(const Vec& s) const {
    double G[kDof] = {};
    const Vec zero{};
    for (int k = 0; k < 4; ++k) {
      Jac J;
      Pt jd;
      kin(s, zero, k, J, jd);
      for (int a = 0; a < kDof; ++a) G[a] += m_[k] * (-kG) * J[a].z;
    }
    return {-G[2], -G[3]};
  }
  // Kinetic + potential energy (plant self-check).
  double energy(const Vec& s, const Vec& sd) const {
    double e = 0.0;
    for (int k = 0; k < 4; ++k) {
      Jac J;
      Pt jd;
      kin(s, sd, k, J, jd);
      double vx = 0, vz = 0;
      for (int j = 0; j < kDof; ++j) { vx += J[j].x * sd[j]; vz += J[j].z * sd[j]; }
      e += 0.5 * m_[k] * (vx * vx + vz * vz) + m_[k] * kG * world(s, k).z;
    }
    const double R = geo::wheel_radius_m;
    const double w_thigh = sd[1] + sd[2], w_shin = sd[1] + sd[2] + sd[3];
    e += 0.5 * i_body_ * sd[1] * sd[1] + 0.5 * i_thigh_ * w_thigh * w_thigh +
        0.5 * i_shin_ * w_shin * w_shin + 0.5 * i_wheel_ * sd[0] * sd[0] / (R * R) +
        hip_rotor_ * sd[2] * sd[2] + knee_rotor_ * sd[3] * sd[3];
    return e;
  }
  // Angle of the axle -> CoM line from vertical (world), true masses.
  double leanAngle(const Vec& s) const {
    double mx = 0, mz = 0, mt = 0;
    const Pt axle = world(s, kAxle);
    for (int k = 0; k < 4; ++k) {
      const Pt w = world(s, k);
      mx += m_[k] * (w.x - axle.x);
      mz += m_[k] * (w.z - axle.z);
      mt += m_[k];
    }
    return std::atan2(mx / mt, mz / mt);
  }

 private:
  static Vec solve(double (&M)[kDof][kDof], double (&Q)[kDof]) {
    double A[kDof][kDof + 1];
    for (int r = 0; r < kDof; ++r) {
      for (int c = 0; c < kDof; ++c) A[r][c] = M[r][c];
      A[r][kDof] = Q[r];
    }
    for (int c = 0; c < kDof; ++c) {
      int piv = c;
      for (int r = c + 1; r < kDof; ++r) if (std::abs(A[r][c]) > std::abs(A[piv][c])) piv = r;
      if (piv != c) for (int k = 0; k <= kDof; ++k) std::swap(A[c][k], A[piv][k]);
      for (int r = c + 1; r < kDof; ++r) {
        const double f = A[r][c] / A[c][c];
        for (int k = c; k <= kDof; ++k) A[r][k] -= f * A[c][k];
      }
    }
    Vec x{};
    for (int r = kDof - 1; r >= 0; --r) {
      double v = A[r][kDof];
      for (int c = r + 1; c < kDof; ++c) v -= A[r][c] * x[c];
      x[r] = v / A[r][r];
    }
    return x;
  }

  double dx_, hip_cad0_, hip_rotor_, knee_rotor_;
  double m_[4] = {};
  double i_body_ = 0.0, i_thigh_ = 0.0, i_shin_ = 0.0, i_wheel_ = 0.0;
};

}  // namespace tap_fw::sim
