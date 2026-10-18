// AirborneModel (src_fw aac) against MuJoCo, free-floating with no gravity (the flight phase):
//   1. CoM line angle (CoM seen from the axle, body frame) for several leg poses;
//   2. angular momentum L from the encoders and the gyro against MuJoCo subtree_angmom, and its
//      conservation while the legs move;
//   3. touchdown attitude prediction (body pitch and virtual pitch after a leg motion with L != 0).
// Simulation evidence only: it checks that the controller's model agrees with the simulator's model.
#include <mujoco/mujoco.h>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "tap_fw/v3/LegHeight.hpp"
#include "tap_fw/v3/aac/AirborneModel.hpp"

using namespace tap_fw::v3;

namespace {

struct Ids {
  int hl, kl, hr, kr;      // qpos addresses
  int dhl, dkl, dhr, dkr;  // dof addresses
  int base, wheel;         // body ids
};

double bodyPitch(const mjData* d) { return 2.0 * std::atan2(d->qpos[5], d->qpos[3]); }

Ids ids(const mjModel* m) {
  auto q = [&](const char* n) { return m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, n)]; };
  auto v = [&](const char* n) { return m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, n)]; };
  return {q("hip_left"), q("knee_left"), q("hip_right"), q("knee_right"),
          v("hip_left"), v("knee_left"), v("hip_right"), v("knee_right"),
          m->jnt_bodyid[0], mj_name2id(m, mjOBJ_BODY, "left_wheel")};
}

void place(const mjModel* m, mjData* d, const Ids& id, double hip, double knee, double body_rate) {
  mj_resetData(m, d);
  d->qpos[2] = 1.0;
  d->qpos[3] = 1.0;
  d->qpos[id.hl] = d->qpos[id.hr] = hip;
  d->qpos[id.kl] = d->qpos[id.kr] = knee;
  d->qvel[4] = body_rate;     // free joint: angular velocity in the body frame, pitch about +Y
  mj_forward(m, d);
}

// CoM of the whole robot relative to the left axle, in the body frame.
void comFromAxle(const mjModel* m, mjData* d, const Ids& id, double& x, double& z) {
  mj_forward(m, d);
  const double* c = d->subtree_com + 3 * id.base;
  const double* a = d->xpos + 3 * id.wheel;
  const double* r = d->xmat + 9 * id.base;    // body -> world, row major
  const double w[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
  x = r[0] * w[0] + r[3] * w[1] + r[6] * w[2];
  z = r[2] * w[0] + r[5] * w[1] + r[8] * w[2];
}

double simAngularMomentum(const mjModel* m, mjData* d, const Ids& id) {
  mj_subtreeVel(m, d);
  return d->subtree_angmom[3 * id.base + 1];
}

void comLineMatches(const mjModel* m, mjData* d, const Ids& id) {
  const double hang = LegHeight::hangHipCad();
  double worst = 0.0;
  for (double hip : {-0.9, -0.6, -0.3, 0.0}) {
    for (double knee : {0.4, 0.9, 1.4, 1.8}) {
      place(m, d, id, hip, knee, 0.0);
      double x = 0.0, z = 0.0;
      comFromAxle(m, d, id, x, z);
      const double sim = std::atan2(x, z);
      const double model = comLineAngle(static_cast<float>(hang + hip), static_cast<float>(knee));
      worst = std::fmax(worst, std::fabs(sim - model));
    }
  }
  std::printf("CoM line angle: worst model - MuJoCo %.5f rad over 16 poses\n", worst);
  assert(worst < 2e-3);
}

// Smooth min-jerk motion from q0 to q1 over T tracked with stiff PD (torque limited like the robot).
void runMotion(const mjModel* m, mjData* d, const Ids& id, double h0, double k0, double h1, double k1, double T,
               std::vector<double>* hips = nullptr, std::vector<double>* knees = nullptr) {
  const double dt = m->opt.timestep;
  for (double t = 0.0; t < T + 0.04; t += dt) {
    const double u = std::fmin(t / T, 1.0);
    const double s = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
    const double tgt[4] = {h0 + (h1 - h0) * s, k0 + (k1 - k0) * s, h0 + (h1 - h0) * s, k0 + (k1 - k0) * s};
    const double q[4] = {d->qpos[id.hl], d->qpos[id.kl], d->qpos[id.hr], d->qpos[id.kr]};
    const double v[4] = {d->qvel[id.dhl], d->qvel[id.dkl], d->qvel[id.dhr], d->qvel[id.dkr]};
    for (int i = 0; i < 4; ++i) d->ctrl[i] = std::fmax(-3.76, std::fmin(3.76, 60.0 * (tgt[i] - q[i]) - 2.0 * v[i]));
    d->ctrl[4] = d->ctrl[5] = 0.0;
    if (hips != nullptr) {
      hips->push_back(d->qpos[id.hl]);
      knees->push_back(d->qpos[id.kl]);
    }
    mj_step(m, d);
  }
}

void momentumMatchesAndIsConserved(const mjModel* m, mjData* d, const Ids& id) {
  const double hang = LegHeight::hangHipCad();
  place(m, d, id, -0.75, 1.55, 1.5);
  const double l0 = simAngularMomentum(m, d, id);
  const auto est = [&]() {
    return bodyMomentum(static_cast<float>(hang + d->qpos[id.hl]), static_cast<float>(d->qpos[id.kl]),
                        static_cast<float>(d->qvel[id.dhl]), static_cast<float>(d->qvel[id.dkl]),
                        static_cast<float>(d->qvel[4])).angular_momentum;
  };
  const double e0 = est();
  runMotion(m, d, id, -0.75, 1.55, -0.35, 0.9, 0.12);
  const double l1 = simAngularMomentum(m, d, id), e1 = est();
  std::printf("L: MuJoCo %.5f -> %.5f, model %.5f -> %.5f N.m.s\n", l0, l1, e0, e1);
  assert(std::fabs(e0 - l0) < 0.02 * std::fabs(l0) + 2e-4);
  assert(std::fabs(l1 - l0) < 0.02 * std::fabs(l0) + 2e-4);   // conserved in the simulator
  assert(std::fabs(e1 - l1) < 0.03 * std::fabs(l1) + 3e-4);   // and seen by the model while the legs moved
}

void touchdownPredictionMatches(const mjModel* m, mjData* d, const Ids& id) {
  const double hang = LegHeight::hangHipCad();
  struct Case { double h0, k0, h1, k1, w, T; };
  const Case cases[] = {{-0.75, 1.55, -0.75, 1.55, 0.8, 0.25},     // legs still: pure free rotation
                        {-0.75, 1.55, -0.45, 1.10, 0.0, 0.25},     // leg motion only
                        {-0.75, 1.55, -0.95, 1.70, -0.9, 0.25},    // both
                        {-0.50, 1.20, -0.80, 1.60, 1.2, 0.20}};
  double worst_body = 0.0, worst_theta = 0.0;
  for (const auto& c : cases) {
    place(m, d, id, c.h0, c.k0, c.w);
    const float hip0 = static_cast<float>(hang + c.h0), knee0 = static_cast<float>(c.k0);
    const float L = bodyMomentum(hip0, knee0, 0.0F, 0.0F, static_cast<float>(c.w)).angular_momentum;
    double x = 0.0, z = 0.0;
    comFromAxle(m, d, id, x, z);
    const double body0 = bodyPitch(d);
    const double theta0 = body0 + std::atan2(x, z);
    // The planned motion, sampled in time for the free rotation (what the controller knows ahead).
    std::vector<double> hs, ks;
    runMotion(m, d, id, c.h0, c.k0, c.h1, c.k1, c.T, &hs, &ks);
    const double t_total = static_cast<double>(hs.size()) * m->opt.timestep;
    constexpr int kSamples = 9;
    float sh[kSamples], sk[kSamples];
    for (int i = 0; i < kSamples; ++i) {
      const double t = t_total * i / (kSamples - 1);
      const double u = std::fmin(t / c.T, 1.0);
      const double s = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
      sh[i] = static_cast<float>(hang + c.h0 + (c.h1 - c.h0) * s);
      sk[i] = static_cast<float>(c.k0 + (c.k1 - c.k0) * s);
    }
    const float free_rot = freeRotation(L, sh, sk, kSamples, static_cast<float>(t_total));
    const float hip1 = static_cast<float>(hang + c.h1), knee1 = static_cast<float>(c.k1);
    const auto p = predictTouchdown(static_cast<float>(body0), static_cast<float>(theta0), free_rot,
                                    bodyTurnFromLegPath(hip0, knee0, hip1, knee1),
                                    comLineAngle(hip0, knee0), comLineAngle(hip1, knee1));
    comFromAxle(m, d, id, x, z);
    const double body1 = bodyPitch(d), theta1 = body1 + std::atan2(x, z);
    std::printf("touchdown: body %+.4f (predicted %+.4f), theta_v %+.4f (predicted %+.4f) rad\n", body1 - body0,
                p.body - body0, theta1 - theta0, p.theta_v - theta0);
    worst_body = std::fmax(worst_body, std::fabs(body1 - p.body));
    worst_theta = std::fmax(worst_theta, std::fabs(theta1 - p.theta_v));
  }
  std::printf("touchdown prediction worst error: body %.4f, theta_v %.4f rad\n", worst_body, worst_theta);
  assert(worst_body < 0.02 && worst_theta < 0.02);
}

void sensitivityMatchesTheModel() {
  const float hip = LegHeight::hangHipCad() - 0.75F, knee = 1.55F;
  const auto s = touchdownSensitivity(hip, knee);
  const float e = 1e-3F;
  const float body_h = bodyTurnFromLegPath(hip, knee, hip + e, knee) / e;
  const float theta_k = (bodyTurnFromLegPath(hip, knee, hip, knee + e) + comLineAngle(hip, knee + e) -
                         comLineAngle(hip, knee)) / e;
  std::printf("sensitivity: body/hip %.3f theta/hip %.3f body/knee %.3f theta/knee %.3f\n", s.body_hip, s.theta_hip,
              s.body_knee, s.theta_knee);
  assert(std::fabs(s.body_hip - body_h) < 0.01F && std::fabs(s.theta_knee - theta_k) < 0.01F);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::puts("usage: aac_model_physics_tests model.xml");
    return 2;
  }
  char err[512] = {};
  mjModel* m = mj_loadXML(argv[1], nullptr, err, sizeof(err));
  if (m == nullptr) {
    std::puts(err);
    return 1;
  }
  m->opt.gravity[2] = 0.0;
  m->opt.timestep = 1.0 / 5000.0;
  mjData* d = mj_makeData(m);
  const Ids id = ids(m);
  comLineMatches(m, d, id);
  momentumMatchesAndIsConserved(m, d, id);
  touchdownPredictionMatches(m, d, id);
  sensitivityMatchesTheModel();
  mj_deleteData(d);
  mj_deleteModel(m);
  std::puts("aac model physics tests passed");
  return 0;
}
