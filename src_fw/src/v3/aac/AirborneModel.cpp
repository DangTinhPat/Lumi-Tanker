#include "tap_fw/v3/aac/AirborneModel.hpp"

#include <algorithm>
#include <cmath>

#include "robot_control/GeneratedGeometry.hpp"

namespace tap_fw::v3 {
namespace {

inline Vec2 rot(Vec2 p, float th) noexcept {      // rotation about +Y (CAD convention)
  const float c = std::cos(th), s = std::sin(th);
  return {c * p.x + s * p.z, -s * p.x + c * p.z};
}
inline Vec2 perp(Vec2 v) noexcept { return {v.z, -v.x}; }   // d/dth rot(v, th) at th = 0
inline Vec2 add(Vec2 a, Vec2 b) noexcept { return {a.x + b.x, a.z + b.z}; }

}  // namespace

LegInertia legInertia(float hip_cad, float knee) noexcept {
  namespace geo = robot_control::geometry;
  // Thigh turns with the hip; shin and wheel with hip + knee (WholeBodyModel::compute chain).
  const float a = hip_cad;
  const float b = hip_cad + knee + geo::knee_zero_rebase_rad;
  const Vec2 h{geo::origins[0].x, geo::origins[0].z};
  const Vec2 o1{geo::origins[1].x, geo::origins[1].z}, o2{geo::origins[2].x, geo::origins[2].z};
  const Vec2 c1{geo::parts[1].x, geo::parts[1].z}, c2{geo::parts[2].x, geo::parts[2].z};
  const Vec2 ro1 = rot(o1, a), ro2 = rot(o2, b), rc1 = rot(c1, a), rc2 = rot(c2, b);
  const Vec2 knee_origin = add(h, ro1), axle = add(knee_origin, ro2);
  struct Link { float m, i; Vec2 p, dp_hip, dp_knee; float w_hip, w_knee; };
  const Link links[4] = {
      {geo::parts[0].mass, geo::parts[0].iyy, {geo::parts[0].x, geo::parts[0].z}, {0, 0}, {0, 0}, 0, 0},
      {2.0F * geo::parts[1].mass, 2.0F * geo::parts[1].iyy, add(h, rc1), perp(rc1), {0, 0}, 1, 0},
      {2.0F * geo::parts[2].mass, 2.0F * geo::parts[2].iyy, add(knee_origin, rc2),
       add(perp(ro1), perp(rc2)), perp(rc2), 1, 1},
      {2.0F * geo::parts[3].mass, 2.0F * geo::parts[3].iyy, axle,
       add(perp(ro1), perp(ro2)), perp(ro2), 1, 1}};
  float mass = 0.0F, cx = 0.0F, cz = 0.0F;
  for (const auto& l : links) { mass += l.m; cx += l.m * l.p.x; cz += l.m * l.p.z; }
  cx /= mass;
  cz /= mass;
  LegInertia out{};
  out.com_from_axle = {cx - axle.x, cz - axle.z};
  for (const auto& l : links) {
    const float dx = l.p.x - cx, dz = l.p.z - cz;
    out.inertia += l.i + l.m * (dx * dx + dz * dz);
    out.l_hip += l.m * (dz * l.dp_hip.x - dx * l.dp_hip.z) + l.i * l.w_hip;
    out.l_knee += l.m * (dz * l.dp_knee.x - dx * l.dp_knee.z) + l.i * l.w_knee;
  }
  return out;
}

float comLineAngle(float hip_cad, float knee, float body_com_dx_m) noexcept {
  namespace geo = robot_control::geometry;
  const Vec2 c = legInertia(hip_cad, knee).com_from_axle;
  const float cx = c.x + geo::parts[0].mass * body_com_dx_m / geo::total_mass_kg;
  return std::atan2(cx, std::max(c.z, 0.01F));
}

float bodyRateFromLegMotion(float hip_cad, float knee, float hip_rate, float knee_rate) noexcept {
  const auto d = legInertia(hip_cad, knee);
  return -(d.l_hip * hip_rate + d.l_knee * knee_rate) / std::max(d.inertia, 1.0e-6F);
}

float hipPerBodyRate(float hip_cad, float knee) noexcept {
  const auto d = legInertia(hip_cad, knee);
  return std::abs(d.l_hip) > 1.0e-6F ? d.inertia / d.l_hip : 0.0F;
}

float bodyTurnFromLegPath(float hip0_cad, float knee0, float hip1_cad, float knee1) noexcept {
  const float dh = hip1_cad - hip0_cad, dk = knee1 - knee0;
  float turn = 0.0F;
  constexpr float w[3] = {1.0F / 6.0F, 4.0F / 6.0F, 1.0F / 6.0F};
  for (int i = 0; i < 3; ++i) {
    const float u = 0.5F * static_cast<float>(i);
    const auto d = legInertia(hip0_cad + u * dh, knee0 + u * dk);
    turn -= w[i] * (d.l_hip * dh + d.l_knee * dk) / std::max(d.inertia, 1.0e-6F);
  }
  return turn;
}

BodyMomentum bodyMomentum(float hip_cad, float knee, float hip_rate, float knee_rate,
                          float body_rate) noexcept {
  const auto d = legInertia(hip_cad, knee);
  BodyMomentum m{};
  m.inertia = d.inertia;
  m.cab = d.com_from_axle.z;
  m.com_from_axle = d.com_from_axle;
  m.angular_momentum = d.inertia * body_rate + d.l_hip * hip_rate + d.l_knee * knee_rate;
  // d(cab)/dt by a small finite difference of the FK.
  constexpr float kEps = 1.0e-3F;
  const float dcab_h = (legInertia(hip_cad + kEps, knee).com_from_axle.z - m.cab) / kEps;
  const float dcab_k = (legInertia(hip_cad, knee + kEps).com_from_axle.z - m.cab) / kEps;
  m.com_vz = dcab_h * hip_rate + dcab_k * knee_rate;
  return m;
}

float freeRotation(float angular_momentum, const float* hip_cad, const float* knee, int samples,
                   float time_left_s) noexcept {
  if (samples <= 0 || !(time_left_s > 0.0F)) return 0.0F;
  if (samples == 1) return angular_momentum * time_left_s / std::max(legInertia(hip_cad[0], knee[0]).inertia, 1.0e-6F);
  // Trapezoid on 1 / I over evenly spaced samples.
  float sum = 0.0F;
  for (int i = 0; i < samples; ++i) {
    const float w = (i == 0 || i == samples - 1) ? 0.5F : 1.0F;
    sum += w / std::max(legInertia(hip_cad[i], knee[i]).inertia, 1.0e-6F);
  }
  return angular_momentum * sum * time_left_s / static_cast<float>(samples - 1);
}

TouchdownAttitude predictTouchdown(float body_now, float theta_v_now, float free_rotation, float leg_turn,
                                   float line_now, float line_td) noexcept {
  TouchdownAttitude a{};
  a.body = body_now + free_rotation + leg_turn;
  a.theta_v = theta_v_now + free_rotation + leg_turn + (line_td - line_now);
  return a;
}

TouchdownSensitivity touchdownSensitivity(float hip_cad, float knee, float body_com_dx_m) noexcept {
  const auto d = legInertia(hip_cad, knee);
  const float inv_i = 1.0F / std::max(d.inertia, 1.0e-6F);
  constexpr float kEps = 1.0e-3F;
  const float line = comLineAngle(hip_cad, knee, body_com_dx_m);
  TouchdownSensitivity s{};
  s.body_hip = -d.l_hip * inv_i;
  s.body_knee = -d.l_knee * inv_i;
  s.theta_hip = s.body_hip + (comLineAngle(hip_cad + kEps, knee, body_com_dx_m) - line) / kEps;
  s.theta_knee = s.body_knee + (comLineAngle(hip_cad, knee + kEps, body_com_dx_m) - line) / kEps;
  return s;
}

}  // namespace tap_fw::v3
