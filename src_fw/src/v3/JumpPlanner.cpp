#include "tap_fw/v3/JumpPlanner.hpp"

#include <algorithm>
#include <cmath>

#include "robot_control/GeneratedGeometry.hpp"
#include "tap_fw/v3/LegHeight.hpp"

namespace tap_fw::v3 {
namespace {

constexpr float kGravity = 9.80665F;
constexpr float kMinJerkAccel = 5.7735F;   // peak acceleration of a min-jerk move = 5.7735 * distance / time^2
constexpr float kMinJerkRate = 1.875F;     // peak rate = 1.875 * distance / time

struct Pt { float x, z; };
inline Pt rot(Pt p, float th) noexcept {
  const float c = std::cos(th), s = std::sin(th);
  return {c * p.x + s * p.z, -s * p.x + c * p.z};
}
inline Pt perp(Pt v) noexcept { return {v.z, -v.x}; }
inline Pt add(Pt a, Pt b) noexcept { return {a.x + b.x, a.z + b.z}; }
inline float minJerk(float u) noexcept {
  u = std::clamp(u, 0.0F, 1.0F);
  return u * u * u * (10.0F + u * (-15.0F + 6.0F * u));
}

// Both legs move together. Links: body, thigh, shin (with the wheel motor), wheel; the CAD chain of
// WholeBodyModel::compute (thigh turns with the hip, shin and wheel with hip+knee).
struct LegDynamics {
  float cab = 0.0F;      // CoM above the axle
  float inertia = 0.0F;  // about the CoM
  float l_hip = 0.0F;    // angular momentum about the CoM per unit hip rate (body held)
  float l_knee = 0.0F;   // ... per unit knee rate
};

LegDynamics legDynamics(float hip_cad, float knee) noexcept {
  namespace geo = robot_control::geometry;
  const float a = hip_cad;
  const float b = hip_cad + knee + geo::knee_zero_rebase_rad;
  const Pt h{geo::origins[0].x, geo::origins[0].z};
  const Pt o1{geo::origins[1].x, geo::origins[1].z}, o2{geo::origins[2].x, geo::origins[2].z};
  const Pt c1{geo::parts[1].x, geo::parts[1].z}, c2{geo::parts[2].x, geo::parts[2].z};
  const Pt ro1 = rot(o1, a), ro2 = rot(o2, b), rc1 = rot(c1, a), rc2 = rot(c2, b);
  const Pt knee_origin = add(h, ro1), axle = add(knee_origin, ro2);
  struct Link { float m, i; Pt p, dp_hip, dp_knee; float w_hip, w_knee; };
  const Link links[4] = {
      {geo::parts[0].mass, geo::parts[0].iyy, {geo::parts[0].x, geo::parts[0].z}, {0, 0}, {0, 0}, 0, 0},
      {2.0F * geo::parts[1].mass, 2.0F * geo::parts[1].iyy, add(h, rc1), perp(rc1), {0, 0}, 1, 0},
      {2.0F * geo::parts[2].mass, 2.0F * geo::parts[2].iyy, add(knee_origin, rc2),
       add(perp(ro1), perp(rc2)), perp(rc2), 1, 1},
      {2.0F * geo::parts[3].mass, 2.0F * geo::parts[3].iyy, axle,
       add(perp(ro1), perp(ro2)), perp(ro2), 1, 1}};
  float mass = 0.0F, cx = 0.0F, cz = 0.0F;
  for (const auto& l : links) { mass += l.m; cx += l.m * l.p.x; cz += l.m * l.p.z; }
  cx /= mass; cz /= mass;
  LegDynamics out{};
  out.cab = cz - axle.z;
  for (const auto& l : links) {
    const float dx = l.p.x - cx, dz = l.p.z - cz;
    out.inertia += l.i + l.m * (dx * dx + dz * dz);
    out.l_hip += l.m * (dz * l.dp_hip.x - dx * l.dp_hip.z) + l.i * l.w_hip;
    out.l_knee += l.m * (dz * l.dp_knee.x - dx * l.dp_knee.z) + l.i * l.w_knee;
  }
  return out;
}

float comAboveAxle(float height_m) noexcept {
  return LegHeight::forHeight(height_m).com_above_axle_m;
}

// Vertical force both legs can add at height h along the axle-under-hip path (N): joint torque limit
// times the joint rates per unit height rate (virtual work).
float legForce(const JumpPlannerConfig& c, float h) noexcept {
  const float d = 0.002F;
  const auto lo = LegHeight::forHeight(h - d);
  const auto hi = LegHeight::forHeight(h + d);
  const float dk = std::abs(hi.knee_offset_rad - lo.knee_offset_rad) / (2.0F * d);
  const float dh = std::abs(hi.hip_offset_rad - lo.hip_offset_rad) / (2.0F * d);
  return 2.0F * c.torque_limit_nm * c.torque_derate * (dk + dh);
}

}  // namespace

float bodyRateFromLegMotion(float hip_cad, float knee, float hip_rate, float knee_rate) noexcept {
  const auto d = legDynamics(hip_cad, knee);
  return -(d.l_hip * hip_rate + d.l_knee * knee_rate) / std::max(d.inertia, 1.0e-6F);
}

float bodyTurnFromLegPath(float hip0_cad, float knee0, float hip1_cad, float knee1) noexcept {
  // Zero total angular momentum: the body turns by -(L_h dh + L_k dk) / I along the leg path (a
  // 1-form, taken along the straight line between the two poses with Simpson weights).
  const float dh = hip1_cad - hip0_cad, dk = knee1 - knee0;
  float turn = 0.0F;
  constexpr float w[3] = {1.0F / 6.0F, 4.0F / 6.0F, 1.0F / 6.0F};
  for (int i = 0; i < 3; ++i) {
    const float u = 0.5F * static_cast<float>(i);
    const auto d = legDynamics(hip0_cad + u * dh, knee0 + u * dk);
    turn -= w[i] * (d.l_hip * dh + d.l_knee * dk) / std::max(d.inertia, 1.0e-6F);
  }
  return turn;
}

BodyMomentum bodyMomentum(float hip_cad, float knee, float hip_rate, float knee_rate,
                          float body_rate) noexcept {
  const auto d = legDynamics(hip_cad, knee);
  BodyMomentum m{};
  m.inertia = d.inertia;
  m.cab = d.cab;
  m.angular_momentum = d.inertia * body_rate + d.l_hip * hip_rate + d.l_knee * knee_rate;
  // d(cab)/dt = (dcab/dhip) hip_rate + (dcab/dknee) knee_rate by a small finite difference of the FK
  constexpr float kEps = 1.0e-3F;
  const float dcab_h = (legDynamics(hip_cad + kEps, knee).cab - d.cab) / kEps;
  const float dcab_k = (legDynamics(hip_cad, knee + kEps).cab - d.cab) / kEps;
  m.com_vz = dcab_h * hip_rate + dcab_k * knee_rate;
  return m;
}

float hipPerBodyRate(float hip_cad, float knee) noexcept {
  const auto d = legDynamics(hip_cad, knee);
  return std::abs(d.l_hip) > 1.0e-6F ? d.inertia / d.l_hip : 0.0F;
}

static float gridInterp(const std::array<float, FlightPlan::kGrid>& a, float lo, float step, float x) noexcept {
  const float u = std::clamp((x - lo) / std::max(step, 1.0e-6F), 0.0F, static_cast<float>(FlightPlan::kGrid - 1));
  const int i = std::min(static_cast<int>(u), FlightPlan::kGrid - 2);
  const float f = u - static_cast<float>(i);
  return a[static_cast<std::size_t>(i)] * (1.0F - f) + a[static_cast<std::size_t>(i + 1)] * f;
}
float FlightPlan::zeroReactionHip(float knee) const noexcept { return gridInterp(hip_cad_rad, knee_lo_rad, knee_step_rad, knee); }
float FlightPlan::cabAt(float knee) const noexcept { return gridInterp(cab_m, knee_lo_rad, knee_step_rad, knee); }

float FlightPlan::kneeAt(float t) const noexcept {
  if (!tuck_planned || t < tuck_start_s) return knee_top_rad;           // brake toward the top pose
  const float u = t - tuck_start_s;
  if (u < tuck_time_s) {                                   // quintic: (knee0, rate0, 0) -> (tuck, 0, 0)
    const float T = std::max(tuck_time_s, 1.0e-3F);
    const float d = knee_tuck_rad - knee_start_rad - knee_rate_start_rad_s * T;
    const float dv = -knee_rate_start_rad_s;
    const float c3 = (20.0F * d - 8.0F * dv * T) / (2.0F * T * T * T);
    const float c4 = (-30.0F * d + 14.0F * dv * T) / (2.0F * T * T * T * T);
    const float c5 = (12.0F * d - 6.0F * dv * T) / (2.0F * T * T * T * T * T);
    return knee_start_rad + knee_rate_start_rad_s * u + u * u * u * (c3 + u * (c4 + u * c5));
  }
  if (t < extend_start_s) return knee_tuck_rad;
  return knee_tuck_rad + (knee_top_rad - knee_tuck_rad) * minJerk((t - extend_start_s) / std::max(extend_time_s, 1.0e-3F));
}

float FlightPlan::hipAt(float t) const noexcept {
  const float k = kneeAt(t);
  const float fade = tuck_planned && t >= tuck_start_s && tuck_time_s > 0.0F
      ? 1.0F - minJerk((t - tuck_start_s) / tuck_time_s) : (tuck_planned ? 1.0F : 0.0F);
  return zeroReactionHip(k) + hip_offset_cad_rad * fade;
}

bool FlightPlan::buildCurve(const JumpPlannerConfig& c) noexcept {
  // Zero-reaction curve through the top pose: hip as a function of the knee, both ways.
  const auto top = LegHeight::forHeight(c.top_height_m);
  const float hip_top = LegHeight::hangHipCad() + top.hip_offset_rad;
  knee_top_rad = top.knee_offset_rad;
  knee_lo_rad = c.tuck_min_knee_rad;
  const float knee_hi = std::min(knee_top_rad + 0.7F, LegHeight::kKneeMaxRad);
  knee_step_rad = (knee_hi - knee_lo_rad) / static_cast<float>(kGrid - 1);
  const int i_top = std::clamp(static_cast<int>(std::lround((knee_top_rad - knee_lo_rad) / knee_step_rad)),
                               1, kGrid - 2);
  knee_lo_rad = knee_top_rad - knee_step_rad * static_cast<float>(i_top);   // grid node exactly at the top pose
  const auto slope = [](float hh, float kk) {
    const auto d = legDynamics(hh, kk);
    return std::abs(d.l_hip) > 1.0e-6F ? -d.l_knee / d.l_hip : 0.0F;      // dhip / dknee
  };
  hip_cad_rad[static_cast<std::size_t>(i_top)] = hip_top;
  cab_m[static_cast<std::size_t>(i_top)] = legDynamics(hip_top, knee_top_rad).cab;
  for (int dir = -1; dir <= 1; dir += 2) {
    float hip = hip_top, k = knee_top_rad;
    for (int i = i_top + dir; i >= 0 && i < kGrid; i += dir) {
      const float dk = knee_step_rad * static_cast<float>(dir);
      const float s1 = slope(hip, k);
      const float s2 = slope(hip + 0.5F * dk * s1, k + 0.5F * dk);          // midpoint step
      hip += dk * s2;
      k += dk;
      hip_cad_rad[static_cast<std::size_t>(i)] = hip;
      cab_m[static_cast<std::size_t>(i)] = legDynamics(hip, k).cab;
    }
  }
  return true;
}

bool FlightPlan::planTuck(const JumpPlannerConfig& c, float v, float t_now, float hip_cad, float knee,
                          float knee_rate) noexcept {
  vertical_speed_m_s = v;
  flight_time_s = 2.0F * v / kGravity;
  com_rise_m = v * v / (2.0F * kGravity);
  tuck_planned = true;
  tuck_start_s = t_now;
  knee_start_rad = knee;
  knee_rate_start_rad_s = std::clamp(knee_rate, -25.0F, 25.0F);
  hip_offset_cad_rad = hip_cad - zeroReactionHip(knee);
  knee_tuck_rad = knee_top_rad;
  tuck_time_s = 0.0F;
  extend_time_s = 0.0F;
  extend_start_s = flight_time_s;
  predicted_clearance_m = com_rise_m;
  tuck_lift_m = 0.0F;
  peak_knee_accel_rad_s2 = 0.0F;
  reason = "ok";
  ok = true;
  const float usable = flight_time_s - t_now - c.hold_s - c.touchdown_margin_s;
  if (usable <= 0.03F) {
    reason = "flight too short to fold the legs";
    return true;
  }
  // Tuck and extension share the usable time; the tuck starts from the measured state (so it also
  // has to turn the knee around if it is still extending) and must stay inside the acceleration budget.
  const float t1 = 0.5F * usable, t2 = 0.5F * usable;
  tuck_time_s = t1;
  extend_time_s = t2;
  extend_start_s = flight_time_s - c.touchdown_margin_s - t2;
  const float lift_needed = c.target_clearance_m - com_rise_m;
  const float cab_top = cabAt(knee_top_rad);
  const float depth_max = knee_top_rad - c.tuck_min_knee_rad;
  const auto peakAccel = [&](float tuck_knee) {
    FlightPlan g = *this;
    g.knee_tuck_rad = tuck_knee;
    float peak = 0.0F;
    const float dt = t1 / 16.0F;
    float prev = g.kneeAt(t_now), prev_rate = g.knee_rate_start_rad_s;
    for (int i = 1; i <= 16; ++i) {
      const float x = g.kneeAt(t_now + dt * static_cast<float>(i));
      const float rate = (x - prev) / dt;
      peak = std::max(peak, std::abs(rate - prev_rate) / dt);
      prev = x;
      prev_rate = rate;
    }
    peak = std::max(peak, kMinJerkAccel * std::abs(g.knee_top_rad - tuck_knee) / (t2 * t2));   // the extension
    return peak;
  };
  float depth = 0.0F;
  if (lift_needed > 0.0F && depth_max > 0.0F) {
    float lo = 0.0F, hi = depth_max;
    if (cab_top - cabAt(knee_top_rad - hi) < lift_needed) {
      depth = hi;
      reason = "tuck limited by the deepest allowed knee";
    } else {
      for (int i = 0; i < 24; ++i) {
        const float mid = 0.5F * (lo + hi);
        if (cab_top - cabAt(knee_top_rad - mid) >= lift_needed) hi = mid;
        else lo = mid;
      }
      depth = hi;
    }
    if (peakAccel(knee_top_rad - depth) > c.knee_accel_rad_s2) {      // ... inside the acceleration budget
      float a = 0.0F, b = depth;
      for (int i = 0; i < 20; ++i) {
        const float mid = 0.5F * (a + b);
        if (peakAccel(knee_top_rad - mid) <= c.knee_accel_rad_s2) a = mid;
        else b = mid;
      }
      depth = a;
      reason = "flight too short for the full tuck: shallower tuck";
    }
  }
  knee_tuck_rad = knee_top_rad - depth;
  peak_knee_accel_rad_s2 = peakAccel(knee_tuck_rad);
  tuck_lift_m = cab_top - cabAt(kneeAt(0.5F * flight_time_s));
  predicted_clearance_m = com_rise_m + tuck_lift_m;
  return true;
}

FlightPlan planFlight(const JumpPlannerConfig& c, float v, float hip_cad, float knee,
                      float knee_rate) noexcept {
  FlightPlan f{};
  if (!(v > 0.15F) || !std::isfinite(v) || !std::isfinite(hip_cad) || !std::isfinite(knee) ||
      !std::isfinite(knee_rate)) {
    f.reason = "no usable lift-off speed";
    return f;
  }
  f.buildCurve(c);
  f.planTuck(c, v, 0.0F, hip_cad, knee, knee_rate);
  return f;
}

JumpPlan planJump(const JumpPlannerConfig& c) noexcept {
  JumpPlan plan{};
  namespace geo = robot_control::geometry;
  const float h_min = LegHeight::minHeight(), h_max = LegHeight::maxHeight();
  const float crouch_floor = c.min_crouch_height_m > 0.0F ? c.min_crouch_height_m : h_min + 0.012F;
  if (!(c.target_clearance_m > 0.0F) || !(c.top_height_m > crouch_floor + c.min_dip_m) ||
      c.top_height_m > h_max || !(c.com_rise_min_m > 0.0F) || c.com_rise_max_m < c.com_rise_min_m) {
    plan.reason = "invalid planner configuration";
    return plan;
  }
  // Smallest CoM flight in [min, max] whose flight lets the joints tuck the wheels the rest of the way.
  const auto top = LegHeight::forHeight(c.top_height_m);
  const float hip_top = LegHeight::hangHipCad() + top.hip_offset_rad;
  FlightPlan best{};
  float rise = c.com_rise_max_m;
  bool reached = false;
  for (int i = 0; i <= 12; ++i) {
    const float r = c.com_rise_min_m + (c.com_rise_max_m - c.com_rise_min_m) * static_cast<float>(i) / 12.0F;
    const FlightPlan f = planFlight(c, std::sqrt(2.0F * kGravity * r), hip_top, top.knee_offset_rad, 0.0F);
    if (!f.ok) continue;
    best = f;
    rise = r;
    if (f.predicted_clearance_m >= c.target_clearance_m - 0.002F) { reached = true; break; }
  }
  if (!best.ok) {
    plan.reason = best.reason;
    return plan;
  }
  const float v = std::sqrt(2.0F * kGravity * rise);

  // Dip: the shortest stroke whose torque-limited work reaches the lift-off energy.
  const float mass = geo::total_mass_kg;
  const float need = c.loss_factor * 0.5F * mass * v * v;
  float stroke = 0.0F, work = 0.0F;
  const float step = 0.002F;
  bool found = false;
  for (float s = step; c.top_height_m - s >= crouch_floor - 1.0e-6F; s += step) {
    const float h = c.top_height_m - s + 0.5F * step;      // midpoint rule on [top - s, top - s + step]
    work += (legForce(c, h) - mass * kGravity) * step;
    if (s >= c.min_dip_m && work >= need) { stroke = s; found = true; break; }
  }
  if (!found) {
    plan.reason = "legs cannot reach the lift-off speed from the lowest crouch";
    return plan;
  }
  plan.ok = true;
  plan.reason = reached ? "ok" : "target clearance not reached with the joint budget";
  plan.dip_m = stroke;
  plan.crouch_height_m = c.top_height_m - stroke;
  plan.top_height_m = c.top_height_m;
  plan.takeoff_speed_m_s = v;
  plan.com_rise_m = rise;
  plan.flight_time_s = 2.0F * v / kGravity;
  plan.predicted_tuck_lift_m = best.tuck_lift_m;
  plan.predicted_clearance_m = best.predicted_clearance_m;
  plan.predicted_knee_travel_rad = best.knee_top_rad - best.knee_tuck_rad;
  return plan;
}

}  // namespace tap_fw::v3
