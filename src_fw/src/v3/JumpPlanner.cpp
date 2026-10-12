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
inline Pt add(Pt a, Pt b) noexcept { return {a.x + b.x, a.z + b.z}; }
inline float minJerk(float u) noexcept {
  u = std::clamp(u, 0.0F, 1.0F);
  return u * u * u * (10.0F + u * (-15.0F + 6.0F * u));
}

// Planner views of the AirborneModel (aac/AirborneModel.hpp).
struct LegDynamics {
  float cab = 0.0F;      // CoM above the axle
  float inertia = 0.0F;  // about the CoM
  float l_hip = 0.0F;
  float l_knee = 0.0F;
};
LegDynamics legDynamics(float hip_cad, float knee) noexcept {
  const LegInertia d = legInertia(hip_cad, knee);
  return {d.com_from_axle.z, d.inertia, d.l_hip, d.l_knee};
}
Pt axleFromCom(float hip_cad, float knee) noexcept {
  const Vec2 c = legInertia(hip_cad, knee).com_from_axle;
  return {-c.x, -c.z};
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
  return knee_tuck_rad + (kneeEnd() - knee_tuck_rad) * minJerk((t - extend_start_s) / std::max(extend_time_s, 1.0e-3F));
}

float FlightPlan::tuckHip(float knee) const noexcept {
  const float span = knee_top_rad - knee_tuck_rad;
  const float s = span > 1.0e-4F ? std::clamp((knee_top_rad - knee) / span, 0.0F, 1.0F) : 0.0F;
  return zeroReactionHip(knee) + hip_dev_rad * s;
}

float FlightPlan::pathTurn(float knee_from, float knee_to) const noexcept {
  return gridInterp(path_turn_rad, knee_lo_rad, knee_step_rad, knee_to) -
         gridInterp(path_turn_rad, knee_lo_rad, knee_step_rad, knee_from);
}

float FlightPlan::hipAt(float t) const noexcept {
  const float k = kneeAt(t);
  const float fade = tuck_planned && t >= tuck_start_s && tuck_time_s > 0.0F
      ? 1.0F - minJerk((t - tuck_start_s) / tuck_time_s) : (tuck_planned ? 1.0F : 0.0F);
  return tuckHip(k) + hip_offset_cad_rad * fade;
}

namespace {

// Tuck path to tuck_knee with pull-up dev: hip = zero-reaction + dev * s. Returns the body turn at the
// tuck (zero total angular momentum, from the top pose) and the wheel lift toward the CoM in the world
// frame there (the frame in which the axle is straight below the CoM at the top pose).
struct TuckEval { float turn, lift; };
TuckEval evalTuck(const FlightPlan& f, float dev, float tuck_knee, float base_angle, Pt top_world) noexcept {
  FlightPlan g = f;
  g.knee_tuck_rad = tuck_knee;
  g.hip_dev_rad = dev;
  constexpr int kSegments = 8;
  float turn = 0.0F;
  float k0 = g.knee_top_rad, h0 = g.tuckHip(k0);
  for (int i = 1; i <= kSegments; ++i) {
    const float k1 = g.knee_top_rad + (tuck_knee - g.knee_top_rad) * static_cast<float>(i) / kSegments;
    const float h1 = g.tuckHip(k1);
    turn += bodyTurnFromLegPath(h0, k0, h1, k1);
    k0 = k1;
    h0 = h1;
  }
  const Pt w = rot(axleFromCom(h0, k0), turn - base_angle);
  return {turn, w.z - top_world.z};
}

}  // namespace

bool FlightPlan::buildCurve(const JumpPlannerConfig& c) noexcept {
  // Zero-reaction curve through the top pose: hip as a function of the knee, both ways.
  const auto top = LegHeight::forHeight(c.top_height_m);
  const float hip_top = LegHeight::hangHipCad() + top.hip_offset_rad;
  knee_top_rad = top.knee_offset_rad;
  knee_tuck_rad = knee_top_rad;
  hip_dev_rad = 0.0F;
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
  // Tuck tables: for every tuck depth the hip pull-up that lifts the wheels most while the body turns
  // at most tuck_body_turn_rad (coarse scan, then a fine one around the best).
  const Pt top_axle = axleFromCom(hip_top, knee_top_rad);
  const float base_angle = std::atan2(top_axle.x, -top_axle.z);
  const Pt top_world = rot(top_axle, -base_angle);
  const float turn_max = std::max(c.tuck_body_turn_rad, 0.0F);
  for (int i = 0; i < kGrid; ++i) {
    const auto u = static_cast<std::size_t>(i);
    const float k = knee_lo_rad + knee_step_rad * static_cast<float>(i);
    dev_rad[u] = 0.0F;
    turn_rad[u] = 0.0F;
    path_turn_rad[u] = 0.0F;
    if (i >= i_top) {
      lift_m[u] = 0.0F;
      continue;
    }
    TuckEval best = evalTuck(*this, 0.0F, k, base_angle, top_world);
    float best_dev = 0.0F;
    if (turn_max > 0.0F) {
      const auto tryDev = [&](float dev) {
        const TuckEval e = evalTuck(*this, dev, k, base_angle, top_world);
        if (std::abs(e.turn) <= turn_max && e.lift > best.lift) { best = e; best_dev = dev; }
      };
      for (float dev = -0.6F; dev <= 0.6001F; dev += 0.1F) tryDev(dev);
      const float centre = best_dev;
      for (float dev = centre - 0.09F; dev <= centre + 0.0901F; dev += 0.01F) tryDev(dev);
    }
    dev_rad[u] = best_dev;
    lift_m[u] = best.lift;
    turn_rad[u] = best.turn;
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
  knee_tuck_rad = knee_top_rad;
  extend_fraction_ = std::clamp(c.extend_fraction, 0.3F, 1.0F);
  hip_dev_rad = 0.0F;
  hip_offset_cad_rad = hip_cad - tuckHip(knee);
  tuck_time_s = 0.0F;
  extend_time_s = 0.0F;
  extend_start_s = flight_time_s;
  predicted_clearance_m = com_rise_m;
  tuck_lift_m = 0.0F;
  tuck_turn_rad = 0.0F;
  peak_knee_accel_rad_s2 = 0.0F;
  peak_hip_accel_rad_s2 = 0.0F;
  path_turn_rad.fill(0.0F);
  reason = "ok";
  ok = true;
  const float usable = flight_time_s - t_now - c.hold_s - c.touchdown_margin_s;
  if (usable <= 0.03F) {
    reason = "flight too short to fold the legs";
    return true;
  }
  // The tuck starts at once, from the measured state (so it also has to turn the knee around if it is still
  // extending), and is complete when the CoM reaches its apex (tuck_done_frac of the flight): that is when
  // the wheels are highest. Then the legs are lowered gradually until shortly before the touchdown. Both
  // joints must stay inside their acceleration budgets along the whole timeline.
  const float t_tuck_end = std::clamp(c.tuck_done_frac * flight_time_s, t_now + 0.03F, flight_time_s - c.touchdown_margin_s - 0.05F);
  const float t1 = t_tuck_end - t_now;
  const float t2 = flight_time_s - c.touchdown_margin_s - t_tuck_end - c.hold_s;
  tuck_time_s = t1;
  extend_time_s = t2;
  extend_start_s = t_tuck_end + c.hold_s;
  const float lift_needed = c.target_clearance_m - com_rise_m;
  const float depth_max = knee_top_rad - c.tuck_min_knee_rad;
  const auto liftAt = [&](float tuck_knee) { return gridInterp(lift_m, knee_lo_rad, knee_step_rad, tuck_knee); };
  const auto devAt = [&](float tuck_knee) { return gridInterp(dev_rad, knee_lo_rad, knee_step_rad, tuck_knee); };
  struct Peaks { float knee, hip; };
  const auto peaks = [&](float tuck_knee) {
    FlightPlan g = *this;
    g.knee_tuck_rad = tuck_knee;
    g.hip_dev_rad = devAt(tuck_knee);
    Peaks pk{0.0F, 0.0F};
    constexpr int kSamples = 48;
    const float dt = (flight_time_s - t_now) / kSamples;
    float k_prev = g.kneeAt(t_now), h_prev = g.hipAt(t_now);
    float kr_prev = g.knee_rate_start_rad_s, hr_prev = 0.0F;
    bool first = true;
    for (int i = 1; i <= kSamples; ++i) {
      const float t = t_now + dt * static_cast<float>(i);
      const float k = g.kneeAt(t), h = g.hipAt(t);
      const float kr = (k - k_prev) / dt, hr = (h - h_prev) / dt;
      pk.knee = std::max(pk.knee, std::abs(kr - kr_prev) / dt);
      if (!first) pk.hip = std::max(pk.hip, std::abs(hr - hr_prev) / dt);   // the hip's start rate is not measured
      first = false;
      k_prev = k; h_prev = h; kr_prev = kr; hr_prev = hr;
    }
    return pk;
  };
  const auto withinBudget = [&](float tuck_knee) {
    const Peaks pk = peaks(tuck_knee);
    return pk.knee <= c.knee_accel_rad_s2 && pk.hip <= c.hip_accel_rad_s2;
  };
  float depth = 0.0F;
  if (lift_needed > 0.0F && depth_max > 0.0F) {
    float lo = 0.0F, hi = depth_max;
    if (liftAt(knee_top_rad - hi) < lift_needed) {
      depth = hi;
      reason = "tuck limited by the deepest allowed knee";
    } else {
      for (int i = 0; i < 24; ++i) {
        const float mid = 0.5F * (lo + hi);
        if (liftAt(knee_top_rad - mid) >= lift_needed) hi = mid;
        else lo = mid;
      }
      depth = hi;
    }
    if (!withinBudget(knee_top_rad - depth)) {      // ... inside the acceleration budgets
      float a = 0.0F, b = depth;
      for (int i = 0; i < 20; ++i) {
        const float mid = 0.5F * (a + b);
        if (withinBudget(knee_top_rad - mid)) a = mid;
        else b = mid;
      }
      depth = a;
      reason = "flight too short for the full tuck: shallower tuck";
    }
  }
  knee_tuck_rad = knee_top_rad - depth;
  hip_dev_rad = depth > 1.0e-4F ? devAt(knee_tuck_rad) : 0.0F;
  hip_offset_cad_rad = hip_cad - tuckHip(knee);
  const Peaks pk = peaks(knee_tuck_rad);
  peak_knee_accel_rad_s2 = pk.knee;
  peak_hip_accel_rad_s2 = pk.hip;
  // Body turn along the planned path, cumulative from the top pose (used by the touchdown law).
  const int i_top = static_cast<int>(std::lround((knee_top_rad - knee_lo_rad) / knee_step_rad));
  for (int dir = -1; dir <= 1; dir += 2) {
    float acc = 0.0F;
    for (int i = i_top + dir; i >= 0 && i < kGrid; i += dir) {
      const float ka = knee_lo_rad + knee_step_rad * static_cast<float>(i - dir);
      const float kb = knee_lo_rad + knee_step_rad * static_cast<float>(i);
      acc += bodyTurnFromLegPath(tuckHip(ka), ka, tuckHip(kb), kb);
      path_turn_rad[static_cast<std::size_t>(i)] = acc;
    }
  }
  tuck_turn_rad = pathTurn(knee_top_rad, knee_tuck_rad);
  // Wheel lift at the apex, on the planned path, in the world frame.
  {
    const float k_apex = kneeAt(0.5F * flight_time_s);
    const auto top = axleFromCom(zeroReactionHip(knee_top_rad), knee_top_rad);
    const float base_angle = std::atan2(top.x, -top.z);
    const Pt w0 = rot(top, -base_angle);
    const Pt w = rot(axleFromCom(tuckHip(k_apex), k_apex), pathTurn(knee_top_rad, k_apex) - base_angle);
    tuck_lift_m = w.z - w0.z;
  }
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
  FlightPlan curve{};
  curve.buildCurve(c);                 // the expensive part, once
  FlightPlan best{};
  float rise = c.com_rise_max_m;
  bool reached = false;
  for (int i = 0; i <= 12; ++i) {
    const float r = c.com_rise_min_m + (c.com_rise_max_m - c.com_rise_min_m) * static_cast<float>(i) / 12.0F;
    FlightPlan f = curve;
    f.planTuck(c, std::sqrt(2.0F * kGravity * r), 0.0F, hip_top, top.knee_offset_rad, 0.0F);
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
  plan.predicted_body_turn_rad = best.tuck_turn_rad;
  plan.curve = curve;
  return plan;
}

}  // namespace tap_fw::v3
