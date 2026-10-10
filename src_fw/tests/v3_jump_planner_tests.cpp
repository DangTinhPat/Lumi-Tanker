// v3.5 jump planner and momentum model: the plan follows from the robot's own FK/IK and mass model.
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>

#include "tap_fw/v3/JumpPlanner.hpp"
#include "tap_fw/v3/LegHeight.hpp"

using namespace tap_fw::v3;

namespace {

constexpr float kG = 9.80665F;

void planJumpPicksTheSmallestFeasibleCoMFlight() {
  JumpPlannerConfig c;
  const JumpPlan p = planJump(c);
  std::printf("plan: ok=%d '%s' dip %.1f mm crouch %.1f mm v %.2f m/s rise %.0f mm flight %.0f ms "
              "tuck lift %.1f mm clearance %.1f mm knee travel %.2f rad\n",
              p.ok, p.reason, 1e3 * p.dip_m, 1e3 * p.crouch_height_m, p.takeoff_speed_m_s, 1e3 * p.com_rise_m,
              1e3 * p.flight_time_s, 1e3 * p.predicted_tuck_lift_m, 1e3 * p.predicted_clearance_m,
              p.predicted_knee_travel_rad);
  assert(p.ok);
  assert(p.crouch_height_m < p.top_height_m && p.crouch_height_m > LegHeight::minHeight());
  assert(p.com_rise_m >= c.com_rise_min_m - 1e-4F && p.com_rise_m <= c.com_rise_max_m + 1e-4F);
  assert(std::abs(p.takeoff_speed_m_s - std::sqrt(2.0F * kG * p.com_rise_m)) < 1e-3F);
  assert(std::abs(p.flight_time_s - 2.0F * p.takeoff_speed_m_s / kG) < 1e-3F);
  // A joint budget that is large enough reaches the clearance with the smallest CoM flight; a weaker
  // one needs a higher flight for the same clearance.
  JumpPlannerConfig strong = c, weak = c;
  strong.target_clearance_m = weak.target_clearance_m = 0.09F;
  strong.knee_accel_rad_s2 = 700.0F;
  weak.knee_accel_rad_s2 = 250.0F;
  strong.max_knee_rate_rad_s = weak.max_knee_rate_rad_s = 40.0F;
  const JumpPlan ps = planJump(strong), pw = planJump(weak);
  assert(ps.ok && pw.ok);
  assert(ps.com_rise_m <= pw.com_rise_m + 1e-4F);
  // A bigger target needs a higher CoM flight and a deeper dip.
  JumpPlannerConfig small = c, big = c;
  small.target_clearance_m = 0.05F;
  big.target_clearance_m = 0.15F;
  const JumpPlan psm = planJump(small), pbg = planJump(big);
  assert(psm.ok && pbg.ok && pbg.takeoff_speed_m_s >= psm.takeoff_speed_m_s);
  assert(pbg.dip_m >= psm.dip_m - 1e-4F);
}

void zeroReactionTuckDoesNotTurnTheBody() {
  JumpPlannerConfig c;
  const auto top = LegHeight::forHeight(c.top_height_m);
  const float hip = LegHeight::hangHipCad() + top.hip_offset_rad;
  const FlightPlan f = planFlight(c, 1.0F, hip, top.knee_offset_rad, 0.0F);
  std::printf("flight: ok=%d '%s' knee %.2f -> %.2f lift %.1f mm clearance %.1f mm peak acc %.0f\n", f.ok,
              f.reason, f.knee_start_rad, f.knee_tuck_rad, 1e3 * f.tuck_lift_m, 1e3 * f.predicted_clearance_m,
              f.peak_knee_accel_rad_s2);
  assert(f.ok && f.knee_tuck_rad < f.knee_top_rad - 0.2F);
  assert(f.peak_knee_accel_rad_s2 <= c.knee_accel_rad_s2 * 1.02F);
  assert(std::abs(f.kneeAt(0.0F) - f.knee_start_rad) < 1e-4F);
  // At rest the tuck starts at once, holds, and is extended again before the touchdown.
  assert(std::abs(f.kneeAt(f.flight_time_s) - f.knee_top_rad) < 1e-3F);
  assert(f.extend_start_s + f.extend_time_s <= f.flight_time_s - c.touchdown_margin_s + 1e-4F);
  // Integrate the body rotation that the planned hip/knee motion causes (angular momentum model):
  // it must stay ~0 along the whole path, while the same knee fold with a fixed hip turns the body.
  float turned_zr = 0.0F, turned_fixed = 0.0F, worst_zr = 0.0F;
  const float dt = 0.001F;
  for (float t = 0.0F; t < f.flight_time_s; t += dt) {
    const float k0 = f.kneeAt(t), k1 = f.kneeAt(t + dt);
    const float h0 = f.hipAt(t), h1 = f.hipAt(t + dt);
    turned_zr += bodyRateFromLegMotion(h0, k0, (h1 - h0) / dt, (k1 - k0) / dt) * dt;
    turned_fixed += bodyRateFromLegMotion(f.hipAt(0.0F), k0, 0.0F, (k1 - k0) / dt) * dt;
    worst_zr = std::max(worst_zr, std::abs(turned_zr));
  }
  std::printf("body rotation over the flight: zero-reaction path %.4f rad (worst %.4f), fixed hip %.4f rad\n",
              turned_zr, worst_zr, turned_fixed);
  assert(worst_zr < 0.02F);
  // (the fixed-hip fold is checked at its deepest point: the body would have turned noticeably)
  float deepest_fixed = 0.0F;
  turned_fixed = 0.0F;
  for (float t = 0.0F; t < 0.5F * f.flight_time_s; t += dt) {
    const float k0 = f.kneeAt(t), k1 = f.kneeAt(t + dt);
    turned_fixed += bodyRateFromLegMotion(f.hipAt(0.0F), k0, 0.0F, (k1 - k0) / dt) * dt;
    deepest_fixed = std::max(deepest_fixed, std::abs(turned_fixed));
  }
  assert(deepest_fixed > 5.0F * std::max(worst_zr, 0.004F));
}

void tuckStartsFromTheMeasuredKneeRate() {
  JumpPlannerConfig c;
  const auto top = LegHeight::forHeight(c.top_height_m);
  const float hip = LegHeight::hangHipCad() + top.hip_offset_rad;
  const float knee = top.knee_offset_rad + 0.1F;       // still extending, a bit above the top pose
  FlightPlan f;
  f.buildCurve(c);
  f.planTuck(c, 1.1F, 0.03F, hip + 0.05F, knee, 3.0F);
  assert(f.ok && f.tuck_planned);
  const float eps = 1e-3F;
  const float rate = (f.kneeAt(f.tuck_start_s + eps) - f.kneeAt(f.tuck_start_s)) / eps;
  assert(std::abs(f.kneeAt(f.tuck_start_s) - knee) < 1e-3F);
  assert(std::abs(rate - 3.0F) < 0.4F);                // continuous with the measured rate
  assert(f.knee_tuck_rad < f.knee_top_rad - 0.05F);    // it still folds, a bit shallower
  assert(f.peak_knee_accel_rad_s2 <= c.knee_accel_rad_s2 * 1.02F);
  // Before the tuck starts the legs brake toward the top pose; at the end they are back at it.
  assert(std::abs(f.kneeAt(0.0F) - f.knee_top_rad) < 1e-4F);
  assert(std::abs(f.kneeAt(f.flight_time_s) - f.knee_top_rad) < 1e-3F);
  // The hip offset measured at the start fades out.
  assert(std::abs(f.hipAt(f.flight_time_s) - f.zeroReactionHip(f.knee_top_rad)) < 1e-3F);
  // Still extending so fast that braking alone exceeds the acceleration budget: no tuck is planned and
  // the plan reports that budget honestly instead of claiming a fold it cannot make.
  FlightPlan g;
  g.buildCurve(c);
  g.planTuck(c, 1.1F, 0.03F, hip, knee, 10.0F);
  assert(g.ok && std::abs(g.knee_tuck_rad - g.knee_top_rad) < 1e-4F);
  assert(g.peak_knee_accel_rad_s2 > c.knee_accel_rad_s2);
}

void flightPlanRefusesWhatCannotFly() {
  JumpPlannerConfig c;
  const auto top = LegHeight::forHeight(c.top_height_m);
  const float hip = LegHeight::hangHipCad() + top.hip_offset_rad;
  assert(!planFlight(c, 0.05F, hip, top.knee_offset_rad, 0.0F).ok);       // too slow to leave the ground
  assert(!planFlight(c, std::nanf(""), hip, top.knee_offset_rad, 0.0F).ok);
  // A flight that is too short for any tuck still plans (no tuck) and reports the clearance honestly.
  const FlightPlan f = planFlight(c, 0.3F, hip, top.knee_offset_rad, 0.0F);
  assert(f.ok && f.predicted_clearance_m < 0.06F);
  JumpPlannerConfig bad;
  bad.top_height_m = 0.01F;
  assert(!planJump(bad).ok);
  JumpPlannerConfig weak;
  weak.torque_limit_nm = 0.2F;                          // legs too weak to lift the body
  assert(!planJump(weak).ok);
}

void momentumModelIsConsistent() {
  const auto p = LegHeight::forHeight(0.15F);
  const float hip = LegHeight::hangHipCad() + p.hip_offset_rad;
  // Only the body turns: L = I w.
  const auto a = bodyMomentum(hip, p.knee_offset_rad, 0.0F, 0.0F, 2.0F);
  assert(std::abs(a.angular_momentum - a.inertia * 2.0F) < 1e-6F);
  // Zero total momentum: the body turns by exactly what bodyRateFromLegMotion says.
  const float w = bodyRateFromLegMotion(hip, p.knee_offset_rad, 1.5F, -2.0F);
  const auto b = bodyMomentum(hip, p.knee_offset_rad, 1.5F, -2.0F, w);
  assert(std::abs(b.angular_momentum) < 1e-5F);
  // I / L_h converts a body rate into the hip rate that cancels it.
  const float ratio = hipPerBodyRate(hip, p.knee_offset_rad);
  assert(ratio != 0.0F && std::isfinite(ratio));
  const float w0 = bodyRateFromLegMotion(hip, p.knee_offset_rad, 0.0F, 0.0F);
  assert(std::abs(w0) < 1e-9F);
  // The path form agrees with integrating the rate form along the same straight path.
  const float h1 = hip + 0.25F, k1 = p.knee_offset_rad - 0.3F;
  float integrated = 0.0F;
  const int n = 200;
  for (int i = 0; i < n; ++i) {
    const float u = (static_cast<float>(i) + 0.5F) / static_cast<float>(n);
    integrated += bodyRateFromLegMotion(hip + u * (h1 - hip), p.knee_offset_rad + u * (k1 - p.knee_offset_rad),
                                        (h1 - hip) / static_cast<float>(n), (k1 - p.knee_offset_rad) / static_cast<float>(n));
  }
  assert(std::abs(bodyTurnFromLegPath(hip, p.knee_offset_rad, h1, k1) - integrated) < 2.0e-3F);
  // Hip only: the body turns against the hip (reaction wheel), by I / L_h per rad.
  const float turn = bodyTurnFromLegPath(hip, p.knee_offset_rad, hip + 0.1F, p.knee_offset_rad);
  assert(turn * ratio < 0.0F && std::abs(std::abs(turn) * ratio - 0.1F) < 0.02F);
  // The CoM rises relative to the axle when the legs extend.
  const auto up = bodyMomentum(hip, p.knee_offset_rad, -0.5F, 3.0F, 0.0F);
  assert(std::isfinite(up.com_vz) && std::isfinite(up.cab) && up.cab > 0.0F);
}

}  // namespace

int main() {
  std::printf("hip-axle height range %.1f .. %.1f mm\n", 1e3 * LegHeight::minHeight(),
              1e3 * LegHeight::maxHeight());
  planJumpPicksTheSmallestFeasibleCoMFlight();
  zeroReactionTuckDoesNotTurnTheBody();
  tuckStartsFromTheMeasuredKneeRate();
  flightPlanRefusesWhatCannotFly();
  momentumModelIsConsistent();
  std::puts("v3 jump planner tests passed");
  return 0;
}
