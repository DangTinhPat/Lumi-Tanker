// v3.5 JumpSequence: dip, thrust, measured lift-off, zero-reaction tuck/extend timeline, one-shot events, aborts.
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>

#include "tap_fw/v3/JumpPlanner.hpp"
#include "tap_fw/v3/JumpSequence.hpp"
#include "tap_fw/v3/LegHeight.hpp"

using tap_fw::v3::JumpPlan;
using tap_fw::v3::JumpSequence;
using tap_fw::v3::LegHeight;
using tap_fw::v3::planJump;
using Phase = JumpSequence::Phase;

namespace {

constexpr float kDt = 0.002F;
constexpr float kStand = 0.172F;      // standing hip-axle height the jump starts from

JumpSequence::Config enabled() {
  JumpSequence::Config c;
  c.enabled = true;
  return c;
}

JumpSequence::Input inputAt(float height_m, float accel = 9.8F, bool moving = false) {
  JumpSequence::Input in;
  in.dt_s = kDt;
  in.accel_norm_m_s2 = accel;
  in.balancing = true;
  in.moving = moving;
  const auto pose = LegHeight::forHeight(height_m);
  in.nominal_hip_rad = pose.hip_offset_rad;
  in.nominal_knee_rad = pose.knee_offset_rad;
  return in;
}

// Runs the dip (the nominal pose stays at the standing height, as in the controller) and
// returns the first Thrust output.
JumpSequence::Output dipToThrust(JumpSequence& jump) {
  JumpSequence::Output out{};
  for (int i = 0; i < 1000; ++i) {
    out = jump.update(inputAt(kStand));
    if (out.phase == Phase::Thrust) return out;
  }
  assert(false && "never left the dip");
  return out;
}

constexpr float kG = 9.80665F;

// Measured lift-off: the CoM leaves with speed v and the acceleration has dropped to -g.
JumpSequence::Input liftedInput(float v) {
  auto in = inputAt(kStand);
  in.vz_valid = true;
  in.com_vz_m_s = v;
  in.com_az_m_s2 = -kG;
  in.hip_meas_rad = LegHeight::forHeight(0.165F).hip_offset_rad;
  in.knee_meas_rad = LegHeight::forHeight(0.165F).knee_offset_rad;
  in.knee_rate_meas_rad_s = 0.0F;
  return in;
}

void disabledByDefaultDoesNothing() {
  JumpSequence jump;
  const JumpPlan plan = planJump({});
  assert(!jump.begin(plan));
  const auto out = jump.update(inputAt(kStand));
  assert(!out.active && out.phase == Phase::Idle && !jump.running());
}

void dipIsQuickSmoothAndEndsAtRest() {
  const JumpPlan plan = planJump({});
  assert(plan.ok);
  auto cfg = enabled();
  JumpSequence jump(cfg);
  assert(jump.begin(plan) && jump.running());
  auto out = jump.update(inputAt(kStand));
  assert(out.phase == Phase::Dip && out.active);
  const float dip_height = std::max(plan.crouch_height_m - cfg.dip_extra_m, LegHeight::minHeight() + 0.005F);
  float last = out.height_ref_m, t = 0.0F, last_speed = 0.0F, max_speed = 0.0F;
  while (out.phase == Phase::Dip) {
    out = jump.update(inputAt(kStand));
    if (out.phase != Phase::Dip) break;
    assert(out.height_ref_m <= last + 1e-6F);                // only goes down
    last_speed = (last - out.height_ref_m) / kDt;
    max_speed = std::max(max_speed, last_speed);
    last = out.height_ref_m;
    t += kDt;
    assert(t < 1.0F);
  }
  // Length from distance and peak acceleration, inside its limits: a counter-movement of ~0.2 s.
  const float dh = kStand - dip_height;
  const float expect = std::clamp(std::sqrt(4.9348F * dh / cfg.dip_accel_m_s2), cfg.dip_time_min_s, cfg.dip_time_max_s);
  std::printf("dip: %.0f mm in %.3f s (expected %.3f)\n", 1e3 * dh, t, expect);
  assert(std::abs(t - expect) < 3 * kDt && t <= cfg.dip_time_max_s + 3 * kDt);
  assert(std::abs(last - dip_height) < 0.002F);               // reaches the planned dip height
  assert(last_speed < 0.1F * max_speed + 0.02F);              // ... at (almost) rest
  assert(out.phase == Phase::Thrust);                         // and goes straight into the thrust
}

void waitsForTheLegsAndThenRefuses() {
  const JumpPlan plan = planJump({});
  auto cfg = enabled();
  cfg.start_wait_s = 0.2F;
  JumpSequence jump(cfg);
  assert(jump.begin(plan));
  // The legs stop in time: the dip starts instead of a refusal.
  for (int i = 0; i < 40; ++i) assert(jump.update(inputAt(kStand, 9.8F, true)).refused == 0);
  assert(jump.update(inputAt(kStand, 9.8F, false)).phase == Phase::Dip);
  jump.reset();
  assert(jump.begin(plan));
  JumpSequence::Output out{};
  for (int i = 0; i < 200 && out.refused == 0; ++i) out = jump.update(inputAt(kStand, 9.8F, true));
  assert(out.refused == 4 && out.phase == Phase::Idle && !jump.running());
  assert(jump.begin(plan));
  assert(!jump.begin(plan));                                 // already armed
}

void thrustEndsAtTheTopAtRestAndLiftOffComesFromTheMeasuredAcceleration() {
  const JumpPlan plan = planJump({});
  JumpSequence jump(enabled());
  assert(jump.begin(plan));
  auto out = dipToThrust(jump);
  float last = out.height_ref_m, last_speed = 0.0F, peak_speed = 0.0F;
  const float start = last;
  while (out.phase == Phase::Thrust) {
    out = jump.update(inputAt(kStand));              // no measurements: the path end is the lift-off
    if (out.phase != Phase::Thrust) break;
    assert(out.height_ref_m >= last - 1e-6F && out.height_ref_m <= plan.top_height_m + 1e-5F);
    last_speed = (out.height_ref_m - last) / kDt;
    peak_speed = std::max(peak_speed, last_speed);
    last = out.height_ref_m;
  }
  assert(start < plan.top_height_m - 0.01F);
  assert(std::abs(last - plan.top_height_m) < 0.004F);
  assert(last_speed < 0.2F * peak_speed);             // min-jerk: the leg path ends at rest
  assert(out.phase == Phase::Flight);

  // With measurements the thrust ends when the CoM acceleration reaches -g, and the flight is
  // planned from the measured speed.
  JumpSequence measured(enabled());
  assert(measured.begin(plan));
  (void)dipToThrust(measured);
  float t = 0.0F;
  while (measured.phase() == Phase::Thrust && t < 0.2F) {
    auto in = inputAt(kStand);
    in.vz_valid = true;
    in.com_vz_m_s = 0.4F;
    in.com_az_m_s2 = t < 0.06F ? 5.0F : -kG;         // the wheels carry the load, then free fall
    if (t >= 0.06F) in.com_vz_m_s = 0.9F;
    (void)measured.update(in);
    t += kDt;
  }
  assert(measured.phase() == Phase::Flight);
  assert(t > 0.045F && t < 0.2F);
  assert(std::abs(measured.flightPlan().vertical_speed_m_s - 0.9F) < 1e-4F);
  assert(std::abs(measured.flightPlan().flight_time_s - 2.0F * 0.9F / kG) < 1e-4F);
}

void flightTucksWithTheZeroReactionHipAndExtendsBeforeTheLanding() {
  const JumpPlan plan = planJump({});
  auto cfg = enabled();
  cfg.flight_delay_s = 0.0F;
  cfg.land_lead_s = 0.0F;     // compliance exactly at the planned touchdown
  JumpSequence jump(cfg);
  assert(jump.begin(plan));
  (void)dipToThrust(jump);
  JumpSequence::Output out{};
  for (int i = 0; i < 400 && jump.phase() != Phase::Flight; ++i) out = jump.update(liftedInput(plan.takeoff_speed_m_s));
  assert(jump.phase() == Phase::Flight);
  float t = 0.0F, lowest = 1e9F, extended_at = -1.0F;
  const auto& fp = jump.flightPlan();
  while (out.phase == Phase::Flight || t == 0.0F) {
    out = jump.update(liftedInput(plan.takeoff_speed_m_s));
    if (out.phase != Phase::Flight) break;
    assert(out.hold_wheels && out.active && out.absolute);
    // Commanded legs follow the zero-reaction curve: the body would not turn.
    assert(std::abs(out.hip_rad + LegHeight::hangHipCad() - fp.hipAt(t + kDt)) < 1e-4F);
    lowest = std::min(lowest, out.knee_rad);
    if (extended_at < 0.0F && fp.tuck_planned && t > fp.tuck_start_s + fp.tuck_time_s &&
        out.knee_rad > fp.knee_top_rad - 0.02F) extended_at = t;
    t += kDt;
    assert(t < 1.0F);
  }
  assert(fp.tuck_planned && fp.ok);
  std::printf("flight %.0f ms: knee %.2f -> %.2f -> back at %.0f ms, planned clearance %.0f mm\n", 1e3 * t,
              fp.knee_top_rad, lowest, 1e3 * extended_at, 1e3 * fp.predicted_clearance_m);
  assert(lowest < fp.knee_top_rad - 0.1F && std::abs(lowest - fp.knee_tuck_rad) < 0.02F);   // folded as planned
  assert(extended_at > 0.0F && extended_at <= t - 0.5F * cfg.planner.touchdown_margin_s);   // out again before the touchdown
  assert(std::abs(t - fp.flight_time_s) < 4 * kDt);                                         // Land at the planned time
  assert(out.phase == Phase::Land);
}

void eventsFireOnce() {
  const JumpPlan plan = planJump({});
  JumpSequence jump(enabled());
  assert(jump.begin(plan));
  auto out = dipToThrust(jump);
  int adopt = 0, landed = 0;
  bool wheels_held_in_land = false, saw_recover = false;
  for (int i = 0; i < 4000 && jump.running(); ++i) {
    const float accel = jump.phase() == Phase::Land ? 30.0F : 5.0F;   // impact once in Land
    out = jump.update(inputAt(kStand, accel));
    adopt += out.adopt;
    landed += out.landed;
    if (out.phase == Phase::Land && out.hold_wheels) wheels_held_in_land = true;
    if (out.phase == Phase::Recover) { saw_recover = true; assert(!out.hold_wheels); }
  }
  assert(adopt == 1 && landed == 1 && wheels_held_in_land && saw_recover && !jump.running());
}

void impactEndsTheFlightEarlyOnlyAfterMinFlight() {
  const JumpPlan plan = planJump({});
  auto cfg = enabled();
  cfg.flight_delay_s = 0.5F;       // plan landing far away, so only the spike can end it
  cfg.min_flight_s = 0.10F;
  JumpSequence jump(cfg);
  assert(jump.begin(plan));
  (void)dipToThrust(jump);
  while (jump.phase() == Phase::Thrust) (void)jump.update(inputAt(kStand));
  float t = 0.0F;
  while (t < 0.09F) { assert(jump.update(inputAt(kStand, 40.0F)).phase == Phase::Flight); t += kDt; }
  bool landed = false;
  while (t < 0.3F && !landed) { landed = jump.update(inputAt(kStand, 40.0F)).phase == Phase::Land; t += kDt; }
  assert(landed && t < 0.2F);
}

void abortsSoftlyAndNeverRunsWithoutBalancing() {
  const JumpPlan plan = planJump({});
  JumpSequence jump(enabled());
  assert(jump.begin(plan));
  (void)dipToThrust(jump);
  auto in = inputAt(kStand);
  in.pitch_rad = 1.2F;                                   // past abort_pitch_rad
  assert(jump.update(in).phase == Phase::Land);
  jump.reset();
  assert(jump.begin(plan));
  in = inputAt(kStand);
  in.balancing = false;
  assert(jump.update(in).phase == Phase::Idle && !jump.running());
}

void aThrustThatNeverLiftsOffEndsSoftly() {
  const JumpPlan plan = planJump({});
  JumpSequence jump(enabled());
  assert(jump.begin(plan));
  (void)dipToThrust(jump);
  // The speed stays below the flight threshold: the legs land again instead of "flying" at nothing.
  JumpSequence::Output out{};
  for (int i = 0; i < 400 && jump.phase() == Phase::Thrust; ++i) {
    auto in = inputAt(kStand);
    in.vz_valid = true;
    in.com_vz_m_s = 0.05F;
    in.com_az_m_s2 = -kG;
    out = jump.update(in);
  }
  assert(jump.phase() == Phase::Land && out.adopt);
}

}  // namespace

int main() {
  disabledByDefaultDoesNothing();
  dipIsQuickSmoothAndEndsAtRest();
  waitsForTheLegsAndThenRefuses();
  thrustEndsAtTheTopAtRestAndLiftOffComesFromTheMeasuredAcceleration();
  flightTucksWithTheZeroReactionHipAndExtendsBeforeTheLanding();
  aThrustThatNeverLiftsOffEndsSoftly();
  eventsFireOnce();
  impactEndsTheFlightEarlyOnlyAfterMinFlight();
  abortsSoftlyAndNeverRunsWithoutBalancing();
  std::puts("v3 jump tests passed");
  return 0;
}
