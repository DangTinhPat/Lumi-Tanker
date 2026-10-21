// HopSequence: pure state machine of the small vertical hop (docs/V3.7_HOP_20261021.md).
#include <cassert>
#include <cmath>
#include <cstdio>

#include "tap_fw/v3/HopSequence.hpp"

using tap_fw::v3::HopSequence;

namespace {
constexpr float kDt = 0.002F;
constexpr float kG = 9.80665F;

HopSequence::Config enabledConfig() {
  HopSequence::Config c;
  c.enabled = true;
  c.adapt = false;
  c.thrust_gain = 1.0F;   // the shipped default is lower (tuned in the sim); tests use the plan as designed
  return c;
}

struct Run {
  int dip = 0, thrust = 0, flight = 0, land = 0, recover = 0;
  bool lift_off = false, touchdown = false, finished = false;
  float knee_min = 9.0F, knee_max = -9.0F, max_rate = 0.0F;
};

// Drives the sequence with a synthetic IMU: |a| = 1.5 g in the thrust, ~0 in the air once the planned thrust is over
// (free_fall false: the robot never leaves the floor), a 2.2 g spike flight_s later, 1 g otherwise.
Run drive(HopSequence& h, float flight_s, bool free_fall = true, bool calm = true) {
  Run r;
  float thrust_t = 0.0F, air_t = 0.0F;
  bool airborne = false, landed = false;
  while (h.active()) {
    HopSequence::Input in;
    in.dt_s = kDt;
    in.calm = calm;
    in.accel_norm_m_s2 = kG;
    if (h.phase() == HopSequence::Phase::Thrust) {
      thrust_t += kDt;
      in.accel_norm_m_s2 = 1.5F * kG;
      if (free_fall && thrust_t >= h.plannedThrustS()) airborne = true;
    }
    if (airborne && !landed) {
      air_t += kDt;
      in.accel_norm_m_s2 = 0.05F * kG;
      if (air_t >= flight_s) { in.accel_norm_m_s2 = 2.2F * kG; landed = true; }
    }
    const auto o = h.update(in);
    r.lift_off = r.lift_off || o.lift_off_edge;
    r.touchdown = r.touchdown || o.touchdown_edge;
    r.finished = r.finished || o.finished_edge;
    switch (o.phase) {
      case HopSequence::Phase::Dip: ++r.dip; break;
      case HopSequence::Phase::Thrust: ++r.thrust; break;
      case HopSequence::Phase::Flight: ++r.flight; break;
      case HopSequence::Phase::Land: ++r.land; break;
      case HopSequence::Phase::Recover: ++r.recover; break;
      default: break;
    }
    if (o.phase != HopSequence::Phase::Idle) {
      r.knee_min = std::fmin(r.knee_min, o.knee_target_rad);
      r.knee_max = std::fmax(r.knee_max, o.knee_target_rad);
      r.max_rate = std::fmax(r.max_rate, std::fabs(o.knee_rate_rad_s));
    }
  }
  return r;
}
}  // namespace

int main() {
  // Disabled / busy.
  {
    HopSequence h;
    assert(!h.begin(0.9F, 0.06F));
    h.config() = enabledConfig();
    assert(!h.begin(0.9F, 0.001F));      // no usable CoM-height gain
    assert(h.begin(0.9F, 0.06F));
    assert(!h.begin(0.9F, 0.06F));       // busy
    assert(h.active());
  }
  // Plan numbers for a 20 mm hop: lift-off speed sqrt(2 g h), constant-acceleration thrust of the knee.
  {
    HopSequence h;
    h.config() = enabledConfig();
    assert(h.begin(0.9F, 0.06F));
    assert(std::fabs(h.plannedLiftSpeed() - std::sqrt(2.0F * kG * 0.020F)) < 1.0e-4F);
    assert(h.kneeBottom() <= h.config().dip_knee_rad + 1.0e-4F);  // dips to the dip knee
    assert(h.kneeTop() <= h.config().knee_max_rad + 1.0e-4F);  // never past the knee limit
    const float rate_end = h.plannedLiftSpeed() / 0.06F;
    const float stroke = h.kneeTop() - h.kneeBottom();
    assert(std::fabs(h.plannedThrustS() - 2.0F * stroke / rate_end) < 1.0e-4F);
    // plannedKneeRate: the dip first, then alpha * t in the thrust.
    float acc = 0.0F;
    const float r0 = h.plannedKneeRate(0.0F, &acc);
    assert(std::fabs(r0) < 1.0e-4F);
    const float r_thrust = h.plannedKneeRate(h.config().dip_s + 0.5F * h.plannedThrustS(), &acc);
    assert(r_thrust > 0.4F * rate_end && r_thrust < 0.6F * rate_end);
    assert(acc > 0.0F);
  }
  // Full sequence with a synthetic IMU: Dip -> Thrust -> Flight -> Land -> Recover -> Idle.
  {
    HopSequence h;
    h.config() = enabledConfig();
    assert(h.begin(0.9F, 0.06F));
    const Run r = drive(h, 0.13F);
    assert(r.dip > 0 && r.thrust > 0 && r.flight > 0 && r.land > 0 && r.recover > 0);
    assert(r.lift_off && r.touchdown && r.finished);
    assert(h.result() == HopSequence::Result::Done);
    assert(!h.active());
    assert(r.knee_min >= h.config().knee_min_rad - 1.0e-4F);
    assert(r.knee_max <= h.config().knee_max_rad + 1.0e-4F);
    assert(h.measuredFlightS() > 0.10F && h.measuredFlightS() < 0.20F);
    // The knee is back where it started once the recovery ends.
    HopSequence::Input in;
    in.dt_s = kDt;
    const auto o = h.update(in);
    assert(o.phase == HopSequence::Phase::Idle);
  }
  // No lift-off (the IMU never sees free fall): the hop gives up and recovers.
  {
    HopSequence h;
    h.config() = enabledConfig();
    assert(h.begin(0.9F, 0.06F));
    const Run r = drive(h, 0.13F, false);
    assert(!r.lift_off && r.finished);
    assert(h.result() == HopSequence::Result::NoLiftOff);
    assert(r.flight == 0);
  }
  // The recovery waits for a calm balance (up to recover_wait_max_s), then hands over anyway.
  {
    HopSequence h;
    h.config() = enabledConfig();
    h.config().recover_wait_max_s = 0.5F;
    assert(h.begin(0.9F, 0.06F));
    const Run r = drive(h, 0.13F, true, false);
    assert(r.finished);
    assert(r.recover * kDt >= h.config().recover_s + 0.45F);
  }
  // Abort from outside.
  {
    HopSequence h;
    h.config() = enabledConfig();
    assert(h.begin(0.9F, 0.06F));
    h.abort();
    assert(!h.active() && h.result() == HopSequence::Result::Aborted);
  }
  // Hop to hop: a flight shorter than planned raises the next thrust gain, a longer one lowers it.
  {
    HopSequence h;
    h.config() = enabledConfig();
    h.config().adapt = true;
    assert(h.begin(0.9F, 0.06F));
    (void)drive(h, 0.08F);
    assert(h.config().thrust_gain > 1.0F);
    HopSequence g;
    g.config() = enabledConfig();
    g.config().adapt = true;
    assert(g.begin(0.9F, 0.06F));
    (void)drive(g, 0.20F);
    assert(g.config().thrust_gain < 1.0F);
  }
  std::puts("hop_sequence_tests: all passed");
  return 0;
}
