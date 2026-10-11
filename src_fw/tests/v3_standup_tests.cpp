// StandUp (rocking stand-up from the lying pose) on synthetic signals.
#include <cassert>
#include <cmath>
#include <cstdio>

#include "tap_fw/v3/StandUp.hpp"

using namespace tap_fw::v3;

namespace {

constexpr float kDt = 0.002F;

StandUp::Config config() {
  StandUp::Config c;
  c.enabled = true;
  return c;
}

StandUp::Input input(float lean, float goal = 0.2F) {
  StandUp::Input in;
  in.dt_s = kDt;
  in.lean_rad = lean;
  in.hip_goal_rad = goal;
  in.wheel_rate_rad_s = 3.0F;
  return in;
}

void holdsThenFoldsTheHipsFastThenSlowlyNeverBelowTheGoal() {
  StandUp su(config());
  su.begin(1.3F, 0.0F);
  auto out = su.update(input(1.0F));
  assert(out.phase == StandUp::Phase::Hold && out.hip_target_rad == 1.3F);
  for (int i = 0; i < 160; ++i) out = su.update(input(1.0F));       // the 0.3 s hold ends
  assert(out.phase == StandUp::Phase::Hold || out.phase == StandUp::Phase::Rise);
  float prev = out.hip_target_rad;
  float rate_fast = 0.0F, rate_slow = 0.0F;
  for (int i = 0; i < 3000 && out.phase == StandUp::Phase::Rise; ++i) {
    out = su.update(input(1.0F));
    const float rate = (prev - out.hip_target_rad) / kDt;
    if (prev > 0.65F && rate > 0.0F) rate_fast = rate;
    if (prev < 0.55F && prev > 0.25F && rate > 0.0F) rate_slow = rate;
    prev = out.hip_target_rad;
    assert(out.hip_target_rad >= 0.2F - 1e-5F);
  }
  assert(std::abs(rate_fast - 0.8F) < 0.02F && std::abs(rate_slow - 0.5F) < 0.02F);
  assert(std::abs(out.hip_target_rad - 0.2F) < 1e-5F);
}

void theWheelsRollFreelyOrPushOnRequest() {
  StandUp su(config());
  su.begin(1.3F, 0.0F);
  for (int i = 0; i < 300; ++i) (void)su.update(input(1.0F));
  auto out = su.update(input(1.0F));
  assert(out.phase == StandUp::Phase::Rise);
  assert(std::abs(out.wheel_rad_s - 3.0F) < 1e-4F);                // free: their own measured speed (no torque)
  auto cfg = config();
  cfg.wheel_push_rad_s = 10.0F;
  cfg.wheel_ramp_s = 0.1F;
  StandUp pushed(cfg);
  pushed.begin(1.3F, 0.0F);
  for (int i = 0; i < 400; ++i) out = pushed.update(input(1.0F));
  assert(std::abs(out.wheel_rad_s - 13.0F) < 1e-3F);               // measured + push
}

void theKneeExtendsOnlyWhenAsked() {
  auto cfg = config();
  cfg.knee_end_rad = 0.5F;
  cfg.knee_rate_rad_s = 1.0F;
  StandUp su(cfg);
  su.begin(1.3F, 0.0F);
  StandUp::Output out;
  for (int i = 0; i < 1000; ++i) out = su.update(input(1.0F));
  assert(std::abs(out.knee_target_rad - 0.5F) < 1e-4F);
  StandUp flat(config());
  flat.begin(1.3F, 0.0F);
  for (int i = 0; i < 1000; ++i) out = flat.update(input(1.0F));
  assert(out.knee_target_rad == 0.0F);
}

void handsOverWhenTheCoMIsOverTheAxleAtTheGoal() {
  StandUp su(config());
  su.begin(0.3F, 0.0F);                                           // already near the goal hip
  StandUp::Output out;
  for (int i = 0; i < 400; ++i) out = su.update(input(0.5F));      // lean too large: no hand-over
  assert(out.phase == StandUp::Phase::Rise);
  for (int i = 0; i < 30 && out.phase == StandUp::Phase::Rise; ++i) out = su.update(input(0.05F));
  assert(out.phase == StandUp::Phase::Caught);
  out = su.update(input(0.05F));
  assert(out.phase == StandUp::Phase::Caught);                    // stays caught (the controller engages the balancer at once)
}

void aFastLeanRateIsNotACatch() {
  StandUp su(config());
  su.begin(0.3F, 0.0F);
  StandUp::Output out;
  auto in = input(0.0F);
  in.lean_rate_rad_s = 4.0F;                                       // swinging through the axle
  for (int i = 0; i < 600; ++i) out = su.update(in);
  assert(out.phase == StandUp::Phase::Rise);
}

void failsWhenItFallsBackOrTimesOut() {
  StandUp su(config());
  su.begin(1.3F, 0.0F);
  StandUp::Output out;
  for (int i = 0; i < 300; ++i) out = su.update(input(1.0F));
  out = su.update(input(-0.7F));                                   // behind the axle
  assert(out.phase == StandUp::Phase::Failed);
  auto cfg = config();
  cfg.timeout_s = 1.0F;
  StandUp slow(cfg);
  slow.begin(1.3F, 0.0F);
  for (int i = 0; i < 700; ++i) out = slow.update(input(1.0F));
  assert(out.phase == StandUp::Phase::Failed);
  StandUp lost(config());
  lost.begin(1.3F, 0.0F);
  auto bad = input(1.0F);
  bad.sensors_ok = false;
  out = lost.update(bad);
  assert(out.phase == StandUp::Phase::Failed);
}

}  // namespace

int main() {
  holdsThenFoldsTheHipsFastThenSlowlyNeverBelowTheGoal();
  theWheelsRollFreelyOrPushOnRequest();
  theKneeExtendsOnlyWhenAsked();
  handsOverWhenTheCoMIsOverTheAxleAtTheGoal();
  aFastLeanRateIsNotACatch();
  failsWhenItFallsBackOrTimesOut();
  std::puts("v3 standup tests passed");
  return 0;
}
