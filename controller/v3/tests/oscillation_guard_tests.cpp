// OscillationGuard (growing balance oscillation detector) and the LQR actuation predictor's tolerance to the
// microsecond jitter of the measured tick period.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>

#include "robot_control/LqrActuationPredictor.hpp"
#include "tap_fw/v3/OscillationGuard.hpp"

using tap_fw::v3::OscillationGuard;
using robot_control::LqrActuationPredictor;

namespace {
constexpr float kPi = 3.14159265358979F;

// Runs a signal through the guard; returns the largest risk seen and the time it first exceeded 0.6 (-1 if never).
struct Result { float max_risk = 0.0F; float t06 = -1.0F; };
template <typename Signal>
Result run(Signal signal, float duration_s, float dt_s) {
  OscillationGuard guard;
  OscillationGuard::Config cfg;
  Result r;
  for (float t = 0.0F; t < duration_s; t += dt_s) {
    const auto out = guard.update(signal(t), dt_s, cfg);
    r.max_risk = std::max(r.max_risk, out.risk);
    if (r.t06 < 0.0F && out.risk > 0.6F) r.t06 = t;
  }
  return r;
}
}  // namespace

int main() {
  for (const float dt : {0.002F, 0.042F}) {
    // Quiet balance: noise of 0.002 rad on a 0.03 rad trim, for a minute.
    {
      std::mt19937 rng(1);
      std::normal_distribution<float> noise(0.0F, 0.002F);
      const auto r = run([&](float) { return 0.03F + noise(rng); }, 60.0F, dt);
      std::printf("dt %.3f quiet: max risk %.2f\n", dt, r.max_risk);
      assert(r.max_risk < 0.15F);
    }
    // A steady small limit cycle of 0.01 rad at 1.5 Hz does not grow: no alarm.
    {
      const auto r = run([](float t) { return 0.01F * std::sin(2.0F * kPi * 1.5F * t); }, 30.0F, dt);
      std::printf("dt %.3f steady 0.01 rad: max risk %.2f\n", dt, r.max_risk);
      assert(r.max_risk < 0.25F);
    }
    // One decaying disturbance (a push): no alarm.
    {
      const auto r = run([](float t) { return 0.06F * std::exp(-t / 0.6F) * std::sin(2.0F * kPi * 1.8F * t); },
                         20.0F, dt);
      std::printf("dt %.3f decaying push: max risk %.2f\n", dt, r.max_risk);
      assert(r.max_risk < 0.35F);
    }
    // Calm for 10 s, then swings that double every half second from 0.01 rad (what the robot did at 70 mm).
    {
      const auto r = run([](float t) {
        if (t < 10.0F) return 0.002F * std::sin(2.0F * kPi * 1.5F * t);
        return 0.01F * std::exp2((t - 10.0F) / 0.5F) * std::sin(2.0F * kPi * 2.0F * t);
      }, 14.0F, dt);
      std::printf("dt %.3f growing: first risk>0.6 at %.2f s (growth starts at 10 s), max %.2f\n", dt, r.t06, r.max_risk);
      assert(r.t06 > 10.0F && r.t06 < 12.5F);
      assert(r.max_risk > 0.9F);
    }
  }

  // Predictor: the measured tick period jitters by microseconds; the command history must survive it.
  {
    LqrActuationPredictor::Config cfg;
    cfg.enabled = true;
    cfg.delay_s = 0.05F;
    cfg.lag_s = 0.075F;
    LqrActuationPredictor predictor;
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> jitter(-6.0e-6F, 6.0e-6F);
    LqrActuationPredictor::State at_rest{0.0F, 0.0F, 0.0F, 0.0F};
    LqrActuationPredictor::State future{};
    for (int i = 0; i < 80; ++i) {
      const float dt = 0.002F + jitter(rng);
      future = predictor.predict(at_rest, cfg, dt, 0.1F, 0.0F, 8.0F);
      predictor.record(0.5F);  // 0.5 m/s commanded every tick
    }
    std::printf("predictor with jittered dt: predicted ground speed %.3f m/s (0 = history was cleared)\n", future[2]);
    assert(future[2] > 0.2F);
  }
  std::printf("oscillation_guard_tests: all checks passed\n");
  return 0;
}
