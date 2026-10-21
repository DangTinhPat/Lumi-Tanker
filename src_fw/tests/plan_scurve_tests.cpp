// Speed-plan acceleration shape (V3Controller::updatePlan). The hip follows the plan acceleration, so the plan has
// to accelerate evenly and decelerate evenly: one ramp up, one plateau, one ramp down, no tail and no second hump.
// updatePlan is private; the test opens it on purpose.
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>
#define private public
#include "tap_fw/MotionApi.hpp"
#include "tap_fw/v3/V3Controller.hpp"
#undef private

using tap_fw::v3::V3Controller;

namespace {

struct Run {
  std::vector<float> v, a;
  float overshoot = 0.0F, peak_a = 0.0F, t_plateau = 0.0F, t_total = 0.0F;
  int humps = 0;  // rises of |a| after it has started to fall
};

Run run(V3Controller& c, float from, float target, float dt, float seconds) {
  Run r;
  c.plan_v_ = from; c.plan_a_ = 0.0F; c.plan_j_ = 0.0F; c.plan_init_ = true; c.plan_scale_ = 1.0F; c.plan_n_ = 0U;
  const float sign = target >= from ? 1.0F : -1.0F;
  float prev_a = 0.0F, max_a = 0.0F;
  bool falling = false;
  const int n = static_cast<int>(seconds / dt);
  for (int i = 0; i < n; ++i) {
    c.updatePlan(target, dt, false);
    r.v.push_back(c.plan_v_); r.a.push_back(c.plan_a_);
    const float m = std::abs(c.plan_a_);
    max_a = std::max(max_a, m);
    if (m < prev_a - 1.0e-6F) falling = true;
    else if (falling && m > prev_a + 1.0e-4F) ++r.humps;
    prev_a = m;
    r.overshoot = std::max(r.overshoot, sign * (c.plan_v_ - target));
    if (m > 0.95F * max_a && m > 0.05F) r.t_plateau += dt;
    if (m > 1.0e-3F) r.t_total = (i + 1) * dt;
  }
  r.peak_a = max_a;
  return r;
}

}  // namespace

int main() {
  auto cfg = tap_fw::v3::defaultV3Config();
  const float dt = 0.004F;
  struct Case { float from, to; };
  const Case cases[] = {{0.0F, 0.3F}, {0.0F, 0.7F}, {0.0F, 1.5F}, {0.0F, 1.6F}, {0.7F, 0.0F}, {1.5F, 0.0F}, {1.0F, 1.5F},
                        {0.0F, -0.7F}};
  for (const Case& k : cases) {
    V3Controller c(cfg);
    const Run r = run(c, k.from, k.to, dt, 14.0F);
    std::printf("plan %+.2f -> %+.2f: %.2f s, peak a %.2f, plateau %.2f s, overshoot %.4f, humps %d, end v %+.4f a %+.4f\n",
                k.from, k.to, r.t_total, r.peak_a, r.t_plateau, r.overshoot, r.humps, r.v.back(), r.a.back());
    assert(r.overshoot < 0.01F);
    assert(r.humps == 0);
    assert(std::abs(r.v.back() - k.to) < 1.0e-3F);
    assert(std::abs(r.a.back()) < 1.0e-3F);
  }
  // Even acceleration: a 1.5 m/s step holds a flat acceleration for most of its span.
  {
    V3Controller c(cfg);
    const Run r = run(c, 0.0F, 1.5F, dt, 14.0F);
    assert(r.t_plateau > 0.45F * r.t_total);
  }
  // The target drops while accelerating: brake at once, no overshoot beyond the new target.
  {
    V3Controller c(cfg);
    (void)run(c, 0.0F, 1.5F, dt, 1.0F);
    float worst = 0.0F;
    for (int i = 0; i < 3000; ++i) {
      c.updatePlan(0.2F, dt, false);
      worst = std::max(worst, c.plan_v_ - 0.2F);
    }
    std::printf("target dropped mid-accel: overshoot %.3f m/s, end v %.4f\n", worst, c.plan_v_);
    assert(std::abs(c.plan_v_ - 0.2F) < 1.0e-3F);
  }
  // The turn command: same shape, reaches the commanded rate, no hump, no overshoot, reverses cleanly.
  {
    V3Controller c(cfg);
    V3Controller::Input in{};
    in.dt_s = dt;
    V3Controller::Command cmd{};
    const float targets[] = {6.0F, 0.0F, -4.0F, 0.0F};
    for (const float target : targets) {
      cmd.yaw_rad_s = target;
      const float start = c.yawCommand();
      float prev_a = 0.0F, max_a = 0.0F, peak_over = 0.0F;
      bool falling = false;
      int humps = 0;
      float prev = start;
      for (int i = 0; i < 1500; ++i) {
        in.now_us += 4000U;
        (void)c.update(in, cmd);
        const float a = std::abs(c.yawCommand() - prev) / dt;
        prev = c.yawCommand();
        max_a = std::max(max_a, a);
        if (a < prev_a - 1.0e-3F) falling = true;
        else if (falling && a > prev_a + 0.05F && std::abs(target - c.yawCommand()) > 0.01F) ++humps;
        prev_a = a;
        peak_over = std::max(peak_over, (target - start >= 0.0F ? 1.0F : -1.0F) * (c.yawCommand() - target));
      }
      std::printf("yaw %+.1f -> %+.1f: peak accel %.1f rad/s2, overshoot %.4f, humps %d, end %+.4f\n", start, target,
                  max_a, peak_over, humps, c.yawCommand());
      assert(std::abs(c.yawCommand() - target) < 1.0e-3F);
      assert(peak_over < 0.05F);
      assert(humps == 0);
      assert(max_a <= cfg.tier.yaw_accel_rad_s2 * 1.05F);
    }
  }
  // MOVE / TURN closed on the real plan and its stopping distance, on a plant that answers with a first-order lag.
  {
    struct MoveCase { float dist, speed; };
    const MoveCase moves[] = {{0.5F, 0.2F}, {1.0F, 0.2F}, {1.0F, 0.3F}, {1.0F, 0.5F}, {1.0F, 1.0F}, {3.0F, 1.0F},
                              {-1.0F, 0.3F}, {0.1F, 0.3F}};
    for (const MoveCase& m : moves) {
      V3Controller c(cfg);
      tap_fw::MotionApi api;
      V3Controller::Input in{};
      in.dt_s = dt;
      V3Controller::Command cmd{};
      float x = 0.0F, v = 0.0F;
      std::uint64_t t_us = 0U;
      auto tick = [&](bool balancing) {
        t_us += 4000U;
        in.now_us = t_us;
        tap_fw::MotionApi::Feedback fb;
        fb.now_us = t_us; fb.dt_s = dt; fb.balancing = balancing; fb.ground_x_m = x; fb.ground_v_m_s = v;
        if (api.active()) { fb.stop_known = true; fb.stop_m = c.planStopDistance(0.12F); fb.stop_rad = 0.0F; }
        const auto out = api.update(fb);
        c.updatePlan(out.active ? out.forward_m_s : 0.0F, dt, false);
        v += (c.plan_v_ - v) * dt / 0.12F;
        x += v * dt;
      };
      tick(true);
      assert(api.move(m.dist, m.speed));
      int guard = 0;
      while (api.active() && ++guard < 20000) tick(true);
      for (int i = 0; i < 1000; ++i) tick(true);  // let it settle
      std::printf("MOVE %+.2f m @ %.1f m/s: %s, ended at %+.3f m (error %+.3f), plan v %.3f, %.1f s\n", m.dist, m.speed,
                  tap_fw::MotionApi::statusName(api.status()), x, x - m.dist, c.plan_v_, guard * dt);
      assert(api.status() == tap_fw::MotionApi::Status::Done);
      assert(std::abs(x - m.dist) < 0.03F);
    }
    struct TurnCase { float angle, rate; };
    const TurnCase turns[] = {{1.5708F, 0.5F}, {6.2832F, 0.52F}, {6.2832F, 2.0F}, {6.2832F, 6.0F}, {-3.1416F, 3.0F}};
    for (const TurnCase& tc : turns) {
      V3Controller c(cfg);
      tap_fw::MotionApi api;
      V3Controller::Input in{};
      in.dt_s = dt;
      V3Controller::Command cmd{};
      float yaw = 0.0F, w = 0.0F;
      std::uint64_t t_us = 0U;
      auto tick = [&]() {
        t_us += 4000U;
        in.now_us = t_us;
        tap_fw::MotionApi::Feedback fb;
        fb.now_us = t_us; fb.dt_s = dt; fb.balancing = true; fb.yaw_rate_rad_s = w;
        if (api.active()) { fb.stop_known = true; fb.stop_m = 0.0F; fb.stop_rad = c.yawStopAngle(0.07F); }
        const auto out = api.update(fb);
        cmd.yaw_rad_s = out.active ? out.yaw_rad_s : 0.0F;
        (void)c.update(in, cmd);
        w += (c.yawCommand() - w) * dt / 0.07F;
        yaw += w * dt;
      };
      tick();
      assert(api.rotate(tc.angle, tc.rate));
      int guard = 0;
      while (api.active() && ++guard < 20000) tick();
      for (int i = 0; i < 500; ++i) tick();
      std::printf("TURN %+.2f rad @ %.2f rad/s: %s, ended at %+.3f (error %+.3f rad = %+.1f deg), %.1f s\n", tc.angle,
                  tc.rate, tap_fw::MotionApi::statusName(api.status()), yaw, yaw - tc.angle,
                  (yaw - tc.angle) * 57.2958F, guard * dt);
      assert(api.status() == tap_fw::MotionApi::Status::Done);
      assert(std::abs(yaw - tc.angle) < 0.06F);
    }
  }
  // Yaw gyro offset: learned while quiet, not learned while a turn is commanded or the gyro really turns fast.
  {
    V3Controller c(cfg);
    V3Controller::Input in{};
    in.dt_s = dt;
    in.yaw_rate_rad_s = 0.042F;
    V3Controller::Command cmd{};
    for (int i = 0; i < 4000; ++i) { in.now_us += 4000U; (void)c.update(in, cmd); }
    std::printf("yaw bias learned %.4f rad/s from a 0.042 offset\n", c.yawBias());
    assert(std::abs(c.yawBias() - 0.042F) < 0.002F);
    V3Controller d(cfg);
    in.now_us = 0U;
    cmd.yaw_rad_s = 2.0F;
    in.yaw_rate_rad_s = 2.0F;
    for (int i = 0; i < 2000; ++i) { in.now_us += 4000U; (void)d.update(in, cmd); }
    std::printf("yaw bias while turning %.4f rad/s (must stay ~0)\n", d.yawBias());
    assert(std::abs(d.yawBias()) < 0.01F);
  }
  std::printf("plan S-curve tests passed\n");
  return 0;
}
