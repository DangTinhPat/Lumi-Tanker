#include <cassert>
#include <cmath>
#include <cstdio>

#include "tap_fw/MotionProfile.hpp"

int motionApiTests();
int main() {
  using tap_fw::approachSpeed;
  // Far from the target: cruise, signed toward it.
  assert(approachSpeed(2.0F, 0.3F, 0.3F) == 0.3F);
  assert(approachSpeed(-2.0F, 0.3F, 0.3F) == -0.3F);
  // Near the target: the speed that stops exactly there, v = sqrt(2 a d).
  assert(std::fabs(approachSpeed(0.1F, 0.3F, 0.3F) - std::sqrt(2.0F * 0.3F * 0.1F)) < 1.0e-6F);
  // On the target: zero; overshoot reverses.
  assert(approachSpeed(0.0F, 0.3F, 0.3F) == 0.0F);
  assert(approachSpeed(-0.01F, 0.3F, 0.3F) < 0.0F);
  // Following the profile with the deceleration it assumes stops within one step of the target.
  float x = 0.0F, v = 0.0F;
  const float target = 1.0F, dt = 0.002F;
  for (int i = 0; i < 20000; ++i) {
    const float want = approachSpeed(target - x, 0.3F, 0.3F);
    const float dv = std::fmax(-0.3F * dt, std::fmin(0.3F * dt, want - v));
    v += dv;
    x += v * dt;
  }
  assert(std::fabs(x - target) < 0.01F && std::fabs(v) < 0.02F);
  motionApiTests();
  std::puts("motion_profile_tests passed");
  return 0;
}

// ---- MotionApi on a first-order plant: the robot follows the speed request with a 0.1 s lag. ----
#include "tap_fw/MotionApi.hpp"

namespace {
struct Plant {
  float x = 0.0F, v = 0.0F, yaw = 0.0F, w = 0.0F;
  bool balancing = true;
  float yaw_sign = 1.0F;
  std::uint64_t t_us = 0U;
  tap_fw::MotionApi::Output out{};
  void step(tap_fw::MotionApi& api, const float dt = 0.002F) {
    tap_fw::MotionApi::Feedback fb;
    t_us += static_cast<std::uint64_t>(dt * 1.0e6F);
    fb.now_us = t_us;
    fb.dt_s = dt;
    fb.balancing = balancing;
    fb.ground_x_m = x;
    fb.ground_v_m_s = v;
    fb.yaw_rate_rad_s = w;
    out = api.update(fb);
    const float fwd = out.active ? out.forward_m_s : 0.0F;
    const float yawcmd = out.active ? out.yaw_rad_s : 0.0F;
    v += (fwd - v) * dt / 0.1F;
    w += (yaw_sign * yawcmd - w) * dt / 0.1F;
    x += v * dt;
    yaw += w * dt;
  }
};
using tap_fw::MotionApi;
}  // namespace

int motionApiTests() {
  // forward / backward reach the distance and finish with the robot nearly still
  for (float d : {0.5F, -0.4F, 1.0F}) {
    MotionApi api;
    Plant p;
    p.step(api);
    assert(api.move(d));
    MotionApi::Event e;
    assert(api.takeEvent(e) && e.status == MotionApi::Status::Running);
    int guard = 0;
    while (api.active() && ++guard < 30000) p.step(api);
    assert(api.status() == MotionApi::Status::Done);
    assert(std::fabs(p.x - d) < 0.04F);
    assert(std::fabs(p.v) < 0.1F);
  }
  // turn
  {
    MotionApi api;
    Plant p;
    p.step(api);
    assert(api.rotate(1.57F, 0.5F));
    int guard = 0;
    while (api.active() && ++guard < 30000) p.step(api);
    assert(api.status() == MotionApi::Status::Done && std::fabs(p.yaw - 1.57F) < 0.12F);
  }
  // a turn whose sensor sign is wrong is aborted (never spins away)
  {
    MotionApi api;
    Plant p;
    p.yaw_sign = -1.0F;
    p.step(api);
    assert(api.rotate(1.57F, 0.5F));
    int guard = 0;
    while (api.active() && ++guard < 30000) p.step(api);
    assert(api.status() == MotionApi::Status::Aborted && std::fabs(p.yaw) < 1.0F);
  }
  // not balancing: refused; leaving balancing mid-move: aborted; stop: cancelled
  {
    MotionApi api;
    Plant p;
    p.balancing = false;
    p.step(api);
    assert(!api.forward(0.5F));
    MotionApi::Event e;
    assert(api.takeEvent(e) && e.status == MotionApi::Status::Refused);
    p.balancing = true;
    p.step(api);
    assert(api.forward(0.5F) && api.active());
    p.balancing = false;
    p.step(api);
    assert(!api.active() && api.status() == MotionApi::Status::Aborted);
    p.balancing = true;
    p.step(api);
    assert(api.forward(0.5F));
    api.stop();
    assert(api.status() == MotionApi::Status::Cancelled && !api.active());
  }
  // out of range and not finite
  {
    MotionApi api;
    Plant p;
    p.step(api);
    assert(!api.forward(0.0F) && !api.forward(2000.0F) && !api.rotate(0.001F));
    assert(!api.forward(NAN));
  }
  // timeout when the robot does not move
  {
    MotionApi api;
    Plant p;
    p.step(api);
    assert(api.forward(0.5F));
    int guard = 0;
    while (api.active() && ++guard < 100000) {
      p.v = 0.0F;
      p.step(api);
    }
    assert(api.status() == MotionApi::Status::Timeout);
  }
  return 0;
}
