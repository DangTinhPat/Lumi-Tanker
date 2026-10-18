#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

#include "tap_fw/v3/StandUp.hpp"
#include "tap_fw/v3/V3Controller.hpp"

using tap_fw::v3::StandUp;

namespace {
// Interface/transition fixture, not a robot dynamics model. The prescribed
// sensor history exercises ownership, bounds and stop priority independently
// of the CAD and motor response. The previous controller never transfers this
// captured rolling state because it requires |v| <= .15 m/s first.
struct Fixture {
  StandUp controller;
  StandUp::Input in{};
  StandUp::Output out{};
  float previous_hip = -.7F, previous_rate = 0.0F;

  explicit Fixture(bool transfer, float speed = -.4F) {
    auto cfg = StandUp::tunedRecovery();
    cfg.enabled = true;
    cfg.catch_rolling_transfer = transfer;
    cfg.fold_travel_rad = cfg.kneel_hold_s = 0.0F;
    // Isolate the transfer law from the wheel acceleration controller: the
    // rolling command starts at measured speed and the wheel tracks it.
    cfg.catch_wheel_prediction = cfg.catch_capture_velocity = cfg.catch_measured_braking = false;
    cfg.catch_accel_gain = cfg.catch_accel_damping = 0.0F;
    controller = StandUp(cfg);
    controller.begin(previous_hip, 0.0F);
    in.dt_s = .002F;
    in.hip_measured_rad = previous_hip;
    in.pitch_rad = .1F;
    in.capture_lean_rad = -.2F;
    in.ground_speed_m_s = speed;
    in.wheel_measured_rad_s = {speed / in.wheel_radius_m, speed / in.wheel_radius_m};
    in.body_clearance_valid = in.floor_bounds_valid = in.floor_bounds_feasible = true;
    in.body_clearance_m = in.floor_predicted_clearance_m = .025F;
    in.floor_hip_rate_lower_rad_s = -2.0F;
    in.floor_hip_rate_upper_rad_s = 2.0F;
    for (int i = 0; i < 400 && previous_hip < -.25F; ++i) {
      tick();
      assert(out.phase != StandUp::Phase::Failed);
      assert(!out.transfer_captured && out.transfer_weight == 0.0F);
    }
    assert(out.phase == StandUp::Phase::Catching && previous_hip >= -.25F);
  }

  void centered() {
    // Representative late-lift region in log 193036: COM nearly centered,
    // wheels still rolling, trunk moving back. Keep its rate below handoff
    // threshold so the fixture can isolate hip settling and speed policy.
    in.capture_lean_rad = -.025F;
    in.capture_rate_rad_s = .15F;
    in.pitch_rad = -.05F;
    in.pitch_rate_rad_s = -.15F;
  }

  void tick() {
    in.hip_measured_rad = previous_hip;
    in.hip_measured_rate_rad_s = previous_rate;
    out = controller.update(in);
    if (out.phase != StandUp::Phase::Failed) {
      assert(std::abs(out.hip_rate_rad_s) <= controller.config().catch_hip_rate_max_rad_s + 1e-4F);
      assert(std::abs(out.hip_rate_rad_s - previous_rate) <=
          controller.config().catch_hip_accel_rad_s2 * std::min(in.dt_s, .01F) + 2e-4F);
      assert(std::abs(out.wheel_rad_s) <= controller.config().catch_wheel_max_rad_s + 1e-5F);
      assert(out.knee_target_rad == 0.0F);
    }
    previous_hip = out.hip_target_rad;
    previous_rate = out.hip_rate_rad_s;
  }

  void run(int n) { for (int i = 0; i < n; ++i) tick(); }
};

void initialLiftIsUnchanged() {
  Fixture old(false), fixed(true);
  assert(std::abs(old.out.hip_target_rad - fixed.out.hip_target_rad) < 1e-6F);
  assert(std::abs(old.out.hip_rate_rad_s - fixed.out.hip_rate_rad_s) < 1e-6F);
  assert(old.out.wheel_rad_s == fixed.out.wheel_rad_s);
}

void capturedRollingStateTransfers() {
  // Below rolling_handoff_speed_m_s (0.30) but above the 0.15 standstill gate.
  for (float speed : {-.25F, .25F}) {
    Fixture old(false, speed), fixed(true, speed);
    old.centered(); fixed.centered();
    for (int i = 0; i < 300 && fixed.out.phase != StandUp::Phase::Support; ++i) {
      old.tick(); fixed.tick();
      assert(old.out.phase == StandUp::Phase::Catching);
    }
    assert(fixed.out.phase == StandUp::Phase::Support);
    assert(fixed.out.transfer_captured && fixed.out.transfer_weight > .999F);
    // The captured rolling path uses its own (shorter) dwell; others keep 200 ms.
    assert(fixed.out.handoff_settle_required_s == fixed.controller.config().rolling_handoff_settle_s);
    assert(fixed.out.settle_s >= fixed.out.handoff_settle_required_s);
    assert(fixed.out.body_level_rate_rad_s < 0.0F);
    assert(std::abs(fixed.out.hip_rate_rad_s) < fixed.controller.config().handoff_hip_rate_tol_rad_s);
    assert(std::abs(fixed.in.ground_speed_m_s) > fixed.controller.config().catch_speed_tol_m_s);
    // Support keeps signed posture authority, without reintroducing the old
    // ongoing hip lift on the following tick. Wheel ownership is now external.
    fixed.tick();
    assert(fixed.out.phase == StandUp::Phase::Support && fixed.out.body_level_rate_rad_s < 0.0F);
  }
}

void fastRollingStateDoesNotTransfer() {
  // Log 201659: handoff at -0.67 m/s left LQR no wheel headroom to brake.
  // Captured COM alone must not hand over while still rolling this fast.
  for (float speed : {-.4F, .4F}) {
    Fixture f(true, speed); f.centered();
    f.run(300);
    assert(f.out.transfer_captured && f.out.phase == StandUp::Phase::Catching);
    assert(!f.out.rolling_handoff);
  }
}

// Planar cart-pendulum (L 0.12 m, wheel 16 ms delay + 30 ms lag), hip not
// coupled to the lean: isolates the B4 wheel law in the regime of logs
// 200417-201659 (COM captured near the axle while rolling back). A lean-zero
// wheel loop has nothing that brakes; following the stopping lean does.
// Not a robot model: beyond ~20 ms delay, or with a 0.03 rad CoM offset,
// both laws fall in this plant. It checks the mechanism, not robustness.
struct WheelPlantResult { bool support = false, failed = false; float speed = 0, handoff_speed = 0; };
WheelPlantResult wheelPlant(bool wheel_speed_lean) {
  auto cfg = StandUp::tunedRecovery();
  cfg.enabled = true;
  cfg.fold_travel_rad = cfg.kneel_hold_s = 0.0F;
  cfg.catch_wheel_speed_lean = wheel_speed_lean;
  StandUp su(cfg);
  su.begin(-.744F, 0.0F);
  const float g = 9.80665F, l = .12F, r = .04F, dt = .002F, tau = .03F;
  float lean = -.02F, lean_rate = 0, w = -.30F / r, hip = -.744F, hip_rate = 0;
  float delayed[8];
  for (float& d : delayed) d = w;
  WheelPlantResult res;
  for (int k = 0; k < 1500; ++k) {
    StandUp::Input in{};
    in.dt_s = dt; in.pitch_rad = .02F;
    in.capture_lean_rad = lean; in.capture_rate_rad_s = lean_rate;
    in.hip_jacobian = .8F; in.hip_measured_rad = hip; in.hip_measured_rate_rad_s = hip_rate;
    in.wheel_measured_rad_s = {w, w}; in.ground_speed_m_s = w * r; in.wheel_radius_m = r;
    in.capture_omega_rad_s = std::sqrt(g / l);
    in.body_clearance_valid = in.floor_bounds_valid = in.floor_bounds_feasible = true;
    in.body_clearance_m = in.floor_predicted_clearance_m = .025F;
    in.floor_hip_rate_lower_rad_s = -2.0F; in.floor_hip_rate_upper_rad_s = 2.0F;
    const auto out = su.update(in);
    if (out.phase == StandUp::Phase::Catching && w * r < -.1F)
      assert(!wheel_speed_lean || out.wheel_lean_ref_rad > 0.0F);  // v<0: lean forward to brake
    if (out.phase == StandUp::Phase::Failed) { res.failed = true; break; }
    if (out.phase == StandUp::Phase::Support) { res.support = true; res.handoff_speed = w * r; break; }
    const float command = delayed[k % 8];
    delayed[k % 8] = out.wheel_rad_s;
    const float wheel_accel = (command - w) / tau;
    w += wheel_accel * dt;
    lean_rate += (g * std::sin(lean) - r * wheel_accel * std::cos(lean)) / l * dt;
    lean += lean_rate * dt;
    hip_rate = (out.hip_target_rad - hip) / dt;
    hip = out.hip_target_rad;
  }
  res.speed = w * r;
  return res;
}

void wheelLoopBrakesTowardStoppingLean() {
  const auto fixed = wheelPlant(true);
  assert(fixed.support && !fixed.failed);
  assert(std::abs(fixed.handoff_speed) <= StandUp::tunedRecovery().rolling_handoff_speed_m_s);
  const auto old = wheelPlant(false);
  assert(!old.support && (old.failed || old.speed < -.5F));
}

void aCrossingOrShortWindowDoesNotTransfer() {
  Fixture f(true); f.centered();
  f.run(10); // 20 ms < the 40 ms confirmation
  assert(!f.out.transfer_captured);
  f.in.dt_s = .02F; f.tick();
  assert(!f.out.transfer_captured && f.out.transfer_confirm_s == 0.0F);
  f.in.dt_s = .002F;
  f.in.capture_lean_rad = 0;
  f.in.capture_rate_rad_s = 2.0F;
  f.run(120);
  assert(f.out.phase == StandUp::Phase::Catching && !f.out.transfer_captured);
}

void existingGuardsStillBlockTransfer() {
  for (int gate = 0; gate < 6; ++gate) {
    Fixture f(true); f.centered();
    switch (gate) {
      case 0: f.in.floor_predicted_clearance_m = .007F; break;
      case 1: f.in.body_clearance_m = .010F; break;
      case 2: f.in.pitch_rate_rad_s = -.7F; break;
      case 3: f.in.wheel_measured_rad_s = {0, 0}; break;
      case 4: f.in.ground_speed_m_s = -.8F; break;
      case 5: f.in.pitch_rad = -.2F; break;
    }
    f.run(250);
    assert(f.out.phase == StandUp::Phase::Catching);
  }
}

void lossOfCaptureRestoresHipTaskAndFaultsWin() {
  Fixture f(true); f.centered(); f.run(100);
  assert(f.out.transfer_captured && f.out.transfer_weight > .99F);
  f.in.capture_lean_rad = -.2F; f.in.capture_rate_rad_s = 0;
  f.run(70);
  assert(!f.out.transfer_captured && f.out.transfer_weight < 1e-5F);
  assert(f.out.hip_feedback_rate_rad_s > 0.0F);
  f.in.floor_bounds_feasible = false;
  f.in.floor_predicted_clearance_m = 0;
  f.tick();
  assert(f.out.phase == StandUp::Phase::Failed && !f.out.wheels_on && f.out.wheel_rad_s == 0);

  Fixture stale(true); stale.centered(); stale.run(100);
  stale.in.sensors_ok = false; stale.tick();
  assert(stale.out.phase == StandUp::Phase::Failed && !stale.out.wheels_on);
  stale.in.sensors_ok = true; stale.tick();
  assert(stale.out.phase == StandUp::Phase::Failed);
  stale.controller.reset();
  stale.out = stale.controller.update(stale.in);
  assert(stale.out.phase == StandUp::Phase::Idle && !stale.out.transfer_captured && stale.out.transfer_weight == 0);

  Fixture invalid(true);
  invalid.controller.config().transfer_ramp_s = std::numeric_limits<float>::quiet_NaN();
  invalid.tick();
  assert(invalid.out.phase == StandUp::Phase::Failed && !invalid.out.wheels_on);
}

void wheelControllerReceivesRollingSeedAndRetainsRecoveryBounds() {
  using namespace tap_fw::v3;
  for (float speed : {-.4F, .4F}) {
    auto cfg = defaultV3Config();
    cfg.standup.enabled = cfg.standup.capture_first = cfg.standup.coordinated_speed = true;
    cfg.standup.fold_travel_rad = 0;
    cfg.standup.kneel_hold_s = cfg.standup.pitch_settle_s = cfg.standup.catch_settle_s = 0;
    // Force the handoff to isolate the destination contract. The capture
    // eligibility guards themselves are exercised in the fixtures above.
    cfg.standup.catch_lean_tol_rad = cfg.standup.catch_capture_tol_rad = 10;
    cfg.standup.catch_wheel_max_rad_s = 12;
    cfg.tier.preview = true;
    V3Controller c(cfg);
    V3Controller::Input in{};
    in.legs_ready = in.legs_measured = in.sensors_ok = true;
    in.accel_z_m_s2 = 9.80665F;
    const auto pose = LegHeight::forKnee(LegHeight::kKneeMinRad);
    in.hip_rad = {pose.hip_offset_rad, pose.hip_offset_rad};
    in.knee_rad = {pose.knee_offset_rad, pose.knee_offset_rad};
    in.leg_chain_rad = pose.hip_offset_rad + pose.knee_offset_rad;
    in.wheel_rad_s = {speed/cfg.wheel_radius_m, speed/cfg.wheel_radius_m};
    V3Controller::Command cmd{}; cmd.standup = true;
    V3Controller::Output out{};
    for (int i = 0; i < 10; ++i) {
      in.now_us += 2000;
      out = c.update(in, cmd); cmd.standup = false;
      if (out.state == V3Controller::State::Balancing) break;
    }
    assert(out.state == V3Controller::State::Balancing);
    assert(c.groundVelocity() * speed > 0);
    assert(std::abs(c.kneeCommand() - pose.knee_offset_rad) < 1e-6F);
    for (float w : out.wheel_cmd_rad_s) assert(std::isfinite(w) && std::abs(w) <= 12.0F + 1e-5F);
    cmd.disable = true; in.now_us += 2000;
    out = c.update(in, cmd);
    assert(out.state == V3Controller::State::Idle && !out.wheels_enabled);
  }
}
}  // namespace

int main() {
  initialLiftIsUnchanged();
  capturedRollingStateTransfers();
  fastRollingStateDoesNotTransfer();
  wheelLoopBrakesTowardStoppingLean();
  aCrossingOrShortWindowDoesNotTransfer();
  existingGuardsStillBlockTransfer();
  lossOfCaptureRestoresHipTaskAndFaultsWin();
  wheelControllerReceivesRollingSeedAndRetainsRecoveryBounds();
  std::puts("recovery transfer: rolling ownership, capture dwell, posture ramp, stopping lean and safety contracts passed");
}
