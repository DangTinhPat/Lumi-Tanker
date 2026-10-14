// StandUp (rocking stand-up from the lying pose) on synthetic signals.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <deque>
#include "LegPlant.hpp"

#include "tap_fw/v3/StandUp.hpp"
#include "tap_fw/v3/V3Controller.hpp"
#include "robot_control/WholeBodyModel.hpp"

using namespace tap_fw::v3;

namespace {

constexpr float kDt = 0.002F;

// The assertions below are about the mechanics, not the tuned defaults: pin what they rely on.
StandUp::Config config() {
  StandUp::Config c;
  c.enabled = true;
  c.hip_rate1_rad_s = 0.8F;
  c.hip_mid_rad = 0.6F;
  c.hip_rate2_rad_s = 0.5F;
  c.hip_goal_offset_rad = 0.0F;
  c.wheel_push_rad_s = 0.0F;
  c.wheel_lean_gain = 0.0F;
  c.knee_end_rad = 0.0F;
  c.catch_lean_rad = 0.12F;
  c.catch_rate_rad_s = 1.5F;
  c.catch_hip_tol_rad = 0.15F;
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

void theWheelTorqueIsOffWhileRockingAndOnAtTheHandOver() {
  StandUp su(config());
  su.begin(0.3F, 0.0F);
  auto out = su.update(input(0.5F));
  assert(out.phase == StandUp::Phase::Hold && !out.wheels_on);
  for (int i = 0; i < 300; ++i) out = su.update(input(0.5F));
  assert(out.phase == StandUp::Phase::Rise && !out.wheels_on);
  for (int i = 0; i < 30 && out.phase == StandUp::Phase::Rise; ++i) out = su.update(input(0.05F));
  assert(out.phase == StandUp::Phase::Caught && out.wheels_on);   // the balancer takes the wheels
  auto cfg = config();
  cfg.wheels_off = false;
  StandUp on(cfg);
  on.begin(1.3F, 0.0F);
  for (int i = 0; i < 400; ++i) out = on.update(input(1.0F));
  assert(out.wheels_on);
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

void balanceHandoverPreservesReferenceAndReleasesHipSlowly() {
  for (int profile : {0, 1, 2}) {
    const bool smooth = profile != 0;
    auto cfg = defaultV3Config();
    cfg.standup.enabled = true;
    cfg.standup.smooth_balance_handover = smooth;
    cfg.standup.settled_balance_handover = profile == 2;
    cfg.standup.hold_s = 0.0F;
    cfg.standup.grace_s = 0.05F;  // exercise timer expiry without a command jump.
    cfg.tier.suspension = cfg.tier.preview = false;
    cfg.tier.settle_s = 1.0F;
    cfg.tier.model_check_rad = 1.0F;
    cfg.lqr.balance_trim_max_rad = 0.20F;
    V3Controller ctrl(cfg);
    V3Controller::Input in{};
    in.dt_s = kDt;
    in.legs_measured = in.legs_ready = in.sensors_ok = true;
    in.hip_rad = {-0.25F, -0.25F};
    in.knee_rad = {cfg.standup.knee_end_rad, cfg.standup.knee_end_rad};
    const auto body = robot_control::WholeBodyModel{}.compute(
        LegHeight::hangHipCad() - 0.25F, in.knee_rad[0],
        LegHeight::hangHipCad() - 0.25F, in.knee_rad[1]);
    in.pitch_rad = -std::atan2(body.com_offset_x_m, body.com_above_axle_m) + 0.055F;
    in.accel_z_m_s2 = 9.80665F;
    V3Controller::Command cmd{};
    cmd.standup = true;
    cmd.height_m = LegHeight::minHeight();
    auto out = ctrl.update(in, cmd);
    cmd.standup = false;
    for (int i = 0; i < 500 && out.state == V3Controller::State::StandUp; ++i) {
      in.now_us += 2000U;
      out = ctrl.update(in, cmd);
    }
    assert(out.state == V3Controller::State::Balancing);
    const float seed = ctrl.virtualPitch();
    assert(seed > 0.20F && seed < 0.35F);
    if (smooth) assert(std::abs(ctrl.balanceRef() - seed) < 1e-5F);
    else assert(std::abs(ctrl.balanceRef() - 0.20F) < 1e-5F);
    float previous = out.leg_hip_offset_rad, peak_rate = 0.0F;
    for (int i = 0; i < 80; ++i) {
      in.now_us += 2000U;
      out = ctrl.update(in, cmd);
      assert(out.state == V3Controller::State::Balancing);
      peak_rate = std::max(peak_rate, std::abs(out.leg_hip_offset_rad - previous) / kDt);
      previous = out.leg_hip_offset_rad;
    }
    std::printf("standup handover smooth=%d hip peak rate=%.3f seed=%.3f ref=%.3f\n",
                smooth, peak_rate, seed, ctrl.balanceRef());
    if (profile == 2) assert(peak_rate < 1e-4F); // retain catch pose throughout settle
    if (smooth) assert(peak_rate <= cfg.standup.handover_hip_rate_rad_s + 0.01F);
    else assert(peak_rate > 2.0F * cfg.standup.handover_hip_rate_rad_s);
    if (smooth) {
      const float last_hip = out.leg_hip_offset_rad;
      in.sensors_ok = false;
      out = ctrl.update(in, cmd);
      assert(out.state == V3Controller::State::Fault && !out.wheels_enabled);
      assert(std::abs(out.leg_hip_offset_rad - last_hip) < 1e-5F);
      in.sensors_ok = true;
      ctrl.config().standup.smooth_balance_handover = false;
      cmd.standup = true;
      out = ctrl.update(in, cmd);
      assert(std::abs(ctrl.balancer().config().balance_trim_max_rad - 0.20F) < 1e-5F);
      cmd.standup = false;
      for (int i = 0; i < 500 && out.state == V3Controller::State::StandUp; ++i) {
        in.now_us += 2000U;
        out = ctrl.update(in, cmd);
      }
      assert(out.state == V3Controller::State::Balancing);
      assert(std::abs(ctrl.balanceRef() - 0.20F) < 1e-5F);
    }
    cmd.disable = true;
    out = ctrl.update(in, cmd);
    assert(out.state == V3Controller::State::Idle && !out.legs_locked);
    assert(std::abs(ctrl.balancer().config().balance_trim_max_rad - 0.20F) < 1e-5F);
  }
}

void extraFoldDoesNotCatchOnTheForwardEdge() {
  for (bool extra : {false, true}) {
    auto cfg = config();
    cfg.hold_s = 0.0F;
    cfg.catch_lean_rad = 0.0654F;
    cfg.extra_fold_rad = extra ? 0.034906585F : 0.0F;
    StandUp su(cfg);
    su.begin(0.0F, 0.0F);
    StandUp::Output out{};
    for (int i = 0; i < 100; ++i) out = su.update(input(0.05F));
    assert(out.phase == (extra ? StandUp::Phase::Rise : StandUp::Phase::Caught));
    for (int i = 0; i < 100; ++i) out = su.update(input(0.02F));
    assert(out.phase == StandUp::Phase::Caught);
    assert(out.wheels_on);
  }
}

void extraFoldShiftsOnlyTheFinalHipGoal() {
  float final_hip[2]{};
  for (int extra = 0; extra < 2; ++extra) {
    auto cfg = defaultV3Config();
    cfg.standup.enabled = true;
    cfg.standup.hold_s = 0.0F;
    cfg.standup.extra_fold_rad = extra * 0.034906585F;
    V3Controller ctrl(cfg);
    V3Controller::Input in{};
    in.dt_s = kDt;
    in.sensors_ok = in.legs_ready = in.legs_measured = true;
    in.hip_rad = {-0.25F, -0.25F};
    in.knee_rad = {0.0F, 0.0F};
    const auto model = robot_control::WholeBodyModel{}.compute(
        LegHeight::hangHipCad()-0.25F, 0.0F, LegHeight::hangHipCad()-0.25F, 0.0F);
    in.pitch_rad = -std::atan2(model.com_offset_x_m, model.com_above_axle_m) + 0.10F;
    V3Controller::Command cmd{};
    cmd.standup = true;
    auto out = ctrl.update(in, cmd);
    cmd.standup = false;
    for (int i = 0; i < 1500; ++i) {
      in.now_us += 2000U;
      out = ctrl.update(in, cmd);
      assert(out.state == V3Controller::State::StandUp && !out.wheels_enabled);
    }
    final_hip[extra] = out.leg_hip_offset_rad;
    assert(std::abs(out.leg_knee_offset_rad - cfg.standup.knee_end_rad) < 1e-5F);
  }
  assert(std::abs(final_hip[0]-final_hip[1]-0.034906585F) < 1e-5F);
}

void postCatchSupervisorPausesAndCompletesWithoutTimedJump() {
  PostStandUpBalance transfer;
  PostStandUpBalance::Input in{};
  in.dt_s = kDt;
  in.desired_hip_rad = 0.0F;
  in.body_rate_rad_s = 0.8F;
  transfer.begin(0.30F, 0.0F);
  for (int i = 0; i < 2500; ++i) {
    const auto out = transfer.update(in);
    assert(out.phase == PostStandUpBalance::Phase::CaptureHold);
    assert(out.limited && std::abs(out.hip_rad-0.30F) < 1e-6F);
  }
  in.body_rate_rad_s = 0.0F;
  float last = 0.30F;
  for (int i = 0; i < 150; ++i) {
    const auto out = transfer.update(in);
    assert(std::abs(out.hip_rad-last)/kDt <= 0.121F);
    last = out.hip_rad;
  }
  assert(transfer.phase() == PostStandUpBalance::Phase::PoseTransfer);
  in.virtual_error_rad = 0.10F;
  for (int i = 0; i < 150; ++i) {
    const auto out = transfer.update(in);
    assert(out.limited && std::abs(out.hip_rad-last) < 1e-6F);
  }
  in.virtual_error_rad = 0.0F;
  in.held = true;
  assert(std::abs(transfer.update(in).hip_rad-last) < 1e-6F);
  in.held = false;
  for (int i = 0; i < 2000 && transfer.phase() != PostStandUpBalance::Phase::Normal; ++i) {
    const auto out = transfer.update(in);
    assert(std::abs(out.hip_rad-last)/kDt <= 0.121F);
    last = out.hip_rad;
  }
  assert(transfer.phase() == PostStandUpBalance::Phase::Normal);
  assert(std::abs(last) < 1e-5F);
  transfer.reset();
  assert(transfer.phase() == PostStandUpBalance::Phase::Inactive);
}

// Diagnostic planar post-catch experiment, not a robot acceptance test.
// Reuses LegPlant and the older tier harness servo assumptions. Ideal body
// pitch/gyro, no shell contacts; wheel/hip lags are NOT calibrated for new motors.
void planarPostCatchDiagnostic() {
  using tap_fw::sim::LegPlant;
  namespace geo = robot_control::geometry;
  double baseline_peak = 0.0;
  for (int hip_delay : {2, 10, 25}) for (int profile : {1, 2}) {
    const bool smooth = true;
    const int extra_deg = 1;
    auto cfg = defaultV3Config();
    cfg.standup.enabled = true;
    cfg.standup.hold_s = 0.0F;
    cfg.standup.smooth_balance_handover = smooth;
    cfg.standup.settled_balance_handover = profile == 2;
    cfg.tier.suspension = cfg.tier.preview = false;
    cfg.standup.extra_fold_rad = extra_deg * 0.0174532925F;
    LegPlant plant(0.0, 1.0, 0.004, 0.004, LegHeight::hangHipCad());
    LegPlant::Vec s{0.0, 0.0, 0.73, cfg.standup.knee_end_rad}, sd{};
    for (int i = 0; i < 40; ++i) s[1] -= plant.leanAngle(s) - 0.02;
    const double initial_pitch = s[1];
    V3Controller ctrl(cfg);
    V3Controller::Input in{};
    in.dt_s = kDt;
    in.legs_measured = in.legs_ready = in.sensors_ok = true;
    in.pitch_rad = s[1];
    in.hip_rad = {float(s[2]), float(s[2])};
    in.knee_rad = {float(s[3]), float(s[3])};
    in.accel_z_m_s2 = 9.80665F;
    V3Controller::Command cmd{};
    cmd.standup = true;
    cmd.height_m = LegHeight::minHeight();
    auto out = ctrl.update(in, cmd);
    cmd.standup = false;
    for (int i = 0; i < 500 && out.state == V3Controller::State::StandUp; ++i) {
      in.now_us += 2000U;
      out = ctrl.update(in, cmd);
    }
    assert(out.state == V3Controller::State::Balancing);
    auto integral = plant.staticJointTorques(s);
    double wheel_rate = 0.0, wheel_pos = 0.0;
    double hip_applied = out.leg_hip_offset_rad, knee_applied = out.leg_knee_offset_rad;
    double wheel_applied = 0.0, peak_pitch = 0.0, peak_hip_rate = 0.0;
    double previous_hip = hip_applied;
    double rate_sq = 0.0, rate_sum = 0.0, pitch_after2 = 0.0, level_peak = 0.0; int rate_n = 0;
    double transfer_peak_rate = 0.0;
    std::deque<double> wq, hq, kq;
    int ticks = 0;
    for (; ticks < 6000; ++ticks) {
      in.now_us += 2000U;
      in.pitch_rad = s[1]; in.pitch_rate_rad_s = sd[1];
      in.hip_rad = {float(s[2]), float(s[2])};
      in.knee_rad = {float(s[3]), float(s[3])};
      in.hip_rate_rad_s = {float(sd[2]), float(sd[2])};
      in.knee_rate_rad_s = {float(sd[3]), float(sd[3])};
      in.wheel_rad_s = {float(wheel_rate), float(wheel_rate)};
      in.wheel_pos_rad = {float(wheel_pos), float(wheel_pos)};
      out = ctrl.update(in, cmd);
      if (out.state != V3Controller::State::Balancing) break;
      peak_hip_rate = std::max(peak_hip_rate,
          std::abs(out.leg_hip_offset_rad - previous_hip) / double(kDt));
      if (ctrl.postStandUpPhase() != PostStandUpBalance::Phase::Normal)
        transfer_peak_rate = std::max(transfer_peak_rate,
            std::abs(out.leg_hip_offset_rad - previous_hip) / double(kDt));
      previous_hip = out.leg_hip_offset_rad;
      level_peak = std::max(level_peak, std::abs(double(ctrl.levelHip())));
      if (ticks >= 1000) {
        rate_sq += sd[1]*sd[1]; rate_sum += sd[1]; ++rate_n;
        pitch_after2 = std::max(pitch_after2, std::abs(s[1]));
      }
      hq.push_back(out.leg_hip_offset_rad); kq.push_back(out.leg_knee_offset_rad);
      wq.push_back(out.wheel_cmd_rad_s[0]);
      if (hq.size() > size_t(hip_delay)) {
        hip_applied = hq.front(); hq.pop_front(); knee_applied = kq.front(); kq.pop_front();
      }
      if (wq.size() > 10U) { wheel_applied = wq.front(); wq.pop_front(); }
      constexpr double dt = 0.00025;
      const double kv = 48.0 * (geo::leg_hip_inertia_kgm2 + 0.004);
      for (int sub = 0; sub < 8; ++sub) {
        double tau[2];
        for (int j = 0; j < 2; ++j) {
          const double target = j == 0 ? hip_applied : knee_applied;
          const double error = 16.0 * (target - s[j+2]) - sd[j+2];
          integral[j] = std::clamp(integral[j] + 4.0 * kv * error * dt, -2.0, 2.0);
          tau[j] = std::clamp(2.0 * kv * error + integral[j], -8.0, 8.0);
        }
        const auto a0 = plant.accel(s, sd, 0.0, tau[0], tau[1], 0.0, 0.0);
        const auto a1 = plant.accel(s, sd, 1.0, tau[0], tau[1], 0.0, 0.0);
        const double u0 = a0[0]/geo::wheel_radius_m - a0[1] - a0[2] - a0[3];
        const double u1 = a1[0]/geo::wheel_radius_m - a1[1] - a1[2] - a1[3];
        const double tw = std::clamp(((wheel_applied-wheel_rate)/0.035-u0)/(u1-u0), -0.6, 0.6);
        LegPlant::Vec a{};
        for (int j = 0; j < 4; ++j) {
          a[j] = a0[j] + tw*(a1[j]-a0[j]);
          assert(std::isfinite(a[j]));
          sd[j] += a[j]*dt; s[j] += sd[j]*dt;
        }
        wheel_rate += (a[0]/geo::wheel_radius_m-a[1]-a[2]-a[3])*dt;
        wheel_pos += wheel_rate*dt;
      }
      peak_pitch = std::max(peak_pitch, std::abs(s[1]-initial_pitch));
    }
    if (profile == 1) baseline_peak = peak_pitch;
    if (profile == 2) {
      assert(!out.fault);
      assert(ctrl.postStandUpPhase() == PostStandUpBalance::Phase::Normal);
      assert(peak_pitch < baseline_peak);
      assert(transfer_peak_rate <= 0.13);
      assert(peak_hip_rate <= 0.36);
    }
    std::printf("planar MIN postcatch profile=%d extra_deg=%d hip_delay_ms=%d duration=%.3f pitch_change_peak=%.3f hip_cmd_rate=%.3f rate_rms_after2=%.3f abs_pitch_after2=%.3f level_peak=%.3f rate_std_after2=%.3f pitch_end=%.3f phase=%d fault=%s\n",
        profile, extra_deg, hip_delay*2, ticks*double(kDt), peak_pitch, peak_hip_rate,
        rate_n ? std::sqrt(rate_sq/rate_n) : 0.0, pitch_after2, level_peak,
        rate_n ? std::sqrt(std::max(rate_sq/rate_n - (rate_sum/rate_n)*(rate_sum/rate_n), 0.0)) : 0.0,
        s[1], int(ctrl.postStandUpPhase()),
        out.state == V3Controller::State::Fault ? ctrl.lastFault() : "none");
  }
}

}  // namespace

int main() {
  holdsThenFoldsTheHipsFastThenSlowlyNeverBelowTheGoal();
  theWheelsRollFreelyOrPushOnRequest();
  theWheelTorqueIsOffWhileRockingAndOnAtTheHandOver();
  theKneeExtendsOnlyWhenAsked();
  handsOverWhenTheCoMIsOverTheAxleAtTheGoal();
  aFastLeanRateIsNotACatch();
  failsWhenItFallsBackOrTimesOut();
  balanceHandoverPreservesReferenceAndReleasesHipSlowly();
  extraFoldDoesNotCatchOnTheForwardEdge();
  extraFoldShiftsOnlyTheFinalHipGoal();
  postCatchSupervisorPausesAndCompletesWithoutTimedJump();
  planarPostCatchDiagnostic();
  std::puts("v3 standup tests passed");
  return 0;
}
