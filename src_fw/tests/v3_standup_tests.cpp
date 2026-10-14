// StandUp (rocking stand-up from the lying pose) on synthetic signals.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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
    while (!ctrl.balancer().buildGainTableStep(32)) {}
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

void transferRampsVelocityAndBrakesOnModerateNoise() {
  PostStandUpBalance t;
  PostStandUpBalance::Input in{};
  in.dt_s = kDt; in.settle_s=0.0F; in.grace_s=0.0F;
  in.smooth_transfer=true;
  t.begin(0.6F,0.2F);
  float last=0.6F, previous_rate=0.0F;
  for(int i=0;i<400;++i) {
    auto o=t.update(in);
    const float rate=(o.hip_rad-last)/kDt;
    assert(std::abs(rate)<=0.121F);
    assert(std::abs(rate-previous_rate)/kDt<=0.62F);
    last=o.hip_rad;previous_rate=rate;
  }
  const float at_pause=last;
  in.body_rate_rad_s=0.36F; // moderate loss of quiet: brake, do not toggle speed
  for(int i=0;i<150;++i) {
    auto o=t.update(in);const float rate=(o.hip_rad-last)/kDt;
    assert(std::abs(rate-previous_rate)/kDt<=0.62F);
    last=o.hip_rad;previous_rate=rate;
  }
  assert(at_pause-last<=0.013F && at_pause-last>0.0F);
  in.held=true;
  assert(std::abs(t.update(in).hip_rad-last)<1e-6F);
  in.held=false;in.body_rate_rad_s=0.0F;
  for(int i=0;i<3000 && t.phase()!=PostStandUpBalance::Phase::Normal;++i) {
    auto o=t.update(in); assert(o.hip_rad>=0.2F-1e-6F);
  }
  assert(t.phase()==PostStandUpBalance::Phase::Normal);
  t.begin(0.6F,0.2F); in.body_rate_rad_s=1.0F;
  PostStandUpBalance::Output stalled{};
  for(int i=0;i<6100;++i) stalled=t.update(in);
  assert(stalled.failed && stalled.phase==PostStandUpBalance::Phase::CaptureHold);
  assert(std::abs(stalled.hip_rad-0.6F)<1e-6F);
  t.reset();t.begin(0.6F,0.2F);in.smooth_transfer=false;
  for(int i=0;i<6100;++i) stalled=t.update(in);
  assert(!stalled.failed); // existing opt-out path unchanged
}

// Reproduce the learned-trim leak across retry with a changed catch angle.
// The flag-off controller is the control case: it reuses the old trim.
void retriesSeedThisCatchAndFreezeOldLevelState() {
  for (bool recover : {false, true}) {
    auto cfg = defaultV3Config();
    cfg.standup.enabled = true;
    cfg.standup.hold_s = 0.0F;
    cfg.standup.smooth_balance_handover = true;
    cfg.standup.settled_balance_handover = true;
    cfg.standup.repeatable_balance_capture = recover;
    cfg.standup.hold_transferred_pose = recover;
    cfg.learn_hold_s = 0.0F;
    cfg.tier.suspension = cfg.tier.preview = false;
    cfg.tier.model_check_rad = 10.0F;
    V3Controller ctrl(cfg);
    V3Controller::Input in{};
    in.dt_s = kDt;
    in.legs_measured = in.legs_ready = in.sensors_ok = true;
    in.hip_rad = {0.63F, 0.63F};
    in.knee_rad = {cfg.standup.knee_end_rad, cfg.standup.knee_end_rad};
    const auto b = robot_control::WholeBodyModel{}.compute(
        LegHeight::hangHipCad()+in.hip_rad[0], in.knee_rad[0],
        LegHeight::hangHipCad()+in.hip_rad[1], in.knee_rad[1]);
    in.pitch_rad = -std::atan2(b.com_offset_x_m,b.com_above_axle_m)+0.005F;
    in.accel_z_m_s2 = 9.80665F;
    V3Controller::Command cmd{};
    cmd.height_m = LegHeight::minHeight();
    auto tick = [&]() { in.now_us += 2000U; return ctrl.update(in,cmd); };
    cmd.standup = true; auto out=tick(); cmd.standup=false;
    for (int i=0; i<500 && out.state==V3Controller::State::StandUp; ++i) out=tick();
    assert(out.state==V3Controller::State::Balancing);
    for (int i=0; i<5000; ++i) { out=tick(); assert(!out.fault); }
    assert(ctrl.learnedTrimValid());
    assert(std::abs(ctrl.levelHip())>0.01F);
    if (recover) {
      assert(ctrl.postStandUpPhase()==PostStandUpBalance::Phase::Normal);
      const float hip=out.leg_hip_offset_rad, knee=out.leg_knee_offset_rad;
      const float body=in.pitch_rad;
      for(int i=0;i<200;++i) {
        in.pitch_rad=body+(i%2 ? 0.02F : -0.02F);
        in.pitch_rate_rad_s=i%2 ? 0.3F : -0.3F;
        out=tick(); assert(!out.fault);
        assert(std::abs(out.leg_hip_offset_rad-hip)<1e-6F);
        assert(std::abs(out.leg_knee_offset_rad-knee)<1e-6F);
      }
      in.pitch_rad=body;in.pitch_rate_rad_s=0.0F;
    }
    const float learned = ctrl.learnedTrim();
    // Test both Fault retry (held POS) and DISABLE retry.
    for (bool disable : {false,true}) {
      if (disable) { cmd.disable=true; out=tick(); cmd.disable=false; }
      else { in.sensors_ok=false; out=tick(); in.sensors_ok=true; }
      in.pitch_rad += 0.005F;
      cmd.standup=true; out=tick(); cmd.standup=false;
      for (int i=0; i<500 && out.state==V3Controller::State::StandUp; ++i) out=tick();
      assert(out.state==V3Controller::State::Balancing);
      const float measured=ctrl.virtualPitch();
      if (recover) {
        assert(std::abs(ctrl.balanceRef()-measured)<1e-5F);
        assert(std::abs(ctrl.balanceRef()-learned)>0.003F);
        for (int i=0; i<300; ++i) {
          out=tick(); assert(!out.fault);
          assert(ctrl.postStandUpPhase()==PostStandUpBalance::Phase::CaptureHold);
          assert(std::abs(ctrl.levelHip())<1e-6F);
        }
      } else assert(std::abs(ctrl.balanceRef()-learned)<1e-5F);
    }
  }
}

// A quiet angular residual is not a stationary capture (robot 2026-10-14).
void movingWheelsCannotQualifyPostureTransfer() {
  for (bool enabled : {false, true}) {
    PostStandUpBalance t;
    PostStandUpBalance::Input in{};
    in.dt_s=kDt; in.settle_s=0.0F; in.grace_s=0.0F;
    in.smooth_transfer=true; in.speed_qualified=enabled;
    in.ground_speed_m_s=0.47F; in.command_speed_m_s=0.60F;
    t.begin(0.6F,0.4F);
    for(int i=0;i<1000;++i) t.update(in);
    if (enabled) {
      assert(t.phase()==PostStandUpBalance::Phase::CaptureHold);
      in.ground_speed_m_s=0.0F;
      for(int i=0;i<1000;++i) t.update(in);
      assert(t.phase()==PostStandUpBalance::Phase::CaptureHold); // command still races
      in.command_speed_m_s=0.0F;
      for(int i=0;i<100;++i) t.update(in);
      assert(t.phase()==PostStandUpBalance::Phase::PoseTransfer);
      in.ground_speed_m_s=0.2F;
      const auto stopped=t.update(in);
      for(int i=0;i<100;++i) assert(std::abs(t.update(in).hip_rad-stopped.hip_rad)<1e-6F);
    } else assert(t.phase()!=PostStandUpBalance::Phase::CaptureHold);
  }
}

void captureBoundsWheelsAndHoldsKneeBeforeNormal() {
  for (bool safe : {false, true}) {
    auto cfg=defaultV3Config();
    cfg.standup.enabled=true; cfg.standup.hold_s=0.0F;
    cfg.standup.smooth_balance_handover=true;
    cfg.standup.settled_balance_handover=true;
    cfg.standup.repeatable_balance_capture=true;
    cfg.standup.hold_transferred_pose=true;
    cfg.standup.safe_balance_capture=safe;
    cfg.tier.model_check_rad=10.0F;
    V3Controller ctrl(cfg);
    V3Controller::Input in{};
    in.dt_s=kDt; in.legs_measured=in.legs_ready=in.sensors_ok=true;
    in.hip_rad={0.63F,0.63F};
    in.knee_rad={cfg.standup.knee_end_rad,cfg.standup.knee_end_rad};
    const auto b=robot_control::WholeBodyModel{}.compute(
        LegHeight::hangHipCad()+in.hip_rad[0],in.knee_rad[0],
        LegHeight::hangHipCad()+in.hip_rad[1],in.knee_rad[1]);
    in.pitch_rad=-std::atan2(b.com_offset_x_m,b.com_above_axle_m)+0.005F;
    in.accel_z_m_s2=9.80665F;
    V3Controller::Command cmd{}; cmd.height_m=LegHeight::minHeight(); cmd.standup=true;
    auto tick=[&](){in.now_us+=2000U; return ctrl.update(in,cmd);};
    auto out=tick(); cmd.standup=false;
    for(int i=0;i<500 && out.state==V3Controller::State::StandUp;++i) out=tick();
    assert(out.state==V3Controller::State::Balancing);
    const float knee=out.leg_knee_offset_rad;
    bool knee_moved=false;
    // Low body rate while wheel speed ramps: the old held detector can mistake
    // real floor motion for a fixture, erasing the station/trim repeatedly.
    for(int i=0;i<1500 && !out.fault;++i) {
      in.wheel_rad_s={float(i)*0.03F,float(i)*0.03F};
      in.wheel_pos_rad[0]+=in.wheel_rad_s[0]*kDt;
      in.wheel_pos_rad[1]=in.wheel_pos_rad[0];
      out=tick();
      knee_moved |= std::abs(out.leg_knee_offset_rad-knee)>1e-4F;
      if(safe) {
        assert(ctrl.heldResets()==0);
        assert(!knee_moved);
        for(float w:out.wheel_cmd_rad_s) assert(std::abs(w)*cfg.wheel_radius_m<=0.60001F);
      }
    }
    if(safe) {
      assert(out.fault && out.state==V3Controller::State::Fault);
      assert(!out.wheels_enabled && out.legs_locked);
      assert(ctrl.postStandUpPhase()!=PostStandUpBalance::Phase::Normal);
      cmd.disable=true;out=tick();assert(out.state==V3Controller::State::Idle);
    } else assert(knee_moved);
  }
}


void rapidBodyMotionCannotMasqueradeAsComCapture() {
  for(bool calm:{false,true}) {
    StandUp::Config cfg{};cfg.hold_s=0.0F;cfg.capture_governor=calm;
    StandUp su(cfg);su.begin(0.6F,0.0F);
    StandUp::Input in{};in.dt_s=kDt;in.hip_goal_rad=0.6F;
    in.pitch_rate_rad_s=1.2F;in.lean_rad=in.lean_rate_rad_s=0.0F;
    StandUp::Output out{};
    for(int i=0;i<100;++i)out=su.update(in);
    if(calm) {
      assert(out.phase!=StandUp::Phase::Caught);
      in.pitch_rate_rad_s=0.3F;
      for(int i=0;i<40;++i)out=su.update(in);
      assert(out.phase==StandUp::Phase::Caught);
    } else assert(out.phase==StandUp::Phase::Caught);
  }
  robot_control::LqrBalancer b;
  b.reset();b.seedWheelCommandVelocity(0.2F);
  robot_control::RobotState state{};
  state.dt_s=kDt;state.floor_contact=true;state.wheel.velocity_m_s=0.2F;
  robot_control::ControlCommand cmd{};
  (void)b.update(state,cmd,true,0.0F,true);
  assert(std::abs(b.debugState().lqr_velocity_reference_m_s)<1e-6F);
  assert(b.debugState().commanded_ground_velocity_m_s>0.1F);
}

void trimmedGoalCanCatchWithoutAnAccidentalRock() {
  // Reproduce the deterministic inconsistency seen in tap_20261014_122220:
  // quiet body, POS following a goal with -0.0396 rad and 1 degree hip trim.
  // A zero-centred 0.025 rad band cannot accept that delivered goal.
  for (bool governor : {false, true}) {
    auto cfg = defaultV3Config();
    cfg.standup.enabled = true;
    cfg.standup.capture_governor = governor;
    cfg.standup.extra_fold_rad = 0.0174532925F;
    V3Controller ctrl(cfg);
    V3Controller::Input in{};
    in.sensors_ok = in.legs_ready = in.legs_measured = true;
    in.pitch_rad = -0.42F;
    float lo = -1.5F, hi = 2.0F;
    for (int i = 0; i < 24; ++i) {
      const float mid = 0.5F * (lo + hi);
      const auto model = robot_control::WholeBodyModel{}.compute(
          LegHeight::hangHipCad() + mid, cfg.standup.knee_end_rad,
          LegHeight::hangHipCad() + mid, cfg.standup.knee_end_rad);
      (in.pitch_rad + std::atan2(model.com_offset_x_m, model.com_above_axle_m) > 0.0F ? hi : lo) = mid;
    }
    const float delivered = 0.5F * (lo + hi) + cfg.standup.hip_goal_offset_rad - cfg.standup.extra_fold_rad;
    in.hip_rad.fill(delivered);
    in.knee_rad.fill(cfg.standup.knee_end_rad);
    in.accel_z_m_s2 = 9.80665F;
    V3Controller::Command cmd{}; cmd.standup = true;
    auto out = ctrl.update(in, cmd); cmd.standup = false;
    float elapsed = 0.0F;
    for (int i = 0; i < 4100 && out.state == V3Controller::State::StandUp; ++i) {
      // An encoder-derived POS response with a 100 ms settling time; body is
      // deliberately static so this tests the handshake, not floor support.
      const float previous = in.hip_rad[0];
      in.hip_rad.fill(previous + kDt / (0.10F + kDt) * (out.leg_hip_offset_rad - previous));
      in.hip_rate_rad_s.fill((in.hip_rad[0] - previous) / kDt);
      in.knee_rad.fill(out.leg_knee_offset_rad);
      in.dt_s = kDt; in.now_us += 2000U; elapsed += kDt;
      out = ctrl.update(in, cmd);
    }
    if (governor) {
      assert(out.state == V3Controller::State::Balancing && out.wheels_enabled);
      assert(elapsed < 3.0F);
      assert(std::abs(ctrl.standupLean()) < cfg.standup.catch_lean_rad);
    } else {
      assert(out.state == V3Controller::State::Fault && !out.wheels_enabled);
      assert(elapsed >= 7.99F);
    }
    std::printf("trimmed catch governor=%d duration=%.3f lean=%.4f state=%s\n",
        governor, elapsed, ctrl.standupLean(), toString(out.state));
  }
  StandUp::Config cfg{}; cfg.capture_governor = true; cfg.hold_s = 0.0F;
  cfg.extra_fold_rad = 0.0174532925F;
  for (int unsafe : {0, 1, 2}) {
    StandUp su(cfg); su.begin(0.6F, 0.0F);
    StandUp::Input in{}; in.dt_s = kDt; in.hip_goal_rad = 0.6F;
    in.lean_rad = in.catch_lean_target_rad = -0.05F;
    if (unsafe == 0) in.pitch_rate_rad_s = 0.8F;
    if (unsafe == 1) in.lean_rate_rad_s = 0.8F;
    if (unsafe == 2) in.lean_rad = in.catch_lean_target_rad = -0.12F;
    StandUp::Output out{};
    for (int i = 0; i < 100; ++i) out = su.update(in);
    assert(out.phase == StandUp::Phase::Rise && !out.wheels_on);
  }
}

// Diagnostic planar post-catch experiment, not a robot acceptance test.
// Reuses LegPlant and the older tier harness servo assumptions. Ideal body
// pitch/gyro, no shell contacts; wheel/hip lags are NOT calibrated for new motors.
void planarPostCatchDiagnostic(double initial_rate = 0.0, double wheel_lag = 0.035, double initial_hip = 0.73, float lead = -1.0F, bool governor_only = false) {
  using tap_fw::sim::LegPlant;
  namespace geo = robot_control::geometry;
  double baseline_peak = 0.0;
  for (int hip_delay : {2, 10, 25}) for (int profile : {1, 2, 3, 4, 5, 6}) {
    if (governor_only && profile!=6) continue;
    const bool smooth = true;
    const int extra_deg = 1;
    auto cfg = defaultV3Config();
    cfg.standup.enabled = true;
    cfg.standup.hold_s = 0.0F;
    cfg.standup.smooth_balance_handover = smooth;
    cfg.standup.settled_balance_handover = profile >= 2;
    cfg.standup.repeatable_balance_capture = profile >= 3;
    cfg.standup.hold_transferred_pose = profile >= 4;
    cfg.standup.safe_balance_capture = profile >= 5;
    cfg.standup.capture_governor = profile == 6;
    if (lead >= 0.0F) {
      cfg.lqr_lead_s=lead;
      cfg.trim_acc=true;
      cfg.tier.height_exp=0.15F;
      cfg.tier.body_damping_s=0.08F;
    }
    cfg.tier.suspension = cfg.tier.preview = false;
    cfg.standup.extra_fold_rad = extra_deg * 0.0174532925F;
    LegPlant plant(0.0, 1.0, 0.004, 0.004, LegHeight::hangHipCad());
    LegPlant::Vec s{0.0, 0.0, initial_hip, cfg.standup.knee_end_rad}, sd{};
    for (int i = 0; i < 40; ++i) s[1] -= plant.leanAngle(s) - 0.02;
    const double initial_pitch = s[1];
    V3Controller ctrl(cfg);
    while (!ctrl.balancer().buildGainTableStep(32)) {}
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
    sd[1] = initial_rate;
    auto integral = plant.staticJointTorques(s);
    double wheel_rate = 0.0, wheel_pos = 0.0;
    double hip_applied = out.leg_hip_offset_rad, knee_applied = out.leg_knee_offset_rad;
    double wheel_applied = 0.0, peak_pitch = 0.0, peak_hip_rate = 0.0;
    double previous_hip = hip_applied;
    double rate_sq = 0.0, rate_sum = 0.0, pitch_after2 = 0.0, level_peak = 0.0; int rate_n = 0;
    double transfer_peak_rate = 0.0;
    std::deque<double> wq, hq, kq;
    int ticks = 0;
    for (; ticks < (profile==6 ? 23000 : 6000); ++ticks) {
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
        const double tw = std::clamp(((wheel_applied-wheel_rate)/wheel_lag-u0)/(u1-u0), -0.6, 0.6);
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
    if (governor_only && std::abs(lead-0.09F)<1e-6F) {
      // Regression of this provisional plant, NOT a physical release gate.
      // Keep the fast-wheel mismatch visible instead of calling all exits a pass.
      assert(transfer_peak_rate<=0.0401);
      if (wheel_lag>=0.13) {
        assert(!out.fault && ctrl.postStandUpPhase()==PostStandUpBalance::Phase::Normal);
        assert(rate_n && std::sqrt(rate_sq/rate_n)<0.02);
      } else {
        assert(out.fault && ctrl.captureGovernorPhase()==int(BalanceCaptureGovernor::Phase::Failed));
      }
    }
    if (profile >= 2 && profile <=5 && wheel_lag <= 0.06 && lead < 0.0F) {
      assert(!out.fault);
      assert(ctrl.postStandUpPhase() == PostStandUpBalance::Phase::Normal);
      if (initial_rate == 0.0 && initial_hip == 0.73 && wheel_lag == 0.035) assert(peak_pitch < baseline_peak);
      assert(transfer_peak_rate <= 0.13);
      assert(peak_hip_rate <= 0.36);
    }
    std::printf("planar MIN postcatch rate0=%.2f wheel_lag=%.3f hip0=%.2f profile=%d extra_deg=%d hip_delay_ms=%d duration=%.3f pitch_change_peak=%.3f hip_cmd_rate=%.3f rate_rms_after2=%.3f abs_pitch_after2=%.3f level_peak=%.3f rate_std_after2=%.3f pitch_end=%.3f phase=%d fault=%s\n",
        initial_rate, wheel_lag, initial_hip, profile, extra_deg, hip_delay*2, ticks*double(kDt), peak_pitch, peak_hip_rate,
        rate_n ? std::sqrt(rate_sq/rate_n) : 0.0, pitch_after2, level_peak,
        rate_n ? std::sqrt(std::max(rate_sq/rate_n - (rate_sum/rate_n)*(rate_sum/rate_n), 0.0)) : 0.0,
        s[1], int(ctrl.postStandUpPhase()),
        out.state == V3Controller::State::Fault ? ctrl.lastFault() : "none");
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc > 1) {
    const float lead=std::strtof(argv[1],nullptr);
    const bool governor_only=argc>2 && std::strcmp(argv[2],"capture")==0;
    planarPostCatchDiagnostic(0.0,0.035,0.73,lead,governor_only);
    planarPostCatchDiagnostic(0.0,0.13,0.63,lead,governor_only);
    planarPostCatchDiagnostic(0.0,0.20,0.63,lead,governor_only);
    return 0;
  }
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
  transferRampsVelocityAndBrakesOnModerateNoise();
  retriesSeedThisCatchAndFreezeOldLevelState();
  rapidBodyMotionCannotMasqueradeAsComCapture();
  trimmedGoalCanCatchWithoutAnAccidentalRock();
  movingWheelsCannotQualifyPostureTransfer();
  captureBoundsWheelsAndHoldsKneeBeforeNormal();
  planarPostCatchDiagnostic();
  planarPostCatchDiagnostic(0.4, 0.060, 0.63);
  planarPostCatchDiagnostic(-0.4, 0.060, 0.42);
  planarPostCatchDiagnostic(0.0, 0.20, 0.63);
  std::puts("v3 standup tests passed");
  return 0;
}
