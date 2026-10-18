#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

#include "tap_fw/v3/StandUp.hpp"
#include "tap_fw/v3/V3Controller.hpp"
#include "robot_control/BodyClearance.hpp"

using namespace tap_fw::v3;

namespace {
StandUp::Input sample(float dt = .01F) {
  StandUp::Input in;
  in.dt_s = dt;
  return in;
}

StandUp catching() {
  StandUp::Config cfg;
  cfg.enabled = true;
  cfg.fold_travel_rad = 0;
  cfg.kneel_hold_s = .02F;
  cfg.pitch_settle_s = .02F;
  StandUp su(cfg);
  su.begin(0, 0);
  for (int i = 0; i < 20 && su.phase() != StandUp::Phase::Catching; ++i)
    (void)su.update(sample());
  assert(su.phase() == StandUp::Phase::Catching);
  return su;
}

void singleCommandAndDiagnosticHolds() {
  StandUp::Config cfg;
  cfg.enabled = true;
  cfg.fold_travel_rad = .1F;
  cfg.fold_rate_rad_s = 1;
  cfg.pitch_settle_s = .02F;
  StandUp su(cfg);
  su.begin(.1F, 0);
  auto in = sample();
  auto out = su.update(in);
  assert(out.phase == StandUp::Phase::Folding && out.hip_target_rad < .1F);
  for (int i=0; i<20 && su.phase()==StandUp::Phase::Folding; ++i) out=su.update(in);
  assert(su.phase()==StandUp::Phase::Kneeling);
  const float held=out.hip_target_rad;
  for(int i=0;i<99;++i) {
    out=su.update(in);
    assert(out.phase==StandUp::Phase::Kneeling);
    assert(out.hip_target_rad==held && out.wheel_rad_s==0);
  }
  // Float accumulation can put the transition one tick beyond exactly 1 s.
  for(int i=0;i<2 && su.phase()==StandUp::Phase::Kneeling;++i) out=su.update(in);
  assert(su.phase()==StandUp::Phase::Pitching);
  for(int i=0;i<10 && su.phase()!=StandUp::Phase::Catching;++i) out=su.update(in);
  assert(su.phase()==StandUp::Phase::Catching);

  cfg.fold_travel_rad=0; cfg.hold_at_kneel=true; cfg.hold_after_flip=true;
  StandUp manual(cfg);manual.begin(0,0);
  for(int i=0;i<200;++i) out=manual.update(in);
  assert(out.phase==StandUp::Phase::Kneeling);
  assert(manual.advanceFromKneeling());
  for(int i=0;i<100;++i)out=manual.update(in);
  assert(out.phase==StandUp::Phase::Flipped && out.wheel_rad_s==0);
  assert(!manual.advanceFromKneeling());
}

void feedbackCanReverseAndIsBounded() {
  auto su=catching();auto in=sample();in.capture_lean_rad=-.30F;
  float previous=0,previous_rate=0;
  for(int i=0;i<30;++i) {
    in.hip_measured_rad=previous;
    const auto out=su.update(in);
    assert(out.phase==StandUp::Phase::Catching);
    assert(std::abs(out.hip_rate_rad_s)<=su.config().catch_hip_rate_max_rad_s+1e-5F);
    assert(std::abs(out.hip_rate_rad_s-previous_rate)<=su.config().catch_hip_accel_rad_s2*in.dt_s+1e-4F);
    assert(std::abs(out.wheel_rad_s)<=su.config().catch_wheel_max_rad_s);
    assert(out.hip_target_rad>=previous);
    previous=out.hip_target_rad;previous_rate=out.hip_rate_rad_s;
  }
  in.capture_lean_rad=.30F;
  bool reversed=false;
  for(int i=0;i<30;++i) {
    in.hip_measured_rad=previous;
    const auto out=su.update(in);
    if(out.hip_target_rad<previous)reversed=true;
    previous=out.hip_target_rad;
  }
  assert(reversed);
}

void crossingZeroIsNotCapture() {
  auto su=catching();auto in=sample();
  in.capture_rate_rad_s=1.0F; // angle is zero but the divergent mode is 0.10 rad
  for(int i=0;i<20;++i) {
    const auto out=su.update(in);
    assert(out.phase==StandUp::Phase::Catching);
    in.hip_measured_rad=out.hip_target_rad;
  }
  in.capture_rate_rad_s=0;
  for(int i=0;i<20 && su.phase()!=StandUp::Phase::Support;++i) {
    const auto out=su.update(in);in.hip_measured_rad=out.hip_target_rad;
  }
  assert(su.phase()==StandUp::Phase::Support);
  const auto held=su.update(in);
  in.pitch_rad=.1F;
  const auto again=su.update(in);
  assert(again.phase==StandUp::Phase::Support && again.hip_target_rad==held.hip_target_rad);
}

void carrierAndSafety() {
  auto su=catching();auto in=sample();in.carrier_rate_rad_s=1.25F;
  auto out=su.update(in);
  assert(std::abs(out.wheel_rad_s+1.25F)<1e-5F);
  // Faster-than-command rolling must not invert the anti-windup clamp bounds.
  su=catching();in=sample();in.ground_speed_m_s=2;
  out=su.update(in);
  assert(std::isfinite(out.wheel_rad_s) && std::abs(out.wheel_rad_s)<=10);
  su=catching();in=sample();in.sensors_ok=false;out=su.update(in);
  assert(out.phase==StandUp::Phase::Failed && !out.wheels_on && out.wheel_rad_s==0);
  su=catching();in=sample();in.capture_rate_rad_s=std::numeric_limits<float>::quiet_NaN();out=su.update(in);
  assert(out.phase==StandUp::Phase::Failed && !out.wheels_on);
  su=catching();in=sample();in.hip_jacobian=0;out=su.update(in);
  assert(out.phase==StandUp::Phase::Failed);
  su=catching();su.config().timeout_s=.015F;in=sample();in.capture_lean_rad=.3F;
  for(int i=0;i<3;++i)out=su.update(in);
  assert(out.phase==StandUp::Phase::Failed);
  su=catching();su.config().catch_wheel_max_rad_s=-1;out=su.update(sample());
  assert(out.phase==StandUp::Phase::Failed);
}

void controllerDisableWins() {
  auto cfg=defaultV3Config();assert(!cfg.standup.enabled);
  cfg.standup.enabled=true;
  V3Controller c(cfg);V3Controller::Input in{};
  in.legs_measured=in.legs_ready=in.sensors_ok=true;
  in.hip_rad={1.3F,1.3F};in.leg_chain_rad=1.3F;
  V3Controller::Command cmd{};cmd.standup=true;
  auto out=c.update(in,cmd);assert(out.state==V3Controller::State::StandUp);
  cmd.disable=true;in.now_us+=2000;out=c.update(in,cmd);
  assert(out.state==V3Controller::State::Idle && !out.legs_locked && !out.wheels_enabled);
  cmd={};in.now_us+=2000;out=c.update(in,cmd);assert(out.state==V3Controller::State::Idle);
  cmd.standup=true;in.now_us+=2000;out=c.update(in,cmd);
  assert(out.state==V3Controller::State::StandUp);
  cmd={};in.sensors_ok=false;in.now_us+=2000;out=c.update(in,cmd);
  assert(out.state==V3Controller::State::Fault && !out.wheels_enabled);
  in.sensors_ok=true;in.now_us+=2000;out=c.update(in,cmd);
  assert(out.state==V3Controller::State::Fault); // explicit restart required
}

void liftOverlapAndClearance() {
  StandUp::Config cfg;
  cfg.enabled=true; cfg.overlap_after_lift=true;
  cfg.fold_travel_rad=0; cfg.kneel_hold_s=0;
  cfg.flip_pitch_kp_rad_s_per_rad=10; cfg.flip_hip_rate_max_rad_s=3;
  StandUp su(cfg);su.begin(0,0);
  auto in=sample();in.pitch_rad=.4F;in.capture_lean_rad=-.3F;
  auto out=su.update(in);out=su.update(in);out=su.update(in);
  assert(out.phase==StandUp::Phase::Pitching);
  // Pitch progress by itself is insufficient; no sensor/model clearance.
  in.pitch_rad=.25F;in.pitch_rate_rad_s=-.8F;
  for(int i=0;i<4;++i)out=su.update(in);
  assert(out.phase==StandUp::Phase::Pitching && out.wheel_rad_s==0);
  in.body_clearance_valid=true;in.body_clearance_m=.002F;
  out=su.update(in);assert(out.phase==StandUp::Phase::Pitching);
  in.body_clearance_m=.006F;
  for(int i=0;i<4 && su.phase()!=StandUp::Phase::Catching;++i)out=su.update(in);
  assert(out.phase==StandUp::Phase::Catching);
  assert(out.catch_blend==0 && out.wheel_rad_s==0);
  assert(std::abs(in.pitch_rate_rad_s)>cfg.pitch_rate_tol_rad_s);
  float last_rate=out.hip_rate_rad_s, last_blend=0;
  for(int i=0;i<40;++i) {
    in.hip_measured_rad=out.hip_target_rad;
    out=su.update(in);
    assert(out.phase==StandUp::Phase::Catching);
    assert(out.catch_blend>=last_blend && out.catch_blend<=1);
    assert(std::abs(out.hip_rate_rad_s-last_rate)<=cfg.catch_hip_accel_rad_s2*in.dt_s+1e-4F);
    assert(std::abs(out.wheel_rad_s)<=cfg.catch_wheel_max_rad_s);
    last_rate=out.hip_rate_rad_s;last_blend=out.catch_blend;
  }
  assert(out.catch_blend==1);

  // An apparently level body still cannot launch with invalid clearance.
  su=StandUp(cfg);su.begin(0,0);in=sample();
  for(int i=0;i<100;++i)out=su.update(in);
  assert(out.phase==StandUp::Phase::Pitching && out.wheel_rad_s==0);
  su.config().timeout_s=.1F;out=su.update(in);
  assert(out.phase==StandUp::Phase::Failed);

  const float flat=robot_control::bodyClearance(0,0,{0,0},{0,0},.04F);
  // Independent CAD-zero axle z=-29.88 mm, shell bottom=-41.721 mm.
  assert(std::abs(flat-.028158F)<3e-6F);
  const float l=robot_control::bodyClearance(.2F,.04F,{0,0},{0,0},.04F);
  const float r=robot_control::bodyClearance(.2F,-.04F,{0,0},{0,0},.04F);
  assert(std::isfinite(l) && std::abs(l-r)<1e-5F);
  assert(std::isnan(robot_control::bodyClearance(0,0,{0,0},{0,0},-1)));
}

void peakWindowExpires() {
  auto su=catching();su.config().catch_wheel_peak_s=.03F;
  auto in=sample();in.capture_lean_rad=-.3F;
  StandUp::Output out{};
  for(int i=0;i<50;++i) {
    in.hip_measured_rad=out.hip_target_rad;
    out=su.update(in);
    if(i==0) assert(out.wheel_peak_requested);
    if(i>=3) assert(!out.wheel_peak_requested);
  }
  su.reset();out=su.update(in);
  assert(!out.wheel_peak_requested && !out.wheels_on);
  su=catching();su.config().catch_wheel_peak_s=1.01F;out=su.update(in);
  assert(out.phase==StandUp::Phase::Failed && !out.wheel_peak_requested);
}

void rollingHandoffSeedsPreview() {
  for (float speed : {-.4F, .4F}) {
    auto cfg=defaultV3Config();
    cfg.standup.enabled=true;
    cfg.standup.fold_travel_rad=0;
    cfg.standup.kneel_hold_s=cfg.standup.pitch_settle_s=0;
    cfg.standup.catch_settle_s=0;
    // Synthetic transition contract; physical capture is covered in MuJoCo.
    cfg.standup.catch_lean_tol_rad=cfg.standup.catch_capture_tol_rad=10;
    cfg.tier.preview=true;
    V3Controller c(cfg);
    V3Controller::Input in{};
    in.legs_ready=in.legs_measured=in.sensors_ok=true;
    in.accel_z_m_s2=9.80665F;
    const auto pose=LegHeight::forKnee(LegHeight::kKneeMinRad);
    in.hip_rad={pose.hip_offset_rad,pose.hip_offset_rad};
    in.knee_rad={pose.knee_offset_rad,pose.knee_offset_rad};
    in.leg_chain_rad=pose.hip_offset_rad+pose.knee_offset_rad;
    in.wheel_rad_s={speed/cfg.wheel_radius_m,speed/cfg.wheel_radius_m};
    V3Controller::Command cmd{};cmd.standup=true;
    V3Controller::Output out{};
    for(int i=0;i<10;++i) {
      in.now_us+=2000;
      out=c.update(in,cmd);cmd.standup=false;
      if(out.state==V3Controller::State::Balancing) break;
    }
    assert(out.state==V3Controller::State::Balancing);
    // The hand-over keeps the speed the wheels already roll at (the controller's plan_v_ is seeded with it).
    assert(c.groundVelocity()*speed>0);
  }
}

}

int main() {
  singleCommandAndDiagnosticHolds();feedbackCanReverseAndIsBounded();
  crossingZeroIsNotCapture();carrierAndSafety();controllerDisableWins();
  liftOverlapAndClearance();
  peakWindowExpires();
  rollingHandoffSeedsPreview();
  std::puts("stand-up: autonomous sequencing, bidirectional capture, bounded commands and stop priority passed");
}
