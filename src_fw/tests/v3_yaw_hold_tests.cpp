#include <cassert>
#include <cmath>
#include <cstdio>
#include "tap_fw/v3/YawHold.hpp"
#include "tap_fw/v3/V3Controller.hpp"
using tap_fw::v3::YawHold;

void signsAndLimits() {
  for (float drive : {-.3F,.3F}) for (int side : {-1,1}) {
    YawHold y;YawHold::Config c;c.enabled=true;
    float previous=0;
    for(int i=0;i<100;++i) {
      // left obstruction => positive yaw forward, negative in reverse
      const float rate=.3F*side*(drive>0?1:-1);
      auto out=y.update(c,{drive/.04F,drive/.04F},.002F,rate,0,drive,true,20,.1545F,.04F);
      assert(y.correction()*rate<=0);
      assert(std::abs(y.correction()-previous)<=c.rise_rad_s2*.002F+1e-5F);
      assert(std::abs(out[0]+out[1]-2*drive/.04F)<1e-5F);
      // side +1 is left blocked: add speed in the travel direction.
      const int blocked=side==1?0:1;
      assert((out[blocked]-drive/.04F)*drive>0);
      previous=y.correction();
    }
    assert(std::abs(y.correction())>.5F);
    // hard block must run out of budget, unwind and stay disarmed
    for(int i=0;i<1600;++i)y.update(c,{drive/.04F,drive/.04F},.002F,.3F,0,drive,true,20,.1545F,.04F);
    assert(y.blocked() && std::abs(y.correction())<1e-5F);
    y.update(c,{0,0},.002F,0,0,0,true,20,.1545F,.04F);
    assert(!y.blocked());
  }
  for(int mode=0;mode<5;++mode) {
    YawHold y;YawHold::Config c;c.enabled=true;
    for(int i=0;i<150;++i)y.update(c,{7,7},.002F,.2F,0,.3F,true,20,.1545F,.04F);
    auto out=y.update(c,{7,7},.002F,0,mode==0?.2F:0,
        mode==1?0:mode==2?-.3F:.3F,mode!=3,20,.1545F,.04F);
    if(mode<4)assert(out[0]==7 && out[1]==7 && y.error()==0);
  }
  YawHold y;YawHold::Config c;c.enabled=true;
  for(int i=0;i<100;++i) {
    auto out=y.update(c,{19.9F,19.9F},.002F,.5F,0,.3F,true,20,.1545F,.04F);
    assert(std::abs(out[0])<=20 && std::abs(out[1])<=20);
    assert(std::abs(out[0]+out[1]-39.8F)<1e-5F);
  }
  y.reset();
  for(int i=0;i<200;++i) {
    auto out=y.update(c,{20,20},.002F,.5F,0,.3F,true,20,.1545F,.04F);
    assert(out[0]==20 && out[1]==20 && y.error()==0);
  }
  y.update(c,{7,7},.02F,.5F,0,.3F,true,20,.1545F,.04F);
  assert(y.correction()==0);
  c.enabled=false;auto out=y.update(c,{7,8},.002F,1,0,.3F,true,20,.1545F,.04F);
  assert(out[0]==7 && out[1]==8 && y.error()==0);
}

// Differential wheel speed plant with prescribed one-sided speed loss.
// This tests yaw recovery across delays, not curb climbing or ground contact.
float simulate(bool enabled,int blocked,float drive,float tau,int delay) {
  YawHold y;YawHold::Config c;c.enabled=enabled;
  float l=drive/.04F,r=l,angle=0;
  float history[80][2]{};
  for(auto& row:history)row[0]=row[1]=l;
  for(int i=0;i<1000;++i) {
    const float gyro=.04F*(r-l)/.309F;
    // Existing inner rate correction, yawkp=.6, plus heading outer loop.
    const float diff=-.6F*gyro*.1545F/.04F;
    auto cmd=y.update(c,{drive/.04F-diff,drive/.04F+diff},.002F,gyro,0,drive,true,20,.1545F,.04F);
    const int slot=i%80,old=(i+80-delay)%80;
    history[slot][0]=cmd[0];history[slot][1]=cmd[1];
    const float loss=(i>=100 && i<350) ? std::copysign(2.0F,drive) : 0;
    l+=.002F/tau*(history[old][0]-(blocked==0?loss:0)-l);
    r+=.002F/tau*(history[old][1]-(blocked==1?loss:0)-r);
    angle+=gyro*.002F;
  }
  return std::abs(angle);
}
void controllerWiring() {
  using tap_fw::v3::V3Controller;
  auto cfg=tap_fw::v3::defaultV3Config();cfg.yaw_hold.enabled=true;
  cfg.tier.yaw_bias_tau_s=0; // prescribed bias-free gyro
  V3Controller c(cfg);V3Controller::Input in{};V3Controller::Command cmd{};
  in.accel_z_m_s2=9.80665F;in.legs_ready=true;
  cmd.start=true;bool active=false;
  for(int i=0;i<1800;++i) {
    in.now_us+=2000;cmd.go=c.state()==V3Controller::State::Ready;
    cmd.forward_m_s=c.state()==V3Controller::State::Balancing?.2F:0;
    in.yaw_rate_rad_s=c.state()==V3Controller::State::Balancing?.15F:0;
    auto out=c.update(in,cmd);cmd.start=false;
    assert(out.state!=V3Controller::State::Fault);
    if(c.yawHold().correction()<-.1F) {active=true;break;}
  }
  assert(active);
  cmd.disable=true;in.now_us+=2000;auto out=c.update(in,cmd);
  assert(out.state==V3Controller::State::Idle);
  assert(out.wheel_cmd_rad_s[0]==0 && out.wheel_cmd_rad_s[1]==0);
  assert(c.yawHold().correction()==0 && c.yawHold().error()==0);
}

void standupYawFeedback() {
  using tap_fw::v3::V3Controller;
  for (bool enabled : {false, true}) for (float yaw : {-.08F, .08F}) {
    auto cfg = tap_fw::v3::defaultV3Config();
    cfg.standup.enabled = true;
    cfg.standup.fold_rate_rad_s = .01F;
    cfg.yaw_hold.enabled = enabled;
    V3Controller c(cfg);
    V3Controller::Input in{};
    in.dt_s = .002F;
    in.legs_measured = in.legs_ready = in.sensors_ok = true;
    in.hip_rad = {1.3F, 1.3F}; in.leg_chain_rad = 1.3F;
    V3Controller::Command cmd{}; cmd.standup = true;
    auto out = c.update(in, cmd);
    assert(out.state == V3Controller::State::StandUp);
    const float entry_bias = c.yawBias();
    cmd.standup = false;
    float previous_diff = 0;
    for (int i = 0; i < 500; ++i) {
      in.now_us += 2000; in.yaw_rate_rad_s = yaw;
      out = c.update(in, cmd);
      assert(out.state == V3Controller::State::StandUp && out.wheels_enabled);
      assert(std::abs(out.wheel_cmd_rad_s[0] + out.wheel_cmd_rad_s[1]) < 1e-6F);
      assert(c.yawBias() == entry_bias); // real slow turn must not train bias
      const float diff = .5F * (out.wheel_cmd_rad_s[1] - out.wheel_cmd_rad_s[0]);
      assert(diff * yaw <= 0);
      assert(std::abs(diff - previous_diff) <= cfg.yaw_hold.rise_rad_s2 * in.dt_s + 1e-6F);
      assert(std::abs(diff) <= 1.5F);
      if (!enabled) assert(diff == 0);
      previous_diff = diff;
    }
    if (enabled) assert(std::abs(previous_diff) > .1F);
    cmd.disable = true; in.now_us += 2000; out = c.update(in, cmd);
    assert(!out.wheels_enabled && out.wheel_cmd_rad_s[0] == 0 && out.wheel_cmd_rad_s[1] == 0);
  }
  puts("standup: both yaw signs, feature off, mean preservation, bounded slew, bias freeze and disable passed");
}

float standupRatePlant(bool enabled, float tau, int delay, float disturbance) {
  auto cfg = tap_fw::v3::defaultV3Config();
  cfg.standup.enabled = true; cfg.standup.fold_rate_rad_s = .01F;
  cfg.yaw_hold.enabled = enabled; cfg.tier.yaw_bias_tau_s = 0;
  tap_fw::v3::V3Controller c(cfg);
  tap_fw::v3::V3Controller::Input in{};
  in.dt_s = .002F; in.legs_measured = in.legs_ready = in.sensors_ok = true;
  in.hip_rad = {1.3F, 1.3F}; in.leg_chain_rad = 1.3F;
  tap_fw::v3::V3Controller::Command cmd{}; cmd.standup = true;
  (void)c.update(in, cmd); cmd.standup = false;
  std::array<std::array<float, 2>, 80> queue{};
  float l = 0, r = 0, angle = 0;
  for (int i = 0; i < 1000; ++i) {
    in.now_us += 2000;
    in.wheel_rad_s = {l, r};
    in.yaw_rate_rad_s = .04F * (r-l) / .309F;
    const auto out = c.update(in, cmd);
    assert(out.state == tap_fw::v3::V3Controller::State::StandUp);
    queue[i % 80] = out.wheel_cmd_rad_s;
    const auto sent = queue[(i + 80 - delay) % 80];
    l += .002F / tau * (sent[0] - l);
    r += .002F / tau * (sent[1] + (i >= 100 ? disturbance : 0) - r);
    angle += in.yaw_rate_rad_s * .002F;
  }
  return std::abs(angle);
}

void yawNoiseFilter() {
  tap_fw::v3::YawRateFilter f;
  float raw_energy = 0, filtered_energy = 0;
  for (int i = 0; i < 2000; ++i) {
    const float gyro = .01F * std::sin(2 * 3.14159265F * 20 * i * .002F);
    const float out = f.update(gyro, .002F, 0);
    if (i > 100) { raw_energy += gyro * gyro; filtered_energy += out * out; }
  }
  assert(filtered_energy < raw_energy * .01F);
  for (float sign : {-1.F, 1.F}) {
    f.reset(); float out = 0;
    for (int i = 0; i < 200; ++i) out = f.update(sign * .3F, .002F, .2F);
    assert(std::abs(out - sign * .3F) < 1e-5F); // deliberate turn retains DC response
  }
  assert(f.update(.5F, .02F, 0) == 0);
  f.reset(); assert(f.update(0, .002F, 0) == 0);
  puts("yaw smoothing: 20 Hz noise energy <1%, turn response and delayed-tick reset passed");
}

int main() {
  yawNoiseFilter();
  signsAndLimits();
  controllerWiring();
  standupYawFeedback();
  float worst_standup = 0;
  for (float tau : {.04F, .09F, .15F}) for (int delay : {0, 25, 55})
    for (float disturbance : {-.8F, .8F}) {
      const float off = standupRatePlant(false, tau, delay, disturbance);
      const float on = standupRatePlant(true, tau, delay, disturbance);
      assert(off > .1F && on < off * .8F);
      worst_standup = std::max(worst_standup, on/off);
    }
  std::printf("18 standup differential plants: worst integrated yaw on/off %.4f\n", worst_standup);
  float worst_ratio=0;
  for(int side:{0,1})for(float drive:{-.3F,.3F})
    for(float tau:{.04F,.09F,.15F})for(int delay:{0,25,55}) {
      const float off=simulate(false,side,drive,tau,delay);
      const float on=simulate(true,side,drive,tau,delay);
      assert(off>.03F);
      assert(on<off*.5F);
      worst_ratio=std::max(worst_ratio,on/off);
    }
  std::printf("36 differential cases: worst final heading on/off ratio %.4f\n",worst_ratio);
  puts("yaw hold: signs, mean preservation, bounded slew/headroom, timeout, stop/turn/safety and 36 delayed differential cases passed");
}
