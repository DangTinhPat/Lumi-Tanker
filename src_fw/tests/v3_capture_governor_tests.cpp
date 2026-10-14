#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include "tap_fw/v3/BalanceCaptureGovernor.hpp"
using G=tap_fw::v3::BalanceCaptureGovernor;

void measuredHip(G::Input& in, float hip) {
  (void)in;
  (void)hip;
}

// Synthetic input/output contracts, not CAD or wheel-actuator validation.
void delayedBodyResponseConvergesWithoutBlindPoseTransfer() {
  for(float lag:{0.05F,0.15F,0.25F}) for(float bias:{-0.35F,-0.20F,0.20F}) {
    G g;g.begin(0.65F,G::Config{});
    G::Input in{};float body=bias,old_hip=0.65F;
    G::Output out{};
    for(int i=0;i<16000;++i) {
      in.dt_s=i%2 ? 0.0018F : 0.0022F;
      const float previous=body;
      body+=(bias-1.4F*(old_hip-0.65F)-body)*in.dt_s/(lag+in.dt_s);
      in.body_pitch_rad=body;in.body_rate_rad_s=(body-previous)/in.dt_s;
      in.virtual_rate_rad_s=in.body_rate_rad_s;in.virtual_error_rad=0.01F*in.body_rate_rad_s;
      measuredHip(in,old_hip);
      out=g.update(in);
      assert(std::abs(out.hip_rad-old_hip)/in.dt_s<=0.0401F);
      assert(std::abs(out.hip_rad-0.65F)<=0.3501F);
      old_hip=out.hip_rad;
      assert(out.phase!=G::Phase::Failed);
      if(out.phase==G::Phase::Normal)break;
    }
    assert(out.phase==G::Phase::Normal && std::abs(body)<=0.0801F);
    const float pose=out.hip_rad;
    in.body_pitch_rad=0.5F;in.body_rate_rad_s=1.0F;
    measuredHip(in,pose);
    for (int i=0;i<1000;++i) assert(g.update(in).hip_rad==pose); // static Normal does not chase gyro
    assert(g.phase()==G::Phase::Normal);
  }
}

void oscillationAndTranslationDoNotQualify() {
  for(int kind:{0,1,2}) {
    G g;G::Config cfg;cfg.timeout_s=4.0F;g.begin(0.6F,cfg);
    G::Input in{};in.body_pitch_rad=-0.3F;
    G::Output out{};
    for(int i=0;i<2100;++i) {
      measuredHip(in,0.6F);
      if(kind==0)in.body_rate_rad_s=i%140<90 ? 0.0F : 1.0F;
      if(kind==1)in.ground_speed_m_s=0.47F;
      if(kind==2)in.command_speed_m_s=0.6F;
      out=g.update(in);assert(out.hip_rad==0.6F);
    }
    assert(out.phase==G::Phase::Failed && out.failure==G::Failure::Timeout);
    g.begin(0.6F,cfg);in={}; // retry is a fresh episode
    measuredHip(in,0.6F);
    for(int i=0;i<1600;++i)out=g.update(in);
    assert(out.phase==G::Phase::Normal);
  }
}

void wrongOrUnreachableResponseFailsBoundedly() {
  for(bool wrong:{false,true}) {
    G g;G::Config cfg;cfg.travel_max_rad=0.04F;g.begin(0.6F,cfg);
    G::Input in{};G::Output out{};float hip=0.6F;
    for(int i=0;i<16000;++i) {
      in.body_pitch_rad=-0.3F+(wrong ? 1.4F*(hip-0.6F) : 0.0F);
      measuredHip(in,hip);
      out=g.update(in);hip=out.hip_rad;
      assert(std::abs(hip-0.6F)<=0.0401F);
      if(out.phase==G::Phase::Failed)break;
    }
    assert(out.phase==G::Phase::Failed);
    assert(out.failure==(wrong ? G::Failure::Response : G::Failure::Travel));
  }
  G g;g.begin(0.6F,G::Config{});G::Input in{};
  in.body_rate_rad_s=std::numeric_limits<float>::quiet_NaN();
  assert(g.update(in).failure==G::Failure::Invalid);
  g.reset();assert(g.phase()==G::Phase::Inactive);
}

void observedQuietIsNotSpentTwice() {
  // The robot spent about 10 s arresting, then timed out while making healthy
  // posture progress. Retain step/rate/energy limits; only observed progress
  // may renew the 30 s deadline, up to the explicitly bounded 45 s ceiling.
  G g; g.begin(0.5786F, G::Config{});
  G::Input in{}; G::Output out{};
  float hip = 0.5786F, body = -0.327F, elapsed = 0.0F;
  for (int i = 0; i < 16000; ++i) {
    elapsed = i * in.dt_s;
    const float previous = body;
    body += (-0.327F - 0.9F * (hip - 0.5786F) - body) * in.dt_s / (0.15F + in.dt_s);
    in.body_pitch_rad = body;
    in.body_rate_rad_s = elapsed < 10.0F
        ? std::sin(2.0F * 3.14159265F * 2.6F * elapsed) : (body - previous) / in.dt_s;
    in.virtual_rate_rad_s = in.body_rate_rad_s;
    measuredHip(in,hip);
    out = g.update(in);
    assert(std::abs(out.hip_rad - hip) / in.dt_s <= 0.0401F);
    if (elapsed < 10.0F) assert(out.hip_rad == 0.5786F);
    hip = out.hip_rad;
    if (out.phase == G::Phase::Normal || out.phase == G::Phase::Failed) break;
  }
  std::printf("arrest then observed progress: duration=%.3f body=%.4f phase=%d\n",
      elapsed, body, int(out.phase));
  assert(out.phase == G::Phase::Normal && elapsed < 33.0F && std::abs(body) <= 0.0801F);
}

void progressRenewalRequiresImprovementAndHasAHardCeiling() {
  for (bool improves : {false, true}) {
    G g; G::Config cfg;
    cfg.timeout_s = 4.0F; cfg.progress_extension_max_s = 3.0F;
    cfg.travel_max_rad = 1.0F;
    g.begin(0.6F, cfg);
    G::Input in{}; G::Output out{}; float hip = 0.6F, elapsed = 0.0F;
    for (int i = 0; i < 4100; ++i) {
      elapsed = i * in.dt_s;
      in.body_pitch_rad = -0.6F - (improves ? 1.4F * (hip - 0.6F) : 0.0F);
      // Small pitch noise is not an observed 0.01 rad improvement.
      if (!improves) in.body_pitch_rad += 0.002F * std::sin(elapsed * 8.0F);
      measuredHip(in,hip);
      out = g.update(in); hip = out.hip_rad;
      if (out.phase == G::Phase::Failed) break;
    }
    assert(out.phase == G::Phase::Failed && out.failure == G::Failure::Timeout);
    assert(elapsed <= (improves ? 7.01F : 4.01F));
    if (improves) assert(elapsed > 4.01F);
  }
}


int main() {
  delayedBodyResponseConvergesWithoutBlindPoseTransfer();
  oscillationAndTranslationDoNotQualify();
  wrongOrUnreachableResponseFailsBoundedly();
  observedQuietIsNotSpentTwice();
  progressRenewalRequiresImprovementAndHasAHardCeiling();
  std::puts("capture governor contracts passed");
}
