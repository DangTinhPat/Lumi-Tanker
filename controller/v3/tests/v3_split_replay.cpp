// Compile this same input generator against the preserved and current core.
// Hexadecimal floats retain every output bit, including signed zero.
#include <cmath>
#include <cstdio>
#include "tap_fw/v3/V3Controller.hpp"
using tap_fw::v3::V3Controller;
int main() {
  auto cfg=tap_fw::v3::defaultV3Config();
#ifdef TAP_SPLIT_REPLAY_CURRENT
  cfg.split.enabled=false;
  // Dormant parameters must be unable to affect the original control path.
  cfg.split.posture_tau_s=13; cfg.split.lean_threshold_rad=.0001F;
#endif
  cfg.engage.immediate=true; cfg.roll.enabled=true;
  V3Controller c(cfg);
  V3Controller::Input in{};V3Controller::Command cmd{};
  in.legs_measured=in.legs_ready=true;
  for(int k=0;k<30000;++k) {
    in.now_us=2000ULL*k;
    cmd.start=(k==350 || k==19000);cmd.disable=(k==18000);
    cmd.height_m=k<5000 ? .150F : k<12000 ? .084F : .110F;
    cmd.forward_m_s=k>8000 && k<10000 ? .2F : 0;
    cmd.yaw_rad_s=k>13000 && k<14000 ? .2F : 0;
    const auto p=c.legPose();
    in.hip_rad={p.hip_offset_rad+.001F*std::sin(k*.08F),p.hip_offset_rad};
    in.knee_rad={p.knee_offset_rad,p.knee_offset_rad};
    in.leg_chain_rad=p.hip_offset_rad+p.knee_offset_rad;
    in.hip_rate_rad_s={.08F*std::cos(k*.08F),0};
    in.pitch_rad=.015F*std::sin(k*.003F);
    in.pitch_rate_rad_s=.025F*std::cos(k*.003F);
    in.wheel_rad_s={.3F*std::sin(k*.013F),.3F*std::sin(k*.013F)};
    in.wheel_pos_rad={.02F*std::sin(k*.013F),.02F*std::sin(k*.013F)};
    in.sensors_ok=k!=17000;
    const auto o=c.update(in,cmd);
    std::printf("%d %d %d %d %a %a %a %a %a %a %a %a %a %s\n",k,int(o.state),
      o.legs_locked,o.wheels_enabled,double(o.wheel_cmd_rad_s[0]),double(o.wheel_cmd_rad_s[1]),
      double(o.leg_hip_offset_rad),double(o.leg_knee_offset_rad),double(c.balanceRef()),
      double(c.trunkHip()),double(c.virtualPitch()),double(c.virtualPitchRate()),
      double(c.groundPosition()),o.fault ? o.fault : "-");
    std::printf("flags %d %d\n",o.engage_legs,o.legs_moving);
    for(unsigned side=0;side<2;++side) {
      std::printf("side %u %a %a %a %a\n",side,
        double(o.leg_roll_hip_rad[side]),double(o.leg_roll_knee_rad[side]),
        double(o.leg_roll_hip_rate[side]),double(o.leg_roll_knee_rate[side]));
    }

  }
}
