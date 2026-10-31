// V3Controller adapter. Motor servos execute on the MCU; no MuJoCo dependency.
#include "tap_fw/v3/V3Controller.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <memory>

using tap_fw::v3::V3Controller;
using tap_fw::v3::StandUp;

static V3Controller::Config config() {
  auto c = tap_fw::v3::defaultV3Config();
  c.standup = StandUp::tunedRecoveryRb(.075F);
  c.standup.enabled = true;
  c.standup.hold_at_kneel = false;
  c.standup.catch_wheel_lead_s = .075F;
  c.imu_home_zero = false;
  c.lqr.sampling_period_s = .005F;
  return c;
}

struct Logic {
  V3Controller controller{config()};
  std::array<float,4> previous{}, ff{};
  std::array<std::array<float,2>,10> wheel_delay{};
  unsigned head = 0;
  bool started = false;
  void step(const std::array<double,24>& s, std::array<double,14>& r) {
    // time,pitch,gyroY,gyroZ,accelX,accelZ,q[6],qd[6],hipTorque[2],sensorsOK,ready,torqueValid,reset
    V3Controller::Input in{};
    in.now_us = static_cast<std::uint64_t>(std::llround(s[0]*1e6));
    in.dt_s = .005F;
    in.pitch_rad=s[1]; in.pitch_rate_rad_s=s[2]; in.yaw_rate_rad_s=s[3];
    in.accel_x_m_s2=s[4]; in.accel_z_m_s2=s[5];
    in.legs_measured=in.sensors_ok=s[20]!=0;
    in.legs_ready=s[21]!=0; in.hip_torque_valid=s[22]!=0;
    const float offset=tap_fw::v3::LegHeight::hangHipCad();
    for(unsigned side=0;side<2;++side) {
      in.hip_rad[side]=s[6+side]-offset; in.knee_rad[side]=s[8+side];
      in.hip_rate_rad_s[side]=s[12+side]; in.knee_rate_rad_s[side]=s[14+side];
      in.wheel_rad_s[side]=s[16+side]; in.wheel_pos_rad[side]=s[10+side];
      in.leg_chain_rad+=.5F*(in.hip_rad[side]+in.knee_rad[side]);
      in.hip_torque_nm[side]=s[18+side];
    }
    V3Controller::Command cmd{}; cmd.standup=!started;
    const auto out=controller.update(in,cmd);
    const auto& su=controller.standupState();
    if(out.state==V3Controller::State::Fault) { r[0]=-1; r[1]=static_cast<int>(su.failure); return; }
    const bool standing=out.state==V3Controller::State::StandUp;
    const float hip=(standing?su.hip_target_rad:controller.hipCommand())+offset;
    const float knee=standing?su.knee_target_rad:controller.kneeCommand();
    r[0]=su.phase==StandUp::Phase::Support?1:0;
    r[1]=static_cast<int>(su.phase);
    r[2]=(out.legs_locked?1:0)|(out.wheels_enabled?2:0);
    r[3]=r[4]=hip; r[5]=r[6]=hip+knee;
    for(unsigned i=0;i<4;++i) {
      const float target=i%2==0?hip:knee;
      if(!started)previous[i]=target;
      const float rate=std::clamp((target-previous[i])/.005F,-6.F,6.F);
      if(standing)ff[i]+=.005F/.025F*(rate-ff[i]); else ff[i]=0.F;
      previous[i]=target;
    }
    // Feedforward order on wire: hip L/R, knee L/R.
    r[9]=ff[0]; r[10]=ff[2]; r[11]=ff[1]; r[12]=ff[3];
    for(unsigned side=0;side<2;++side) {
      r[7+side]=out.wheels_enabled?wheel_delay[head][side]:0.F;
      wheel_delay[head][side]=out.wheel_cmd_rad_s[side];
    }
    head=(head+1)%wheel_delay.size();
    if(!out.wheels_enabled)wheel_delay={};
    r[13]=0.; started=true;
  }
};

int main() {
  static_assert(sizeof(double)==8,"binary pipe protocol requires float64");
  auto logic=std::make_unique<Logic>();
  std::array<double,24> request{};
  while(std::fread(request.data(),sizeof(double),request.size(),stdin)==request.size()) {
    std::array<double,14> reply{};
    try {
      bool valid=true; for(auto v:request)valid=valid&&std::isfinite(v);
      if(!valid)reply[0]=-1;
      else if(request[23]!=0)logic=std::make_unique<Logic>();
      else logic->step(request,reply);
    } catch(...) { reply[0]=-1; }
    if(std::fwrite(reply.data(),sizeof(double),reply.size(),stdout)!=reply.size() || std::fflush(stdout))return 2;
  }
  return std::ferror(stdin)?1:0;
}
