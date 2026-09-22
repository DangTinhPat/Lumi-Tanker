#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include "robot_control/JumpController.hpp"
#include "robot_control/WholeBodyModel.hpp"
using namespace robot_control;
namespace {
int checks=0,failures=0;
void check(bool ok,const char* name) {++checks;if(!ok){++failures;std::cerr<<"FAIL "<<name<<'\n';}}
JumpController::Input input() {
  JumpController::Input i;i.eligible=i.sensors_fresh=i.contacts_fresh=i.left_contact=i.right_contact=true;
  i.position_rad=JumpController::neutralPose(0);i.com_z_m=.2173;return i;
}
JumpController::Config config() {
  JumpController::Config c;c.enabled=true;c.crouch_duration_s=.2F;
  c.thrust_duration_s=.15F;c.landing_duration_s=.1F;c.settle_duration_s=.1F;c.max_settle_s=1;
  return c;
}
void airborne(JumpController::Input& i) {
  i.left_contact=i.right_contact=false;
  i.airborne=true;
  i.ballistic=true;
}
void grounded(JumpController::Input& i) {
  i.left_contact=i.right_contact=true;
  i.airborne=false;
  i.ballistic=false;
}
void ready(JumpController& j,JumpController::Input& i) {
  i.request=false;for(int n=0;n<25;++n)(void)j.update(i);
}
JumpController::Output thrust(JumpController& j,JumpController::Input& i) {
  ready(j,i);i.request=true;auto o=j.update(i);i.request=false;
  for(int n=0;n<150 && o.phase==JumpController::Phase::Crouch;++n){i.position_rad=o.position_rad;o=j.update(i);}
  return o;
}
}
int main() {
  {
    auto c=config();c.smooth_phase_transitions=true;c.flight_hip_rate_kp=1.F;
    bool rejected=false;
    try {JumpController invalid(c);} catch(const std::invalid_argument&) {rejected=true;}
    check(rejected,"reject conflicting flight reference ownership");
    c=config();c.landing_compression=true;c.compression_duration_s=.5F;c.max_settle_s=.55F;
    rejected=false;
    try {JumpController invalid(c);} catch(const std::invalid_argument&) {rejected=true;}
    check(rejected,"compression must leave time for settling before timeout");
  }
  {
    auto c=config();c.thrust_world_frame=true;c.thrust_force_n=10.F;
    c.body_pitch_kp=c.body_pitch_kd=0.F;
    JumpController world(c);auto state=input();(void)thrust(world,state);
    for(float pitch:{-.2F,0.F,.2F}) {
      state.pitch_rad=pitch;const auto out=world.update(state);
      for(int side=0;side<2;++side) {
        const int h=3*side,k=h+1;
        const auto& thigh=geometry::origins[h+1];const auto& shin=geometry::origins[h+2];
        auto z=[&](double hip,double knee) {
          const double a=hip+pitch,
              b=a+knee+geometry::knee_zero_rebase_rad;
          return -std::sin(a)*thigh.x+std::cos(a)*thigh.z-
              std::sin(b)*shin.x+std::cos(b)*shin.z;
        };
        constexpr double eps=1e-5;
        const double hip=state.position_rad[h],knee=state.position_rad[k];
        const double jh=(z(hip+eps,knee)-z(hip-eps,knee))/(2*eps);
        const double jk=(z(hip,knee+eps)-z(hip,knee-eps))/(2*eps);
        check(std::abs(out.feedforward_nm[h]+5*jh)<1e-5&&
              std::abs(out.feedforward_nm[k]+5*jk)<1e-5,
              "world vertical thrust agrees with finite-difference foot Jacobian on each native leg");
      }
    }
  }
  {
    auto c=config();c.smooth_phase_transitions=true;c.landing_compression=true;
    JumpController smooth_jump(c);auto state=input();auto out=thrust(smooth_jump,state);
    airborne(state);
    bool continuous=true,bounded=true;bool flight=false,land=false;
    for(int n=0;n<300&&out.phase!=JumpController::Phase::Complete;++n) {
      const auto previous=out;
      // A measurement jump at the contact transition must not reset the reference.
      state.position_rad=out.position_rad;
      if(n==30) {grounded(state);state.position_rad[1]-=.04F;}
      out=smooth_jump.update(state);
      if(out.phase!=previous.phase&&(out.phase==JumpController::Phase::Flight||out.phase==JumpController::Phase::Land)) {
        continuous=continuous&&out.position_rad==previous.position_rad;
        continuous=continuous&&out.velocity_rad_s==previous.velocity_rad_s;
        flight=flight||out.phase==JumpController::Phase::Flight;
        land=land||out.phase==JumpController::Phase::Land;
      }
      for(auto k:{0,1,3,4}) bounded=bounded&&std::isfinite(out.position_rad[k])&&
          std::abs(out.position_rad[k]-previous.position_rad[k])<=c.max_joint_speed_rad_s*state.dt_s+1e-5F;
    }
    check(continuous&&flight&&land,"smooth contact transitions preserve last commanded position");
    check(bounded&&out.phase==JumpController::Phase::Complete,"bounded Hermite trajectory reaches stable completion");
    state.stop=true;smooth_jump.reset();state=input();out=thrust(smooth_jump,state);
    state.stop=true;out=smooth_jump.update(state);
    check(out.stop_motors&&out.velocity_rad_s[1]==0,"smooth trajectory respects stop after reset");
  }
  {
    auto c=config();c.landing_compression=true;
    JumpController absorb(c);auto state=input();auto out=thrust(absorb,state);
    airborne(state);
    for(int n=0;n<25;++n) {state.position_rad=out.position_rad;out=absorb.update(state);}
    grounded(state);
    for(int n=0;n<5&&out.phase!=JumpController::Phase::Land;++n) {
      state.position_rad=out.position_rad;out=absorb.update(state);
    }
    check(out.phase==JumpController::Phase::Land,"compression requires measured touchdown");
    const float impact_knee=out.position_rad[1];
    bool compressed=false,soft=true,bounded=true;
    for(int n=0;n<400&&out.phase!=JumpController::Phase::Complete;++n) {
      const auto previous=out.position_rad;
      state.position_rad=out.position_rad;out=absorb.update(state);
      if(out.phase==JumpController::Phase::Land&&out.phase_elapsed_s<c.compression_duration_s)
        soft=soft&&out.leg_kp_scale==.3F&&out.station_blend==0;
      compressed=compressed||out.position_rad[1]<impact_knee-.1F;
      for(auto k:{0,1,3,4}) bounded=bounded&&std::abs(out.position_rad[k]-previous[k])<=
          c.max_joint_speed_rad_s*state.dt_s+1e-5F;
    }
    check(compressed&&soft&&bounded,"compression bends softly with bounded target speed");
    check(out.phase==JumpController::Phase::Complete,"compression returns through stable recovery");
    absorb.reset();state=input();out=thrust(absorb,state);
    airborne(state);
    for(int n=0;n<5;++n)out=absorb.update(state);
    grounded(state);
    for(int n=0;n<5;++n)out=absorb.update(state);
    state.stop=true;out=absorb.update(state);
    check(out.stop_motors&&out.velocity_rad_s[1]==0&&!out.owns_pose,
          "compression never overrides safety stop");
    c.compression_knee_rad=c.landing_knee_rad;bool rejected=false;
    try {JumpController invalid(c);} catch(const std::invalid_argument&) {rejected=true;}
    check(rejected,"compression must leave actual bending stroke");
  }
  {
    for(float pitch:{-.3F,0.F,.3F}) for(float knee:{-.8F,-.4F,0.F}) {
      const auto pose=JumpController::landingPose(knee,pitch);
      const auto body=WholeBodyModel{}.compute(pose[0],knee,pose[3],knee);
      check(std::abs(std::cos(pitch)*body.com_offset_x_m+
            std::sin(pitch)*body.com_above_axle_m)<1e-5F,
            "landing geometry aligns world COM and wheel axle");
      check(pose[0]>=-.6F&&pose[0]<=1.2F&&pose[0]==pose[3],
            "landing geometry preserves native symmetric travel");
    }
  }
  {
    auto c=config();c.landing_world_frame=true;c.thrust_hip_offset_rad=.1F;
    JumpController adjusted(c);auto state=input();auto out=thrust(adjusted,state);
    bool bounded=true;
    for(int n=0;n<20;++n) {
      const auto previous=out.position_rad;
      state.position_rad=out.position_rad;out=adjusted.update(state);
      for(auto h:{0,3}) bounded=bounded&&out.position_rad[h]>=-.6F&&out.position_rad[h]<=1.2F&&
          std::abs(out.position_rad[h]-previous[h])<=c.max_joint_speed_rad_s*state.dt_s+1e-5F;
    }
    check(bounded,"thrust attitude offset stays inside position and speed bounds");
    airborne(state);state.pitch_rad=-.15F;
    for(int n=0;n<30;++n) {state.position_rad=out.position_rad;out=adjusted.update(state);}
    check(out.phase==JumpController::Phase::Flight&&out.position_rad[0]>
          JumpController::neutralPose(out.position_rad[1])[0],
          "world landing compensates negative body pitch with native hip direction");
    state.stop=true;out=adjusted.update(state);
    check(out.stop_motors&&!out.owns_pose&&out.velocity_rad_s[0]==0,
          "world landing and thrust correction obey immediate stop");
    for(float bad:{.151F,std::numeric_limits<float>::quiet_NaN()}) {
      c.thrust_hip_offset_rad=bad;bool rejected=false;
      try {JumpController invalid(c);} catch(const std::invalid_argument&) {rejected=true;}
      check(rejected,"reject nonfinite or excessive thrust correction");
    }
  }
  {
    auto c=config();c.thrust_wheel_control=true;c.thrust_wheel_speed_m_s=-.15F;
    JumpController wheel(c);auto state=input();auto out=thrust(wheel,state);
    check(out.override_wheel_speed&&std::abs(out.wheel_speed_rad_s+3.75F)<1e-5F,
          "thrust wheel reference uses native absolute rolling rate");
    airborne(state);
    for(int n=0;n<5;++n)out=wheel.update(state);
    check(out.phase==JumpController::Phase::Flight&&!out.override_wheel_speed&&out.wheel_speed_rad_s==0,
          "flight releases thrust wheel override");
    state.stop=true;out=wheel.update(state);
    check(out.stop_motors&&!out.override_wheel_speed&&out.wheel_speed_rad_s==0,
          "thrust wheel override cannot survive stop");
    c.thrust_wheel_speed_m_s=.351F;bool rejected=false;
    try {JumpController invalid(c);} catch(const std::invalid_argument&) {rejected=true;}
    check(rejected,"thrust wheel speed bounded independently of torque cap");
  }
  {
    auto c=config();c.extension_knee_rad=-.5F;c.landing_knee_rad=-.8F;
    JumpController early(c);auto state=input();auto out=thrust(early,state);
    check(out.phase==JumpController::Phase::Thrust&&!out.airborne_seen,
          "bent-knee extension endpoint does not infer takeoff");
    c.landing_knee_rad=-.4F;
    JumpController open_landing(c);
    check(open_landing.phase()==JumpController::Phase::Idle,
          "landing may be more extended than the release pose");
    c.extension_knee_rad=c.crouch_knee_rad;
    bool rejected=false;
    try {JumpController invalid(c);} catch(const std::invalid_argument&) {rejected=true;}
    check(rejected,"extension must exceed crouch knee angle");
  }
  {
    auto c=config();c.thrust_knee_force_control=true;c.require_bilateral_thrust=false;
    JumpController force(c);auto state=input();auto out=thrust(force,state);
    state.velocity_rad_s[1]=3.F;state.velocity_rad_s[4]=4.F;
    out=force.update(state);
    check(out.phase==JumpController::Phase::Thrust&&out.knee_kp_scale[0]==0&&
          out.knee_kp_scale[1]==0,"force thrust removes knee position spring only");
    check(out.velocity_rad_s[1]==c.max_joint_speed_rad_s&&
          out.velocity_rad_s[4]==c.max_joint_speed_rad_s,
          "force thrust requests bounded extension speed despite sensor lag");
    check(out.feedforward_nm[1]>0&&out.feedforward_nm[1]<=5.F,
          "force thrust retains real motor torque limit");
    state.position_rad[1]=c.extension_knee_rad;
    out=force.update(state);
    check(out.velocity_rad_s[1]==0&&out.velocity_rad_s[4]==c.max_joint_speed_rad_s,
          "each knee stops extension at its endpoint");
    check(out.feedforward_nm[1]==0,"released knee has no competing thrust feedforward");
    state.position_rad[1]-=.05F;out=force.update(state);
    check(out.velocity_rad_s[1]==0&&out.feedforward_nm[1]==0,
          "released knee cannot retrigger thrust after recoil");
    state.left_contact=false;state.ballistic=false;state.airborne=false;out=force.update(state);
    check(out.knee_kp_scale[0]==1&&out.knee_kp_scale[1]==0,
          "force thrust requires each foot contact separately");
    state.right_contact=false;state.ballistic=true;state.airborne=true;
    for(int n=0;n<4;++n)out=force.update(state);
    check(out.phase==JumpController::Phase::Flight&&out.knee_kp_scale[0]==1&&
          out.knee_kp_scale[1]==1,"flight restores knee impedance");
    state.stop=true;out=force.update(state);
    check(out.stop_motors&&out.velocity_rad_s[1]==0&&out.feedforward_nm[1]==0,
          "force thrust never survives safety stop");
    force.reset();state=input();out=thrust(force,state);
    check(out.velocity_rad_s[1]==c.max_joint_speed_rad_s&&out.feedforward_nm[1]>0,
          "explicit reset clears completed thrust latch");
  }
  auto i=input();JumpController disabled;ready(disabled,i);i.request=true;
  check(!disabled.update(i).owns_pose,"default opt-out unchanged");
  {
    auto deep_config=config();
    JumpController deep(deep_config);auto deep_input=input();
    deep_input.position_rad=JumpController::neutralPose(-1.2F);
    ready(deep,deep_input);deep_input.request=true;
    auto deep_out=deep.update(deep_input);
    check(deep_out.phase==JumpController::Phase::Crouch&&deep_out.owns_pose,
          "grounded deep launch stance is admitted from encoder observation");
  }
  JumpController j(config());i=input();i.request=true;
  check(!j.update(i).owns_pose,"cannot jump without stable grounded dwell");
  ready(j,i);
  check(j.update(i).ready,"ready telemetry matches grounded neutral eligibility");
  i.position_rad[1]=-.5F;i.request=true;
  check(!j.update(i).owns_pose,"nonneutral request rejected in core");
  j.reset();i=input();ready(j,i);i.request=true;auto o=j.update(i);
  check(o.phase==JumpController::Phase::Crouch,"ready request begins crouch");
  check(!o.ready,"active jump cannot advertise ready for another request");
  i.request=false;i.left_contact=false;o=j.update(i);
  check(o.phase==JumpController::Phase::Crouch,"single short contact flicker debounced");
  i.left_contact=true;
  bool neutral=true,speed=true;
  for(int n=0;n<120&&o.phase==JumpController::Phase::Crouch;++n) {
    const auto prev=o.position_rad;i.position_rad=prev;o=j.update(i);
    const auto w=WholeBodyModel{}.compute(o.position_rad[0],o.position_rad[1],o.position_rad[3],o.position_rad[4]);
    neutral=neutral&&std::abs(w.com_offset_x_m)<1e-5F;
    for(auto k:{0,1,3,4}) speed=speed&&std::abs(o.position_rad[k]-prev[k])<=config().max_joint_speed_rad_s*i.dt_s+1e-5F;
  }
  check(neutral&&speed,"crouch path COM-neutral and speed-bounded");
  check(o.phase==JumpController::Phase::Thrust,"measured crouch tracking gates thrust");
  i.position_rad=o.position_rad;o=j.update(i);
  bool capped=true;for(float tau:o.feedforward_nm)capped=capped&&std::isfinite(tau)&&std::abs(tau)<=5.F;
  check(capped&&o.feedforward_nm[1]>0,"Jacobian thrust positive knee and at most nominal torque");
  for(int n=0;n<3;++n){airborne(i);i.com_z_m=.25;i.position_rad=o.position_rad;o=j.update(i);}
  check(o.phase==JumpController::Phase::Flight&&o.airborne_seen&&!o.balance_wheels,"flight requires measured bilateral liftoff");
  i.com_z_m=.36;o=j.update(i);
  check(o.apex_gain_m>.1,"apex records input COM relative last grounded sample");
  grounded(i);
  for(int n=0;n<500&&o.phase!=JumpController::Phase::Complete;++n) {
    i.position_rad=o.position_rad;o=j.update(i);
  }
  check(o.phase==JumpController::Phase::Complete&&!o.owns_pose,"measured landing and settled pose finish");
  // Held requests cannot restart an already consumed edge.
  JumpController held(config());i=input();ready(held,i);i.request=true;o=held.update(i);
  i.contacts_fresh=false;o=held.update(i);
  check(o.stop_motors&&o.phase==JumpController::Phase::Fault,"stale contact immediately aborts active jump");
  check(!o.ready,"fault cannot advertise ready");
  i.contacts_fresh=true;
  for(int n=0;n<50;++n)o=held.update(i);
  check(o.stop_motors&&!o.owns_pose,"fault latched despite held request and fresh recovery");
  JumpController noflight(config());i=input();o=thrust(noflight,i);
  for(int n=0;n<150&&!o.stop_motors;++n){i.position_rad=o.position_rad;o=noflight.update(i);}
  check(o.stop_motors&&!o.airborne_seen,"no contact loss cannot fabricate successful takeoff");
  for(int which=0;which<4;++which){
    JumpController stop(config());i=input();o=thrust(stop,i);
    if(which==0)i.stop=true;
    if(which==1)i.operator_driving=true;
    if(which==2)i.pitch_rad=.6F;
    if(which==3)i.position_rad[0]=std::numeric_limits<float>::quiet_NaN();
    o=stop.update(i);check(o.stop_motors,"stop/drive/pitch/NaN safety interrupts thrust");
  }
  for(bool wheel:{false,true}) {
    auto c=config();if(wheel)c.max_wheel_torque_nm=.14F;else c.max_leg_torque_nm=11;
    bool threw=false;try{JumpController bad(c);}catch(const std::invalid_argument&){threw=true;}
    check(threw,"constructor rejects exceeding nominal actuator torque");
  }
  for(float max_speed:{12.5663706F,28.274334F}) {
    auto c=config();c.max_joint_speed_rad_s=max_speed;
    JumpController trajectory(c);i=input();ready(trajectory,i);
    auto previous=i.position_rad;i.request=true;
    bool coherent=true,bounded=true,finite=true,saw_flight=false,saw_land=false;
    JumpController::Output sample;
    for(int n=0;n<500;++n) {
      sample=trajectory.update(i);i.request=false;
      for(auto k:{0,1,3,4}) {
        finite=finite&&std::isfinite(sample.velocity_rad_s[k]);
        bounded=bounded&&std::abs(sample.velocity_rad_s[k])<=max_speed+1e-4F;
        if(sample.owns_pose) coherent=coherent&&std::abs(sample.velocity_rad_s[k]-
            (sample.position_rad[k]-previous[k])/i.dt_s)<1e-5F;
        else coherent=coherent&&sample.velocity_rad_s[k]==0;
      }
      previous=sample.position_rad;i.position_rad=sample.position_rad;
      if(sample.phase==JumpController::Phase::Thrust && sample.phase_elapsed_s>.03F)
        airborne(i);
      if(sample.phase==JumpController::Phase::Flight) {
        saw_flight=true;
        if(sample.phase_elapsed_s>.04F)grounded(i);
      }
      saw_land=saw_land||sample.phase==JumpController::Phase::Land;
      if(sample.phase==JumpController::Phase::Complete)break;
    }
    check(coherent&&bounded&&finite&&saw_flight&&saw_land,
          "target finite-difference velocity exact and bounded across debounced phase handoffs");
    check(sample.phase==JumpController::Phase::Complete,"velocity-reference test completes measured-contact cycle");
    trajectory.reset();i=input();sample=thrust(trajectory,i);i.stop=true;sample=trajectory.update(i);
    bool zero=true;for(float v:sample.velocity_rad_s)zero=zero&&v==0;
    check(zero&&sample.stop_motors,"stop returns zero velocity references");
  }
  {
    auto c=config();c.body_pitch_kp=4;c.body_pitch_kd=.4F;
    c.hip_attitude_torque_limit_nm=1;
    JumpController attitude(c);i=input();ready(attitude,i);i.request=true;
    o=attitude.update(i);i.request=false;i.pitch_rad=.1F;i.pitch_rate_rad_s=.2F;
    o=attitude.update(i);
    bool zero=true;for(float t:o.feedforward_nm)zero=zero&&t==0;
    check(o.phase==JumpController::Phase::Crouch&&zero,"no attitude torque in crouch");
    attitude.reset();i=input();o=thrust(attitude,i);
    // Compare identical phase states to isolate attitude torque from thrust FF.
    auto base_controller=attitude;auto base=base_controller.update(i);
    for(float sign:{-1.F,1.F}) {
      auto trial=attitude;auto sample=i;sample.pitch_rad=sign*.1F;
      sample.pitch_rate_rad_s=sign*.2F;auto result=trial.update(sample);
      const float expected=sign*.24F;
      check(result.phase==JumpController::Phase::Thrust&&
            std::abs(result.feedforward_nm[0]-base.feedforward_nm[0]-expected)<1e-5F&&
            std::abs(result.feedforward_nm[3]-base.feedforward_nm[3]-expected)<1e-5F,
            "thrust symmetric hip reaction opposes either body pitch sign");
      check(result.feedforward_nm[1]==base.feedforward_nm[1]&&
            result.feedforward_nm[4]==base.feedforward_nm[4],"attitude leaves knee thrust unchanged");
    }
    airborne(i);
    for(int n=0;n<5&&o.phase!=JumpController::Phase::Flight;++n)o=attitude.update(i);
    check(o.phase==JumpController::Phase::Flight,"attitude test enters measured flight");
    for(float sign:{-1.F,1.F}) {
      auto trial=attitude;auto sample=i;sample.pitch_rate_rad_s=sign*1.6F;
      auto result=trial.update(sample);
      check(std::abs(result.feedforward_nm[0]-sign*.32F)<1e-5F&&
            result.feedforward_nm[0]==result.feedforward_nm[3],"flight rate damping sign and symmetry");
      sample.pitch_rate_rad_s=sign*20;result=trial.update(sample);
      check(result.feedforward_nm[0]==sign&&result.feedforward_nm[3]==sign,
            "flight attitude saturates at one Nm per hip");
      sample.stop=true;result=trial.update(sample);
      zero=true;for(float t:result.feedforward_nm)zero=zero&&t==0;
      check(result.stop_motors&&zero,"fault clears attitude torque");
    }
    grounded(i);i.pitch_rad=.1F;
    for(int n=0;n<5&&o.phase!=JumpController::Phase::Land;++n)o=attitude.update(i);
    check(o.phase==JumpController::Phase::Land&&std::abs(o.feedforward_nm[0]-.2F)<1e-5F&&
          o.feedforward_nm[0]==o.feedforward_nm[3],"landing retains symmetric attitude correction");
    for(int n=0;n<100&&o.phase==JumpController::Phase::Land;++n) {
      i.position_rad=o.position_rad;o=attitude.update(i);
    }
    zero=true;for(float t:o.feedforward_nm)zero=zero&&t==0;
    check(o.phase==JumpController::Phase::Settle&&zero,"settle clears attitude correction");
    c.hip_attitude_torque_limit_nm=5;JumpController capped_attitude(c);
    i=input();o=thrust(capped_attitude,i);i.pitch_rate_rad_s=100;
    o=capped_attitude.update(i);
    bool bounded=true;for(float t:o.feedforward_nm)bounded=bounded&&std::abs(t)<=5;
    check(o.phase==JumpController::Phase::Thrust&&bounded&&
          std::abs(o.feedforward_nm[0])<=5,
          "combined thrust and attitude respects nominal five Nm cap");
  }
  {
    check(config().flight_hip_rate_kp==0&&config().flight_hip_rate_kd==0,
          "flight hip integration defaults opt-out");
    auto enter_flight=[](JumpController& controller,JumpController::Input& sample) {
      sample=input();auto result=thrust(controller,sample);
      airborne(sample);
      for(int n=0;n<5&&result.phase!=JumpController::Phase::Flight;++n) {
        sample.position_rad=result.position_rad;result=controller.update(sample);
      }
      return result;
    };
    for(bool derivative:{false,true}) {
      auto c=config();c.flight_hip_rate_kp=derivative?0:2;
      c.flight_hip_rate_kd=derivative?2:0;
      JumpController controller(c);auto sample=input();auto initial=enter_flight(controller,sample);
      for(float sign:{-1.F,1.F}) {
        auto trial=controller;auto s=sample;s.pitch_rad=derivative?0:sign*.1F;
        s.pitch_rate_rad_s=derivative?sign*.1F:0;auto result=trial.update(s);
        check(result.phase==JumpController::Phase::Flight&&
              std::abs(result.position_rad[0]-initial.position_rad[0]-sign*.2F*s.dt_s)<1e-6F&&
              std::abs(result.velocity_rad_s[0]-sign*.2F)<1e-4F&&
              result.position_rad[0]==result.position_rad[3]&&
              result.velocity_rad_s[0]==result.velocity_rad_s[3],
              "flight hip P/D integration signed and symmetric");
      }
    }
    auto c=config();c.flight_hip_rate_kd=100;
    for(float sign:{-1.F,1.F}) {
      JumpController controller(c);auto sample=input();auto result=enter_flight(controller,sample);
      bool bounded=true;
      sample.pitch_rate_rad_s=sign*10;
      for(int n=0;n<60;++n) {
        const auto previous=result.position_rad;result=controller.update(sample);
        for(auto h:{0,3}) bounded=bounded&&std::isfinite(result.velocity_rad_s[h])&&
            result.position_rad[h]>=-.6F&&result.position_rad[h]<=1.2F&&
            std::abs(result.velocity_rad_s[h])<=c.max_joint_speed_rad_s+1e-4F&&
            std::abs(result.velocity_rad_s[h]-(result.position_rad[h]-previous[h])/sample.dt_s)<1e-5F;
        sample.position_rad=result.position_rad;
      }
      check(result.phase==JumpController::Phase::Flight&&bounded&&
            std::abs(result.position_rad[0]-(sign>0?1.2F:-.6F))<1e-6F,
            "flight integrated hip reaches position bound with finite speed-limited references");
      sample.stop=true;result=controller.update(sample);
      bool zero=true;for(float v:result.velocity_rad_s)zero=zero&&v==0;
      check(result.stop_motors&&!result.owns_pose&&zero,"stop clears integrated flight velocity");
    }
    JumpController zero_gain(config());auto sample=input();auto result=enter_flight(zero_gain,sample);
    const auto tuck=JumpController::neutralPose(config().flight_tuck_knee_rad);
    sample.pitch_rad=.1F;sample.pitch_rate_rad_s=.2F;
    bool tuck_seen=false,symmetric=true,bounded=true;
    for(int n=0;n<25;++n) {
      const auto previous=result.position_rad;result=zero_gain.update(sample);
      tuck_seen=tuck_seen||std::abs(result.position_rad[1]-tuck[1])<.08F;
      symmetric=symmetric&&result.position_rad[0]==result.position_rad[3]&&
          result.position_rad[1]==result.position_rad[4];
      for(auto j:{0,1,3,4}) bounded=bounded&&
          std::abs(result.position_rad[j]-previous[j])<=config().max_joint_speed_rad_s*sample.dt_s+1e-5F;
      sample.position_rad=result.position_rad;
    }
    check(tuck_seen&&symmetric&&bounded&&result.phase==JumpController::Phase::Flight,
          "flight trajectory retracts symmetrically into a bounded airborne tuck");
    JumpController timed(config());auto timed_sample=input();
    auto timed_out=enter_flight(timed,timed_sample);
    bool prelanding=false,extension_seen=false;
    const auto timed_tuck=JumpController::neutralPose(config().flight_tuck_knee_rad);
    for(int n=0;n<90&&timed_out.phase==JumpController::Phase::Flight;++n) {
      timed_sample.position_rad=timed_out.position_rad;
      timed_out=timed.update(timed_sample);
      prelanding=prelanding||timed_out.prelanding_active;
      extension_seen=extension_seen||
          (timed_out.prelanding_active&&timed_out.position_rad[1]>timed_tuck[1]+.05F);
    }
    check(prelanding&&extension_seen&&timed_out.target_apex_height_m>=.20F,
          "flight reference actively extends before the 20 cm ballistic touchdown window");
    for(bool derivative:{false,true}) {
      auto invalid=config();
      if(derivative)invalid.flight_hip_rate_kd=std::numeric_limits<float>::quiet_NaN();
      else invalid.flight_hip_rate_kp=std::numeric_limits<float>::quiet_NaN();
      bool threw=false;try{JumpController rejected(invalid);}catch(const std::invalid_argument&){threw=true;}
      check(threw,"constructor rejects NaN flight hip gain");
    }
  }
  {
    check(!config().thrust_track_measured_knee,"measured-knee thrust coordination defaults off");
    auto tracked_config=config();tracked_config.thrust_track_measured_knee=true;
    auto legacy_config=config();legacy_config.thrust_track_measured_knee=false;
    JumpController tracked(tracked_config),legacy(legacy_config),defaults(config());
    auto tracked_input=input(),legacy_input=input(),default_input=input();
    auto measured=thrust(tracked,tracked_input),timed=thrust(legacy,legacy_input);
    auto default_output=thrust(defaults,default_input);
    bool unchanged=true,knees_unchanged=true,holds_measured=true;
    const auto actual=JumpController::neutralPose(tracked_config.crouch_knee_rad);
    tracked_input.position_rad=legacy_input.position_rad=default_input.position_rad=actual;
    for(int n=0;n<20;++n) {
      measured=tracked.update(tracked_input);timed=legacy.update(legacy_input);
      default_output=defaults.update(default_input);
      unchanged=unchanged&&default_output.phase==timed.phase&&
          default_output.position_rad==timed.position_rad&&
          default_output.velocity_rad_s==timed.velocity_rad_s&&
          default_output.feedforward_nm==timed.feedforward_nm;
      knees_unchanged=knees_unchanged&&measured.position_rad[1]==timed.position_rad[1]&&
          measured.position_rad[4]==timed.position_rad[4];
      holds_measured=holds_measured&&std::abs(measured.position_rad[0]-actual[0])<1e-5F&&
          std::abs(measured.position_rad[3]-actual[3])<1e-5F;
    }
    check(unchanged,"default thrust outputs identical to explicit disabled coordination");
    check(knees_unchanged,"measured coordination preserves timed knee extension targets");
    check(measured.phase==JumpController::Phase::Thrust&&holds_measured&&
          timed.position_rad[1]>actual[1]+.2F&&timed.position_rad[0]<measured.position_rad[0]-.1F,
          "lagging actual knees hold coordinated hips while legacy hips run ahead");
    tracked_input.position_rad[1]=-1.F;tracked_input.position_rad[4]=-.7F;
    bool per_side=true,velocity=true;
    for(int n=0;n<12;++n) {
      const auto previous=measured.position_rad;measured=tracked.update(tracked_input);
      for(auto h:{0,3}) {
        const float target=JumpController::neutralPose(tracked_input.position_rad[h+1])[h];
        const float delta=tracked_config.max_joint_speed_rad_s*tracked_input.dt_s;
        const float expected=std::clamp(target,previous[h]-delta,previous[h]+delta);
        per_side=per_side&&std::abs(measured.position_rad[h]-expected)<1e-6F;
        velocity=velocity&&std::abs(measured.velocity_rad_s[h])<=tracked_config.max_joint_speed_rad_s+1e-4F&&
            std::abs(measured.velocity_rad_s[h]-(measured.position_rad[h]-previous[h])/tracked_input.dt_s)<1e-5F;
      }
    }
    check(per_side&&measured.position_rad[0]>measured.position_rad[3]+.1F,
          "asymmetric measured knees independently coordinate each native hip");
    check(velocity,"measured-knee hip references retain final speed bound and exact velocities");
    tracked_input.position_rad[1]=-1.7F;tracked_input.position_rad[4]=.25F;
    bool clamp_ok=true;
    for(int n=0;n<20;++n) {
      const auto previous=measured.position_rad;measured=tracked.update(tracked_input);
      for(auto h:{0,3}) {
        const float bounded_knee=std::clamp(tracked_input.position_rad[h+1],-1.6F,.2F);
        const float delta=tracked_config.max_joint_speed_rad_s*tracked_input.dt_s;
        const float expected=std::clamp(JumpController::neutralPose(bounded_knee)[h],
            previous[h]-delta,previous[h]+delta);
        clamp_ok=clamp_ok&&std::isfinite(measured.position_rad[h])&&
            std::abs(measured.position_rad[h]-expected)<1e-6F;
      }
    }
    check(measured.phase==JumpController::Phase::Thrust&&clamp_ok,
          "measured knees clamp to supported geometry before hip rate limiting");
  }
  {
    for(auto joint:{0,1,3,4}) for(bool upper:{false,true}) {
      JumpController controller(config());auto sample=input();auto result=thrust(controller,sample);
      const bool hip=joint==0||joint==3;
      sample.position_rad[joint]=hip?(upper?1.26F:-.66F):(upper?.31F:-1.76F);
      result=controller.update(sample);
      bool zero=true;
      for(float v:result.velocity_rad_s)zero=zero&&v==0;
      for(float t:result.feedforward_nm)zero=zero&&t==0;
      check(result.phase==JumpController::Phase::Fault&&result.stop_motors&&!result.owns_pose&&zero&&
            std::string(result.reason)=="measured_joint_travel",
            "each hip/knee travel boundary faults with zero actuation references");
      sample.position_rad=JumpController::neutralPose(0);sample.request=true;
      for(int n=0;n<30;++n)result=controller.update(sample);
      check(result.phase==JumpController::Phase::Fault&&result.stop_motors&&!result.owns_pose,
            "measured travel fault stays latched after valid measurement and request");
    }
    JumpController deep(config());auto sample=input();auto result=thrust(deep,sample);
    sample.position_rad=JumpController::neutralPose(-1.643F);result=deep.update(sample);
    check(result.phase==JumpController::Phase::Thrust&&result.owns_pose&&!result.stop_motors,
          "valid deep measured knee minus 1.643 remains active");
    auto invalid=config();invalid.extension_knee_rad=.3F;
    bool threw=false;try{JumpController rejected(invalid);}catch(const std::invalid_argument&){threw=true;}
    check(threw,"constructor rejects extension beyond plus 0.2 knee envelope");
  }
  std::cout<<checks-failures<<'/'<<checks<<" jump checks passed (synthetic only)\n";
  return failures?1:0;
}
