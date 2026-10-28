#include "robot_control/JumpController.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <cstring>

using robot_control::JumpController;
using robot_control::JumpPlan;

static JumpPlan plan() {
  JumpPlan p;
  p.count=51;p.thrust_duration_s=.2F;p.flight_duration_s=.4F;
  for(std::size_t i=0;i<p.count;++i) {
    const float t=float(i)/50.F;
    const float k=-1.2F+.6F*t*t*(3-2*t),kd=.6F*6*t*(1-t);
    const float h=JumpController::neutralPose(k)[0];
    const float dh=(JumpController::neutralPose(k+.001F)[0]-
                    JumpController::neutralPose(k-.001F)[0])/.002F;
    auto& s=p.samples[i];
    s.time_s=t;s.hip_rad=h;s.knee_rad=k;
    s.hip_velocity_rad_s=dh*kd;s.knee_velocity_rad_s=kd;
    s.base_z_m=.2F;
    s.feedback_gain[2]=1.F;  // A small synthetic pitch-to-hip feedback.
  }
  assert(p.valid());
  return p;
}

static JumpController::Input input() {
  JumpController::Input in;
  in.dt_s=.005F;in.eligible=in.sensors_fresh=in.contacts_fresh=true;
  in.left_contact=in.right_contact=true;
  in.position_rad=JumpController::neutralPose(0);
  in.body_z_m=in.com_z_m=.2;
  return in;
}

static JumpController::Config config(const JumpPlan& p) {
  JumpController::Config c;
  c.enabled=true;c.plan=&p;c.max_joint_speed_rad_s=28.274334F;
  c.crouch_duration_s=.1F;c.crouch_knee_rad=-1.2F;c.landing_knee_rad=-.6F;
  return c;
}

static JumpController::Output thrust(JumpController& controller, JumpController::Input& in) {
  JumpController::Output out;
  for(int i=0;i<30;++i)out=controller.update(in);
  in.request=true;out=controller.update(in);in.request=false;
  for(int i=0;i<400&&out.phase!=JumpController::Phase::Thrust;++i) {
    in.position_rad=out.position_rad;in.velocity_rad_s={};out=controller.update(in);
    assert(!out.stop_motors);
  }
  assert(out.phase==JumpController::Phase::Thrust);
  return out;
}

static void plannedAdmissionGate() {
  const auto p=plan();
  for(bool ground_speed:{true,false}) for(float sign:{-1.F,1.F}) {
    JumpController controller(config(p));auto in=input();
    JumpController::Output out;
    for(int i=0;i<30;++i)out=controller.update(in);
    assert(out.ready);
    in.request=true;out=controller.update(in);in.request=false;
    assert(out.phase==JumpController::Phase::Crouch);
    if(ground_speed)in.ground_speed_m_s=sign*.01F;
    else in.pitch_rate_rad_s=sign*.1F;
    // Exact preparation pose isolates the velocity gate after crouch duration.
    in.position_rad=JumpController::neutralPose(config(p).crouch_knee_rad);
    in.velocity_rad_s={};
    for(int i=0;i<40;++i) {
      out=controller.update(in);
      assert(out.phase==JumpController::Phase::Crouch&&!out.stop_motors);
      assert(out.plan_elapsed_s==0.F);
    }
    assert(out.phase_elapsed_s>config(p).crouch_duration_s);
    in.ground_speed_m_s=0.F;in.pitch_rate_rad_s=0.F;
    out=controller.update(in);
    assert(out.phase==JumpController::Phase::Thrust&&!out.stop_motors);
    assert(out.leg_kp_scale==0.F&&out.override_wheel_speed&&out.wheel_speed_is_relative);
  }
}

static void step(JumpController& controller, JumpController::Input& in,
                 JumpController::Output& out,bool left=false,bool right=false) {
  in.position_rad=out.position_rad;in.velocity_rad_s=out.velocity_rad_s;
  in.left_contact=left;in.right_contact=right;
  in.airborne=!left&&!right;
  in.ballistic=in.airborne;
  out=controller.update(in);
}

static JumpController::Output flight(JumpController& controller, JumpController::Input& in,
                                     float rise_per_tick=0.F) {
  auto out=thrust(controller,in);
  for(int i=0;i<100&&out.phase==JumpController::Phase::Thrust;++i) {
    in.com_z_m+=rise_per_tick;
    const bool contact=out.plan_elapsed_s+in.dt_s<.2F;
    step(controller,in,out,contact,contact);
    assert(!out.stop_motors);
  }
  assert(out.phase==JumpController::Phase::Flight);
  return out;
}

static void until(JumpController& controller,JumpController::Input& in,
                  JumpController::Output& out,float time) {
  for(int i=0;i<250&&out.plan_elapsed_s<time;++i) {
    step(controller,in,out);
    assert(out.phase==JumpController::Phase::Flight&&!out.stop_motors);
  }
  assert(out.plan_elapsed_s>=time);
}

static void retimingAndContacts() {
  auto p=plan();p.touchdown_com_z_m=.2F;
  {
    JumpController controller(config(p));auto in=input();auto out=flight(controller,in,.01F);
    const float previous=out.plan_elapsed_s;
    step(controller,in,out);
    const float rate=(out.plan_elapsed_s-previous)/in.dt_s;
    assert(out.phase==JumpController::Phase::Flight);
    assert(std::abs(rate-1.F)<1e-5F);
    const auto ref=p.sample(out.plan_elapsed_s);
    assert(std::abs(out.position_rad[1]-ref.knee_rad)<1e-6F);
    assert(std::abs(out.velocity_rad_s[1]-rate*ref.knee_velocity_rad_s)<1e-5F);
    assert(std::abs(out.velocity_rad_s[0]-rate*ref.hip_velocity_rad_s)<1e-5F);
    // Ballistic height changes must not dilate the optimized flight reference.
    controller.reset();in=input();out=flight(controller,in);
    const float reset_time=out.plan_elapsed_s;step(controller,in,out);
    assert(std::abs(out.plan_elapsed_s-reset_time-in.dt_s)<1e-6F);
  }
  {
    auto legacy=p;legacy.touchdown_com_z_m=0;
    JumpController controller(config(legacy));auto in=input();auto out=flight(controller,in,.01F);
    const float previous=out.plan_elapsed_s;step(controller,in,out);
    assert(std::abs(out.plan_elapsed_s-previous-in.dt_s)<1e-6F);
  }
  {
    auto invalid_prediction=p;invalid_prediction.touchdown_com_z_m=.6F;
    JumpController controller(config(invalid_prediction));auto in=input();auto out=flight(controller,in);
    const float previous=out.plan_elapsed_s;step(controller,in,out);
    assert(std::abs(out.plan_elapsed_s-previous-in.dt_s)<1e-6F);
    assert(std::isfinite(out.velocity_rad_s[1]));
  }
  const float touchdown=p.thrust_duration_s+p.flight_duration_s;
  {
    JumpController controller(config(p));auto in=input();auto out=flight(controller,in);
    until(controller,in,out,touchdown);
    const auto ref=p.sample(touchdown);
    for(int i=0;i<4;++i) {
      step(controller,in,out);
      assert(out.phase==JumpController::Phase::Flight&&!out.stop_motors);
      assert(out.plan_elapsed_s==touchdown);
      assert(out.position_rad[1]==ref.knee_rad&&out.position_rad[0]==ref.hip_rad);
      assert(out.velocity_rad_s[0]==0&&out.velocity_rad_s[1]==0);
    }
    // One-sided contact is sufficient only after the configured debounce.
    step(controller,in,out,true,false);
    assert(out.phase==JumpController::Phase::Flight);
    step(controller,in,out,true,false);
    assert(out.phase==JumpController::Phase::Land&&!out.stop_motors);
    assert(out.plan_elapsed_s==touchdown);
    step(controller,in,out,true,true);
    assert(out.phase==JumpController::Phase::Land&&out.plan_elapsed_s>touchdown);
    assert(out.position_rad[1]>ref.knee_rad);
  }
  {
    JumpController controller(config(p));auto in=input();auto out=flight(controller,in);
    until(controller,in,out,.57F);
    step(controller,in,out,true,false);
    assert(out.phase==JumpController::Phase::Flight);
    step(controller,in,out);
    step(controller,in,out,false,true);
    assert(out.phase==JumpController::Phase::Flight); // Contact loss cleared debounce.
    step(controller,in,out,false,true);
    assert(out.phase==JumpController::Phase::Land&&!out.stop_motors);
    assert(out.plan_elapsed_s==touchdown); // Accepted slightly early contact retimes recovery.
  }
  {
    JumpController controller(config(p));auto in=input();auto out=flight(controller,in);
    until(controller,in,out,.3F);
    step(controller,in,out,true,true);step(controller,in,out,true,true);
    assert(out.phase==JumpController::Phase::Fault&&out.stop_motors);
    assert(std::strcmp(out.reason,"plan_contact_timing")==0);
    assert(out.plan_elapsed_s<touchdown-.05F); // Fault before resetting the timeline.
    assert(!out.override_wheel_speed);
    for(float tau:out.feedforward_nm)assert(tau==0);
  }
  {
    JumpController controller(config(p));auto in=input();auto out=flight(controller,in);
    until(controller,in,out,touchdown);
    int waits=0;
    while(!out.stop_motors&&waits<20) {step(controller,in,out);++waits;}
    assert(waits>=8&&waits<=11);
    assert(out.phase==JumpController::Phase::Fault&&out.stop_motors);
    assert(std::strcmp(out.reason,"plan_contact_timing")==0);
    assert(out.plan_elapsed_s==touchdown&&!out.override_wheel_speed);
    for(float speed:out.velocity_rad_s)assert(speed==0);
    for(float tau:out.feedforward_nm)assert(tau==0);
  }
}

int main() {
  {
    const auto p=plan();JumpController controller(config(p));auto in=input();
    auto out=flight(controller,in);
    until(controller,in,out,p.thrust_duration_s+p.flight_duration_s);
    in.pitch_rate_rad_s=.1F;
    in.velocity_rad_s={1.F,2.F,37.F,1.F,2.F,37.F};
    out=controller.update(in);
    assert(out.phase==JumpController::Phase::Flight&&!out.stop_motors);
    assert(std::abs(out.wheel_speed_rad_s+3.1F)<1e-4F);
    assert(out.wheel_speed_is_relative);
  }
  {
    const auto p=plan();auto c=config(p);c.plan_preparation_wait_s=3.F;
    JumpController controller(c);auto in=input();JumpController::Output out;
    for(int i=0;i<30;++i)out=controller.update(in);
    in.request=true;out=controller.update(in);in.request=false;
    in.ground_speed_m_s=.01F;
    for(int i=0;i<500;++i) {
      in.position_rad=out.position_rad;in.velocity_rad_s={};out=controller.update(in);
      assert(out.phase==JumpController::Phase::Crouch&&!out.stop_motors);
    }
    in.stop=true;out=controller.update(in);
    assert(out.stop_motors&&out.phase==JumpController::Phase::Fault);
    for(float wait:{1.F,11.F,std::numeric_limits<float>::quiet_NaN()}) {
      c.plan_preparation_wait_s=wait;bool rejected=false;
      try{JumpController invalid(c);}catch(const std::invalid_argument&){rejected=true;}
      assert(rejected);
    }
  }
  plannedAdmissionGate();
  retimingAndContacts();
  const auto p=plan();
  auto c=config(p);
  JumpController controller(c);
  auto in=input();
  JumpController::Output out;
  for(int i=0;i<30;++i)out=controller.update(in);
  in.request=true;out=controller.update(in);in.request=false;
  for(int i=0;i<400&&out.phase!=JumpController::Phase::Thrust;++i) {
    in.position_rad=out.position_rad;
    in.velocity_rad_s={};
    out=controller.update(in);
  }
  assert(out.phase==JumpController::Phase::Thrust);
  bool flight=false,land=false,complete=false;
  for(int i=0;i<1000;++i) {
    in.position_rad=out.position_rad;
    in.velocity_rad_s=out.velocity_rad_s;
    const float t=out.plan_elapsed_s+.005F;
    in.left_contact=in.right_contact=!(t>=.2F&&t<.6F);
    in.airborne=!in.left_contact&&!in.right_contact;
    in.ballistic=in.airborne;
    out=controller.update(in);
    assert(!out.stop_motors);
    if(out.phase==JumpController::Phase::Flight)flight=true;
    if(out.phase==JumpController::Phase::Land)land=true;
    if(out.phase==JumpController::Phase::Complete){complete=true;break;}
    if(out.phase==JumpController::Phase::Thrust||out.phase==JumpController::Phase::Flight||
       out.phase==JumpController::Phase::Land) {
      assert(out.wheel_speed_is_relative&&out.override_wheel_speed);
      assert(out.leg_kp_scale==0.F);
      assert(std::abs(out.feedforward_nm[0])<=5.F);
    }
  }
  assert(flight&&land&&complete);
  controller.reset();
  assert(controller.phase()==JumpController::Phase::Idle);
  in.pitch_rad=std::numeric_limits<float>::quiet_NaN();
  assert(controller.safetyFault(in)!=nullptr);
  bool rejected=false;
  c.max_joint_speed_rad_s=12.5663706F;
  try {JumpController invalid(c);}catch(const std::invalid_argument&){rejected=true;}
  assert(rejected);
  std::cout<<"Planned jump sequencing, ownership, feedback and safety checks passed\n";
}
