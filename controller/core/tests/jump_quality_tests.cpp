#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include "robot_control/JumpController.hpp"
#include "robot_control/WholeBodyModel.hpp"
using namespace robot_control;
int failures=0;
void check(bool ok,const char* message) {
  if(!ok) {++failures;std::cerr<<message<<'\n';}
}
JumpController::Input input() {
  JumpController::Input i;
  i.eligible=i.sensors_fresh=i.contacts_fresh=i.left_contact=i.right_contact=true;
  i.position_rad=JumpController::neutralPose(0);i.com_z_m=.217;
  return i;
}
JumpController::Output begin_thrust(JumpController& controller,
                                    JumpController::Input& sample) {
  JumpController::Output output;
  for(int n=0;n<30;++n) output=controller.update(sample);
  sample.request=true;
  output=controller.update(sample);
  sample.request=false;
  for(int n=0;n<200&&output.phase==JumpController::Phase::Crouch;++n) {
    sample.position_rad=output.position_rad;
    output=controller.update(sample);
  }
  return output;
}
int main() {
  JumpController::Config c;c.enabled=true;c.crouch_duration_s=.2;
  JumpController j(c);auto i=input();
  check(j.safetyFault(i)==nullptr,"valid shared safety");
  i.dt_s=0;
  check(j.safetyFault(i)!=nullptr && j.safetyFault(i,false)==nullptr,"zero-time safety does not invent dt");
  i.position_rad[4]=-1.8;
  check(j.safetyFault(i,false)!=nullptr,"travel applies without elapsed time");
  i=input();i.ground_speed_m_s=std::numeric_limits<float>::quiet_NaN();
  check(j.safetyFault(i,false)!=nullptr,"nonfinite ground velocity rejected");
  {
    auto gate_config=c;
    gate_config.crouch_max_pitch_rad=.20F;
    gate_config.crouch_max_pitch_rate_rad_s=.60F;
    JumpController pitch_gate(gate_config);
    auto sample=input();
    for(int n=0;n<30;++n)(void)pitch_gate.update(sample);
    sample.request=true;
    auto gated=pitch_gate.update(sample);
    sample.request=false;
    check(gated.phase==JumpController::Phase::Crouch,"pitch gate test enters crouch");
    sample.pitch_rad=.21F;
    gated=pitch_gate.update(sample);
    check(gated.phase==JumpController::Phase::Fault && gated.stop_motors &&
          std::strcmp(gated.reason,"crouch_pitch_limit")==0,
          "crouch pitch limit aborts before thrust");

    JumpController rate_gate(gate_config);
    sample=input();
    for(int n=0;n<30;++n)(void)rate_gate.update(sample);
    sample.request=true;
    gated=rate_gate.update(sample);
    sample.request=false;
    sample.pitch_rate_rad_s=.61F;
    gated=rate_gate.update(sample);
    check(gated.phase==JumpController::Phase::Fault && gated.stop_motors &&
          std::strcmp(gated.reason,"crouch_pitch_rate_limit")==0,
          "crouch pitch-rate limit aborts before thrust");
  }
  {
    auto slow_config=c;
    slow_config.crouch_joint_speed_rad_s=4.F;
    slow_config.crouch_min_speed_scale=.25F;
    JumpController nominal(slow_config), slowed(slow_config);
    auto nominal_input=input(), slowed_input=input();
    for(int n=0;n<30;++n) {
      (void)nominal.update(nominal_input);
      (void)slowed.update(slowed_input);
    }
    nominal_input.request=true;slowed_input.request=true;
    auto nominal_out=nominal.update(nominal_input);
    auto slowed_out=slowed.update(slowed_input);
    nominal_input.request=false;slowed_input.request=false;
    nominal_input.pitch_rate_rad_s=0.F;
    slowed_input.pitch_rate_rad_s=.55F;
    bool saw_rate_slowdown=false;
    for(int n=0;n<60&&nominal_out.phase==JumpController::Phase::Crouch;++n) {
      nominal_input.position_rad=nominal_out.position_rad;
      slowed_input.position_rad=slowed_out.position_rad;
      nominal_out=nominal.update(nominal_input);
      slowed_out=slowed.update(slowed_input);
      saw_rate_slowdown=saw_rate_slowdown||
          std::abs(slowed_out.velocity_rad_s[1])+1e-6F <
          std::abs(nominal_out.velocity_rad_s[1]);
    }
    check(saw_rate_slowdown,"crouch pitch-rate feedback reduces joint speed");
    slow_config.crouch_joint_speed_rad_s=slow_config.max_joint_speed_rad_s+.01F;
    bool rejected=false;
    try { JumpController invalid(slow_config); } catch(const std::invalid_argument&) {
      rejected=true;
    }
    check(rejected,"crouch speed cannot exceed actuator profile");
  }
  i=input();JumpController::Output o;
  for(int n=0;n<30;++n)o=j.update(i);
  i.request=true;o=j.update(i);i.request=false;
  for(int n=0;n<200&&o.phase==JumpController::Phase::Crouch;++n) {
    i.position_rad=o.position_rad;o=j.update(i);
  }
  check(o.phase==JumpController::Phase::Thrust&&o.station_blend==0,"thrust freezes station correction");
  check(std::abs(o.feedforward_nm[0]-o.feedforward_nm[3])<1e-6F &&
        std::abs(o.feedforward_nm[1]-o.feedforward_nm[4])<1e-6F &&
        std::abs(o.feedforward_nm[0])+std::abs(o.feedforward_nm[1])>0.F,
        "bilateral thrust allocates symmetric hip/knee effort to both legs");
  {
    JumpController bilateral_guard(c);
    auto partial=input();
    auto partial_out=begin_thrust(bilateral_guard,partial);
    partial.left_contact=false;
    partial.ballistic=false;
    partial_out=bilateral_guard.update(partial);
    check(partial_out.phase==JumpController::Phase::Fault &&
          std::strcmp(partial_out.reason,"bilateral_support_lost_before_takeoff")==0,
          "bilateral thrust never continues with one support side");
  }
  i.left_contact=i.right_contact=false;
  i.airborne=true;i.ballistic=true;
  for(int n=0;n<5;++n)o=j.update(i);
  check(o.phase==JumpController::Phase::Flight&&o.leg_kp_scale==.3F,"flight soft impedance");
  i.left_contact=i.right_contact=true;
  i.airborne=false;i.ballistic=false;
  for(int n=0;n<5&&o.phase!=JumpController::Phase::Land;++n)o=j.update(i);
  check(o.phase==JumpController::Phase::Land&&o.leg_kp_scale<.4F,"touchdown avoids full-gain step");
  i.velocity_rad_s[1]=1;
  float previous=o.leg_kp_scale;
  for(int n=0;n<65;++n) {
    i.position_rad=o.position_rad;o=j.update(i);
    check(o.leg_kp_scale>=previous&&o.leg_kp_scale<=1,"landing gain ramp bounded monotonic");
    previous=o.leg_kp_scale;
  }
  check(o.phase==JumpController::Phase::Land,"fast landing joint cannot start recovery");
  i.velocity_rad_s[1]=0;o=j.update(i);
  check(o.phase==JumpController::Phase::Settle&&o.station_blend==0,"station resumes from zero correction");
  i.ground_speed_m_s=.1;
  previous=o.station_blend;
  for(int n=0;n<150;++n) {
    i.position_rad=o.position_rad;o=j.update(i);
    check(o.station_blend>=previous&&o.station_blend<=1,"station recovery ramp bounded monotonic");
    previous=o.station_blend;
  }
  check(o.phase==JumpController::Phase::Settle,"moving axle cannot report complete");
  i.ground_speed_m_s=0;
  for(int n=0;n<150&&o.phase!=JumpController::Phase::Complete;++n) {
    i.position_rad=o.position_rad;o=j.update(i);
  }
  check(o.phase==JumpController::Phase::Complete,"stationary posture completes after dwell");
  for(int n=0;n<=1000;++n) {
    const auto q=JumpController::neutralPose(-1.6F+1.8F*n/1000.F);
    check(std::abs(WholeBodyModel{}.compute(q[0],q[1],q[3],q[4]).com_offset_x_m)<1e-6F,
          "neutral FK remains accurate across full command range");
  }
  j.reset();i=input();o=j.update(i);
  check(o.position_rad==JumpController::neutralPose(0),"cached neutral pose survives reset");
  std::cout<<"Jump quality failures: "<<failures<<'\n';
  return failures?1:0;
}
