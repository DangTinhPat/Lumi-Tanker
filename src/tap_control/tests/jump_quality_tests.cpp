#include <cmath>
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
  i=input();JumpController::Output o;
  for(int n=0;n<30;++n)o=j.update(i);
  i.request=true;o=j.update(i);i.request=false;
  for(int n=0;n<200&&o.phase==JumpController::Phase::Crouch;++n) {
    i.position_rad=o.position_rad;o=j.update(i);
  }
  check(o.phase==JumpController::Phase::Thrust&&o.station_blend==0,"thrust freezes station correction");
  i.left_contact=i.right_contact=false;
  for(int n=0;n<5;++n)o=j.update(i);
  check(o.phase==JumpController::Phase::Flight&&o.leg_kp_scale==.3F,"flight soft impedance");
  i.left_contact=i.right_contact=true;
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
