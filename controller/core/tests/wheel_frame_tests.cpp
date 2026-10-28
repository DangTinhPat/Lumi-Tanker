#include <iostream>
#include <limits>
#include "robot_control/WheelFrame.hpp"
using namespace robot_control;
int main() {
  int checks=0,failures=0;
  auto check=[&](bool ok,const char* msg){++checks;if(!ok){++failures;std::cerr<<msg<<'\n';}};
  auto near=[](float a,float b){return std::abs(a-b)<1e-6F;};
  JointState j;
  j.velocity_rad_s={1,2,-3.5F,1,2,-3.5F};
  j.position_rad={.1F,.2F,-.35F,.1F,.2F,-.35F};
  auto r=WheelFrame::observe(j,.05F,.5F,.03F);
  check(r.valid&&near(r.axle.velocity_m_s,0)&&near(r.axle.position_m,0),"counterrotating joints yield zero axle travel");
  j.velocity_rad_s[2]=j.velocity_rad_s[5]=1;
  r=WheelFrame::observe(j,0,.5F,.03F);
  check(near(r.axle.velocity_m_s,.135F),"positive native spins add both sides");
  j.velocity_rad_s={1,2,0,-2,1,0};r=WheelFrame::observe(j,0,.5F,.03F);
  const float left=WheelFrame::relativeCommand(2,r.carrier_rate[0],true);
  const float right=WheelFrame::relativeCommand(4,r.carrier_rate[1],true);
  check(near(left,-1.5F)&&near(right,4.5F),"asymmetric carrier conversion has no right-side sign flip");
  check(near(left+r.carrier_rate[0],2)&&near(right+r.carrier_rate[1],4),"grounded command roundtrip preserves MPC axle speeds");
  check(near(.03F*((right+r.carrier_rate[1])-(left+r.carrier_rate[0]))/.309F,.06F/.309F),"positive yaw retains right-minus-left axle sign");
  check(near(WheelFrame::relativeCommand(2,3,false),2),"airborne output not carrier compensated");
  for(auto bad:{std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()}) {
    check(!WheelFrame::observe(j,bad,0,.03F).valid,"invalid pitch rejected");
    check(!WheelFrame::observe(j,0,bad,.03F).valid,"invalid rate rejected");
    check(!WheelFrame::observe(j,0,0,bad).valid,"invalid radius rejected");
    for(std::size_t i=0;i<kJointCount;++i) {
      auto invalid=j;invalid.velocity_rad_s[i]=bad;
      check(!WheelFrame::observe(invalid,0,0,.03F).valid,"invalid encoder rejected");
    }
    check(WheelFrame::relativeCommand(1,bad,true)==0,"invalid output conversion is zero");
  }
  std::cout<<checks-failures<<'/'<<checks<<" wheel frame checks passed\n";
  return failures?1:0;
}
