#pragma once
#include <array>
#include <cmath>
#include "robot_control/Types.hpp"

namespace robot_control {
// Planar, no-slip axle odometry. Native +Y axes on BOTH sides.
// MPC ground state is axle travel, not body-origin travel: no axle Jacobian
// subtraction belongs here. World-body station keeping is a separate loop.
struct WheelFrame {
  struct Result {
    WheelOdometry axle{};
    std::array<float,2> carrier_rate{};
    bool valid = false;
  };
  static Result observe(const JointState& joints,float pitch,float pitch_rate,float radius) {
    Result out;
    if(!std::isfinite(pitch)||!std::isfinite(pitch_rate)||!std::isfinite(radius)||radius<=0) return out;
    for(std::size_t i=0;i<kJointCount;++i)
      if(!std::isfinite(joints.position_rad[i])||!std::isfinite(joints.velocity_rad_s[i])) return out;
    double pos=0,vel=0;
    for(int side=0;side<2;++side) {
      const int h=side*3,k=h+1,w=h+2;
      const double carrier=double(joints.velocity_rad_s[h])+joints.velocity_rad_s[k]+pitch_rate;
      out.carrier_rate[side]=static_cast<float>(carrier);
      pos+=double(radius)*(double(joints.position_rad[w])+joints.position_rad[h]+joints.position_rad[k]+pitch)/2;
      vel+=double(radius)*(double(joints.velocity_rad_s[w])+carrier)/2;
    }
    out.axle={static_cast<float>(pos),static_cast<float>(vel)};
    out.valid=std::isfinite(out.axle.position_m)&&std::isfinite(out.axle.velocity_m_s)&&
              std::isfinite(out.carrier_rate[0])&&std::isfinite(out.carrier_rate[1]);
    return out.valid?out:Result{};
  }
  static float relativeCommand(float absolute_rate,float carrier_rate,bool grounded) {
    if(!std::isfinite(absolute_rate)||!std::isfinite(carrier_rate)) return 0;
    const double value=double(absolute_rate)-(grounded?carrier_rate:0.F);
    const float out=static_cast<float>(value);
    return std::isfinite(out)?out:0.F;
  }
};
}
