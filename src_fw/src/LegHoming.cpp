#include "tap_fw/LegHoming.hpp"

#include <cmath>

#include "robot_control/WholeBodyModel.hpp"

namespace tap_fw {

HangPose hangPose(float knee_stop_rad) noexcept {
  // The downward equilibrium lies between these hip angles for every knee in
  // the joint envelope (the upward one is ~pi away); bisect the hip gravity
  // torque sign change.
  float lo = -0.6F;
  float hi = 0.9F;
  const auto torque = [&](float hip) {
    return robot_control::WholeBodyModel::legGravityTorque(hip, knee_stop_rad).hip_nm;
  };
  const bool lo_positive = torque(lo) > 0.0F;
  for (int i = 0; i < 40; ++i) {
    const float mid = 0.5F * (lo + hi);
    if ((torque(mid) > 0.0F) == lo_positive) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return HangPose{0.5F * (lo + hi), knee_stop_rad};
}

robot_control::JointArray hangingLegPose(const HangPose& hang,
                                         float body_pitch_rad) noexcept {
  robot_control::JointArray pose{};
  const float hip = hang.hip_rad - body_pitch_rad;
  pose[0] = hip;
  pose[1] = hang.knee_rad;
  pose[3] = hip;
  pose[4] = hang.knee_rad;
  return pose;
}

}  // namespace tap_fw
