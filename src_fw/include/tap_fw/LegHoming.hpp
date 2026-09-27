#pragma once

#include "robot_control/Types.hpp"

namespace tap_fw {

// Gravity-hang leg reference. GDS68 positions restart from zero at every
// driver power-up (fixture logs of 2026-09-26: raw ~0 at each homing), so the
// legs need a pose that is known without a fixture. With the body held up and
// both legs hanging free, each knee rests at its extension stop and the hip
// settles where the leg's own centre of mass is straight below it. The CAD
// leg model gives that hip angle for a level body; a pitched body (IMU) moves
// it by -pitch, because the leg stays vertical in the world.
struct HangPose {
  float hip_rad = 0.0F;   // at body pitch 0
  float knee_rad = 0.0F;  // extension stop
};

// Solves the hanging hip angle from WholeBodyModel::legGravityTorque().
[[nodiscard]] HangPose hangPose(float knee_stop_rad) noexcept;

// Core joint angles of the hanging legs at the measured body pitch.
[[nodiscard]] robot_control::JointArray hangingLegPose(const HangPose& hang,
                                                       float body_pitch_rad) noexcept;

}  // namespace tap_fw
