#pragma once

#include "robot_control/EmbeddedController.hpp"

namespace tap_fw {

// Controller profile the ESP32 runs on the physical robot: the simulator-tuned
// profile (robot_control::tunedControllerConfig(), kept a verbatim copy of the
// native runner) plus the hardware bring-up deltas below. The simulator never
// uses this profile.
//
// - Adaptive balance angle: the true equilibrium differs from the CAD model
//   (IMU mounting tilt, estimated masses, fixture-homing joint zero). The
//   trim is seeded with the pitch at release and integrates ground-speed
//   drift, the role of the outer speed loop's integral in the Impulse robot.
// - Anti-windup: the UART wheel drives do not always realize the commanded
//   speed; the LQR velocity command stays within a margin of the measured one.
// - Legs hold the natural pose (plus gravity feed-forward) instead of chasing
//   the CAD COM model with the stance task, so the wheel LQR gets its full
//   pitch feedback back (wheel_pitch_feedback_scale 1.0).
[[nodiscard]] robot_control::EmbeddedController::Config firmwareControllerConfig();

// Impulse-inspired cold-start profile: manual natural-pose zero capture,
// minimum-jerk preposition, pitch-seeded TAP balance, and a staged release.
[[nodiscard]] robot_control::EmbeddedController::Config
impulseStyleFirmwareControllerConfig();

}  // namespace tap_fw
