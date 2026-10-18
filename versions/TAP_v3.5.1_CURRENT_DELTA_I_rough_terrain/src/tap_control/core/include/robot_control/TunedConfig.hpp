#pragma once

#include "robot_control/EmbeddedController.hpp"

namespace robot_control {

// The tuned robot controller profile, shared by the ESP32 firmware and host
// tests. It is a verbatim copy of the native simulator's configureController()
// (src/tap_sim/native/native_runner.cpp); the simulator could not be rebuilt
// when this was split out, so that copy was left untouched. Keep the two in
// step until the simulator calls this function directly.
[[nodiscard]] EmbeddedController::Config tunedControllerConfig();

}  // namespace robot_control
