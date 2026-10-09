#pragma once

#include "robot_control/EmbeddedController.hpp"

namespace robot_control {

// Shared LQR defaults used by V3Controller and host tests. The GIM MuJoCo
// runner applies its model-specific setup on top of V3Controller.
[[nodiscard]] EmbeddedController::Config tunedControllerConfig();

}  // namespace robot_control
