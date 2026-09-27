#pragma once

#include <cstddef>

#include "robot_control/EmbeddedController.hpp"

namespace tap_fw {

// Live bring-up tuning of the adaptive balance angle over the USB serial
// console (the role of getTestingInput() in the Impulse firmware). Lines:
//   GET                      print the tunables
//   SET <key> <value>        key: trim_gain, trim_max, trim, margin, accel,
//                            pitch_scale
//   TRIM RESET               learned balance trim back to 0
// Values are clamped to bring-up ranges; nothing here touches enable state.
// Returns false when the line is not a tuning command. `reply` always gets a
// NUL-terminated one-line answer when true is returned.
bool handleTuningLine(const char* line, robot_control::EmbeddedController& controller,
                      char* reply, std::size_t reply_size) noexcept;

}  // namespace tap_fw
