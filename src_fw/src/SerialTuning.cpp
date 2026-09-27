#include "tap_fw/SerialTuning.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace tap_fw {
namespace {

void printTunables(const robot_control::EmbeddedController& controller, char* reply,
                   std::size_t reply_size) noexcept {
  const auto& lqr = controller.lqrConfig();
  std::snprintf(reply, reply_size,
                "TUNE trim_gain=%.3f trim_max=%.3f trim=%+.4f margin=%.3f accel=%.3f "
                "pitch_scale=%.2f",
                lqr.balance_trim_gain_rad_per_m, lqr.balance_trim_max_rad,
                controller.balanceTrim(), lqr.command_velocity_tracking_margin_m_s,
                lqr.max_ground_accel_m_s2, lqr.wheel_pitch_feedback_scale);
}

}  // namespace

bool handleTuningLine(const char* line, robot_control::EmbeddedController& controller,
                      char* reply, std::size_t reply_size) noexcept {
  if (line == nullptr || reply == nullptr || reply_size == 0U) return false;
  reply[0] = '\0';
  if (std::strcmp(line, "GET") == 0) {
    printTunables(controller, reply, reply_size);
    return true;
  }
  if (std::strcmp(line, "TRIM RESET") == 0) {
    controller.setBalanceTrim(0.0F);
    printTunables(controller, reply, reply_size);
    return true;
  }
  if (std::strncmp(line, "SET ", 4U) != 0) return false;

  char key[16]{};
  char value_text[24]{};
  if (std::sscanf(line + 4, "%15s %23s", key, value_text) != 2) {
    std::snprintf(reply, reply_size, "TUNE error: usage SET <key> <value>");
    return true;
  }
  char* end = nullptr;
  const float value = std::strtof(value_text, &end);
  if (end == value_text || *end != '\0' || !std::isfinite(value)) {
    std::snprintf(reply, reply_size, "TUNE error: bad value '%s'", value_text);
    return true;
  }
  if (std::strcmp(key, "trim_gain") == 0) {
    controller.setBalanceTrimGain(std::clamp(value, 0.0F, 2.0F));
  } else if (std::strcmp(key, "trim_max") == 0) {
    controller.setBalanceTrimMax(std::clamp(value, 0.0F, 0.35F));
  } else if (std::strcmp(key, "trim") == 0) {
    controller.setBalanceTrim(value);  // clamped to trim_max by the balancer
  } else if (std::strcmp(key, "margin") == 0) {
    controller.setCommandTrackingMargin(std::clamp(value, 0.0F, 1.0F));
  } else if (std::strcmp(key, "accel") == 0) {
    controller.setMaxGroundAccel(std::clamp(value, 0.2F, 2.0F));
  } else if (std::strcmp(key, "pitch_scale") == 0) {
    controller.setWheelPitchFeedbackScale(std::clamp(value, 0.3F, 1.0F));
  } else {
    std::snprintf(reply, reply_size,
                  "TUNE error: unknown key '%s' (trim_gain trim_max trim margin accel pitch_scale)",
                  key);
    return true;
  }
  printTunables(controller, reply, reply_size);
  return true;
}

}  // namespace tap_fw
