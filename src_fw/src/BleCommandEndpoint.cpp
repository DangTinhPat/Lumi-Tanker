#include "tap_fw/BleCommandEndpoint.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "robot_control/GeneratedGeometry.hpp"

namespace tap_fw::ble {

namespace {

using robot_control::ControlCommand;

bool parseDrive(const char* text, float& forward, float& yaw) noexcept {
  char extra = '\0';
  return std::sscanf(text, "%f %f %c", &forward, &yaw, &extra) == 2 &&
      std::isfinite(forward) && std::isfinite(yaw);
}

bool parseHeight(const char* text, float& height) noexcept {
  char extra = '\0';
  return std::sscanf(text, "%f %c", &height, &extra) == 1 &&
      std::isfinite(height);
}

bool parsePose(const char* text, float& lh, float& lk,
               float& rh, float& rk) noexcept {
  char extra = '\0';
  return std::sscanf(text, "%f %f %f %f %c", &lh, &lk, &rh, &rk, &extra) == 4 &&
      std::isfinite(lh) && std::isfinite(lk) &&
      std::isfinite(rh) && std::isfinite(rk);
}

void trim(char* text) noexcept {
  std::size_t length = std::strlen(text);
  while (length > 0U && std::isspace(static_cast<unsigned char>(text[length - 1U]))) {
    text[--length] = '\0';
  }
  std::size_t first = 0U;
  while (text[first] != '\0' &&
         std::isspace(static_cast<unsigned char>(text[first]))) {
    ++first;
  }
  if (first != 0U) std::memmove(text, text + first, length - first + 1U);
}

}  // namespace

CommandEndpoint::CommandEndpoint() noexcept : CommandEndpoint(Config{}) {}

CommandEndpoint::CommandEndpoint(Config config) noexcept : config_(config) {
  resetCommand();
}

void CommandEndpoint::resetCommand() noexcept {
  command_ = ControlCommand{};
  command_.target_joint_position_rad = {
      robot_control::geometry::neutral_hip_rad,
      robot_control::geometry::neutral_knee_rad,
      0.0F,
      robot_control::geometry::neutral_hip_rad,
      robot_control::geometry::neutral_knee_rad,
      0.0F};
  command_.target_body_height_m = robot_control::geometry::neutral_spawn_z_m;
  command_.request_disable = false;
  command_.request_emergency_stop = emergency_latched_;
}

void CommandEndpoint::safeStop() noexcept {
  command_.request_enable = false;
  command_.request_disable = true;
  command_.request_pose_change = false;
  command_.request_height_change = false;
  command_.request_stand_up = false;
  command_.request_fixture_release = false;
  command_.target_forward_velocity_m_s = 0.0F;
  command_.target_yaw_rate_rad_s = 0.0F;
  command_.request_emergency_stop = emergency_latched_;
}

void CommandEndpoint::refresh(std::uint64_t now_us) noexcept {
  last_rx_us_ = now_us;
  has_rx_ = true;
}

bool CommandEndpoint::motionAllowed() const noexcept {
  return command_.request_enable && !command_.request_disable &&
      !emergency_latched_;
}

bool CommandEndpoint::timedOut(std::uint64_t now_us) const noexcept {
  // A command stamped after the caller's `now` (the USB console stamps with
  // the time it parsed the line, later than the control loop's tick time) is
  // fresh; the unsigned difference would otherwise wrap to a huge age and
  // disable the robot on every console heartbeat.
  return !has_rx_ ||
      (now_us > last_rx_us_ && now_us - last_rx_us_ > config_.heartbeat_timeout_us);
}

WriteResult CommandEndpoint::onWrite(const std::uint8_t* data,
                                     std::size_t length,
                                     std::uint64_t now_us) noexcept {
  if (data == nullptr || length == 0U || length >= kMaxCommandBytes) {
    return WriteResult::Rejected;
  }
  char text[kMaxCommandBytes]{};
  std::memcpy(text, data, length);
  text[length] = '\0';
  trim(text);
  if (text[0] == '\0') return WriteResult::Rejected;

  if (std::strcmp(text, "HEARTBEAT") == 0) {
    refresh(now_us);
    return WriteResult::Heartbeat;
  }
  if (std::strcmp(text, "ESTOP") == 0) {
    emergency_latched_ = true;
    safeStop();
    refresh(now_us);
    return WriteResult::Accepted;
  }
  if (std::strcmp(text, "CLEAR_ESTOP") == 0) {
    emergency_latched_ = false;
    command_.request_emergency_stop = false;
    command_.request_enable = false;
    command_.request_disable = false;
    refresh(now_us);
    return WriteResult::Accepted;
  }
  if (std::strcmp(text, "RESET") == 0) {
    emergency_latched_ = false;
    resetCommand();
    refresh(now_us);
    return WriteResult::ResetRequested;
  }
  if (std::strcmp(text, "ENABLE") == 0) {
    if (emergency_latched_) return WriteResult::Rejected;
    command_.request_enable = true;
    command_.request_disable = false;
    command_.request_emergency_stop = false;
    refresh(now_us);
    return WriteResult::Accepted;
  }
  if (std::strcmp(text, "DISABLE") == 0) {
    safeStop();
    refresh(now_us);
    return WriteResult::Accepted;
  }
  if (!motionAllowed()) return WriteResult::Rejected;

  float first = 0.0F;
  float second = 0.0F;
  if (std::strcmp(text, "STOP") == 0) {
    command_.target_forward_velocity_m_s = 0.0F;
    command_.target_yaw_rate_rad_s = 0.0F;
    refresh(now_us);
    return WriteResult::Accepted;
  }
  if (std::strncmp(text, "DRIVE ", 6U) == 0 &&
      parseDrive(text + 6, first, second)) {
    command_.target_forward_velocity_m_s = std::clamp(
        first, -config_.max_forward_velocity_m_s,
        config_.max_forward_velocity_m_s);
    command_.target_yaw_rate_rad_s = std::clamp(
        second, -config_.max_yaw_rate_rad_s, config_.max_yaw_rate_rad_s);
    refresh(now_us);
    return WriteResult::Accepted;
  }
  if (std::strncmp(text, "HEIGHT ", 7U) == 0 &&
      parseHeight(text + 7, first)) {
    command_.target_body_height_m = first;
    command_.request_height_change = true;
    refresh(now_us);
    return WriteResult::Accepted;
  }
  if (std::strncmp(text, "POSE ", 5U) == 0) {
    float rh = 0.0F;
    float rk = 0.0F;
    if (parsePose(text + 5, first, second, rh, rk)) {
      command_.target_joint_position_rad[0] = first;
      command_.target_joint_position_rad[1] = second;
      command_.target_joint_position_rad[3] = rh;
      command_.target_joint_position_rad[4] = rk;
      command_.request_pose_change = true;
      refresh(now_us);
      return WriteResult::Accepted;
    }
  }
  if (std::strcmp(text, "STAND_UP") == 0) {
    command_.request_stand_up = true;
    refresh(now_us);
    return WriteResult::Accepted;
  }
  if (std::strcmp(text, "FIXTURE_RELEASE") == 0) {
    command_.request_fixture_release = true;
    refresh(now_us);
    return WriteResult::Accepted;
  }
  return WriteResult::Rejected;
}

robot_control::ControlCommand CommandEndpoint::command(
    std::uint64_t now_us) const noexcept {
  auto output = command_;
  if (timedOut(now_us)) {
    const bool active_session = command_.request_enable ||
        command_.request_disable;
    output.request_enable = false;
    output.request_disable = active_session;
    output.request_pose_change = false;
    output.request_height_change = false;
    output.request_stand_up = false;
    output.request_fixture_release = false;
    output.target_forward_velocity_m_s = 0.0F;
    output.target_yaw_rate_rad_s = 0.0F;
  }
  output.request_emergency_stop = emergency_latched_;
  return output;
}

void CommandEndpoint::onDisconnect() noexcept {
  safeStop();
  has_rx_ = false;
}

}  // namespace tap_fw::ble
