#pragma once

#include <cstdint>

#include "tap_fw/BleCommandEndpoint.hpp"

namespace tap_fw::ble {

// The application maps Dabble's GamePad API to this small POD. No Dabble or
// Arduino header enters the controller library.
struct GamepadState {
  bool up = false;
  bool down = false;
  bool left = false;
  bool right = false;
  bool start = false;
  bool select = false;
  bool triangle = false;
  bool circle = false;
  bool cross = false;
  bool square = false;
};

class DabbleGamepadMapper {
 public:
  struct Config {
    float drive_speed_m_s = 0.25F;
    float yaw_rate_rad_s = 0.35F;
    float natural_height_m = robot_control::geometry::neutral_spawn_z_m;
    float max_stand_height_m = 0.245F;
    float deep_squat_height_m = 0.140F;
    // Start = ENABLE + FIXTURE_RELEASE (straight into balance). When false,
    // the first Start = ENABLE only (controller holds in FixtureHold) and a
    // second Start within the live session = FIXTURE_RELEASE, so the operator
    // can stand the robot at its balance point before releasing it.
    bool auto_fixture_release = true;
  };

  explicit DabbleGamepadMapper(CommandEndpoint& endpoint) noexcept
      : DabbleGamepadMapper(endpoint, Config{}) {}
  DabbleGamepadMapper(CommandEndpoint& endpoint, Config config) noexcept
      : endpoint_(endpoint), config_(config) {}

  // A Start press enables the controller and requests automatic release of
  // the startup fixture once sensor qualification has completed.
  void update(const GamepadState& input, std::uint64_t now_us) noexcept;

 private:
  CommandEndpoint& endpoint_;
  Config config_{};
  GamepadState previous_{};
};

}  // namespace tap_fw::ble
