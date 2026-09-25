#include "tap_fw/DabbleGamepadMapper.hpp"

#include <cstdio>
#include <cstring>

namespace tap_fw::ble {

void DabbleGamepadMapper::update(const GamepadState& input,
                                 std::uint64_t now_us) noexcept {
  const auto rising = [&](bool current, bool previous) {
    return current && !previous;
  };
  if (input.square && rising(input.square, previous_.square)) {
    static constexpr char kEstop[] = "ESTOP";
    (void)endpoint_.onWrite(reinterpret_cast<const std::uint8_t*>(kEstop),
                             sizeof(kEstop) - 1U, now_us);
    previous_ = input;
    return;
  }
  if (input.select) {
    static constexpr char kDisable[] = "DISABLE";
    (void)endpoint_.onWrite(reinterpret_cast<const std::uint8_t*>(kDisable),
                             sizeof(kDisable) - 1U, now_us);
    previous_ = input;
    return;
  }
  if (input.start && rising(input.start, previous_.start)) {
    static constexpr char kEnable[] = "ENABLE";
    static constexpr char kFixtureRelease[] = "FIXTURE_RELEASE";
    (void)endpoint_.onWrite(reinterpret_cast<const std::uint8_t*>(kEnable),
                             sizeof(kEnable) - 1U, now_us);
    (void)endpoint_.onWrite(
        reinterpret_cast<const std::uint8_t*>(kFixtureRelease),
        sizeof(kFixtureRelease) - 1U, now_us);
  }

  bool pose_requested = false;
  const auto sendHeight = [&](float height_m) {
    char command[32]{};
    std::snprintf(command, sizeof(command), "HEIGHT %.6f", height_m);
    (void)endpoint_.onWrite(reinterpret_cast<const std::uint8_t*>(command),
                             std::strlen(command), now_us);
    pose_requested = true;
  };
  if (rising(input.triangle, previous_.triangle)) {
    sendHeight(config_.max_stand_height_m);
  }
  if (rising(input.circle, previous_.circle)) {
    sendHeight(config_.deep_squat_height_m);
  }
  if (rising(input.cross, previous_.cross)) {
    sendHeight(config_.natural_height_m);
  }
  if (pose_requested) {
    static constexpr char kStop[] = "STOP";
    (void)endpoint_.onWrite(reinterpret_cast<const std::uint8_t*>(kStop),
                             sizeof(kStop) - 1U, now_us);
    previous_ = input;
    return;
  }

  const float forward = (input.up ? config_.drive_speed_m_s : 0.0F) -
      (input.down ? config_.drive_speed_m_s : 0.0F);
  const float yaw = (input.right ? config_.yaw_rate_rad_s : 0.0F) -
      (input.left ? config_.yaw_rate_rad_s : 0.0F);
  char command[48]{};
  std::snprintf(command, sizeof(command), "DRIVE %.3f %.3f", forward, yaw);
  (void)endpoint_.onWrite(reinterpret_cast<const std::uint8_t*>(command),
                           std::strlen(command), now_us);
  previous_ = input;
}

}  // namespace tap_fw::ble
