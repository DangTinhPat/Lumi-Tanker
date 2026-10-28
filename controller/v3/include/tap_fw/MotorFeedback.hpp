#pragma once

#include <array>
#include <cstdint>

#include "robot_control/Types.hpp"
#include "robot_hal/MotorMapping.hpp"
#include "robot_hal/MotorTransport.hpp"
#include "tap_fw/BoardConfig.hpp"

namespace tap_fw {

namespace motor_frame = robot_control::motor_frame;
namespace motor_transport = robot_control::motor_transport;

class MotorFeedback {
 public:
  enum class HomeResult { Homed, StaleFeedback, Moving };

  // Latest ODrive heartbeat of a leg driver (Heartbeat, cmd 0x001).
  struct AxisStatus {
    std::uint8_t state = 0U;
    std::uint32_t error = 0U;
    std::uint64_t stamp_us = 0U;
    bool valid = false;
  };

  bool poll(motor_transport::Transport& transport, std::uint64_t now_us,
            bool legs_enabled) noexcept;
  bool ingestCan(const motor_transport::CanFrame& frame,
                 std::uint64_t now_us) noexcept;
  bool ingestUart(std::uint8_t bus, const motor_frame::DxlFrame& frame,
                  std::uint64_t now_us) noexcept;

  void writeTo(robot_control::RobotState& state) const noexcept;
  [[nodiscard]] bool ready(std::uint64_t now_us) const noexcept;
  // While enabled: wheels fresh, legs heard within board::kLegRideThroughUs
  // (a short CAN outage; the drivers keep their last set-point).
  [[nodiscard]] bool readyToHold(std::uint64_t now_us) const noexcept;
  // Lowest GDS68 DC-bus voltage reported since the last call (NaN if none).
  [[nodiscard]] float takeMinBusVoltage() noexcept;
  [[nodiscard]] bool allLegsFresh(std::uint64_t now_us) const noexcept;
  [[nodiscard]] bool allWheelsFresh(std::uint64_t now_us) const noexcept;
  // Same, with a caller-chosen age limit [us] (the controller allows a longer outage while the wheels only hold).
  [[nodiscard]] bool allWheelsFreshWithin(std::uint64_t now_us, std::uint32_t max_age_us) const noexcept;
  // Oldest wheel reply [us]; 0xFFFFFFFF if a wheel has never answered (telemetry).
  [[nodiscard]] std::uint32_t oldestWheelAgeUs(std::uint64_t now_us) const noexcept;
  [[nodiscard]] std::uint64_t sequence() const noexcept { return sequence_; }

  // Leg zero reference. GDS68 positions restart from zero at every driver
  // power-up, so the legs are homed on the support fixture, whose pose is the
  // neutral geometry: core = driver_sign * driver_raw + offset.
  HomeResult homeLegs(std::uint64_t now_us) noexcept;
  // Same, for an arbitrary known leg pose. Only the leg joints of `pose` are used.
  HomeResult homeLegsAt(const robot_control::JointArray& pose,
                        std::uint64_t now_us) noexcept;
  // Drops the zero when any leg's MIT feedback has been silent long enough
  // for a driver reboot. Returns true on the call that invalidates it.
  bool superviseLegZero(std::uint64_t now_us) noexcept;
  // Restores a zero taken earlier in the same driver power cycle (the
  // drivers kept counting, e.g. across an ESP reset). Offsets per joint.
  void restoreLegOffsets(const robot_control::JointArray& offsets) noexcept;
  [[nodiscard]] bool legsHomed() const noexcept { return legs_homed_; }

  // Get_Encoder_Estimates (0x009) of a leg driver, broadcast by the GDS68 at
  // ~100 Hz in every axis state. Driver frame, rotor turns (8:1 to the joint);
  // zero is the driver's power-up position. Used by the v3 leg lock.
  struct LegEstimate {
    float pos_turns = 0.0F;
    float vel_turns_s = 0.0F;
    std::uint64_t stamp_us = 0U;
    bool valid = false;
  };
  [[nodiscard]] const LegEstimate& legEstimate(std::size_t joint) const noexcept {
    return estimate_[joint];
  }
  // Torque estimate from the driver's MIT feedback; retained for diagnostics.
  [[nodiscard]] float legEffort(std::size_t joint) const noexcept { return effort_nm_[joint]; }
  [[nodiscard]] float legCurrentA(std::size_t joint) const noexcept { return current_a_[joint]; }
  [[nodiscard]] bool legCurrentFresh(std::size_t joint, std::uint64_t now_us) const noexcept {
    return joint < robot_control::kJointCount && current_valid_[joint] &&
        now_us >= current_stamp_us_[joint] &&
        now_us - current_stamp_us_[joint] <= board::kFeedbackMaxAgeUs;
  }
  [[nodiscard]] bool legEffortFresh(std::size_t joint, std::uint64_t now_us) const noexcept {
    return joint < robot_control::kJointCount && fresh(joint, now_us);
  }
  [[nodiscard]] bool allLegCurrentsFresh(std::uint64_t now_us) const noexcept {
    for (const std::size_t joint : {0U, 1U, 3U, 4U}) {
      if (!legCurrentFresh(joint, now_us)) return false;
    }
    return true;
  }
  [[nodiscard]] const AxisStatus& legAxis(std::size_t joint) const noexcept {
    return axis_[joint];
  }
  // First leg joint whose heartbeat is stale, reports an axis error or, when
  // require_closed_loop, is not in CLOSED_LOOP. kJointCount when all are fine.
  [[nodiscard]] std::size_t firstLegAxisFault(std::uint64_t now_us,
                                              bool require_closed_loop) const noexcept;
  [[nodiscard]] const robot_control::JointArray& legOffsets() const noexcept {
    return leg_offset_rad_;
  }

 private:
  [[nodiscard]] bool fresh(std::size_t joint,
                           std::uint64_t now_us) const noexcept;
  bool ingestMit(const robot_control::motor_mapping::LegCanBinding& binding,
                 std::uint8_t node_id, const motor_transport::CanFrame& frame,
                 std::uint64_t now_us) noexcept;

  robot_control::JointArray position_rad_{};
  robot_control::JointArray velocity_rad_s_{};
  robot_control::JointArray effort_nm_{};
  robot_control::JointArray current_a_{};
  std::array<std::uint64_t, robot_control::kJointCount> current_stamp_us_{};
  std::array<bool, robot_control::kJointCount> current_valid_{};
  std::array<std::uint64_t, robot_control::kJointCount> stamp_us_{};
  // Leg positions before the zero offset (driver_sign * driver_raw).
  robot_control::JointArray leg_uncalibrated_rad_{};
  robot_control::JointArray leg_offset_rad_{};
  bool legs_homed_ = false;
  std::array<bool, robot_control::kJointCount> leg_silent_{};
  bool leg_zero_dropped_ = false;
  float bus_voltage_min_v_ = 0.0F;
  bool bus_voltage_seen_ = false;
  std::uint64_t last_vbus_poll_us_ = 0U;
  std::size_t vbus_poll_slot_ = 0U;
  void dropLegZero() noexcept;
  std::array<AxisStatus, robot_control::kJointCount> axis_{};
  std::array<LegEstimate, robot_control::kJointCount> estimate_{};
  std::array<bool, robot_control::kJointCount> valid_{};
  std::array<std::uint16_t, 2> wheel_previous_raw_{};
  std::array<float, 2> wheel_unwrapped_rad_{};
  std::array<bool, 2> wheel_position_seeded_{};
  std::uint64_t last_can_probe_us_ = 0U;
  std::uint64_t last_wheel_poll_us_ = 0U;
  std::uint64_t sequence_ = 0U;
  bool can_probe_initialized_ = false;
  bool wheel_poll_initialized_ = false;
};

}  // namespace tap_fw
