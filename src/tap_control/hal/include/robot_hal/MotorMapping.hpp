#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace robot_control::motor_mapping {

// Joint index contract from robot_control::JointId without taking a dependency
// on the control core: [LH, LK, LW, RH, RK, RW].
struct LegCanBinding {
  std::size_t joint_index = 0;
  std::uint8_t node_id = 0;
  // Driver coordinate = core coordinate * driver_sign. Applies to position,
  // velocity and torque/feed-forward at the GDS68 wire boundary.
  float driver_sign = 1.0F;
};

struct WheelUartBinding {
  std::size_t joint_index = 0;
  std::uint8_t motor_id = 1;
  // Driver coordinate = core coordinate * driver_sign. The wheel motor
  // directions have not yet been checked on the assembled robot.
  float driver_sign = 1.0F;
};

inline constexpr std::array<LegCanBinding, 4> kVerifiedLegCanBindings{{
    // Verified on the physical robot, 2026-09-25:
    // left hip: lifting the straight leg toward the robot head is positive.
    {0U, 3U, 1.0F},
    // left knee: folding from straight is positive; core knee folding is negative.
    {1U, 4U, -1.0F},
    // right hip: lifting the straight leg toward the robot head is negative.
    {3U, 1U, -1.0F},
    // right knee: folding from straight is negative; core knee folding is negative.
    {4U, 2U, 1.0F},
}};

inline constexpr std::array<WheelUartBinding, 2> kWheelUartBindings{{
    // Verified on the physical robot, 2026-09-25: positive speed moves forward.
    {2U, 1U, 1.0F},  // Left wheel, UART1.
    {5U, 1U, 1.0F},  // Right wheel, UART2.
}};
inline constexpr bool kWheelDirectionsVerified = true;
// Protocol documentation infers raw speed = RPM; retain an enable lock until
// this conversion has been measured on the assembled wheel drive.
inline constexpr bool kWheelSpeedScaleVerified = false;

}  // namespace robot_control::motor_mapping
