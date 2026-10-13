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
    // Driver directions observed on the physical robot, 2026-09-25: left hip
    // driver + lifts the straight leg toward the robot head, right hip
    // driver - does. The core hip turns about +Y, so a positive core hip
    // swings the foot toward the TAIL (CAD natural hip +1.01 rad). The
    // 2026-09-25 signs assumed the opposite; corrected 2026-09-26 when the
    // right leg, hand-set to the natural pose after gravity-hang homing, read
    // hip -0.828 / knee -1.069 (knee exact, hip = hang - (natural - hang)).
    // left hip: core + (foot toward tail) = driver -.
    {0U, 3U, -1.0F},
    // left knee: folding from straight is positive; core knee folding is negative.
    {1U, 4U, -1.0F},
    // right hip: core + (foot toward tail) = driver +.
    {3U, 1U, 1.0F},
    // right knee: folding from straight is negative; core knee folding is negative.
    {4U, 2U, 1.0F},
}};

inline constexpr std::array<WheelUartBinding, 2> kWheelUartBindings{{
    // Kpower BL4012, one half-duplex single-wire bus per wheel (verified on the
    // robot 2026-10-12/13): the left wheel is ID 2 on the "UART2" connector
    // (GPIO15) and follows the core direction; the right wheel is ID 1 on the
    // "UART1" connector (GPIO17) and is reversed. Index = bus handed to the
    // transport: 0 = left, 1 = right.
    {2U, 2U, 1.0F},   // left wheel (joint LW), id 2.
    {5U, 1U, -1.0F},  // right wheel (joint RW), id 1.
}};
inline constexpr bool kWheelDirectionsVerified = true;
// Present Speed readback scale. 2026-09-26 a balance trace suggested the
// readback was 0.585x the encoder-position derivative, but the 2026-09-27 v3
// free-spinning run (wheels off the ground, 35 rad/s) showed raw readback =
// position derivative = commanded speed (e.g. -19.6 rad/s derivative, -19.8
// command, scaled readback -34). The 1/0.585 correction overstated wheel speed
// ~1.7x, so the readback is used unscaled.
inline constexpr float kWheelPresentSpeedScale = 1.0F;
// Protocol documentation infers raw speed = RPM; retain an enable lock until
// this conversion has been measured on the assembled wheel drive.
inline constexpr bool kWheelSpeedScaleVerified = true;

}  // namespace robot_control::motor_mapping
