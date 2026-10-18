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
    // Corrected on the robot 2026-09-26: a positive core wheel command drove
    // both wheels backward (first balance run ran away at full speed), so a
    // forward core velocity is a negative driver Moving Speed on both sides.
    // Left/right in the IMU frame (+X forward, +Z up). Checked 2026-09-26: a
    // swapped assignment spun the robot counter-clockwise with the gyro
    // reading +yaw while the joint-2 wheel ran ~5x the joint-5 wheel.
    // 2026-09-27 TILTTEST (operator, by eye): with {-1, -1} the two wheels
    // turned opposite ways; with {+1, -1} both turned together but against
    // the pitch (nose-down rolled toward the tail). So the right wheel was
    // the reversed one: {-1, +1}. Encoders cannot show this because the same
    // sign is applied to command and feedback.
    {2U, 1U, -1.0F},  // UART1: left wheel (joint LW).
    {5U, 1U, 1.0F},   // UART2: right wheel (joint RW).
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
