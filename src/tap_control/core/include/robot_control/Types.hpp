#pragma once

#include <array>
#include <cstddef>

namespace robot_control {

constexpr std::size_t kJointCount = 6;

enum class JointId : std::size_t {
  LeftHip = 0,
  LeftKnee = 1,
  LeftWheel = 2,
  RightHip = 3,
  RightKnee = 4,
  RightWheel = 5,
};

using JointArray = std::array<float, kJointCount>;

struct ImuState {
  float pitch_rad = 0.0F;
  float pitch_rate_rad_s = 0.0F;
  float yaw_rad = 0.0F;
  float yaw_rate_rad_s = 0.0F;
  // Body-frame roll rate (2026-09-16, LandingReflex's differential-knee roll
  // gate - see Config::roll_rate_knee_gain_rad_per_rad_s's own comment).
  // Source: raw gyro angular_velocity.x, never fused/estimated (there is no
  // roll estimator in this stack - only pitch has one). Defaults to 0 so any
  // caller/test that never sets this keeps the prior no-roll-signal
  // behavior.
  float roll_rate_rad_s = 0.0F;
  // Raw accelerometer specific-force magnitude (2026-09-16, LandingImpedance
  // hard-impact gate - see its Config::hard_impact_accel_mps2 comment).
  // Defaults to 1g (stationary, grounded) so any caller/test that never
  // sets this keeps the prior no-impact-signal behavior.
  float accel_magnitude_m_s2 = 9.81F;
  // One-shot/short-lived event latched from a raw accelerometer + gyro
  // touchdown signature. The IMU callback owns the latch because a real
  // impact can last only one or two samples while the control loop may run at
  // a different rate. It is sensor-backed; it is not a pose/contact ground
  // truth shortcut.
  bool hard_impact_event = false;
};

struct WheelOdometry {
  float position_m = 0.0F;
  float velocity_m_s = 0.0F;
  // Native +Y wheel encoder motion converted to ground-tangent metres.  These
  // fields preserve per-side data for relative trajectory observation; the
  // averaged fields above remain the balance controller's existing interface.
  float left_position_m = 0.0F;
  float right_position_m = 0.0F;
  float left_velocity_m_s = 0.0F;
  float right_velocity_m_s = 0.0F;
};

struct JointState {
  JointArray position_rad{};
  JointArray velocity_rad_s{};
  JointArray effort_nm{};
};

struct RobotState {
  float dt_s = 0.005F;
  bool floor_contact = true;
  // Per-side contact (2026-09-16, ROLL differential-leg impedance): real
  // touchdown is never simultaneous on both wheels (asymmetric landing,
  // uneven ground) - floor_contact alone (their OR) cannot tell which side
  // actually touched. Default true, matching floor_contact's own default,
  // so any caller/test that never sets these keeps prior behavior exactly.
  bool left_floor_contact = true;
  bool right_floor_contact = true;
  bool emergency_stop = false;
  ImuState imu{};
  WheelOdometry wheel{};
  JointState joints{};
};

struct ControlCommand {
  float target_forward_velocity_m_s = 0.0F;
  float target_yaw_rate_rad_s = 0.0F;
  JointArray target_joint_position_rad{};
  bool request_enable = false;
  bool request_disable = false;
  bool request_pose_change = false;
  bool request_stand_up = false;
};

struct MotorCommand {
  JointArray position_rad{};
  JointArray velocity_rad_s{};
  JointArray torque_nm{};
  JointArray kp_scale{};
  JointArray kd_scale{};
  // Additive feedforward torque, added to the joint PD term before the
  // torque_nm clamp (see BalanceControllerNode::publishLegCommand). Zero by
  // default; used by LandingImpedance's virtual spring-damper force and by
  // JumpController's own feedforward.
  JointArray feedforward_nm{};
  bool enable_motors = false;
};

}  // namespace robot_control
