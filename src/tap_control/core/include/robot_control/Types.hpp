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
};

struct WheelOdometry {
  float position_m = 0.0F;
  float velocity_m_s = 0.0F;
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
