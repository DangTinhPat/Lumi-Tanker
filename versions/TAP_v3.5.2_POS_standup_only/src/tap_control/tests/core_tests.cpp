#include <cmath>
#include <iostream>
#include <limits>

#include "robot_control/EmbeddedController.hpp"
#include "robot_control/LqrBalancer.hpp"

using namespace robot_control;

namespace {

int failures = 0;

void check(bool condition, const char* name) {
  std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
  if (!condition) ++failures;
}

bool near(float lhs, float rhs, float epsilon = 1.0e-5F) {
  return std::abs(lhs - rhs) <= epsilon;
}

bool safeStop(const MotorCommand& output) {
  if (output.enable_motors) return false;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (output.velocity_rad_s[i] != 0.0F || output.torque_nm[i] != 0.0F ||
        output.kp_scale[i] != 0.0F || output.kd_scale[i] != 0.0F) {
      return false;
    }
  }
  return true;
}

void geometryTests() {
  WholeBodyModel model;
  const auto neutral = model.compute(geometry::neutral_hip_rad,
                                     geometry::neutral_knee_rad,
                                     geometry::neutral_hip_rad,
                                     geometry::neutral_knee_rad);
  const auto asymmetric = model.compute(.3F, -.2F, .5F, -.8F);
  check(std::isfinite(neutral.com_height_m) &&
            neutral.com_height_m > geometry::wheel_radius_m &&
            std::abs(neutral.com_offset_x_m) < 0.01F,
        "native natural FK has finite near-centered COM");
  check(std::abs(asymmetric.axle_height_difference_m) > .001F,
        "asymmetric FK preserves unequal axle heights");
}

void lqrTests() {
  LqrBalancer::Config config;
  LqrBalancer lqr(config);
  RobotState state;
  ControlCommand command;
  auto output = lqr.update(state, command);
  check(near(output.ground_velocity_m_s, 0.0F), "zero LQR state is neutral");

  state.imu.pitch_rad = .05F;
  output = lqr.update(state, command);
  check(output.left_wheel_velocity_rad_s > 0.0F &&
            output.right_wheel_velocity_rad_s > 0.0F,
        "positive pitch commands both native wheels forward");
  lqr.reset();
  state.imu.pitch_rad = -.05F;
  output = lqr.update(state, command);
  check(output.left_wheel_velocity_rad_s < 0.0F &&
            output.right_wheel_velocity_rad_s < 0.0F,
        "negative pitch commands both native wheels backward");

  state.imu.pitch_rad = 0.0F;
  lqr.reset();
  command.target_yaw_rate_rad_s = .4F;
  output = lqr.update(state, command);
  const float yaw = config.wheel_radius_m *
      (output.right_wheel_velocity_rad_s - output.left_wheel_velocity_rad_s) /
      (2.0F * config.half_wheel_base_m);
  check(output.left_wheel_velocity_rad_s < 0.0F &&
            output.right_wheel_velocity_rad_s > 0.0F && near(yaw, .4F, 1.0e-3F),
        "positive yaw is right-minus-left");

  command = {};
  lqr.reset();
  state.imu.pitch_rad = .30F;
  bool bounded = true;
  for (int i = 0; i < 200; ++i) {
    output = lqr.update(state, command);
    bounded &= std::isfinite(output.ground_velocity_m_s) &&
        std::abs(output.ground_velocity_m_s) <=
            config.max_ground_velocity_m_s + 1.0e-5F &&
        std::abs(lqr.debugState().optimal_command_m_s) <=
            config.max_ground_accel_m_s2 + 1.0e-5F;
  }
  check(bounded, "LQR output remains finite and actuator-bounded");

  state.floor_contact = false;
  const float before = std::abs(output.ground_velocity_m_s);
  output = lqr.update(state, command);
  check(std::abs(output.ground_velocity_m_s) < before,
        "unsupported command decays instead of integrating");

  lqr.setComHeight(.1F);
  check(near(lqr.activePendulumLengthM(), config.min_pendulum_length_m),
        "height schedule respects lower pendulum bound");
  lqr.setComHeight(1.0F);
  check(near(lqr.activePendulumLengthM(), config.max_pendulum_length_m),
        "height schedule respects upper pendulum bound");

  state.imu.pitch_rad = std::numeric_limits<float>::quiet_NaN();
  output = lqr.update(state, command);
  check(output.fallen && output.left_wheel_velocity_rad_s == 0.0F &&
            output.right_wheel_velocity_rad_s == 0.0F,
        "invalid sensor input fails closed");
}

void lifecycleTests() {
  EmbeddedController::Config config;
  config.state_machine.enable_cycles = 1;
  EmbeddedController controller(config);
  RobotState state;
  state.joints.position_rad = config.stand_up.stand_pose_rad;
  ControlCommand command;
  command.target_joint_position_rad = state.joints.position_rad;
  controller.reset(state);

  auto output = controller.update(state, command);
  check(safeStop(output), "disabled controller is safe");
  command.request_enable = true;
  output = controller.update(state, command);
  command.request_enable = false;
  output = controller.update(state, command);
  check(controller.mode() == ControlMode::Balance && output.enable_motors,
        "explicit enable reaches balance");

  const auto cached = controller.update(state, command);
  check(cached.enable_motors, "repeated fresh legacy sample remains active");
  command.request_disable = true;
  output = controller.update(state, command);
  check(controller.mode() == ControlMode::EmergencyStop && safeStop(output),
        "disable has immediate stop priority");
}

void fallGuardTests() {
  EmbeddedController::Config config;
  config.lqr.fall_pitch_rad = 0.55F;
  config.state_machine.enable_cycles = 1;
  EmbeddedController controller(config);
  RobotState state;
  state.joints.position_rad = config.stand_up.stand_pose_rad;
  ControlCommand command;
  command.target_joint_position_rad = state.joints.position_rad;
  controller.reset(state);
  command.request_enable = true;
  (void)controller.update(state, command);
  command.request_enable = false;
  (void)controller.update(state, command);
  state.imu.pitch_rad = 0.60F;
  const auto output = controller.update(state, command);
  check(controller.mode() == ControlMode::Fallen && safeStop(output),
        "supported over-pitch latches Fallen and safe-stops");
}

}  // namespace

int main() {
  geometryTests();
  lqrTests();
  lifecycleTests();
  fallGuardTests();
  return failures == 0 ? 0 : 1;
}
