#include <cmath>
#include <iostream>
#include "robot_control/EmbeddedController.hpp"

int main() {
  using namespace robot_control;
  EmbeddedController::Config config;
  EmbeddedController controller(config);
  RobotState state;
  state.joints.position_rad = config.stand_up.stand_pose_rad;
  ControlCommand command;
  command.target_joint_position_rad = state.joints.position_rad;
  controller.reset(state);
  command.request_enable = true;
  MotorCommand out;
  for (int i = 0; i < 25; ++i) out = controller.update(state, command);
  command.request_enable = false;
  if (controller.mode() != ControlMode::Balance) return 1;
  state.imu.pitch_rad = 0.05F;
  out = controller.update(state, command);
  std::cout << "TAP core synthetic input demo (not a physics test)\n"
            << "mode=" << toString(controller.mode())
            << " wheel_left=" << out.velocity_rad_s[2]
            << " wheel_right=" << out.velocity_rad_s[5]
            << " com_height_m=" << controller.wholeBodyState().com_height_m
            << '\n';
  if (!(out.velocity_rad_s[2] > 0 && out.velocity_rad_s[5] > 0)) return 1;
  state.emergency_stop = true;
  out = controller.update(state, command);
  return out.enable_motors ? 1 : 0;
}
