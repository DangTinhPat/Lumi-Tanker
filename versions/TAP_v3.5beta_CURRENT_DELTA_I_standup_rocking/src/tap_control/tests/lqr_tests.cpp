#include <cmath>
#include <iostream>

#include "robot_control/LqrBalancer.hpp"

using namespace robot_control;

int main() {
  LqrBalancer::Config config;
  config.max_ground_accel_m_s2 = 1.0F;
  config.wheel_ground_mu = 10.0F;
  LqrBalancer controller(config);
  RobotState state;
  ControlCommand command;
  state.imu.pitch_rad = 0.05F;
  const auto positive = controller.update(state, command);
  if (!(positive.left_wheel_velocity_rad_s > 0.0F &&
        positive.right_wheel_velocity_rad_s > 0.0F)) {
    std::cerr << "LQR pitch sign failed\n";
    return 1;
  }
  if (std::abs(controller.debugState().optimal_command_m_s) >
      config.max_ground_accel_m_s2 + 1.0e-6F) {
    std::cerr << "LQR acceleration limit failed\n";
    return 1;
  }
  state.imu.pitch_rad = 0.0F;
  state.imu.pitch_rate_rad_s = 0.0F;
  command.target_yaw_rate_rad_s = 0.25F;
  const auto turn = controller.update(state, command);
  const float yaw = config.wheel_radius_m *
      (turn.right_wheel_velocity_rad_s - turn.left_wheel_velocity_rad_s) /
      (2.0F * config.half_wheel_base_m);
  if (!std::isfinite(yaw) || std::abs(yaw) > config.max_yaw_rate_rad_s + 1.0e-6F) {
    std::cerr << "LQR yaw limit failed\n";
    return 1;
  }

  LqrBalancer::Config station_config = config;
  station_config.position_anchor_hold = true;
  station_config.station_hold_gain_hz = 15.0F;
  station_config.station_hold_max_v = 0.25F;
  station_config.station_hold_dynamic_max_v = 0.25F;
  station_config.station_adaptive_full_capture_distance_m = 0.10F;
  station_config.station_hold_command_slew_rate_m_s2 = 0.10F;
  station_config.wheel_velocity_reference_from_measured = true;
  LqrBalancer station_controller(station_config);
  RobotState parked;
  parked.dt_s = 0.002F;
  parked.imu.sample_dt_s = 0.002F;
  parked.floor_contact = true;
  parked.wheel.position_m = 0.20F;
  ControlCommand stop;
  for (int i = 0; i < 100; ++i) {
    (void)station_controller.update(parked, stop, true, 0.0F, true);
  }
  if (station_controller.debugState().station_gain_blend != 0.0F ||
      std::abs(station_controller.debugState().lqr_velocity_reference_m_s) >
          1.0e-5F) {
    std::cerr << "station return armed before pitch quiet dwell\n";
    return 1;
  }
  for (int i = 0; i < 300; ++i) {
    (void)station_controller.update(parked, stop, true, 0.0F, true);
  }
  if (!(station_controller.debugState().station_gain_blend > 0.5F &&
        station_controller.debugState().lqr_velocity_reference_m_s <
            -1.0e-3F)) {
    std::cerr << "station return reference governor failed\n";
    return 1;
  }
  std::cout << "PASS LQR DARE/sign/limit contract\n";
  return 0;
}
