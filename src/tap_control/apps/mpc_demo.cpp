#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>

#define private public
#include "robot_control/MpcBalancer.hpp"
#undef private

namespace {

constexpr auto kLeftWheel =
    static_cast<std::size_t>(robot_control::JointId::LeftWheel);
constexpr auto kRightWheel =
    static_cast<std::size_t>(robot_control::JointId::RightWheel);

bool expect(bool condition, const char* label) {
  std::cout << (condition ? "[ok]   " : "[fail] ") << label << '\n';
  return condition;
}

robot_control::MpcBalancer::Config testConfig() {
  robot_control::MpcBalancer::Config config;
  config.max_ground_velocity_m_s = 2.0F;
  config.left_wheel_sign = 1.0F;
  return config;
}

robot_control::RobotState baseState() {
  robot_control::RobotState state;
  state.dt_s = 0.02F;
  state.floor_contact = true;
  state.imu.pitch_rad = 0.0F;
  state.imu.pitch_rate_rad_s = 0.0F;
  state.wheel.position_m = 0.0F;
  state.wheel.velocity_m_s = 0.0F;
  return state;
}

void printStep(const char* label, const robot_control::MpcBalancer& balancer,
               const robot_control::MpcBalancer::Output& output,
               const robot_control::RobotState& state) {
  const auto debug = balancer.debugState();
  std::cout << std::setw(12) << label << " pitch=" << state.imu.pitch_rad
            << " pitch_rate=" << state.imu.pitch_rate_rad_s
            << " wheel_v=" << state.wheel.velocity_m_s
            << " cmd=" << output.ground_velocity_m_s
            << " left=" << output.left_wheel_velocity_rad_s
            << " right=" << output.right_wheel_velocity_rad_s
            << " opt=" << debug.optimal_command_m_s
            << " end_pitch=" << debug.horizon_end_pitch_rad << '\n';
}

}  // namespace

int main() {
  robot_control::MpcBalancer balancer(testConfig());
  robot_control::ControlCommand command;
  robot_control::RobotState state = baseState();

  bool ok = true;

  robot_control::MpcBalancer::StateVec zero_x{};
  const float zero_solve = balancer.solveHorizon(zero_x, 0.0F);
  std::cout << " solveHorizon zero_x u0=" << zero_solve << '\n';
  ok &= expect(std::abs(zero_solve) < 1.0e-5F,
               "solveHorizon returns zero for zero state");

  robot_control::MpcBalancer::StateVec positive_pitch_x{};
  positive_pitch_x[1] = 0.05F;
  const float positive_pitch_solve =
      balancer.solveHorizon(positive_pitch_x, 0.0F);
  std::cout << " solveHorizon +pitch u0=" << positive_pitch_solve << '\n';
  ok &= expect(positive_pitch_solve > 0.0F,
               "solveHorizon positive pitch has corrective forward sign");

  robot_control::MpcBalancer::StateVec negative_pitch_x{};
  negative_pitch_x[1] = -0.05F;
  const float negative_pitch_solve =
      balancer.solveHorizon(negative_pitch_x, 0.0F);
  std::cout << " solveHorizon -pitch u0=" << negative_pitch_solve << '\n';
  ok &= expect(negative_pitch_solve < 0.0F,
               "solveHorizon negative pitch has corrective backward sign");

  robot_control::MpcBalancer::StateVec positive_pitch_rate_x{};
  positive_pitch_rate_x[3] = 0.2F;
  const float positive_pitch_rate_solve =
      balancer.solveHorizon(positive_pitch_rate_x, 0.0F);
  std::cout << " solveHorizon +pitch_rate u0="
            << positive_pitch_rate_solve << '\n';
  ok &= expect(positive_pitch_rate_solve > 0.0F,
               "solveHorizon positive pitch rate has corrective forward sign");

  robot_control::MpcBalancer::StateVec negative_pitch_rate_x{};
  negative_pitch_rate_x[3] = -0.2F;
  const float negative_pitch_rate_solve =
      balancer.solveHorizon(negative_pitch_rate_x, 0.0F);
  std::cout << " solveHorizon -pitch_rate u0="
            << negative_pitch_rate_solve << '\n';
  ok &= expect(negative_pitch_rate_solve < 0.0F,
               "solveHorizon negative pitch rate has corrective backward sign");

  robot_control::MpcBalancer::StateVec positive_position_x{};
  positive_position_x[0] = 0.25F;
  const float positive_position_solve =
      balancer.solveHorizon(positive_position_x, 0.0F);
  std::cout << " solveHorizon +position u0=" << positive_position_solve << '\n';

  robot_control::MpcBalancer::StateVec positive_velocity_x{};
  positive_velocity_x[2] = 0.5F;
  const float positive_velocity_solve =
      balancer.solveHorizon(positive_velocity_x, 0.0F);
  std::cout << " solveHorizon +velocity u0=" << positive_velocity_solve << '\n';

  float max_abs_warm_zero_solve = 0.0F;
  for (int step = 0; step < 30; ++step) {
    const float u = balancer.solveHorizon(zero_x, 0.0F);
    max_abs_warm_zero_solve = std::max(max_abs_warm_zero_solve, std::abs(u));
  }
  std::cout << " solveHorizon warm zero max_u="
            << max_abs_warm_zero_solve << '\n';
  ok &= expect(max_abs_warm_zero_solve < 1.0e-3F,
               "solveHorizon warm start decays at zero state");

  // First upright update starts the balancer from rest.
  balancer.reset(0.0F);
  auto output = balancer.update(state, command, true);
  printStep("seed", balancer, output, state);
  ok &= expect(std::abs(output.ground_velocity_m_s) < 1.0e-5F,
               "upright zero-error state commands zero velocity");

  float max_abs_upright_cmd = 0.0F;
  for (int step = 0; step < 100; ++step) {
    output = balancer.update(state, command, true);
    max_abs_upright_cmd =
        std::max(max_abs_upright_cmd, std::abs(output.ground_velocity_m_s));
  }
  printStep("upright", balancer, output, state);
  ok &= expect(max_abs_upright_cmd < 1.0e-4F,
               "upright repeated updates do not ramp or warm-start runaway");

  state.imu.pitch_rad = 0.05F;
  state.imu.pitch_rate_rad_s = 0.0F;
  output = balancer.update(state, command, true);
  printStep("+pitch", balancer, output, state);
  ok &= expect(output.ground_velocity_m_s > 0.0F,
               "positive pitch commands corrective forward ground velocity");
  ok &= expect(output.left_wheel_velocity_rad_s > 0.0F &&
                   output.right_wheel_velocity_rad_s > 0.0F,
               "positive pitch preserves wheel sign invariant");

  balancer.reset(0.0F);
  state.imu.pitch_rad = -0.05F;
  state.imu.pitch_rate_rad_s = 0.0F;
  output = balancer.update(state, command, true);
  printStep("-pitch", balancer, output, state);
  ok &= expect(output.ground_velocity_m_s < 0.0F,
               "negative pitch commands corrective backward ground velocity");
  ok &= expect(output.left_wheel_velocity_rad_s < 0.0F &&
                   output.right_wheel_velocity_rad_s < 0.0F,
               "negative pitch preserves wheel sign invariant");

  balancer.reset(0.0F);
  state = baseState();
  output = balancer.update(state, command, true);
  float max_abs_after_reset_cmd = std::abs(output.ground_velocity_m_s);
  for (int step = 0; step < 100; ++step) {
    output = balancer.update(state, command, true);
    max_abs_after_reset_cmd =
        std::max(max_abs_after_reset_cmd, std::abs(output.ground_velocity_m_s));
  }
  printStep("reset", balancer, output, state);
  ok &= expect(max_abs_after_reset_cmd < 1.0e-4F,
               "reset followed by standstill remains neutral");

  return ok ? 0 : 1;
}
