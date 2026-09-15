#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>

#include "mpc_reference/MpcBalancer.hpp"
#include "robot_control/MpcBalancer.hpp"

namespace {
using Current = robot_control::MpcBalancer;
using Reference = reference_control::MpcBalancer;
int horizon = 0;
int variant = 0;
int sample = 0;
int comparisons = 0;
float world_offset_m = 0.0F;
float max_accel_error = 0.0F;
float max_velocity_error = 0.0F;

void require(bool condition, const char* field) {
  if (!condition) {
    std::cerr << "FAIL horizon=" << horizon << " variant=" << variant
              << " world_offset_m=" << world_offset_m
              << " sample=" << sample << " field=" << field << '\n';
    std::exit(EXIT_FAILURE);
  }
}

// Reordered float sums need not be bitwise identical. Absolute limits are
// 0.3 mm/s ground speed, 0.01 rad/s wheels (same linear tolerance), and
// 0.001 m/s^2 first acceleration. The long unstable N80 open-loop prediction
// uses 0.003 rad + 0.03% relative tolerance to allow cancellation error.
// Flags, iteration counts and non-solver configuration remain exact.
void near(float actual, float expected, const char* field, float absolute,
          float relative = 0.0F) {
  ++comparisons;
  if (!std::isfinite(actual) || !std::isfinite(expected) ||
      std::abs(actual - expected) > absolute + relative * std::abs(expected)) {
    std::cerr << "actual=" << actual << " reference=" << expected
              << " error=" << std::abs(actual - expected) << '\n';
    require(false, field);
  }
}

template <class Config>
Config configFor(int n, int mode) {
  Config config;
  config.nb_timesteps = n;
  config.solver_iterations = 120;
  if (mode == 3) {
    // Simulation N20 profile, with the requested 0.125 m minimum (the YAML
    // inspected when adding this test still specified 0.15 m).
    config.leg_length_m = 0.234F;
    config.pendulum_length_per_com_height = 1.077F;
    config.min_pendulum_length_m = 0.125F;
    config.max_pendulum_length_m = 0.27F;
    config.position_cost_scale = 1.5F;
    config.velocity_cost_scale = 1.0F;
    config.max_ground_accel_m_s2 = 6.0F;
    config.max_ground_velocity_m_s = 0.6F;
    config.fall_pitch_rad = 0.8F;
  } else if (mode != 0) {
    config.position_anchor_hold = true;
    config.position_cost_scale = 0.08F;
    config.velocity_cost_scale = 0.35F;
    config.station_hold_gain_hz = mode == 2 ? 0.2F : 0.0F;
    config.left_wheel_sign = mode == 2 ? -1.0F : 1.0F;
    config.max_ground_accel_m_s2 = 0.7F;
    config.max_ground_velocity_m_s = 0.12F;
  }
  return config;
}

void compare(Current& current, Reference& reference,
             const robot_control::RobotState& state,
             const robot_control::ControlCommand& command,
             bool integrate, float pitch_ref, bool hold) {
  const auto a = current.update(state, command, integrate, pitch_ref, hold);
  const auto b = reference.update(state, command, integrate, pitch_ref, hold);
  near(a.ground_velocity_m_s, b.ground_velocity_m_s, "ground velocity", 3e-4F);
  max_velocity_error = std::max(max_velocity_error,
      std::abs(a.ground_velocity_m_s - b.ground_velocity_m_s));
  near(a.left_wheel_velocity_rad_s, b.left_wheel_velocity_rad_s, "left wheel", 0.01F);
  near(a.right_wheel_velocity_rad_s, b.right_wheel_velocity_rad_s, "right wheel", 0.01F);
  near(a.leg_kp_scale, b.leg_kp_scale, "leg kp", 0.0F);
  near(a.leg_kd_scale, b.leg_kd_scale, "leg kd", 0.0F);
  require(a.fallen == b.fallen, "fallen");
  const auto da = current.debugState();
  const auto db = reference.debugState();
  near(da.commanded_ground_velocity_m_s, db.commanded_ground_velocity_m_s,
       "debug velocity", 3e-4F);
  near(da.target_ground_position_m, db.target_ground_position_m, "anchor", 1e-6F);
  near(da.optimal_command_m_s, db.optimal_command_m_s, "acceleration", 1e-3F);
  max_accel_error = std::max(max_accel_error,
      std::abs(da.optimal_command_m_s - db.optimal_command_m_s));
  near(da.horizon_end_pitch_rad, db.horizon_end_pitch_rad, "end pitch", 3e-3F, 3e-4F);
  near(da.balance_pitch_ref_rad, db.balance_pitch_ref_rad, "pitch ref", 0.0F);
  require(da.solver_iterations == db.solver_iterations, "solver iterations");
  near(current.activePendulumLengthM(), reference.activePendulumLengthM(),
       "active length", 0.0F);
}

void sequence() {
  const auto config = configFor<Current::Config>(horizon, variant);
  // The deliberately large frozen implementation belongs on the host heap.
  auto current = std::make_unique<Current>(config);
  auto reference = std::make_unique<Reference>(configFor<Reference::Config>(horizon, variant));
  int saturated_acceleration = 0;
  int saturated_velocity = 0;
  int solved = 0;
  const float heights[] = {0.218F, 0.219F, 0.17F, 0.171F, 0.24F, 0.1F, 0.4F};
  for (sample = 0; sample < 360; ++sample) {
    if (sample % 19 == 0) {
      const float height = heights[(sample / 19) % 7];
      current->setComHeight(height);
      reference->setComHeight(height);
    }
    if (sample == 0 || sample == 173 || sample == 291) {
      const float position = world_offset_m + (sample == 0 ? 0.0F : -0.23F);
      current->reset(position);
      reference->reset(position);
      near(current->debugState().optimal_command_m_s, 0.0F, "reset debug", 0.0F);
    }
    robot_control::RobotState state;
    robot_control::ControlCommand command;
    const float t = static_cast<float>(sample);
    state.dt_s = sample % 17 == 0 ? 0.08F : (sample % 13 == 0 ? 1e-7F : 0.005F);
    state.wheel.position_m = world_offset_m + 0.17F * std::sin(t * 0.037F);
    state.wheel.velocity_m_s = 0.08F * std::cos(t * 0.091F);
    const float pitch_ref = 0.07F * std::sin(t * 0.021F);
    state.imu.pitch_rad = pitch_ref + 0.055F * std::sin(t * 0.071F) + 0.012F;
    state.imu.pitch_rate_rad_s = 0.13F * std::cos(t * 0.053F);
    command.target_forward_velocity_m_s = sample % 60 < 30 ? 0.0F : 0.7F * std::sin(t * 0.031F);
    command.target_yaw_rate_rad_s = 1.6F * std::sin(t * 0.11F);
    state.floor_contact = sample % 53 >= 4;
    bool integrate = sample % 47 >= 3;
    // Sustained positive/negative disturbances exercise both box limits and
    // accumulated velocity clipping without a plant model masking divergence.
    if (sample >= 90 && sample < 170) {
      state.imu.pitch_rad = pitch_ref + (sample < 130 ? 0.55F : -0.55F);
      state.dt_s = 0.02F;
      state.floor_contact = true;
      integrate = true;
    }
    if (sample == 250) state.imu.pitch_rad = pitch_ref + config.fall_pitch_rad + 0.15F;
    compare(*current, *reference, state, command, integrate, pitch_ref, sample % 60 < 30);
    const auto debug = reference->debugState();
    solved += debug.solver_iterations == 120;
    saturated_acceleration += std::abs(debug.optimal_command_m_s) >= config.max_ground_accel_m_s2 - 1e-5F;
    saturated_velocity += std::abs(debug.commanded_ground_velocity_m_s) >= config.max_ground_velocity_m_s - 1e-5F;
  }
  require(solved > 200, "non-equilibrium solver coverage");
  // Existing low-limit profiles enforce saturation coverage. The simulation
  // profile targets cancellation at its unchanged, higher configured limits.
  if (variant != 3) {
    require(saturated_acceleration > 0, "acceleration saturation coverage");
    require(saturated_velocity > 0, "velocity saturation coverage");
  }

  robot_control::RobotState state;
  robot_control::ControlCommand command;
  state.wheel.position_m = world_offset_m;
  current->reset(world_offset_m);
  reference->reset(world_offset_m);
  compare(*current, *reference, state, command, true, 0.0F, true);
  require(current->debugState().solver_iterations == 0, "equilibrium fastpath");
  state.imu.pitch_rad = 0.03F;
  compare(*current, *reference, state, command, true, 0.02F, false);
  state.dt_s = std::numeric_limits<float>::quiet_NaN();
  compare(*current, *reference, state, command, true, 0.0F, false);
  state.dt_s = 0.005F;
  compare(*current, *reference, state, command, true, 0.0F, false);
  current->setFallPitchRad(0.02F);
  reference->setFallPitchRad(0.02F);
  near(current->fallPitchRad(), reference->fallPitchRad(), "fall config", 0.0F);
  compare(*current, *reference, state, command, true, 0.0F, false);
}
}  // namespace

int main() {
  for (const int n : {4, 20, 30, 31, 80}) {
    if (n > Current::kMaxHorizon) continue;
    horizon = n;
    for (variant = 0; variant < (n == 20 ? 4 : 3); ++variant) {
      for (const float offset : {0.0F, -10.0F, 10.0F}) {
        world_offset_m = offset;
        sequence();
      }
    }
  }
  std::cout << "MPC equivalence passed: " << comparisons
            << " comparisons; max_accel_error=" << max_accel_error
            << " max_velocity_error=" << max_velocity_error << '\n';
}
