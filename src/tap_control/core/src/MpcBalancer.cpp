#include "robot_control/MpcBalancer.hpp"

#include <algorithm>
#include <cmath>

namespace robot_control {

namespace {

constexpr int kCondensedHorizonLimit = 30;

[[nodiscard]] float clampf(float value, float lo, float hi) {
  return std::max(lo, std::min(value, hi));
}

[[nodiscard]] float lowPass(float prev, float cutoff_period_s, float target,
                            float dt_s) {
  if (cutoff_period_s <= 0.0F) {
    return target;
  }
  const float alpha = clampf(dt_s / cutoff_period_s, 0.0F, 1.0F);
  return prev + alpha * (target - prev);
}

[[nodiscard]] MpcBalancer::StateVec matVec(
    const MpcBalancer::StateMatrix& matrix, const MpcBalancer::StateVec& vector) {
  MpcBalancer::StateVec output{};
  for (int row = 0; row < MpcBalancer::kStateDim; ++row) {
    for (int col = 0; col < MpcBalancer::kStateDim; ++col) {
      output[row] += matrix[row][col] * vector[col];
    }
  }
  return output;
}

[[nodiscard]] MpcBalancer::StateMatrix matMul(
    const MpcBalancer::StateMatrix& lhs, const MpcBalancer::StateMatrix& rhs) {
  MpcBalancer::StateMatrix output{};
  for (int row = 0; row < MpcBalancer::kStateDim; ++row) {
    for (int col = 0; col < MpcBalancer::kStateDim; ++col) {
      for (int k = 0; k < MpcBalancer::kStateDim; ++k) {
        output[row][col] += lhs[row][k] * rhs[k][col];
      }
    }
  }
  return output;
}

// Weighted inner product - lets the QP weight [position, pitch, velocity,
// pitch_rate] differently. Position gets a low weight so the balancer will
// spend a metre of travel to save the pitch when the CoM shifts, but a small
// steady pitch error is still opposed instead of drifting forever.
[[nodiscard]] float dotW(const MpcBalancer::StateVec& lhs,
                         const MpcBalancer::StateVec& rhs,
                         const MpcBalancer::StateVec& w) {
  float output = 0.0F;
  for (int i = 0; i < MpcBalancer::kStateDim; ++i) {
    output += w[i] * lhs[i] * rhs[i];
  }
  return output;
}

}  // namespace

MpcBalancer::MpcBalancer() : MpcBalancer(Config{}) {}

MpcBalancer::MpcBalancer(Config config) : config_(config) {
  active_length_m_ = config_.leg_length_m;
  buildModel();
  reset();
}

void MpcBalancer::setComHeight(float com_height_m) {
  const float target =
      std::clamp(config_.pendulum_length_per_com_height * com_height_m,
                 config_.min_pendulum_length_m, config_.max_pendulum_length_m);
  // Rebuild only after a meaningful change of the short TAP pendulum arm.
  if (std::abs(target - active_length_m_) > 0.003F) {
    active_length_m_ = target;
    buildModel();
  }
}

void MpcBalancer::reset(float ground_position_m, bool floor_contact) {
  commanded_ground_velocity_m_s_ = 0.0F;
  ground_position_target_m_ = ground_position_m;
  u_warm_.fill(0.0F);
  prev_floor_contact_ = floor_contact;
  landing_window_remaining_s_ = 0.0F;
  last_debug_ = DebugState{};
}

void MpcBalancer::buildModel() {
  n_ = std::clamp(config_.nb_timesteps, 4, kMaxHorizon);

  const float length = std::max(active_length_m_, 0.02F);
  const float gravity = std::max(config_.gravity_m_s2, 0.1F);
  const float dt = std::max(config_.sampling_period_s, 1.0e-4F);
  const float omega = std::sqrt(gravity / length);
  const float c = std::cosh(dt * omega);
  const float s = std::sinh(dt * omega);

  for (auto& row : a_d_) {
    row.fill(0.0F);
  }
  a_d_[0][0] = 1.0F;
  a_d_[0][2] = dt;
  a_d_[1][1] = c;
  a_d_[1][3] = s / omega;
  a_d_[2][2] = 1.0F;
  a_d_[3][1] = omega * s;
  a_d_[3][3] = c;

  b_d_.fill(0.0F);
  b_d_[0] = 0.5F * dt * dt;
  b_d_[1] = (1.0F - c) / gravity;
  b_d_[2] = dt;
  b_d_[3] = -omega * s / gravity;

  StateMatrix power{};
  for (int i = 0; i < kStateDim; ++i) {
    power[i][i] = 1.0F;
  }

  for (int k = 0; k < n_; ++k) {
    impulse_[k] = matVec(power, b_d_);
    power = matMul(a_d_, power);
  }

  const float wu = std::max(config_.stage_input_cost_weight, 1.0e-6F);
  const float wx = std::max(config_.stage_state_cost_weight, 0.0F);
  const float terminal = std::max(config_.terminal_cost_weight, 0.0F);
  state_cost_ = {std::max(config_.position_cost_scale, 0.0F), 1.0F,
                 std::max(config_.velocity_cost_scale, 0.0F), 1.0F};
  for (auto& row : hessian_) {
    row.fill(0.0F);
  }
  // Adjacent entries on each diagonal share all but one stage-cost term.
  for (int offset = 0; offset < n_; ++offset) {
    float stage_sum = 0.0F;
    for (int i = n_ - 1 - offset; i >= 0; --i) {
      const int j = i + offset;
      if (n_ <= kCondensedHorizonLimit && j < n_ - 1) {
        stage_sum += wx * dotW(impulse_[n_-2-i], impulse_[n_-2-j], state_cost_);
      }
      float value = (i == j) ? wu : 0.0F;
      value += terminal * dotW(impulse_[n_-1-i], impulse_[n_-1-j], state_cost_);
      if (n_ <= kCondensedHorizonLimit) {
        value += stage_sum;
      } else {
        // Long unstable predictions amplify float reassociation. Retain the
        // original summation order outside the embedded short-horizon path.
        for (int k = j + 1; k < n_; ++k) {
          value += wx * dotW(impulse_[k-1-i], impulse_[k-1-j], state_cost_);
        }
      }
      hessian_[i][j] = value;
      hessian_[j][i] = value;
    }
  }

  for (auto& map : gradient_map_) map.fill(0.0F);
  power = {};
  for (int d = 0; d < kStateDim; ++d) power[d][d] = 1.0F;
  for (int k = 1; n_ <= kCondensedHorizonLimit && k <= n_; ++k) {
    power = matMul(a_d_, power);
    const float weight = k == n_ ? terminal : wx;
    for (int j = 0; j < k; ++j) {
      auto& map = gradient_map_[j];
      const auto& response = impulse_[k-1-j];
      for (int d = 0; d < kStateDim; ++d) {
        for (int row = 0; row < kStateDim; ++row) {
          map[d] += weight * state_cost_[row] * response[row] * power[row][d];
        }
      }
      map[4] -= weight * state_cost_[0] * response[0];
      map[5] -= weight * (state_cost_[0] * response[0] * (k * dt) +
                           state_cost_[2] * response[2]);
    }
  }

  hessian_row_bound_ = 1.0e-3F;
  for (int i = 0; i < n_; ++i) {
    float row_sum = 0.0F;
    for (int j = 0; j < n_; ++j) {
      row_sum += std::abs(hessian_[i][j]);
    }
    hessian_row_bound_ = std::max(hessian_row_bound_, row_sum);
  }
}

float MpcBalancer::solveHorizon(const StateVec& x0,
                                float target_ground_velocity_m_s,
                                float pitch_ref_rad,
                                float accel_limit_boost_m_s2) {
  const float friction_accel_limit_m_s2 =
      std::max(config_.wheel_ground_mu, 0.0F) * std::max(config_.gravity_m_s2, 0.0F);
  const float accel_limit = std::min(
      std::max(config_.max_ground_accel_m_s2 + std::max(accel_limit_boost_m_s2, 0.0F),
               0.1F),
      std::max(friction_accel_limit_m_s2, 0.1F));

  bool near_ref_state = std::abs(target_ground_velocity_m_s) < 1.0e-6F &&
                        std::abs(x0[0] - ground_position_target_m_) < 1.0e-6F &&
                        std::abs(x0[1] - pitch_ref_rad) < 1.0e-6F &&
                        std::abs(x0[2]) < 1.0e-6F && std::abs(x0[3]) < 1.0e-6F;
  if (near_ref_state) {
    u_warm_.fill(0.0F);
    last_debug_.horizon_end_pitch_rad = pitch_ref_rad;
    last_debug_.solver_iterations = 0;
    return 0.0F;
  }

  // Linearise about the live equilibrium: gravity acts on pitch - pitch_ref.
  StateVec relative = x0;
  relative[1] -= pitch_ref_rad;

  // Position anchor: without one, any small steady-state pitch error (e.g. a few
  // mm of residual CoM-offset estimate) makes the robot creep away forever. The
  // anchor is held fixed at reset and only walked by drive commands (see
  // update()), clamped near the robot so an excursion cannot build a huge
  // restoring force.
  const float anchor = clampf(ground_position_target_m_,
                              x0[0] - config_.max_target_distance_m,
                              x0[0] + config_.max_target_distance_m);

  std::array<float, kMaxHorizon> gradient{};
  for (int j = 0; n_ <= kCondensedHorizonLimit && j < n_; ++j) {
    const auto& map = gradient_map_[j];
    for (int d = 0; d < kStateDim; ++d) {
      gradient[j] += map[d] * relative[d];
    }
    gradient[j] += map[4] * anchor + map[5] * target_ground_velocity_m_s;
  }
  if (n_ > kCondensedHorizonLimit) {
    const float dt = std::max(config_.sampling_period_s, 1.0e-4F);
    const float wx = std::max(config_.stage_state_cost_weight, 0.0F);
    const float terminal = std::max(config_.terminal_cost_weight, 0.0F);
    StateMatrix power{};
    for (int d = 0; d < kStateDim; ++d) power[d][d] = 1.0F;
    for (int k = 0; k < n_; ++k) power = matMul(a_d_, power);
    auto error = matVec(power, relative);
    error[1] += pitch_ref_rad;
    error[0] -= anchor + n_ * dt * target_ground_velocity_m_s;
    error[1] -= pitch_ref_rad;
    error[2] -= target_ground_velocity_m_s;
    for (int j = 0; j < n_; ++j)
      gradient[j] = terminal * dotW(error, impulse_[n_-1-j], state_cost_);
    power = {};
    for (int d = 0; d < kStateDim; ++d) power[d][d] = 1.0F;
    for (int k = 0; k < n_; ++k) {
      error = matVec(power, relative);
      error[1] += pitch_ref_rad;
      error[0] -= anchor + k * dt * target_ground_velocity_m_s;
      error[1] -= pitch_ref_rad;
      error[2] -= target_ground_velocity_m_s;
      for (int j = 0; j < k; ++j)
        gradient[j] += wx * dotW(error, impulse_[k-1-j], state_cost_);
      power = matMul(a_d_, power);
    }
  }

  std::array<float, kMaxHorizon> u{};
  for (int i = 0; i < n_; ++i) {
    u[i] = clampf(u_warm_[i], -accel_limit, accel_limit);
  }

  const float step = 1.0F / hessian_row_bound_;
  const int iters = std::clamp(config_.solver_iterations, 5, 500);
  for (int it = 0; it < iters; ++it) {
    for (int i = 0; i < n_; ++i) {
      float grad = gradient[i];
      for (int j = 0; j < n_; ++j) {
        grad += hessian_[i][j] * u[j];
      }
      u[i] = clampf(u[i] - step * grad, -accel_limit, accel_limit);
    }
  }

  for (int i = 0; i < n_; ++i) {
    u_warm_[i] = u[i];
  }

  StateVec x = relative;
  for (int k = 0; k < n_; ++k) {
    const StateVec ax = matVec(a_d_, x);
    for (int d = 0; d < kStateDim; ++d) {
      x[d] = ax[d] + b_d_[d] * u[k];
    }
  }
  last_debug_.horizon_end_pitch_rad = x[1] + pitch_ref_rad;
  last_debug_.solver_iterations = iters;

  float ground_accel = u[0];
  // See Config::nonlinear_gravity_correction_gain's own comment for the
  // derivation. `relative[1]` is theta_err = pitch - pitch_ref at the real
  // current tick (never reassigned above - only the separate `x` trajectory
  // var is forward-simulated), exactly the operand the correction needs.
  if (config_.nonlinear_gravity_correction_gain > 0.0F) {
    const float theta_err = relative[1];
    const float cos_err = std::cos(theta_err);
    constexpr float kCosFloor = 0.15F;  // ~81.4 deg: real actuation singularity guard.
    if (std::abs(cos_err) > kCosFloor) {
      const float gravity = std::max(config_.gravity_m_s2, 0.1F);
      const float u_real =
          (ground_accel + gravity * (std::sin(theta_err) - theta_err)) / cos_err;
      const float corrected = ground_accel +
          config_.nonlinear_gravity_correction_gain * (u_real - ground_accel);
      ground_accel = clampf(corrected, -accel_limit, accel_limit);
    }
  }

  return ground_accel;
}

MpcBalancer::Output MpcBalancer::update(const RobotState& state,
                                        const ControlCommand& command,
                                        bool integrate,
                                        float balance_pitch_ref_rad,
                                        bool hold_station) {
  Output output{};

  if (!std::isfinite(state.dt_s) || state.dt_s <= 0.0F ||
      !std::isfinite(state.imu.pitch_rad) ||
      !std::isfinite(state.imu.pitch_rate_rad_s) ||
      !std::isfinite(state.wheel.position_m) ||
      !std::isfinite(state.wheel.velocity_m_s) ||
      !std::isfinite(command.target_forward_velocity_m_s) ||
      !std::isfinite(command.target_yaw_rate_rad_s) ||
      !std::isfinite(balance_pitch_ref_rad)) {
    reset();
    output.fallen = true;
    return output;
  }

  const float dt_s = std::clamp(state.dt_s, 1.0e-6F, 0.05F);
  const float forward = clampf(command.target_forward_velocity_m_s,
      -config_.max_ground_velocity_m_s, config_.max_ground_velocity_m_s);
  const float yaw = state.floor_contact && integrate ?
      clampf(command.target_yaw_rate_rad_s,
             -config_.max_yaw_rate_rad_s, config_.max_yaw_rate_rad_s) : 0.0F;
  const float pitch_rad = state.imu.pitch_rad;

  // Edge-triggered landing window: starts the instant floor_contact flips
  // false->true (the actual touchdown tick) and counts down regardless of what
  // happens afterward, so a single missed contact tick mid-window cannot reset
  // it early. landing_window_s alone changes nothing without a nonzero boost.
  const bool touchdown_edge = state.floor_contact && !prev_floor_contact_;
  prev_floor_contact_ = state.floor_contact;
  landing_window_remaining_s_ = touchdown_edge
      ? std::max(config_.landing_window_s, 0.0F)
      : std::max(landing_window_remaining_s_ - dt_s, 0.0F);
  const bool landing_active = landing_window_remaining_s_ > 0.0F;

  last_debug_ = DebugState{};
  last_debug_.commanded_ground_velocity_m_s = commanded_ground_velocity_m_s_;
  last_debug_.target_ground_position_m = state.wheel.position_m;
  last_debug_.balance_pitch_ref_rad = balance_pitch_ref_rad;
  last_debug_.landing_window_remaining_s = landing_window_remaining_s_;
  // Fall is measured against the leaned target, not upright, so a deep crouch
  // (which needs a real lean) does not trip the fall check on its own.
  if (std::abs(pitch_rad - balance_pitch_ref_rad) > config_.fall_pitch_rad) {
    output.fallen = true;
    return output;
  }

  if (!integrate || !state.floor_contact) {
    const bool hold_landing_velocity =
        integrate && !state.floor_contact && landing_active &&
        config_.landing_hold_airborne_velocity;
    if (!hold_landing_velocity) {
      commanded_ground_velocity_m_s_ =
          lowPass(commanded_ground_velocity_m_s_, 0.1F, 0.0F, dt_s);
    }
    // Off-contact: let the anchor follow the robot so it does not snap back on
    // re-contact.
    ground_position_target_m_ = state.wheel.position_m;
  } else {
    // Position anchor. Sticky only while holding station (then it opposes a
    // steady drift); otherwise it follows the wheels so the balancer can roll
    // freely - e.g. forward to catch a CoM that a leg bend just shifted, or
    // wherever a drive command takes it.
    if (hold_station && config_.position_anchor_hold) {
      ground_position_target_m_ = clampf(
          ground_position_target_m_ +
              forward * dt_s,
          state.wheel.position_m - config_.max_target_distance_m,
          state.wheel.position_m + config_.max_target_distance_m);
    } else {
      ground_position_target_m_ = state.wheel.position_m;
    }
    const StateVec x0 = {state.wheel.position_m, pitch_rad,
                         state.wheel.velocity_m_s,
                         state.imu.pitch_rate_rad_s};
    const float ground_accel_m_s2 =
        solveHorizon(x0, forward, balance_pitch_ref_rad,
                     landing_active ? config_.landing_ground_accel_boost_m_s2
                                    : 0.0F);
    commanded_ground_velocity_m_s_ =
        commanded_ground_velocity_m_s_ + ground_accel_m_s2 * dt_s;
    // Slow outer station-keeping loop. It only runs while holding station (no
    // drive input, legs not moving), so it cannot fight the transient when a
    // pose change deliberately rolls the robot; but once settled it blends the
    // ground-velocity command toward a gentle position-return velocity, which
    // sheds the drift the MPC's own weak position authority leaves behind.
    if (hold_station && config_.station_hold_gain_hz > 0.0F) {
      const float return_v =
          clampf(config_.station_hold_gain_hz *
                     (ground_position_target_m_ - state.wheel.position_m),
                 -config_.station_hold_max_v, config_.station_hold_max_v);
      const float alpha = clampf(
          dt_s / std::max(config_.station_hold_period_s, 1.0e-3F), 0.0F, 1.0F);
      commanded_ground_velocity_m_s_ +=
          alpha * (return_v - commanded_ground_velocity_m_s_);
    }
    commanded_ground_velocity_m_s_ =
        clampf(commanded_ground_velocity_m_s_, -config_.max_ground_velocity_m_s,
               config_.max_ground_velocity_m_s);
    last_debug_.optimal_command_m_s = ground_accel_m_s2;
  }

  output.ground_velocity_m_s = commanded_ground_velocity_m_s_;

  const float s_sign = config_.left_wheel_sign;
  const float wheel_radius = std::max(config_.wheel_radius_m, 1.0e-4F);
  const float ref_wheel_velocity_rad_s =
      output.ground_velocity_m_s / wheel_radius;
  const float yaw_to_wheel = s_sign * config_.half_wheel_base_m / wheel_radius;
  // Close the loop on real yaw disturbance: `yaw` above is the commanded
  // rate alone (open-loop feed-forward); measured gyro-Z yaw rate is
  // available every tick via state.imu.yaw_rate_rad_s but was never
  // consulted before. Only active while grounded and integrating, matching
  // `yaw` itself above - airborne/disabled yaw rate is real but this wheel
  // differential cannot do anything about it (no ground contact to push
  // against), so correcting it would just wind up for nothing.
  const float yaw_command =
      (state.floor_contact && integrate && std::isfinite(state.imu.yaw_rate_rad_s))
          ? yaw + config_.yaw_rate_feedback_kp * (yaw - state.imu.yaw_rate_rad_s)
          : yaw;
  output.left_wheel_velocity_rad_s = s_sign * ref_wheel_velocity_rad_s -
                                     yaw_to_wheel * yaw_command;
  output.right_wheel_velocity_rad_s =
      s_sign * ref_wheel_velocity_rad_s + yaw_to_wheel * yaw_command;

  const float turning_prob =
      (std::abs(yaw) > config_.stiff_yaw_rate_rad_s)
          ? 1.0F
          : 0.0F;
  // kp keeps the existing turning-stiffness coupling. kd_scale was dead
  // weight until publishLegCommand() started consuming it for landing damping
  // (see Config::landing_kd_boost), so it deliberately does NOT also pick up
  // the turning boost: coupling it there would have made turning silently
  // start applying active damping in production the moment kd_scale went
  // live, an untested side effect outside this change's landing scope. Both
  // boosts default to 0, so a config that never sets them is byte-identical
  // to the pre-landing behavior.
  const float landing_kp = landing_active ? config_.landing_kp_boost : 0.0F;
  const float landing_kd = landing_active ? config_.landing_kd_boost : 0.0F;
  output.leg_kp_scale = config_.leg_kp_scale +
      config_.turning_stiffness_boost * turning_prob + landing_kp;
  output.leg_kd_scale = config_.leg_kd_scale + landing_kd;

  last_debug_.commanded_ground_velocity_m_s = commanded_ground_velocity_m_s_;
  last_debug_.target_ground_position_m =
      ground_position_target_m_;
  return output;
}

}  // namespace robot_control
