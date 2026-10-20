#include "robot_control/LqrBalancer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace robot_control {

namespace {

// The DARE below has a defective (Jordan-block) eigenvalue at exactly 1.0 -
// the position/velocity pair is an undamped double integrator - paired with
// a deliberately weak position cost. That combination is barely detectable,
// and the textbook Riccati *value* iteration (P_{k+1} = Q + A'PA - ...,
// started from P0) only reaches a stabilizing gain after ~1e3 iterations for
// this plant, by which point float/double round-off in the repeated outer
// products has already pushed P past its fixed point into unbounded growth -
// P doubles roughly every ~200 further iterations once it starts drifting,
// so a K read off *any* fixed iteration count is a coin flip between
// "not yet stabilizing" and "numerically exploded", and 80 iterations (the
// robot's config) lands on the wrong side (max|eig(A-BK)| computed offline:
// 1.0005, an unstable closed loop). No amount of cost tuning fixes that,
// because the failure is in the solver rather than in the weights.
//
// The structure-preserving doubling algorithm (SDA; Chu/Fan/Lin 2005) solves
// the same DARE without that failure mode: it needs no initial stabilizing
// gain, only a handful of iterations (quadratic convergence - each one
// roughly doubles the correct digits), and it sits at its fixed point
// instead of drifting past it, so running it longer than necessary is
// harmless. All SDA algebra runs in double for headroom; only the final gain
// is narrowed to float.
using Matrix = std::array<std::array<double, LqrBalancer::kStateDim>,
                          LqrBalancer::kStateDim>;
using Vector = std::array<double, LqrBalancer::kStateDim>;
constexpr int kDim = LqrBalancer::kStateDim;

[[nodiscard]] Matrix identity() {
  Matrix out{};
  for (int i = 0; i < kDim; ++i) out[i][i] = 1.0;
  return out;
}

[[nodiscard]] Matrix add(const Matrix& lhs, const Matrix& rhs) {
  Matrix out{};
  for (int r = 0; r < kDim; ++r)
    for (int c = 0; c < kDim; ++c) out[r][c] = lhs[r][c] + rhs[r][c];
  return out;
}

[[nodiscard]] Matrix transpose(const Matrix& in) {
  Matrix out{};
  for (int r = 0; r < kDim; ++r)
    for (int c = 0; c < kDim; ++c) out[r][c] = in[c][r];
  return out;
}

[[nodiscard]] Matrix multiply(const Matrix& lhs, const Matrix& rhs) {
  Matrix out{};
  for (int r = 0; r < kDim; ++r)
    for (int c = 0; c < kDim; ++c)
      for (int k = 0; k < kDim; ++k) out[r][c] += lhs[r][k] * rhs[k][c];
  return out;
}

[[nodiscard]] Vector multiply(const Matrix& lhs, const Vector& rhs) {
  Vector out{};
  for (int r = 0; r < kDim; ++r)
    for (int c = 0; c < kDim; ++c) out[r] += lhs[r][c] * rhs[c];
  return out;
}

[[nodiscard]] double dot(const Vector& lhs, const Vector& rhs) {
  double out = 0.0;
  for (int i = 0; i < kDim; ++i) out += lhs[i] * rhs[i];
  return out;
}

// Gauss-Jordan with partial pivoting. Only ever called on (I + G*H), which
// is symmetric positive definite for this DARE, so it is always invertible;
// a pivot of exactly 0 cannot occur for a well-posed plant/cost pair.
[[nodiscard]] Matrix invert(const Matrix& in) {
  std::array<std::array<double, 2 * kDim>, kDim> aug{};
  for (int r = 0; r < kDim; ++r) {
    for (int c = 0; c < kDim; ++c) aug[r][c] = in[r][c];
    aug[r][kDim + r] = 1.0;
  }
  for (int col = 0; col < kDim; ++col) {
    int pivot = col;
    for (int r = col + 1; r < kDim; ++r)
      if (std::abs(aug[r][col]) > std::abs(aug[pivot][col])) pivot = r;
    std::swap(aug[col], aug[pivot]);
    const double p = aug[col][col];
    const double inv_p = std::abs(p) > 1.0e-300 ? 1.0 / p : 0.0;
    for (int c = 0; c < 2 * kDim; ++c) aug[col][c] *= inv_p;
    for (int r = 0; r < kDim; ++r) {
      if (r == col) continue;
      const double factor = aug[r][col];
      for (int c = 0; c < 2 * kDim; ++c) aug[r][c] -= factor * aug[col][c];
    }
  }
  Matrix out{};
  for (int r = 0; r < kDim; ++r)
    for (int c = 0; c < kDim; ++c) out[r][c] = aug[r][kDim + c];
  return out;
}

}  // namespace

LqrBalancer::LqrBalancer() : LqrBalancer(Config{}) {}

LqrBalancer::LqrBalancer(Config config) : config_(config) {
  active_length_m_ = std::clamp(config_.leg_length_m,
                                config_.min_pendulum_length_m,
                                config_.max_pendulum_length_m);
  rebuildGain();
  reset();
}

float LqrBalancer::clampFinite(float value, float lo, float hi) noexcept {
  return std::isfinite(value) ? std::clamp(value, lo, hi) : 0.0F;
}

void LqrBalancer::rebuildGain() {
  if (!table_ready_) {
    solveGainsAt(active_length_m_, gain_, station_gain_, posture_station_gain_);
    ++online_solves_;
    return;
  }
  // Linear interpolation in pendulum length between precomputed solves.
  const float lo = config_.min_pendulum_length_m;
  const float hi = std::max(config_.max_pendulum_length_m, lo);
  const float span = hi - lo;
  const float position = span > 0.0F
      ? std::clamp((active_length_m_ - lo) / span, 0.0F, 1.0F) *
            static_cast<float>(table_points_ - 1U)
      : 0.0F;
  const unsigned index = std::min(static_cast<unsigned>(position), table_points_ - 2U);
  const float frac = position - static_cast<float>(index);
  for (int i = 0; i < kStateDim; ++i) {
    gain_[i] = table_gain_[index][i] +
        frac * (table_gain_[index + 1U][i] - table_gain_[index][i]);
    station_gain_[i] = table_station_gain_[index][i] +
        frac * (table_station_gain_[index + 1U][i] - table_station_gain_[index][i]);
    posture_station_gain_[i] = table_posture_station_gain_[index][i] +
        frac * (table_posture_station_gain_[index + 1U][i] -
                table_posture_station_gain_[index][i]);
  }
}

bool LqrBalancer::buildGainTableStep(unsigned points) {
  if (table_ready_) return true;
  points = std::clamp(points, 2U, kMaxGainTablePoints);
  if (table_points_ != points) {
    table_points_ = points;
    table_built_ = 0U;
  }
  const float lo = config_.min_pendulum_length_m;
  const float hi = std::max(config_.max_pendulum_length_m, lo);
  const float length = lo + (hi - lo) * static_cast<float>(table_built_) /
      static_cast<float>(points - 1U);
  solveGainsAt(length, table_gain_[table_built_], table_station_gain_[table_built_],
               table_posture_station_gain_[table_built_]);
  if (++table_built_ >= points) {
    table_ready_ = true;
    rebuildGain();
  }
  return table_ready_;
}

void LqrBalancer::solveGainsAt(float length_m, GainVec& gain,
                               GainVec& station_gain,
                               GainVec& posture_station_gain) const {
  const float length = std::max(length_m, 0.02F);
  const float gravity = std::max(config_.gravity_m_s2, 0.1F);
  const float dt = std::clamp(config_.sampling_period_s, 1.0e-4F, 0.05F);
  const float omega = std::sqrt(gravity / length);
  const double c = std::cosh(static_cast<double>(dt) * omega);
  const double s = std::sinh(static_cast<double>(dt) * omega);
  Matrix a{};
  a[0][0] = 1.0;
  a[0][2] = dt;
  a[1][1] = c;
  a[1][3] = s / omega;
  a[2][2] = 1.0;
  a[3][1] = omega * s;
  a[3][3] = c;
  Vector b{0.5 * dt * dt, (1.0 - c) / gravity, dt, -omega * s / gravity};
  const Matrix at = transpose(a);
  const Matrix i4 = identity();

  // SDA doubling iterations; quadratic convergence reaches machine precision
  // in well under 30 steps for this plant, and (unlike value iteration) the
  // sequence sits at its fixed point rather than drifting past it, so this
  // clamp only needs to be "enough", not "exact" - see the note above.
  const unsigned iterations = std::clamp(config_.riccati_iterations, 8U, 40U);
  const auto solve_gain = [&](float position_cost, float pitch_cost,
                              float velocity_cost, float rate_cost,
                              float input_cost) {
    Matrix q{};
    q[0][0] = std::max(position_cost, 0.0F);
    q[1][1] = std::max(pitch_cost, 0.0F);
    q[2][2] = std::max(velocity_cost, 0.0F);
    q[3][3] = std::max(rate_cost, 0.0F);
    const double r = std::max(input_cost, 1.0e-4F);
    Matrix g{};
    for (int row = 0; row < kStateDim; ++row)
      for (int col = 0; col < kStateDim; ++col)
        g[row][col] = b[row] * b[col] / r;
    Matrix h = q;
    Matrix ak = a;
    for (unsigned used = 0U; used < iterations; ++used) {
      const Matrix m = invert(add(i4, multiply(g, h)));
      const Matrix akm = multiply(ak, m);
      const Matrix ak_next = multiply(akm, ak);
      const Matrix g_next = add(g, multiply(akm, multiply(g, transpose(ak))));
      const Matrix h_next = add(h, multiply(multiply(transpose(ak), h),
                                            multiply(m, ak)));
      ak = ak_next;
      g = g_next;
      h = h_next;
    }
    // K = (R + B'XB)^-1 B'XA, X = h (the DARE's stabilizing solution).
    const Vector hb = multiply(h, b);
    const double denom = r + dot(b, hb);
    const Vector bt_h_a = multiply(at, hb);
    GainVec result{};
    for (int i = 0; i < kStateDim; ++i) {
      const double gain = bt_h_a[i] / denom;
      result[i] = std::isfinite(gain) ? static_cast<float>(gain) : 0.0F;
    }
    return result;
  };

  gain = solve_gain(config_.position_cost_scale, config_.pitch_cost_scale,
                    config_.velocity_cost_scale, config_.pitch_rate_cost_scale,
                    config_.input_cost_scale);
  station_gain = solve_gain(
      config_.station_position_cost_scale, config_.station_pitch_cost_scale,
      config_.station_velocity_cost_scale,
      config_.station_pitch_rate_cost_scale, config_.station_input_cost_scale);
  posture_station_gain = solve_gain(
      3.0F * config_.station_position_cost_scale,
      config_.station_pitch_cost_scale, config_.station_velocity_cost_scale,
      config_.station_pitch_rate_cost_scale, config_.station_input_cost_scale);
}

void LqrBalancer::setComHeight(float com_height_m) {
  const float target = std::clamp(
      config_.pendulum_length_per_com_height * std::max(com_height_m, 0.02F),
      config_.min_pendulum_length_m, config_.max_pendulum_length_m);
  if (std::abs(target - active_length_m_) > 0.002F) {
    active_length_m_ = target;
    rebuildGain();
  }
}

void LqrBalancer::reset(float ground_position_m, bool /*floor_contact*/) {
  commanded_ground_velocity_m_s_ = 0.0F;
  station_gain_quiet_elapsed_s_ = 0.0F;
  station_gain_blend_ = 0.0F;
  station_gain_active_ = false;
  target_forward_velocity_m_s_ = 0.0F;
  station_velocity_reference_m_s_ = 0.0F;
  target_yaw_rate_rad_s_ = 0.0F;
  ground_position_target_m_ = ground_position_m;
  station_origin_position_m_ = ground_position_m;
  last_debug_ = DebugState{};
  last_debug_.effective_pendulum_length_m = active_length_m_;
  last_debug_.riccati_iterations = config_.riccati_iterations;
}

void LqrBalancer::reanchorPosition(float ground_position_m) noexcept {
  if (!std::isfinite(ground_position_m)) return;
  ground_position_target_m_ = ground_position_m;
  station_origin_position_m_ = ground_position_m;
}

void LqrBalancer::movePositionAnchorToward(float ground_position_m,
                                            float alpha) noexcept {
  if (std::isfinite(ground_position_m) && std::isfinite(alpha)) {
    ground_position_target_m_ += std::clamp(alpha, 0.0F, 1.0F) *
        (ground_position_m - ground_position_target_m_);
  }
}

void LqrBalancer::clearForwardVelocityReference() noexcept {
  commanded_ground_velocity_m_s_ = 0.0F;
  target_forward_velocity_m_s_ = 0.0F;
}

void LqrBalancer::setStationVelocityReference(float velocity_m_s) noexcept {
  if (std::isfinite(velocity_m_s)) {
    station_velocity_reference_m_s_ = velocity_m_s;
  }
}

float LqrBalancer::adaptiveStationGain(float capture_distance) const noexcept {
  const float full = std::max(config_.station_adaptive_full_capture_distance_m,
                              1.0e-4F);
  const float fraction = std::clamp(capture_distance / full, 0.0F, 1.0F);
  const float quiet = std::clamp(config_.station_quiet_gain_scale, 0.0F, 1.0F);
  return quiet + (1.0F - quiet) * (1.0F - fraction);
}

float LqrBalancer::stationReturnVelocity(const RobotState& state,
                                          float pitch_error,
                                          float pitch_rate,
                                          bool hold_station,
                                          float forward_target,
                                          float& capture_distance) const noexcept {
  capture_distance = 0.0F;
  // During a settled zero-command hold the Riccati position state is the
  // station controller.  Feeding a second position-derived velocity target
  // into that same LQR creates a limit cycle around encoder quantisation.
  // Keep the bounded return target for posture capture, where leg motion can
  // legitimately move the support point.
  if (!hold_station || !posture_motion_active_) return forward_target;
  const float omega = std::sqrt(std::max(config_.gravity_m_s2, 0.1F) /
                                std::max(active_length_m_, 0.02F));
  const float position_error = state.wheel.position_m - ground_position_target_m_;
  capture_distance = std::abs(position_error) +
      std::abs(state.wheel.velocity_m_s) / std::max(omega, 1.0e-3F) +
      active_length_m_ * std::abs(std::sin(pitch_error)) +
      std::abs(pitch_rate) / std::max(omega * omega, 1.0e-3F);
  const float schedule = adaptiveStationGain(capture_distance);
  const float gain = std::max(config_.station_hold_gain_hz, 0.0F) * schedule;
  const float damping = std::max(config_.station_hold_velocity_damping_gain, 0.0F) *
      schedule;
  float return_velocity = -gain * position_error - damping * state.wheel.velocity_m_s;
  if (config_.station_capture_gain > 0.0F) {
    return_velocity = -std::clamp(config_.station_capture_gain * schedule, 0.0F, 2.0F) *
        omega * (position_error + state.wheel.velocity_m_s / std::max(omega, 1.0e-3F));
  }
  const float quiet_limit = std::max(config_.station_hold_max_v, 0.0F);
  const float posture_limit = std::max(config_.station_posture_hold_max_v, quiet_limit);
  const float dynamic_limit = std::max(config_.station_hold_dynamic_max_v, posture_limit);
  const float normalized = std::clamp(
      capture_distance / std::max(config_.station_adaptive_full_capture_distance_m,
                                  0.05F), 0.0F, 1.0F);
  const float limit = posture_motion_active_
      ? quiet_limit + (posture_limit - quiet_limit) * normalized
      : quiet_limit;
  const float effective_limit = capture_distance >
      std::max(config_.station_adaptive_full_capture_distance_m, 0.0F)
      ? dynamic_limit : limit;
  return clampFinite(forward_target + std::clamp(return_velocity,
                                                  -effective_limit,
                                                  effective_limit),
                     -config_.max_ground_velocity_m_s,
                     config_.max_ground_velocity_m_s);
}

float LqrBalancer::solveGroundAcceleration(
    const StateVec& state, float station_gain_blend) const noexcept {
  float sum = 0.0F;
  const float blend = std::clamp(station_gain_blend, 0.0F, 1.0F);
  const GainVec& station_gain = station_position_high_gain_
      ? posture_station_gain_ : station_gain_;
  for (int i = 0; i < kStateDim; ++i) {
    const float gain = gain_[i] + blend * (station_gain[i] - gain_[i]);
    sum += gain * state[i];
  }
  return -sum;
}

float LqrBalancer::stationGainBlend(float pitch_error, float pitch_rate,
                                    bool hold_station, float dt_s) noexcept {
  if (!hold_station) {
    station_gain_quiet_elapsed_s_ = 0.0F;
    station_gain_active_ = false;
  } else {
    const bool quiet = std::abs(pitch_error) <=
            std::max(config_.station_gain_pitch_quiet_rad, 0.0F) &&
        std::abs(pitch_rate) <=
            std::max(config_.station_gain_rate_quiet_rad_s, 0.0F);
    station_gain_quiet_elapsed_s_ = quiet
        ? station_gain_quiet_elapsed_s_ + dt_s : 0.0F;
    const float pitch_start = std::max(
        config_.station_gain_pitch_start_rad,
        config_.station_gain_pitch_quiet_rad + 1.0e-4F);
    const float rate_start = std::max(
        config_.station_gain_rate_start_rad_s,
        config_.station_gain_rate_quiet_rad_s + 1.0e-4F);
    if (station_gain_active_ &&
        (std::abs(pitch_error) > pitch_start ||
         std::abs(pitch_rate) > rate_start)) {
      station_gain_active_ = false;
      station_gain_quiet_elapsed_s_ = 0.0F;
    } else if (!station_gain_active_ &&
               station_gain_quiet_elapsed_s_ >=
                   std::max(config_.station_gain_entry_dwell_s, 0.0F)) {
      station_gain_active_ = true;
    }
  }
  const float target = station_gain_active_ ? 1.0F : 0.0F;
  const float transition = std::max(config_.station_gain_transition_s, dt_s);
  station_gain_blend_ += std::clamp(dt_s / transition, 0.0F, 1.0F) *
      (target - station_gain_blend_);
  return station_gain_blend_;
}

LqrBalancer::Output LqrBalancer::update(const RobotState& state,
                                        const ControlCommand& command,
                                        bool integrate,
                                        float balance_pitch_ref_rad,
                                        bool hold_station) {
  Output output{};
  float requested_dt = state.imu.sample_dt_s > 0.0F
      ? state.imu.sample_dt_s : state.dt_s;
  if (!std::isfinite(requested_dt) || requested_dt <= 0.0F)
    requested_dt = std::isfinite(config_.sampling_period_s)
        ? config_.sampling_period_s : 0.002F;
  const float dt = std::clamp(requested_dt, 1.0e-4F, 0.05F);
  if (!std::isfinite(state.imu.pitch_rad) ||
      !std::isfinite(state.imu.pitch_rate_rad_s) ||
      !std::isfinite(state.wheel.position_m) ||
      !std::isfinite(state.wheel.velocity_m_s) ||
      !std::isfinite(balance_pitch_ref_rad)) {
    output.fallen = true;
    last_debug_ = DebugState{};
    return output;
  }

  float forward = clampFinite(command.target_forward_velocity_m_s,
                              -config_.max_ground_velocity_m_s,
                              config_.max_ground_velocity_m_s);
  const float requested_forward = forward;
  const bool stop_command_active = !external_shaping_ &&
      std::abs(requested_forward) < 1.0e-3F &&
      std::abs(target_forward_velocity_m_s_) >= 1.0e-3F;
  if (stop_command_active) {
    const float normal_slew = std::max(
        config_.target_velocity_slew_rate_m_s2, 0.0F);
    const float stop_slew = config_.stop_velocity_slew_rate_m_s2 > 0.0F
        ? config_.stop_velocity_slew_rate_m_s2 : normal_slew;
    if (stop_slew <= 0.0F) {
      forward = requested_forward;
      command_accel_m_s2_ = 0.0F;
    } else {
      const float desired_accel = target_forward_velocity_m_s_ > 0.0F
          ? -stop_slew : stop_slew;
      float stop_accel = desired_accel;
      if (config_.stop_velocity_jerk_m_s3 > 0.0F) {
        const float jerk_step = config_.stop_velocity_jerk_m_s3 * dt;
        stop_accel = std::clamp(
            desired_accel, command_accel_m_s2_ - jerk_step,
            command_accel_m_s2_ + jerk_step);
      }
      const float next = target_forward_velocity_m_s_ + stop_accel * dt;
      const bool crossed_zero = (target_forward_velocity_m_s_ > 0.0F &&
                                 next <= 0.0F) ||
          (target_forward_velocity_m_s_ < 0.0F && next >= 0.0F);
      if (crossed_zero) {
        forward = requested_forward;
        command_accel_m_s2_ = 0.0F;
      } else {
        forward = next;
        command_accel_m_s2_ = stop_accel;
      }
    }
  } else if (!external_shaping_ && config_.target_velocity_slew_rate_m_s2 > 0.0F) {
    const float slew = config_.target_velocity_slew_rate_m_s2;
    if (config_.target_velocity_jerk_m_s3 > 0.0F) {
      // S-curve: bound the acceleration itself, then bound its rate of change.
      const float wanted = std::clamp(
          (forward - target_forward_velocity_m_s_) / dt, -slew, slew);
      const float jerk_step = config_.target_velocity_jerk_m_s3 * dt;
      command_accel_m_s2_ = std::clamp(wanted,
                                       command_accel_m_s2_ - jerk_step,
                                       command_accel_m_s2_ + jerk_step);
      const float next = target_forward_velocity_m_s_ +
          command_accel_m_s2_ * dt;
      // The acceleration state is only a slew limiter. Without this crossing
      // guard it keeps integrating after the requested speed is reached, so a
      // steady 0.3 m/s command becomes a repeating 0.26..0.34 m/s target.
      if ((requested_forward - target_forward_velocity_m_s_) *
              (requested_forward - next) <= 0.0F) {
        forward = requested_forward;
        command_accel_m_s2_ = 0.0F;
      } else {
        forward = next;
      }
    } else {
      const float max_step = slew * dt;
      forward = std::clamp(forward, target_forward_velocity_m_s_ - max_step,
                           target_forward_velocity_m_s_ + max_step);
    }
  }
  target_forward_velocity_m_s_ = forward;

  float yaw = clampFinite(command.target_yaw_rate_rad_s,
                          -config_.max_yaw_rate_rad_s,
                          config_.max_yaw_rate_rad_s);
  if (config_.target_yaw_slew_rate_rad_s2 > 0.0F) {
    const float max_step = config_.target_yaw_slew_rate_rad_s2 * dt;
    yaw = std::clamp(yaw, target_yaw_rate_rad_s_ - max_step,
                     target_yaw_rate_rad_s_ + max_step);
  }
  target_yaw_rate_rad_s_ = yaw;

  const float raw_rate = state.imu.pitch_rate_rad_s;
  const float corrected_rate = config_.use_bias_corrected_pitch_rate &&
      state.imu.pitch_rate_bias_corrected_valid &&
      std::isfinite(state.imu.pitch_rate_bias_corrected_rad_s)
      ? state.imu.pitch_rate_bias_corrected_rad_s : raw_rate;
  const float pitch_scale = std::clamp(config_.wheel_pitch_feedback_scale,
                                       0.0F, 1.0F);
  // Adaptive balance angle: integrate the ground speed error into the pitch
  // reference. Rolling forward in steady state means the reference sits ahead
  // of the true equilibrium, so lean the reference back (and vice versa).
  const float trim_max = std::abs(config_.balance_trim_max_rad);
  if (config_.balance_trim_gain_rad_per_m > 0.0F && integrate &&
      state.floor_contact &&
      std::abs(state.imu.pitch_rad - balance_pitch_ref_rad - balance_trim_rad_) <
          std::max(config_.balance_trim_pitch_gate_rad, 0.0F)) {
    balance_trim_rad_ -= config_.balance_trim_gain_rad_per_m *
        (state.wheel.velocity_m_s - forward) * dt;
    balance_trim_rad_ = std::clamp(balance_trim_rad_, -trim_max, trim_max);
  }
  const float trimmed_pitch_ref = balance_pitch_ref_rad + balance_trim_rad_;
  const float base_pitch_error = pitch_scale *
      (state.imu.pitch_rad - trimmed_pitch_ref);
  const float pitch_rate = pitch_scale * corrected_rate;
  const bool preserve_anchor = hold_station && config_.position_anchor_hold;
  if (preserve_anchor) {
    // A station anchor is a reference, not a follower. Clamping it to the
    // measured wheel position made the reference walk with a drifting robot;
    // the LQR then saw a constant position error and kept spending wheel
    // speed in the same direction until the actuator saturated. Explicit
    // reanchorPosition()/the drive handoff owns reference changes.
  } else {
    if (!posture_motion_active_) station_origin_position_m_ = state.wheel.position_m;
    ground_position_target_m_ = station_origin_position_m_;
  }
  const float position_error = state.wheel.position_m - ground_position_target_m_;

  float capture_distance = 0.0F;
  const float station_target = stationReturnVelocity(
      state, base_pitch_error, pitch_rate, hold_station, forward, capture_distance);
  const float station_gain_blend = stationGainBlend(
      base_pitch_error, pitch_rate, hold_station, dt);
  const float station_velocity_error = station_target - state.wheel.velocity_m_s;
  const float station_pitch_bias = station_gain_blend * std::clamp(
      -config_.station_pitch_return_gain_rad_per_m_s * station_velocity_error,
      -std::max(config_.station_pitch_return_max_rad, 0.0F),
      std::max(config_.station_pitch_return_max_rad, 0.0F));
  const float pitch_error = pitch_scale *
      (state.imu.pitch_rad - trimmed_pitch_ref - station_pitch_bias);
  // Station return is allowed to alter the LQR velocity reference only after
  // the pitch capture has been quiet long enough to arm station_gain_blend.
  // Slewing this reference prevents a position error from competing with the
  // first pitch catch; it also makes the existing station target an actual
  // control input instead of telemetry-only bookkeeping.
  const float station_reference = hold_station
      ? forward + station_gain_blend * (station_target - forward) : forward;
  if (stop_command_active) {
    station_velocity_reference_m_s_ = station_reference;
  } else if (hold_station && !external_shaping_ &&
             config_.station_hold_command_slew_rate_m_s2 > 0.0F) {
    float slew_rate = config_.station_hold_command_slew_rate_m_s2;
    const float max_step = slew_rate * dt;
    station_velocity_reference_m_s_ = std::clamp(
        station_reference,
        station_velocity_reference_m_s_ - max_step,
        station_velocity_reference_m_s_ + max_step);
  } else {
    station_velocity_reference_m_s_ = station_reference;
  }
  const float velocity_target = station_velocity_reference_m_s_;
  const float velocity_reference = config_.wheel_velocity_reference_from_measured
      ? state.wheel.velocity_m_s : commanded_ground_velocity_m_s_;
  const StateVec x{preserve_anchor ? position_error : 0.0F,
                   pitch_error,
                   velocity_reference - velocity_target,
                   pitch_rate};
  float ground_accel = solveGroundAcceleration(x, station_gain_blend);
  // Once pitch energy is quiet, the station reference is allowed to bleed a
  // stale integrated wheel-speed command back toward the bounded capture
  // velocity. This is a reference governor around the same LQR, not a second
  // pitch controller; it is deliberately inactive during the fast transient.
  if (hold_station && station_gain_blend > 0.0F &&
      config_.station_hold_velocity_servo_gain_hz > 0.0F) {
    ground_accel += station_gain_blend *
        config_.station_hold_velocity_servo_gain_hz *
        (station_target - commanded_ground_velocity_m_s_);
  }
  const float friction_accel_limit =
      std::max(config_.wheel_ground_mu, 0.0F) *
      std::max(config_.gravity_m_s2, 0.0F);
  const float accel_limit = std::min(
      std::max(config_.max_ground_accel_m_s2,
               0.1F),
      std::max(friction_accel_limit, 0.1F));
  ground_accel = clampFinite(ground_accel + accel_ff_m_s2_, -accel_limit, accel_limit);

  if (!integrate || !state.floor_contact) {
    commanded_ground_velocity_m_s_ +=
        std::clamp(-commanded_ground_velocity_m_s_ * dt / 0.1F,
                   -std::abs(commanded_ground_velocity_m_s_),
                   std::abs(commanded_ground_velocity_m_s_));
  } else {
    // The LQR input is physical ground acceleration. Integrate it once into
    // the speed target; the motor's internal speed servo then realizes it.
    commanded_ground_velocity_m_s_ += ground_accel * dt;
    if (hold_station && station_velocity_decay_enabled_ &&
        config_.station_velocity_decay_gain_hz > 0.0F) {
      commanded_ground_velocity_m_s_ *= std::clamp(
          1.0F - config_.station_velocity_decay_gain_hz * dt, 0.0F, 1.0F);
    }
  }
  // Anti-windup against a wheel that is not realizing the command.
  const float tracking_margin = config_.command_velocity_tracking_margin_m_s;
  if (tracking_margin > 0.0F && integrate && state.floor_contact) {
    commanded_ground_velocity_m_s_ = std::clamp(
        commanded_ground_velocity_m_s_,
        state.wheel.velocity_m_s - tracking_margin,
        state.wheel.velocity_m_s + tracking_margin);
  }
  commanded_ground_velocity_m_s_ = clampFinite(commanded_ground_velocity_m_s_,
                                               -config_.max_ground_velocity_m_s,
                                               config_.max_ground_velocity_m_s);
  last_debug_ = DebugState{};
  last_debug_.commanded_ground_velocity_m_s = commanded_ground_velocity_m_s_;
  last_debug_.lqr_velocity_reference_m_s = velocity_target;
  last_debug_.station_velocity_target_m_s = station_target;
  last_debug_.target_ground_position_m = ground_position_target_m_;
  last_debug_.optimal_command_m_s = ground_accel;
  last_debug_.balance_pitch_ref_rad = balance_pitch_ref_rad;
  last_debug_.lqr_pitch_ref_rad = trimmed_pitch_ref + station_pitch_bias;
  last_debug_.balance_trim_rad = balance_trim_rad_;
  last_debug_.lqr_pitch_state_rad = state.imu.pitch_rad;
  last_debug_.effective_pendulum_length_m = active_length_m_;
  last_debug_.adaptive_gain_scale = adaptiveStationGain(capture_distance);
  last_debug_.station_gain_blend = station_gain_blend;
  last_debug_.capture_distance_m = capture_distance;
  last_debug_.station_hold_active = hold_station;
  last_debug_.riccati_iterations = config_.riccati_iterations;

  if (std::abs(state.imu.pitch_rad - balance_pitch_ref_rad) > config_.fall_pitch_rad) {
    output.fallen = true;
    return output;
  }

  const float yaw_feedback = config_.yaw_rate_feedback_kp *
      (yaw - clampFinite(state.imu.yaw_rate_rad_s, -10.0F, 10.0F));
  const float yaw_command = clampFinite(yaw + yaw_feedback,
                                         -config_.max_yaw_rate_rad_s,
                                         config_.max_yaw_rate_rad_s);
  const float left_ground = commanded_ground_velocity_m_s_ -
      config_.half_wheel_base_m * yaw_command;
  const float right_ground = commanded_ground_velocity_m_s_ +
      config_.half_wheel_base_m * yaw_command;
  const float radius = std::max(config_.wheel_radius_m, 1.0e-4F);
  // Wheel speed bounded by the configured ground speed cap and by 50 rad/s = 2.0 m/s on the 4 cm wheel, the hard
  // limit of the 15:20 balance build that held 320 s on the robot (the 4.19 m/s cap alone would allow 105 rad/s).
  const float wheel_limit = std::min(std::max(config_.max_ground_velocity_m_s, 0.05F) / radius, 50.0F);
  output.ground_velocity_m_s = commanded_ground_velocity_m_s_;
  output.left_wheel_velocity_rad_s = clampFinite(config_.left_wheel_sign *
      left_ground / radius, -wheel_limit, wheel_limit);
  output.right_wheel_velocity_rad_s = clampFinite(config_.left_wheel_sign *
      right_ground / radius, -wheel_limit, wheel_limit);
  // Speed error handed to the leg task - see the Output field's note on why
  // this, and not either form of acceleration, is the signal that keeps the
  // leg loop negative.
  output.velocity_error_m_s = clampFinite(
      forward - state.wheel.velocity_m_s,
      -config_.max_ground_velocity_m_s, config_.max_ground_velocity_m_s);
  last_debug_.leg_velocity_error_m_s = output.velocity_error_m_s;
  const float yaw_scale = std::abs(yaw_command) >= config_.stiff_yaw_rate_rad_s
      ? config_.turning_stiffness_boost : 0.0F;
  output.leg_kp_scale = config_.leg_kp_scale + yaw_scale;
  output.leg_kd_scale = config_.leg_kd_scale + yaw_scale;
  return output;
}

void LqrBalancer::seedForwardVelocity(float velocity_m_s) noexcept {
  if (!std::isfinite(velocity_m_s)) return;
  commanded_ground_velocity_m_s_ = velocity_m_s;
  target_forward_velocity_m_s_ = velocity_m_s;
  station_velocity_reference_m_s_ = velocity_m_s;
}

}  // namespace robot_control
