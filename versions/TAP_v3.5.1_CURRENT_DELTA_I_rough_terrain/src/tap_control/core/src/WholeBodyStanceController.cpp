#include "robot_control/WholeBodyStanceController.hpp"

#include <algorithm>
#include <cmath>

namespace robot_control {
namespace {

constexpr std::size_t left_hip = static_cast<std::size_t>(JointId::LeftHip);
constexpr std::size_t left_knee = static_cast<std::size_t>(JointId::LeftKnee);
constexpr std::size_t right_hip = static_cast<std::size_t>(JointId::RightHip);
constexpr std::size_t right_knee = static_cast<std::size_t>(JointId::RightKnee);

struct Vec2 {
  float first = 0.0F;
  float second = 0.0F;
};

bool solve2x2(float a, float b, float c, float d, Vec2 rhs,
              Vec2& solution) noexcept {
  const float determinant = a * d - b * c;
  const float scale = std::max({std::abs(a), std::abs(b), std::abs(c),
                                std::abs(d), 1.0e-6F});
  const float determinant_floor =
      std::max(1.0e-12F, 1.0e-6F * scale * scale);
  if (!std::isfinite(determinant) ||
      std::abs(determinant) < determinant_floor) {
    return false;
  }
  solution.first = (rhs.first * d - b * rhs.second) / determinant;
  solution.second = (a * rhs.second - rhs.first * c) / determinant;
  return std::isfinite(solution.first) && std::isfinite(solution.second);
}

bool solveWeightedSymmetricTask(const WholeBodyModel::SymmetricComJacobian& com,
                                const WholeBodyModel::LegHeight& leg,
                                float com_target, float com_weight,
                                float axle_target, float height_weight,
                                float axle_weight,
                                float damping,
                                Vec2& solution) noexcept {
  const float wc = std::max(com_weight, 0.0F);
  const float wh = std::max(height_weight, 0.0F);
  const float wx = std::max(axle_weight, 0.0F);
  const float wd = std::max(damping, 0.0F);
  const float jcx = com.d_com_offset_x_d_hip;
  const float jck = com.d_com_offset_x_d_knee;
  const float jhx = leg.d_height_d_hip;
  const float jhk = leg.d_height_d_knee;
  const float jxx = leg.d_x_d_hip;
  const float jxk = leg.d_x_d_knee;
  const float a = wc * jcx * jcx + wh * jhx * jhx + wx * jxx * jxx + wd;
  const float b = wc * jcx * jck + wh * jhx * jhk + wx * jxx * jxk;
  const float d = wc * jck * jck + wh * jhk * jhk + wx * jxk * jxk + wd;
  const Vec2 rhs{wc * jcx * com_target + wx * jxx * axle_target,
                 wc * jck * com_target + wx * jxk * axle_target};
  return solve2x2(a, b, b, d, rhs, solution);
}

float deadband(float value, float band) noexcept {
  if (std::abs(value) <= std::max(band, 0.0F)) return 0.0F;
  return value - std::copysign(std::max(band, 0.0F), value);
}

void clampCorrection(JointArray& correction, float limit) noexcept {
  const float bound = std::max(limit, 0.0F);
  for (const auto joint : {left_hip, left_knee, right_hip, right_knee}) {
    correction[joint] = std::clamp(correction[joint], -bound, bound);
  }
}

}  // namespace

void WholeBodyStanceController::reset() noexcept {
  correction_.fill(0.0F);
  applied_correction_.fill(0.0F);
  task_command_filter_.fill(0.0F);
  speed_loop_.fill(0.0F);
  speed_loop_filter_.fill(0.0F);
  previous_velocity_error_m_s_ = 0.0F;
  velocity_error_rate_m_s2_ = 0.0F;
  speed_loop_integral_m_ = 0.0F;
  last_output_ = Output{};
}

WholeBodyStanceController::Output WholeBodyStanceController::update(
    const RobotState& state, const JointArray& base_target,
    const WholeBodyModel::Result& body, bool allow, float dt_s,
    bool allow_static_com,
    float velocity_error_m_s, bool allow_speed_loop) noexcept {
  Output output{};
  output.target = base_target;

  const float dt = std::clamp(dt_s, 1.0e-4F, 0.05F);
  const bool valid = std::isfinite(state.imu.pitch_rad) &&
      std::isfinite(state.imu.pitch_rate_rad_s) &&
      std::isfinite(state.wheel.velocity_m_s) &&
      std::isfinite(state.joints.position_rad[left_hip]) &&
      std::isfinite(state.joints.position_rad[left_knee]) &&
      std::isfinite(state.joints.velocity_rad_s[left_hip]) &&
      std::isfinite(state.joints.velocity_rad_s[left_knee]) &&
      std::isfinite(state.joints.position_rad[right_hip]) &&
      std::isfinite(state.joints.position_rad[right_knee]) &&
      std::isfinite(state.joints.velocity_rad_s[right_hip]) &&
      std::isfinite(state.joints.velocity_rad_s[right_knee]) &&
      std::isfinite(body.com_offset_x_m) &&
      std::isfinite(body.com_above_axle_m) &&
      std::isfinite(body.pitch_inertia_at_axle_kg_m2) &&
      std::isfinite(body.axle_height_difference_m);
  if (!config_.enabled || !valid) {
    last_output_ = output;
    return output;
  }

  const float hip = 0.5F *
      (state.joints.position_rad[left_hip] + state.joints.position_rad[right_hip]);
  const float knee = 0.5F *
      (state.joints.position_rad[left_knee] + state.joints.position_rad[right_knee]);
  const auto com_jacobian = WholeBodyModel::symmetricComOffsetJacobian(hip, knee);
  const auto left_leg = WholeBodyModel::legHeight(
      state.joints.position_rad[left_hip],
      state.joints.position_rad[left_knee]);
  const auto right_leg = WholeBodyModel::legHeight(
      state.joints.position_rad[right_hip],
      state.joints.position_rad[right_knee]);
  const auto leg = WholeBodyModel::legHeight(hip, knee);
  const float hip_rate = 0.5F *
      (state.joints.velocity_rad_s[left_hip] +
       state.joints.velocity_rad_s[right_hip]);
  const float knee_rate = 0.5F *
      (state.joints.velocity_rad_s[left_knee] +
       state.joints.velocity_rad_s[right_knee]);
  const float com_x_rate = com_jacobian.d_com_offset_x_d_hip * hip_rate +
      com_jacobian.d_com_offset_x_d_knee * knee_rate;
  // Geometry shared by the slow COM and axle tasks below.
  const auto natural_leg = WholeBodyModel::legHeight(
      geometry::neutral_hip_rad, geometry::neutral_knee_rad);
  const float axle_height = std::max(-leg.height_m, 0.05F);
  const float body_pitch = std::clamp(state.imu.pitch_rad, -0.35F, 0.35F);

  // Outer speed loop, closed through the legs. Shifting the body forward
  // over the contact makes the robot accelerate, so the leg task can own
  // speed tracking while the wheel controller owns nothing but balance.
  // Driving this from the speed ERROR is what keeps it stable: body forward
  // raises speed, which shrinks the error that moved it - negative feedback.
  // (Driving the same geometry from measured acceleration inverts that into
  // positive feedback and the robot tips; see the config note.)
  const float com_height = std::max(body.com_above_axle_m, 0.02F);
  const float gravity_ff = std::max(config_.gravity_m_s2, 0.1F);
  const float v_err = std::isfinite(velocity_error_m_s) ? velocity_error_m_s : 0.0F;
  const float tau = std::max(config_.speed_loop_tau_s, 0.0F);
  const bool loop_on = allow_speed_loop && tau > 0.0F;

  // Inertial lead from the live pose - one pendulum time constant.
  const float mass_ff = std::max(geometry::total_mass_kg, 0.01F);
  const float inertia_ff = std::max(body.pitch_inertia_at_axle_kg_m2, 1.0e-4F);
  const float gravity_stiffness_ff =
      std::max(mass_ff * gravity_ff * com_height, 1.0e-6F);
  const float tau_j = std::sqrt(inertia_ff / gravity_stiffness_ff);
  output.speed_loop_lead_s = tau_j;
  const float raw_rate = loop_on
      ? (v_err - previous_velocity_error_m_s_) / dt : 0.0F;
  if (!loop_on) {
    // The leg speed loop is an optional actuator owner, not a hidden state
    // owner that survives stop, yaw-only or height-only posture states.
    previous_velocity_error_m_s_ = v_err;
    velocity_error_rate_m_s2_ = 0.0F;
    speed_loop_integral_m_ = 0.0F;
  } else {
    previous_velocity_error_m_s_ = v_err;
  }
  const float rate_alpha = std::clamp(
      std::max(config_.speed_loop_filter_hz, 0.0F) * dt, 0.0F, 1.0F);
  velocity_error_rate_m_s2_ += rate_alpha *
      ((std::isfinite(raw_rate) ? raw_rate : 0.0F) - velocity_error_rate_m_s2_);
  const float led_error = v_err +
      config_.speed_loop_inertia_lead * tau_j * velocity_error_rate_m_s2_;

  // Yield to the wheel controller as pitch grows - this loop commands an
  // open-loop moment through gravity, so it must not keep asking while the
  // wheels are already using everything they have to stay upright.
  const float release_start =
      std::max(config_.speed_loop_release_pitch_rad, 0.0F);
  const float release_span =
      std::max(config_.speed_loop_release_span_rad, 1.0e-3F);
  const float release_scale = 1.0F - std::clamp(
      (std::abs(state.imu.pitch_rad) - release_start) / release_span,
      0.0F, 1.0F);
  output.speed_loop_release_scale = release_scale;

  const float bound = std::max(config_.speed_loop_max_rad, 0.0F);
  // x = (h/(g*tau)) * v_err gives a first-order speed loop of constant tau.
  const float com_gain = loop_on ? com_height / (gravity_ff * tau) : 0.0F;
  if (loop_on && config_.speed_loop_integral_hz > 0.0F && release_scale > 0.0F) {
    speed_loop_integral_m_ += config_.speed_loop_integral_hz * com_gain *
        v_err * dt;
    // Bound the integral in the same units the solve is bounded in, using the
    // COM travel one clamp's worth of hip motion can actually produce.
    const float integral_bound = std::abs(
        com_jacobian.d_com_offset_x_d_hip * bound);
    speed_loop_integral_m_ =
        std::clamp(speed_loop_integral_m_, -integral_bound, integral_bound);
  } else {
    speed_loop_integral_m_ = 0.0F;
  }
  const float com_target = loop_on ? release_scale *
      (com_gain * led_error + speed_loop_integral_m_) : 0.0F;
  output.speed_loop_com_m = com_target;

  Vec2 speed_task{};
  if (!loop_on ||
      !solveWeightedSymmetricTask(
          com_jacobian, leg, com_target, 1.0F, 0.0F,
          config_.height_position_weight,
          config_.axle_position_weight,
          config_.symmetric_task_damping, speed_task)) {
    speed_task = {};
  }
  {
    const float alpha = std::clamp(
        std::max(config_.speed_loop_filter_hz, 0.0F) * dt, 0.0F, 1.0F);
    const float max_step =
        std::max(config_.speed_loop_max_rate_rad_s, 0.0F) * dt;
    JointArray goal{};
    goal[left_hip] = std::clamp(speed_task.first, -bound, bound);
    goal[right_hip] = goal[left_hip];
    goal[left_knee] = std::clamp(speed_task.second, -bound, bound);
    goal[right_knee] = goal[left_knee];
    for (const auto joint : {left_hip, left_knee, right_hip, right_knee}) {
      speed_loop_filter_[joint] +=
          alpha * (goal[joint] - speed_loop_filter_[joint]);
      const float step = std::clamp(
          speed_loop_filter_[joint] - speed_loop_[joint], -max_step, max_step);
      speed_loop_[joint] =
          std::clamp(speed_loop_[joint] + step, -bound, bound);
    }
    output.speed_loop_hip_rad = speed_loop_[left_hip];
  }
  // The slow trim must not see the feed-forward's own contribution to the
  // measured COM, or it spends its integrator undoing it.
  const float speed_loop_com_x =
      com_jacobian.d_com_offset_x_d_hip * speed_loop_[left_hip] +
      com_jacobian.d_com_offset_x_d_knee * speed_loop_[left_knee];

  if (!allow) {
    // This is a sample gate, not an ownership reset. Hold the last bounded
    // correction at the actuator boundary while the sensor state is briefly
    // outside the quiet window. Otherwise the internal trim survives but the
    // commanded pose jumps back to the untrimmed path for one sample.
    for (const auto joint : {left_hip, left_knee, right_hip, right_knee}) {
      applied_correction_[joint] = correction_[joint] + speed_loop_[joint];
      output.target[joint] += applied_correction_[joint];
    }
    output.target[left_hip] = std::clamp(output.target[left_hip],
                                         config_.hip_min_rad, config_.hip_max_rad);
    output.target[right_hip] = std::clamp(output.target[right_hip],
                                          config_.hip_min_rad, config_.hip_max_rad);
    output.target[left_knee] = std::clamp(output.target[left_knee],
                                          config_.knee_min_rad, config_.knee_max_rad);
    output.target[right_knee] = std::clamp(output.target[right_knee],
                                           config_.knee_min_rad, config_.knee_max_rad);
    const float left_hip_correction = correction_[left_hip];
    const float left_knee_correction = correction_[left_knee];
    const float right_hip_correction = correction_[right_hip];
    const float right_knee_correction = correction_[right_knee];
    output.symmetric_trim_norm_rad = std::hypot(
        left_hip_correction, left_knee_correction);
    output.differential_trim_norm_rad = std::hypot(
        left_hip_correction - right_hip_correction,
        left_knee_correction - right_knee_correction);
    output.active = false;
    last_output_ = output;
    return output;
  }

  // The leg-side gimbal owns only the body-frame COM line. Body pitch is
  // deliberately absent here; LQR owns that state through the wheels.
  output.com_target_pitch_rad = 0.0F;
  const float desired_com_x = 0.0F;
  const float com_error = allow_static_com
      ? deadband(desired_com_x - (body.com_offset_x_m - speed_loop_com_x),
                 config_.com_deadband_m)
      : 0.0F;
  output.com_error_m = com_error;

  // Symmetric hip/knee motion shifts COM while softly preserving both average
  // leg height and the axle's fore/aft coordinate. The latter is essential
  // during a height path: a pure COM+height solve has no remaining preference
  // against translating the wheel contact point, which appears as drift.
  Vec2 symmetric_rate{};
  const float dheight_dhip = leg.d_height_d_hip;
  const float dheight_dknee = leg.d_height_d_knee;
  const float damped_com_error = com_error -
      std::max(config_.com_velocity_damping_s, 0.0F) *
          com_x_rate;
  const float com_task = config_.com_position_gain * damped_com_error;
  // A supported body pitch means the axle is displaced in the body frame.
  // Give that small geometric error to the legs; otherwise the only actuator
  // that can react to pitch is the wheel and the robot translates to balance.
  // The native pitch/leg convention maps this body-frame correction with the
  // positive sign below; the closed-loop model contract tests this mapping.
  const float desired_axle_x = natural_leg.x_m + axle_height * std::sin(body_pitch);
  const float axle_pitch_error = std::clamp(
      desired_axle_x - leg.x_m,
      -std::max(config_.body_pitch_task_max_axle_error_m, 0.0F),
      std::max(config_.body_pitch_task_max_axle_error_m, 0.0F));
  const float axle_task = config_.body_pitch_task_enabled
      ? config_.body_pitch_task_gain * axle_pitch_error : 0.0F;
  const float axle_task_weight = config_.body_pitch_task_enabled
      ? std::max(config_.body_pitch_task_weight,
                 config_.axle_position_weight)
      : config_.axle_position_weight;
  if (std::isfinite(com_task) &&
      std::isfinite(dheight_dhip) && std::isfinite(dheight_dknee) &&
      solveWeightedSymmetricTask(
          com_jacobian, leg, com_task, 1.0F, axle_task,
          config_.height_position_weight, axle_task_weight,
          config_.symmetric_task_damping,
          symmetric_rate)) {
    // This is a joint displacement (rad), not a velocity. The output slew
    // limit below is the only rad/s clamp; keeping the task solve dimensional
    // prevents correction_filter_hz from silently changing its gain.
    symmetric_rate.first = std::clamp(symmetric_rate.first,
                                      -config_.max_correction_rad,
                                      config_.max_correction_rad);
    symmetric_rate.second = std::clamp(symmetric_rate.second,
                                       -config_.max_correction_rad,
                                       config_.max_correction_rad);
  } else {
    symmetric_rate = {};
  }

  // Differential leg trim equalizes the two inferred wheel-axle heights. It
  // holds each side's fore/aft axle coordinate, so it does not secretly ask
  // the wheels to drive while correcting roll geometry.
  const float axle_error = deadband(body.axle_height_difference_m,
                                    config_.axle_height_deadband_m);
  output.axle_height_error_m = axle_error;
  Vec2 left_rate{}, right_rate{};
  const float left_height_task =
      -0.5F * config_.roll_position_gain * axle_error;
  const float right_height_task =
      0.5F * config_.roll_position_gain * axle_error;
  const bool left_ok = solve2x2(
      left_leg.d_x_d_hip, left_leg.d_x_d_knee,
      left_leg.d_height_d_hip, left_leg.d_height_d_knee,
      {0.0F, left_height_task}, left_rate);
  const bool right_ok = solve2x2(
      right_leg.d_x_d_hip, right_leg.d_x_d_knee,
      right_leg.d_height_d_hip, right_leg.d_height_d_knee,
      {0.0F, right_height_task}, right_rate);
  if (!left_ok) left_rate = {};
  if (!right_ok) right_rate = {};

  // The solve returns a bounded task displacement in radians.  First filter
  // that task command, then integrate it into the persistent trim with its
  // own bandwidth.  Keeping these operations separate is important: changing
  // correction_filter_hz must not secretly change the COM integrator gain.
  JointArray task_command{};
  task_command[left_hip] = symmetric_rate.first + left_rate.first;
  task_command[left_knee] = symmetric_rate.second + left_rate.second;
  task_command[right_hip] = symmetric_rate.first + right_rate.first;
  task_command[right_knee] = symmetric_rate.second + right_rate.second;
  clampCorrection(task_command, config_.max_correction_rad);

  const float filter_alpha = std::clamp(
      config_.correction_filter_hz * dt, 0.0F, 1.0F);
  for (const auto joint : {left_hip, left_knee, right_hip, right_knee}) {
    task_command_filter_[joint] += filter_alpha *
        (task_command[joint] - task_command_filter_[joint]);
  }

  // A symmetric COM correction is a posture target, not a torque-like trim
  // to integrate forever. Express it as a bounded offset from the base
  // posture and slew that offset toward the filtered task. The inner joint
  // servo must retain its measured-position error/preload under load.
  const float integrator_gain = allow_static_com
      ? std::max(config_.com_integrator_gain_hz, 0.0F)
      : 0.0F;
  const float max_step = std::max(config_.max_correction_rate_rad_s, 0.0F) * dt;
  for (const auto joint : {left_hip, left_knee, right_hip, right_knee}) {
    const float residual_task = task_command_filter_[joint];
    float step = integrator_gain * dt * residual_task;
    step = std::clamp(step, -max_step, max_step);
    correction_[joint] += step;
  }
  clampCorrection(correction_, config_.max_correction_rad);

  output.target = base_target;
  for (const auto joint : {left_hip, left_knee, right_hip, right_knee}) {
    applied_correction_[joint] = correction_[joint] + speed_loop_[joint];
    output.target[joint] += applied_correction_[joint];
  }
  output.target[left_hip] = std::clamp(output.target[left_hip],
                                       config_.hip_min_rad, config_.hip_max_rad);
  output.target[right_hip] = std::clamp(output.target[right_hip],
                                        config_.hip_min_rad, config_.hip_max_rad);
  output.target[left_knee] = std::clamp(output.target[left_knee],
                                        config_.knee_min_rad, config_.knee_max_rad);
  output.target[right_knee] = std::clamp(output.target[right_knee],
                                         config_.knee_min_rad, config_.knee_max_rad);
  output.active = true;
  const float left_hip_correction = correction_[left_hip];
  const float left_knee_correction = correction_[left_knee];
  const float right_hip_correction = correction_[right_hip];
  const float right_knee_correction = correction_[right_knee];
  output.symmetric_trim_norm_rad = std::hypot(
      left_hip_correction, left_knee_correction);
  output.differential_trim_norm_rad = std::hypot(
      left_hip_correction - right_hip_correction,
      left_knee_correction - right_knee_correction);
  last_output_ = output;
  return output;
}

}  // namespace robot_control
