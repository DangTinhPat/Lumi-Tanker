#pragma once

#include <algorithm>
#include <cmath>

namespace tap_fw::v3 {

// Recovery from the configured lying pose. Only IMU and encoder-derived
// kinematics enter this bounded, allocation-free state machine.
class StandUp {
 public:
  enum class Phase { Idle, Folding, Kneeling, Pitching, Flipped, Catching, Support, Failed };

  struct Config {
    bool enabled = false;
    float fold_rate_rad_s = 0.35F;
    float fold_travel_rad = 2.30F;
    float fold_direction = -1.0F;
    float fold_pitch_limit_rad = 0.72F;
    float kneel_hold_s = 1.0F;
    bool hold_at_kneel = false;    // diagnostic operator hold, FLIP advances
    bool hold_after_flip = false;  // diagnostic stop after step 3
    float flip_pitch_kp_rad_s_per_rad = 1.0F;
    float flip_pitch_kd = 0.08F;
    float flip_hip_rate_max_rad_s = 0.20F;
    float flip_travel_max_rad = 1.0F;
    float pitch_target_tol_rad = 0.04F;
    float pitch_rate_tol_rad_s = 0.10F;
    float pitch_settle_s = 0.15F;
    // Start capture while still pitching, once the body envelope clears the
    // floor. Core opt-in; the GIM runner enables this overlapping sequence.
    bool overlap_after_lift = false;
    float lift_clearance_m = 0.003F;
    float lift_progress_rad = 0.06F;
    float lift_confirm_s = 0.02F;
    float catch_blend_s = 0.15F;
    // Hip allocation: reduce CoM lean, damp its motion, and level the body.
    // Signed feedback permits correction in either direction; no oscillation
    // or timed sweep is injected to "search" for the centre of mass.
    float catch_hip_gain = 4.0F;       // (rad/s)/rad, before dividing by FK Jacobian
    float catch_hip_damping_s = 0.2F;
    float catch_body_gain = 1.0F;
    float catch_body_damping_s = 0.0F;
    float catch_wheel_peak_s = 0.0F;    // opt-in actuator request, bounded window
    float catch_hip_rate_max_rad_s = 1.0F;
    float catch_hip_accel_rad_s2 = 12.0F;
    float catch_hip_travel_rad = 1.2F;
    float catch_hip_tracking_max_rad = 0.12F;
    // Wheel inner loop on physical CoM lean: acceleration -> speed, with
    // actuator-lag lead, tracking anti-windup and inverse carrier kinematics.
    float catch_accel_gain = 35.0F;     // (m/s^2)/rad
    float catch_accel_damping = 4.0F;   // (m/s^2)/(rad/s)
    float catch_speed_gain = 2.0F;      // 1/s, outer-loop stopping feedback
    float catch_accel_max = 2.0F;       // m/s^2, below the 0.13 Nm pair's ideal cap
    float catch_wheel_lead_s = 0.075F;
    float catch_wheel_max_rad_s = 10.0F;
    float catch_tracking_margin_m_s = 0.15F;
    float catch_lean_tol_rad = 0.08F;
    float catch_capture_tol_rad = 0.06F;
    float catch_body_tol_rad = 0.25F;
    float catch_rate_tol_rad_s = 1.5F;
    float catch_speed_tol_m_s = 0.5F;
    float catch_settle_s = 0.04F;
    float pitch_limit_rad = 0.72F;
    float timeout_s = 20.0F;            // per moving phase
  };

  struct Input {
    float dt_s = 0.002F;
    float pitch_rad = 0.0F;             // +Y rotation, positive nose-down
    float pitch_rate_rad_s = 0.0F;
    float capture_lean_rad = 0.0F;      // physical axle->CoM angle, no hanging-pose rebase
    float capture_rate_rad_s = 0.0F;    // gyro + full measured FK Jacobian * joint rates
    float hip_jacobian = 1.0F;
    float hip_measured_rad = 0.0F;
    float carrier_rate_rad_s = 0.0F;    // body + hip + knee angular velocity
    float ground_speed_m_s = 0.0F;      // encoders + carrier, assumes rolling support
    float wheel_radius_m = 0.04F;
    float capture_omega_rad_s = 10.0F;  // sqrt(m*g*l/I_axle), model constants + encoders
    float body_clearance_m = 0.0F;     // conservative CAD/IMU/encoder estimate
    bool body_clearance_valid = false;
    bool sensors_ok = true;
  };

  struct Output {
    Phase phase = Phase::Idle;
    float hip_target_rad = 0.0F;
    float knee_target_rad = 0.0F;
    float wheel_rad_s = 0.0F;
    bool wheels_on = true;
    float capture_point_rad = 0.0F;
    float capture_rate_rad_s = 0.0F;
    float hip_rate_rad_s = 0.0F;
    float phase_elapsed_s = 0.0F;
    float body_clearance_m = 0.0F;
    bool body_clearance_valid = false;
    float catch_blend = 0.0F;
    bool wheel_peak_requested = false;
  };

  StandUp() noexcept = default;
  explicit StandUp(Config config) noexcept : config_(config) {}

  void begin(float hip_rad, float knee_rad) noexcept {
    phase_ = std::isfinite(hip_rad) && std::isfinite(knee_rad) ? Phase::Folding : Phase::Failed;
    elapsed_s_ = pitch_settle_s_ = 0.0F;
    hip_start_rad_ = hip_cmd_ = std::isfinite(hip_rad) ? hip_rad : 0.0F;
    flip_start_hip_rad_ = catch_start_hip_rad_ = hip_cmd_;
    knee_cmd_ = std::isfinite(knee_rad) ? knee_rad : 0.0F;
    hip_rate_cmd_ = wheel_velocity_m_s_ = 0.0F;
    catch_leveling_ = true;
    catch_stall_s_ = 0.0F;
    flip_pitch_initialized_ = catch_overlap_ = false;
    lift_clear_s_ = catch_entry_rate_ = 0.0F;
  }

  void reset() noexcept { phase_ = Phase::Idle; }

  bool advanceFromKneeling() noexcept {
    if (phase_ != Phase::Kneeling) return false;
    phase_ = Phase::Pitching;
    elapsed_s_ = pitch_settle_s_ = 0.0F;
    flip_start_hip_rad_ = hip_cmd_;
    flip_pitch_initialized_ = false;
    lift_clear_s_ = 0.0F;
    return true;
  }

  [[nodiscard]] Output update(const Input& in) noexcept {
    Output out{};
    if (phase_ == Phase::Idle || phase_ == Phase::Failed) return output(out);
    if (!valid(in)) {
      phase_ = Phase::Failed;
      return output(out);
    }
    const float dt = std::clamp(in.dt_s, 0.0F, 0.01F);
    elapsed_s_ += dt;
    const float pitch_limit = phase_ == Phase::Folding
        ? config_.fold_pitch_limit_rad : config_.pitch_limit_rad;
    const bool moving = phase_ == Phase::Folding || phase_ == Phase::Pitching || phase_ == Phase::Catching;
    if (std::abs(in.pitch_rad) > pitch_limit || (moving && elapsed_s_ > config_.timeout_s)) {
      phase_ = Phase::Failed;
      return output(out);
    }
    out.capture_rate_rad_s = in.capture_rate_rad_s;
    out.body_clearance_m = in.body_clearance_m;
    out.body_clearance_valid = in.body_clearance_valid && std::isfinite(in.body_clearance_m);
    // Divergent-mode/capture-point gate, following Pratt et al., Humanoids 2006,
    // doi:10.1109/ICHR.2006.321385. This angular approximation is used only
    // inside the small-angle capture region, not as proof of ground contact.
    out.capture_point_rad = in.capture_lean_rad + in.capture_rate_rad_s / in.capture_omega_rad_s;
    const float previous_hip = hip_cmd_;
    if (phase_ == Phase::Folding) {
      const float remaining = std::max(config_.fold_travel_rad - std::abs(hip_cmd_ - hip_start_rad_), 0.0F);
      const float step = std::min(config_.fold_rate_rad_s * dt, remaining);
      hip_cmd_ += (config_.fold_direction < 0.0F ? -1.0F : 1.0F) * step;
      if (step >= remaining) {
        phase_ = Phase::Kneeling;
        elapsed_s_ = 0.0F;
      }
    } else if (phase_ == Phase::Kneeling) {
      if (!config_.hold_at_kneel && elapsed_s_ >= config_.kneel_hold_s) advanceFromKneeling();
    } else if (phase_ == Phase::Pitching) {
      if (!flip_pitch_initialized_) {
        flip_start_pitch_rad_ = in.pitch_rad;
        flip_pitch_initialized_ = true;
      }
      const float rate = std::clamp(config_.flip_pitch_kp_rad_s_per_rad * in.pitch_rad +
          config_.flip_pitch_kd * in.pitch_rate_rad_s,
          -config_.flip_hip_rate_max_rad_s, config_.flip_hip_rate_max_rad_s);
      hip_cmd_ = std::clamp(hip_cmd_ + rate * dt,
          flip_start_hip_rad_ - config_.flip_travel_max_rad,
          flip_start_hip_rad_ + config_.flip_travel_max_rad);
      const bool level = std::abs(in.pitch_rad) <= config_.pitch_target_tol_rad &&
          std::abs(in.pitch_rate_rad_s) <= config_.pitch_rate_tol_rad_s;
      pitch_settle_s_ = level ? pitch_settle_s_ + dt : 0.0F;
      const bool overlap = config_.overlap_after_lift && !config_.hold_after_flip;
      const bool lifted = out.body_clearance_valid &&
          in.body_clearance_m >= config_.lift_clearance_m &&
          (std::abs(flip_start_pitch_rad_) - std::abs(in.pitch_rad) >= config_.lift_progress_rad ||
           std::abs(in.pitch_rad) <= config_.pitch_target_tol_rad) &&
          in.pitch_rad * in.pitch_rate_rad_s <= 0.0F;
      lift_clear_s_ = lifted ? lift_clear_s_ + dt : 0.0F;
      // Overlap deliberately does not wait for pitch zero or a stationary
      // body. Missing clearance never falls back to a timed blind launch.
      const bool ready = overlap ? (lifted && lift_clear_s_ >= config_.lift_confirm_s)
                                 : (pitch_settle_s_ >= config_.pitch_settle_s);
      if (ready) {
        phase_ = config_.hold_after_flip ? Phase::Flipped : Phase::Catching;
        catch_overlap_ = overlap;
        catch_entry_rate_ = dt > 0.0F ? (hip_cmd_ - previous_hip) / dt : 0.0F;
        catch_start_hip_rad_ = hip_cmd_;
        elapsed_s_ = pitch_settle_s_ = 0.0F;
        wheel_velocity_m_s_ = std::clamp(in.ground_speed_m_s,
            -config_.catch_wheel_max_rad_s * in.wheel_radius_m,
            config_.catch_wheel_max_rad_s * in.wheel_radius_m);
      }
    } else if (phase_ == Phase::Catching) {
      // If body leveling cancels hip progress while CoM remains outside the
      // capture region, temporarily prioritize capture. This progress test
      // uses measured motion, not a simulator contact or a scripted sweep.
      if (std::abs(in.capture_lean_rad) < 0.08F) catch_leveling_ = true;
      const bool stalled = std::abs(in.capture_lean_rad) > 0.12F &&
          std::abs(in.capture_rate_rad_s) < 0.15F && std::abs(hip_rate_cmd_) < 0.25F;
      catch_stall_s_ = stalled ? catch_stall_s_ + dt : 0.0F;
      if (catch_stall_s_ > 0.15F) catch_leveling_ = false;
      const float feedback_rate = std::clamp(
          -(config_.catch_hip_gain * in.capture_lean_rad +
            config_.catch_hip_damping_s * in.capture_rate_rad_s) / in.hip_jacobian +
          (catch_leveling_ ? config_.catch_body_gain * in.pitch_rad : 0.0F) +
          config_.catch_body_damping_s * in.pitch_rate_rad_s,
          -config_.catch_hip_rate_max_rad_s, config_.catch_hip_rate_max_rad_s);
      const float blend = catchBlend();
      // Continue the ongoing lift velocity while capture progressively takes
      // over. The acceleration bound brakes an inherited fast flip velocity;
      // the steady catch speed cap applies to the feedback target.
      const float rate = (1.0F - blend) * catch_entry_rate_ + blend * feedback_rate;
      hip_rate_cmd_ += std::clamp(rate - hip_rate_cmd_,
          -config_.catch_hip_accel_rad_s2 * dt, config_.catch_hip_accel_rad_s2 * dt);
      // Intersect travel and tracking bounds; never wind up against a stall
      // or silently extend the allowed travel when encoder tracking is lost.
      const float lower = std::max(catch_start_hip_rad_ - config_.catch_hip_travel_rad,
          in.hip_measured_rad - config_.catch_hip_tracking_max_rad);
      const float upper = std::min(catch_start_hip_rad_ + config_.catch_hip_travel_rad,
          in.hip_measured_rad + config_.catch_hip_tracking_max_rad);
      if (lower > upper) {
        phase_ = Phase::Failed;
        return output(out);
      }
      hip_cmd_ = std::clamp(hip_cmd_ + hip_rate_cmd_ * dt, lower, upper);
      const bool captured = blend >= 1.0F &&
          std::abs(in.capture_lean_rad) <= config_.catch_lean_tol_rad &&
          std::abs(out.capture_point_rad) <= config_.catch_capture_tol_rad &&
          std::abs(in.capture_rate_rad_s) <= config_.catch_rate_tol_rad_s &&
          std::abs(in.pitch_rad) <= config_.catch_body_tol_rad &&
          std::abs(in.ground_speed_m_s) <= config_.catch_speed_tol_m_s &&
          std::abs(hip_cmd_ - in.hip_measured_rad) < 0.06F;
      pitch_settle_s_ = captured ? pitch_settle_s_ + dt : 0.0F;
      if (pitch_settle_s_ >= config_.catch_settle_s) phase_ = Phase::Support;
    }
    if (dt > 0.0F) hip_rate_cmd_ = (hip_cmd_ - previous_hip) / dt;
    if (phase_ == Phase::Catching) {
      const float blend = catchBlend();
      out.catch_blend = blend;
      const float accel = std::clamp(config_.catch_accel_gain * in.capture_lean_rad +
          config_.catch_accel_damping * in.capture_rate_rad_s +
          config_.catch_speed_gain * in.ground_speed_m_s,
          -config_.catch_accel_max, config_.catch_accel_max);
      const float vmax = config_.catch_wheel_max_rad_s * in.wheel_radius_m;
      // Bounds remain ordered even if the robot is rolling faster than vmax.
      wheel_velocity_m_s_ = std::clamp(wheel_velocity_m_s_ + blend * accel * dt,
          std::max(-vmax, std::min(in.ground_speed_m_s, 0.0F) - config_.catch_tracking_margin_m_s),
          std::min(vmax, std::max(in.ground_speed_m_s, 0.0F) + config_.catch_tracking_margin_m_s));
      out.wheel_rad_s = blend * std::clamp(
          (wheel_velocity_m_s_ + accel * config_.catch_wheel_lead_s) / in.wheel_radius_m -
              in.carrier_rate_rad_s,
          -config_.catch_wheel_max_rad_s, config_.catch_wheel_max_rad_s);
    }
    return output(out);
  }

  [[nodiscard]] Phase phase() const noexcept { return phase_; }
  [[nodiscard]] float time() const noexcept { return elapsed_s_; }
  [[nodiscard]] Config& config() noexcept { return config_; }
  [[nodiscard]] const Config& config() const noexcept { return config_; }
  static const char* name(Phase phase) noexcept {
    switch (phase) {
      case Phase::Idle: return "Idle";
      case Phase::Folding: return "Folding";
      case Phase::Kneeling: return "Kneeling";
      case Phase::Pitching: return "Pitching";
      case Phase::Flipped: return "Flipped";
      case Phase::Catching: return "Catching";
      case Phase::Support: return "Support";
      case Phase::Failed: return "Failed";
    }
    return "?";
  }

 private:
  float catchBlend() const noexcept {
    if (!catch_overlap_ || config_.catch_blend_s <= 0.0F) return 1.0F;
    const float t = std::clamp(elapsed_s_ / config_.catch_blend_s, 0.0F, 1.0F);
    return t * t * (3.0F - 2.0F * t);
  }

  bool valid(const Input& in) const noexcept {
    if (!in.sensors_ok || !std::isfinite(in.dt_s) || in.dt_s < 0.0F ||
        !std::isfinite(in.pitch_rad) || !std::isfinite(in.pitch_rate_rad_s) ||
        !std::isfinite(in.capture_lean_rad) || !std::isfinite(in.capture_rate_rad_s) ||
        !std::isfinite(in.hip_jacobian) || !std::isfinite(in.hip_measured_rad) ||
        !std::isfinite(in.carrier_rate_rad_s) || !std::isfinite(in.ground_speed_m_s) ||
        !std::isfinite(in.wheel_radius_m) || in.wheel_radius_m <= 0.0F ||
        !std::isfinite(in.capture_omega_rad_s) || in.capture_omega_rad_s <= 0.0F ||
        (phase_ == Phase::Catching && std::abs(in.hip_jacobian) < 0.1F)) return false;
    for (float v : {config_.fold_rate_rad_s, config_.fold_travel_rad, config_.fold_pitch_limit_rad,
         config_.kneel_hold_s, config_.flip_pitch_kp_rad_s_per_rad, config_.flip_pitch_kd,
         config_.flip_hip_rate_max_rad_s, config_.flip_travel_max_rad, config_.pitch_target_tol_rad,
         config_.pitch_rate_tol_rad_s, config_.pitch_settle_s,
         config_.lift_clearance_m, config_.lift_progress_rad, config_.lift_confirm_s,
         config_.catch_blend_s, config_.catch_hip_gain,
         config_.catch_hip_damping_s, config_.catch_body_gain, config_.catch_body_damping_s,
         config_.catch_wheel_peak_s, config_.catch_hip_rate_max_rad_s,
         config_.catch_hip_accel_rad_s2, config_.catch_hip_travel_rad, config_.catch_hip_tracking_max_rad,
         config_.catch_accel_gain, config_.catch_accel_damping, config_.catch_speed_gain,
         config_.catch_accel_max, config_.catch_wheel_lead_s, config_.catch_wheel_max_rad_s,
         config_.catch_tracking_margin_m_s, config_.catch_lean_tol_rad, config_.catch_capture_tol_rad,
         config_.catch_body_tol_rad, config_.catch_rate_tol_rad_s, config_.catch_speed_tol_m_s,
         config_.catch_settle_s, config_.pitch_limit_rad, config_.timeout_s}) {
      if (!std::isfinite(v) || v < 0.0F) return false;
    }
    // Software ceiling on one recovery request, not a motor thermal rating.
    return std::isfinite(config_.fold_direction) && config_.catch_wheel_peak_s <= 1.0F;
  }

  Output output(Output out) const noexcept {
    out.phase = phase_;
    out.hip_target_rad = hip_cmd_;
    out.knee_target_rad = knee_cmd_;
    out.wheels_on = phase_ != Phase::Failed && phase_ != Phase::Idle;
    out.hip_rate_rad_s = hip_rate_cmd_;
    out.phase_elapsed_s = elapsed_s_;
    out.wheel_peak_requested = phase_ == Phase::Catching && elapsed_s_ < config_.catch_wheel_peak_s;
    return out;
  }

  bool catch_leveling_ = true;
  bool flip_pitch_initialized_ = false, catch_overlap_ = false;
  float flip_start_pitch_rad_ = 0.0F, lift_clear_s_ = 0.0F, catch_entry_rate_ = 0.0F;
  float catch_stall_s_ = 0.0F;
  Config config_{};
  Phase phase_ = Phase::Idle;
  float elapsed_s_ = 0.0F, pitch_settle_s_ = 0.0F;
  float hip_start_rad_ = 0.0F, flip_start_hip_rad_ = 0.0F, catch_start_hip_rad_ = 0.0F;
  float hip_cmd_ = 0.0F, knee_cmd_ = 0.0F, hip_rate_cmd_ = 0.0F, wheel_velocity_m_s_ = 0.0F;
};

}  // namespace tap_fw::v3
