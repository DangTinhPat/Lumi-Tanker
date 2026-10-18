#include "tap_fw/v3/V3Controller.hpp"

#include "robot_control/BodyClearance.hpp"

#include <algorithm>
#include <cmath>

#include "robot_control/GeneratedGeometry.hpp"
#include "robot_control/TunedConfig.hpp"
#include "robot_control/WholeBodyModel.hpp"
#include "tap_fw/BoardConfig.hpp"

namespace tap_fw::v3 {

// Height planning, knee hold and suspension/posture offsets.

float V3Controller::kneeHold(float dhip) noexcept {
  if (!kneeHoldOn() || dhip == 0.0F) {
    knee_hold_ = 0.0F;
    return 0.0F;
  }
  const float hip = LegHeight::hangHipCad() + pose_.hip_offset_rad + dhip;
  const float target = -pose_.axle_below_hip_m;  // legHeight's sign: z up
  float k = std::clamp(pose_.knee_offset_rad + knee_hold_, LegHeight::kKneeMinRad,
                       LegHeight::kKneeMaxRad);
  for (int i = 0; i < 3; ++i) {
    const auto leg = robot_control::WholeBodyModel::legHeight(hip, k);
    if (std::abs(leg.d_height_d_knee) < 1.0e-4F) break;
    const float step = (leg.height_m - target) / leg.d_height_d_knee;
    k = std::clamp(k - step, LegHeight::kKneeMinRad, LegHeight::kKneeMaxRad);
    if (std::abs(step) < 1.0e-5F) break;
  }
  knee_hold_ = k - pose_.knee_offset_rad;
  return knee_hold_;
}

float V3Controller::planCap(float vmax) const noexcept {
  const Tier& t = config_.tier;
  if (!envelopeOn()) return std::clamp(t.plan_vmax_m_s, 0.05F, vmax);
  const float l = std::max(pose_.com_above_axle_m, 0.01F);
  return std::clamp(vmax - t.env_catch_rad * std::sqrt(9.80665F * l), 0.05F, vmax);
}

void V3Controller::updatePlan(float target_v, float dt, bool reset) {
  const Tier& t = config_.tier;
  if (reset || !plan_init_) {
    plan_v_ = reset ? 0.0F : target_v;
    plan_a_ = plan_j_ = 0.0F;
    plan_n_ = 0U;
    plan_init_ = true;
  }
  const float amax = envelopeOn() && wholeBody()
      ? std::max(9.80665F * std::tan(std::clamp(t.env_lean_rad, 0.0F, 0.3F)), 0.05F)
      : std::max(t.plan_accel_m_s2, 0.05F);
  float jmax = std::max(t.plan_jerk_m_s3, 0.1F);
  if (envelopeOn()) {
    jmax *= std::sqrt(std::max(com_reference_m_, 0.01F) /
                       std::max(pose_.com_above_axle_m, 0.01F));
  }
  // Jerk-limited (S-curve) tracking of the operator speed: aim the
  // acceleration at the sign of the error left after ramping the current
  // acceleration back to zero at jmax, so it lands on the target without
  // overshoot.
  const float v_stop = plan_v_ + plan_a_ * std::abs(plan_a_) / (2.0F * jmax);
  const float err = target_v - v_stop;
  const float a_des = std::abs(err) < 1.0e-4F ? 0.0F : std::copysign(amax, err);
  const float da = std::clamp(a_des - plan_a_, -jmax * dt, jmax * dt);
  plan_a_ += da;
  plan_j_ = dt > 0.0F ? da / dt : 0.0F;
  plan_v_ += plan_a_ * dt;
  if (std::abs(target_v - plan_v_) < 2.0e-3F && std::abs(plan_a_) <= 2.0F * jmax * dt) {
    plan_v_ = target_v;
    plan_a_ = plan_j_ = 0.0F;
  }
  plan_buf_[plan_head_] = {plan_v_, plan_a_};
  plan_head_ = (plan_head_ + 1U) % plan_buf_.size();
  if (plan_n_ < plan_buf_.size()) ++plan_n_;
  plan_dt_ = dt;
}

V3Controller::PlanSample V3Controller::planDelayed() const noexcept {
  return planAt(config_.tier.preview_s);
}

V3Controller::PlanSample V3Controller::planAt(float delay_s) const noexcept {
  const float dt = std::max(plan_dt_, 1.0e-4F);
  std::size_t lag = static_cast<std::size_t>(std::lround(std::max(delay_s, 0.0F) / dt));
  lag = std::min({lag, plan_buf_.size() - 1U, plan_n_ > 0U ? plan_n_ - 1U : 0U});
  const std::size_t i = (plan_head_ + plan_buf_.size() - 1U - lag) % plan_buf_.size();
  return plan_n_ > 0U ? plan_buf_[i] : PlanSample{plan_v_, plan_a_};
}

float V3Controller::heightStoppingDistance(float speed_m_s, float accel_m_s2,
                                           float max_accel_m_s2,
                                           float max_jerk_m_s3) noexcept {
  const float speed = std::max(speed_m_s, 0.0F);
  const float accel = std::max(max_accel_m_s2, 1.0e-3F);
  const float jerk = std::max(max_jerk_m_s3, 1.0e-3F);
  const float a0 = std::clamp(accel_m_s2, -accel, accel);
  if (speed <= 1.0e-7F && a0 <= 0.0F) return 0.0F;

  // Apply maximum reverse jerk until -accel, then maximum deceleration. This
  // includes the extra distance caused by the acceleration ramp; v^2/(2a)
  // alone stops too early once the profile is jerk limited.
  const float ramp_s = std::max((a0 + accel) / jerk, 0.0F);
  const float speed_after_ramp = speed + a0 * ramp_s - 0.5F * jerk * ramp_s * ramp_s;
  const float distance_after_ramp = speed * ramp_s + 0.5F * a0 * ramp_s * ramp_s -
      (jerk * ramp_s * ramp_s * ramp_s) / 6.0F;
  if (speed_after_ramp <= 0.0F) {
    // The speed reaches zero before the acceleration ramp reaches its limit.
    const float stop_s = (a0 + std::sqrt(std::max(a0 * a0 + 2.0F * jerk * speed, 0.0F))) /
        jerk;
    const float distance = speed * stop_s + 0.5F * a0 * stop_s * stop_s -
        (jerk * stop_s * stop_s * stop_s) / 6.0F;
    return std::max(distance, 0.0F);
  }
  return std::max(distance_after_ramp + speed_after_ramp * speed_after_ramp / (2.0F * accel),
                  0.0F);
}

float V3Controller::heightStopTarget() const noexcept {
  const float speed = height_speed_m_s_;
  const float direction = std::copysign(1.0F, speed);
  const Height& h = config_.height;
  const float max_accel = height_startup_
      ? std::min(h.accel_m_s2, h.startup_accel_m_s2) : h.accel_m_s2;
  const float max_jerk = height_startup_
      ? std::min(h.jerk_m_s3, h.startup_jerk_m_s3) : h.jerk_m_s3;
  const float distance = heightStoppingDistance(std::abs(speed),
      height_accel_m_s2_ * direction, max_accel, max_jerk);
  const float current = LegHeight::forKnee(knee_cmd_rad_).axle_below_hip_m;
  const float hmax = supportedMaxHeight();
  return std::clamp(current + direction * distance, LegHeight::minHeight(), hmax);
}

void V3Controller::updateHeight(const Input& in, const Command& cmd) {
  const Height& h = config_.height;
  // A straight knee has no height-preserving fore/aft workspace. Reserve
  // the existing 12 mm for paired trunk motion even if roll is switched off.
  const float hmax = supportedMaxHeight();
  if (height_startup_ || cmd.height_m > 0.0F || height_target_m_ > hmax + 1.0e-4F) {
    const float want = height_startup_ ? startup_height_m_
        : cmd.height_m > 0.0F ? cmd.height_m : height_target_m_;
    const float target = std::clamp(want, LegHeight::minHeight(), hmax);
    if (std::abs(target - height_target_m_) > 1.0e-4F) {
      height_target_m_ = target;
      knee_target_rad_ = LegHeight::forHeight(target).knee_offset_rad;
    }
  }
  // OscillationGuard raise: the body goal is lifted by the guard while the loop is swinging; the plan below runs
  // toward target_now / knee_goal with the usual rate, acceleration and jerk limits.
  const float target_now = std::clamp(height_target_m_ + guard_raise_m_, LegHeight::minHeight(), hmax);
  const float knee_goal = kneeGoalRad();
  if (knee_cmd_rad_ == knee_goal) {
    height_speed_m_s_ = 0.0F;
    height_accel_m_s2_ = 0.0F;
    // Keep the delay even on a retry already at the reserved height.
    if (in.now_us - state_since_us_ >=
        static_cast<std::uint64_t>(std::max(h.startup_delay_s, 0.0F) * 1.0e6F)) {
      height_startup_ = false;
    }
    return;
  }
  if (height_startup_ && in.now_us - state_since_us_ <
      static_cast<std::uint64_t>(std::max(h.startup_delay_s, 0.0F) * 1.0e6F)) {
    height_speed_m_s_ = 0.0F;
    height_accel_m_s2_ = 0.0F;
    return;
  }
  // Speed quality: full while held (stand, hands: the wheels are not
  // carrying it, so leg motion cannot upset it), otherwise falling smoothly
  // to zero as the balance gets busy - never an on/off gate.
  float quality = 1.0F;
  if (held_quiet_) {
    quality = 0.0F;
  } else if (!held_outer_off_) {
    quality = std::clamp(1.0F - std::abs(in.pitch_rad - pid_ref_) / h.max_pitch_error_rad,
                         0.0F, 1.0F) *
        std::clamp(1.0F - std::abs(in.pitch_rate_rad_s) / h.max_pitch_rate_rad_s, 0.0F, 1.0F);
  }
  // Signed height speed (+ = up): cap speed to the balance quality and to
  // the jerk-aware distance needed to stop at the target. Then shape both
  // acceleration and speed so reversals remain continuous too.
  // pose_ may include last tick's suspension/excitation offset. The nominal
  // height planner must not integrate that offset back into its own target.
  const auto planned = LegHeight::forKnee(knee_cmd_rad_);
  const float remaining = target_now - planned.axle_below_hip_m;
  // A descent slows to a stop as the oscillation risk grows (never an on/off gate); a rise is never held back.
  if (config_.guard.mode >= 2 && remaining < 0.0F) {
    quality *= std::clamp(1.0F - guard_out_.risk / std::max(config_.guard.hold_risk, 0.05F), 0.0F, 1.0F);
  }
  const float accel = std::max(height_startup_
      ? std::min(h.accel_m_s2, h.startup_accel_m_s2) : h.accel_m_s2, 1.0e-3F);
  const float jerk = std::max(height_startup_
      ? std::min(h.jerk_m_s3, h.startup_jerk_m_s3) : h.jerk_m_s3, 1.0e-3F);
  const float rate = std::max(height_startup_
      ? std::min(h.rate_m_s, h.startup_rate_m_s) : h.rate_m_s, 0.0F);
  const float direction = std::copysign(1.0F, remaining);
  const float max_speed = rate * quality;
  // Reserve enough speed headroom to ramp a positive acceleration back to
  // zero under the jerk limit. Without this anticipatory reduction, the
  // actual profile velocity can exceed rate by a^2/(2j) while acceleration
  // unwinds, even though the requested velocity is clamped to max_speed.
  const float accel_toward_target = std::max(height_accel_m_s2_ * direction, 0.0F);
  const float unwind_speed = 0.5F * accel_toward_target * accel_toward_target / jerk;
  float speed_limit = std::max(max_speed - unwind_speed, 0.0F);
  if (max_speed > 0.0F && height_speed_m_s_ * direction > 0.0F) {
    const float current_accel_toward_target = height_accel_m_s2_ * direction;
    float lo = 0.0F;
    float hi = std::max(max_speed, height_speed_m_s_ * direction);
    if (heightStoppingDistance(hi, current_accel_toward_target, accel, jerk) >
        std::abs(remaining)) {
      for (int i = 0; i < 18; ++i) {
        const float mid = 0.5F * (lo + hi);
        if (heightStoppingDistance(mid, current_accel_toward_target, accel, jerk) <=
            std::abs(remaining)) {
          lo = mid;
        } else {
          hi = mid;
        }
      }
      speed_limit = std::min(speed_limit, lo);
    }
  }
  const float v_des = direction * speed_limit;
  const float dt = std::max(in.dt_s, 1.0e-4F);
  const float accel_des = std::clamp((v_des - height_speed_m_s_) / dt, -accel, accel);
  height_accel_m_s2_ += std::clamp(accel_des - height_accel_m_s2_, -jerk * dt, jerk * dt);
  height_accel_m_s2_ = std::clamp(height_accel_m_s2_, -accel, accel);
  height_speed_m_s_ += height_accel_m_s2_ * dt;
  // Integrate in the quantity whose speed is limited, then use exact IK.
  // The old 0.2 mm endpoint snap could jump ~0.1 mm in one 2 ms tick,
  // exceeding the startup rate even when its internal speed was near zero.
  const float next_height = planned.axle_below_hip_m + height_speed_m_s_ * in.dt_s;
  const bool passed = remaining < 0 ? next_height <= target_now
                                   : next_height >= target_now;
  // Treat micrometre-scale FK/float residuals as reached so the speed
  // profile cannot stall below its endpoint forever.
  constexpr float kHeightEndpointToleranceM = 1.0e-5F;
  if (passed || std::abs(remaining) < kHeightEndpointToleranceM) {
    knee_cmd_rad_ = knee_goal;
    height_speed_m_s_ = 0.0F;
    height_accel_m_s2_ = 0.0F;
    height_startup_ = false;
  } else {
    knee_cmd_rad_ = LegHeight::forHeight(next_height).knee_offset_rad;
  }
  pose_ = LegHeight::forKnee(knee_cmd_rad_);
  legs_moved_us_ = in.now_us;
}

void V3Controller::updateSuspension(const Input& in) {
  const Tier& t = config_.tier;
  // Specific force along gravity at the IMU (hip origin): its slow mean is
  // gravity (and the accelerometer's scale/bias), the rest is the body's
  // vertical acceleration.
  const float fzw = -in.accel_x_m_s2 * std::sin(body_pitch_) + in.accel_z_m_s2 * std::cos(body_pitch_);
  if (!sus_init_) {
    sus_g_ = fzw;
    sus_init_ = true;
  }
  sus_g_ += std::clamp(in.dt_s / 2.0F, 0.0F, 1.0F) * (fzw - sus_g_);
  // Off during KNEETEST FLOOR: it would fight the excitation it measures.
  const bool on = t.suspension && tierVirtual() && !held_outer_off_ && !held_quiet_ &&
      knee_excite_rad_ == 0.0F;
  // Startup reserve motion is a commanded posture change, not a bump.
  // Do not integrate its transient and release it as a knee offset later.
  if (height_startup_) height_settled_since_us_ = 0U;
  else if (height_settled_since_us_ == 0U) height_settled_since_us_ = in.now_us;
  const bool settled = height_settled_since_us_ != 0U &&
      in.now_us - height_settled_since_us_ >=
      static_cast<std::uint64_t>(std::max(t.sus_startup_settle_s, 0.0F) * 1.0e6F);
  const float target = on && settled ? 1.0F : 0.0F;
  const float blend_step = in.dt_s / std::max(t.sus_startup_ramp_s, 1.0e-3F);
  sus_alpha_ += std::clamp(target - sus_alpha_, -blend_step, blend_step);
  if (on && settled) {
    // Complementary: integrate the IMU, pulled towards the legs' height rate
    // (the body's vertical speed on a flat floor) with sus_leak_s.
    float vz_legs = 0.0F;
    for (std::size_t side = 0; side < 2U; ++side) {
      const auto v = robot_control::WholeBodyModel::imuVelocityThroughLeg(
          body_pitch_, body_rate_, LegHeight::hangHipCad() + in.hip_rad[side], in.knee_rad[side],
          in.hip_rate_rad_s[side], in.knee_rate_rad_s[side], in.wheel_rad_s[side],
          config_.wheel_radius_m);
      vz_legs += 0.5F * v.vertical_m_s;
    }
    sus_vz_ += in.dt_s * (fzw - sus_g_) +
        in.dt_s / std::max(t.sus_leak_s, 0.05F) * (vz_legs - sus_vz_);
  } else {
    sus_vz_ = 0.0F;
  }
  // Relative to the planned height change: a commanded raise/lower is the
  // body moving on purpose (damping it fought the plan: 90 mm ended 6 mm high).
  const float vz_err = sus_vz_ - height_speed_m_s_;
  float db = std::max(t.sus_deadband_m_s, 0.0F);
  bool sus_active = on;
  if (t.sus_adaptive) {
    if (on && settled) {
      sus_vz2_ += std::clamp(in.dt_s / std::max(t.sus_noise_tau_s, 0.2F), 0.0F, 1.0F) *
          (vz_err * vz_err - sus_vz2_);
    } else {
      sus_vz2_ = 0.0F;
    }
    db = std::max(db, t.sus_noise_k * std::sqrt(sus_vz2_));
    const bool waited = in.now_us - state_since_us_ >=
        static_cast<std::uint64_t>(std::max(t.sus_engage_delay_s, 0.0F) * 1.0e6F);
    const bool calm = std::abs(ground_v_) < t.sus_calm_speed_m_s &&
        std::abs(body_rate_) < t.sus_calm_rate_rad_s;
    sus_active = on && waited && calm;
  }
  sus_db_eff_ = db;
  const float vz_eff = std::copysign(std::max(std::abs(vz_err) - db, 0.0F), vz_err);
  const float rate = sus_active ? -t.sus_gain * sus_alpha_ * vz_eff : 0.0F;
  const float lim = std::max(t.sus_max_m, 0.0F);
  sus_dz_ = std::clamp(sus_dz_ + in.dt_s * (rate - sus_dz_ / std::max(t.sus_return_s, 0.05F)),
                       -lim, lim);
  // Invert the SAME coordinated hip/knee path used by applyPoseOffsets.
  // A partial knee Jacobian at a mixed (offset hip / nominal knee) pose
  // does not describe that path. Clamp the height state too: no windup at
  // a straight knee or at the lower end of the supported model range.
  const auto base = LegHeight::forKnee(knee_cmd_rad_);
  const float target_height = std::clamp(base.axle_below_hip_m + sus_dz_,
                                        LegHeight::minHeight(), LegHeight::maxHeight());
  sus_dz_ = target_height - base.axle_below_hip_m;
  sus_dk_ = std::abs(sus_dz_) < 1.0e-7F ? 0.0F
      : LegHeight::forHeight(target_height).knee_offset_rad - knee_cmd_rad_;
}

void V3Controller::applyPoseOffsets(const Input& in) {
  updateSuspension(in);
  const float dk = knee_excite_rad_ + sus_dk_;
  if (dk == 0.0F) {
    if (excite_applied_) {
      pose_ = LegHeight::forKnee(knee_cmd_rad_);
      excite_applied_ = false;
    }
    return;
  }
  // Evaluate the nominal pose/COM only when the offset or the height pose
  // under it moved; the analytic IK needs no Newton iterations.
  if (excite_applied_ && std::abs(dk - applied_dk_) < 2.0e-4F && applied_base_ == knee_cmd_rad_) {
    return;
  }
  pose_ = LegHeight::forKnee(std::clamp(knee_cmd_rad_ + dk, LegHeight::kKneeMinRad,
                                       LegHeight::kKneeMaxRad));
  applied_dk_ = dk;
  applied_base_ = knee_cmd_rad_;
  excite_applied_ = true;
  // The joint test is a deliberate leg motion (held test off); the
  // suspension's few mm are not.
  if (knee_excite_rad_ != 0.0F) legs_moved_us_ = in.now_us;
}


}  // namespace tap_fw::v3
