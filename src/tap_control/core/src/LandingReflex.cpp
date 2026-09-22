#include "robot_control/LandingReflex.hpp"

#include <algorithm>
#include <cmath>

#include "robot_control/WholeBodyModel.hpp"

namespace robot_control {

namespace {
constexpr auto kLh = static_cast<std::size_t>(JointId::LeftHip);
constexpr auto kLk = static_cast<std::size_t>(JointId::LeftKnee);
constexpr auto kRh = static_cast<std::size_t>(JointId::RightHip);
constexpr auto kRk = static_cast<std::size_t>(JointId::RightKnee);

float slewToward(float current, float target, float rate_rad_s, float dt_s) {
  if (rate_rad_s <= 0.0F) return target;
  const float step = std::max(rate_rad_s, 0.0F) * dt_s;
  return current + std::clamp(target - current, -step, step);
}
}  // namespace

LandingReflex::LandingReflex() : LandingReflex(Config{}) {}

void LandingReflex::reset() {
  reset(true);
}

void LandingReflex::reset(bool floor_contact) {
  phase_ = floor_contact ? Phase::Grounded : Phase::Airborne;
  prev_floor_contact_ = floor_contact;
  armed_by_unloading_ = false;
  unloading_elapsed_s_ = 0.0F;
  latched_hip_offset_rad_ = 0.0F;
  catch_elapsed_s_ = 0.0F;
  airborne_elapsed_s_ = 0.0F;
  extension_jacobian_ = 0.0F;
  capture_jacobian_ = 0.0F;
  extension_hip_offset_rad_ = 0.0F;
  extension_knee_offset_rad_ = 0.0F;
  last_extension_offset_rad_ = 0.0F;
}

JointArray LandingReflex::update(bool floor_contact, float pitch_rate_rad_s,
                                 float roll_rate_rad_s,
                                 float accel_magnitude_mps2,
                                 float forward_velocity_m_s,
                                 float com_height_m, float hip_base_rad,
                                 float knee_base_rad, float dt_s) {
  JointArray offset{};
  if (!std::isfinite(pitch_rate_rad_s) || !std::isfinite(roll_rate_rad_s) ||
      !std::isfinite(accel_magnitude_mps2) ||
      !std::isfinite(forward_velocity_m_s) || !std::isfinite(com_height_m) ||
      !std::isfinite(hip_base_rad) || !std::isfinite(knee_base_rad) ||
      !std::isfinite(dt_s) || dt_s <= 0.0F) {
    // Invalid input: stay put rather than latch a phase off bad data.
    return offset;
  }

  const bool unloading_sample = config_.unloading_accel_mps2 > 0.0F &&
      accel_magnitude_mps2 <= config_.unloading_accel_mps2;
  if (unloading_sample) {
    unloading_elapsed_s_ += dt_s;
  } else {
    unloading_elapsed_s_ = 0.0F;
  }
  const float unloading_debounce =
      std::max(config_.unloading_debounce_s, 0.0F);
  const bool unloading = unloading_sample &&
      (unloading_debounce <= 0.0F ||
       unloading_elapsed_s_ >= unloading_debounce);
  const bool airborne_signal = !floor_contact || unloading;
  const bool liftoff_edge = airborne_signal && prev_floor_contact_;
  prev_floor_contact_ = floor_contact;

  if (liftoff_edge && phase_ != Phase::Airborne) {
    phase_ = Phase::Airborne;
    armed_by_unloading_ = floor_contact && unloading;
    latched_hip_offset_rad_ =
        std::clamp(config_.hip_offset_per_pitch_rate * pitch_rate_rad_s,
                   -config_.max_hip_offset_rad, config_.max_hip_offset_rad);
    catch_elapsed_s_ = 0.0F;
    airborne_elapsed_s_ = armed_by_unloading_ ? 0.0F : dt_s;
    // Evaluate the real leg-height Jacobian once, at the resting hip/knee
    // target this fall started from - see active_extension_gravity_m_s2's
    // comment for why a fixed per-event value is an acceptable
    // approximation (the knee only moves by up to active_extension_max_rad
    // during the maneuver, a second-order effect on the Jacobian itself).
    // IMPORTANT: d_height_d_knee genuinely depends on BOTH hip and knee (the
    // shin's absolute orientation is hip+knee plus the calibrated zero
    // rebase) - passing hip=0 here would be
    // wrong for any real hip angle; verified directly against this robot's
    // real FK (2026-09-16): at hip=0.9/knee=-1.396 the true tallest-reach
    // point is knee~=-0.46 rad, NOT knee=0 as an earlier version of this
    // file's own header comment assumed (that was only ever true near
    // hip~=0, the pre-2026-09-16 neutral stance).
    const auto leg = WholeBodyModel::legHeight(hip_base_rad, knee_base_rad);
    extension_jacobian_ = leg.d_height_d_knee;
    capture_jacobian_ = leg.d_x_d_hip;
    extension_hip_offset_rad_ = 0.0F;
    extension_knee_offset_rad_ = 0.0F;
    // Capture-point offset latched HERE, at liftoff, using the real launch
    // forward_velocity_m_s/com_height_m - NOT recomputed live through
    // Catching (2026-09-16 fix: the first version recomputed this every
    // tick off state.wheel.velocity_m_s for the ENTIRE Airborne+Catching
    // window; measured on a real 50cm drop to make results WORSE than
    // without it - root cause found by re-reading this exact code path: post
    // -touchdown wheel velocity is genuinely chaotic (impact spikes,
    // bounces), and feeding that raw, unfiltered signal into a live hip
    // position command during Catching - the one window every other
    // mechanism here deliberately HOLDS a value through, not chases a noisy
    // one - injected exactly the kind of noise-driven jitter this file's
    // other latched terms were built to avoid. Capture-point theory itself
    // is about a PRE-touchdown placement decision from the launch
    // trajectory, not a continuously-revised one from an already-landed
    // wheel, so latching at liftoff is the theoretically correct choice too,
    // not just the pragmatic one - ballistic forward velocity barely changes
    // during flight anyway (no horizontal deceleration), so nothing real is
    // lost relative to a live Airborne-only value.
    const bool capture_enabled = config_.capture_point_gravity_m_s2 > 0.0F &&
        std::abs(capture_jacobian_) > 1.0e-4F;
    if (capture_enabled) {
      const float omega = std::sqrt(config_.capture_point_gravity_m_s2 /
                                    std::max(com_height_m, 0.01F));
      const float capture_offset_m = forward_velocity_m_s / std::max(omega, 1.0e-3F);
      const float capture_term = std::clamp(
          capture_offset_m / capture_jacobian_,
          -std::max(config_.capture_point_max_hip_rad, 0.0F),
          std::max(config_.capture_point_max_hip_rad, 0.0F));
      latched_hip_offset_rad_ += capture_term;
    }
  } else if (phase_ == Phase::Airborne) {
    if (!floor_contact) {
      armed_by_unloading_ = false;
    }
    if (!armed_by_unloading_) {
      airborne_elapsed_s_ += dt_s;
    }
    // Level check, not edge-triggered: a spuriously-true support hypothesis
    // this soon after liftoff (estimator transient, see
    // min_airborne_before_touchdown_s above) must not consume the one
    // false->true edge and get stuck Airborne forever once rejected - retry
    // every tick floor_contact reads true until either the debounce elapses
    // (real touchdown, transition to Catching) or contact drops again
    // (confirmed still airborne, no state change either way).
    if (!armed_by_unloading_ && floor_contact &&
        airborne_elapsed_s_ >= config_.min_airborne_before_touchdown_s) {
      phase_ = Phase::Catching;
      catch_elapsed_s_ = 0.0F;
    }
  }

  // Active velocity-matched extension: during confirmed free fall, closing
  // velocity grows as g*t (real free-fall kinematics, no distance sensor
  // needed - see Config::active_extension_gravity_m_s2). The knee offset
  // that would let the leg's own extension rate match that closing
  // velocity is offset(t) = integral of v(tau)/|J| dtau = g*t^2/(2*|J|),
  // signed by -1/J so it comes out positive (extending) for this robot's
  // real Jacobian sign at typical crouched poses - clamped to
  // [0, active_extension_max_rad] either way as the hard safety bound.
  // 0 gravity (default) or 0 Jacobian both collapse this to exactly 0, an
  // exact no-op.
  const bool extension_enabled = config_.active_extension_gravity_m_s2 > 0.0F &&
      std::abs(extension_jacobian_) > 1.0e-4F;
  float extension_hip_target = 0.0F;
  float extension_knee_target = 0.0F;
  if (extension_enabled) {
    const auto leg = WholeBodyModel::legHeight(hip_base_rad, knee_base_rad);
    // Preserve the legacy radian cap as a conservative distance cap by
    // projecting it through the knee height Jacobian at liftoff.
    const float max_extension_m =
        std::max(config_.active_extension_max_rad, 0.0F) *
        std::abs(extension_jacobian_);
    const float extension_elapsed_s = std::max(
        0.0F, airborne_elapsed_s_ - std::max(config_.active_extension_delay_s, 0.0F));
    const float desired_extension_m = std::clamp(
        0.5F * config_.active_extension_gravity_m_s2 *
            extension_elapsed_s * extension_elapsed_s,
        0.0F, max_extension_m);
    const float desired_dh = -desired_extension_m;  // more negative height = wheel lower.
    constexpr float desired_dx = 0.0F;              // avoid a fore/aft kick.
    const float det =
        leg.d_height_d_hip * leg.d_x_d_knee -
        leg.d_height_d_knee * leg.d_x_d_hip;
    if (std::abs(det) > 1.0e-5F) {
      extension_hip_target =
          (leg.d_x_d_knee * desired_dh -
           leg.d_height_d_knee * desired_dx) / det;
      extension_knee_target =
          (-leg.d_x_d_hip * desired_dh +
           leg.d_height_d_hip * desired_dx) / det;
    } else {
      extension_knee_target = -(desired_extension_m / extension_jacobian_);
    }
    const float hip_cap = config_.active_extension_max_hip_rad >= 0.0F
        ? config_.active_extension_max_hip_rad
        : std::max(config_.active_extension_max_rad, 0.0F);
    extension_hip_target = std::clamp(
        extension_hip_target, -hip_cap, hip_cap);
    if (std::abs(leg.d_height_d_knee) > 1.0e-5F) {
      extension_knee_target =
          (desired_dh - leg.d_height_d_hip * extension_hip_target) /
          leg.d_height_d_knee;
    }
    extension_knee_target = std::clamp(
        extension_knee_target, 0.0F,
        std::max(config_.active_extension_max_rad, 0.0F));
  }
  extension_hip_offset_rad_ = slewToward(
      extension_hip_offset_rad_, extension_hip_target,
      config_.target_slew_rate_rad_s, dt_s);
  extension_knee_offset_rad_ = slewToward(
      extension_knee_offset_rad_, extension_knee_target,
      config_.target_slew_rate_rad_s, dt_s);

  float knee_offset = 0.0F;
  float hip_offset = 0.0F;
  if (phase_ == Phase::Airborne && !armed_by_unloading_) {
    knee_offset = -std::abs(config_.airborne_knee_crouch_rad) +
                  extension_knee_offset_rad_;
    hip_offset = latched_hip_offset_rad_ + extension_hip_offset_rad_;
    last_extension_offset_rad_ = extension_knee_offset_rad_;
  } else if (phase_ == Phase::Catching) {
    catch_elapsed_s_ += dt_s;
    const float hold = std::max(config_.catch_hold_s, 0.0F);
    const float blend = std::max(config_.catch_blend_s, 1.0e-3F);
    float scale = 1.0F;
    if (catch_elapsed_s_ > hold) {
      scale = std::clamp(1.0F - (catch_elapsed_s_ - hold) / blend, 0.0F, 1.0F);
    }
    knee_offset = (-std::abs(config_.airborne_knee_crouch_rad) +
                  last_extension_offset_rad_) * scale;
    hip_offset = (latched_hip_offset_rad_ + extension_hip_offset_rad_) * scale;
    if (scale <= 0.0F) {
      phase_ = Phase::Grounded;
    }
  }

  // Differential roll term: only meaningful while a fall/landing event is
  // actually in progress (Grounded phase leaves it exactly 0, matching every
  // other mechanism here's "default is a no-op" contract). Applied to the
  // HIP, not the knee - see Config::roll_rate_hip_gain_rad_per_rad_s's own
  // comment for the physical justification (the knee's stiffness is
  // deliberately crushed by LandingImpedance during exactly this window;
  // the hip's is not) and the sign caveat.
  const bool roll_active = phase_ == Phase::Airborne || phase_ == Phase::Catching;
  const float roll_term = roll_active
      ? std::clamp(config_.roll_rate_hip_gain_rad_per_rad_s * roll_rate_rad_s,
                   -std::max(config_.roll_hip_offset_max_rad, 0.0F),
                   std::max(config_.roll_hip_offset_max_rad, 0.0F))
      : 0.0F;

  // Capture-point hip placement is now latched into hip_offset at liftoff
  // (see the liftoff_edge block above) and rides the SAME hold+blend as
  // latched_hip_offset_rad_ - no separate live term here anymore.
  offset[kLh] = hip_offset + roll_term;
  offset[kRh] = hip_offset - roll_term;
  offset[kLk] = knee_offset;
  offset[kRk] = knee_offset;
  return offset;
}

}  // namespace robot_control
