#include "tap_fw/v3/RockerLower.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "robot_control/GeneratedGeometry.hpp"
#include "robot_control/WholeBodyModel.hpp"

namespace tap_fw::v3 {
namespace {

// COM lean angle about the axle (+ : P ahead of N along +X) and its radius,
// from joint encoders and IMU pitch only.
struct Lean {
  float angle_rad;
  float radius_m;
};

Lean comLean(float hip_rad, float knee_rad, float pitch_rad) noexcept {
  const auto body = robot_control::WholeBodyModel{}.compute(hip_rad, knee_rad,
                                                            hip_rad, knee_rad);
  return {pitch_rad + body.com_line_angle_rad,
          std::hypot(body.com_offset_x_m, body.com_above_axle_m)};
}

// The hip angle where WholeBodyModel's com_line_angle_rad(hip, knee) = 0 for
// a given knee, i.e. the one hip that puts pitch 0 (body level) and lean 0
// (COM over the axle) at the same instant - the same bisection used offline
// to derive Config::level_target_hip_rad for knee = -1.2, generalised to any
// knee so a height change (which moves the knee) can keep tracking it live.
float levelHipForKnee(float knee_rad) noexcept {
  float lo = 0.0F, hi = 2.0F;
  for (int i = 0; i < 16; ++i) {
    const float mid = 0.5F * (lo + hi);
    const auto body = robot_control::WholeBodyModel{}.compute(mid, knee_rad, mid, knee_rad);
    if (body.com_line_angle_rad < 0.0F) lo = mid; else hi = mid;
  }
  return 0.5F * (lo + hi);
}

// Absolute ground position from a rolling wheel, exactly like V3Controller's
// own ground_x_: the wheel encoder, IMU pitch and leg (hip+knee) encoders are
// each an absolute angle, and their sum is the wheel's absolute world angle,
// so this needs no integrated (and therefore drifting) velocity estimate.
float groundPositionM(const RockerLower::Input& in) noexcept {
  return robot_control::geometry::wheel_radius_m *
      (in.wheel_pos_rad + in.pitch_rad + in.hip_rad + in.knee_rad);
}

}  // namespace

float RockerLower::nToP(float hip_rad, float knee_rad, float pitch_rad) noexcept {
  const Lean lean = comLean(hip_rad, knee_rad, pitch_rad);
  return lean.radius_m * std::sin(lean.angle_rad);
}

void RockerLower::reset(float hip_rad, float knee_rad) noexcept {
  phase_ = Phase::Lowering;
  hip_cmd_rad_ = hip_rad;
  knee_hold_rad_ = knee_rad;
  best_n_to_p_m_ = std::numeric_limits<float>::infinity();
  phase_elapsed_s_ = 0.0F;
  speed_cmd_m_s_ = 0.0F;
  ground_x0_m_ = 0.0F;
  drive_target_m_ = 0.0F;
  driving_ = false;
  knee_target_rad_ = knee_rad;
}

// Offline MJCF study (2026-09-28/29, both legs and both wheels together):
// folding the hips alone cannot lift the belly off the floor at this stance -
// the hip's reaction torque just back-drives the wheels (0.13 Nm/wheel is far
// too weak to hold the axle in place, let alone push it under the COM).
// A wheelie - both wheels pushing forward hard while the hips turn the body
// up - works once the wheels can supply roughly the torque needed to swing
// the COM over the axle (~m*g*(N-P), a few tenths of a Nm here): the wheel's
// forward reaction torque does the lifting, the hip twist only aims it.
// Measured at 0.8 Nm/wheel with a 2.5 rad/s hip twist (both user-tuned, the
// twist rate down from an initial 6 rad/s that launched the robot airborne):
// the belly clears the floor and a plain lean-only wheel balancer (no
// position/velocity outer loop - that would need root odometry this robot
// does not have) catches it with the COM within ~3 mm of the axle. Turning
// the hips back down afterward (fast while far from level, slow near it)
// brings the body to level in a further ~0.25 s.
RockerLower::Output RockerLower::update(const Input& in) noexcept {
  Output out{};
  const Lean lean = comLean(in.hip_rad, in.knee_rad, in.pitch_rad);
  out.n_to_p_m = lean.radius_m * std::sin(lean.angle_rad);
  phase_elapsed_s_ += in.dt_s;
  if (phase_ == Phase::Balanced && in.height_change_requested) {
    knee_target_rad_ = std::clamp(in.requested_knee_rad,
        robot_control::geometry::native_knee_min_rad, 0.0F);
  }

  if ((phase_ == Phase::Lowering || phase_ == Phase::Flinging) &&
      (!in.sensors_ok || !std::isfinite(out.n_to_p_m) ||
       in.pitch_rad + in.hip_rad < config_.leg_angle_min_rad)) {
    phase_ = Phase::Aborted;
    hip_cmd_rad_ = in.hip_rad;
  } else if ((phase_ == Phase::Wheelie || phase_ == Phase::Catch ||
              phase_ == Phase::Level || phase_ == Phase::Balanced) &&
             (!in.sensors_ok || !std::isfinite(lean.angle_rad))) {
    phase_ = Phase::Aborted;
  } else if ((phase_ == Phase::Catch || phase_ == Phase::Level ||
              phase_ == Phase::Balanced) &&
             std::abs(lean.angle_rad) > config_.fall_lean_rad) {
    phase_ = Phase::Aborted;
  }

  // First pass: advance the hip target and any phase transition. This never
  // reads or writes the wheel command, so a same-tick transition (e.g.
  // Lowering -> Flinging) cannot leave a stale wheel value from the phase
  // being left.
  switch (phase_) {
    case Phase::Lowering:
      // Slow fold until the floor, or earlier if N-P turns back up.
      best_n_to_p_m_ = std::min(best_n_to_p_m_, std::abs(out.n_to_p_m));
      hip_cmd_rad_ = std::max(hip_cmd_rad_ - config_.hip_rate_rad_s * in.dt_s,
                              config_.hip_floor_rad);
      // Judge the command, not the encoder: the loaded hip servo sits
      // 0.05-0.07 rad short of its target.
      if (std::abs(out.n_to_p_m) > best_n_to_p_m_ + config_.rise_margin_m ||
          hip_cmd_rad_ <= config_.hip_floor_rad) {
        phase_ = Phase::Flinging;
      }
      break;
    case Phase::Flinging:
      hip_cmd_rad_ = std::max(hip_cmd_rad_ - config_.fling_rate_rad_s * in.dt_s,
                              config_.fling_lock_rad);
      if (hip_cmd_rad_ <= config_.fling_lock_rad) {
        phase_ = Phase::Pivot;
        phase_elapsed_s_ = 0.0F;
      }
      break;
    case Phase::Pivot:
      if (phase_elapsed_s_ >= config_.pivot_hold_s) {
        phase_ = Phase::Wheelie;
        phase_elapsed_s_ = 0.0F;
      }
      break;
    case Phase::Wheelie:
      hip_cmd_rad_ = std::min(hip_cmd_rad_ + config_.wheelie_hip_rate_rad_s * in.dt_s,
                              config_.wheelie_hip_target_rad);
      if (phase_elapsed_s_ >= config_.wheelie_min_time_s &&
          std::abs(lean.angle_rad) < config_.catch_lean_max_rad) {
        phase_ = Phase::Catch;
        phase_elapsed_s_ = 0.0F;
        speed_cmd_m_s_ = 0.0F;
        ground_x0_m_ = groundPositionM(in);
      } else if (phase_elapsed_s_ > config_.wheelie_timeout_s) {
        phase_ = Phase::Aborted;
      }
      break;
    case Phase::Catch:
      if (phase_elapsed_s_ >= config_.catch_settle_s) {
        phase_ = Phase::Level;
        phase_elapsed_s_ = 0.0F;
      }
      break;
    case Phase::Level:
      // The balancer (below) continuously re-aims pitch to keep lean at 0
      // for whatever hip currently is, so pitch cannot be used to judge how
      // close the hip is to the level target - only the hip encoder can.
      if (hip_cmd_rad_ > config_.level_target_hip_rad) {
        const float rate = std::clamp(
            config_.level_gain * (hip_cmd_rad_ - config_.level_target_hip_rad),
            config_.level_min_rad_s, config_.level_max_rad_s);
        hip_cmd_rad_ = std::max(hip_cmd_rad_ - rate * in.dt_s, config_.level_target_hip_rad);
      } else {
        phase_ = Phase::Balanced;
      }
      break;
    case Phase::Balanced: {
      const bool settled =
          std::abs(in.pitch_rad) < config_.height_settle_pitch_rad &&
          std::abs(in.pitch_rate_rad_s) < config_.height_settle_rate_rad_s;
      // Each leg is servoed independently (fixed-impedance PD, see
      // native_runner.cpp applyMotorCommand) against this ONE shared
      // target, so nothing here can see or correct a left/right split -
      // only the two-leg MEAN (in.knee_rad/in.hip_rad, Rule 1) comes back.
      // An open-loop time ramp can still outrun that mean: whichever leg
      // happens to move first also happens to pick up more of the body's
      // support, which lowers its own remaining torque demand and raises
      // the other's, so the other's fixed-Kp tracking error grows instead
      // of shrinking - a positive feedback that widens with every further
      // step of an unconditional ramp. Clamping the commanded target to
      // within max_lead_rad of the MEASURED mean makes the ramp pace
      // itself on actual two-leg progress instead of the clock, so it
      // cannot run away from a lagging leg pair.
      if (settled && knee_hold_rad_ != knee_target_rad_) {
        const float step = config_.knee_rate_rad_s * in.dt_s;
        float next = knee_hold_rad_ < knee_target_rad_
            ? std::min(knee_hold_rad_ + step, knee_target_rad_)
            : std::max(knee_hold_rad_ - step, knee_target_rad_);
        next = std::clamp(next, in.knee_rad - config_.height_max_lead_rad,
                           in.knee_rad + config_.height_max_lead_rad);
        knee_hold_rad_ = next;
      }
      // The hip re-target always tracks the CURRENT knee (even while
      // paused) so the COM stays over the axle at whatever knee angle is
      // actually commanded right now.
      {
        const float hip_target = levelHipForKnee(knee_hold_rad_);
        const float hip_step = config_.hip_retarget_rate_rad_s * in.dt_s;
        float next = hip_cmd_rad_ < hip_target
            ? std::min(hip_cmd_rad_ + hip_step, hip_target)
            : std::max(hip_cmd_rad_ - hip_step, hip_target);
        next = std::clamp(next, in.hip_rad - config_.height_max_lead_rad,
                           in.hip_rad + config_.height_max_lead_rad);
        hip_cmd_rad_ = next;
      }
      break;
    }
    case Phase::Aborted:
      break;
  }
  // Second pass: present the wheel command for whichever phase this tick
  // ends in, so a transition above takes effect immediately.
  float wheel = 0.0F;
  switch (phase_) {
    case Phase::Lowering:
      wheel = config_.wheel_roll_rad_s;
      break;
    case Phase::Wheelie:
      wheel = config_.wheelie_wheel_speed_rad_s;
      out.wheel_torque_ceiling_nm = config_.wheelie_wheel_torque_nm;
      break;
    case Phase::Catch:
    case Phase::Level:
      out.wheel_torque_ceiling_nm = config_.wheelie_wheel_torque_nm;
      wheel = balanceWheelCommand(in, lean.angle_rad);
      break;
    case Phase::Balanced: {
      out.wheel_torque_ceiling_nm = config_.wheelie_wheel_torque_nm;
      wheel = balanceWheelCommand(in, lean.angle_rad);
      const float yaw_m_s = config_.half_wheel_base_m *
          std::clamp(in.target_yaw_rad_s, -config_.max_yaw_rate_rad_s,
                     config_.max_yaw_rate_rad_s);
      out.wheel_yaw_rad_s = yaw_m_s / robot_control::geometry::wheel_radius_m;
      break;
    }
    case Phase::Flinging:
    case Phase::Pivot:
    case Phase::Aborted:
      break;
  }
  out.phase = phase_;
  out.hip_target_rad = hip_cmd_rad_;
  out.knee_target_rad = knee_hold_rad_;
  out.wheel_rad_s = wheel;
  return out;
}

float RockerLower::balanceWheelCommand(const Input& in, float lean_rad) noexcept {
  const float radius = robot_control::geometry::wheel_radius_m;
  const float shin_rate = in.pitch_rate_rad_s + in.hip_rate_rad_s + in.knee_rate_rad_s;
  // Drift from where Catch began, and ground speed, both from wheel-encoder +
  // IMU dead reckoning (never the simulator's free-joint pose) - without this
  // outer loop a small uncorrected gyro rate bias integrates into a steadily
  // growing lean until the robot tips (the same failure V3Controller's own
  // "held" path guards against).
  const float drift_m = groundPositionM(in) - ground_x0_m_;
  const float ground_v_m_s = radius * (in.wheel_rate_rad_s + shin_rate);
  // Balanced only: track a commanded forward speed by moving the position
  // target with it (same pattern as V3Controller's pid_x_target_); on
  // release, freeze the target where driving stopped rather than snapping
  // back to Catch's starting point. Every earlier phase drives at 0.
  const float target_v = phase_ == Phase::Balanced ? in.target_forward_m_s : 0.0F;
  if (std::abs(target_v) > 1.0e-3F) {
    drive_target_m_ += target_v * in.dt_s;
    driving_ = true;
  } else if (driving_) {
    drive_target_m_ = drift_m;
    driving_ = false;
  }
  const float lean_ref_rad = -std::clamp(
      config_.balance_outer_kx * (drift_m - drive_target_m_) +
          config_.balance_outer_kv * (ground_v_m_s - target_v),
      -config_.balance_outer_max_rad, config_.balance_outer_max_rad);
  // The gyro rate is used for damping, not a finite difference of the lean
  // estimate: differencing a quantised, noisy angle at the control rate would
  // amplify sensor noise into a large spurious derivative (see V3Controller's
  // own inner loop, which damps on the raw gyro rate the same way).
  const float accel = std::clamp(
      config_.balance_kp * (lean_rad - lean_ref_rad) + config_.balance_kd * in.pitch_rate_rad_s,
      -config_.balance_accel_max, config_.balance_accel_max);
  speed_cmd_m_s_ = std::clamp(speed_cmd_m_s_ + accel * in.dt_s,
                              -config_.balance_speed_max_m_s, config_.balance_speed_max_m_s);
  return (speed_cmd_m_s_ + config_.balance_lead_s * accel) / radius - shin_rate;
}

const char* toString(RockerLower::Phase phase) noexcept {
  switch (phase) {
    case RockerLower::Phase::Lowering: return "lowering";
    case RockerLower::Phase::Flinging: return "fling";
    case RockerLower::Phase::Pivot: return "pivot";
    case RockerLower::Phase::Wheelie: return "wheelie";
    case RockerLower::Phase::Catch: return "catch";
    case RockerLower::Phase::Level: return "level";
    case RockerLower::Phase::Balanced: return "balanced";
    case RockerLower::Phase::Aborted: return "aborted";
  }
  return "?";
}

}  // namespace tap_fw::v3
