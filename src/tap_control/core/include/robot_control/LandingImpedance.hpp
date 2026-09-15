#pragma once

#include "robot_control/Types.hpp"

namespace robot_control {

// Virtual leg-length spring-damper via GAIN SCHEDULING, not an active force
// or position target. Implements the classic 4-phase touchdown FSM (Flight
// -> Impact -> Rebound -> Ground): on real touchdown, position stiffness
// (Kp) is lowered and damping (Kd) is raised toward a near-critical value,
// so the leg yields freely under whatever real impact force is already
// acting on it (gravity + ground reaction transmitted through the
// kinematic chain) instead of being pulled toward a computed target - the
// leg absorbs kinetic energy through the damper (Kd term), the same way a
// real shock absorber works, rather than an active spring pushing the
// joint somewhere. Once the leg's own (measured, not commanded)
// compression velocity shows it has bottomed out, Kp ramps back up
// smoothly to restore normal stance-tracking authority.
//
// This is a deliberate, evidence-driven departure from three earlier
// designs in this file's history that all ACTIVELY drove the hip (a
// virtual spring force, then a coordinated position/IK target, then this
// same gain-schedule applied to the hip too) and all measured worse than
// doing nothing on the 20 cm ledge drive-off test: touching the hip in ANY
// way - force, position, or even just its stiffness/damping scale - during
// an already-violent touchdown event consistently hurt, regardless of the
// mechanism. Only excluding the hip entirely (this revision) beat the old
// stock baseline (70%) at the time. So although a passive/dissipative gain
// schedule is generally safer than an active force by construction (it can
// only ever dissipate energy that's already there, never inject a new
// target), this robot's specific ledge-drop failure mode (a chaotic
// wheel-edge-catch flip, not a graceful vertical absorption problem) still
// does not tolerate ANY hip involvement in practice - see
// EmbeddedController.cpp's writeBalancedCommand for where this scale is
// applied to the knee only.
//
// RE-MEASURED 2026-09-15 against the current, widened baseline
// (`mpc.fall_pitch_rad=1.1`/`mpc.max_ground_accel=10.0`, promoted into
// balance_controller.yaml's own defaults - see docs/LEDGE_DRIVEOFF_LIMIT.md):
// 88% baseline vs. 85% (n=41) with this mechanism layered on top -
// statistically tied, not a reproduced win. NOT enabled in production
// defaults. Do not cite the "beat stock" framing above as still current;
// it described the old, narrower 70%-baseline condition, not this one.
// This also broadly matches the real touchdown mechanism on Ascento (a
// virtual spring-damper simulated in the hip motors of ITS single-DOF,
// mechanically-decoupled leg) and the general legged-robot impedance-
// control literature, adapted to this robot's specific measured
// constraint.
//
// Trigger: the touchdown edge (floor_contact false->true) after a real,
// debounced flight (same idiom as LandingReflex/earlier LandingImpedance
// revisions). Additionally gated on the leg's own compression velocity
// magnitude at that instant exceeding touchdown_velocity_threshold_m_s - a
// gentle, low-speed contact (e.g. settling from a small pose change) does
// not need or get any intervention, only a genuine dynamic impact does.
// Impact -> Rebound triggers when the leg's real compression velocity
// (from live joint velocities via WholeBodyModel::legHeight's Jacobian, NOT
// a commanded motion) stops increasing - a physical bottom-of-stroke
// signal - or after max_impact_duration_s as a stuck-sensor safety net.
//
// All gains default to 1.0 (no scaling) and the velocity threshold is high
// enough that no real contact ever triggers Impact by default, so an
// unconfigured controller is byte-identical to no impedance component at
// all (mirrors LandingReflex's own NoOpAtDefaults guarantee) - see the
// explicit early-out in update() for the exact gate.
class LandingImpedance {
 public:
  struct Config {
    // Kp/Kd multipliers applied to ALL FOUR leg joints while in Impact,
    // ramping through Rebound back to 1.0. impact_kp_scale should be well
    // below 1 (soft, e.g. 0.1-0.3); impact_kd_scale should be well above 1,
    // aiming at the leg's critical damping ratio for its effective inertia
    // (Kd ~= 2*sqrt(Kp*M_effective) per the classic mass-spring-damper
    // relation - M_effective is not derived here, tune empirically).
    float impact_kp_scale = 1.0F;
    float impact_kd_scale = 1.0F;
    // Real-impact gate: |leg height velocity| at the debounced touchdown
    // instant must exceed this for Impact to engage at all. High enough by
    // default that no config is needed to disable it outright.
    float touchdown_velocity_threshold_m_s = 1.0e6F;
    // Ramp Kp/Kd back to 1.0 over this long once bottomed out.
    float rebound_blend_s = 0.15F;
    // Minimum time in Impact before the velocity-bottomed-out trigger is
    // honored (discretization-noise guard), and a hard cap on how long
    // Impact can run before forcing Rebound regardless (stuck-sensor
    // safety net).
    float min_impact_before_rebound_s = 0.01F;
    float max_impact_duration_s = 0.4F;
    // Debounce: a touchdown edge is only honored once at least this much
    // time has elapsed continuously airborne since the last confirmed
    // liftoff - matches LandingReflex::Config::min_airborne_before_touchdown_s.
    float min_airborne_before_touchdown_s = 0.15F;
  };

  struct Output {
    float leg_kp_scale = 1.0F;
    float leg_kd_scale = 1.0F;
  };

  LandingImpedance();
  explicit LandingImpedance(Config config) : config_(config) {}

  void reset();

  [[nodiscard]] Output update(bool floor_contact, float left_hip,
                              float left_knee, float right_hip,
                              float right_knee, float left_hip_vel,
                              float left_knee_vel, float right_hip_vel,
                              float right_knee_vel, float dt_s);

  [[nodiscard]] bool active() const noexcept {
    return phase_ != Phase::Grounded;
  }

 private:
  enum class Phase { Grounded, Airborne, Impact, Rebound };

  Config config_;
  Phase phase_ = Phase::Grounded;
  bool prev_floor_contact_ = true;
  float airborne_elapsed_s_ = 0.0F;
  float phase_elapsed_s_ = 0.0F;
};

}  // namespace robot_control
