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
// PER-SIDE since 2026-09-16 (ROLL axis): a real touchdown is never
// simultaneous on both wheels (asymmetric landing: uneven ground, or one
// side reaching the edge first) - the earlier combined-both-sides version
// used one averaged compression velocity and one shared Kp/Kd scale for
// all four leg joints, so it could not let the side that's already down
// absorb energy while the side still reaching for the ground kept its
// normal tracking stiffness. This runs two fully independent phase state
// machines (one per side) against each side's OWN measured leg height/
// velocity and OWN floor_contact reading, still knee-only per the hip
// finding above. All gains default to 1.0 (no scaling) so an unconfigured
// controller is byte-identical to no impedance component at all, on
// either side independently.
//
// Trigger: each side's own touchdown edge (that side's floor_contact
// false->true) after a real, debounced flight, additionally gated on that
// side's own leg compression velocity magnitude at that instant exceeding
// touchdown_velocity_threshold_m_s.
class LandingImpedance {
 public:
  struct Config {
    // Kp/Kd multipliers applied to ONE side's knee while that side is in
    // Impact, ramping through Rebound back to 1.0. impact_kp_scale should
    // be well below 1 (soft, e.g. 0.1-0.3); impact_kd_scale should be well
    // above 1, aiming at the leg's critical damping ratio for its
    // effective inertia (Kd ~= 2*sqrt(Kp*M_effective) per the classic
    // mass-spring-damper relation - see BalanceControllerNode's leg_kd_
    // default for where that inertia is now actually derived).
    float impact_kp_scale = 1.0F;
    float impact_kd_scale = 1.0F;
    // Real-impact gate: |leg height velocity| at that side's debounced
    // touchdown instant must exceed this for Impact to engage at all on
    // that side. High enough by default that no config is needed to
    // disable it outright.
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
    // time has elapsed continuously airborne (that side) since the last
    // confirmed liftoff (that side) - matches
    // LandingReflex::Config::min_airborne_before_touchdown_s.
    float min_airborne_before_touchdown_s = 0.15F;
  };

  struct Output {
    float left_leg_kp_scale = 1.0F;
    float left_leg_kd_scale = 1.0F;
    float right_leg_kp_scale = 1.0F;
    float right_leg_kd_scale = 1.0F;
  };

  LandingImpedance();
  explicit LandingImpedance(Config config) : config_(config) {}

  void reset();

  [[nodiscard]] Output update(bool left_floor_contact, bool right_floor_contact,
                              float left_hip, float left_knee, float right_hip,
                              float right_knee, float left_hip_vel,
                              float left_knee_vel, float right_hip_vel,
                              float right_knee_vel, float dt_s);

  [[nodiscard]] bool active() const noexcept {
    return left_side_.phase != Phase::Grounded ||
           right_side_.phase != Phase::Grounded;
  }

 private:
  enum class Phase { Grounded, Airborne, Impact, Rebound };

  struct Side {
    Phase phase = Phase::Grounded;
    bool prev_floor_contact = true;
    float airborne_elapsed_s = 0.0F;
    float phase_elapsed_s = 0.0F;
  };

  // One side's phase machine + its own (Kp,Kd) output, sharing config_.
  void updateSide(Side& side, bool floor_contact, float h_dot, float dt_s,
                  float& kp_scale, float& kd_scale) const;

  Config config_;
  Side left_side_{};
  Side right_side_{};
};

}  // namespace robot_control
