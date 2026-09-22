#pragma once

#include "robot_control/Types.hpp"

namespace robot_control {

// Per-side touchdown gain scheduling for the knee only.  After a debounced
// sensor-derived touchdown, Kp is reduced and Kd is increased during Impact,
// then both scales return to one through Rebound.  It changes gains rather
// than position targets, so it dissipates the measured compression motion
// without injecting a new leg trajectory.  Each side has an independent
// phase machine and compression-velocity gate; default scales of 1.0 make
// the component inert.
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
    // Optional compliance while airborne, before touchdown is confirmed. This
    // is separate from Impact: a ledge drive-off can create edge/lip impulses
    // immediately after contact loss, while the old logic still held full
    // stance stiffness until a debounced touchdown edge. Defaults preserve the
    // previous behavior exactly.
    float airborne_hip_kp_scale = 1.0F;
    float airborne_hip_kd_scale = 1.0F;
    float airborne_knee_kp_scale = 1.0F;
    float airborne_knee_kd_scale = 1.0F;
    // Energy-based sizing (2026-09-16): instead of a single fixed
    // impact_kp_scale/impact_kd_scale tuned for one "average" impact, size
    // the virtual spring-damper from the REAL measured knee joint velocity
    // at the instant of touchdown and the REAL available knee stroke left
    // before the hard/soft limit - a harder landing (higher measured
    // velocity) gets a stiffer, better-damped response; a gentle one gets a
    // soft one; either way the leg is sized to arrest the impact within the
    // stroke that's actually left, not an assumed worst case. Energy
    // balance: 0.5*I*v^2 (kinetic energy at touchdown) == 0.5*K*x^2 (virtual
    // spring energy at full compression), solved for K, with a safety
    // margin so the target stopping point is short of the real limit, not
    // exactly at it. Kd is then critical damping for that K (matches how
    // BalanceControllerNode's own leg_kd_ default is derived - see its
    // constructor). Requires leg_knee_inertia_kgm2, leg_kp_nominal_nm_rad,
    // leg_kd_nominal_nm_s_rad, knee_hard_limit_rad all to be nonzero/finite;
    // if any is missing, falls back to the plain impact_kp_scale/
    // impact_kd_scale constants above unchanged. Default off (0 inertia) -
    // exact no-op unless explicitly configured.
    float leg_knee_inertia_kgm2 = 0.0F;
    float leg_kp_nominal_nm_rad = 0.0F;
    float leg_kd_nominal_nm_s_rad = 0.0F;
    // The real knee joint limit impact absorption must stop short of -
    // pass the true hard actuator limit from the native model, not a
    // softer operating bound, since this sizing already includes its own
    // safety margin via energy_stroke_margin.
    float knee_hard_limit_rad = 0.0F;
    // Fraction of the true remaining stroke (to knee_hard_limit_rad from
    // the CURRENT commanded knee target) the energy sizing targets as the
    // stopping point, so a real impact lands short of the hard limit
    // instead of exactly at it. 0.7 means: size K so the spring would fully
    // absorb the measured impact energy within 70% of the real remaining
    // travel.
    float energy_stroke_margin = 0.7F;
    // Absolute bounds on the resulting scale factors regardless of the
    // energy calculation, so a sensor glitch or an extreme velocity can't
    // command an absurd gain.
    float energy_kp_scale_min = 0.02F;
    float energy_kp_scale_max = 1.0F;
    float energy_kd_scale_max = 6.0F;
    // Real-impact gate: |leg height velocity| at that side's debounced
    // touchdown instant must exceed this for Impact to engage at all on
    // that side. High enough by default that no config is needed to
    // disable it outright.
    float touchdown_velocity_threshold_m_s = 1.0e6F;
    // Alternate/supplementary gate (2026-09-16): the raw accelerometer
    // specific-force magnitude spikes IMMEDIATELY on real contact (a
    // physical fact already measured directly - see
    // docs/LEDGE_DRIVEOFF_LIMIT.md's "IMU accelerometer touchdown
    // detection" section - spikes into the hundreds of m/s^2 for 1-2
    // samples), whereas the leg's own joint velocity (h_dot above) has not
    // yet responded at the SAME tick contact first reads true - physics
    // needs a tick or two to actually accelerate the joint. Root-caused
    // 2026-09-16 on a real 50cm drop: h_dot measured at the exact touchdown
    // tick was still ~0 (leg hadn't started compressing yet), so the plain
    // h_dot gate silently rejected a genuinely hard impact as "not dynamic
    // enough" and the whole mechanism never engaged for that event. OR'd
    // with the h_dot gate so either signal can trigger. High enough by
    // default that no config is needed to disable it outright.
    float hard_impact_accel_mps2 = 1.0e6F;
    // How long after Impact entry to keep tracking the PEAK |knee joint
    // velocity| before finalizing the energy-based (Kp,Kd) sizing, instead
    // of using the single entry-tick value (which the same real-impact
    // trace showed is unreliable - the joint hasn't caught up to the real
    // impact yet at that instant). 0 preserves the original single-tick
    // capture (exact prior behavior). While capturing, the softest
    // configured scale (energy_kp_scale_min-derived) applies as a safe
    // interim default, on the reasoning that under-stiff is always safer
    // than over-stiff during a real, still-unfolding impact.
    float energy_capture_window_s = 0.0F;
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
    float left_hip_kp_scale = 1.0F;
    float left_hip_kd_scale = 1.0F;
    float left_leg_kp_scale = 1.0F;
    float left_leg_kd_scale = 1.0F;
    float right_hip_kp_scale = 1.0F;
    float right_hip_kd_scale = 1.0F;
    float right_leg_kp_scale = 1.0F;
    float right_leg_kd_scale = 1.0F;
  };

  LandingImpedance();
  explicit LandingImpedance(Config config) : config_(config) {}

  void reset();
  // Seed each side independently from the first validated contact snapshot.
  void reset(bool left_floor_contact, bool right_floor_contact);

  [[nodiscard]] Output update(bool left_floor_contact, bool right_floor_contact,
                              float left_hip, float left_knee, float right_hip,
                              float right_knee, float left_hip_vel,
                              float left_knee_vel, float right_hip_vel,
                              float right_knee_vel, float accel_mag_mps2,
                              float dt_s, bool hard_impact_event = false);

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
    // Captured once this side's energy-sizing capture window closes (or
    // immediately at Impact entry if energy_capture_window_s == 0); held
    // fixed through the rest of Impact/Rebound so the target doesn't chase
    // noise mid-event.
    float impact_kp_scale = 1.0F;
    float impact_kd_scale = 1.0F;
    // While energy_capture_window_s > 0: true from Impact entry until the
    // window closes, tracking the peak |knee_vel_rad_s| actually seen
    // (a single entry-tick reading is unreliable - see
    // Config::energy_capture_window_s's own comment).
    bool capturing_peak = false;
    float peak_knee_vel_rad_s = 0.0F;
    float knee_rad_at_impact = 0.0F;
  };

  // One side's phase machine + its own (Kp,Kd) output, sharing config_.
  // knee_rad/knee_vel_rad_s are that side's own knee joint state, needed
  // (in addition to h_dot, the Cartesian height rate already used for the
  // touchdown trigger) for energy-based sizing, which works in joint space
  // to match what leg_kp/leg_kd actually act on. accel_mag_mps2 is the raw
  // accelerometer specific-force magnitude, a supplementary impact gate -
  // see Config::hard_impact_accel_mps2.
  void updateSide(Side& side, bool floor_contact, float h_dot, float knee_rad,
                  float knee_vel_rad_s, float accel_mag_mps2, float dt_s,
                  bool hard_impact_event,
                  float& hip_kp_scale, float& hip_kd_scale,
                  float& knee_kp_scale, float& knee_kd_scale) const;
  void sizeFromEnergy(Side& side, float knee_vel_rad_s) const;
  [[nodiscard]] bool energySizingConfigured() const noexcept;

  Config config_;
  Side left_side_{};
  Side right_side_{};
};

}  // namespace robot_control
