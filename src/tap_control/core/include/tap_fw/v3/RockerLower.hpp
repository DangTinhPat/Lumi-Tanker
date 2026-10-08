#pragma once

namespace tap_fw::v3 {

// Rocking stand-up from the prone homing pose (knees hold the homing stop
// throughout; both legs share one command, both wheels share one command).
// Every decision uses only the MPU6500 pitch/rate and the leg encoders; the
// wheel-to-COM lean comes from the CAD FK (WholeBodyModel). Simulator
// contacts, pose or velocity never enter this class.
//
//  Lowering  hips fold down slowly while both wheels roll forward, until the
//            sensor-derived N-P offset stops shrinking (belly loaded)
//  Flinging  hips fold fast to the lock (fling's momentum briefly lifts the
//            belly; see RockerLower.cpp for the offline MJCF measurements)
//  Pivot     short hold, wheels held
//  Wheelie   both wheels push forward hard while the hips turn the body up;
//            the wheel's forward reaction torque is what lifts the belly
//            clear of the floor - the hip twist alone cannot (see .cpp)
//  Catch     a V3.1-style wheel balancer (lean -> wheel accel, no ground-
//            truth outer loop) takes over once the COM lean is small
//  Level     the balancer keeps running while the hips turn the body back
//            down to level (fast while far from level, slow near it)
//  Balanced  terminal: the balancer keeps holding the robot level and
//            standing on its wheels, now tracking Input::target_forward_m_s/
//            target_yaw_rad_s and Input::height_change_requested (all
//            ignored in every earlier phase, so a drive/height command sent
//            mid-stand-up cannot disturb Catch/Level's settling); only
//            sensor loss or a fall aborts it
class RockerLower {
 public:
  enum class Phase { Lowering, Flinging, Pivot, Wheelie, Catch, Level, Balanced, Aborted };

  struct Config {
    float hip_rate_rad_s = 0.5F;
    float hip_floor_rad = 1.5F;  // slow lowering stops here at the latest
    float fling_rate_rad_s = 10.0F;
    float fling_lock_rad = 1.2F;
    float wheel_roll_rad_s = 2.0F;  // + rolls the axle toward P
    // N-P rise over its running minimum that marks the minimum as passed.
    float rise_margin_m = 0.0015F;
    // pitch + hip below this puts thigh/shin within 10 mm of the floor next
    // to the wheel (knee at the homing stop). Only checked below the lock.
    float leg_angle_min_rad = 0.70F;
    // The fling's fast hip command still has 0.05-0.07 rad of servo lag when
    // it reaches the lock; too short a hold here launches Wheelie while the
    // hip is still catching up to Flinging's target (the two fight each
    // other for an instant), which roughly doubled the forward lurch
    // measured through Catch/Level (0.55 m at 0.05 s vs ~0.30 m here).
    float pivot_hold_s = 0.20F;

    float wheelie_hip_target_rad = 2.69F;
    float wheelie_hip_rate_rad_s = 2.5F;
    // SIM ceiling only, well above the shipped BL4820M12 spec (0.13 Nm per
    // wheel, docs/MOTOR_SELECTION.md): a wheelie's reaction torque needs
    // roughly 6x that to counter the hip's own reaction and pull the axle
    // under the COM (see .cpp). This is a simulation actuator assumption
    // pending a wheel/gearbox decision, not a claim about the currently
    // specified hardware; the command channel itself (a forward speed
    // target) is the same Moving-Speed interface the real BL4820M12 uses.
    float wheelie_wheel_torque_nm = 0.8F;
    float wheelie_wheel_speed_rad_s = 30.0F;
    float wheelie_min_time_s = 0.05F;  // ignore the lean test before this
    float wheelie_timeout_s = 2.0F;
    float catch_lean_max_rad = 0.12F;
    float catch_settle_s = 0.05F;
    // Level: fold the hip down to the one angle (knee at the homing stop)
    // where the COM-line FK angle is zero, so pitch 0 and lean 0 coincide -
    // the balancer is what actually holds pitch near 0 there, not this rate
    // law, which only shapes how fast the hip gets there (proportional to
    // its remaining distance, clamped to this range).
    float level_target_hip_rad = 1.1248F;
    float level_gain = 6.0F;
    float level_min_rad_s = 0.3F;
    float level_max_rad_s = 5.0F;
    float fall_lean_rad = 0.8F;  // Catch/Level/Balanced abort past this lean

    // Balanced-only height change: both knees ramp to a requested angle
    // (native frame, matches tap_fw::v3::LegHeight's own [-native_knee_min,
    // 0] convention so a caller can reuse LegHeight::forHeight's knee_offset_
    // rad directly) while the hip target is continuously re-solved so the
    // COM stays over the axle at the CURRENT knee (see .cpp
    // levelHipForKnee) - level_target_hip_rad above is just that solve's
    // knee=-1.2 special case, cached as a compile-time constant.
    float knee_rate_rad_s = 0.3F;
    float hip_retarget_rate_rad_s = 1.0F;
    // Height ramp only steps while the balance is actually settled (same
    // gate V3Controller::updateHeight uses): without it, a height change
    // pushes its shared target forward on a fixed clock regardless of how
    // the two independently-servoed legs are actually tracking it, and any
    // small asymmetry (contact/load) compounds - one leg gets ahead, takes
    // more of the body's support, and the other's fixed-impedance PD stalls
    // under the growing load it cannot track. Pausing on pitch disturbance
    // lets the balancer null the lean before the ramp advances again.
    float height_settle_pitch_rad = 0.05F;
    float height_settle_rate_rad_s = 0.5F;
    // Max rad the shared knee/hip target may lead the MEASURED two-leg
    // mean encoder (see the Balanced case in .cpp): bounds how far a
    // lagging leg pair can fall behind before the ramp itself pauses,
    // which is what actually prevents one leg racing ahead of the other.
    float height_max_lead_rad = 0.15F;

    // V3.1-style angle balancer (Catch/Level/Balanced): lean and its rate to
    // a ground acceleration, integrated to a speed command. The outer loop
    // (position/speed hold) uses wheel-encoder + IMU dead reckoning relative
    // to where Catch began, exactly like V3Controller's own ground_x_/
    // ground_v_ - never the simulator's free-joint pose.
    float balance_kp = 30.0F;
    float balance_kd = 4.0F;
    float balance_accel_max = 15.0F;
    float balance_speed_max_m_s = 1.0F;
    float balance_lead_s = 0.12F;
    // kv well above kx for damping (a position loop this underdamped let the
    // robot wander >0.7 m over ~10 s while still reading as "balanced" on
    // lean/pitch alone - lean stability does not imply position stability).
    float balance_outer_kx = 0.05F;   // rad of lean ref per m of drift
    float balance_outer_kv = 0.50F;   // rad of lean ref per m/s of ground speed
    float balance_outer_max_rad = 0.2F;
    // Balanced-phase drive (both wheels get the common speed; this splits it
    // per side), same convention as LqrBalancer::Config.
    float half_wheel_base_m = 0.1545F;
    float max_yaw_rate_rad_s = 0.8F;
  };

  struct Input {
    float dt_s = 0.0F;
    float pitch_rad = 0.0F;       // MPU6500 estimate
    float pitch_rate_rad_s = 0.0F;
    float hip_rad = 0.0F;         // mean of both hip encoders, native frame
    float knee_rad = 0.0F;        // mean of both knee encoders, native frame
    float hip_rate_rad_s = 0.0F;
    float knee_rate_rad_s = 0.0F;
    float wheel_pos_rad = 0.0F;    // mean of both wheel encoders, native frame
    float wheel_rate_rad_s = 0.0F;
    // Only read once Balanced (see the Phase::Balanced doc above).
    float target_forward_m_s = 0.0F;
    float target_yaw_rad_s = 0.0F;
    // Edge-triggered like STAND_UP itself: the caller sets this true for one
    // tick with the desired knee angle (native frame); RockerLower owns the
    // ramp from there and the caller need not track completion.
    bool height_change_requested = false;
    float requested_knee_rad = 0.0F;
    bool sensors_ok = false;
  };

  struct Output {
    Phase phase = Phase::Lowering;
    float hip_target_rad = 0.0F;
    float knee_target_rad = 0.0F;
    float wheel_rad_s = 0.0F;   // joint-space command, common to both wheels
    float wheel_yaw_rad_s = 0.0F;  // differential: left -= this, right += this
    float wheel_torque_ceiling_nm = 0.13F;  // driver torque limit to apply
    float n_to_p_m = 0.0F;      // + : P ahead of N along +X
  };

  RockerLower() = default;
  explicit RockerLower(Config config) : config_(config) {}

  void reset(float hip_rad, float knee_rad) noexcept;
  [[nodiscard]] Output update(const Input& in) noexcept;

  // Horizontal COM-minus-axle offset from joint encoders and IMU pitch.
  [[nodiscard]] static float nToP(float hip_rad, float knee_rad,
                                  float pitch_rad) noexcept;

 private:
  [[nodiscard]] float balanceWheelCommand(const Input& in, float lean_rad) noexcept;

  Config config_{};
  Phase phase_ = Phase::Lowering;
  float hip_cmd_rad_ = 0.0F;
  float knee_hold_rad_ = 0.0F;
  float best_n_to_p_m_ = 0.0F;   // running minimum while lowering
  float phase_elapsed_s_ = 0.0F;
  float speed_cmd_m_s_ = 0.0F;
  float ground_x0_m_ = 0.0F;  // dead-reckoned ground position where Catch began
  float drive_target_m_ = 0.0F;  // moving position target while driving
  bool driving_ = false;
  float knee_target_rad_ = 0.0F;  // Balanced-only requested knee (see reset())
};

[[nodiscard]] const char* toString(RockerLower::Phase phase) noexcept;

}  // namespace tap_fw::v3
