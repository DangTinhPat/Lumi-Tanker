#pragma once

#include "robot_control/Types.hpp"

namespace robot_control {

// Reactive leg-TARGET offset for the airborne/touchdown phase of a fall or
// drive-off ledge event.
//
// Three prior fixes for the 20 cm/1.0 m/s ledge drive-off (see
// docs/LEDGE_DRIVEOFF_LIMIT.md) all only changed stiffness/damping around a
// fixed, open-loop leg trajectory (EmbeddedController otherwise feeds
// command.target_joint_position_rad straight through with zero dependency on
// IMU/contact state - see EmbeddedController::update). This component instead
// changes WHAT position the legs are commanded to while airborne/landing.
//
// TAP's knee range peaks at knee = 0 rad = full leg extension already at
// neutral standing pose (see WholeBodyModel/GeneratedGeometry: the two leg
// links are colinear, i.e. at maximum reach, when knee = 0), so a "reach
// further down to shorten the fall" motion does not exist - confirmed
// geometrically, not just asserted. What IS available is the opposite: bend
// (crouch) the knees BEFORE touchdown so that on impact the legs have a real
// compression stroke to extend back through, under actual commanded motion,
// instead of already sitting at the knee_max_rad limit with zero give. This
// mirrors human/quadruped pre-landing crouch and the touchdown-posture idea in
// "Real-time Optimal Landing Control of the MIT Mini Cheetah" (arXiv
// 2110.02799) adapted to this robot's one real remaining degree of freedom.
//
// The airborne signal reused here (`floor_contact` false) is the same
// sensor-derived support hypothesis produced by FlightObserver from MPU6500
// specific force and encoder evidence; no simulator pose/contact API is used.
//
// All gains default to 0 (crouch 0, hip gain 0, roll gain 0): `update()` then
// always returns an all-zero offset for any floor_contact/pitch_rate/dt
// sequence, so an unconfigured controller is byte-identical to no reflex at
// all (see LandingReflexTest.NoOpAtDefaults / EmbeddedController tests).
class LandingReflex {
 public:
  struct Config {
    // Magnitude of the symmetric pre-impact knee crouch, radians (sign is
    // normalized internally - always subtracts, i.e. bends further). 0 = inert.
    float airborne_knee_crouch_rad = 0.0F;
    // Optional one-shot hip counter-lean, latched from the pitch rate sensed
    // at the instant contact is lost (rad of hip offset per rad/s of pitch
    // rate at liftoff), a wheel-legged CoM-shift assist for the wheel-balance
    // MPC (cf. posture-based CoM shift for wheel-legged balance). 0 = inert;
    // left available for future tuning once its sign is validated against a
    // real drive-off trial (see docs/LEDGE_DRIVEOFF_LIMIT.md discussion).
    float hip_offset_per_pitch_rate = 0.0F;
    float max_hip_offset_rad = 0.3F;
    // Active velocity-matched extension (2026-09-16): real legged-robot
    // landing controllers (e.g. "Real-time Optimal Landing Control of the
    // MIT Mini Cheetah", arXiv 2110.02799 - "desired foot trajectory is set
    // so the ground velocity at landing becomes zero or substantially
    // zero") actively EXTEND the leg during the fall to reduce the actual
    // closing velocity at contact, rather than only absorbing the full
    // impact passively afterward (that is LandingImpedance's job, a
    // separate, complementary component). Requires real knee extension
    // room: the OLD near-straight default stance (knee~0, already at full
    // reach) geometrically ruled this out entirely - see this file's own
    // header history. The 2026-09-16 deep-crouch default (knee~=-1.4 rad)
    // reopened ~80 deg of real extension travel, making this newly
    // possible. Physics: during confirmed free fall (no ground reaction),
    // closing velocity grows as g*t from the moment of liftoff (standard
    // free-fall kinematics - no height/distance sensor needed, just
    // elapsed airborne time, which this class already tracks). Commanding
    // knee extension at a matching rate (via the real leg-height Jacobian,
    // WholeBodyModel::legHeight) closes some of that velocity gap before
    // contact. 0 (default) is an exact no-op.
    float active_extension_gravity_m_s2 = 0.0F;
    // Hard cap on how far this maneuver may extend the knee beyond the
    // resting target, regardless of the physics estimate - a real safety
    // margin against overshoot/sensor error, and against extending past
    // what full-reach (knee=0) geometrically allows.
    float active_extension_max_rad = 0.0F;
    // Separate hip cap for active extension. The 2-DOF leg IK can otherwise
    // solve a clean task-space extension by moving the hip too far, injecting
    // a large sagittal body disturbance during the fall. <0 preserves the old
    // shared cap; 0 makes active extension knee-dominant.
    float active_extension_max_hip_rad = -1.0F;
    // Wait this long after confirmed contact loss before active extension.
    // A drive-off ledge is not a vertical drop: extending while the wheels are
    // still near the edge can snag the lip and create a pitch impulse.
    float active_extension_delay_s = 0.0F;
    // Optional IMU specific-force pre-arm. Contact loss can arrive late or
    // flicker on the ledge; a real IMU reads low specific force during
    // unloading/free fall before a reliable support-hypothesis edge. 0 disables.
    float unloading_accel_mps2 = 0.0F;
    // Limit how fast the reflex target may move once active. 0 disables the
    // slew limiter and preserves the historical instantaneous offset.
    float target_slew_rate_rad_s = 0.0F;
    // Hold the airborne offset through touchdown for this long, then blend
    // linearly back to zero over catch_blend_s - an actual commanded motion
    // through the impact window, not an instantaneous snap.
    float catch_hold_s = 0.2F;
    float catch_blend_s = 0.2F;
    // Debounce: a touchdown_edge is only honored once at least this much time
    // has elapsed since the liftoff_edge that opened the current Airborne
    // phase. Measured directly against a real 20 cm/1.0 m/s ledge drive-off
    // (see docs/LEDGE_DRIVEOFF_LIMIT.md): a transient support hypothesis can
    // read a single spurious
    // "grounded" tick within ~10-50 ms of actually leaving the pad, well
    // before the robot has fallen any meaningful distance -- without this
    // guard that single tick fires touchdown_edge immediately, collapsing the
    // whole airborne crouch back to neutral long before the real impact. 0 =
    // no debounce (matches the original, vulnerable behavior).
    float min_airborne_before_touchdown_s = 0.15F;
    // Differential (ROLL-axis) HIP offset, 2026-09-16: every mechanism in
    // this file up to this point is left/right SYMMETRIC - it never reasoned
    // about roll at all. Real telemetry from a 50 cm ledge drop showed
    // roll_rate hitting ~2.8 rad/s at touchdown and only partially decaying
    // (~0.9 rad/s) 80 ms later, with ZERO active correction anywhere in the
    // control stack (grep-verified: state.imu.roll_rate_rad_s was measured
    // but never read by any controller before this). The two legs sit either
    // side of the roll axis (same geometry as a car's anti-roll bar / a
    // human's lateral leg-length response to a sideways stumble), so
    // extending one leg while retracting the other by the same amount is a
    // real, physically-grounded lever for this axis - unlike a wheel-spin
    // "reaction wheel" correction, already measured this session as
    // negligible (<1% of body inertia) for TAP's mass distribution.
    //
    // TARGETS THE HIP, NOT THE KNEE (first version, 2026-09-16, targeted the
    // knee and measured NO net effect either sign, n=8 at 50 cm - re-examined
    // the signal path afterward and found why: this reflex's own
    // Catching-phase window (catch_hold_s+catch_blend_s, 0.4s by default)
    // overlaps almost exactly with LandingImpedance's Impact-phase window
    // (max_impact_duration_s, also 0.4s by default) - EmbeddedController
    // applies BOTH to the same knee joint (writeBalancedCommand:
    // output.kp_scale[left_knee] = balance.leg_kp_scale *
    // impedance.left_leg_kp_scale), and Impact intentionally drops that scale
    // as low as energy_kp_scale_min (0.02, i.e. 2% of nominal stiffness) for
    // exactly this window - a real spring-damper, deliberately softened so
    // the leg yields to the impact instead of fighting it. A position offset
    // commanded onto a joint at 2% stiffness produces ~2% of the intended
    // corrective torque: the roll term was effectively unenforceable during
    // the one window it mattered most, which plausibly explains why +0.08
    // and -0.08 measured statistically indistinguishable (both were mostly
    // noise, not signal). The HIP is untouched by LandingImpedance at any
    // phase (output.kp_scale[left_hip] = balance.leg_kp_scale, no impedance
    // factor - see EmbeddedController::writeBalancedCommand's own comment on
    // why hip was excluded from that mechanism) and so keeps full torque
    // authority through the exact window this needs it.
    //
    // offset = clamp(gain * roll_rate_rad_s, -max, max), added to the LEFT
    // hip and subtracted from the RIGHT hip (see update()'s own doc for the
    // sign convention and the caveat that it has NOT yet been validated
    // against real telemetry - flip the gain's sign if a real trial shows
    // roll growing instead of shrinking). 0 (default) is an exact no-op.
    float roll_rate_hip_gain_rad_per_rad_s = 0.0F;
    float roll_hip_offset_max_rad = 0.0F;
    // Capture-point hip placement (2026-09-16): every mechanism above reacts
    // AFTER contact (impedance) or only changes leg LENGTH (active
    // extension). Neither repositions WHERE the wheel will land relative to
    // the CoM's own forward motion - and during the entire airborne window
    // the wheel-drive MPC has zero authority anyway (no ground, no friction -
    // see MpcBalancer::update's own !state.floor_contact branch, which skips
    // the solve and lets commanded_ground_velocity_m_s_ decay to 0). This is
    // the textbook "Instantaneous Capture Point" idea (Pratt, Carff, Raibert,
    // Neuhaus, "Capture Point: A Step toward Humanoid Push Recovery", IEEE
    // Humanoids 2006 - the standard basis for nearly all modern humanoid
    // push-recovery/foot-placement controllers): for a linear-inverted-
    // pendulum model (the SAME model MpcBalancer already uses - same
    // omega = sqrt(g / com_height)), placing the support point at
    // x_capture = x_com + v_com / omega arrests the CoM's horizontal motion
    // at touchdown instead of letting it keep driving the fall. Human stumble
    // recovery does this instinctively (step the foot out further in the
    // direction you are already falling); TAP has no stepping foot, but DOES
    // have a hip DOF that shifts the wheel's ground-contact x-position
    // (WholeBodyModel::legHeight's d_x_d_hip) - so the same target can be
    // realized as a hip angle instead of a footstep.
    // capture_offset_m = forward_velocity_m_s / omega, using the REAL
    // forward_velocity_m_s/com_height_m sampled once at liftoff (LATCHED,
    // not live - 2026-09-16 fix: an earlier version recomputed this every
    // tick off the live, post-touchdown wheel velocity for the ENTIRE
    // Airborne+Catching window; measured on a real 50cm drop to make
    // results WORSE, not better - post-touchdown wheel velocity is
    // genuinely chaotic (impact spikes, bounces), and driving a live hip
    // position command off that raw signal injected noise into exactly the
    // window every other term here deliberately HOLDS a value through
    // instead. Capture-point theory itself is a PRE-touchdown placement
    // decision from the launch trajectory anyway - latching at liftoff is
    // the theoretically correct choice, not just the pragmatic one; see
    // update()'s own liftoff-edge comment for the full account). Converted
    // to a hip angle via the REAL local Jacobian (d_x_d_hip, evaluated at
    // the same liftoff instant exactly like active_extension_gravity_m_s2's
    // own extension_jacobian_ - see that field's comment for why a fixed
    // per-event value is an acceptable approximation), i.e.
    // hip_offset = capture_offset_m / d_x_d_hip - this is an actual
    // Jacobian inversion, not a heuristic gain to sign-guess (cf. the roll
    // term above, which genuinely needed empirical sign validation since it
    // was a heuristic proportional gain, not a geometric inverse).
    // Applied SYMMETRICALLY (both hips together - this is a sagittal/pitch-
    // plane strategy, orthogonal to the roll term's differential L/R split),
    // folded directly into the same latched_hip_offset_rad_/hold+blend path
    // as the pitch-rate hip latch above. 0 (default) is an exact no-op.
    float capture_point_gravity_m_s2 = 0.0F;
    float capture_point_max_hip_rad = 0.0F;
  };

  LandingReflex();
  explicit LandingReflex(Config config) : config_(config) {}

  // True at construction/reset so bring-up (already grounded) never reads as
  // a spurious liftoff, matching MpcBalancer's own prev_floor_contact_ idiom.
  void reset();
  // Seed the edge detector from the first validated contact observation so a
  // startup sample with no contact cannot look like a real liftoff.
  void reset(bool floor_contact);

  // Returns a 6-element offset (always 0 at wheel indices) to ADD to whatever
  // leg targets are otherwise commanded. Caller is responsible for re-clamping
  // the summed pose to the documented hip/knee envelopes. hip_base_rad/
  // knee_base_rad are the current (pre-offset) commanded hip/knee target,
  // used once at liftoff to evaluate the real leg-height Jacobian for
  // active_extension_gravity_m_s2 - see that config field's own comment.
  // roll_rate_rad_s is state.imu.roll_rate_rad_s (angular_velocity.x, body
  // frame - see BalanceControllerNode::onImu). Only applied while airborne or
  // catching (Config::roll_rate_hip_gain_rad_per_rad_s's own comment) -
  // added to the LEFT hip offset, subtracted from the RIGHT.
  // forward_velocity_m_s is the real (measured, not commanded) forward
  // velocity selected by EmbeddedController: a valid/confident
  // TrajectoryObserver estimate from the two encoders, with averaged wheel
  // odometry as a safe fallback. com_height_m is the live whole-body CoM
  // height (EmbeddedController's own last_whole_body_.com_height_m, already
  // computed every tick for MpcBalancer's pendulum length) - both feed
  // Config::capture_point_gravity_m_s2's mechanism, applied symmetrically to
  // both hips.
  [[nodiscard]] JointArray update(bool floor_contact, float pitch_rate_rad_s,
                                  float roll_rate_rad_s,
                                  float accel_magnitude_mps2,
                                  float forward_velocity_m_s,
                                  float com_height_m, float hip_base_rad,
                                  float knee_base_rad, float dt_s);

  [[nodiscard]] bool active() const noexcept {
    return phase_ != Phase::Grounded;
  }

 private:
  enum class Phase { Grounded, Airborne, Catching };

  Config config_;
  Phase phase_ = Phase::Grounded;
  bool prev_floor_contact_ = true;
  bool armed_by_unloading_ = false;
  float latched_hip_offset_rad_ = 0.0F;
  float catch_elapsed_s_ = 0.0F;
  float airborne_elapsed_s_ = 0.0F;
  // d(leg height)/d(knee) evaluated once at liftoff (see
  // Config::active_extension_gravity_m_s2's comment for why a fixed
  // per-event value is an acceptable approximation).
  float extension_jacobian_ = 0.0F;
  // d(leg x)/d(hip) evaluated once at liftoff, same approximation and same
  // instant as extension_jacobian_ above - see
  // Config::capture_point_gravity_m_s2's own comment.
  float capture_jacobian_ = 0.0F;
  float extension_hip_offset_rad_ = 0.0F;
  float extension_knee_offset_rad_ = 0.0F;
  // The actual extension offset commanded last tick, so Catching can hold
  // and blend it back down the same way the crouch/hip offsets already do,
  // instead of snapping to 0 the instant contact is confirmed.
  float last_extension_offset_rad_ = 0.0F;
};

}  // namespace robot_control
