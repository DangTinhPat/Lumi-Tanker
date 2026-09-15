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
// The airborne signal reused here (`floor_contact` false) is the SAME
// pitch/height-based simulation contact inference already gating
// MpcBalancer's landing_window (see BalanceControllerNode's sim_pose contact
// check) - no new sensor, no ROS dependency added to this class.
//
// All gains default to 0 (crouch 0, hip gain 0): `update()` then always
// returns an all-zero offset for any floor_contact/pitch_rate/dt sequence, so
// an unconfigured controller is byte-identical to no reflex at all (see
// LandingReflexTest.NoOpAtDefaults / EmbeddedController tests).
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
    // Hold the airborne offset through touchdown for this long, then blend
    // linearly back to zero over catch_blend_s - an actual commanded motion
    // through the impact window, not an instantaneous snap.
    float catch_hold_s = 0.2F;
    float catch_blend_s = 0.2F;
    // Debounce: a touchdown_edge is only honored once at least this much time
    // has elapsed since the liftoff_edge that opened the current Airborne
    // phase. Measured directly against a real 20 cm/1.0 m/s ledge drive-off
    // (see docs/LEDGE_DRIVEOFF_LIMIT.md): the sim_pose floor_contact heuristic
    // (BalanceControllerNode's z/pitch thresholds) can read a single spurious
    // "grounded" tick within ~10-50 ms of actually leaving the pad, well
    // before the robot has fallen any meaningful distance -- without this
    // guard that single tick fires touchdown_edge immediately, collapsing the
    // whole airborne crouch back to neutral long before the real impact. 0 =
    // no debounce (matches the original, vulnerable behavior).
    float min_airborne_before_touchdown_s = 0.15F;
  };

  LandingReflex();
  explicit LandingReflex(Config config) : config_(config) {}

  // True at construction/reset so bring-up (already grounded) never reads as
  // a spurious liftoff, matching MpcBalancer's own prev_floor_contact_ idiom.
  void reset();

  // Returns a 6-element offset (always 0 at wheel indices) to ADD to whatever
  // leg targets are otherwise commanded. Caller is responsible for re-clamping
  // the summed pose to the documented hip/knee envelopes.
  [[nodiscard]] JointArray update(bool floor_contact, float pitch_rate_rad_s,
                                  float dt_s);

  [[nodiscard]] bool active() const noexcept {
    return phase_ != Phase::Grounded;
  }

 private:
  enum class Phase { Grounded, Airborne, Catching };

  Config config_;
  Phase phase_ = Phase::Grounded;
  bool prev_floor_contact_ = true;
  float latched_hip_offset_rad_ = 0.0F;
  float catch_elapsed_s_ = 0.0F;
  float airborne_elapsed_s_ = 0.0F;
};

}  // namespace robot_control
