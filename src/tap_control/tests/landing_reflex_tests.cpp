#include <cmath>
#include <iostream>

#include "robot_control/EmbeddedController.hpp"
#include "robot_control/LandingReflex.hpp"

using namespace robot_control;
namespace {
int failures = 0;
void check(bool condition, const char* name) {
  std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
  if (!condition) ++failures;
}
bool allZero(const JointArray& a) {
  for (float v : a) {
    if (v != 0.0F) return false;
  }
  return true;
}
bool near_equal(float a, float b, float eps = 1e-5F) {
  return std::abs(a - b) < eps;
}

// Direct unit tests of LandingReflex in isolation.
void noOpAtDefaults() {
  LandingReflex reflex;  // Config{} - every gain 0.
  bool ok = true;
  // Exercise every contact transition an actual sim run could produce,
  // including liftoff/touchdown edges, and confirm the offset is exactly
  // zero every single cycle regardless.
  const bool contact_sequence[] = {true, true, false, false, false,
                                   true, true, false, true, true};
  for (bool contact : contact_sequence) {
    const auto offset = reflex.update(contact, 3.0F, 0.001F);
    ok = ok && allZero(offset);
  }
  check(ok, "LandingReflex: all-zero config is a no-op across every contact transition");
}

void crouchesOnLiftoffAndHoldsThenBlends() {
  LandingReflex::Config cfg;
  cfg.airborne_knee_crouch_rad = 0.3F;
  cfg.hip_offset_per_pitch_rate = 0.0F;
  cfg.catch_hold_s = 0.1F;
  cfg.catch_blend_s = 0.1F;
  LandingReflex reflex(cfg);

  // Grounded: inert.
  auto grounded = reflex.update(true, 0.0F, 0.01F);
  check(allZero(grounded), "LandingReflex: inert while grounded");

  // Liftoff edge -> immediately crouches both knees, hips untouched (gain 0).
  auto airborne = reflex.update(false, 0.0F, 0.01F);
  check(near_equal(airborne[static_cast<std::size_t>(JointId::LeftKnee)], -0.3F) &&
        near_equal(airborne[static_cast<std::size_t>(JointId::RightKnee)], -0.3F) &&
        airborne[static_cast<std::size_t>(JointId::LeftHip)] == 0.0F,
        "LandingReflex: symmetric knee crouch on liftoff, hips untouched at zero gain");

  // Stays crouched while still airborne.
  auto still_airborne = reflex.update(false, 0.0F, 0.05F);
  check(near_equal(still_airborne[static_cast<std::size_t>(JointId::LeftKnee)], -0.3F),
        "LandingReflex: holds crouch while airborne");

  // Touchdown edge -> still fully crouched at the instant of contact (hold window).
  auto touchdown = reflex.update(true, 0.0F, 0.01F);
  check(near_equal(touchdown[static_cast<std::size_t>(JointId::LeftKnee)], -0.3F),
        "LandingReflex: crouch held at the touchdown instant");

  // Past the hold window, mid-blend: strictly between -0.3 and 0.
  auto mid_blend = reflex.update(true, 0.0F, 0.15F);
  const float mid_knee = mid_blend[static_cast<std::size_t>(JointId::LeftKnee)];
  check(mid_knee < -1.0e-4F && mid_knee > -0.3F + 1.0e-4F,
        "LandingReflex: blends back partway after the hold window");

  // Long after touchdown: fully blended back to zero and phase is Grounded.
  auto settled = reflex.update(true, 0.0F, 0.5F);
  check(allZero(settled) && !reflex.active(),
        "LandingReflex: fully blended back to zero and inactive well after touchdown");
}

// Regression for a real ledge-drive-off trial (see docs/LEDGE_DRIVEOFF_LIMIT.md):
// the sim_pose floor_contact heuristic read one spurious "grounded" tick
// ~10-50 ms after a genuine liftoff, well before the robot had actually
// fallen, which collapsed the airborne crouch back to neutral almost
// immediately. min_airborne_before_touchdown_s must reject that early blip
// and keep crouching until real touchdown.
void rejectsSpuriousEarlyTouchdown() {
  LandingReflex::Config cfg;
  cfg.airborne_knee_crouch_rad = 0.3F;
  cfg.min_airborne_before_touchdown_s = 0.15F;
  cfg.catch_hold_s = 0.1F;
  cfg.catch_blend_s = 0.1F;
  LandingReflex reflex(cfg);
  (void)reflex.update(true, 0.0F, 0.01F);
  auto airborne = reflex.update(false, 0.0F, 0.01F);
  check(near_equal(airborne[static_cast<std::size_t>(JointId::LeftKnee)], -0.3F),
        "LandingReflex: crouches on the real liftoff");
  // A single-tick spurious "grounded" reading only 20 ms after liftoff -
  // well under the 150 ms debounce - must be ignored, staying crouched.
  auto spurious = reflex.update(true, 0.0F, 0.02F);
  check(near_equal(spurious[static_cast<std::size_t>(JointId::LeftKnee)], -0.3F) &&
        reflex.active(),
        "LandingReflex: ignores a single spurious early-touchdown reading");
  // Back to a real airborne reading; must still be fully crouched, not stuck
  // mid-blend from the rejected touchdown.
  auto still_airborne = reflex.update(false, 0.0F, 0.05F);
  check(near_equal(still_airborne[static_cast<std::size_t>(JointId::LeftKnee)], -0.3F),
        "LandingReflex: resumes full crouch after the spurious reading clears");
  // Real touchdown, well past the debounce window this time (accumulated
  // airborne time across all prior ticks now exceeds 0.15 s).
  auto touchdown = reflex.update(true, 0.0F, 0.2F);
  check(touchdown[static_cast<std::size_t>(JointId::LeftKnee)] > -0.3F + 1.0e-4F,
        "LandingReflex: a touchdown reading held past the debounce window is honored");
}

void hipOffsetLatchesAtLiftoffAndClamps() {
  LandingReflex::Config cfg;
  cfg.airborne_knee_crouch_rad = 0.0F;
  cfg.hip_offset_per_pitch_rate = 0.1F;
  cfg.max_hip_offset_rad = 0.15F;
  cfg.catch_hold_s = 1.0F;
  cfg.catch_blend_s = 0.1F;
  LandingReflex reflex(cfg);
  (void)reflex.update(true, 0.0F, 0.01F);
  // Liftoff at a large pitch rate: 0.1 * 5.0 = 0.5, clamped to 0.15.
  auto airborne = reflex.update(false, 5.0F, 0.01F);
  check(near_equal(airborne[static_cast<std::size_t>(JointId::LeftHip)], 0.15F) &&
        near_equal(airborne[static_cast<std::size_t>(JointId::RightHip)], 0.15F),
        "LandingReflex: hip offset latched at liftoff and clamped to max_hip_offset_rad");
  // Later pitch-rate changes mid-flight must not re-latch the offset.
  auto still_airborne = reflex.update(false, -5.0F, 0.01F);
  check(near_equal(still_airborne[static_cast<std::size_t>(JointId::LeftHip)], 0.15F),
        "LandingReflex: latched hip offset does not react to pitch rate after liftoff");
}

// Integration: confirm EmbeddedController with an all-default Config produces
// byte-identical MotorCommand output whether or not floor_contact toggles,
// proving the reflex wiring introduces no behavior change in production.
void embeddedControllerNoOpAtDefaults() {
  EmbeddedController controller;
  RobotState state;
  state.dt_s = 0.001F;
  state.joints.position_rad = {geometry::neutral_hip_rad, 0.0F, 0.0F,
                               geometry::neutral_hip_rad, 0.0F, 0.0F};
  ControlCommand command;
  command.target_forward_velocity_m_s = 0.2F;
  command.target_joint_position_rad = state.joints.position_rad;

  controller.reset(state);
  // Enable and settle into Balance the same way core_tests does elsewhere:
  command.request_enable = true;
  MotorCommand baseline{};
  for (int i = 0; i < 200; ++i) {
    state.floor_contact = true;
    baseline = controller.update(state, command);
  }
  check(controller.mode() == ControlMode::Balance ||
        controller.mode() == ControlMode::Drive,
        "EmbeddedController: reaches Balance/Drive before the no-op comparison");

  EmbeddedController controller_b;
  controller_b.reset(state);
  MotorCommand toggled{};
  for (int i = 0; i < 200; ++i) {
    // Toggle floor_contact every cycle - with default (zero-gain) config this
    // must have ZERO effect on the commanded leg targets, since LandingReflex
    // returns all-zero offsets regardless of the contact sequence.
    state.floor_contact = (i % 2 == 0);
    toggled = controller_b.update(state, command);
  }
  bool same = baseline.enable_motors == toggled.enable_motors;
  for (std::size_t i = 0; i < kJointCount && same; ++i) {
    same = same && baseline.position_rad[i] == toggled.position_rad[i] &&
           baseline.kp_scale[i] == toggled.kp_scale[i] &&
           baseline.kd_scale[i] == toggled.kd_scale[i];
  }
  check(same,
        "EmbeddedController: default LandingReflex config is a no-op even "
        "under a toggling floor_contact signal");
}
}  // namespace

int main() {
  noOpAtDefaults();
  crouchesOnLiftoffAndHoldsThenBlends();
  rejectsSpuriousEarlyTouchdown();
  hipOffsetLatchesAtLiftoffAndClamps();
  embeddedControllerNoOpAtDefaults();
  std::cout << (failures == 0 ? "ALL PASS\n" : "FAILURES PRESENT\n");
  return failures == 0 ? 0 : 1;
}
