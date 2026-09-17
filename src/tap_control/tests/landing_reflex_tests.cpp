#include <cmath>
#include <iostream>

#include "robot_control/EmbeddedController.hpp"
#include "robot_control/LandingReflex.hpp"
#include "robot_control/WholeBodyModel.hpp"

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
JointArray updateReflex(LandingReflex& reflex, bool floor_contact,
                        float pitch_rate_rad_s, float roll_rate_rad_s,
                        float forward_velocity_m_s, float com_height_m,
                        float hip_base_rad, float knee_base_rad, float dt_s,
                        float accel_magnitude_mps2 = 9.81F) {
  return reflex.update(floor_contact, pitch_rate_rad_s, roll_rate_rad_s,
                       accel_magnitude_mps2, forward_velocity_m_s,
                       com_height_m, hip_base_rad, knee_base_rad, dt_s);
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
    const auto offset = updateReflex(reflex, contact, 3.0F, 0.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.001F);
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
  auto grounded = updateReflex(reflex, true, 0.0F, 0.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.01F);
  check(allZero(grounded), "LandingReflex: inert while grounded");

  // Liftoff edge -> immediately crouches both knees, hips untouched (gain 0).
  auto airborne = updateReflex(reflex, false, 0.0F, 0.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.01F);
  check(near_equal(airborne[static_cast<std::size_t>(JointId::LeftKnee)], -0.3F) &&
        near_equal(airborne[static_cast<std::size_t>(JointId::RightKnee)], -0.3F) &&
        airborne[static_cast<std::size_t>(JointId::LeftHip)] == 0.0F,
        "LandingReflex: symmetric knee crouch on liftoff, hips untouched at zero gain");

  // Stays crouched while still airborne.
  auto still_airborne = updateReflex(reflex, false, 0.0F, 0.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.05F);
  check(near_equal(still_airborne[static_cast<std::size_t>(JointId::LeftKnee)], -0.3F),
        "LandingReflex: holds crouch while airborne");

  // Touchdown edge -> still fully crouched at the instant of contact (hold window).
  auto touchdown = updateReflex(reflex, true, 0.0F, 0.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.01F);
  check(near_equal(touchdown[static_cast<std::size_t>(JointId::LeftKnee)], -0.3F),
        "LandingReflex: crouch held at the touchdown instant");

  // Past the hold window, mid-blend: strictly between -0.3 and 0.
  auto mid_blend = updateReflex(reflex, true, 0.0F, 0.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.15F);
  const float mid_knee = mid_blend[static_cast<std::size_t>(JointId::LeftKnee)];
  check(mid_knee < -1.0e-4F && mid_knee > -0.3F + 1.0e-4F,
        "LandingReflex: blends back partway after the hold window");

  // Long after touchdown: fully blended back to zero and phase is Grounded.
  auto settled = updateReflex(reflex, true, 0.0F, 0.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.5F);
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
  (void)updateReflex(reflex, true, 0.0F, 0.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.01F);
  auto airborne = updateReflex(reflex, false, 0.0F, 0.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.01F);
  check(near_equal(airborne[static_cast<std::size_t>(JointId::LeftKnee)], -0.3F),
        "LandingReflex: crouches on the real liftoff");
  // A single-tick spurious "grounded" reading only 20 ms after liftoff -
  // well under the 150 ms debounce - must be ignored, staying crouched.
  auto spurious = updateReflex(reflex, true, 0.0F, 0.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.02F);
  check(near_equal(spurious[static_cast<std::size_t>(JointId::LeftKnee)], -0.3F) &&
        reflex.active(),
        "LandingReflex: ignores a single spurious early-touchdown reading");
  // Back to a real airborne reading; must still be fully crouched, not stuck
  // mid-blend from the rejected touchdown.
  auto still_airborne = updateReflex(reflex, false, 0.0F, 0.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.05F);
  check(near_equal(still_airborne[static_cast<std::size_t>(JointId::LeftKnee)], -0.3F),
        "LandingReflex: resumes full crouch after the spurious reading clears");
  // Real touchdown, well past the debounce window this time (accumulated
  // airborne time across all prior ticks now exceeds 0.15 s).
  auto touchdown = updateReflex(reflex, true, 0.0F, 0.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.2F);
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
  (void)updateReflex(reflex, true, 0.0F, 0.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.01F);
  // Liftoff at a large pitch rate: 0.1 * 5.0 = 0.5, clamped to 0.15.
  auto airborne = updateReflex(reflex, false, 5.0F, 0.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.01F);
  check(near_equal(airborne[static_cast<std::size_t>(JointId::LeftHip)], 0.15F) &&
        near_equal(airborne[static_cast<std::size_t>(JointId::RightHip)], 0.15F),
        "LandingReflex: hip offset latched at liftoff and clamped to max_hip_offset_rad");
  // Later pitch-rate changes mid-flight must not re-latch the offset.
  auto still_airborne = updateReflex(reflex, false, -5.0F, 0.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.01F);
  check(near_equal(still_airborne[static_cast<std::size_t>(JointId::LeftHip)], 0.15F),
        "LandingReflex: latched hip offset does not react to pitch rate after liftoff");
}

void rollTermIsDifferentialClampedAndGroundedInert() {
  LandingReflex::Config cfg;
  cfg.roll_rate_hip_gain_rad_per_rad_s = 0.05F;
  cfg.roll_hip_offset_max_rad = 0.1F;
  LandingReflex reflex(cfg);

  // Grounded: roll gain must stay inert like every other mechanism here.
  auto grounded = updateReflex(reflex, true, 0.0F, 4.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.01F);
  check(allZero(grounded),
        "LandingReflex: roll term inert while grounded despite nonzero roll_rate");

  // Liftoff, moderate roll rate: 0.05 * 2.0 = 0.1, within the 0.1 clamp -
  // added to the left hip, subtracted from the right (opposite signs). The
  // hip, not the knee, is the target - see Config::
  // roll_rate_hip_gain_rad_per_rad_s's own comment for why (LandingImpedance
  // crushes knee stiffness during exactly the window this needs authority).
  auto airborne = updateReflex(reflex, false, 0.0F, 2.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.01F);
  const float l_hip = airborne[static_cast<std::size_t>(JointId::LeftHip)];
  const float r_hip = airborne[static_cast<std::size_t>(JointId::RightHip)];
  check(near_equal(l_hip, 0.1F) && near_equal(r_hip, -0.1F),
        "LandingReflex: roll term added to left hip, subtracted from right");
  check(allZero({0.0F, airborne[static_cast<std::size_t>(JointId::LeftKnee)],
                0.0F, 0.0F,
                airborne[static_cast<std::size_t>(JointId::RightKnee)], 0.0F}),
        "LandingReflex: roll term does not touch the knees");

  // Large roll rate must clamp, not blow past roll_hip_offset_max_rad.
  auto clamped = updateReflex(reflex, false, 0.0F, 100.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.01F);
  check(near_equal(clamped[static_cast<std::size_t>(JointId::LeftHip)], 0.1F) &&
        near_equal(clamped[static_cast<std::size_t>(JointId::RightHip)], -0.1F),
        "LandingReflex: roll term clamps to roll_hip_offset_max_rad");

  // Opposite-sign roll rate flips which hip extends.
  auto flipped = updateReflex(reflex, false, 0.0F, -2.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.01F);
  check(near_equal(flipped[static_cast<std::size_t>(JointId::LeftHip)], -0.1F) &&
        near_equal(flipped[static_cast<std::size_t>(JointId::RightHip)], 0.1F),
        "LandingReflex: negative roll rate flips which hip extends");
}

void capturePointIsSymmetricInvertsJacobianAndGroundedInert() {
  LandingReflex::Config cfg;
  cfg.capture_point_gravity_m_s2 = 9.81F;
  cfg.capture_point_max_hip_rad = 1.0F;  // generous, not the thing under test
  LandingReflex reflex(cfg);

  // Grounded: inert like every other mechanism here.
  auto grounded = updateReflex(reflex, true, 0.0F, 0.0F, 0.5F, 0.2F, 0.0F, 0.0F, 0.01F);
  check(allZero(grounded),
        "LandingReflex: capture term inert while grounded despite nonzero forward velocity");

  // Liftoff at hip=knee=0 captures the real Jacobian there.
  (void)updateReflex(reflex, true, 0.0F, 0.0F, 0.0F, 0.2F, 0.0F, 0.0F, 0.01F);
  const auto leg0 = WholeBodyModel::legHeight(0.0F, 0.0F);
  check(std::abs(leg0.d_x_d_hip) > 1.0e-4F,
        "LandingReflex test precondition: d_x_d_hip at hip=knee=0 is not degenerate");

  // Forward velocity 0.5 m/s, com_height 0.2m: omega = sqrt(9.81/0.2),
  // capture_offset_m = 0.5 / omega, hip term = that / d_x_d_hip. Applied
  // SYMMETRICALLY (same sign both hips) - unlike the roll term.
  const float omega = std::sqrt(9.81F / 0.2F);
  const float expected = (0.5F / omega) / leg0.d_x_d_hip;
  auto airborne = updateReflex(reflex, false, 0.0F, 0.0F, 0.5F, 0.2F, 0.0F, 0.0F, 0.01F);
  check(near_equal(airborne[static_cast<std::size_t>(JointId::LeftHip)], expected, 1.0e-3F) &&
        near_equal(airborne[static_cast<std::size_t>(JointId::RightHip)], expected, 1.0e-3F),
        "LandingReflex: capture term applied symmetrically, matches Jacobian-inverted formula");

  // The term is LATCHED at liftoff (2026-09-16 fix - see Config::
  // capture_point_gravity_m_s2's own comment): a changed forward_velocity_m_s
  // mid-flight, with no new liftoff edge, must NOT move it.
  auto still_airborne = updateReflex(reflex, false, 0.0F, 0.0F, -0.5F, 0.2F, 0.0F, 0.0F, 0.01F);
  check(near_equal(still_airborne[static_cast<std::size_t>(JointId::LeftHip)], expected, 1.0e-3F),
        "LandingReflex: capture term stays latched mid-flight despite a changed forward velocity");

  // Force a full touchdown-and-settle (large dt blows past the debounce and
  // the catch hold+blend in one call), then a FRESH liftoff with reversed
  // velocity must relatch with the opposite sign.
  (void)updateReflex(reflex, true, 0.0F, 0.0F, -0.5F, 0.2F, 0.0F, 0.0F, 0.6F);
  auto reversed = updateReflex(reflex, false, 0.0F, 0.0F, -0.5F, 0.2F, 0.0F, 0.0F, 0.01F);
  check(near_equal(reversed[static_cast<std::size_t>(JointId::LeftHip)], -expected, 1.0e-3F) &&
        near_equal(reversed[static_cast<std::size_t>(JointId::RightHip)], -expected, 1.0e-3F),
        "LandingReflex: a fresh liftoff relatches the capture term with reversed velocity's sign");
}

void imuUnloadingPrearmsBeforeContactDrops() {
  LandingReflex::Config cfg;
  cfg.unloading_accel_mps2 = 2.0F;
  cfg.airborne_knee_crouch_rad = 0.2F;
  LandingReflex reflex(cfg);

  auto grounded = updateReflex(reflex, true, 0.0F, 0.0F, 0.0F, 0.2F,
                               0.9F, -1.3F, 0.01F, 9.81F);
  check(allZero(grounded),
        "LandingReflex: normal grounded accel does not pre-arm");

  auto unloading = updateReflex(reflex, true, 0.0F, 0.0F, 0.0F, 0.2F,
                                0.9F, -1.3F, 0.01F, 0.4F);
  check(allZero(unloading) && reflex.active(),
        "LandingReflex: low IMU specific force pre-arms without moving while contact remains true");
  auto airborne = updateReflex(reflex, false, 0.0F, 0.0F, 0.0F, 0.2F,
                               0.9F, -1.3F, 0.01F, 0.4F);
  check(near_equal(airborne[static_cast<std::size_t>(JointId::LeftKnee)], -0.2F),
        "LandingReflex: pre-armed reflex commands once contact drops");
}

void activeExtensionUsesHipAndKneeWithSlewLimit() {
  LandingReflex::Config cfg;
  cfg.active_extension_gravity_m_s2 = 9.81F;
  cfg.active_extension_max_rad = 0.22F;
  cfg.active_extension_max_hip_rad = 0.04F;
  cfg.target_slew_rate_rad_s = 1.0F;
  LandingReflex reflex(cfg);
  const float hip = 0.9F;
  const float knee = -1.3962634F;

  (void)updateReflex(reflex, true, 0.0F, 0.0F, 0.0F, 0.14F, hip, knee, 0.01F);
  auto airborne = updateReflex(reflex, false, 0.0F, 0.0F, 0.0F, 0.14F,
                               hip, knee, 0.10F);
  const float l_hip = airborne[static_cast<std::size_t>(JointId::LeftHip)];
  const float l_knee = airborne[static_cast<std::size_t>(JointId::LeftKnee)];
  check(l_hip < -1.0e-4F && l_knee > 1.0e-4F,
        "LandingReflex: task-space extension uses hip and knee, not knee-only");
  check(std::abs(l_hip) <= 0.0401F,
        "LandingReflex: active extension respects the separate hip cap");
  check(std::abs(l_hip) <= 0.1001F && std::abs(l_knee) <= 0.1001F,
        "LandingReflex: target slew limit bounds first extension step");
}

void activeExtensionDelayPreventsEarlyEdgeMotion() {
  LandingReflex::Config cfg;
  cfg.active_extension_gravity_m_s2 = 9.81F;
  cfg.active_extension_max_rad = 0.22F;
  cfg.active_extension_max_hip_rad = 0.04F;
  cfg.active_extension_delay_s = 0.08F;
  LandingReflex reflex(cfg);
  const float hip = 0.9F;
  const float knee = -1.3962634F;

  (void)updateReflex(reflex, true, 0.0F, 0.0F, 0.0F, 0.14F, hip, knee, 0.01F);
  auto early = updateReflex(reflex, false, 0.0F, 0.0F, 0.0F, 0.14F,
                            hip, knee, 0.05F);
  check(near_equal(early[static_cast<std::size_t>(JointId::LeftHip)], 0.0F) &&
        near_equal(early[static_cast<std::size_t>(JointId::LeftKnee)], 0.0F),
        "LandingReflex: active extension delay prevents early ledge-edge motion");

  auto delayed = updateReflex(reflex, false, 0.0F, 0.0F, 0.0F, 0.14F,
                              hip, knee, 0.08F);
  check(delayed[static_cast<std::size_t>(JointId::LeftKnee)] > 0.0F,
        "LandingReflex: active extension starts after the configured delay");
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
  rollTermIsDifferentialClampedAndGroundedInert();
  capturePointIsSymmetricInvertsJacobianAndGroundedInert();
  imuUnloadingPrearmsBeforeContactDrops();
  activeExtensionUsesHipAndKneeWithSlewLimit();
  activeExtensionDelayPreventsEarlyEdgeMotion();
  embeddedControllerNoOpAtDefaults();
  std::cout << (failures == 0 ? "ALL PASS\n" : "FAILURES PRESENT\n");
  return failures == 0 ? 0 : 1;
}
