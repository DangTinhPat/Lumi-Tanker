#include "tap_fw/FirmwareProfile.hpp"

#include "robot_control/TunedConfig.hpp"
#include "tap_fw/BoardConfig.hpp"
#include "tap_fw/LegHoming.hpp"

namespace tap_fw {

robot_control::EmbeddedController::Config firmwareControllerConfig() {
  auto cfg = robot_control::tunedControllerConfig();
  cfg.state_machine.fixture_startup_enabled = true;

  // Values from src/tap_control/tests/balance_trim_tests.cpp: seeded trim
  // plus 0.3 rad/m settles +-8 deg equilibria with a +-2 deg release error.
  cfg.seed_balance_trim_from_pitch = true;
  cfg.lqr.balance_trim_gain_rad_per_m = 0.30F;
  cfg.lqr.balance_trim_max_rad = 0.20F;
  cfg.lqr.balance_trim_pitch_gate_rad = 0.25F;
  cfg.lqr.command_velocity_tracking_margin_m_s = 0.25F;

  cfg.whole_body_stance.enabled = false;
  cfg.whole_body_stance_owns_static_com = false;
  // The simulator profile scales the wheel pitch feedback to 0.7 because the
  // stance task's leg gimbal absorbs the rest of a small pitch error. With the
  // stance task off nothing absorbs it; the wheel LQR needs its full designed
  // gain. On a pendulum plant with a lagging wheel speed servo, 0.7 loses the
  // robot once the lag reaches ~20 ms; 1.0 holds up to ~60 ms
  // (firmware_profile_tests).
  cfg.lqr.wheel_pitch_feedback_scale = 1.0F;

  // Start #1 happens with the body held up and the legs hanging free (gravity
  // homing): accept any leg pose and fold the legs to the stance in the air
  // (~5 s from hanging), then Start #2 releases.
  cfg.state_machine.startup_joint_tolerance_rad = 4.0F;
  cfg.fixture_hold_slew_rad_s = 0.30F;
  // From wherever the legs are, first back to the homing reference (legs
  // straight, hanging), then to the stance.
  cfg.fixture_hold_via_enabled = true;
  // The legs stopped 0.25 rad short of the stance on the robot (soft joint
  // gain against gearbox friction/load at the 3 N.m envelope). The integral
  // closes that gap in ~1 s without raising Kp (which buzzed).
  // The torque-feed-forward integral is not realized by the GDS68 (see
  // board::kLegTargetIntegralPerS): the bridge integrates on the target.
  cfg.leg_position_integral_gain_nm_per_rad_s = 0.0F;
  cfg.leg_position_integral_max_nm = 2.5F;
  // With the integral the legs do reach the stance: require it within 0.03 rad
  // (1.7 deg) before Start #2 may release.
  cfg.fixture_hold_arrival_tolerance_rad = 0.03F;
  cfg.fixture_hold_via_rad = hangingLegPose(hangPose(board::kKneeExtensionStopRad), 0.0F);
  if (board::kLegsLockedAtZeroPose) {
    // Two-wheel mode: the legs stay at the captured pose; no via, no fold.
    cfg.fixture_hold_via_enabled = false;
    cfg.leg_max_torque_nm = board::kLegLockTorqueNm;
  }
  return cfg;
}

robot_control::EmbeddedController::Config impulseStyleFirmwareControllerConfig() {
  auto cfg = firmwareControllerConfig();
  // The operator lifts the body and manually presents both straight legs below
  // it before Start. Capture and hold that same pose through the balance ramp;
  // do not fold to CAD natural before support is removed.
  cfg.fixture_hold_via_enabled = false;
  cfg.seed_balance_trim_from_pitch = true;
  // The homing layer maps the manually presented extended pose to the model's
  // gravity-hang coordinates before SensorQualify runs.
  cfg.state_machine.startup_joint_tolerance_rad = 4.0F;
  cfg.state_machine.startup_pitch_rad = 0.15F;
  cfg.state_machine.startup_pitch_rate_rad_s = 0.35F;
  cfg.state_machine.sensor_qualify_cycles = 50U;
  cfg.state_machine.balance_ramp_cycles = 100U;
  cfg.fixture_hold_slew_rad_s = 0.30F;
  cfg.fixture_hold_arrival_tolerance_rad = 0.03F;
  return cfg;
}

}  // namespace tap_fw
