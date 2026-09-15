#include <cmath>
#include <iostream>
#include <limits>
#include "robot_control/EmbeddedController.hpp"

using namespace robot_control;
namespace {
int failures = 0;
void check(bool condition, const char* name) {
  std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
  if (!condition) ++failures;
}
bool near(float a, float b, float eps = 1e-5F) { return std::abs(a-b) < eps; }
bool stopped(const MotorCommand& out) {
  if (out.enable_motors) return false;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (out.velocity_rad_s[i] != 0 || out.torque_nm[i] != 0 ||
        out.kp_scale[i] != 0 || out.kd_scale[i] != 0) return false;
  }
  return true;
}
void motionTests() {
  JointMotionPlanner planner;
  JointArray start{}, target{.6F, -1.2F, 0, .6F, -1.2F, 0};
  planner.reset(start);
  planner.startMove(start, target, 1.0F);
  check(planner.update(-.5F) == start, "negative dt does not move pose");
  const auto mid = planner.update(.5F);
  check(near(mid[0], .3F) && near(mid[1], -.6F) && mid[0] == mid[3],
        "smooth interpolation midpoint and native same-sign legs");
  check(planner.update(.5F) == target && !planner.active(), "exact pose endpoint");
  planner.startMove(target, start, 1);
  planner.reset(target);
  check(!planner.active() && planner.update(1) == target, "pose cancellation");
}
void geometryTests() {
  WholeBodyModel model;
  const float h = geometry::neutral_hip_rad;
  const auto stand = model.compute(h, 0, h, 0);
  const auto squat = model.compute(.75838357F, -1.2F, .75838357F, -1.2F);
  check(near(geometry::total_mass_kg, 3.938F), "seven link mass is 3.938 kg");
  check(std::abs(stand.com_offset_x_m) < 1e-6F && near(stand.com_height_m, .21730207F),
        "native neutral standing COM from rotated-body URDF");
  check(std::abs(squat.com_offset_x_m) < 1e-6F && squat.com_height_m < stand.com_height_m-.05F,
        "negative-knee native squat lowers COM without lean");
  const auto shifted = model.compute(h+.1F, 0, h+.1F, 0);
  const float ref = WholeBodyModel::equilibriumPitch(shifted);
  check(near(std::cos(ref)*shifted.com_offset_x_m +
             std::sin(ref)*shifted.com_above_axle_m, 0), "equilibrium rotates COM over axle");
  const auto asym = model.compute(.3F, -.2F, .5F, -.8F);
  check(std::abs(asym.axle_height_difference_m) > .001F, "asymmetric FK retains unequal axle heights");
}
void mpcTests() {
  MpcBalancer::Config config;
  MpcBalancer mpc(config);
  RobotState state;
  ControlCommand command;
  auto out = mpc.update(state, command);
  check(near(out.ground_velocity_m_s, 0), "zero MPC state is neutral");
  state.imu.pitch_rad = .05F;
  out = mpc.update(state, command);
  check(out.left_wheel_velocity_rad_s > 0 && out.right_wheel_velocity_rad_s > 0,
        "positive pitch commands both native wheels forward");
  mpc.reset();
  state.imu.pitch_rad = -.05F;
  out = mpc.update(state, command);
  check(out.left_wheel_velocity_rad_s < 0 && out.right_wheel_velocity_rad_s < 0,
        "negative pitch commands both native wheels backward");
  state.imu.pitch_rad = 0;
  mpc.reset();
  command.target_yaw_rate_rad_s = .4F;
  out = mpc.update(state, command);
  check(out.left_wheel_velocity_rad_s < 0 && out.right_wheel_velocity_rad_s > 0 &&
        near(config.wheel_radius_m*(out.right_wheel_velocity_rad_s-out.left_wheel_velocity_rad_s)/
             (2*config.half_wheel_base_m), .4F), "positive ROS yaw is right-minus-left");
  command = {};
  state.imu.pitch_rad = .2F;
  mpc.reset();
  out = mpc.update(state, command, true, .2F);
  check(near(out.ground_velocity_m_s, 0), "nonzero equilibrium does not invent gravity drift");
  state.imu.pitch_rad = .25F;
  out = mpc.update(state, command, true, .2F);
  const float shifted_command = out.ground_velocity_m_s;
  mpc.reset();
  state.imu.pitch_rad = .05F;
  out = mpc.update(state, command);
  check(near(shifted_command, out.ground_velocity_m_s),
        "QP dynamics are invariant to equilibrium pitch frame shift");
  bool integration_matches = true;
  for (const float pitch : {-.02F, .02F}) {
    mpc.reset();
    state.imu.pitch_rad = pitch;
    float previous_velocity = 0.0F;
    for (const float dt : {.0025F, .005F, .01F, .02F}) {
      state.dt_s = dt;
      out = mpc.update(state, command, true, 0.0F, false);
      const float accel = mpc.debugState().optimal_command_m_s;
      integration_matches &= !out.fallen && std::abs(accel) > .01F &&
          std::abs(accel) < config.max_ground_accel_m_s2 &&
          std::abs(out.ground_velocity_m_s) < config.max_ground_velocity_m_s &&
          near(out.ground_velocity_m_s-previous_velocity, accel*dt, 1e-7F);
      previous_velocity = out.ground_velocity_m_s;
    }
  }
  check(integration_matches,
        "unsaturated velocity increment equals reported acceleration times elapsed dt");
  state.dt_s = .005F;
  mpc.reset();
  state.imu.pitch_rad = .55F;
  command.target_forward_velocity_m_s = 100;
  bool constrained = true;
  for (int i = 0; i < 200; ++i) {
    out = mpc.update(state, command);
    constrained &= std::isfinite(out.ground_velocity_m_s) &&
      std::abs(out.ground_velocity_m_s) <= config.max_ground_velocity_m_s+1e-6F &&
      std::abs(mpc.debugState().optimal_command_m_s) <= config.max_ground_accel_m_s2+1e-6F;
  }
  check(constrained, "MPC acceleration projection and velocity saturation");
  state.floor_contact = false;
  const float before = std::abs(out.ground_velocity_m_s);
  out = mpc.update(state, command);
  check(std::abs(out.ground_velocity_m_s) < before, "airborne command decays");
  mpc.setComHeight(.1F);
  check(near(mpc.activePendulumLengthM(), config.min_pendulum_length_m), "adaptive pendulum lower clamp");
  mpc.setComHeight(1);
  check(near(mpc.activePendulumLengthM(), config.max_pendulum_length_m), "adaptive pendulum upper clamp");
  state.imu.pitch_rad = std::numeric_limits<float>::quiet_NaN();
  out = mpc.update(state, command);
  check(out.fallen && out.left_wheel_velocity_rad_s == 0 && out.right_wheel_velocity_rad_s == 0,
        "MPC rejects nonfinite sensor input");
}
void landingWindowTests() {
  // Disabled config (all landing_* boosts default 0): a touchdown edge must
  // not change the output at all, matching the previous no-landing-logic path.
  {
    MpcBalancer::Config config;
    MpcBalancer mpc(config);
    RobotState state;
    ControlCommand command;
    state.floor_contact = false;
    (void)mpc.update(state, command);
    state.floor_contact = true;
    const auto touchdown = mpc.update(state, command);
    check(near(touchdown.leg_kp_scale, config.leg_kp_scale) &&
          near(touchdown.leg_kd_scale, config.leg_kd_scale),
          "disabled landing config leaves scales unchanged across a touchdown edge");
  }

  // Nonzero boosts: the window opens at its full configured length exactly on
  // the touchdown tick and decays to zero once landing_window_s has elapsed.
  {
    MpcBalancer::Config config;
    config.landing_window_s = 0.1F;
    config.landing_kp_boost = 0.4F;
    config.landing_kd_boost = 0.3F;
    MpcBalancer mpc(config);
    RobotState state;
    ControlCommand command;
    state.dt_s = 0.02F;
    state.floor_contact = false;
    auto out = mpc.update(state, command);
    check(near(out.leg_kp_scale, config.leg_kp_scale) &&
          near(mpc.debugState().landing_window_remaining_s, 0.0F),
          "airborne tick keeps the landing window closed");

    state.floor_contact = true;
    out = mpc.update(state, command);
    check(near(out.leg_kp_scale, config.leg_kp_scale + config.landing_kp_boost) &&
          near(out.leg_kd_scale, config.leg_kd_scale + config.landing_kd_boost) &&
          near(mpc.debugState().landing_window_remaining_s, config.landing_window_s),
          "touchdown edge opens the landing window at its full configured length");

    for (int i = 0; i < 4; ++i) out = mpc.update(state, command);  // 0.08s elapsed
    check(out.leg_kp_scale > config.leg_kp_scale &&
          mpc.debugState().landing_window_remaining_s > 0.0F,
          "landing boost still active partway through the window");

    for (int i = 0; i < 3; ++i) out = mpc.update(state, command);  // >0.1s elapsed
    check(near(out.leg_kp_scale, config.leg_kp_scale) &&
          near(out.leg_kd_scale, config.leg_kd_scale) &&
          mpc.debugState().landing_window_remaining_s == 0.0F,
          "landing boost expires once the window elapses");
  }

  // Reaching Balance already grounded (reset() default) is not itself an
  // edge; a second liftoff/touchdown cycle re-opens the window from scratch.
  {
    MpcBalancer::Config config;
    config.landing_window_s = 0.1F;
    config.landing_kp_boost = 0.4F;
    MpcBalancer mpc(config);
    RobotState state;
    ControlCommand command;
    state.dt_s = 0.02F;
    state.floor_contact = true;
    (void)mpc.update(state, command);
    check(near(mpc.debugState().landing_window_remaining_s, 0.0F),
          "starting already grounded is not itself a touchdown edge");

    state.floor_contact = false;
    for (int i = 0; i < 10; ++i) (void)mpc.update(state, command);
    state.floor_contact = true;
    const auto out = mpc.update(state, command);
    check(near(out.leg_kp_scale, config.leg_kp_scale + config.landing_kp_boost),
          "a second touchdown after going airborne again reopens the window");
  }

  // Ground-accel authority boost: under a saturating pitch disturbance, the
  // boosted window commands a larger-magnitude acceleration than the
  // unboosted baseline's own configured limit allows.
  {
    MpcBalancer::Config base_config;
    base_config.max_ground_accel_m_s2 = 1.0F;
    MpcBalancer baseline(base_config);
    MpcBalancer::Config boosted_config = base_config;
    boosted_config.landing_window_s = 0.5F;
    boosted_config.landing_ground_accel_boost_m_s2 = 4.0F;
    MpcBalancer boosted(boosted_config);
    RobotState state;
    ControlCommand command;
    state.dt_s = 0.02F;
    state.imu.pitch_rad = 0.5F;
    state.imu.pitch_rate_rad_s = 3.0F;
    state.floor_contact = false;
    (void)baseline.update(state, command);
    (void)boosted.update(state, command);
    state.floor_contact = true;
    (void)baseline.update(state, command);
    (void)boosted.update(state, command);
    check(std::abs(baseline.debugState().optimal_command_m_s) <=
          base_config.max_ground_accel_m_s2 + 1e-3F,
          "baseline stays within its unboosted accel limit");
    check(std::abs(boosted.debugState().optimal_command_m_s) >
          std::abs(baseline.debugState().optimal_command_m_s) + 0.5F,
          "landing ground-accel boost raises wheel authority during the window");
  }
}
void controllerTests() {
  EmbeddedController::Config config;
  config.state_machine.enable_cycles = 2;
  config.stand_up.tuck_duration_s = .02F;
  config.stand_up.rise_duration_s = .02F;
  config.stand_up.settle_cycles = 3;
  config.stand_up.max_settle_duration_s = .08F;
  EmbeddedController controller(config);
  RobotState state;
  state.dt_s = .01F;
  state.joints.position_rad = config.stand_up.stand_pose_rad;
  ControlCommand command;
  command.target_joint_position_rad = state.joints.position_rad;
  controller.reset(state);
  check(stopped(controller.update(state, command)), "disabled controller is limp");
  command.request_enable = true;
  auto out = controller.update(state, command);
  command.request_enable = false;
  check(controller.mode() == ControlMode::Enabling && out.enable_motors, "explicit enable starts posture hold");
  for (int i=0; i<3; ++i) out = controller.update(state, command);
  check(controller.mode() == ControlMode::Balance, "enable transition to balance");
  command.target_forward_velocity_m_s = .1F;
  out = controller.update(state, command);
  check(controller.mode() == ControlMode::Drive, "drive transition");
  command.target_forward_velocity_m_s = 0;
  command.request_pose_change = true;
  command.target_joint_position_rad = config.stand_up.tuck_pose_rad;
  out = controller.update(state, command);
  check(controller.mode() == ControlMode::PoseChange && out.position_rad[0] < .12F,
        "pose begins from measured joints with interpolation");
  command.request_pose_change = false;
  state.emergency_stop = true;
  out = controller.update(state, command);
  check(controller.mode() == ControlMode::EmergencyStop && stopped(out), "estop interrupts pose and clears actuation");
  state.emergency_stop = false;
  state.imu.pitch_rad = 1.4F;
  command.request_stand_up = true;
  out = controller.update(state, command);
  check(controller.mode() == ControlMode::EmergencyStop && stopped(out), "estop latch not bypassed by fall or standup");
  command.request_stand_up = false;
  state.imu.pitch_rad = 0;
  command.request_enable = true;
  for (int i=0; i<4; ++i) out = controller.update(state, command);
  command.request_enable = false;
  state.imu.pitch_rad = .9F;
  out = controller.update(state, command);
  check(controller.mode() == ControlMode::Fallen && stopped(out), "fall disables motors");
  command.request_stand_up = true;
  out = controller.update(state, command);
  check(controller.mode() == ControlMode::StandingUp && out.enable_motors,
        "explicit standup accepted from Fallen");
  // Leave pitch fixed: synthetic trajectory cannot pretend this robot rose.
  command.request_stand_up = false;
  for (int i=0; i<30; ++i) out = controller.update(state, command);
  check(controller.mode() == ControlMode::Fallen && stopped(out), "standup timeout is failure, never false success");
  controller.reset(state);
  command.request_stand_up = true;
  out = controller.update(state, command);
  check(controller.mode() == ControlMode::StandingUp, "standup accepted from Disabled");
  // Synthetic feedback only, deliberately not a physical recovery assertion.
  state.imu.pitch_rad = 0;
  for (int i=0; i<30; ++i) {
    state.joints.position_rad = out.position_rad;
    out = controller.update(state, command);
  }
  check(controller.mode() == ControlMode::Balance && !controller.standingUp(),
        "settled recovery hands off even if request remains high");
  command.request_stand_up = false;
  command.target_joint_position_rad = {5, -5, 0, 5, -5, 0};
  out = controller.update(state, command);
  check(near(out.position_rad[0], config.hip_max_rad) && near(out.position_rad[1], config.knee_min_rad),
        "provisional native leg command envelope enforced");
  command.target_joint_position_rad[0] = std::numeric_limits<float>::quiet_NaN();
  out = controller.update(state, command);
  check(stopped(out) && controller.mode() == ControlMode::EmergencyStop,
        "invalid command latches safe stop instead of NaN actuation");
}
void safetyRegressionTests() {
  EmbeddedController::Config config;
  config.state_machine.enable_cycles = 1;
  config.balance_pitch_ref_period_s = 0;
  EmbeddedController controller(config);
  RobotState state;
  state.joints.position_rad = config.stand_up.stand_pose_rad;
  state.joints.position_rad[0] += .2F;
  state.joints.position_rad[3] += .2F;
  ControlCommand command;
  command.target_joint_position_rad = state.joints.position_rad;
  controller.reset(state);
  command.request_enable = true;
  auto out = controller.update(state, command);
  command.request_enable = false;
  out = controller.update(state, command);
  check(controller.mode() == ControlMode::Balance && controller.balancePitchRef() < -.05F,
        "relative-fall regression starts balanced with negative pitch reference");
  state.imu.pitch_rad = config.mpc.fall_pitch_rad-.01F;
  out = controller.update(state, command);
  check(controller.mode() == ControlMode::Fallen && stopped(out),
        "MPC-only relative pitch fall latches Fallen");
  state.imu.pitch_rad = 0;
  for (int i=0; i<5; ++i) out = controller.update(state, command);
  check(controller.mode() == ControlMode::Fallen && stopped(out),
        "relative fall stays disabled after pitch recovers without enable");
  command.request_enable = true;
  out = controller.update(state, command);
  check(controller.mode() == ControlMode::Enabling && out.enable_motors,
        "relative fall permits explicit re-enable");
  command.request_enable = false;
  const JointArray valid_positions = state.joints.position_rad;
  for (const float invalid : {std::numeric_limits<float>::quiet_NaN(),
                              std::numeric_limits<float>::infinity(),
                              -std::numeric_limits<float>::infinity()}) {
    for (std::size_t joint=0; joint<kJointCount; ++joint) {
      state.joints.position_rad = valid_positions;
      state.joints.position_rad[joint] = invalid;
      out = controller.update(state, command);
      bool finite = stopped(out) && controller.mode() == ControlMode::EmergencyStop;
      for (std::size_t i=0; i<kJointCount; ++i) {
        finite &= std::isfinite(out.position_rad[i]) && std::isfinite(out.velocity_rad_s[i]) &&
            std::isfinite(out.torque_nm[i]) && std::isfinite(out.kp_scale[i]) &&
            std::isfinite(out.kd_scale[i]);
        if (i != joint) finite &= out.position_rad[i] == valid_positions[i];
      }
      check(finite, "nonfinite encoder yields finite disabled output and preserves valid positions");
    }
  }
  state.joints.position_rad = valid_positions;
  out = controller.update(state, command);
  check(stopped(out) && controller.mode() == ControlMode::EmergencyStop,
        "encoder recovery does not clear emergency latch");
}
}
int main() {
  motionTests(); geometryTests(); mpcTests(); landingWindowTests(); controllerTests(); safetyRegressionTests();
  return failures ? 1 : 0;
}
