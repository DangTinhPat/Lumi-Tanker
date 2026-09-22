#include <cmath>
#include <iostream>

#include "robot_control/EmbeddedController.hpp"
#include "robot_control/LandingController.hpp"
#include "robot_control/WholeBodyModel.hpp"

using namespace robot_control;
namespace {
int failures = 0;
void check(bool condition, const char* name) {
  std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
  if (!condition) ++failures;
}
bool near(float a, float b, float eps = 1.0e-5F) {
  return std::abs(a - b) < eps;
}

RobotState baseState() {
  RobotState state;
  state.dt_s = 0.01F;
  state.floor_contact = true;
  state.left_floor_contact = true;
  state.right_floor_contact = true;
  state.imu.accel_magnitude_m_s2 = 9.81F;
  state.joints.position_rad = {0.9F, -1.3962634F, 0.0F,
                               0.9F, -1.3962634F, 0.0F};
  return state;
}

// Explicitly model the observer output in this unit fixture. Production code
// never uses RobotState contact fields; LandingController receives the
// sensor-derived FlightObserver output from EmbeddedController.
FlightObserver::Output fixtureFlight(const RobotState& state) {
  FlightObserver::Output out;
  out.support_contact = state.floor_contact;
  out.left_support_contact = state.left_floor_contact;
  out.right_support_contact = state.right_floor_contact;
  out.airborne = !out.support_contact;
  out.phase = out.support_contact ? FlightObserver::Phase::Grounded
                                  : FlightObserver::Phase::BallisticFlight;
  return out;
}

LandingController::Output sensorUpdate(LandingController& ctrl,
                                       const RobotState& state,
                                       const JointArray& target) {
  auto flight = fixtureFlight(state);
  return ctrl.update(state, target, &flight);
}

void outputDefaultsAreNeutral() {
  LandingController::Output out;
  bool ok = true;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    ok = ok && out.kp_scale[i] == 1.0F && out.kd_scale[i] == 1.0F &&
         out.position_offset_rad[i] == 0.0F && out.feedforward_nm[i] == 0.0F;
  }
  check(ok, "LandingController: default Output is neutral, not zero-stiffness");
}

void disabledControllerIsNoOp() {
  LandingController ctrl;
  RobotState state = baseState();
  JointArray target = state.joints.position_rad;
  state.floor_contact = false;
  const auto out = sensorUpdate(ctrl, state, target);
  bool ok = !out.active && out.phase == LandingController::Phase::Grounded;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    ok = ok && out.kp_scale[i] == 1.0F && out.kd_scale[i] == 1.0F &&
         out.position_offset_rad[i] == 0.0F && out.feedforward_nm[i] == 0.0F;
  }
  check(ok, "LandingController: disabled config is an exact no-op");
}

void flightDelaysAndBoundsLegExtension() {
  LandingController::Config cfg;
  cfg.enabled = true;
  cfg.flight_extension_delay_s = 0.05F;
  cfg.flight_extension_max_m = 0.02F;
  cfg.flight_hip_offset_max_rad = 0.015F;
  cfg.flight_knee_offset_max_rad = 0.08F;
  cfg.flight_target_slew_rate_rad_s = 100.0F;
  LandingController ctrl(cfg);
  RobotState state = baseState();
  JointArray target = state.joints.position_rad;

  (void)sensorUpdate(ctrl, state, target);
  state.floor_contact = false;
  auto early = sensorUpdate(ctrl, state, target);
  check(early.active && early.suspend_wheel_balance &&
            near(early.position_offset_rad[static_cast<std::size_t>(JointId::LeftKnee)], 0.0F),
        "LandingController: Flight starts but extension waits for edge-clear delay");

  for (int i = 0; i < 20; ++i) {
    early = sensorUpdate(ctrl, state, target);
  }
  const float hip = early.position_offset_rad[static_cast<std::size_t>(JointId::LeftHip)];
  const float knee = early.position_offset_rad[static_cast<std::size_t>(JointId::LeftKnee)];
  check(knee >= 0.0F && knee <= cfg.flight_knee_offset_max_rad + 1.0e-5F &&
            std::abs(hip) <= cfg.flight_hip_offset_max_rad + 1.0e-5F,
        "LandingController: Flight extension is bounded at hip and knee");
}

void flightPitchRateShapesHipPlacement() {
  LandingController::Config cfg;
  cfg.enabled = true;
  cfg.flight_extension_delay_s = 10.0F;
  cfg.flight_target_slew_rate_rad_s = 100.0F;
  cfg.flight_pitch_rate_hip_gain_rad_per_rad_s = 0.02F;
  cfg.flight_pitch_rate_hip_offset_max_rad = 0.05F;
  LandingController ctrl(cfg);
  RobotState state = baseState();
  JointArray target = state.joints.position_rad;

  (void)sensorUpdate(ctrl, state, target);
  state.floor_contact = false;
  state.left_floor_contact = false;
  state.right_floor_contact = false;
  state.imu.pitch_rate_rad_s = 10.0F;
  const auto out = sensorUpdate(ctrl, state, target);
  const float hip =
      out.position_offset_rad[static_cast<std::size_t>(JointId::LeftHip)];
  check(out.phase == LandingController::Phase::Flight && hip < 0.0F &&
            std::abs(hip) <= cfg.flight_pitch_rate_hip_offset_max_rad + 1.0e-5F,
        "LandingController: Flight pitch-rate shaping moves hip negative and bounded");
}

void holdsActiveExtensionThroughImpact() {
  LandingController::Config cfg;
  cfg.enabled = true;
  cfg.flight_extension_delay_s = 0.0F;
  cfg.flight_extension_max_m = 0.035F;
  cfg.flight_knee_offset_max_rad = 0.20F;
  cfg.flight_target_slew_rate_rad_s = 100.0F;
  cfg.min_flight_before_impact_s = 0.02F;
  cfg.min_impact_s = 0.01F;
  cfg.max_impact_s = 0.05F;
  cfg.recovery_blend_s = 0.10F;
  LandingController ctrl(cfg);
  RobotState state = baseState();
  JointArray target = state.joints.position_rad;

  (void)sensorUpdate(ctrl, state, target);
  state.floor_contact = false;
  state.left_floor_contact = false;
  state.right_floor_contact = false;
  for (int i = 0; i < 5; ++i) {
    (void)sensorUpdate(ctrl, state, target);
  }
  const auto flight = sensorUpdate(ctrl, state, target);
  const float flight_knee =
      flight.position_offset_rad[static_cast<std::size_t>(JointId::LeftKnee)];
  check(flight.phase == LandingController::Phase::Flight && flight_knee > 0.0F,
        "LandingController: active extension is present before touchdown");

  state.floor_contact = true;
  state.left_floor_contact = true;
  state.right_floor_contact = true;
  auto impact = sensorUpdate(ctrl, state, target);
  const float impact_knee =
      impact.position_offset_rad[static_cast<std::size_t>(JointId::LeftKnee)];
  check(impact.phase == LandingController::Phase::Impact &&
            near(impact_knee, flight_knee, 1.0e-4F),
        "LandingController: extension is held through Impact for compression stroke");

  state.joints.velocity_rad_s.fill(0.0F);
  LandingController::Output recovery;
  for (int i = 0; i < 6; ++i) {
    recovery = sensorUpdate(ctrl, state, target);
  }
  const float recovery_knee =
      recovery.position_offset_rad[static_cast<std::size_t>(JointId::LeftKnee)];
  check(recovery.phase == LandingController::Phase::Recovery &&
            recovery_knee < impact_knee && recovery_knee > 0.0F,
        "LandingController: extension blends out after impact, not at touchdown");
}

void impactProducesCartesianFeedforwardAndRecovery() {
  LandingController::Config cfg;
  cfg.enabled = true;
  cfg.min_flight_before_impact_s = 0.02F;
  cfg.height_kd_n_s_m = 20.0F;
  cfg.max_virtual_force_n = 10.0F;
  cfg.max_feedforward_torque_nm = 0.5F;
  cfg.impact_kp_scale = 0.2F;
  cfg.min_impact_s = 0.01F;
  cfg.max_impact_s = 0.05F;
  cfg.recovery_blend_s = 0.05F;
  LandingController ctrl(cfg);
  RobotState state = baseState();
  JointArray target = state.joints.position_rad;

  (void)sensorUpdate(ctrl, state, target);
  state.floor_contact = false;
  for (int i = 0; i < 4; ++i) {
    (void)sensorUpdate(ctrl, state, target);
  }
  state.floor_contact = true;
  state.imu.accel_magnitude_m_s2 = 30.0F;
  state.imu.pitch_rate_rad_s = 2.0F;
  state.joints.velocity_rad_s[static_cast<std::size_t>(JointId::LeftKnee)] = -4.0F;
  state.joints.velocity_rad_s[static_cast<std::size_t>(JointId::RightKnee)] = -4.0F;
  const auto impact = sensorUpdate(ctrl, state, target);
  bool has_tau = false;
  for (float tau : impact.feedforward_nm) {
    has_tau = has_tau || std::abs(tau) > 1.0e-5F;
    check(std::abs(tau) <= cfg.max_feedforward_torque_nm + 1.0e-5F,
          "LandingController: impact feedforward respects torque clamp");
  }
  check(impact.phase == LandingController::Phase::Impact && has_tau &&
            !impact.suspend_wheel_balance &&
            near(impact.kp_scale[static_cast<std::size_t>(JointId::LeftKnee)],
                 cfg.impact_kp_scale),
        "LandingController: contact Impact uses leg damping while retaining wheel authority");

  state.imu.pitch_rate_rad_s = 0.0F;
  state.imu.accel_magnitude_m_s2 = 9.81F;
  state.joints.velocity_rad_s.fill(0.0F);
  LandingController::Output out;
  for (int i = 0; i < 20; ++i) {
    out = sensorUpdate(ctrl, state, target);
  }
  check(out.phase == LandingController::Phase::Grounded && !out.active,
        "LandingController: Impact/Recovery returns to Grounded");
}

void hardImuSpikeWithoutContactDoesNotEnterImpact() {
  LandingController::Config cfg;
  cfg.enabled = true;
  cfg.min_flight_before_impact_s = 0.02F;
  cfg.height_kd_n_s_m = 20.0F;
  LandingController ctrl(cfg);
  RobotState state = baseState();
  JointArray target = state.joints.position_rad;

  (void)sensorUpdate(ctrl, state, target);
  state.floor_contact = false;
  state.left_floor_contact = false;
  state.right_floor_contact = false;
  for (int i = 0; i < 4; ++i) {
    (void)sensorUpdate(ctrl, state, target);
  }
  state.imu.accel_magnitude_m_s2 = 100.0F;
  state.imu.pitch_rate_rad_s = 8.0F;
  const auto out = sensorUpdate(ctrl, state, target);
  check(out.phase == LandingController::Phase::Flight &&
            out.suspend_wheel_balance,
        "LandingController: single-tick IMU spike without contact stays Flight");
}

void repeatedHardImuShockWithoutWheelContactStaysFlight() {
  LandingController::Config cfg;
  cfg.enabled = true;
  cfg.min_flight_before_impact_s = 0.02F;
  cfg.height_kd_n_s_m = 20.0F;
  cfg.max_feedforward_torque_nm = 0.5F;
  LandingController ctrl(cfg);
  RobotState state = baseState();
  JointArray target = state.joints.position_rad;

  (void)sensorUpdate(ctrl, state, target);
  state.floor_contact = false;
  state.left_floor_contact = false;
  state.right_floor_contact = false;
  for (int i = 0; i < 4; ++i) {
    (void)sensorUpdate(ctrl, state, target);
  }
  state.imu.accel_magnitude_m_s2 = 100.0F;
  state.imu.pitch_rate_rad_s = 8.0F;
  state.joints.velocity_rad_s[static_cast<std::size_t>(JointId::LeftKnee)] = -4.0F;
  state.joints.velocity_rad_s[static_cast<std::size_t>(JointId::RightKnee)] = -4.0F;
  (void)sensorUpdate(ctrl, state, target);
  const auto out = sensorUpdate(ctrl, state, target);
  check(out.phase == LandingController::Phase::Flight && out.suspend_wheel_balance &&
            out.feedforward_nm[static_cast<std::size_t>(JointId::LeftKnee)] == 0.0F &&
            out.feedforward_nm[static_cast<std::size_t>(JointId::RightKnee)] == 0.0F,
        "LandingController: repeated hard IMU shock never creates a ground phase without contact");
}

void observerNoSupportTimeoutIsNotGroundContact() {
  LandingController::Config cfg;
  cfg.enabled = true;
  LandingController ctrl(cfg);
  RobotState state = baseState();
  JointArray target = state.joints.position_rad;
  (void)sensorUpdate(ctrl, state, target);

  state.floor_contact = false;
  state.left_floor_contact = false;
  state.right_floor_contact = false;
  FlightObserver::Output timeout;
  timeout.phase = FlightObserver::Phase::NoSupportTimeout;
  timeout.airborne = false;
  timeout.support_lost = true;
  const auto out = ctrl.update(state, target, &timeout);
  check(out.phase == LandingController::Phase::Flight &&
            out.suspend_wheel_balance &&
            out.feedforward_nm[static_cast<std::size_t>(JointId::LeftKnee)] ==
                0.0F,
        "LandingController: NoSupportTimeout never masquerades as confirmed support");
}

void touchdownCandidateWaitsForStableSupport() {
  LandingController::Config cfg;
  cfg.enabled = true;
  cfg.min_flight_before_impact_s = 0.01F;
  LandingController ctrl(cfg);
  RobotState state = baseState();
  JointArray target = state.joints.position_rad;
  (void)sensorUpdate(ctrl, state, target);

  state.floor_contact = false;
  state.left_floor_contact = false;
  state.right_floor_contact = false;
  FlightObserver::Output flight;
  flight.phase = FlightObserver::Phase::BallisticFlight;
  flight.airborne = true;
  (void)ctrl.update(state, target, &flight);

  state.floor_contact = true;
  state.left_floor_contact = true;
  state.right_floor_contact = true;
  flight.phase = FlightObserver::Phase::TouchdownCandidate;
  flight.airborne = true;
  const auto candidate = ctrl.update(state, target, &flight);
  check(candidate.phase == LandingController::Phase::Flight &&
            candidate.suspend_wheel_balance,
        "LandingController: TouchdownCandidate cannot trigger Impact early");

  flight.phase = FlightObserver::Phase::Grounded;
  flight.airborne = false;
  const auto grounded = ctrl.update(state, target, &flight);
  check(grounded.phase == LandingController::Phase::Impact &&
            !grounded.suspend_wheel_balance,
        "LandingController: stable Grounded phase triggers Impact");
}

void hardImuShockFollowedByContactEntersImpact() {
  LandingController::Config cfg;
  cfg.enabled = true;
  cfg.min_flight_before_impact_s = 0.02F;
  cfg.height_kd_n_s_m = 20.0F;
  cfg.max_feedforward_torque_nm = 0.5F;
  LandingController ctrl(cfg);
  RobotState state = baseState();
  JointArray target = state.joints.position_rad;

  (void)sensorUpdate(ctrl, state, target);
  state.floor_contact = false;
  state.left_floor_contact = false;
  state.right_floor_contact = false;
  for (int i = 0; i < 5; ++i) {
    state.imu.accel_magnitude_m_s2 = 100.0F;
    state.imu.pitch_rate_rad_s = 8.0F;
    (void)sensorUpdate(ctrl, state, target);
  }

  state.floor_contact = true;
  state.left_floor_contact = true;
  state.right_floor_contact = true;
  state.imu.hard_impact_event = true;
  const auto out = sensorUpdate(ctrl, state, target);
  check(out.phase == LandingController::Phase::Impact &&
            !out.suspend_wheel_balance,
        "LandingController: a later validated contact creates Impact after an IMU shock");
}

void impactFeedforwardOnlyOnContactedSide() {
  LandingController::Config cfg;
  cfg.enabled = true;
  cfg.min_flight_before_impact_s = 0.02F;
  cfg.height_kd_n_s_m = 20.0F;
  cfg.max_feedforward_torque_nm = 0.5F;
  LandingController ctrl(cfg);
  RobotState state = baseState();
  JointArray target = state.joints.position_rad;

  (void)sensorUpdate(ctrl, state, target);
  state.floor_contact = false;
  state.left_floor_contact = false;
  state.right_floor_contact = false;
  for (int i = 0; i < 4; ++i) {
    (void)sensorUpdate(ctrl, state, target);
  }
  state.floor_contact = true;
  state.left_floor_contact = true;
  state.right_floor_contact = false;
  state.joints.velocity_rad_s[static_cast<std::size_t>(JointId::LeftKnee)] = -4.0F;
  state.joints.velocity_rad_s[static_cast<std::size_t>(JointId::RightKnee)] = -4.0F;
  const auto out = sensorUpdate(ctrl, state, target);
  check(out.phase == LandingController::Phase::Impact &&
            std::abs(out.feedforward_nm[static_cast<std::size_t>(JointId::LeftKnee)]) > 1.0e-5F &&
            out.feedforward_nm[static_cast<std::size_t>(JointId::RightHip)] == 0.0F &&
            out.feedforward_nm[static_cast<std::size_t>(JointId::RightKnee)] == 0.0F,
        "LandingController: Impact feedforward applies only to contacted side");
}

void recoveryDriveHandoffIsBoundedAndRamps() {
  LandingController::Config cfg;
  cfg.enabled = true;
  cfg.min_flight_before_impact_s = 0.01F;
  cfg.min_impact_s = 0.01F;
  cfg.max_impact_s = 0.02F;
  cfg.recovery_blend_s = 0.01F;
  cfg.recovery_drive_ramp_s = 0.10F;
  LandingController ctrl(cfg);
  RobotState state = baseState();
  JointArray target = state.joints.position_rad;
  (void)sensorUpdate(ctrl, state, target);
  state.floor_contact = false;
  state.left_floor_contact = false;
  state.right_floor_contact = false;
  (void)sensorUpdate(ctrl, state, target);
  state.floor_contact = true;
  state.left_floor_contact = true;
  state.right_floor_contact = true;
  auto out = sensorUpdate(ctrl, state, target);
  check(out.drive_velocity_scale == 0.0F,
        "LandingController: drive handoff blocks command during Impact");

  state.imu.pitch_rate_rad_s = 0.0F;
  state.joints.velocity_rad_s.fill(0.0F);
  for (int i = 0; i < 2; ++i) {
    out = sensorUpdate(ctrl, state, target);
  }
  check(out.phase == LandingController::Phase::Grounded &&
            out.drive_velocity_scale > 0.0F &&
            out.drive_velocity_scale < 1.0F,
        "LandingController: drive handoff ramps after Grounded");
  for (int i = 0; i < 20; ++i) {
    out = sensorUpdate(ctrl, state, target);
  }
  check(out.drive_velocity_scale == 1.0F,
        "LandingController: drive handoff returns full command");
}

void recoveryDriveWaitsForSensorStableBody() {
  LandingController::Config cfg;
  cfg.enabled = true;
  cfg.min_flight_before_impact_s = 0.01F;
  cfg.min_impact_s = 0.01F;
  cfg.max_impact_s = 0.02F;
  cfg.recovery_blend_s = 0.01F;
  cfg.recovery_drive_ramp_s = 0.10F;
  cfg.recovery_settle_pitch_rad = 0.20F;
  cfg.recovery_settle_pitch_rate_rad_s = 0.50F;
  cfg.recovery_settle_hold_s = 0.03F;
  LandingController ctrl(cfg);
  RobotState state = baseState();
  JointArray target = state.joints.position_rad;

  (void)sensorUpdate(ctrl, state, target);
  state.floor_contact = false;
  state.left_floor_contact = false;
  state.right_floor_contact = false;
  (void)sensorUpdate(ctrl, state, target);
  state.floor_contact = true;
  state.left_floor_contact = true;
  state.right_floor_contact = true;
  state.imu.pitch_rad = 0.30F;
  state.imu.pitch_rate_rad_s = 0.0F;
  auto out = sensorUpdate(ctrl, state, target);
  for (int i = 0; i < 5; ++i) {
    out = sensorUpdate(ctrl, state, target);
  }
  check(out.phase == LandingController::Phase::Grounded &&
            out.drive_velocity_scale == 0.0F,
        "LandingController: drive remains held while body is outside settle window");

  state.imu.pitch_rad = 0.10F;
  for (int i = 0; i < 3; ++i) {
    out = sensorUpdate(ctrl, state, target);
  }
  check(out.drive_velocity_scale > 0.0F && out.drive_velocity_scale < 1.0F,
        "LandingController: drive ramp starts only after sensor-stable dwell");
}

void transientUnloadingKeepsWheelBalanceAlive() {
  LandingController::Config cfg;
  cfg.enabled = true;
  LandingController controller(cfg);
  RobotState state = baseState();
  const JointArray target = state.joints.position_rad;

  FlightObserver::Output grounded;
  grounded.phase = FlightObserver::Phase::Grounded;
  grounded.support_contact = true;
  (void)controller.update(state, target, &grounded);

  FlightObserver::Output transient;
  transient.phase = FlightObserver::Phase::TransientUnloading;
  transient.support_contact = false;
  transient.support_unloading = true;
  transient.airborne = false;
  auto out = controller.update(state, target, &transient);
  check(out.phase == LandingController::Phase::Flight &&
            !out.suspend_wheel_balance,
        "LandingController: unconfirmed support loss does not cut wheel MPC");

  transient.airborne = true;
  transient.phase = FlightObserver::Phase::BallisticFlight;
  out = controller.update(state, target, &transient);
  check(out.suspend_wheel_balance,
        "LandingController: confirmed airborne state suspends wheel MPC");
}

void embeddedControllerDefaultLandingControllerIsNoOp() {
  EmbeddedController controller;
  RobotState state = baseState();
  ControlCommand command;
  command.request_enable = true;
  command.target_joint_position_rad = state.joints.position_rad;
  controller.reset(state);
  MotorCommand grounded;
  for (int i = 0; i < 200; ++i) {
    state.floor_contact = true;
    grounded = controller.update(state, command);
  }
  EmbeddedController controller_b;
  controller_b.reset(state);
  MotorCommand airborne;
  for (int i = 0; i < 200; ++i) {
    state.floor_contact = (i % 2) == 0;
    airborne = controller_b.update(state, command);
  }
  bool same = grounded.enable_motors == airborne.enable_motors;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    same = same && grounded.kp_scale[i] == airborne.kp_scale[i] &&
           grounded.kd_scale[i] == airborne.kd_scale[i] &&
           grounded.feedforward_nm[i] == airborne.feedforward_nm[i];
  }
  check(same, "EmbeddedController: default LandingController wiring is no-op");
}
}  // namespace

int main() {
  outputDefaultsAreNeutral();
  disabledControllerIsNoOp();
  flightDelaysAndBoundsLegExtension();
  flightPitchRateShapesHipPlacement();
  holdsActiveExtensionThroughImpact();
  impactProducesCartesianFeedforwardAndRecovery();
  hardImuSpikeWithoutContactDoesNotEnterImpact();
  repeatedHardImuShockWithoutWheelContactStaysFlight();
  observerNoSupportTimeoutIsNotGroundContact();
  touchdownCandidateWaitsForStableSupport();
  hardImuShockFollowedByContactEntersImpact();
  impactFeedforwardOnlyOnContactedSide();
  recoveryDriveHandoffIsBoundedAndRamps();
  recoveryDriveWaitsForSensorStableBody();
  transientUnloadingKeepsWheelBalanceAlive();
  embeddedControllerDefaultLandingControllerIsNoOp();
  std::cout << (failures == 0 ? "ALL PASS\n" : "FAILURES PRESENT\n");
  return failures == 0 ? 0 : 1;
}
