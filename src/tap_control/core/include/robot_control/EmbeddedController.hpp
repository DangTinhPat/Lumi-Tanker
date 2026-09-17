#pragma once

#include "robot_control/JointMotionPlanner.hpp"
#include "robot_control/FlightObserver.hpp"
#include "robot_control/LandingController.hpp"
#include "robot_control/LandingImpedance.hpp"
#include "robot_control/LandingReflex.hpp"
#include "robot_control/MpcBalancer.hpp"
#include "robot_control/StateMachine.hpp"
#include "robot_control/TrajectoryObserver.hpp"
#include "robot_control/WholeBodyModel.hpp"

namespace robot_control {

class EmbeddedController {
 public:
  // Experimental Tuck -> Rise -> Settle sequence, not a validated physical
  // stand-up strategy. TAP knees cannot be mirrored fore/aft about CAD zero.
  struct StandUpConfig {
    JointArray tuck_pose_rad{0.75838357F, -1.2F, 0.0F, 0.75838357F, -1.2F, 0.0F};
    JointArray stand_pose_rad{geometry::neutral_hip_rad, 0.0F, 0.0F,
                             geometry::neutral_hip_rad, 0.0F, 0.0F};
    bool fall_direction_aware = false;  // compatibility only; no unvalidated mirroring
    float tuck_duration_s = 1.0F;
    float rise_duration_s = 1.6F;
    float fall_pitch_rad = 1.6F;
    float settle_pitch_rad = 0.20F;
    unsigned settle_cycles = 40;
    float max_settle_duration_s = 3.0F;
  };

  struct Config {
    MpcBalancer::Config mpc{};
    JointMotionPlanner::Config joint_motion{};
    StateMachine::Config state_machine{};
    StandUpConfig stand_up{};
    WholeBodyModel::Config whole_body{};
    // Reactive airborne/touchdown leg-target offset, see LandingReflex.hpp.
    // All gains default to 0 (inert / byte-identical to no reflex).
    LandingReflex::Config landing_reflex{};
    // Task-space stance-phase virtual spring-damper, see LandingImpedance.hpp.
    // All gains default to 0 (inert / byte-identical to no impedance).
    LandingImpedance::Config landing_impedance{};
    // Phase-structured landing controller, see LandingController.hpp.
    // Disabled by default.
    LandingController::Config landing_controller{};
    FlightObserver::Config flight_observer{};
    // Relative encoder/IMU trajectory observer. It is used by landing only
    // when the sample is valid and its signal-quality confidence is high
    // enough; otherwise landing falls back to the averaged wheel odometry.
    TrajectoryObserver::Config trajectory_observer{};
    float trajectory_observer_min_confidence = 0.50F;
    // Real-time kinematic compensation: feed the MPC the leaned base-pitch
    // target computed from the live CoM offset, so the legs can bend / the pose
    // can change while the wheels keep the CoM caught. `whole_body_pitch_gain`
    // scales it (0 disables, 1 = exact geometric equilibrium).
    bool whole_body_compensation = true;
    float whole_body_pitch_gain = 1.0F;
    float max_balance_pitch_ref_rad = 0.35F;
    // Slow integral trim on top of the feed-forward, to absorb leg sag / dynamic
    // load the FK model does not see. Gated on bent legs + settled Balance + a
    // dwell time (trim_dwell_s), inert at bring-up / during a pose transition,
    // decays back to zero as the legs straighten. OFF by default: on the
    // CoM-neutral squat the true equilibrium is already pitch = 0, so trimming
    // the pitch target only re-introduces a lean; it needs the settled-gate
    // reworked to tell a real equilibrium error from the residual creep before
    // it earns its keep. Left wired for that work.
    float whole_body_trim_gain_hz = 0.0F;
    float whole_body_trim_velocity_band = 0.35F;
    float max_whole_body_trim_rad = 0.18F;
    float whole_body_trim_dwell_s = 1.0F;
    // Low-pass the pitch reference (period, s) so an encoder glitch or a fast
    // pose command does not step-change the target.
    float balance_pitch_ref_period_s = 0.15F;
    float pose_change_duration_s = 1.0F;
    // Provisional simulator caps, NOT measured actuator ratings.
    float leg_max_torque_nm = 1.0F;
    float wheel_max_torque_nm = 0.3F;
    float hip_min_rad = -0.2F;
    float hip_max_rad = 1.0F;
    float knee_min_rad = -1.2F;
    float knee_max_rad = 0.0F;
    // 2026-09-16: MpcBalancer::Output::fallen means "the wheel-drive MPC's
    // OWN linear model no longer trusts a wheel command at this pitch" (see
    // MpcBalancer::Config::fall_pitch_rad) - a statement about the WHEELS
    // specifically. Before this flag, EmbeddedController::update() treated it
    // as "the whole robot has crashed": forceFallen() (ControlMode::Fallen)
    // plus writeSafeStop() zeroed torque/kp/kd on EVERY joint, LEGS INCLUDED,
    // and returned before the reflex/impedance code ever ran - so at exactly
    // the moment a real fall most needed the leg mechanisms (capture-point
    // hip placement, differential roll correction, active knee extension,
    // impedance), all of them were silently skipped and the legs went limp
    // too, even though none of them depend on wheel-ground friction and all
    // remain physically valid regardless of pitch. false (default) preserves
    // the exact prior behavior byte-for-byte; true keeps the legs (and their
    // reflex/impedance outputs) live while the wheels alone brake to a stop -
    // see EmbeddedController::update's own comment at the fallen branch.
    bool keep_legs_active_when_fallen = false;
    // Landing-specific recovery handoff. Disabled by default. When enabled
    // together with keep_legs_active_when_fallen, a settled post-impact Fallen
    // state may temporarily run wheel balance with a wider fall limit, then
    // return to normal Balance only after pitch/rate are inside the handoff
    // window. This is for controlled simulation recovery, not hardware bringup.
    bool landing_fallen_recovery_enabled = false;
    float landing_recovery_control_pitch_rad = 1.25F;
    float landing_recovery_handoff_pitch_rad = 0.25F;
    float landing_recovery_pitch_rate_rad_s = 0.6F;
    float landing_recovery_hold_s = 0.20F;
  };

  EmbeddedController();
  explicit EmbeddedController(Config config);

  void reset(const RobotState& state);
  [[nodiscard]] MotorCommand update(const RobotState& state,
                                    const ControlCommand& command);
  [[nodiscard]] ControlMode mode() const noexcept;
  [[nodiscard]] MpcBalancer::DebugState mpcDebugState() const noexcept;
  [[nodiscard]] WholeBodyModel::Result wholeBodyState() const noexcept {
    return last_whole_body_;
  }
  [[nodiscard]] float balancePitchRef() const noexcept {
    return balance_pitch_ref_rad_;
  }
  [[nodiscard]] LandingController::Phase landingPhase() const noexcept {
    return last_landing_output_.phase;
  }
  [[nodiscard]] FlightObserver::Output flightEstimate() const noexcept {
    return last_flight_estimate_;
  }
  [[nodiscard]] TrajectoryObserver::Output trajectoryEstimate() const noexcept {
    return last_trajectory_estimate_;
  }
  [[nodiscard]] float landingForwardVelocity() const noexcept {
    return last_landing_forward_velocity_m_s_;
  }
  [[nodiscard]] bool landingUsesTrajectoryObserver() const noexcept {
    return last_landing_uses_trajectory_observer_;
  }

  [[nodiscard]] bool standingUp() const noexcept;

 private:
  enum class StandUpStage { Idle, Tuck, Rise, Settle };

  void writeSafeStop(MotorCommand& output, const RobotState& state) const;
  void writePostureHold(MotorCommand& output,
                        const JointArray& joint_target) const;
  void writeBalancedCommand(MotorCommand& output,
                            const MpcBalancer::Output& balance,
                            const JointArray& joint_target,
                            const LandingImpedance::Output& impedance,
                            const LandingController::Output& landing) const;

  // Run / reset / configure the wheel-balance controller.
  [[nodiscard]] MpcBalancer::Output runBalance(const RobotState& state,
                                               const ControlCommand& command,
                                               bool integrate);
  // Whole-body equilibrium pitch from the current leg pose, low-passed.
  float updateBalancePitchRef(const RobotState& state,
                              const ControlCommand& command);
  void resetBalance(float ground_position_m, bool floor_contact = true);
  void setBalanceFallPitch(float fall_pitch_rad) noexcept;

  void beginStandUp(const RobotState& state);
  void advanceStandUp(const RobotState& state);
  void finishStandUp(const RobotState& state);
  [[nodiscard]] JointArray standUpPose(const JointArray& pose) const;

  Config config_;
  MpcBalancer mpc_;
  WholeBodyModel whole_body_;
  JointMotionPlanner joint_motion_;
  StateMachine state_machine_;
  LandingReflex landing_reflex_;
  LandingImpedance landing_impedance_;
  LandingController landing_controller_;
  FlightObserver flight_observer_;
  TrajectoryObserver trajectory_observer_;
  LandingController::Output last_landing_output_{};
  FlightObserver::Output last_flight_estimate_{};
  TrajectoryObserver::Output last_trajectory_estimate_{};
  WholeBodyModel::Result last_whole_body_{};
  float balance_pitch_ref_rad_ = 0.0F;
  float pitch_trim_rad_ = 0.0F;
  float trim_dwell_elapsed_s_ = 0.0F;  // consecutive settled time before trimming
  float station_dwell_s_ = 0.0F;       // quiet time before the position hold arms
  float landing_recovery_hold_elapsed_s_ = 0.0F;
  float last_landing_forward_velocity_m_s_ = 0.0F;
  bool last_landing_uses_trajectory_observer_ = false;
  // The wheel encoder velocity can collapse to zero in the first sample after
  // support is lost because the wheels are unloaded (and the simulated servo
  // may stop tracking).  Capture-point placement must use the last velocity
  // while the robot was still supported, not that post-liftoff zero.  This is
  // an event-local memory, cleared when balance is re-entered.
  float last_supported_forward_velocity_m_s_ = 0.0F;
  bool last_supported_trajectory_velocity_valid_ = false;
  StandUpStage stand_up_stage_ = StandUpStage::Idle;
  unsigned stand_up_settle_cycles_ = 0;
  float stand_up_settle_elapsed_s_ = 0.0F;
  float stand_up_fall_sign_ = 1.0F;
};

}  // namespace robot_control
