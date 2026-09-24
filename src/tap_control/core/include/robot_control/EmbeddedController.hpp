#pragma once

#include <array>

#include "robot_control/JointMotionPlanner.hpp"
#include "robot_control/FlightObserver.hpp"
#include "robot_control/LqrBalancer.hpp"
#include "robot_control/StateMachine.hpp"
#include "robot_control/WholeBodyModel.hpp"
#include "robot_control/WholeBodyStanceController.hpp"

namespace robot_control {

class EmbeddedController {
 public:
  // Experimental Tuck -> Rise -> Settle sequence, not a validated physical
  // stand-up strategy. TAP knees cannot be mirrored fore/aft about CAD zero.
  struct StandUpConfig {
    JointArray tuck_pose_rad{0.75838357F, -1.2F, 0.0F, 0.75838357F, -1.2F, 0.0F};
    // The stand-up endpoint must be the current CAD natural ENABLE pose.
    // A former robot used knee=0; on TAP that is an extended, non-neutral
    // configuration and shifts the COM during the Rise stage.
    JointArray stand_pose_rad{geometry::neutral_hip_rad,
                              geometry::neutral_knee_rad, 0.0F,
                              geometry::neutral_hip_rad,
                              geometry::neutral_knee_rad, 0.0F};
    bool fall_direction_aware = false;  // compatibility only; no unvalidated mirroring
    float tuck_duration_s = 1.0F;
    float rise_duration_s = 1.6F;
    float fall_pitch_rad = 1.6F;
    // Wheel capture is useful only while the body is near the supported
    // upright branch. A wide recovery fall limit must not authorize the MPC
    // to integrate a full wheel command while the robot is already lying.
    float wheel_balance_pitch_rad = 0.35F;
    float wheel_balance_pitch_rate_rad_s = 1.50F;
    float settle_pitch_rad = 0.20F;
    unsigned settle_cycles = 40;
    float max_settle_duration_s = 3.0F;
  };

  struct Config {
    LqrBalancer::Config lqr{};
    JointMotionPlanner::Config joint_motion{};
    StateMachine::Config state_machine{};
    StandUpConfig stand_up{};
    WholeBodyModel::Config whole_body{};
    FlightObserver::Config flight_observer{};
    WholeBodyStanceController::Config whole_body_stance{};
    // Real-time kinematic compensation: feed the wheel LQR the leaned base-pitch
    // target computed from the live CoM offset, so the legs can bend / the pose
    // can change while the wheels keep the CoM caught. `whole_body_pitch_gain`
    // scales it (0 disables, 1 = exact geometric equilibrium).
    bool whole_body_compensation = true;
    float whole_body_pitch_gain = 1.0F;
    // Natural ENABLE is deliberately allowed to use zero geometric pitch
    // feed-forward while a height transaction may still use the live CoM
    // equilibrium to catch the changing support geometry.
    bool height_whole_body_compensation = true;
    float height_whole_body_pitch_gain = 1.0F;
    // When stance WBC is enabled and the robot is stationary on the floor,
    // the legs own the quasi-static COM task. LQR remains responsible for
    // dynamic pitch capture, but must not create a permanent geometric lean
    // that the leg loop then treats as its own target.
    bool whole_body_stance_owns_static_com = true;
    bool whole_body_gravity_feedforward = false;
    float whole_body_gravity_feedforward_scale = 1.0F;
    float max_balance_pitch_ref_rad = 0.35F;
    // Slow integral trim on top of the feed-forward, to absorb leg sag / dynamic
    // load the FK model does not see. Gated on bent legs + settled Balance + a
    // dwell time (trim_dwell_s), inert at bring-up / during a pose transition,
    // decays back to zero as the legs straighten. OFF by default: on the
    // CoM-neutral squat the true equilibrium is already pitch = 0, so trimming
    // the pitch target only re-introduces a lean; it needs the settled-gate
    // reworked to tell a real equilibrium error from the residual creep before
    // it earns its keep. Left wired for that work.
    // Ceiling on how fast a leg POSITION command may change, rad/s. This is
    // an actuator-rate constraint, not a fix for any particular bug: a GDS68
    // cannot follow tens of rad/s, so a command faster than this is a
    // trajectory the hardware would never have produced, and emitting it
    // makes the simulation disagree with the robot. Applied at the single
    // output chokepoint so no upstream path can bypass it. Sized well above
    // real leg motion (a height path is ~1 rad/s, a jump extension a few
    // rad/s), so in correct operation it never binds. That is what makes it
    // useful as a DETECTOR as well as a guard: legCommandRateLimitedTicks()
    // counts every tick it actually clipped, so a discontinuity introduced
    // upstream is surfaced instead of being quietly absorbed. A guard that
    // only absorbs would let the next such bug through unnoticed.
    // 0 disables the limit.
    float leg_command_max_rate_rad_s = 0.0F;
    // Time over which an abandoned posture transaction hands the leg target
    // back to the standing command. Abandoning ownership is a handoff, not an
    // event: the core has been holding one pose and the command carries
    // another, so switching between them in a single tick is a step no
    // matter how small the gap. Blending over this window makes the handoff
    // continuous by construction instead of relying on a downstream limit to
    // absorb it. 0 restores the instantaneous switch.
    float posture_abort_blend_s = 0.0F;
    float whole_body_trim_gain_hz = 0.0F;
    float whole_body_trim_velocity_band = 0.35F;
    float max_whole_body_trim_rad = 0.18F;
    float whole_body_trim_dwell_s = 1.0F;
    // Station anchoring is gated by an adaptive capture-distance test.  The
    // margin is a physical localization tolerance, not a velocity threshold:
    // the required stopping/capture distance is recomputed from the live
    // pendulum length, wheel velocity, LQR velocity and IMU pitch state.
    float station_capture_margin_m = 0.025F;
    float station_capture_filter_period_s = 0.08F;
    float station_settle_dwell_s = 0.5F;
    float posture_reanchor_settle_s = 0.5F;
    // A posture request may arrive on the first sample after the prior path
    // retires.  Require the sensor state itself to be quiet for this dwell;
    // low wheel velocity alone is insufficient while the body is still
    // pitching through a leg-load transient.
    float posture_settle_dwell_s = 0.35F;
    float posture_settle_pitch_rate_rad_s = 0.12F;
    float posture_settle_pitch_error_rad = 0.015F;
    float posture_settle_wheel_velocity_m_s = 0.08F;
    // A height request may wait for a quiet capture state, but it must never
    // wait forever while the same station gate is disabled. Zero disables the
    // timeout for legacy callers; native runtime uses a finite value.
    float height_pending_timeout_s = 2.0F;
    // If a request has waited through the timeout but the measured state is
    // safe (supported, low wheel speed and low pitch rate), start it at a
    // reduced task-space rate instead of rejecting a transient station gate.
    // This is a bounded fallback, not permission to start while moving.
    float height_pending_fallback_rate_scale = 0.5F;
    // Give a transient IMU-rate spike time to clear after the fallback window;
    // a request still cannot remain pending indefinitely.
    float height_pending_abort_timeout_s = 6.0F;
    // Low-pass the pitch reference (period, s) so an encoder glitch or a fast
    // pose command does not step-change the target.
    float balance_pitch_ref_period_s = 0.15F;
    float pose_change_duration_s = 1.0F;
    // Optional feasibility layer for operator pose/height commands.  The
    // cubic planner has zero endpoint velocity, but its duration must still
    // scale with the largest joint displacement.  A non-positive velocity
    // leaves legacy callers on the fixed pose_change_duration_s.
    float pose_max_joint_velocity_rad_s = 0.0F;
    float pose_max_duration_s = 12.0F;
    float height_min_duration_s = 2.5F;
    float height_reached_tolerance_m = 0.0005F;
    float height_max_rate_m_s = 0.018F;
    float height_lowering_rate_m_s = 0.018F;
    float height_max_duration_s = 16.0F;
    // Adaptive task pacing: when the sensor-derived capture distance grows
    // during a height change, spend the next samples recovering balance
    // instead of continuing to move the legs into a larger disturbance.
  float height_capture_slowdown_min_scale = 0.12F;
    // Pause an active height path when capture distance leaves the safe
  // envelope; the same MPC station loop must recover before the legs move
  // again. Zero disables this transaction guard for legacy callers.
  float height_capture_pause_distance_m = 0.0F;
    // Do not hold a paused partial pose forever. A stalled transaction is
    // rejected and retried from the current safe actuator target after the
    // station loop has recovered.
    float height_capture_abort_s = 0.75F;
    // Wheel values follow the published CJC BL4820M12 operating envelope.
    // Leg values remain provisional because their motor rating is unresolved.
    float leg_max_torque_nm = 1.0F;
    float wheel_max_torque_nm = 0.13F;
    float hip_min_rad = -0.2F;
    float hip_max_rad = 1.0F;
    // The encoder knee zero is an interior mechanical reference, not a stop.
    // The lowered natural ENABLE pose is configured in GeneratedGeometry;
    // keep the controller envelope inside the native CAD range until the
    // physical 30..145 deg limits are mapped to this encoder coordinate.
    float knee_min_rad = -1.2F;
    float knee_max_rad = 0.45F;
    // 2026-09-16: LqrBalancer::Output::fallen means "the wheel-drive LQR's
    // OWN linear model no longer trusts a wheel command at this pitch" (see
    // LqrBalancer::Config::fall_pitch_rad) - a statement about the WHEELS
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
  };

  EmbeddedController();
  explicit EmbeddedController(Config config);

  void reset(const RobotState& state);
  [[nodiscard]] MotorCommand update(const RobotState& state,
                                    const ControlCommand& command);
  [[nodiscard]] ControlMode mode() const noexcept;
  // The native IPC layer retires a one-shot POSE request when the internal
  // interpolation has reached its endpoint.
  [[nodiscard]] bool poseMotionActive() const noexcept {
    return joint_motion_.active();
  }
  void requestStationaryStop(float ground_position_m,
                             bool floor_contact = true) noexcept;
  [[nodiscard]] bool heightMotionActive() const noexcept {
    return height_motion_active_;
  }
  [[nodiscard]] bool heightMotionSettling() const noexcept {
    return height_motion_endpoint_reached_ && !height_motion_settled_;
  }
  [[nodiscard]] bool heightMotionSettled() const noexcept {
    return height_motion_settled_;
  }
  [[nodiscard]] bool heightMotionRejected() const noexcept {
    return height_motion_rejected_;
  }
  [[nodiscard]] bool heightMotionPending() const noexcept {
    return height_motion_pending_;
  }
  [[nodiscard]] float heightPendingElapsedS() const noexcept {
    return height_pending_elapsed_s_;
  }
  [[nodiscard]] float heightMotionElapsedS() const noexcept {
    return height_motion_elapsed_s_;
  }
  [[nodiscard]] float heightMotionDurationS() const noexcept {
    return height_motion_duration_s_;
  }
  [[nodiscard]] float heightMotionStartM() const noexcept {
    return height_motion_start_m_;
  }
  [[nodiscard]] float heightMotionTargetM() const noexcept {
    return height_motion_target_m_;
  }
  [[nodiscard]] float fallPitchThresholdRad() const noexcept {
    return state_machine_.fallPitchRad();
  }
  [[nodiscard]] const LqrBalancer::DebugState& lqrDebugState() const noexcept;
  // Controller-side command snapshot for native diagnostics and embedded
  // bring-up. This is actuator intent, not simulator pose feedback.
  [[nodiscard]] const MotorCommand& lastOutput() const noexcept {
    return last_output_;
  }
  [[nodiscard]] float stationCaptureDistanceM() const noexcept {
    return station_capture_distance_m_;
  }
  [[nodiscard]] float stationCaptureMarginM() const noexcept {
    return config_.station_capture_margin_m;
  }
  [[nodiscard]] bool stationCaptureReady() const noexcept {
    return station_capture_ready_;
  }
  [[nodiscard]] float stationDwellS() const noexcept {
    return station_dwell_s_;
  }
  [[nodiscard]] bool stationEligible() const noexcept {
    return station_eligible_;
  }
  [[nodiscard]] bool stationHoldActive() const noexcept {
    return station_hold_active_;
  }
  [[nodiscard]] const WholeBodyModel::Result& wholeBodyState() const noexcept {
    return last_whole_body_;
  }
  [[nodiscard]] const WholeBodyStanceController::Output& wholeBodyStanceState()
      const noexcept {
    return last_whole_body_stance_;
  }
  [[nodiscard]] const JointArray& wholeBodyStanceCorrection() const noexcept {
    return whole_body_stance_.correction();
  }
  [[nodiscard]] float balancePitchRef() const noexcept {
    return balance_pitch_ref_rad_;
  }
  [[nodiscard]] const FlightObserver::Output& flightEstimate() const noexcept {
    return last_flight_estimate_;
  }
  [[nodiscard]] bool standingUp() const noexcept;

  // Ticks on which the leg-command rate limit actually clipped. Zero in

  // correct operation; any non-zero value is an upstream discontinuity.

  [[nodiscard]] unsigned legCommandRateLimitedTicks() const noexcept {

    return leg_command_rate_limited_ticks_;

  }

 private:
  enum class StandUpStage { Idle, Tuck, Rise, Settle };

  void writeSafeStop(MotorCommand& output, const RobotState& state) const;
  void writePostureHold(MotorCommand& output,
                        const JointArray& joint_target,
                        const RobotState& state) const;
  void writeBalancedCommand(MotorCommand& output,
                            const LqrBalancer::Output& balance,
                            const JointArray& joint_target) const;

  // Run / reset / configure the wheel-balance controller.
  [[nodiscard]] LqrBalancer::Output runBalance(const RobotState& state,
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
  LqrBalancer lqr_;
  WholeBodyModel whole_body_;
  WholeBodyStanceController whole_body_stance_;
  JointMotionPlanner joint_motion_;
  bool height_motion_active_ = false;
  bool height_motion_endpoint_reached_ = false;
  bool height_motion_settled_ = false;
  bool height_motion_rejected_ = false;
  bool height_capture_recovery_active_ = false;
  // A valid posture request may arrive during IMU/encoder settling.  Keep it
  // pending at the core boundary instead of starting from a moving capture
  // state or reporting a false feasibility rejection.
  bool height_motion_pending_ = false;
  bool height_pending_anchor_captured_ = false;
  float height_pending_elapsed_s_ = 0.0F;
  bool height_rejected_target_valid_ = false;
  float height_rejected_target_m_ = geometry::neutral_spawn_z_m;
  float height_motion_start_m_ = geometry::neutral_spawn_z_m;
  float height_motion_target_m_ = geometry::neutral_spawn_z_m;
  float height_motion_elapsed_s_ = 0.0F;
  float height_capture_pause_elapsed_s_ = 0.0F;
  float height_motion_duration_s_ = 0.0F;
  float height_motion_last_hip_rad_ = geometry::neutral_hip_rad;
  float height_motion_last_knee_rad_ = geometry::neutral_knee_rad;
  float height_motion_endpoint_anchor_m_ = 0.0F;
  JointArray height_pending_pose_{};
  // Shared by HEIGHT and legacy POSE transitions: the wheel anchor is
  // re-acquired at the actual endpoint instead of pulling back to the pose's
  // starting contact position.
  bool posture_motion_needs_station_reanchor_ = false;
  static constexpr std::size_t kHeightPathSamples = 65;
  std::array<JointArray, kHeightPathSamples> height_path_{};
  StateMachine state_machine_;
  FlightObserver flight_observer_;
  FlightObserver::Output last_flight_estimate_{};
  WholeBodyModel::Result last_whole_body_{};
  WholeBodyStanceController::Output last_whole_body_stance_{};
  float last_leg_velocity_error_m_s_ = 0.0F;
  unsigned leg_command_rate_limited_ticks_ = 0U;
  JointArray posture_abort_pose_{};
  float posture_abort_blend_ = 0.0F;
  JointArray whole_body_gravity_feedforward_{};
  MotorCommand last_output_{};
  std::uint64_t last_imu_sample_sequence_ = 0;
  std::uint64_t last_encoder_sample_sequence_ = 0;
  bool sensor_sequence_initialized_ = false;
  float balance_pitch_ref_rad_ = 0.0F;
  float pitch_trim_rad_ = 0.0F;
  float trim_dwell_elapsed_s_ = 0.0F;  // consecutive settled time before trimming
  float station_dwell_s_ = 0.0F;       // quiet time before the position hold arms
  float posture_settle_dwell_s_ = 0.0F;
  bool station_hold_active_ = false;
  float station_reanchor_cooldown_s_ = 0.0F;
  float stationary_stop_hold_s_ = 0.0F;
  bool yaw_only_handoff_done_ = false;
  bool station_eligible_ = false;
  float station_capture_distance_m_ = 0.0F;
  // Endpoint handoff checks pendulum motion separately from the station
  // position error; the normal station loop owns that return after handoff.
  float posture_capture_distance_m_ = 0.0F;
  float station_capture_distance_filtered_m_ = 0.0F;
  bool station_capture_ready_ = false;
  bool station_capture_filter_initialized_ = false;
  StandUpStage stand_up_stage_ = StandUpStage::Idle;
  unsigned stand_up_settle_cycles_ = 0;
  float stand_up_settle_elapsed_s_ = 0.0F;
  float stand_up_fall_sign_ = 1.0F;
};

}  // namespace robot_control
