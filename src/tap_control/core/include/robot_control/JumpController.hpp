#pragma once
#include "robot_control/Types.hpp"
#include "robot_control/JumpPlan.hpp"

namespace robot_control {
// Experimental measured-contact jump sequencer; no simulator or force APIs.
class JumpController {
 public:
  enum class Phase { Idle, Crouch, Thrust, Flight, Land, Settle, Complete, Fault };
  struct Config {
    bool enabled = false;
    float crouch_knee_rad = -1.2F;
    float extension_knee_rad = 0.0F;
    float landing_knee_rad = -0.6F;
    float crouch_duration_s = 2.0F;
    float thrust_duration_s = 0.18F;
    float max_thrust_s = 0.5F;
    float max_flight_s = 1.0F;
    // The controller has no absolute height sensor.  This is therefore a
    // ballistic design target used to derive the time reference; achieved
    // apex height is verified offline from simulation telemetry only.
    float target_apex_height_m = 0.20F;
    float gravity_m_s2 = 9.81F;
    float flight_tuck_knee_rad = -1.45F;
    float flight_tuck_duration_s = 0.08F;
    float prelanding_lead_s = 0.12F;
    float landing_duration_s = 0.25F;
    float settle_duration_s = 0.5F;
    float max_settle_s = 3.0F;
    float grounded_duration_s = 0.10F;
    float contact_debounce_s = 0.01F;
    float tracking_tolerance_rad = 0.12F;
    float max_pitch_rad = 0.5F;
    // Crouch has a tighter sensor-only envelope than the general jump. A
    // pre-thrust pitch is a failed preparation, not a valid take-off cue.
    float crouch_max_pitch_rad = 0.25F;
    float crouch_max_pitch_rate_rad_s = 0.75F;
    // Counter-movement speed is independent from the thrust/flight profile.
    float crouch_joint_speed_rad_s = 12.5663706F;
    float crouch_min_speed_scale = 0.35F;
    float max_leg_torque_nm = 5.0F;
    float max_wheel_torque_nm = 0.13F;
    float max_joint_speed_rad_s = 12.5663706F;  // rated 120 rpm profile
    float thrust_force_n = 90.F;  // desired total upward reaction, bounded per joint
    float body_pitch_kp = 4.F;  // total body attitude feedback, Nm/rad
    float body_pitch_kd = .4F;  // total body attitude feedback, Nms/rad
    float hip_attitude_torque_limit_nm = 1.F;
    float flight_hip_rate_kp = 0.F;  // optional integrated flight posture, 1/s
    float flight_hip_rate_kd = 0.F;
    bool thrust_track_measured_knee = false;
    bool thrust_knee_force_control = false;
    // A normal jump is a bilateral support event. Do not continue a
    // pre-takeoff thrust with one leg when the sensor-only support hypothesis
    // is no longer bilateral.
    bool require_bilateral_thrust = true;
    // Keep the bounded ground-reaction feedforward after the knee reaches its
    // extension endpoint until the sensor-only support hypothesis is lost.
    // This is needed for a real impulse: releasing at the endpoint merely
    // unloads the wheels and produces a false ballistic transition.
    bool hold_thrust_after_extension = false;
    bool landing_world_frame = false;
    float thrust_hip_offset_rad = 0.F;
    bool thrust_wheel_control = false;
    float thrust_wheel_speed_m_s = 0.F;
    bool landing_compression = false;
    // The landing compression target must be deeper than the airborne tuck;
    // otherwise touchdown would extend the legs instead of absorbing energy.
    float compression_knee_rad = -1.60F;
    float compression_duration_s = .12F;
    bool smooth_phase_transitions = false;
    bool thrust_world_frame = false;
    const JumpPlan* plan = nullptr;  // validated, immutable, outlives controller
    float plan_prep_hip_torque_nm = 0.F;
    float plan_prep_knee_torque_nm = 0.F;
    float plan_preparation_wait_s = 2.F;
  };
  struct Input {
    float dt_s = .005F;
    bool request = false;
    bool eligible = false;  // enabled balance-capable mode
    bool stop = false;
    bool sensors_fresh = false;
    bool contacts_fresh = false;
    bool left_contact = false;
    bool right_contact = false;
    // Support/flight hypothesis produced by FlightObserver from MPU6500 and
    // encoders. These are not measured contact switches.
    bool support_contact = true;
    bool airborne = false;
    bool ballistic = false;
    bool touchdown_candidate = false;
    float flight_time_s = 0.0F;
    float downward_velocity_m_s = 0.0F;
    bool operator_driving = false;
    float pitch_rad = 0;
    float pitch_rate_rad_s = 0;
    float ground_speed_m_s = 0;
    // Relative FK height used only for offline diagnostic fields in Output;
    // never used to gate or time the jump. Absolute body/world pose is not an
    // input to this controller.
    double com_z_m = 0;
    // Kept for source compatibility with offline plan tooling. Runtime jump
    // feedback deliberately ignores these unobservable world coordinates.
    double body_x_m = 0, body_z_m = 0;
    JointArray position_rad{};
    JointArray velocity_rad_s{};
  };
  struct Output {
    Phase phase = Phase::Idle;
    const char* reason = "idle";
    bool owns_pose = false;
    bool ready = false;
    bool stop_motors = false;
    bool balance_wheels = true;
    bool override_wheel_speed = false;
    bool wheel_speed_is_relative = false;
    float wheel_speed_rad_s = 0.F;
    bool airborne_seen = false;
    float leg_kp_scale = 1.F;
    std::array<float, 2> knee_kp_scale{1.F, 1.F};
    float station_blend = 1.F;
    JointArray position_rad{};
    JointArray feedforward_nm{};
    // Bounded target differences / dt; Hermite boundaries preserve the incoming
    // command rate, and contact-gated force thrust knees use an extension rate.
    JointArray velocity_rad_s{};
    double takeoff_com_z_m = 0;
    double apex_com_z_m = 0;
    double apex_gain_m = 0;
    float target_apex_height_m = 0;
    float flight_time_s = 0;
    bool prelanding_active = false;
    float phase_elapsed_s = 0;
    float plan_elapsed_s = 0;
    float grounded_dwell_s = 0;
    bool stance_neutral = false;
    bool input_safe = false;
    float stance_position_error_rad = 0;
    float stance_velocity_error_rad_s = 0;
  };
  JumpController();
  explicit JumpController(Config config);
  [[nodiscard]] Output update(const Input& input);
  [[nodiscard]] const char* safetyFault(const Input& input, bool check_dt=true) const;
  void reset();
  void abort(const char* reason);
  [[nodiscard]] bool active() const noexcept;
  [[nodiscard]] Phase phase() const noexcept { return phase_; }
  [[nodiscard]] static const char* phaseName(Phase phase) noexcept;
  [[nodiscard]] static JointArray neutralPose(float knee);
  [[nodiscard]] static JointArray landingPose(float knee, float pitch);
 private:
  Config config_;
  JointArray rest_pose_=neutralPose(0);
  Phase phase_ = Phase::Idle;
  const char* reason_ = "idle";
  JointArray start_{}, target_{};
  float elapsed_ = 0, grounded_ = 0, lost_contact_ = 0, landed_ = 0, settled_ = 0;
  float crouch_contact_lost_ = 0;
  bool last_request_ = false, airborne_ = false;
  double last_ground_com_ = 0, takeoff_com_ = 0, apex_com_ = 0;
  JointArray last_command_{};
  JointArray last_velocity_{}, start_velocity_{};
  std::array<bool, 2> thrust_finished_{};
  float plan_time_s_ = 0.F;
  float plan_wheel_offset_ = 0;
  float plan_contact_wait_s_ = 0.F;
  float nominal_flight_s_ = 0.4F;
  float flight_elapsed_s_ = 0.F;
  JointArray flight_start_pose_{};
  bool rest_pose_latched_ = false;
  void enter(Phase phase, const char* reason);
};
}
