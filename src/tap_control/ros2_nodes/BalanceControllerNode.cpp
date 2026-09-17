#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <sstream>

#include "robot_control/EmbeddedController.hpp"
#include "robot_control/StationKeeper.hpp"
#include "robot_control/JumpController.hpp"
#include "robot_control/WheelFrame.hpp"
#include "robot_control/TrajectoryObserver.hpp"

namespace {

constexpr std::array<const char*, robot_control::kJointCount> kJointNames = {
    "cad_left_hip", "cad_left_knee", "cad_left_wheel",
    "cad_right_hip", "cad_right_knee", "cad_right_wheel"};

constexpr auto kLeftHip =
    static_cast<std::size_t>(robot_control::JointId::LeftHip);
constexpr auto kLeftKnee =
    static_cast<std::size_t>(robot_control::JointId::LeftKnee);
constexpr auto kLeftWheel =
    static_cast<std::size_t>(robot_control::JointId::LeftWheel);
constexpr auto kRightHip =
    static_cast<std::size_t>(robot_control::JointId::RightHip);
constexpr auto kRightKnee =
    static_cast<std::size_t>(robot_control::JointId::RightKnee);
constexpr auto kRightWheel =
    static_cast<std::size_t>(robot_control::JointId::RightWheel);

float getOrDefault(const std::vector<double>& values, std::size_t index) {
  if (index >= values.size()) {
    return 0.0F;
  }
  return static_cast<float>(values[index]);
}

float yawFromQuaternion(double w, double x, double y, double z) {
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return static_cast<float>(std::atan2(siny_cosp, cosy_cosp));
}

// Sagittal tilt from world vertical in body coordinates, independent of yaw.
// The planar model assumes small roll; atan2 also covers recovery beyond 90 deg.
float basePitchFromQuaternion(double w, double x, double y, double z) {
  const double base_z_in_world_x = 2.0 * (w * y - x * z);
  const double base_z_in_world_z = 1.0 - 2.0 * (x * x + y * y);
  return static_cast<float>(
      std::atan2(base_z_in_world_x, base_z_in_world_z));
}

float clampf(float v, float lo, float hi) {
  return std::max(lo, std::min(v, hi));
}

float normalizeAngle(float angle_rad) {
  while (angle_rad > static_cast<float>(M_PI)) {
    angle_rad -= static_cast<float>(2.0 * M_PI);
  }
  while (angle_rad < -static_cast<float>(M_PI)) {
    angle_rad += static_cast<float>(2.0 * M_PI);
  }
  return angle_rad;
}

}  // namespace

class BalanceControllerNode : public rclcpp::Node {
 public:
  BalanceControllerNode()
      : Node("balance_controller_node"),
        control_period_s_(declare_parameter<double>("control_period_s", 0.005)),
        wheel_radius_m_(declare_parameter<double>("wheel_radius_m", 0.03)),
        pitch_offset_rad_(declare_parameter<double>("pitch_offset_rad", 0.0)),
        // The production path is IMU-backed. sim_pose remains available only
        // for legacy/offline experiments that explicitly opt into it; it must
        // never be the implicit source for a landing claim.
        pitch_source_(declare_parameter<std::string>("pitch_source", "imu")),
        sim_pose_child_frame_(declare_parameter<std::string>(
            "sim_pose_child_frame", "tap_robot")),
        sim_pose_index_(declare_parameter<int>("sim_pose_index", 0)),
        sim_contact_min_base_z_m_(static_cast<float>(declare_parameter<double>(
            "sim_contact_min_base_z_m", 0.10))),
        sim_contact_max_pitch_rad_(static_cast<float>(declare_parameter<double>(
            "sim_contact_max_pitch_rad", 0.75))),
        // Ascento (arXiv 2005.11435): "ground contact is detected when the
        // torques in the hip joints exceed a threshold" -- a direct physical
        // measurement, immune to the sim_pose heuristic's latency/noise (see
        // docs/LEDGE_DRIVEOFF_LIMIT.md). Off by default (false): only ever
        // widens state_.floor_contact from false to true (never the other
        // way), and only while sim_pose ALREADY reads airborne, so enabling
        // it cannot change any already-grounded/normal-operation behavior.
        torque_touchdown_enabled_(
            declare_parameter<bool>("torque_touchdown_enabled", false)),
        hip_touchdown_torque_nm_(static_cast<float>(declare_parameter<double>(
            "hip_touchdown_torque_nm", 2.5))),
        hip_touchdown_max_velocity_rad_s_(static_cast<float>(
            declare_parameter<double>("hip_touchdown_max_velocity_rad_s", 1.0))),
        // Free-fall/impact detection from the raw (unfiltered) IMU specific-
        // force magnitude (see onImu). Same widen-only-while-airborne safety
        // pattern as torque_touchdown_enabled_: off by default, can only make
        // state_.floor_contact become true EARLIER than the pose heuristic,
        // never later, and only while already reading airborne.
        imu_touchdown_enabled_(
            declare_parameter<bool>("imu_touchdown_enabled", false)),
        imu_touchdown_accel_mps2_(static_cast<float>(declare_parameter<double>(
            "imu_touchdown_accel_mps2", 20.0))),
        // Fused hard-impact classifier: unlike the two detectors above (each
        // widens floor_contact EARLIER on a single signal - both measured to
        // hurt, see docs/LEDGE_DRIVEOFF_LIMIT.md), this one requires BOTH the
        // raw accel spike AND a high measured pitch rate together (real
        // fall-detection literature fuses multiple axes rather than
        // thresholding one signal alone - e.g. combined acceleration +
        // angular velocity + tilt features reach 100% accuracy in published
        // pre-impact fall detectors) to flag a genuinely violent, chaotic
        // impact, and on that fused signal it does the OPPOSITE: DELAYS
        // floor_contact a bit further past whatever the pose heuristic would
        // otherwise say, giving the chaotic rotation time to damp passively
        // before the MPC resumes actively commanding wheel torque. This
        // follows directly from the empirical pattern of the three earlier,
        // single-signal, earlier-reaction attempts: reacting sooner during
        // the chaotic snag consistently measured worse, not better, so this
        // tests reacting LATER specifically for the flagged-severe case only
        // (ordinary landings are untouched, since the fused condition should
        // rarely if ever coincide with a benign touchdown).
        hard_impact_delay_enabled_(
            declare_parameter<bool>("hard_impact_delay_enabled", false)),
        hard_impact_accel_mps2_(static_cast<float>(declare_parameter<double>(
            "hard_impact_accel_mps2", 50.0))),
        hard_impact_pitch_rate_rad_s_(static_cast<float>(declare_parameter<double>(
            "hard_impact_pitch_rate_rad_s", 6.0))),
        hard_impact_settle_s_(static_cast<float>(
            declare_parameter<double>("hard_impact_settle_s", 0.15))),
        left_wheel_sign_(
            declare_parameter<bool>("left_wheeled", true) ? 1.0F : -1.0F),
        // Kalman filter process/measurement noise, the standard defaults
        // for this exact design (Kristian Lauszus's TKJElectronics Kalman
        // filter for accel+gyro angle estimation) - kept tunable as ROS
        // parameters rather than hardcoded, but not re-derived here.
        kalman_q_angle_(static_cast<float>(
            declare_parameter<double>("kalman_q_angle", 0.001))),
        kalman_q_bias_(static_cast<float>(
            declare_parameter<double>("kalman_q_bias", 0.003))),
        kalman_r_measure_(static_cast<float>(
            declare_parameter<double>("kalman_r_measure", 0.03))),
        // Adaptive-trust term, not part of the base TKJElectronics design:
        // scales R_measure up when |accel_mag - g| is large, i.e. when a
        // real linear acceleration (not tilt) is disturbing the
        // accelerometer reading. The drive-direction case already has a
        // feedforward correction (MPC's own commanded ground accel,
        // subtracted from accel_x above); squat/pose changes have no such
        // model (CoM height change perturbs accel_z, not something the
        // planner exposes as a clean feedforward term), so this generic,
        // axis-agnostic fallback catches that case too. 0 disables it
        // (pre-2026-09-16 behavior).
        kalman_accel_trust_gain_(static_cast<float>(
            declare_parameter<double>("kalman_accel_trust_gain", 0.0))),
        // An accelerometer measures specific force, not gravity. During
        // free-fall its magnitude approaches zero and during impact/fast
        // rotation its direction is not a valid tilt measurement. These
        // gates keep the estimator on gyro prediction through those dynamic
        // intervals instead of feeding atan2() an arbitrary vector.
        kalman_accel_max_deviation_mps2_(static_cast<float>(
            declare_parameter<double>("kalman_accel_max_deviation_mps2", 1.5))),
        kalman_accel_max_pitch_rate_rad_s_(static_cast<float>(
            declare_parameter<double>("kalman_accel_max_pitch_rate_rad_s", 1.5))),
        leg_kp_(static_cast<float>(declare_parameter<double>("leg_kp", 12.0))),
        // Default is now derived from the real mass/geometry data (Track B
        // item 3, 2026-09-16), not a guess: critical damping for a 2nd-order
        // system I*theta'' + Kd*theta' + Kp*theta = 0 is Kd = 2*sqrt(Kp*I).
        // leg_hip_inertia_kgm2/leg_knee_inertia_kgm2 (tools/
        // generate_control_model.py's leg_inertia(), parallel-axis theorem
        // from mesh_inertials.yaml/masses.yaml at the CAD-zero neutral
        // pose) differ ~9x (hip swings thigh+shin+wheel, knee only
        // shin+wheel) - the same single leg_kd_ already drives both joints
        // (see the per-joint loop below), so take the larger (hip) inertia:
        // that keeps the smaller (knee) joint non-oscillatory too, at the
        // cost of slightly overdamping it, rather than risking the hip
        // joint being underdamped by tuning to the knee's smaller inertia.
        // Still a plain declare_parameter default - balance_controller.yaml
        // or any override remains free to replace it.
        leg_kd_(static_cast<float>(declare_parameter<double>(
            "leg_kd", 2.0 * std::sqrt(static_cast<double>(leg_kp_) *
                static_cast<double>(std::max(robot_control::geometry::leg_hip_inertia_kgm2,
                                             robot_control::geometry::leg_knee_inertia_kgm2)))))),
        wheel_velocity_kp_(static_cast<float>(
            declare_parameter<double>("wheel_velocity_kp", 2.0))),
        sensor_timeout_s_(declare_parameter<double>("sensor_timeout_s", 0.15)),
        sensor_stale_grace_cycles_(static_cast<int>(
            declare_parameter<int>("sensor_stale_grace_cycles", 8))),
        auto_enable_(declare_parameter<bool>("auto_enable", false)),
        allow_motion_commands_(
            declare_parameter<bool>("allow_motion_commands", true)),
        imu_noise_enabled_(
            declare_parameter<bool>("imu_noise.enabled", false)),
        imu_pitch_noise_stddev_rad_(declare_parameter<double>(
            "imu_noise.pitch_stddev_rad", 0.0)),
        imu_gyro_noise_stddev_rad_s_(declare_parameter<double>(
            "imu_noise.gyro_stddev_rad_s", 0.0)),
        imu_noise_rng_(static_cast<std::mt19937::result_type>(
            declare_parameter<int>("imu_noise.seed", 42))) {
    // The MPC wheel balancer is the only balance strategy. Wheel geometry and
    // limits, then the model and QP cost weights.
    robot_control::EmbeddedController::Config config;
    config.mpc.wheel_radius_m = static_cast<float>(
        declare_parameter<double>("mpc.wheel_radius_m", wheel_radius_m_));
    config.mpc.half_wheel_base_m = static_cast<float>(
        declare_parameter<double>("mpc.half_wheel_base_m", 0.1545));
    config.mpc.fall_pitch_rad = static_cast<float>(
        declare_parameter<double>("mpc.fall_pitch_rad", 1.0));
    config.state_machine.fall_pitch_rad = config.mpc.fall_pitch_rad;
    // Partial feedback linearization - see MpcBalancer::Config::
    // nonlinear_gravity_correction_gain's own comment. 0 (default) is an
    // exact no-op, matching this stack's usual promotion discipline (must be
    // n>=25-evidence-gated before becoming a yaml default).
    config.mpc.nonlinear_gravity_correction_gain = static_cast<float>(
        declare_parameter<double>("mpc.nonlinear_gravity_correction_gain", 0.0));
    config.mpc.left_wheel_sign = left_wheel_sign_;
    config.mpc.max_ground_velocity_m_s = static_cast<float>(
        declare_parameter<double>("mpc.max_ground_velocity", 0.6));
    config.mpc.leg_length_m = static_cast<float>(
        declare_parameter<double>("mpc.leg_length", 0.38));
    // Pendulum arm adapts to the live CoM height (squat) between these bounds;
    // per_com_height * stand-height reproduces leg_length at the straight stance.
    config.mpc.pendulum_length_per_com_height = static_cast<float>(
        declare_parameter<double>("mpc.pendulum_length_per_com_height", 1.87));
    config.mpc.min_pendulum_length_m = static_cast<float>(
        declare_parameter<double>("mpc.min_pendulum_length", 0.20));
    config.mpc.max_pendulum_length_m = static_cast<float>(
        declare_parameter<double>("mpc.max_pendulum_length", 0.42));
    config.mpc.gravity_m_s2 = static_cast<float>(
        declare_parameter<double>("mpc.gravity_m_s2", 9.81));
    config.mpc.max_ground_accel_m_s2 = static_cast<float>(
        declare_parameter<double>("mpc.max_ground_accel", 10.0));
    // Track B item 2 (2026-09-16): static friction-cone ceiling on top of
    // max_ground_accel - see MpcBalancer::Config::wheel_ground_mu's own
    // comment for the physics and why this is deliberately not gated on
    // live contact force.
    config.mpc.wheel_ground_mu = static_cast<float>(
        declare_parameter<double>("mpc.wheel_ground_mu", 1.0));
    // Yaw disturbance rejection via real gyro-Z feedback - see
    // MpcBalancer::Config::yaw_rate_feedback_kp's own comment. 0 default:
    // exact no-op, byte-identical to pre-2026-09-16 behavior.
    config.mpc.yaw_rate_feedback_kp = static_cast<float>(
        declare_parameter<double>("mpc.yaw_rate_feedback_kp", 0.0));
    config.mpc.nb_timesteps =
        static_cast<int>(declare_parameter<int>("mpc.nb_timesteps", 50));
    config.mpc.sampling_period_s = static_cast<float>(
        declare_parameter<double>("mpc.sampling_period", 0.02));
    config.mpc.stage_input_cost_weight = static_cast<float>(
        declare_parameter<double>("mpc.stage_input_cost_weight", 1.0e-3));
    config.mpc.stage_state_cost_weight = static_cast<float>(
        declare_parameter<double>("mpc.stage_state_cost_weight", 1.0e-3));
    config.mpc.terminal_cost_weight = static_cast<float>(
        declare_parameter<double>("mpc.terminal_cost_weight", 1.0));
    config.mpc.position_cost_scale = static_cast<float>(
        declare_parameter<double>("mpc.position_cost_scale", 1.0));
    config.mpc.velocity_cost_scale = static_cast<float>(
        declare_parameter<double>("mpc.velocity_cost_scale", 1.0));
    config.mpc.max_target_distance_m = static_cast<float>(
        declare_parameter<double>("mpc.max_target_distance_m", 1.0));
    // Slow position hold. EmbeddedController only lets it run while the robot is
    // holding a bent pose with no drive input (bring-up / driving keep it off),
    // where it bleeds the residual creep the MPC's own weak position authority
    // leaves. anchor_hold makes the target sticky instead of following.
    config.mpc.position_anchor_hold =
        declare_parameter<bool>("mpc.position_anchor_hold", false);
    config.mpc.station_hold_gain_hz = static_cast<float>(
        declare_parameter<double>("mpc.station_hold_gain_hz", 0.0));
    config.mpc.station_hold_max_v = static_cast<float>(
        declare_parameter<double>("mpc.station_hold_max_v", 0.2));
    config.mpc.station_hold_period_s = static_cast<float>(
        declare_parameter<double>("mpc.station_hold_period_s", 1.0));
    config.mpc.solver_iterations =
        static_cast<int>(declare_parameter<int>("mpc.solver_iterations", 120));
    // Landing recovery: bounded post-touchdown window (see MpcBalancer.hpp).
    // leg_kd_scale and every landing_* boost default to 0 so a stock launch is
    // byte-identical to the pre-landing-logic controller; only a profile yaml
    // that sets these nonzero (e.g. tools/*landing* test profiles) changes
    // anything. mpc.leg_kd_scale defaulting to 0 also means the always-on
    // active-damping term publishLegCommand() now applies is inert outside the
    // landing window - an earlier constant (always-on) active-damping attempt
    // was found to either regress stand-transition tracking or, at a milder
    // gain, show no measurable landing benefit while adding risk; this phase-
    // gated form is scoped to a bounded post-touchdown window instead.
    config.mpc.leg_kd_scale = static_cast<float>(
        declare_parameter<double>("mpc.leg_kd_scale", 0.0));
    config.mpc.landing_window_s = static_cast<float>(
        declare_parameter<double>("mpc.landing_window_s", 0.25));
    config.mpc.landing_kp_boost = static_cast<float>(
        declare_parameter<double>("mpc.landing_kp_boost", 0.0));
    config.mpc.landing_kd_boost = static_cast<float>(
        declare_parameter<double>("mpc.landing_kd_boost", 0.0));
    config.mpc.landing_ground_accel_boost_m_s2 = static_cast<float>(
        declare_parameter<double>("mpc.landing_ground_accel_boost_m_s2", 0.0));
    config.mpc.landing_hold_airborne_velocity =
        declare_parameter<bool>("mpc.landing_hold_airborne_velocity", false);

    // Reactive airborne/touchdown leg-TARGET offset (see LandingReflex.hpp) -
    // distinct from the stiffness-only landing_* boosts above: this changes
    // what position the legs are commanded to, not how hard they resist a
    // fixed target. All gains default to 0 so a stock launch never engages it.
    config.landing_reflex.airborne_knee_crouch_rad = static_cast<float>(
        declare_parameter<double>("landing_reflex.airborne_knee_crouch_rad", 0.0));
    config.landing_reflex.hip_offset_per_pitch_rate = static_cast<float>(
        declare_parameter<double>("landing_reflex.hip_offset_per_pitch_rate", 0.0));
    config.landing_reflex.max_hip_offset_rad = static_cast<float>(
        declare_parameter<double>("landing_reflex.max_hip_offset_rad", 0.3));
    config.landing_reflex.catch_hold_s = static_cast<float>(
        declare_parameter<double>("landing_reflex.catch_hold_s", 0.2));
    config.landing_reflex.catch_blend_s = static_cast<float>(
        declare_parameter<double>("landing_reflex.catch_blend_s", 0.2));
    // Active velocity-matched extension - see LandingReflex::Config::
    // active_extension_gravity_m_s2's own comment. Both default off (0 = no
    // extension attempted, exact no-op).
    config.landing_reflex.active_extension_gravity_m_s2 = static_cast<float>(
        declare_parameter<double>("landing_reflex.active_extension_gravity_m_s2", 0.0));
    config.landing_reflex.active_extension_max_rad = static_cast<float>(
        declare_parameter<double>("landing_reflex.active_extension_max_rad", 0.0));
    config.landing_reflex.active_extension_max_hip_rad = static_cast<float>(
        declare_parameter<double>("landing_reflex.active_extension_max_hip_rad", -1.0));
    config.landing_reflex.active_extension_delay_s = static_cast<float>(
        declare_parameter<double>("landing_reflex.active_extension_delay_s", 0.0));
    config.landing_reflex.unloading_accel_mps2 = static_cast<float>(
        declare_parameter<double>("landing_reflex.unloading_accel_mps2", 0.0));
    config.landing_reflex.target_slew_rate_rad_s = static_cast<float>(
        declare_parameter<double>("landing_reflex.target_slew_rate_rad_s", 0.0));
    // Differential (ROLL-axis) HIP offset - see LandingReflex::Config::
    // roll_rate_hip_gain_rad_per_rad_s's own comment (targets the hip, not
    // the knee, because LandingImpedance crushes the knee's stiffness during
    // exactly this window). Both default off (0 = exact no-op).
    config.landing_reflex.roll_rate_hip_gain_rad_per_rad_s = static_cast<float>(
        declare_parameter<double>(
            "landing_reflex.roll_rate_hip_gain_rad_per_rad_s", 0.0));
    config.landing_reflex.roll_hip_offset_max_rad = static_cast<float>(
        declare_parameter<double>("landing_reflex.roll_hip_offset_max_rad", 0.0));
    // Capture-point hip placement - see LandingReflex::Config::
    // capture_point_gravity_m_s2's own comment. Both default off (0 = exact
    // no-op).
    config.landing_reflex.capture_point_gravity_m_s2 = static_cast<float>(
        declare_parameter<double>("landing_reflex.capture_point_gravity_m_s2", 0.0));
    config.landing_reflex.capture_point_max_hip_rad = static_cast<float>(
        declare_parameter<double>("landing_reflex.capture_point_max_hip_rad", 0.0));

    // Virtual leg-length spring-damper via GAIN SCHEDULING (see
    // LandingImpedance.hpp) - distinct from landing_reflex (which moves the
    // target) and from mpc.landing_* (wheel authority): on a real touchdown
    // this softens Kp / raises Kd toward critical damping on ALL FOUR leg
    // joints so the leg yields passively under the real impact instead of
    // being actively driven. impact_kp_scale/impact_kd_scale default to 1.0
    // (no scaling) and the velocity gate defaults impossibly high, so a
    // stock launch never engages it.
    config.landing_impedance.impact_kp_scale = static_cast<float>(
        declare_parameter<double>("landing_impedance.impact_kp_scale", 1.0));
    config.landing_impedance.impact_kd_scale = static_cast<float>(
        declare_parameter<double>("landing_impedance.impact_kd_scale", 1.0));
    config.landing_impedance.airborne_hip_kp_scale = static_cast<float>(
        declare_parameter<double>("landing_impedance.airborne_hip_kp_scale", 1.0));
    config.landing_impedance.airborne_hip_kd_scale = static_cast<float>(
        declare_parameter<double>("landing_impedance.airborne_hip_kd_scale", 1.0));
    config.landing_impedance.airborne_knee_kp_scale = static_cast<float>(
        declare_parameter<double>("landing_impedance.airborne_knee_kp_scale", 1.0));
    config.landing_impedance.airborne_knee_kd_scale = static_cast<float>(
        declare_parameter<double>("landing_impedance.airborne_knee_kd_scale", 1.0));
    config.landing_impedance.touchdown_velocity_threshold_m_s = static_cast<float>(
        declare_parameter<double>(
            "landing_impedance.touchdown_velocity_threshold_m_s", 1.0e6));
    config.landing_impedance.rebound_blend_s = static_cast<float>(
        declare_parameter<double>("landing_impedance.rebound_blend_s", 0.15));
    config.landing_impedance.max_impact_duration_s = static_cast<float>(
        declare_parameter<double>("landing_impedance.max_impact_duration_s", 0.4));
    config.landing_impedance.min_airborne_before_touchdown_s = static_cast<float>(
        declare_parameter<double>(
            "landing_impedance.min_airborne_before_touchdown_s", 0.15));
    // Energy-based sizing (see LandingImpedance::Config::
    // leg_knee_inertia_kgm2's own comment). Reuses the SAME real knee
    // inertia and the SAME leg_kp_/leg_kd_ this node already computed above
    // (critical-damping-derived default, not a separate guess) as the
    // "nominal" gains the energy calculation scales relative to.
    // knee_hard_limit_rad defaults to the URDF's own actuator limit
    // (src/tap_robot_description/description/cad_robot/robot.urdf.xacro,
    // <joint cad_*_knee> lower="-2.1") - the TRUE mechanical limit impact
    // absorption must stay short of, not a softer operating bound.
    // leg_knee_inertia_kgm2 defaults to 0 (energySizingConfigured() false,
    // exact no-op) since enabling it also requires deciding
    // energy_stroke_margin/knee_hard_limit_rad deliberately, not silently.
    config.landing_impedance.leg_knee_inertia_kgm2 = static_cast<float>(
        declare_parameter<double>("landing_impedance.leg_knee_inertia_kgm2", 0.0));
    config.landing_impedance.leg_kp_nominal_nm_rad = leg_kp_;
    config.landing_impedance.leg_kd_nominal_nm_s_rad = leg_kd_;
    config.landing_impedance.knee_hard_limit_rad = static_cast<float>(
        declare_parameter<double>("landing_impedance.knee_hard_limit_rad", -2.1));
    config.landing_impedance.energy_stroke_margin = static_cast<float>(
        declare_parameter<double>("landing_impedance.energy_stroke_margin", 0.7));
    config.landing_impedance.energy_kp_scale_min = static_cast<float>(
        declare_parameter<double>("landing_impedance.energy_kp_scale_min", 0.02));
    config.landing_impedance.energy_kp_scale_max = static_cast<float>(
        declare_parameter<double>("landing_impedance.energy_kp_scale_max", 1.0));
    config.landing_impedance.energy_kd_scale_max = static_cast<float>(
        declare_parameter<double>("landing_impedance.energy_kd_scale_max", 6.0));
    // See LandingImpedance::Config::hard_impact_accel_mps2/
    // energy_capture_window_s's own comments. Both default off (impossibly
    // high accel gate, 0 capture window = original single-tick behavior).
    config.landing_impedance.hard_impact_accel_mps2 = static_cast<float>(
        declare_parameter<double>("landing_impedance.hard_impact_accel_mps2", 1.0e6));
    config.landing_impedance.energy_capture_window_s = static_cast<float>(
        declare_parameter<double>("landing_impedance.energy_capture_window_s", 0.0));

    config.landing_controller.enabled =
        declare_parameter<bool>("landing_controller.enabled", false);
    config.landing_controller.flight_gravity_m_s2 = static_cast<float>(
        declare_parameter<double>("landing_controller.flight_gravity_m_s2", 9.81));
    config.landing_controller.flight_extension_delay_s = static_cast<float>(
        declare_parameter<double>("landing_controller.flight_extension_delay_s", 0.08));
    config.landing_controller.flight_extension_max_m = static_cast<float>(
        declare_parameter<double>("landing_controller.flight_extension_max_m", 0.035));
    config.landing_controller.flight_hip_offset_max_rad = static_cast<float>(
        declare_parameter<double>("landing_controller.flight_hip_offset_max_rad", 0.04));
    config.landing_controller.flight_knee_offset_max_rad = static_cast<float>(
        declare_parameter<double>("landing_controller.flight_knee_offset_max_rad", 0.25));
    config.landing_controller.flight_target_slew_rate_rad_s = static_cast<float>(
        declare_parameter<double>("landing_controller.flight_target_slew_rate_rad_s", 2.0));
    config.landing_controller.flight_kp_scale = static_cast<float>(
        declare_parameter<double>("landing_controller.flight_kp_scale", 0.5));
    config.landing_controller.flight_kd_scale = static_cast<float>(
        declare_parameter<double>("landing_controller.flight_kd_scale", 1.0));
    config.landing_controller.flight_pitch_rate_hip_gain_rad_per_rad_s =
        static_cast<float>(declare_parameter<double>(
            "landing_controller.flight_pitch_rate_hip_gain_rad_per_rad_s", 0.0));
    config.landing_controller.flight_pitch_rate_hip_offset_max_rad =
        static_cast<float>(declare_parameter<double>(
            "landing_controller.flight_pitch_rate_hip_offset_max_rad", 0.0));
    config.landing_controller.min_flight_before_impact_s = static_cast<float>(
        declare_parameter<double>("landing_controller.min_flight_before_impact_s", 0.03));
    config.landing_controller.impact_accel_mps2 = static_cast<float>(
        declare_parameter<double>("landing_controller.impact_accel_mps2", 35.0));
    config.landing_controller.impact_pitch_rate_rad_s = static_cast<float>(
        declare_parameter<double>("landing_controller.impact_pitch_rate_rad_s", 4.0));
    config.landing_controller.imu_impact_debounce_s = static_cast<float>(
        declare_parameter<double>("landing_controller.imu_impact_debounce_s", 0.02));
    config.landing_controller.imu_impact_without_wheel_contact =
        declare_parameter<bool>("landing_controller.imu_impact_without_wheel_contact", false);
    config.landing_controller.height_kp_n_m = static_cast<float>(
        declare_parameter<double>("landing_controller.height_kp_n_m", 0.0));
    config.landing_controller.height_kd_n_s_m = static_cast<float>(
        declare_parameter<double>("landing_controller.height_kd_n_s_m", 0.0));
    config.landing_controller.x_kp_n_m = static_cast<float>(
        declare_parameter<double>("landing_controller.x_kp_n_m", 0.0));
    config.landing_controller.x_kd_n_s_m = static_cast<float>(
        declare_parameter<double>("landing_controller.x_kd_n_s_m", 0.0));
    config.landing_controller.max_virtual_force_n = static_cast<float>(
        declare_parameter<double>("landing_controller.max_virtual_force_n", 40.0));
    config.landing_controller.max_feedforward_torque_nm = static_cast<float>(
        declare_parameter<double>("landing_controller.max_feedforward_torque_nm", 3.0));
    config.landing_controller.impact_kp_scale = static_cast<float>(
        declare_parameter<double>("landing_controller.impact_kp_scale", 0.05));
    config.landing_controller.impact_kd_scale = static_cast<float>(
        declare_parameter<double>("landing_controller.impact_kd_scale", 1.0));
    config.landing_controller.min_impact_s = static_cast<float>(
        declare_parameter<double>("landing_controller.min_impact_s", 0.04));
    config.landing_controller.max_impact_s = static_cast<float>(
        declare_parameter<double>("landing_controller.max_impact_s", 0.35));
    config.landing_controller.recovery_blend_s = static_cast<float>(
        declare_parameter<double>("landing_controller.recovery_blend_s", 0.25));
    config.landing_controller.recovery_pitch_rate_rad_s = static_cast<float>(
        declare_parameter<double>("landing_controller.recovery_pitch_rate_rad_s", 1.2));
    config.landing_controller.recovery_joint_speed_rad_s = static_cast<float>(
        declare_parameter<double>("landing_controller.recovery_joint_speed_rad_s", 1.5));
    // Sensor-only support/flight observer. Absolute ledge height is not
    // observable from this sensor set during flight; these parameters only
    // gate support-loss, ballistic, collision and touchdown phases.
    config.flight_observer.gravity_m_s2 = static_cast<float>(
        declare_parameter<double>("flight_observer.gravity_m_s2", 9.81));
    config.flight_observer.support_loss_debounce_s = static_cast<float>(
        declare_parameter<double>("flight_observer.support_loss_debounce_s", 0.012));
    config.flight_observer.free_fall_accel_threshold_m_s2 = static_cast<float>(
        declare_parameter<double>("flight_observer.free_fall_accel_threshold_m_s2", 3.5));
    config.flight_observer.free_fall_debounce_s = static_cast<float>(
        declare_parameter<double>("flight_observer.free_fall_debounce_s", 0.012));
    config.flight_observer.collision_accel_threshold_m_s2 = static_cast<float>(
        declare_parameter<double>("flight_observer.collision_accel_threshold_m_s2", 25.0));
    config.flight_observer.collision_hold_s = static_cast<float>(
        declare_parameter<double>("flight_observer.collision_hold_s", 0.02));
    config.flight_observer.touchdown_debounce_s = static_cast<float>(
        declare_parameter<double>("flight_observer.touchdown_debounce_s", 0.015));
    config.flight_observer.support_settle_s = static_cast<float>(
        declare_parameter<double>("flight_observer.support_settle_s", 0.10));
    config.flight_observer.max_airborne_s = static_cast<float>(
        declare_parameter<double>("flight_observer.max_airborne_s", 0.75));
    config.trajectory_observer.wheel_radius_m =
        static_cast<float>(wheel_radius_m_);
    config.trajectory_observer.half_track_m = config.mpc.half_wheel_base_m;
    config.trajectory_observer_min_confidence = static_cast<float>(
        declare_parameter<double>("trajectory_observer.min_confidence", 0.50));
    // The raw IMU event is latched in onImu because the impact signature can
    // be shorter than one control tick. Keep the thresholds identical to the
    // LandingController configuration so the ROS adapter and core classify
    // the same physical event.
    landing_impact_accel_mps2_ = config.landing_controller.impact_accel_mps2;
    landing_impact_pitch_rate_rad_s_ =
        config.landing_controller.impact_pitch_rate_rad_s;
    landing_impact_latch_s_ = static_cast<float>(declare_parameter<double>(
        "landing_controller.imu_impact_latch_s", 0.06));

    // Real-time whole-body kinematic compensation: the MPC gets a leaned
    // base-pitch target computed from the live leg pose (CoM offset), so the
    // legs can bend / a pose command can run while the wheels keep balance.
    config.whole_body_compensation =
        declare_parameter<bool>("whole_body.enabled", true);
    config.whole_body_pitch_gain = static_cast<float>(
        declare_parameter<double>("whole_body.pitch_gain", 1.0));
    config.max_balance_pitch_ref_rad = static_cast<float>(
        declare_parameter<double>("whole_body.max_pitch_ref_rad", 0.35));
    config.balance_pitch_ref_period_s = static_cast<float>(
        declare_parameter<double>("whole_body.ref_lowpass_period_s", 0.15));
    // Slow integral pitch trim that removes the residual drift while the robot
    // sits in a bent pose (leg sag + dynamic load the FK model misses). Gated
    // on: bent legs + settled Balance + a dwell time, so it is inert during
    // bring-up and pose transitions. gain 0 disables it.
    config.whole_body_trim_gain_hz = static_cast<float>(
        declare_parameter<double>("whole_body.trim_gain_hz", 0.0));
    config.whole_body_trim_dwell_s = static_cast<float>(
        declare_parameter<double>("whole_body.trim_dwell_s", 1.0));
    config.max_whole_body_trim_rad = static_cast<float>(
        declare_parameter<double>("whole_body.max_trim_rad", 0.18));

    config.pose_change_duration_s = static_cast<float>(
        declare_parameter<double>("pose_change_duration_s", 1.0));
    config.stand_up.tuck_duration_s = static_cast<float>(
        declare_parameter<double>("stand_up.tuck_duration_s", 1.0));
    config.stand_up.rise_duration_s = static_cast<float>(
        declare_parameter<double>("stand_up.rise_duration_s", 1.6));
    config.stand_up.fall_pitch_rad = static_cast<float>(
        declare_parameter<double>("stand_up.fall_pitch_rad", 2.0));
    config.stand_up.settle_pitch_rad = static_cast<float>(
        declare_parameter<double>("stand_up.settle_pitch_rad", 0.2));
    config.stand_up.max_settle_duration_s = static_cast<float>(
        declare_parameter<double>("stand_up.max_settle_duration_s", 3.0));
    config.stand_up.fall_direction_aware =
        declare_parameter<bool>("stand_up.fall_direction_aware", true);
    config.leg_max_torque_nm = static_cast<float>(
        declare_parameter<double>("leg_max_torque_nm", 6.0));
    config.wheel_max_torque_nm = static_cast<float>(
        declare_parameter<double>("wheel_max_torque_nm", 0.5));

    // Real-time /target_pose slew + safe leg-joint envelope.
    pose_slew_rate_rad_s_ = static_cast<float>(
        declare_parameter<double>("pose_slew_rate_rad_s", 2.0));
    pose_hip_min_rad_ = static_cast<float>(
        declare_parameter<double>("pose_hip_min_rad", -0.5));
    pose_hip_max_rad_ = static_cast<float>(
        declare_parameter<double>("pose_hip_max_rad", 0.8));
    pose_knee_min_rad_ = static_cast<float>(
        declare_parameter<double>("pose_knee_min_rad", -1.6));
    pose_knee_max_rad_ = static_cast<float>(
        declare_parameter<double>("pose_knee_max_rad", 0.3));

    config.hip_min_rad = pose_hip_min_rad_;
    config.hip_max_rad = pose_hip_max_rad_;
    config.knee_min_rad = pose_knee_min_rad_;
    config.knee_max_rad = pose_knee_max_rad_;
    // See EmbeddedController::Config::keep_legs_active_when_fallen's own
    // comment. false (default) preserves the prior total-safe-stop behavior
    // byte-for-byte.
    config.keep_legs_active_when_fallen = declare_parameter<bool>(
        "keep_legs_active_when_fallen", false);
    keep_legs_active_when_fallen_ = config.keep_legs_active_when_fallen;
    config.landing_fallen_recovery_enabled = declare_parameter<bool>(
        "landing_fallen_recovery.enabled", false);
    config.landing_recovery_control_pitch_rad = static_cast<float>(
        declare_parameter<double>("landing_fallen_recovery.control_pitch_rad", 1.25));
    config.landing_recovery_handoff_pitch_rad = static_cast<float>(
        declare_parameter<double>("landing_fallen_recovery.handoff_pitch_rad", 0.25));
    config.landing_recovery_pitch_rate_rad_s = static_cast<float>(
        declare_parameter<double>("landing_fallen_recovery.pitch_rate_rad_s", 0.6));
    config.landing_recovery_hold_s = static_cast<float>(
        declare_parameter<double>("landing_fallen_recovery.hold_s", 0.20));
    const auto load_pose = [this](const std::string& name, robot_control::JointArray& pose) {
      const auto values = declare_parameter<std::vector<double>>(
          name, std::vector<double>(pose.begin(), pose.end()));
      if (values.size() != pose.size() || !std::all_of(values.begin(), values.end(),
                                                     [](double v) { return std::isfinite(v); })) {
        throw std::invalid_argument(name + " requires six finite native joint angles");
      }
      std::copy(values.begin(), values.end(), pose.begin());
    };
    load_pose("stand_up.stand_pose_rad", config.stand_up.stand_pose_rad);
    load_pose("stand_up.tuck_pose_rad", config.stand_up.tuck_pose_rad);

    controller_ = std::make_unique<robot_control::EmbeddedController>(config);
    jump_config_.enabled = declare_parameter<bool>("jump.enabled", false);
    jump_wheel_frame_compensation_=declare_parameter<bool>("jump.wheel_frame_compensation",false);
    if(jump_config_.enabled && jump_wheel_frame_compensation_ && left_wheel_sign_!=1.F)
      throw std::invalid_argument("Wheel frame compensation requires native +Y wheel signs");
    jump_profile_ = declare_parameter<std::string>("jump.profile", "rated_120rpm");
    if (jump_profile_ != "rated_120rpm" && jump_profile_ != "manual_curve_experimental")
      throw std::invalid_argument("Unknown jump speed profile");
    jump_config_.max_joint_speed_rad_s = jump_profile_ == "rated_120rpm" ? 12.5663706F : 28.274334F;
    jump_config_.crouch_knee_rad = declare_parameter<double>("jump.crouch_knee_rad", -1.2);
    jump_config_.extension_knee_rad = declare_parameter<double>("jump.extension_knee_rad", 0.);
    jump_config_.landing_knee_rad = declare_parameter<double>("jump.landing_knee_rad", -.6);
    jump_config_.crouch_duration_s = declare_parameter<double>("jump.crouch_duration_s", 2.0);
    jump_config_.thrust_duration_s = declare_parameter<double>("jump.thrust_duration_s", .18);
    jump_config_.max_thrust_s = declare_parameter<double>("jump.max_thrust_s", .5);
    jump_config_.landing_duration_s = declare_parameter<double>("jump.landing_duration_s", .25);
    jump_config_.settle_duration_s = declare_parameter<double>("jump.settle_duration_s", .5);
    jump_config_.contact_debounce_s = declare_parameter<double>("jump.contact_debounce_s", .01);
    jump_config_.thrust_force_n = declare_parameter<double>("jump.thrust_force_n", 90.0);
    jump_config_.body_pitch_kp = declare_parameter<double>("jump.body_pitch_kp", 4.0);
    jump_config_.body_pitch_kd = declare_parameter<double>("jump.body_pitch_kd", .4);
    jump_config_.hip_attitude_torque_limit_nm = declare_parameter<double>("jump.hip_attitude_torque_limit_nm", 1.0);
    jump_config_.flight_hip_rate_kp = declare_parameter<double>("jump.flight_hip_rate_kp", 0.0);
    jump_config_.flight_hip_rate_kd = declare_parameter<double>("jump.flight_hip_rate_kd", 0.0);
    jump_config_.thrust_track_measured_knee = declare_parameter<bool>("jump.thrust_track_measured_knee", false);
    jump_config_.thrust_knee_force_control = declare_parameter<bool>("jump.thrust_knee_force_control", false);
    jump_config_.landing_world_frame = declare_parameter<bool>("jump.landing_world_frame", false);
    jump_config_.thrust_hip_offset_rad = declare_parameter<double>("jump.thrust_hip_offset_rad", 0.0);
    jump_config_.thrust_wheel_control = declare_parameter<bool>("jump.thrust_wheel_control", false);
    jump_config_.thrust_wheel_speed_m_s = declare_parameter<double>("jump.thrust_wheel_speed_m_s", 0.0);
    jump_config_.landing_compression = declare_parameter<bool>("jump.landing_compression", false);
    jump_config_.compression_knee_rad = declare_parameter<double>("jump.compression_knee_rad", -1.0);
    jump_config_.compression_duration_s = declare_parameter<double>("jump.compression_duration_s", .12);
    jump_config_.smooth_phase_transitions = declare_parameter<bool>("jump.smooth_phase_transitions", false);
    jump_config_.thrust_world_frame = declare_parameter<bool>("jump.thrust_world_frame", false);
    jump_config_.max_leg_torque_nm = declare_parameter<double>("jump.leg_torque_limit_nm", 5.0);
    jump_config_.max_wheel_torque_nm = declare_parameter<double>("jump.wheel_torque_limit_nm", .13);
    const bool use_plan=declare_parameter<bool>("jump.optimized_plan",false);
    const auto plan_values=declare_parameter<std::vector<double>>("jump.plan_samples",std::vector<double>{});
    const auto plan_durations=declare_parameter<std::vector<double>>("jump.plan_durations_s",std::vector<double>{});
    jump_config_.plan_prep_hip_torque_nm=declare_parameter<double>("jump.plan_prep_hip_torque_nm",0.);
    jump_config_.plan_prep_knee_torque_nm=declare_parameter<double>("jump.plan_prep_knee_torque_nm",0.);
    jump_config_.plan_preparation_wait_s=declare_parameter<double>("jump.plan_preparation_wait_s",2.);
    if(use_plan) {
      if(!jump_config_.enabled || jump_profile_!="manual_curve_experimental" ||
         plan_durations.size()!=3 || plan_values.empty() || plan_values.size()%52 ||
         plan_values.size()/52>256 ||
         !std::all_of(plan_values.begin(),plan_values.end(),[](double x){return std::isfinite(x);})||
         !std::all_of(plan_durations.begin(),plan_durations.end(),[](double x){return std::isfinite(x)&&x>0;}))
        throw std::invalid_argument("Optimized plan requires bounded manual profile and finite samples");
      jump_plan_=std::make_unique<robot_control::JumpPlan>();
      jump_plan_->count=plan_values.size()/52;
      jump_plan_->thrust_duration_s=plan_durations[0];
      jump_plan_->flight_duration_s=plan_durations[1];
      jump_plan_->touchdown_com_z_m=declare_parameter<double>("jump.plan_touchdown_com_z_m",0.);
      if(!std::isfinite(jump_plan_->touchdown_com_z_m)||jump_plan_->touchdown_com_z_m<=0.F)
        throw std::invalid_argument("Optimized plan requires a positive touchdown COM height");
      for(std::size_t i=0;i<jump_plan_->count;++i) {
        const auto* p=plan_values.data()+52*i;
        jump_plan_->samples[i]={float(p[0]),float(p[1]),float(p[2]),float(p[3]),float(p[4]),
          float(p[5]),float(p[6]),float(p[7]),float(p[8]),float(p[9]),float(p[10])};
        auto& s=jump_plan_->samples[i];
        s.base_x_m=p[11];s.base_z_m=p[12];s.base_vx_m_s=p[13];s.base_vz_m_s=p[14];
        s.wheel_position_rad=p[15];
        for(std::size_t j=0;j<36;++j)s.feedback_gain[j]=p[16+j];
      }
      for(std::size_t i=0;i<jump_plan_->count;++i) {
        const auto& p=jump_plan_->samples[i];
        if(p.hip_rad<pose_hip_min_rad_||p.hip_rad>pose_hip_max_rad_||
           p.knee_rad<pose_knee_min_rad_||p.knee_rad>pose_knee_max_rad_)
          throw std::invalid_argument("Optimized plan exceeds controller pose envelope");
      }
      if(std::abs(jump_plan_->samples[jump_plan_->count-1].time_s-
            (plan_durations[0]+plan_durations[1]+plan_durations[2]))>1e-4)
        throw std::invalid_argument("Optimized plan duration differs from samples");
      jump_config_.plan=jump_plan_.get();
    } else if(!plan_values.empty()||!plan_durations.empty()) {
      throw std::invalid_argument("Plan data supplied without explicit optimized-plan opt-in");
    }
    if (jump_config_.enabled &&
        (get_parameter("jump.leg_torque_limit_nm").as_double() != 5.0 ||
         get_parameter("jump.wheel_torque_limit_nm").as_double() != 0.13)) {
      throw std::invalid_argument(
          "Jump actuator profile requires jump.leg_torque_limit_nm=5.0 and "
          "jump.wheel_torque_limit_nm=0.13; lower or higher budgets are not "
          "supported by the fixed-cap Gazebo actuator plugin");
    }
    jump_contact_timeout_s_ = declare_parameter<double>("jump.contact_timeout_s", .05);
    jump_contact_topics_[0] = declare_parameter<std::string>("jump.left_contacts_topic", "left_wheel_contacts");
    jump_contact_topics_[1] = declare_parameter<std::string>("jump.right_contacts_topic", "right_wheel_contacts");
    jump_ground_collision_ = declare_parameter<std::string>("jump.ground_collision", "ground_plane");
    contact_support_surface_tokens_ = declare_parameter<std::vector<std::string>>(
        "contact.support_surface_tokens",
        std::vector<std::string>{"ground_plane", "ledge_pad",
                                 "rough_arena_spawn_pad",
                                 "parabolic_uneven_arena", "step_band_"});
    contact_reject_surface_tokens_ = declare_parameter<std::vector<std::string>>(
        "contact.reject_surface_tokens", std::vector<std::string>{"fillet"});
    contact_min_support_normal_z_ = static_cast<float>(declare_parameter<double>(
        "contact.min_support_normal_z", 0.5));
    if (!std::isfinite(contact_min_support_normal_z_) ||
        contact_min_support_normal_z_ < 0.0F ||
        contact_min_support_normal_z_ > 1.0F) {
      throw std::invalid_argument(
          "contact.min_support_normal_z must be finite and in [0,1]");
    }
    if (!std::isfinite(jump_contact_timeout_s_) || jump_contact_timeout_s_ <= 0 ||
        (jump_config_.enabled && pitch_source_ != "sim_pose"))
      throw std::invalid_argument("Jump requires sim_pose and positive contact timeout");
    jump_ = std::make_unique<robot_control::JumpController>(jump_config_);
    if (jump_config_.enabled) {
      if((jump_config_.landing_world_frame||jump_config_.smooth_phase_transitions||
          jump_config_.thrust_hip_offset_rad!=0.F)&&
          (config.hip_min_rad>-.6F||config.hip_max_rad<1.05F||
           config.knee_min_rad> -1.6F||config.knee_max_rad<.2F))
        throw std::invalid_argument("Dynamic jump trajectories require the full provisional pose envelope");
      for (float knee : {jump_config_.crouch_knee_rad,jump_config_.extension_knee_rad,
                         jump_config_.landing_knee_rad,jump_config_.landing_compression?
                         jump_config_.compression_knee_rad:jump_config_.landing_knee_rad}) {
        const auto pose=robot_control::JumpController::neutralPose(knee);
        if (pose[0]<config.hip_min_rad || pose[0]>config.hip_max_rad ||
            knee<config.knee_min_rad || knee>config.knee_max_rad)
          throw std::invalid_argument("Jump poses exceed controller profile bounds");
      }
    }
    jump_status_pub_ = create_publisher<std_msgs::msg::String>("jump_status", 10);
    actuator_enable_pub_=create_publisher<std_msgs::msg::Bool>("actuator_enable",10);
    leg_velocity_reference_pub_=create_publisher<sensor_msgs::msg::JointState>("leg_velocity_reference",10);
    jump_request_sub_ = create_subscription<std_msgs::msg::Bool>("jump_request", 10,
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
          if(msg->data && !jump_request_level_) jump_request_pending_=true;
          jump_request_level_=msg->data;
        });
    for(std::size_t side=0;side<2;++side) {
    jump_contact_subs_[side] = create_subscription<ros_gz_interfaces::msg::Contacts>(jump_contact_topics_[side],
        rclcpp::SensorDataQoS(), [this,side](ros_gz_interfaces::msg::Contacts::ConstSharedPtr msg) {
          const double stamp=stampToSeconds(msg->header.stamp);
          if(stamp<=0 || !std::isfinite(stamp) || stamp<=jump_contact_last_seen_[side] ||
             stamp>now().seconds()+kJumpContactClockToleranceS) {
            // Ignore transport replay/skew; it must not refresh OR erase a
            // valid observation. Sustained bad timestamps still age out.
            return;
          }
          jump_contact_last_seen_[side]=stamp;
          any_wheel_contact_stamps_[side]=stamp;
          // A contact sensor publishes a heartbeat even when its contact list
          // is empty. Keep that heartbeat fresh, but derive support only from
          // a validated wheel/support-surface pair. In particular, a wheel
          // touching the departure fillet, body, or another obstacle is not a
          // floor contact and must not terminate Flight.
          bool support_touching=false;
          bool jump_ground_touching=false;
          for(const auto& c:msg->contacts) {
            support_touching = support_touching ||
                               isValidatedWheelSupportContact(c, side);
            jump_ground_touching = jump_ground_touching ||
                                   isJumpGroundContact(c, side);
          }
          any_wheel_contact_[side]=support_touching;
          // Jump keeps its stricter, single-ground-surface contract. A
          // non-ground contact still counts as a live sensor heartbeat, but
          // never as jump touchdown.
          jump_contact_stamps_[side]=stamp;
          if(side==0) jump_left_contact_=jump_ground_touching;
          else jump_right_contact_=jump_ground_touching;
        });
    }
    station_enabled_ = declare_parameter<bool>("station_keeper.enabled", false);
    // Station-keeping's own x/y/yaw feedback (StationKeeper::Input, below)
    // reads last_sim_pose_x_m_/_y_m_ and state_.imu.yaw_rad directly and is
    // gated on have_sim_pose_ at every update() call - it has no dependency
    // on pitch_source_ at all (StationKeeper never reads pitch). This used
    // to hard-require pitch_source=="sim_pose" too, which was never a real
    // technical dependency, just historical coupling from when sim_pose was
    // the only pitch source that existed. That stale guard crashed
    // deep_squat.yaml and stationary_show.yaml at startup (std::invalid_
    // argument, exit -6) the moment pitch_source's default became imu
    // (2026-09-15) - found 2026-09-16 while re-running Deep Squat under the
    // new default, exactly the kind of stale-config trap already seen once
    // with the ledge landing test. World x/y/yaw position still has no
    // sensor-backed estimate (a separate, still-open Rule 1 gap - see
    // docs/RULES.md), but that gap is orthogonal to pitch and must not
    // force pitch back to ground truth too.
    robot_control::StationKeeper::Config station_config;
    station_config.position_gain_hz = declare_parameter<double>("station_keeper.position_gain_hz", 0.8);
    station_config.acquire_dwell_s = declare_parameter<double>("station_keeper.acquire_dwell_s", 0.0);
    station_config.yaw_gain_hz = declare_parameter<double>("station_keeper.yaw_gain_hz", 2.0);
    station_config.max_forward = declare_parameter<double>("station_keeper.max_forward_m_s", 0.08);
    station_config.max_yaw = declare_parameter<double>("station_keeper.max_yaw_rad_s", 0.25);
    station_keeper_ = std::make_unique<robot_control::StationKeeper>(station_config);
    cmd_vel_timeout_s_ = declare_parameter<double>("cmd_vel_timeout_s", 0.5);

    command_.request_enable = auto_enable_;
    command_.target_joint_position_rad = config.stand_up.stand_pose_rad;
    commanded_leg_pose_ = {config.stand_up.stand_pose_rad[kLeftHip],
                           config.stand_up.stand_pose_rad[kLeftKnee],
                           config.stand_up.stand_pose_rad[kRightHip],
                           config.stand_up.stand_pose_rad[kRightKnee]};

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) { onImu(*msg); });
    sim_pose_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        declare_parameter<std::string>("sim_pose_topic", "/tap_robot/pose"), rclcpp::SensorDataQoS(),
        [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
          onSimPose(*msg);
        });
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          onJointState(*msg);
        });
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {
            return;
          }
          last_cmd_vel_stamp_s_ = now().seconds();
          if (!allow_motion_commands_) {
            command_.target_forward_velocity_m_s = 0.0F;
            command_.target_yaw_rate_rad_s = 0.0F;
            return;
          }
          command_.target_forward_velocity_m_s =
              static_cast<float>(msg->linear.x);
          command_.target_yaw_rate_rad_s =
              static_cast<float>(msg->angular.z);
        });
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
        "enable", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data && jump_->phase() == robot_control::JumpController::Phase::Fault) {
            command_.request_enable = false;
            command_.request_disable = true;
            return;
          }
          command_.request_enable = msg->data;
          command_.request_disable = !msg->data;
          if (!msg->data) {
            discardPendingPose();
            command_.target_forward_velocity_m_s = 0.0F;
            command_.target_yaw_rate_rad_s = 0.0F;
          }
        });
    emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "emergency_stop", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          state_.emergency_stop = msg->data;
          if (msg->data) discardPendingPose();
        });
    stand_up_sub_ = create_subscription<std_msgs::msg::Bool>(
        "stand_up", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data && !state_.emergency_stop && allow_motion_commands_) {
            command_.request_disable = false;
            command_.request_stand_up = true;
          }
        });
    reset_sub_ = create_subscription<std_msgs::msg::Bool>(
        "reset_controller", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          reset_requested_ = msg->data;
        });
    // Target leg pose: [left_hip, left_knee, right_hip, right_knee] in rad. The
    // node slews the leg command toward it every cycle (pose_slew_rate_rad_s)
    // so a slider can drag it in real time; balance keeps running and the
    // whole-body compensation leans the torso to keep the shifting CoM caught.
    target_pose_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "target_pose", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          if (jump_->active() || jump_->phase()==robot_control::JumpController::Phase::Fault ||
              !allow_motion_commands_ || msg->data.size() != 4 || !std::all_of(msg->data.begin(), msg->data.end(),
                                                [](double v) { return std::isfinite(v); })) {
            return;
          }
          commanded_leg_pose_[0] = clampf(static_cast<float>(msg->data[0]),
                                          pose_hip_min_rad_, pose_hip_max_rad_);
          commanded_leg_pose_[1] = clampf(static_cast<float>(msg->data[1]),
                                          pose_knee_min_rad_, pose_knee_max_rad_);
          commanded_leg_pose_[2] = clampf(static_cast<float>(msg->data[2]),
                                          pose_hip_min_rad_, pose_hip_max_rad_);
          commanded_leg_pose_[3] = clampf(static_cast<float>(msg->data[3]),
                                          pose_knee_min_rad_, pose_knee_max_rad_);
        });
    wheel_command_pub_ =
        create_publisher<std_msgs::msg::Float64MultiArray>(
            "wheel_effort_controller/commands", 10);
    leg_command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "leg_effort_controller/commands", 10);
    debug_pub_ =
        create_publisher<sensor_msgs::msg::JointState>("motor_commands", 10);
    mode_pub_ = create_publisher<std_msgs::msg::String>("control_mode", 10);

    // Sim-time timer (respects use_sim_time): it does NOT fire while Gazebo is
    // paused, so the controller cannot wind up its integrator during bring-up.
    const auto period = std::chrono::duration<double>(control_period_s_);
    timer_ = rclcpp::create_timer(
        this, get_clock(),
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { onTimer(); });

    controller_->reset(state_);
  }

 private:
  bool containsWheelCollision(const std::string& name,
                              std::size_t side) const {
    const char* leaf = side == 0 ? "cad_left_wheel_link_collision"
                                 : "cad_right_wheel_link_collision";
    if (name == leaf) return true;
    const std::string suffix = std::string("::") + leaf;
    if (name.size() > suffix.size() &&
        name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
      return true;
    }
    // Gazebo Sim expands an URDF collision named
    // `cad_*_wheel_link_collision` to the scoped entity name
    // `...::cad_*_wheel_link_collision_collision`.  The extra suffix is
    // version/description-format specific, so accept it explicitly while
    // keeping the side-specific collision check strict.
    const std::string sdf_suffix = suffix + "_collision";
    return name.size() > sdf_suffix.size() &&
           name.compare(name.size() - sdf_suffix.size(), sdf_suffix.size(),
                        sdf_suffix) == 0;
  }

  bool isConfiguredSupportSurface(const std::string& name) const {
    if (name.empty()) return false;
    for (const auto& token : contact_reject_surface_tokens_) {
      if (!token.empty() && name.find(token) != std::string::npos) {
        return false;
      }
    }
    for (const auto& token : contact_support_surface_tokens_) {
      if (!token.empty() && name.find(token) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  bool hasSupportNormal(const ros_gz_interfaces::msg::Contact& contact) const {
    // Some ros_gz_bridge versions omit normals in the ROS conversion. In that
    // case the explicit support-surface name is the only available physical
    // classifier; never infer support from a merely nonempty contact list.
    if (contact.normals.empty()) return true;
    for (const auto& normal : contact.normals) {
      const double nx = normal.x;
      const double ny = normal.y;
      const double nz = normal.z;
      const double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
      if (std::isfinite(norm) && norm > 1.0e-9 &&
          std::isfinite(nz) &&
          std::abs(nz) / norm >= contact_min_support_normal_z_) {
        return true;
      }
    }
    return false;
  }

  bool isValidatedWheelSupportContact(
      const ros_gz_interfaces::msg::Contact& contact, std::size_t side) const {
    const auto& first = contact.collision1.name;
    const auto& second = contact.collision2.name;
    const bool first_wheel = containsWheelCollision(first, side);
    const bool second_wheel = containsWheelCollision(second, side);
    // Exactly one member of the pair must be this side's wheel. This rejects
    // wheel-wheel and wrong-side/body contacts even if they contain a known
    // terrain token elsewhere in their fully-qualified names.
    if (first_wheel == second_wheel) return false;
    const auto& support = first_wheel ? second : first;
    return isConfiguredSupportSurface(support) && hasSupportNormal(contact);
  }

  bool isJumpGroundContact(const ros_gz_interfaces::msg::Contact& contact,
                           std::size_t side) const {
    const auto& first = contact.collision1.name;
    const auto& second = contact.collision2.name;
    const bool first_wheel = containsWheelCollision(first, side);
    const bool second_wheel = containsWheelCollision(second, side);
    if (first_wheel == second_wheel || jump_ground_collision_.empty()) {
      return false;
    }
    const auto& support = first_wheel ? second : first;
    return support.find(jump_ground_collision_) != std::string::npos;
  }

  void onImu(const sensor_msgs::msg::Imu& msg) {
    const auto& q = msg.orientation;
    if (!std::isfinite(msg.angular_velocity.y) || !std::isfinite(msg.angular_velocity.z) ||
        !std::isfinite(msg.linear_acceleration.x) || !std::isfinite(msg.linear_acceleration.y) ||
        !std::isfinite(msg.linear_acceleration.z) || !std::isfinite(q.w) ||
        !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) {
      return;
    }
    const float pitch_rate_rad_s = static_cast<float>(msg.angular_velocity.y);
    const double stamp_s = stampToSeconds(msg.header.stamp);
    // Real elapsed time since the last IMU sample, not the nominal control-
    // loop period: onImu runs per IMU message (e.g. 500 Hz), which need not
    // match control_period_s_ (e.g. 1 kHz). Falls back to the nominal
    // period for the first sample or a bad/stale stamp.
    const float raw_dt =
        (last_imu_stamp_s_ > 0.0 && stamp_s > last_imu_stamp_s_)
            ? static_cast<float>(stamp_s - last_imu_stamp_s_)
            : static_cast<float>(control_period_s_);
    // A delayed bridge sample must not integrate a whole wall-clock gap as a
    // single gyro step. The watchdog handles sustained staleness; this clamp
    // protects the estimator during a short scheduling hiccup.
    const float dt = std::clamp(raw_dt, 1.0e-4F, 0.02F);

    raw_imu_pitch_rate_rad_s_ = pitch_rate_rad_s;
    raw_imu_yaw_rate_rad_s_ = static_cast<float>(msg.angular_velocity.z);
    // See the member declaration - an invalid sample here just skips this
    // one update rather than rejecting the whole IMU message the pitch/yaw
    // path already validated.
    if (std::isfinite(msg.angular_velocity.x)) {
      raw_imu_roll_rate_rad_s_ = static_cast<float>(msg.angular_velocity.x);
    }
    // Always updated regardless of pitch_source, mirroring
    // accel_magnitude_m_s2 below - see Types.hpp's ImuState::roll_rate_rad_s.
    state_.imu.roll_rate_rad_s = raw_imu_roll_rate_rad_s_;
    raw_imu_yaw_rad_ = yawFromQuaternion(q.w, q.x, q.y, q.z);
    // Raw (unfiltered) specific-force magnitude, independent of pitch_source_
    // and of the separate low-passed accel_*_lp_ gravity estimate below (that
    // one is deliberately smoothed to reject exactly the fast transients this
    // one exists to catch). In free fall this reads ~0 (proper acceleration
    // cancels gravity in the specific-force measurement); on impact it spikes
    // well above 1 g for one or two 500 Hz samples. This is the standard
    // free-fall/impact detection principle (the same one behind phone drop
    // sensors), applied here per the user's request to get more out of the
    // IMU specifically, and is a materially different, lower-latency signal
    // than either the sim_pose height/pitch heuristic or the hip-torque
    // threshold already tried (see docs/LEDGE_DRIVEOFF_LIMIT.md).
    last_accel_magnitude_m_s2_ = std::sqrt(
        static_cast<float>(msg.linear_acceleration.x) * static_cast<float>(msg.linear_acceleration.x) +
        static_cast<float>(msg.linear_acceleration.y) * static_cast<float>(msg.linear_acceleration.y) +
        static_cast<float>(msg.linear_acceleration.z) * static_cast<float>(msg.linear_acceleration.z));
    // Real, always-updated regardless of pitch_source - see Types.hpp's
    // ImuState::accel_magnitude_m_s2 (LandingImpedance's hard-impact gate).
    state_.imu.accel_magnitude_m_s2 = last_accel_magnitude_m_s2_;
    // Preserve a short fused acceleration/gyro event across the asynchronous
    // IMU-to-control-loop boundary. A single valid sample is enough here: the
    // event is only armed after the airborne gate in the core, and its lifetime
    // is bounded by landing_impact_latch_s_.
    // Use receipt/sim-clock time for the expiry. A valid sensor timestamp can
    // already be older than the next timer tick when the bridge is delayed;
    // using that stamp directly would erase the event before the controller
    // ever sees it.
    const double event_time_s = now().seconds();
    if (landing_impact_latch_s_ > 0.0F &&
        last_accel_magnitude_m_s2_ >= landing_impact_accel_mps2_ &&
        std::abs(pitch_rate_rad_s) >= landing_impact_pitch_rate_rad_s_) {
      pending_imu_impact_event_ = true;
      pending_imu_impact_expiry_s_ =
          event_time_s + static_cast<double>(landing_impact_latch_s_);
    }
    // A zero timestamp is not evidence that this sample is fresh. Keep the
    // watchdog fail-closed instead of replacing an invalid stamp with now().
    last_imu_stamp_s_ = stamp_s > 0.0 ? stamp_s : 0.0;
    have_imu_ = true;

    if (pitch_source_ == "sim_pose") {
      return;
    }

    // Plain accel+gyro complementary filter for a bare 6-axis IMU (no
    // onboard AHRS/fusion chip - real target hardware is an MPU6500,
    // confirmed 2026-09-15). See docs/RULES.md Rule 1's "Attempted
    // 2026-09-15" history for two earlier ideas that didn't pan out
    // (reading a fused orientation quaternion assumes hardware this robot
    // won't have; correcting the accelerometer with wheel-odometry/MPC
    // terms introduced new instabilities of their own).
    //
    // ROOT CAUSE FOUND, not guessed: instrumenting every attempt's seed
    // value against ground truth showed the FIRST one or two `/imu`
    // messages after launch arrive irregularly spaced (hundreds of ms
    // apart, not the steady ~2 ms of real 500 Hz data) with near-zero
    // linear_acceleration on all three axes - a startup artifact from the
    // Gazebo/bridge process pair, not real physics data (real 500 Hz
    // samples only start a few messages later). atan2 of two near-zero
    // numbers is numerically arbitrary, and every earlier attempt seeded
    // `complementary_pitch_rad_` from whichever message happened to be
    // first - baking a small but real garbage offset (~0.09 rad measured)
    // into the filter's persistent state right as the aggressive
    // Track-A-promoted MPC gains started reacting to it, before the slow
    // (alpha=0.9) blend had any chance to wash it out. This explains why
    // every prior fix "to the correction term" kept failing identically:
    // the correction term was never the dominant bug, the unvalidated seed
    // was.
    //
    // Fix: only seed from a sample whose specific-force magnitude is
    // physically plausible (close to 1 g) - reject early
    // startup/placeholder messages instead of trusting whichever one
    // arrives first.
    const float accel_mag = std::sqrt(
        static_cast<float>(msg.linear_acceleration.x) * static_cast<float>(msg.linear_acceleration.x) +
        static_cast<float>(msg.linear_acceleration.y) * static_cast<float>(msg.linear_acceleration.y) +
        static_cast<float>(msg.linear_acceleration.z) * static_cast<float>(msg.linear_acceleration.z));
    constexpr float kMinPlausibleAccelMag = 5.0F;   // reject near-zero startup artifacts
    constexpr float kMaxPlausibleAccelMag = 20.0F;  // reject implausible spikes as a seed
    // A single magnitude-plausible sample is not enough: measured 2026-09-15
    // that a real settling bounce right after spawn can produce one or two
    // samples with a plausible magnitude but a skewed direction (not clean
    // at-rest gravity), still seeding a wrong pitch (~0.2 rad measured).
    // Require a short run of consecutive plausible samples and seed from
    // their average instead of a single reading, so a brief transient
    // cannot dominate the seed.
    constexpr int kSeedWindowSamples = 50;  // ~100 ms at 500 Hz
    if (!imu_seeded_) {
      if (accel_mag < kMinPlausibleAccelMag || accel_mag > kMaxPlausibleAccelMag) {
        seed_window_count_ = 0;
        seed_accum_x_ = seed_accum_y_ = seed_accum_z_ = 0.0F;
        return;  // not a real sample yet; wait for one that looks like gravity
      }
      seed_accum_x_ += static_cast<float>(msg.linear_acceleration.x);
      seed_accum_y_ += static_cast<float>(msg.linear_acceleration.y);
      seed_accum_z_ += static_cast<float>(msg.linear_acceleration.z);
      ++seed_window_count_;
      if (seed_window_count_ < kSeedWindowSamples) {
        return;  // keep accumulating until the window is settled
      }
      const float n = static_cast<float>(seed_window_count_);
      accel_x_lp_ = seed_accum_x_ / n;
      accel_y_lp_ = seed_accum_y_ / n;
      accel_z_lp_ = seed_accum_z_ / n;
      complementary_pitch_rad_ = std::atan2(-accel_x_lp_, accel_z_lp_);
      // Kalman state = [angle, gyro_bias]. Bug found and fixed 2026-09-15:
      // checked this implementation against the actual canonical reference
      // (TKJElectronics/KalmanFilter, Kristian Lauszus - the design this is
      // ported from) instead of trusting memory of the algorithm alone.
      // The math matched exactly, but the INITIAL COVARIANCE didn't: this
      // code used P=1.0 ("trust the accelerometer heavily at first"), the
      // opposite of the reference's own documented intent - "we assume
      // that the bias is 0 and we KNOW the starting angle (via setAngle),
      // [so] the error covariance matrix is set like so" - i.e. P=0,
      // fully trust the seeded angle and let uncertainty grow gradually via
      // Q_angle/Q_bias, not distrust it from sample zero. With P=1.0 the
      // very first post-seed Kalman gain was already ~0.97 (S=1.0+R_measure,
      // K0=P00/S), nearly fully substituting the first raw accelerometer
      // reading for the (already-good, window-averaged) seed - reintroducing
      // exactly the single-sample fragility the seed-averaging fix was
      // built to avoid. This explains why lowering P further (tried
      // earlier, P=0.05) made things worse, not better: 1.0 was already
      // wrong in that same direction, just less so.
      kalman_bias_ = 0.0F;
      kalman_p_[0][0] = 0.0F;
      kalman_p_[0][1] = 0.0F;
      kalman_p_[1][0] = 0.0F;
      kalman_p_[1][1] = 0.0F;
      imu_seeded_ = true;
    } else {
      // Proper (linear) Kalman filter for angle+gyro-bias fusion, the
      // standard, widely-validated design for exactly this accel+gyro
      // sensor pair (state=[angle,bias], e.g. Kristian Lauszus's
      // TKJElectronics Kalman filter, used across countless real MPU6050/
      // 6500 balancing-robot projects) - replaces the earlier fixed-alpha
      // complementary blend, whose gain didn't adapt to how confident the
      // estimate actually was. A covariance-driven gain starts high (trust
      // the accelerometer) when uncertain and narrows as it converges,
      // self-correcting a still-imperfect seed far faster than a fixed
      // alpha=0.9 blend could (measured 2026-09-15: fixed-alpha took many
      // dozens of samples to meaningfully reduce a seed error).
      //
      // Feedforward correction kept from the previous attempt: subtract
      // the MPC's own just-computed commanded ground acceleration (known
      // instantly, zero measurement lag) from accel_x before treating it
      // as a pitch measurement, so the robot's own balance-correcting
      // motion isn't misread as tilt. Causally cleaner than differentiating
      // measured wheel velocity (tried and rejected earlier: that closes a
      // real actuator-lag feedback loop with the estimate).
      const float commanded_ground_accel_m_s2 =
          controller_->mpcDebugState().optimal_command_m_s;
      const float accel_x_corrected =
          static_cast<float>(msg.linear_acceleration.x) - commanded_ground_accel_m_s2;
      const float pitch_measured =
          std::atan2(-accel_x_corrected, static_cast<float>(msg.linear_acceleration.z));

      // Predict.
      const float rate = pitch_rate_rad_s - kalman_bias_;
      complementary_pitch_rad_ += dt * rate;
      kalman_p_[0][0] += dt * (dt * kalman_p_[1][1] - kalman_p_[0][1] - kalman_p_[1][0] + kalman_q_angle_);
      kalman_p_[0][1] -= dt * kalman_p_[1][1];
      kalman_p_[1][0] -= dt * kalman_p_[1][1];
      kalman_p_[1][1] += kalman_q_bias_ * dt;

      // Update. Inflate R when the total specific-force magnitude departs
      // from 1 g - evidence a real linear acceleration (squat CoM motion)
      // is present, not just tilt, so the accel-derived pitch_measured is
      // less trustworthy right now. Squared so small sensor noise near g
      // barely changes anything but a real multi-m/s^2 event (measured
      // ~1-2 m/s^2 sustained deviation during a 25% squat transition)
      // visibly discounts the measurement.
      //
      // Gated to PoseChange only (measured 2026-09-16): applying this
      // unconditionally, even at a small gain, measurably hurt plain Hold
      // (drift 0.087->0.10-0.12 m at gain>=2) - normal settling-noise accel
      // deviation during quiescent balancing got mistaken for a real
      // disturbance and pushed the filter toward the noisier gyro-only
      // path. Gating on the planner's own known mode (like the drive-
      // direction feedforward above uses the MPC's own known command)
      // targets exactly the CoM-motion case instead of reacting to
      // ambiguous accel noise everywhere.
      // Deadband before squaring: measured 2026-09-16 that gating on
      // PoseChange mode alone still let ordinary sensor-noise-level accel
      // deviations (~0.3-0.7 m/s^2, present even in quiescent Hold/startup)
      // get amplified at gain>=40, marginally regressing Hold drift over a
      // full 30s run. A real squat CoM-motion disturbance measured
      // consistently >=1 m/s^2 sustained, well above ordinary noise - only
      // count deviation past that as evidence of real motion.
      constexpr float kExpectedGravityMps2 = 9.81F;
      constexpr float kAccelDevDeadbandMps2 = 0.4F;
      const float accel_dev = accel_mag - kExpectedGravityMps2;
      const float accel_dev_excess =
          std::max(0.0F, std::fabs(accel_dev) - kAccelDevDeadbandMps2);
      const bool pose_changing =
          controller_->mode() == robot_control::ControlMode::PoseChange;
      // Extended 2026-09-16 (ledge-landing touchdown diagnosis): a fast
      // whole-body rotation - measured directly, not inferred - is the same
      // "real motion, not tilt" case PoseChange already covers, just from a
      // touchdown impact instead of a commanded squat. Confirmed by tracing
      // a real drive-off-the-ledge fall: dt_ms/jitter_ms/imu_age_ms were
      // perfect (1.0/~0/1-2, no scheduling bug) throughout, yet the
      // estimate lagged real pitch by up to 0.46 rad while pitch_rate
      // climbed 0.65->3.5+ rad/s - the accelerometer-derived pitch_measured
      // was corrupted by the fall's own uncontrolled linear acceleration
      // (the same g*sin(pitch)-a_true ambiguity behind the PoseChange
      // case), pulling the Kalman estimate away from gyro's own correct
      // trajectory during exactly the window recovery depends on.
      // kHighPitchRateRadS is set well above every normal-operation
      // pitch_rate observed this session (hold/drive/yaw stayed under
      // ~0.3 rad/s) so it does not fire during ordinary balancing.
      constexpr float kHighPitchRateRadS = 1.5F;
      const bool fast_rotation = std::fabs(pitch_rate_rad_s) > kHighPitchRateRadS;
      const bool accel_force_valid =
          accel_mag >= kMinPlausibleAccelMag &&
          accel_mag <= kMaxPlausibleAccelMag &&
          std::fabs(accel_dev) <= kalman_accel_max_deviation_mps2_ &&
          std::fabs(pitch_rate_rad_s) <= kalman_accel_max_pitch_rate_rad_s_;
      accel_tilt_measurement_valid_ = accel_force_valid;
      if (accel_force_valid) {
        const float r_effective =
            kalman_r_measure_ *
            (1.0F + ((pose_changing || fast_rotation) ? kalman_accel_trust_gain_ : 0.0F) *
                        accel_dev_excess * accel_dev_excess);
        const float innovation = pitch_measured - complementary_pitch_rad_;
        const float s = kalman_p_[0][0] + r_effective;
        const float k0 = kalman_p_[0][0] / s;
        const float k1 = kalman_p_[1][0] / s;
        complementary_pitch_rad_ += k0 * innovation;
        kalman_bias_ += k1 * innovation;
        const float p00 = kalman_p_[0][0];
        const float p01 = kalman_p_[0][1];
        kalman_p_[0][0] -= k0 * p00;
        kalman_p_[0][1] -= k0 * p01;
        kalman_p_[1][0] -= k1 * p00;
        kalman_p_[1][1] -= k1 * p01;
      }
    }

    float pitch_rad =
        complementary_pitch_rad_ - static_cast<float>(pitch_offset_rad_);
    float pitch_rate_out = pitch_rate_rad_s;
    if (imu_noise_enabled_) {
      pitch_rad += sampleNoise(static_cast<float>(imu_pitch_noise_stddev_rad_));
      pitch_rate_out +=
          sampleNoise(static_cast<float>(imu_gyro_noise_stddev_rad_s_));
    }
    state_.imu.pitch_rad = pitch_rad;
    state_.imu.yaw_rad = raw_imu_yaw_rad_;
    state_.imu.pitch_rate_rad_s = pitch_rate_out;
    state_.imu.yaw_rate_rad_s = raw_imu_yaw_rate_rad_s_;
    last_orientation_stamp_s_ = last_imu_stamp_s_;
    have_orientation_ = true;
  }

  void onSimPose(const tf2_msgs::msg::TFMessage& msg) {
    if (pitch_source_ != "sim_pose") {
      return;
    }

    last_sim_pose_count_ = msg.transforms.size();
    for (std::size_t i = 0; i < msg.transforms.size(); ++i) {
      const auto& transform = msg.transforms[i];
      const bool frame_matches =
          !transform.child_frame_id.empty() &&
          transform.child_frame_id == sim_pose_child_frame_;
      const bool index_matches =
          msg.transforms.size() == 1 && transform.child_frame_id.empty() &&
          static_cast<int>(i) == sim_pose_index_;
      if (!frame_matches && !index_matches) {
        continue;
      }

      const auto& q = transform.transform.rotation;
      const double qnorm = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
      if (!std::isfinite(qnorm) || std::abs(qnorm - 1.0) > 0.01 ||
          !std::isfinite(transform.transform.translation.x) ||
          !std::isfinite(transform.transform.translation.y) ||
          !std::isfinite(transform.transform.translation.z)) {
        continue;
      }
      float pitch_rad = basePitchFromQuaternion(q.w, q.x, q.y, q.z) -
                        static_cast<float>(pitch_offset_rad_);
      pitch_rad = normalizeAngle(pitch_rad);

      double stamp_s =
          static_cast<double>(transform.header.stamp.sec) +
          1.0e-9 * static_cast<double>(transform.header.stamp.nanosec);
      // ros_gz_bridge emits zero stamps for Pose_V transforms on this setup.
      // Unlike a stale sensor replay, the callback itself is the only
      // available freshness signal, so use the receipt time for this one
      // documented bridge exception.
      if (stamp_s <= 0.0) {
        stamp_s = now().seconds();
      }
      // Pitch RATE comes from the gyro, not from differencing this pose. The
      // imu link is fixed to the base with zero relative rotation (torso.xacro:
      // imu_fix rpy="0 0 0"), so /imu angular_velocity.y already is the clean
      // sagittal pitch rate, at the full 500 Hz IMU rate with no differencing
      // lag. dynamic_pose/info is slower and irregular, and a finite-difference
      // derivative off it adds ~30 ms of phase lag - enough to turn the
      // upstream gains into a growing oscillation at the ~1 s pendulum period.
      // (Only the pose *orientation* is unreliable on this bridge, not the
      // body rates.) A finite difference is the fallback until the IMU is up.
      float pitch_rate_rad_s = raw_imu_pitch_rate_rad_s_;
      if (!have_imu_ && have_sim_pose_ && stamp_s > last_sim_pose_stamp_s_) {
        const float dt_s =
            static_cast<float>(stamp_s - last_sim_pose_stamp_s_);
        pitch_rate_rad_s =
            normalizeAngle(pitch_rad - last_sim_pose_pitch_rad_) /
            std::max(dt_s, 1.0e-6F);
      }

      if (imu_noise_enabled_) {
        pitch_rad +=
            sampleNoise(static_cast<float>(imu_pitch_noise_stddev_rad_));
        pitch_rate_rad_s +=
            sampleNoise(static_cast<float>(imu_gyro_noise_stddev_rad_s_));
      }

      state_.imu.pitch_rad = pitch_rad;
      state_.imu.pitch_rate_rad_s = pitch_rate_rad_s;
      state_.imu.yaw_rad = yawFromQuaternion(q.w, q.x, q.y, q.z);
      state_.imu.yaw_rate_rad_s = raw_imu_yaw_rate_rad_s_;

      last_sim_pose_stamp_s_ = stamp_s;
      last_sim_pose_pitch_rad_ = pitch_rad;
      last_sim_pose_index_ = static_cast<int>(i);
      last_sim_pose_z_m_ = transform.transform.translation.z;
      last_sim_pose_x_m_ = transform.transform.translation.x;
      last_sim_pose_y_m_ = transform.transform.translation.y;
      last_orientation_stamp_s_ = stamp_s;
      have_sim_pose_ = true;
      have_orientation_ = true;
      return;
    }
  }

  float sampleNoise(float stddev) {
    if (stddev <= 0.0F) {
      return 0.0F;
    }
    std::normal_distribution<float> distribution(0.0F, stddev);
    return distribution(imu_noise_rng_);
  }

  void onJointState(const sensor_msgs::msg::JointState& msg) {
    const double stamp_s = stampToSeconds(msg.header.stamp);
    std::unordered_map<std::string, std::size_t> index_by_name;
    for (std::size_t i = 0; i < msg.name.size(); ++i) {
      index_by_name[msg.name[i]] = i;
    }

    for (const char* name : kJointNames) {
      const auto it = index_by_name.find(name);
      if (it == index_by_name.end() || it->second >= msg.position.size() ||
          it->second >= msg.velocity.size() || !std::isfinite(msg.position[it->second]) ||
          !std::isfinite(msg.velocity[it->second])) {
        return;
      }
    }
    for (std::size_t joint = 0; joint < kJointNames.size(); ++joint) {
      const auto it = index_by_name.find(kJointNames[joint]);
      if (it == index_by_name.end()) {
        continue;
      }
      state_.joints.position_rad[joint] =
          getOrDefault(msg.position, it->second);
      state_.joints.velocity_rad_s[joint] =
          getOrDefault(msg.velocity, it->second);
      state_.joints.effort_nm[joint] = getOrDefault(msg.effort, it->second);
    }

    // Both CAD wheel axes are +Y: positive speed rolls forward on either side.
    const float s = left_wheel_sign_;
    const float r = static_cast<float>(wheel_radius_m_);
    const float left_position =
        s * state_.joints.position_rad[kLeftWheel] * r;
    const float right_position =
        s * state_.joints.position_rad[kRightWheel] * r;
    const float left_velocity =
        s * state_.joints.velocity_rad_s[kLeftWheel] * r;
    const float right_velocity =
        s * state_.joints.velocity_rad_s[kRightWheel] * r;
    state_.wheel.position_m = 0.5F * (left_position + right_position);
    state_.wheel.velocity_m_s = 0.5F * (left_velocity + right_velocity);
    state_.wheel.left_position_m = left_position;
    state_.wheel.right_position_m = right_position;
    state_.wheel.left_velocity_m_s = left_velocity;
    state_.wheel.right_velocity_m_s = right_velocity;
    last_joint_state_stamp_s_ = stamp_s > 0.0 ? stamp_s : 0.0;
    have_joint_state_ = true;
  }

  void onTimer() {
    updateLoopTiming();
    if (!reset_requested_ && holdJumpWithoutElapsedTime()) return;
    // Use the real elapsed sim time between control cycles, not the nominal
    // period: if /clock is delivered in bursts the timer fires a catch-up
    // train, and integrating each of those as a full period over-counts. Clamp
    // to a sane band so one big gap or a zero-dt catch-up step cannot blow up
    // the integrators.
    const double nominal = control_period_s_;
    double dt = measured_control_dt_s_ > 0.0 ? measured_control_dt_s_ : nominal;
    dt = std::clamp(dt, 0.25 * nominal, 4.0 * nominal);
    state_.dt_s = static_cast<float>(dt);
    if (cmd_vel_timeout_s_ > 0.0 &&
        now().seconds() - last_cmd_vel_stamp_s_ > cmd_vel_timeout_s_) {
      command_.target_forward_velocity_m_s = 0.0F;
      command_.target_yaw_rate_rad_s = 0.0F;
    }

    // Slew the four leg-joint targets toward the latest /target_pose so a GUI
    // slider tracks in real time and a preset jump is still smooth. StandUp /
    // PoseChange own the target directly, so only slew in the balancing modes.
    const auto ctrl_mode = controller_->mode();
    if (jump_control_dt_s_ > 0.0 && !jump_->active() && (ctrl_mode == robot_control::ControlMode::Balance ||
        ctrl_mode == robot_control::ControlMode::Drive)) {
      const float step = pose_slew_rate_rad_s_ * static_cast<float>(dt);
      const std::array<std::size_t, 4> legs = {kLeftHip, kLeftKnee, kRightHip,
                                               kRightKnee};
      for (std::size_t i = 0; i < 4; ++i) {
        float& cur = command_.target_joint_position_rad[legs[i]];
        cur += clampf(commanded_leg_pose_[i] - cur, -step, step);
      }
    }

    if (!have_imu_ || !have_orientation_ || !have_joint_state_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting for imu, orientation source and joint_states");
      publishMode(robot_control::MotorCommand{});
      publishWheelCommand(robot_control::MotorCommand{});
      publishLegCommand(robot_control::MotorCommand{});
      return;
    }

    // Do not let the core interpret the absence of the first contact sample as
    // a real liftoff. Hold actuators off until both per-wheel streams have
    // produced at least one timestamped sample; later, a stale/empty sample is
    // handled as genuine no-contact by the fail-closed state below.
    if (pitch_source_ != "sim_pose" && !wheelContactObservationReady()) {
      publishMode(robot_control::MotorCommand{});
      publishWheelCommand(robot_control::MotorCommand{});
      publishLegCommand(robot_control::MotorCommand{});
      return;
    }

    const double control_now_s = now().seconds();
    if (pending_imu_impact_event_ &&
        control_now_s <= pending_imu_impact_expiry_s_) {
      state_.imu.hard_impact_event = true;
    } else {
      state_.imu.hard_impact_event = false;
      pending_imu_impact_event_ = false;
    }

    // Real per-wheel contact sensing (docs/RULES.md Rule 1 / Track B.1):
    // when not using sim_pose ground truth, floor_contact comes from actual
    // Gazebo contact sensor data (requires measure_contacts:=true at launch,
    // see sim.launch.py; defaults on for this reason), never from a
    // simulator-omniscient pose heuristic. Missing/stale contacts must not
    // imply grounded balance -- fail closed (not-fresh means not-grounded),
    // matching jumpContactFresh()'s own fail-closed idiom below.
    const bool left_contact_fresh = wheelContactFresh(0);
    const bool right_contact_fresh = wheelContactFresh(1);
    state_.floor_contact = pitch_source_ == "sim_pose"
        ? true
        : ((left_contact_fresh && any_wheel_contact_[0]) ||
           (right_contact_fresh && any_wheel_contact_[1]));
    // Per-side (ROLL differential impedance): sim_pose's global height/pitch
    // heuristic has no per-wheel granularity, so both sides mirror the
    // combined value there; the real per-wheel contact path already has it.
    state_.left_floor_contact = pitch_source_ == "sim_pose"
        ? state_.floor_contact : (left_contact_fresh && any_wheel_contact_[0]);
    state_.right_floor_contact = pitch_source_ == "sim_pose"
        ? state_.floor_contact : (right_contact_fresh && any_wheel_contact_[1]);
    if (pitch_source_ == "sim_pose" && have_sim_pose_) {
      state_.floor_contact =
          last_sim_pose_z_m_ >= sim_contact_min_base_z_m_ &&
          std::abs(state_.imu.pitch_rad) <= sim_contact_max_pitch_rad_;
      // Ascento-style torque-threshold contact confirmation (see the
      // torque_touchdown_enabled_ declaration above for the citation and
      // rationale). Only ever widens false->true, and only while sim_pose
      // ALREADY reads airborne, so this cannot change grounded/normal-
      // operation behavior even when enabled; disabled by default, a no-op.
      if (torque_touchdown_enabled_ && !state_.floor_contact) {
        const float hip_l_nm = std::abs(state_.joints.effort_nm[kLeftHip]);
        const float hip_r_nm = std::abs(state_.joints.effort_nm[kRightHip]);
        const float hip_l_vel = std::abs(state_.joints.velocity_rad_s[kLeftHip]);
        const float hip_r_vel = std::abs(state_.joints.velocity_rad_s[kRightHip]);
        // Require a stall signature (high commanded torque AND low actual
        // velocity), not just high torque alone -- a deliberate fast pose
        // change also saturates torque (kp=40, cap=3 Nm saturates past just
        // 0.075 rad of error) while genuinely moving, which must not be
        // mistaken for hitting an obstruction.
        if (std::max(hip_l_nm, hip_r_nm) >= hip_touchdown_torque_nm_ &&
            std::max(hip_l_vel, hip_r_vel) <= hip_touchdown_max_velocity_rad_s_) {
          state_.floor_contact = true;
        }
      }
      // Raw-accelerometer free-fall/impact confirmation (see
      // last_accel_magnitude_m_s2_ / onImu for the citation and rationale).
      // Same widen-only-while-airborne safety pattern; off by default.
      if (imu_touchdown_enabled_ && !state_.floor_contact &&
          last_accel_magnitude_m_s2_ >= imu_touchdown_accel_mps2_) {
        state_.floor_contact = true;
      }
      state_.left_floor_contact = state_.floor_contact;
      state_.right_floor_contact = state_.floor_contact;
    }
    if(jump_config_.enabled) {
      // Missing contacts may hold posture, but never imply grounded balance.
      state_.floor_contact=jumpContactFresh()&&(jump_left_contact_||jump_right_contact_);
      state_.left_floor_contact=jumpContactFresh()&&jump_left_contact_;
      state_.right_floor_contact=jumpContactFresh()&&jump_right_contact_;
    }
    // Fused hard-impact settle delay (see hard_impact_delay_enabled_
    // declaration above for the citation and rationale). It is allowed to
    // start only from a validated wheel/support contact in the real-sensor
    // path. An accelerometer/gyro shock while airborne is therefore telemetry,
    // not permission to manufacture an airborne/grounded transition. The
    // override keeps per-side contact consistent with the combined contract.
    if (hard_impact_delay_enabled_) {
      const bool hard_impact_now =
          last_accel_magnitude_m_s2_ >= hard_impact_accel_mps2_ &&
          std::abs(state_.imu.pitch_rate_rad_s) >= hard_impact_pitch_rate_rad_s_;
      const bool validated_support_contact = pitch_source_ != "sim_pose" &&
                                             state_.floor_contact;
      if (validated_support_contact && hard_impact_now) {
        hard_impact_settle_remaining_s_ = hard_impact_settle_s_;
      } else if (validated_support_contact &&
                 hard_impact_settle_remaining_s_ > 0.0F) {
        hard_impact_settle_remaining_s_ =
            std::max(0.0F, hard_impact_settle_remaining_s_ - state_.dt_s);
      } else if (!validated_support_contact) {
        // A shock after leaving the ledge must not leave a stale delay armed
        // until the next unrelated touchdown.
        hard_impact_settle_remaining_s_ = 0.0F;
      }
      if (hard_impact_settle_remaining_s_ > 0.0F) {
        state_.floor_contact = false;
        state_.left_floor_contact = false;
        state_.right_floor_contact = false;
      }
    }

    if(jump_config_.enabled && jump_wheel_frame_compensation_) {
      wheel_frame_=robot_control::WheelFrame::observe(state_.joints,state_.imu.pitch_rad,
          state_.imu.pitch_rate_rad_s,static_cast<float>(wheel_radius_m_));
      if(wheel_frame_.valid) {
        // Keep the raw per-side encoder channels for TrajectoryObserver.  The
        // compensated axle value is still the balance/jump input, but assigning
        // the aggregate wholesale would erase the observer's encoder evidence.
        const float left_position = state_.wheel.left_position_m;
        const float right_position = state_.wheel.right_position_m;
        const float left_velocity = state_.wheel.left_velocity_m_s;
        const float right_velocity = state_.wheel.right_velocity_m_s;
        state_.wheel = wheel_frame_.axle;
        state_.wheel.left_position_m = left_position;
        state_.wheel.right_position_m = right_position;
        state_.wheel.left_velocity_m_s = left_velocity;
        state_.wheel.right_velocity_m_s = right_velocity;
      }
      else { command_.request_enable=false;command_.request_disable=true; }
    }

    // Sensor-freshness watchdog with debounce. A single late /imu, /dynamic_pose
    // or /joint_states message (common when the Gazebo GUI or the dashboard
    // steals CPU) must NOT kill a balancing run: keep driving the loop off the
    // last known state for up to `sensor_stale_grace_cycles_` control cycles.
    // Only a sustained loss latches the safe-stop.
    if (!sensorDataFresh(now().seconds())) {
      if (jump_control_dt_s_ > 0.0) ++stale_cycle_count_;
    } else {
      stale_cycle_count_ = 0;
    }
    if (stale_cycle_count_ > sensor_stale_grace_cycles_) {
      discardPendingPose();
      if(jump_->active()) stopJump("sensor_watchdog");
      station_keeper_->reset();
      station_output_ = {};
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Sensor data stale for %d cycles: imu=%.1f ms, orientation=%.1f ms, "
          "joint_states=%.1f ms",
          stale_cycle_count_,
          1000.0 * (now().seconds() - last_imu_stamp_s_),
          1000.0 * (now().seconds() - last_orientation_stamp_s_),
          1000.0 * (now().seconds() - last_joint_state_stamp_s_));
      // Latch the safe state. A local stop command would leave the real
      // request_enable flag set and could re-enable the robot automatically
      // as soon as a delayed sensor message arrives.
      command_.request_enable = false;
      command_.request_disable = true;
      command_.target_forward_velocity_m_s = 0.0F;
      command_.target_yaw_rate_rad_s = 0.0F;
      const robot_control::MotorCommand motors =
          controller_->update(state_, command_);
      publishWheelCommand(motors);
      publishLegCommand(motors);
      publishDebugCommand(motors);
      publishMode(motors);
      return;
    }

    if (reset_requested_) {
      discardPendingPose(true);
      jump_->reset();jump_output_={};jump_request_pending_=false;resetJumpContacts();
      station_keeper_->reset();
      station_output_ = {};
      const bool reenable = command_.request_enable && !command_.request_disable &&
                            !state_.emergency_stop;
      controller_->reset(state_);
      // Disabled is already motor-off; a held disable would recreate EStop
      // immediately and prevent explicit recovery after clearing the stop.
      command_.request_disable = false;
      command_.request_enable = reenable;
      imu_seeded_ = false;  // re-seed the pitch filter from a fresh, validated sample
      seed_window_count_ = 0;
      seed_accum_x_ = seed_accum_y_ = seed_accum_z_ = 0.0F;
      pending_imu_impact_event_ = false;
      pending_imu_impact_expiry_s_ = 0.0;
      state_.imu.hard_impact_event = false;
      reset_requested_ = false;
    }

    if (holdJumpWithoutElapsedTime()) return;

    // Preserve operator intent separately: position corrections must not look
    // like fresh drive commands or move the anchor they are trying to restore.
    const bool was_jumping=jump_->active();
    if(jump_config_.enabled) {
      auto in=jumpInput();
      in.request=jump_request_pending_;jump_request_pending_=false;
      jump_output_=jump_->update(in);
      if(jump_output_.stop_motors) {command_.request_disable=true;discardPendingPose();}
      if(was_jumping && jump_output_.phase==robot_control::JumpController::Phase::Complete) {
        command_.target_joint_position_rad=jump_output_.position_rad;
        commanded_leg_pose_={jump_output_.position_rad[0],jump_output_.position_rad[1],
                             jump_output_.position_rad[3],jump_output_.position_rad[4]};
      }
      if(jump_->active()) state_.floor_contact=jump_left_contact_||jump_right_contact_;
    } else jump_request_pending_=false;
    robot_control::ControlCommand effective_command = command_;
    if(jump_output_.owns_pose) {
      effective_command.target_joint_position_rad=jump_output_.position_rad;
      effective_command.request_pose_change=false;
      effective_command.target_forward_velocity_m_s=0;
      effective_command.target_yaw_rate_rad_s=0;
    }
    robot_control::StationKeeper::Input station_input;
    station_input.x = last_sim_pose_x_m_;
    station_input.y = last_sim_pose_y_m_;
    station_input.yaw = state_.imu.yaw_rad;
    station_input.dt = state_.dt_s;
    station_input.ready_to_acquire=std::abs(state_.imu.pitch_rad-controller_->balancePitchRef())<.05F &&
        std::abs(state_.imu.pitch_rate_rad_s)<.3F;
    for(auto j:{kLeftHip,kLeftKnee,kRightHip,kRightKnee})
      station_input.ready_to_acquire=station_input.ready_to_acquire&&std::abs(state_.joints.velocity_rad_s[j])<.3F;
    const auto previous_mode = controller_->mode();
    station_input.active = station_enabled_ && have_sim_pose_ &&
        state_.floor_contact &&
        !state_.emergency_stop && !command_.request_disable && !command_.request_stand_up &&
        (previous_mode == robot_control::ControlMode::Balance ||
         previous_mode == robot_control::ControlMode::Drive ||
         previous_mode == robot_control::ControlMode::PoseChange);
    station_input.operator_driving = std::abs(command_.target_forward_velocity_m_s) > 1e-4F ||
                                     std::abs(command_.target_yaw_rate_rad_s) > 1e-4F;
    if (jump_output_.owns_pose && (!state_.floor_contact || jump_output_.station_blend<=0.F)) {
      // Freeze the existing world anchor across takeoff and landing.
      station_output_={};
    } else if (!station_input.active) {
      station_output_ = station_keeper_->update(station_input);
      station_recapture_pending_ = false;
    } else if (!sensorDataFresh(now().seconds())) {
      // During watchdog grace, suppress correction without forgetting the
      // anchor. Driving through that gap must instead capture a fresh pose.
      station_output_ = {};
      station_recapture_pending_ = station_recapture_pending_ || station_input.operator_driving;
    } else {
      if (station_recapture_pending_) station_keeper_->reset();
      station_recapture_pending_ = false;
      station_output_ = station_keeper_->update(station_input);
    }
    if(jump_output_.owns_pose) {
      station_output_.forward_correction_m_s*=jump_output_.station_blend;
      station_output_.yaw_correction_rad_s*=jump_output_.station_blend;
    }
    effective_command.target_forward_velocity_m_s += station_output_.forward_correction_m_s;
    effective_command.target_yaw_rate_rad_s += station_output_.yaw_correction_rad_s;
    robot_control::MotorCommand motors =
        controller_->update(state_, effective_command);
    motion_started_=motion_started_||motors.enable_motors;
    if(jump_output_.owns_pose && motors.enable_motors) {
      for(auto j:{kLeftHip,kLeftKnee,kRightHip,kRightKnee}) {
        motors.position_rad[j]=jump_output_.position_rad[j];
        motors.torque_nm[j]=jump_config_.max_leg_torque_nm;
        motors.kp_scale[j]=jump_output_.leg_kp_scale;
        if(j==kLeftKnee || j==kRightKnee)
          motors.kp_scale[j]*=jump_output_.knee_kp_scale[j==kLeftKnee?0:1];
      }
      for(auto j:{kLeftWheel,kRightWheel}) {
        motors.torque_nm[j]=jump_config_.max_wheel_torque_nm;
        if(!jump_output_.balance_wheels) motors.velocity_rad_s[j]=0;
        if(jump_output_.override_wheel_speed) motors.velocity_rad_s[j]=jump_output_.wheel_speed_rad_s;
      }
    }

    // Falling or entering e-stop is a latched stop. Re-enabling must come
    // from a fresh /enable message (or an explicit controller reset), never
    // from the previous enable request surviving the transition.
    const auto mode = controller_->mode();
    if (mode == robot_control::ControlMode::Fallen ||
        mode == robot_control::ControlMode::EmergencyStop) {
      // Legacy Fallen safe-stops the legs, so discarding to measured joints is
      // fine. Landing experiments with keep_legs_active_when_fallen keep the
      // legs live; repeatedly discarding there makes the stance target chase
      // impact-distorted joints instead of remaining a suspension setpoint.
      if (mode == robot_control::ControlMode::EmergencyStop ||
          !keep_legs_active_when_fallen_) {
        discardPendingPose();
      }
      if(jump_->active()) stopJump("controller_fault");
      station_keeper_->reset();
      station_output_ = {};
      command_.request_enable = false;
      command_.request_disable = mode == robot_control::ControlMode::EmergencyStop;
      command_.target_forward_velocity_m_s = 0.0F;
      command_.target_yaw_rate_rad_s = 0.0F;
    }

    // The stand-up request is a trigger: once the recovery owns the robot,
    // clear it so the controller can hand back to balance when it settles.
    if (controller_->mode() == robot_control::ControlMode::StandingUp) {
      command_.request_stand_up = false;
    }

    publishWheelCommand(motors);
    publishLegCommand(motors);
    publishDebugCommand(motors);
    publishMode(motors);
  }

  static double stampToSeconds(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec) +
           1.0e-9 * static_cast<double>(stamp.nanosec);
  }

  void discardPendingPose(bool explicit_reset=false) {
    // The paused-startup Disable handshake must retain the configured stand
    // pose. Only real motion ownership or explicit Reset invalidates its target.
    if ((!motion_started_&&!explicit_reset)||!have_joint_state_) return;
    for (std::size_t i=0;i<4;++i) {
      const std::size_t joint=i<2?i:i+1;
      const float measured=state_.joints.position_rad[joint];
      const float position=std::isfinite(measured)?measured:0.F;
      commanded_leg_pose_[i]=clampf(position,i%2?pose_knee_min_rad_:pose_hip_min_rad_,
                                   i%2?pose_knee_max_rad_:pose_hip_max_rad_);
      command_.target_joint_position_rad[joint]=commanded_leg_pose_[i];
    }
    command_.request_pose_change=false;
  }

  bool posePendingForJump() const {
    if(command_.request_pose_change || controller_->mode()==robot_control::ControlMode::PoseChange) return true;
    static const auto neutral=robot_control::JumpController::neutralPose(0);
    for(std::size_t i=0;i<4;++i) {
      const std::size_t joint=i<2?i:i+1;
      if(std::abs(commanded_leg_pose_[i]-neutral[joint])>1e-3F ||
         std::abs(command_.target_joint_position_rad[joint]-commanded_leg_pose_[i])>1e-4F) return true;
    }
    return false;
  }

  robot_control::JumpController::Input jumpInput() const {
    robot_control::JumpController::Input in;
    in.dt_s=static_cast<float>(jump_control_dt_s_);
    const auto mode=controller_->mode();
    in.eligible=(mode==robot_control::ControlMode::Balance || mode==robot_control::ControlMode::Drive ||
                 mode==robot_control::ControlMode::PoseChange) && (jump_->active() || !posePendingForJump());
    in.stop=state_.emergency_stop||command_.request_disable||command_.request_stand_up;
    in.sensors_fresh=sensorDataFresh(now().seconds());in.contacts_fresh=jumpContactFresh();
    in.left_contact=jump_left_contact_;in.right_contact=jump_right_contact_;
    in.operator_driving=std::abs(command_.target_forward_velocity_m_s)>1e-4F ||
                        std::abs(command_.target_yaw_rate_rad_s)>1e-4F;
    in.pitch_rad=state_.imu.pitch_rad;in.pitch_rate_rad_s=state_.imu.pitch_rate_rad_s;
    in.position_rad=state_.joints.position_rad;in.velocity_rad_s=state_.joints.velocity_rad_s;
    in.com_z_m=jumpEstimatedComZ();
    in.body_x_m=last_sim_pose_x_m_;in.body_z_m=last_sim_pose_z_m_;
    in.ground_speed_m_s=state_.wheel.velocity_m_s;
    return in;
  }

  bool holdJumpWithoutElapsedTime() {
    if (!jump_config_.enabled || jump_control_dt_s_ > 0.) return false;
    // Repeated /clock samples must not advance the sequencer, contact debounce,
    // station/MPC integrators or consume a pending request. Safety is immediate.
    bool finite = std::isfinite(state_.imu.pitch_rad) &&
        std::isfinite(state_.imu.pitch_rate_rad_s) && std::isfinite(jumpEstimatedComZ());
    for (float q : state_.joints.position_rad) finite = finite && std::isfinite(q);
    for (float v : state_.joints.velocity_rad_s) finite = finite && std::isfinite(v);
    if (jump_->active()) {
      if (const char* fault=jump_->safetyFault(jumpInput(),false)) stopJump(fault);
    }
    if (jump_->phase() == robot_control::JumpController::Phase::Fault ||
        state_.emergency_stop || command_.request_disable ||
        (have_imu_ && have_orientation_ && have_joint_state_ &&
         (!finite || !sensorDataFresh(now().seconds())))) {
      command_.request_enable = false;
      command_.request_disable = true;
      discardPendingPose();
      command_.target_forward_velocity_m_s = 0.F;
      command_.target_yaw_rate_rad_s = 0.F;
      const auto motors = controller_->update(state_, command_);
      publishWheelCommand(motors);
      publishLegCommand(motors);
      publishDebugCommand(motors);
      publishMode(motors);
    } else {
      // No new actuator trajectory sample; the plugin retains its own watchdog.
      publishJumpStatus();
    }
    return true;
  }

  void updateLoopTiming() {
    const double timer_now_s = now().seconds();
    jump_control_dt_s_ = last_timer_stamp_s_ > 0.0 && timer_now_s > last_timer_stamp_s_
        ? timer_now_s - last_timer_stamp_s_ : 0.0;
    if (last_timer_stamp_s_ > 0.0 && timer_now_s < last_timer_stamp_s_ - 1.0e-6) {
      discardPendingPose();
      command_.request_enable = false;
      command_.request_disable = true;
      command_.target_forward_velocity_m_s = 0.0F;
      command_.target_yaw_rate_rad_s = 0.0F;
      controller_->reset(state_);
      if(jump_->active()) stopJump("clock_rewind");
      jump_request_pending_=false;
      resetJumpContacts();
      pending_imu_impact_event_ = false;
      pending_imu_impact_expiry_s_ = 0.0;
      state_.imu.hard_impact_event = false;
      station_keeper_->reset();
      station_output_ = {};
      measured_control_dt_s_ = control_period_s_;
    }
    if (last_timer_stamp_s_ > 0.0 && timer_now_s > last_timer_stamp_s_) {
      measured_control_dt_s_ = timer_now_s - last_timer_stamp_s_;
      control_jitter_s_ =
          measured_control_dt_s_ - static_cast<double>(control_period_s_);
    }
    last_timer_stamp_s_ = timer_now_s;
  }

  bool sensorDataFresh(double now_s) const {
    if (sensor_timeout_s_ <= 0.0) {
      return true;
    }
    return stampFresh(last_imu_stamp_s_, now_s) &&
           stampFresh(last_orientation_stamp_s_, now_s) &&
           stampFresh(last_joint_state_stamp_s_, now_s);
  }

  bool jumpContactFresh() const {
    for(double stamp:jump_contact_stamps_) {
      const double age=now().seconds()-stamp;
      if(!(stamp>0 && age>=-kJumpContactClockToleranceS && age<=jump_contact_timeout_s_)) return false;
    }
    return true;
  }

  bool anyWheelContactFresh() const {
    return wheelContactFresh(0) || wheelContactFresh(1);
  }

  bool wheelContactFresh(std::size_t side) const {
    if (side >= any_wheel_contact_stamps_.size()) return false;
    const double stamp = any_wheel_contact_stamps_[side];
    const double age = now().seconds() - stamp;
    return stamp > 0.0 && age >= -kJumpContactClockToleranceS &&
           age <= jump_contact_timeout_s_;
  }

  bool wheelContactObservationReady() const {
    // Both streams must initialize before enabling. Once initialized, contact
    // is deliberately an OR: the wheel that touches first is a valid
    // touchdown event, while its per-side freshness remains independently
    // visible to the landing controller.
    return any_wheel_contact_stamps_[0] > 0.0 &&
           any_wheel_contact_stamps_[1] > 0.0;
  }

  void resetJumpContacts() {
    jump_contact_stamps_.fill(0);jump_contact_last_seen_.fill(0);
    jump_left_contact_=jump_right_contact_=false;
    any_wheel_contact_.fill(false);any_wheel_contact_stamps_.fill(0);
  }

  double jumpEstimatedComZ() const {
    const auto& q=state_.joints.position_rad;
    const auto wb=robot_control::WholeBodyModel{}.compute(q[0],q[1],q[3],q[4]);
    double ax=0,az=0;
    for(int side=0;side<2;++side) {
      const auto& hip=robot_control::geometry::origins[3*side];
      const auto& knee=robot_control::geometry::origins[3*side+1];
      const auto& wheel=robot_control::geometry::origins[3*side+2];
      const double h=q[3*side],k=q[3*side+1];
      ax+=(hip.x+std::cos(h)*knee.x+std::sin(h)*knee.z+
           std::cos(h+k)*wheel.x+std::sin(h+k)*wheel.z)/2;
      az+=(hip.z-std::sin(h)*knee.x+std::cos(h)*knee.z-
           std::sin(h+k)*wheel.x+std::cos(h+k)*wheel.z)/2;
    }
    // Sagittal telemetry estimate only; harness owns full world COM validation.
    const double pitch=state_.imu.pitch_rad;
    return last_sim_pose_z_m_-std::sin(pitch)*(ax+wb.com_offset_x_m)+
        std::cos(pitch)*(az+wb.com_above_axle_m);
  }

  void stopJump(const char* reason) {
    discardPendingPose();
    jump_->abort(reason);jump_output_.phase=robot_control::JumpController::Phase::Fault;
    jump_output_.reason=reason;jump_output_.owns_pose=false;jump_output_.stop_motors=true;
    jump_output_.ready=false;
    jump_request_pending_=false;
  }

  void publishJumpStatus() {
    if(!jump_status_pub_) return;
    std::ostringstream text;
    text<<robot_control::JumpController::phaseName(jump_->phase())
        <<"|phase="<<robot_control::JumpController::phaseName(jump_->phase())
        <<"|reason="<<jump_output_.reason<<"|enabled="<<jump_config_.enabled
        <<"|active="<<jump_->active()<<"|ready="<<(jump_output_.ready&&!posePendingForJump()&&jumpContactFresh()&&sensorDataFresh(now().seconds()))
        <<"|contact_fresh="<<jumpContactFresh()
        <<"|pose_pending="<<posePendingForJump()
        <<"|left_contact="<<jump_left_contact_<<"|right_contact="<<jump_right_contact_
        <<"|estimated_com_z="<<jumpEstimatedComZ()<<"|takeoff_com_z="<<jump_output_.takeoff_com_z_m
        <<"|apex_gain="<<jump_output_.apex_gain_m<<"|airborne_seen="<<jump_output_.airborne_seen
        <<"|phase_elapsed="<<jump_output_.phase_elapsed_s<<"|profile="<<jump_profile_
        <<"|speed_limit_rad_s="<<jump_config_.max_joint_speed_rad_s;
    text<<"|optimized_plan="<<(jump_config_.plan!=nullptr)
        <<"|plan_elapsed_s="<<jump_output_.plan_elapsed_s;
    text<<"|wheel_frame_compensation="<<(jump_config_.enabled&&jump_wheel_frame_compensation_)
        <<"|wheel_frame_valid="<<wheel_frame_.valid
        <<"|carrier_left_rad_s="<<wheel_frame_.carrier_rate[0]
        <<"|carrier_right_rad_s="<<wheel_frame_.carrier_rate[1];
    std_msgs::msg::String msg;msg.data=text.str();jump_status_pub_->publish(msg);
  }

  bool stampFresh(double stamp_s, double now_s) const {
    if (stamp_s <= 0.0) {
      return false;
    }
    const double age_s = now_s - stamp_s;
    // Decimal millisecond stamps can subtract just below the -1 ms boundary.
    return age_s >= -1.0e-3 - 1.0e-9 && age_s <= sensor_timeout_s_;
  }

  void publishWheelCommand(const robot_control::MotorCommand& motors) {
    if(jump_config_.enabled) {
      sensor_msgs::msg::JointState reference;
      reference.header.stamp=now();
      for(auto j:{kLeftHip,kLeftKnee,kRightHip,kRightKnee}) {
        reference.name.emplace_back(kJointNames[j]);
        reference.velocity.push_back(motors.enable_motors&&jump_output_.owns_pose?
            jump_output_.velocity_rad_s[j]:0.F);
      }
      leg_velocity_reference_pub_->publish(reference);
      std_msgs::msg::Bool gate;gate.data=motors.enable_motors;
      actuator_enable_pub_->publish(gate);
    }
    // Gazebo wheel velocity command: send qd_ref directly so the servo closes
    // inside the physics update and rejects ground-reaction torque without a
    // delayed /joint_states -> effort feedback loop.
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(2);
    const std::array<std::size_t, 2> wheels = {kLeftWheel, kRightWheel};
    for (std::size_t i = 0; i < wheels.size(); ++i) {
      const std::size_t j = wheels[i];
      float qd_ref =
          motors.enable_motors ? motors.velocity_rad_s[j] : 0.0F;
      if(motors.enable_motors && jump_config_.enabled && jump_wheel_frame_compensation_ && wheel_frame_.valid &&
         !(jump_output_.owns_pose&&jump_output_.override_wheel_speed&&jump_output_.wheel_speed_is_relative)) {
        const bool grounded=jumpContactFresh()&&(i==0?jump_left_contact_:jump_right_contact_);
        qd_ref=robot_control::WheelFrame::relativeCommand(qd_ref,wheel_frame_.carrier_rate[i],grounded);
      }
      msg.data[i] = static_cast<double>(qd_ref);
      last_wheel_command_rad_s_[i] = msg.data[i];
    }
    wheel_command_pub_->publish(msg);
  }

  void publishLegCommand(const robot_control::MotorCommand& motors) {
    // Position effort plus optional jump feedforward. The bounded jump plugin
    // closes velocity tracking inside physics; the non-jump model uses local
    // joint damping (motors.kd_scale, live only inside the landing window -
    // see MpcBalancer::Config::landing_kd_boost - so no delayed -kd*qd term
    // fights normal balance/squat/drive tracking). Do not duplicate delayed
    // -kd*qd feedback in the jump path below.
    // Zero position stiffness alone does not disable damping/feedforward:
    // torque-off in the bounded model is enforced by actuator_enable.
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(4);
    const std::array<std::size_t, 4> leg = {kLeftHip, kLeftKnee, kRightHip,
                                            kRightKnee};
    for (std::size_t i = 0; i < 4; ++i) {
      const std::size_t j = leg[i];
      const float q = state_.joints.position_rad[j];
      const float qd = state_.joints.velocity_rad_s[j];
      const float q_ref =
          motors.enable_motors ? motors.position_rad[j] : q;
      const float kp = leg_kp_ * motors.kp_scale[j];
      const float kd = leg_kd_ * motors.kd_scale[j];
      float tau = kp * (q_ref - q) - kd * qd + motors.feedforward_nm[j];
      if(jump_output_.owns_pose && motors.enable_motors) {
        tau+=jump_output_.feedforward_nm[j];
        const float speed=state_.joints.velocity_rad_s[j];
        if(std::abs(speed)>=jump_config_.max_joint_speed_rad_s && tau*speed>0) tau=0;
        if(jump_profile_=="manual_curve_experimental" && tau*speed>0) {
          const float cap=jump_config_.max_leg_torque_nm*std::clamp(
              (28.274334F-std::abs(speed))/(28.274334F-19.896753F),0.F,1.F);
          tau=std::clamp(tau,-cap,cap);
        }
      }
      const float lim = std::max(motors.torque_nm[j], 0.1F);
      msg.data[i] = static_cast<double>(clampf(tau, -lim, lim));
    }
    leg_command_pub_->publish(msg);
  }

  void publishDebugCommand(const robot_control::MotorCommand& motors) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    for (const char* name : kJointNames) {
      msg.name.emplace_back(name);
    }
    msg.position.assign(motors.position_rad.begin(), motors.position_rad.end());
    msg.velocity.assign(motors.velocity_rad_s.begin(),
                        motors.velocity_rad_s.end());
    msg.effort.assign(motors.torque_nm.begin(), motors.torque_nm.end());
    debug_pub_->publish(msg);
  }

  void publishMode(const robot_control::MotorCommand& motors) {
    publishJumpStatus();
    const auto mode = controller_->mode();
    const double now_s = now().seconds();
    const double orientation_age_ms =
        last_orientation_stamp_s_ > 0.0 ? 1000.0 * (now_s - last_orientation_stamp_s_)
                                        : -1.0;
    const double imu_age_ms =
        last_imu_stamp_s_ > 0.0 ? 1000.0 * (now_s - last_imu_stamp_s_) : -1.0;
    const double joint_age_ms =
        last_joint_state_stamp_s_ > 0.0
            ? 1000.0 * (now_s - last_joint_state_stamp_s_)
            : -1.0;

    // Compact machine-readable status: the GUI parses this to show exactly
    // where the drive chain stands.
    const bool fresh = sensorDataFresh(now_s);
    const robot_control::MpcBalancer::DebugState mpc_debug =
        controller_->mpcDebugState();
    const char* landing_phase = "Grounded";
    switch (controller_->landingPhase()) {
      case robot_control::LandingController::Phase::Grounded:
        landing_phase = "Grounded";
        break;
      case robot_control::LandingController::Phase::Flight:
        landing_phase = "Flight";
        break;
      case robot_control::LandingController::Phase::Impact:
        landing_phase = "Impact";
        break;
      case robot_control::LandingController::Phase::Recovery:
        landing_phase = "Recovery";
        break;
    }
    const auto flight = controller_->flightEstimate();
    const auto trajectory = controller_->trajectoryEstimate();
    char buffer[2048];
    const int written = std::snprintf(
        buffer, sizeof(buffer),
        "%s|pitch=%.3f|pitch_rate=%.3f|cmd_v=%.2f|cmd_w=%.2f|enable=%d|estop=%d|imu=%d|joints=%d"
        "|dt_ms=%.2f|jitter_ms=%.2f|imu_age_ms=%.1f|ori_age_ms=%.1f|js_age_ms=%.1f"
        "|fresh=%d|stale_n=%d|timeout_ms=%.0f|pose_idx=%d|pose_n=%zu|pose_z=%.3f"
        "|contact=%d|left_contact=%d|right_contact=%d|contact_fresh=%d|impact_event=%d"
        "|accel_mag=%.2f|accel_tilt_valid=%d|roll_rate=%.4f"
        "|wheel_pos=%.3f|wheel_vel=%.3f|l_qd=%.2f|r_qd=%.2f"
        "|l_qd_meas=%.2f|r_qd_meas=%.2f|l_qd_cmd=%.2f|r_qd_cmd=%.2f"
        "|l_tau_meas=%.2f|r_tau_meas=%.2f"
        "|l_hip=%.3f|l_knee=%.3f|kp_scale=%.2f|l_knee_kp_scale=%.2f"
        "|landing_phase=%s|landing_ff_lh=%.3f|landing_ff_lk=%.3f|landing_ff_rh=%.3f|landing_ff_rk=%.3f"
        "|flight_phase=%s|flight_airborne=%d|flight_ballistic=%d|flight_collision=%d|flight_touchdown=%d"
        "|support_loss_ms=%.1f|flight_time_ms=%.1f|vz_down=%.3f|freefall_conf=%.2f|collision_conf=%.2f"
        "|obs_valid=%d|obs_x=%.4f|obs_y=%.4f|obs_v=%.4f|obs_w=%.4f|obs_yaw=%.4f|obs_conf=%.2f|obs_stationary=%d"
        "|landing_v=%.4f|landing_v_observer=%d"
        "|ctrl=mpc|mpc_cmd=%.3f|mpc_end_pitch=%.3f|mpc_cmd_v=%.3f|mpc_iters=%d"
        "|wb_dx=%.4f|wb_h=%.3f|pitch_ref=%.4f|pose_tgt=%.2f,%.2f"
        "|station=%d|anchor_ex=%.4f|anchor_ey=%.4f|anchor_yaw=%.4f|station_v=%.4f|station_w=%.4f"
        "|pose_hip_max=%.6f|pose_knee_min=%.6f|pose_hip_min=%.6f|pose_knee_max=%.6f",
        robot_control::toString(mode), static_cast<double>(state_.imu.pitch_rad),
        static_cast<double>(state_.imu.pitch_rate_rad_s),
        static_cast<double>(command_.target_forward_velocity_m_s),
        static_cast<double>(command_.target_yaw_rate_rad_s),
        command_.request_enable ? 1 : 0, state_.emergency_stop ? 1 : 0,
        have_imu_ ? 1 : 0, have_joint_state_ ? 1 : 0,
        1000.0 * measured_control_dt_s_, 1000.0 * control_jitter_s_,
        imu_age_ms, orientation_age_ms, joint_age_ms,
        fresh ? 1 : 0, stale_cycle_count_, 1000.0 * sensor_timeout_s_,
        last_sim_pose_index_,
        last_sim_pose_count_, last_sim_pose_z_m_, state_.floor_contact ? 1 : 0,
        state_.left_floor_contact ? 1 : 0,
        state_.right_floor_contact ? 1 : 0,
        anyWheelContactFresh() ? 1 : 0,
        state_.imu.hard_impact_event ? 1 : 0,
        static_cast<double>(last_accel_magnitude_m_s2_),
        accel_tilt_measurement_valid_ ? 1 : 0,
        static_cast<double>(raw_imu_roll_rate_rad_s_),
        static_cast<double>(state_.wheel.position_m),
        static_cast<double>(state_.wheel.velocity_m_s),
        static_cast<double>(motors.velocity_rad_s[kLeftWheel]),
        static_cast<double>(motors.velocity_rad_s[kRightWheel]),
        static_cast<double>(state_.joints.velocity_rad_s[kLeftWheel]),
        static_cast<double>(state_.joints.velocity_rad_s[kRightWheel]),
        last_wheel_command_rad_s_[0], last_wheel_command_rad_s_[1],
        static_cast<double>(state_.joints.effort_nm[kLeftWheel]),
        static_cast<double>(state_.joints.effort_nm[kRightWheel]),
        static_cast<double>(state_.joints.position_rad[kLeftHip]),
        static_cast<double>(state_.joints.position_rad[kLeftKnee]),
        static_cast<double>(motors.kp_scale[kLeftHip]),
        static_cast<double>(motors.kp_scale[kLeftKnee]),
        landing_phase,
        static_cast<double>(motors.feedforward_nm[kLeftHip]),
        static_cast<double>(motors.feedforward_nm[kLeftKnee]),
        static_cast<double>(motors.feedforward_nm[kRightHip]),
        static_cast<double>(motors.feedforward_nm[kRightKnee]),
        robot_control::toString(flight.phase), flight.airborne ? 1 : 0,
        flight.ballistic ? 1 : 0, flight.collision_candidate ? 1 : 0,
        flight.touchdown_candidate ? 1 : 0,
        1000.0 * static_cast<double>(flight.support_loss_elapsed_s),
        1000.0 * static_cast<double>(flight.flight_time_s),
        static_cast<double>(flight.downward_velocity_m_s),
        static_cast<double>(flight.free_fall_confidence),
        static_cast<double>(flight.collision_confidence),
        trajectory.valid ? 1 : 0,
        static_cast<double>(trajectory.forward_position_m),
        static_cast<double>(trajectory.lateral_position_m),
        static_cast<double>(trajectory.forward_velocity_m_s),
        static_cast<double>(trajectory.yaw_rate_rad_s),
        static_cast<double>(trajectory.yaw_rad),
        static_cast<double>(trajectory.confidence),
        trajectory.stationary ? 1 : 0,
        static_cast<double>(controller_->landingForwardVelocity()),
        controller_->landingUsesTrajectoryObserver() ? 1 : 0,
        static_cast<double>(mpc_debug.optimal_command_m_s),
        static_cast<double>(mpc_debug.horizon_end_pitch_rad),
        static_cast<double>(mpc_debug.commanded_ground_velocity_m_s),
        mpc_debug.solver_iterations,
        static_cast<double>(controller_->wholeBodyState().com_offset_x_m),
        static_cast<double>(controller_->wholeBodyState().com_height_m),
        static_cast<double>(controller_->balancePitchRef()),
        static_cast<double>(command_.target_joint_position_rad[kLeftHip]),
        static_cast<double>(command_.target_joint_position_rad[kLeftKnee]),
        station_output_.holding ? 1 : 0, station_output_.ex, station_output_.ey,
        station_output_.yawerror, station_output_.forward_correction_m_s,
        station_output_.yaw_correction_rad_s,
        static_cast<double>(pose_hip_max_rad_), static_cast<double>(pose_knee_min_rad_),
        static_cast<double>(pose_hip_min_rad_), static_cast<double>(pose_knee_max_rad_));
    if (written < 0 || static_cast<std::size_t>(written) >= sizeof(buffer)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "control_mode status was truncated");
    }

    std_msgs::msg::String msg;
    msg.data = buffer;
    mode_pub_->publish(msg);

    if (mode != last_logged_mode_) {
      RCLCPP_INFO(get_logger(), "Control mode: %s, pitch: %.3f rad",
                  robot_control::toString(mode),
                  static_cast<double>(state_.imu.pitch_rad));
      last_logged_mode_ = mode;
    }
  }

  double control_period_s_;
  double wheel_radius_m_;
  double cmd_vel_timeout_s_ = 0.5;
  double last_cmd_vel_stamp_s_ = 0.0;
  double pitch_offset_rad_;
  std::string pitch_source_;
  std::string sim_pose_child_frame_;
  int sim_pose_index_;
  float sim_contact_min_base_z_m_;
  float sim_contact_max_pitch_rad_;
  bool torque_touchdown_enabled_;
  float hip_touchdown_torque_nm_;
  float hip_touchdown_max_velocity_rad_s_;
  bool imu_touchdown_enabled_;
  float imu_touchdown_accel_mps2_;
  bool hard_impact_delay_enabled_;
  float hard_impact_accel_mps2_;
  float hard_impact_pitch_rate_rad_s_;
  float hard_impact_settle_s_;
  float hard_impact_settle_remaining_s_ = 0.0F;
  float left_wheel_sign_;
  float kalman_q_angle_;
  float kalman_q_bias_;
  float kalman_r_measure_;
  float kalman_accel_trust_gain_;
  float kalman_accel_max_deviation_mps2_;
  float kalman_accel_max_pitch_rate_rad_s_;
  float kalman_bias_ = 0.0F;
  float kalman_p_[2][2] = {{1.0F, 0.0F}, {0.0F, 1.0F}};
  float leg_kp_;
  float leg_kd_;
  float wheel_velocity_kp_;
  double sensor_timeout_s_;
  int sensor_stale_grace_cycles_;
  int stale_cycle_count_ = 0;
  float complementary_pitch_rad_ = 0.0F;
  float accel_x_lp_ = 0.0F;
  float accel_y_lp_ = 0.0F;
  float accel_z_lp_ = 9.81F;
  bool imu_seeded_ = false;
  int seed_window_count_ = 0;
  float seed_accum_x_ = 0.0F;
  float seed_accum_y_ = 0.0F;
  float seed_accum_z_ = 0.0F;
  float last_accel_magnitude_m_s2_ = 9.81F;
  bool accel_tilt_measurement_valid_ = false;
  float landing_impact_accel_mps2_ = 35.0F;
  float landing_impact_pitch_rate_rad_s_ = 4.0F;
  float landing_impact_latch_s_ = 0.06F;
  bool pending_imu_impact_event_ = false;
  double pending_imu_impact_expiry_s_ = 0.0;
  float raw_imu_pitch_rate_rad_s_ = 0.0F;
  float raw_imu_yaw_rate_rad_s_ = 0.0F;
  // Real gyro roll rate: published for telemetry (publishMode's "roll_rate="
  // field) AND, since 2026-09-16, fed into state_.imu.roll_rate_rad_s for
  // LandingReflex's differential-knee roll gate (see that Config field's own
  // comment). The direct raw signal, not an Euler-angle-derived estimate
  // (which is degenerate near pitch = pi, see docs/LEDGE_DRIVEOFF_LIMIT.md's
  // own note on peak_roll_rad measurement artifacts).
  float raw_imu_roll_rate_rad_s_ = 0.0F;
  float raw_imu_yaw_rad_ = 0.0F;
  float last_sim_pose_pitch_rad_ = 0.0F;
  double last_sim_pose_stamp_s_ = 0.0;
  double last_sim_pose_z_m_ = 0.0;
  double last_sim_pose_x_m_ = 0.0;
  double last_sim_pose_y_m_ = 0.0;
  double last_orientation_stamp_s_ = 0.0;
  double last_imu_stamp_s_ = 0.0;
  double last_joint_state_stamp_s_ = 0.0;
  double last_timer_stamp_s_ = 0.0;
  double measured_control_dt_s_ = 0.0;
  double jump_control_dt_s_ = 0.0;
  double control_jitter_s_ = 0.0;
  std::array<double, 2> last_wheel_command_rad_s_{0.0, 0.0};
  int last_sim_pose_index_ = -1;
  std::size_t last_sim_pose_count_ = 0;
  bool auto_enable_;
  bool allow_motion_commands_;
  bool imu_noise_enabled_;
  double imu_pitch_noise_stddev_rad_;
  double imu_gyro_noise_stddev_rad_s_;
  std::mt19937 imu_noise_rng_;
  bool have_imu_ = false;
  bool have_sim_pose_ = false;
  bool have_orientation_ = false;
  bool have_joint_state_ = false;
  bool reset_requested_ = false;
  std::array<float, 4> commanded_leg_pose_{0.0F, 0.0F, 0.0F, 0.0F};
  float pose_slew_rate_rad_s_ = 2.0F;
  float pose_hip_min_rad_ = -0.5F;
  float pose_hip_max_rad_ = 0.8F;
  float pose_knee_min_rad_ = -1.6F;
  float pose_knee_max_rad_ = 0.3F;
  bool keep_legs_active_when_fallen_ = false;

  robot_control::RobotState state_{};
  robot_control::ControlCommand command_{};
  std::unique_ptr<robot_control::EmbeddedController> controller_;
  bool station_enabled_ = false;
  robot_control::JumpController::Config jump_config_;
  std::unique_ptr<robot_control::JumpPlan> jump_plan_;
  bool jump_wheel_frame_compensation_=false;
  robot_control::WheelFrame::Result wheel_frame_;
  std::unique_ptr<robot_control::JumpController> jump_;
  robot_control::JumpController::Output jump_output_;
  bool jump_request_level_=false,jump_request_pending_=false;
  bool motion_started_=false;
  bool jump_left_contact_=false,jump_right_contact_=false;
  double jump_contact_timeout_s_=.05;
  // Contact and /clock arrive independently (250 Hz contact, 1 kHz physics).
  static constexpr double kJumpContactClockToleranceS=0.010;
  std::array<double,2> jump_contact_stamps_{};
  std::array<double,2> jump_contact_last_seen_{};
  // Validated per-wheel support signal. It is deliberately narrower than
  // "touching anything": edge fillets/body collisions are not support for
  // the wheel MPC or landing phase machine.
  std::array<bool,2> any_wheel_contact_{};
  std::array<double,2> any_wheel_contact_stamps_{};
  std::array<std::string,2> jump_contact_topics_;
  std::string jump_profile_,jump_ground_collision_;
  std::vector<std::string> contact_support_surface_tokens_;
  std::vector<std::string> contact_reject_surface_tokens_;
  float contact_min_support_normal_z_ = 0.5F;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr jump_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr actuator_enable_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr leg_velocity_reference_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr jump_request_sub_;
  std::array<rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr,2> jump_contact_subs_;
  bool station_recapture_pending_ = false;
  std::unique_ptr<robot_control::StationKeeper> station_keeper_;
  robot_control::StationKeeper::Output station_output_{};
  robot_control::ControlMode last_logged_mode_ =
      robot_control::ControlMode::Disabled;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sim_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stand_up_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      target_pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      wheel_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BalanceControllerNode>());
  rclcpp::shutdown();
  return 0;
}
