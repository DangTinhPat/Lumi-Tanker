#pragma once

#include "robot_control/Types.hpp"
#include "robot_control/WholeBodyModel.hpp"

namespace robot_control {

// Whole-body stance control for the wheel-legged body.  The slow COM/roll
// trim owns geometry; the optional VMC path is the fast active suspension:
// the body-frame COM-line angle/rate become a virtual damper torque and are
// mapped through the live leg Jacobian into hip/knee feedforward torque.
class WholeBodyStanceController {
 public:
  struct Config {
    bool enabled = false;
    // Dimensionless task-space gains.  They are applied through a filtered
    // target, not as an instantaneous joint command.
    float com_position_gain = 0.20F;
    // Small task-space lead/damping term. It prevents the slow leg trim from
    // continuing to push while station MPC is already returning the support
    // point; zero disables this optional term.
    float com_velocity_damping_s = 0.02F;
    float roll_position_gain = 0.25F;
    // Penalizes fore/aft axle motion while the symmetric COM task trims the
    // legs.  This is a soft task in the fixed 2x2 solve: COM remains the
    // priority, while height and axle position prevent the legs from asking
    // the wheels to translate during a posture transition.
    float height_position_weight = 1.0F;
    // Contact-point fore/aft preservation is a soft null-space preference.
    // Station MPC owns translation; a large value here suppresses the COM
    // task and recreates the old axle_x=constant lock.
    float axle_position_weight = 0.10F;
    float symmetric_task_damping = 1.0e-5F;
    // Low-pass bandwidth for the solved joint-space task command.  This is
    // deliberately independent from the rate at which the bounded trim is
    // integrated into the actuator target.
    float correction_filter_hz = 1.5F;
    // Outer COM/roll trim integrator bandwidth.  Keep this slower than the
    // wheel station loop so both loops do not chase the same support error.
    float com_integrator_gain_hz = 0.30F;
    float max_correction_rad = 0.08F;
    // Independent actuator-target slew limit, in rad/s.
    float max_correction_rate_rad_s = 0.20F;
    // Small body-pitch task for the leg-side gimbal. It changes the axle
    // fore/aft coordinate in the body frame while the wheel controller keeps
    // the contact point; the task is bounded and filtered like the COM trim.
    bool body_pitch_task_enabled = false;
    float body_pitch_task_gain = 1.0F;
    float body_pitch_task_weight = 1.0F;
    float body_pitch_task_max_axle_error_m = 0.025F;
    // Acceleration feed-forward for the legs. To accelerate at a the gravity
    // vector must tilt atan(a/g) relative to the contact, which the body can
    // only reach by first tipping - and on a wheeled inverted pendulum it can
    // only tip by first accelerating the WRONG way. Shifting the COM by
    // h*a/g with the legs supplies the same moment geometrically, without
    // that detour and without tilting the chassis, which is what makes
    // accelerating, braking and stopping crisp. Driven by the balance
    // controller's command-side feed-forward, so it is silent at rest.
    // Outer speed loop, closed through the LEGS. The wheel controller keeps
    // the body upright; this shifts the body horizontally over the contact
    // so the robot accelerates, which is the division of labour a wheel-leg
    // machine wants - wheels balance, legs translate. Expressed as the time
    // constant of the resulting speed loop: a body offset x produces ground
    // acceleration g*x/h, so tracking x = (h/(g*tau))*v_err gives a
    // first-order speed response of time constant tau. Smaller tau is more
    // aggressive. 0 disables the leg speed loop entirely.
    float speed_loop_tau_s = 0.0F;
    // Integral trim on the same loop, in Hz. Absorbs a standing bias (slope,
    // COM calibration) that proportional action alone would leave as a
    // permanent speed offset. Bounded by speed_loop_max_rad.
    float speed_loop_integral_hz = 0.0F;
    // Inertial lead on the same loop. The body carries rotational inertia J
    // about the contact, so while the demand is still changing the legs must
    // lead it; whatever they do not supply in time the chassis supplies by
    // tipping - which is exactly the visible body lean this loop exists to
    // remove. One pendulum time constant sqrt(J/(M*g*h)) is the natural
    // lead, and it shrinks on its own as the robot crouches.
    float speed_loop_inertia_lead = 0.0F;
    float speed_loop_filter_hz = 6.0F;
    float speed_loop_max_rad = 0.10F;
    float speed_loop_max_rate_rad_s = 1.50F;
    // The feed-forward injects an OPEN-LOOP moment: nothing about it is
    // conditioned on the robot coping. Its moment and the wheel controller's
    // own correction simply add, which is a win while the wheels have
    // authority to spare and a fall when they do not - and the reverse
    // direction reaches that point first, because the residual station drift
    // makes a reversal the larger manoeuvre. So it yields as pitch grows:
    // full authority inside the normal working band, faded to nothing by the
    // time the wheels need all of it. This only ever ATTENUATES, never adds,
    // so it is a saturation rather than a feedback path and cannot close a
    // loop with the controller it is helping.
    float speed_loop_release_pitch_rad = 0.06F;
    float speed_loop_release_span_rad = 0.09F;
    // Adaptive virtual model control. The spring ratio is relative to the
    // live body gravitational stiffness M*g*h; damping is sized from that
    // physical reference and the live pitch inertia, so ratio=0 remains a
    // pure damper. Natural and squat poses do not share fixed Nm gains. The
    // torque clamp is an actuator-safety ceiling.
    bool vmc_enabled = false;
    float vmc_stiffness_ratio = 0.35F;
    float vmc_damping_ratio = 0.90F;
    float vmc_max_torque_nm = 0.80F;
    float vmc_max_pitch_rad = 0.25F;
    float vmc_max_pitch_rate_rad_s = 1.50F;
    float vmc_release_time_s = 0.05F;
    float gravity_m_s2 = 9.81F;
    float com_deadband_m = 0.0003F;
    float axle_height_deadband_m = 0.001F;
    float max_pitch_for_trim_rad = 0.25F;
    float hip_min_rad = geometry::native_hip_min_rad;
    float hip_max_rad = geometry::native_hip_max_rad;
    float knee_min_rad = geometry::native_knee_min_rad;
    float knee_max_rad = geometry::native_knee_max_rad;
  };

  struct Output {
    JointArray target{};
    bool active = false;
    float com_error_m = 0.0F;
    float com_target_pitch_rad = 0.0F;
    float vmc_pitch_torque_nm = 0.0F;
    JointArray vmc_torque_nm{};
    float axle_height_error_m = 0.0F;
    float symmetric_trim_norm_rad = 0.0F;
    float differential_trim_norm_rad = 0.0F;
    float speed_loop_com_m = 0.0F;
    float speed_loop_hip_rad = 0.0F;
    float speed_loop_lead_s = 0.0F;
    float speed_loop_release_scale = 1.0F;
  };

  WholeBodyStanceController() = default;
  explicit WholeBodyStanceController(Config config) : config_(config) {}

  void reset() noexcept;

  [[nodiscard]] Output update(const RobotState& state,
                              const JointArray& base_target,
                              const WholeBodyModel::Result& body,
                              bool allow,
                              float dt_s,
                              bool allow_static_com = true,
                              bool allow_vmc = true,
                              float velocity_error_m_s = 0.0F) noexcept;

  [[nodiscard]] const Output& lastOutput() const noexcept { return last_output_; }
  // The held trim is needed when a posture transaction snapshots an actuator
  // target. Keep the state read-only at this boundary so it is subtracted
  // exactly once by the trajectory owner. This is everything this controller
  // added to the target - slow trim plus acceleration feed-forward - because
  // the caller is recovering the untrimmed base pose, not inspecting either
  // loop's state.
  [[nodiscard]] const JointArray& correction() const noexcept {
    return applied_correction_;
  }
 private:
  Config config_{};
  JointArray correction_{};
  JointArray applied_correction_{};
  JointArray task_command_filter_{};
  JointArray speed_loop_{};
  JointArray speed_loop_filter_{};
  float previous_velocity_error_m_s_ = 0.0F;
  float velocity_error_rate_m_s2_ = 0.0F;
  float speed_loop_integral_m_ = 0.0F;
  float vmc_safety_scale_ = 0.0F;
  Output last_output_{};
};

}  // namespace robot_control
