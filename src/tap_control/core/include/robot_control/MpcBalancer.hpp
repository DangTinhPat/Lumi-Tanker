#pragma once

#include <array>

#include "robot_control/Types.hpp"

#ifndef TAP_MPC_MAX_HORIZON
#define TAP_MPC_MAX_HORIZON 80
#endif

namespace robot_control {

// C++ port of upstream upkie.controllers.MPCBalancer:
// https://github.com/upkie/upkie/blob/main/upkie/controllers/mpc_balancer.py
//
// The model is qpmpc.systems.WheeledInvertedPendulum with state
//   x = [ground_position, base_pitch, ground_velocity, base_pitch_rate]
// and input
//   u = ground_acceleration.
// The QP uses scalar state/input weights like qpmpc, then the first acceleration
// is integrated into a ground-velocity command as upstream does.
class MpcBalancer {
 public:
  // Wheel-balance command consumed by EmbeddedController. (Previously shared
  // with the removed PID BalanceController; now owned here.)
  struct Output {
    float ground_velocity_m_s = 0.0F;
    float left_wheel_velocity_rad_s = 0.0F;
    float right_wheel_velocity_rad_s = 0.0F;
    float leg_kp_scale = 1.0F;
    float leg_kd_scale = 1.0F;
    bool fallen = false;
  };

  static constexpr int kStateDim = 4;
  static constexpr int kMaxHorizon = TAP_MPC_MAX_HORIZON;
  static_assert(kMaxHorizon >= 4 && kMaxHorizon <= 80,
                "MPC storage horizon must be in [4,80]");

  struct Config {
    float wheel_radius_m = 0.03F;
    float half_wheel_base_m = 0.1545F;
    float max_ground_velocity_m_s = 0.35F;
    float max_yaw_rate_rad_s = 0.8F;
    bool position_anchor_hold = false;    // hold station against steady drift
    float max_target_distance_m = 0.25F;
    float fall_pitch_rad = 0.65F;
    float leg_kp_scale = 1.0F;
    float leg_kd_scale = 1.0F;
    float turning_stiffness_boost = 0.25F;
    float stiff_yaw_rate_rad_s = 0.1F;
    // Bounded post-touchdown recovery window, edge-triggered on floor_contact
    // going false->true. All landing_* terms default to 0 (window length aside,
    // which is inert unless a boost is nonzero) so a disabled config is
    // byte-identical to the no-landing-logic behavior it replaces - only the
    // node's ROS params can turn this on. kp/kd boosts stack additively on top
    // of the existing turning-stiffness boost; the ground-accel boost
    // temporarily raises wheel authority to arrest the touchdown pitch/velocity
    // transient (the actual whole-body catch; leg stiffening alone only guards
    // body-height/geometry, see MpcBalancer.cpp).
    float landing_window_s = 0.25F;
    float landing_kp_boost = 0.0F;
    float landing_kd_boost = 0.0F;
    float landing_ground_accel_boost_m_s2 = 0.0F;
    float left_wheel_sign = 1.0F;  // common polarity, BOTH native wheels +forward

    float leg_length_m = 0.24F;  // provisional effective pendulum, not link length
    // Provisional reduced-order pendulum scheduling. Neutral URDF inertia gives
    // I_axle/(mass*height_above_axle) ~0.234 m; deep squat gives ~0.180 m.
    // Height scaling approximates this, not the full multibody/contact plant.
    float pendulum_length_per_com_height = 1.10F;
    float min_pendulum_length_m = 0.15F;
    float max_pendulum_length_m = 0.27F;
    float gravity_m_s2 = 9.81F;
    float max_ground_accel_m_s2 = 2.0F;
    // Static friction-cone ceiling (Track B item 2, 2026-09-16): a wheel
    // rolling without slip cannot deliver more tangential acceleration than
    // mu*g regardless of commanded torque or robot mass (F=ma and
    // F_friction=mu*m*g both scale with m, so it cancels) - nothing
    // previously stopped the QP from planning an acceleration beyond what
    // the wheel/ground contact can actually supply. mu=1.0 matches every
    // world's ground_plane <surface><friction><ode><mu> (empty.sdf,
    // ledge_test.sdf, jump.sdf, rough_training.sdf all use mu=mu2=1 there;
    // not auto-synced from the SDF - there is no runtime channel from the
    // physics engine's surface parameters back to this controller, so this
    // must be kept in sync by hand if a world's ground friction changes).
    // Deliberately static, not gated on live/measured contact force: this
    // codebase has repeatedly measured that reactive touchdown-detection
    // signals (hip-torque threshold, IMU-accelerometer threshold, contact
    // sensor threshold - see docs/LEDGE_DRIVEOFF_LIMIT.md) either did
    // nothing or hurt landing recovery, so a live Fz-gated version of this
    // bound is not implemented without the same n>=25 evidence-gated
    // comparison those attempts got, not just reasoning from first
    // principles.
    float wheel_ground_mu = 1.0F;
    // Yaw-rate disturbance rejection (2026-09-16): the wheel differential
    // below only ever tracked *commanded* yaw rate - a real spin-up from
    // asymmetric friction/impact (a wheel catching a ledge corner, uneven
    // touchdown) was never opposed, even though the gyro's own Z axis
    // (state.imu.yaw_rate_rad_s) measures it directly every tick already.
    // 0 (default) is an exact no-op: yaw_command == yaw, byte-identical to
    // pre-2026-09-16 behavior. A positive value closes that loop: the wheel
    // differential drives measured yaw rate toward the commanded one
    // instead of just feed-forwarding the commanded value open-loop.
    float yaw_rate_feedback_kp = 0.0F;
    int nb_timesteps = 30;
    float sampling_period_s = 0.02F;
    float stage_input_cost_weight = 1.0e-3F;
    float stage_state_cost_weight = 1.0e-3F;
    float terminal_cost_weight = 1.0F;
    // Relative weight of the [position] and [velocity] state components against
    // [pitch]/[pitch_rate] (which are 1.0). Lowering position lets the balancer
    // travel further to catch a shifting CoM; 1.0/1.0 = upstream-style uniform.
    float position_cost_scale = 1.0F;
    float velocity_cost_scale = 1.0F;
    // Experimental slow outer station-keeping loop (active only while holding
    // station). Meant to bleed the post-pose-change drift, but any position
    // authority strong enough to matter also disturbs the paused-step bring-up,
    // so it is off by default (gain 0). Left in for future tuning.
    float station_hold_gain_hz = 0.0F;   // return velocity per metre of error
    float station_hold_max_v = 0.10F;    // m/s cap on the return velocity
    float station_hold_period_s = 0.8F;  // blend time constant
    int solver_iterations = 120;
  };

  struct DebugState {
    float commanded_ground_velocity_m_s = 0.0F;
    float target_ground_position_m = 0.0F;
    float optimal_command_m_s = 0.0F;  // upstream first input: accel, m/s^2
    float horizon_end_pitch_rad = 0.0F;
    float balance_pitch_ref_rad = 0.0F;  // leaned target from the CoM offset
    int solver_iterations = 0;
    float landing_window_remaining_s = 0.0F;  // >0 while the landing boost is active
  };

  MpcBalancer();
  explicit MpcBalancer(Config config);

  void reset(float ground_position_m = 0.0F);
  // `balance_pitch_ref_rad` is the base-pitch target the wheels should hold the
  // robot at (0 = upright). Feed it the whole-body equilibrium pitch so the
  // legs can bend / the pose can change without the balancer rolling away to
  // chase a CoM that is no longer over the contact.
  // `hold_station` true (no drive input, legs not moving) lets the balancer
  // bleed off a stale ground-velocity command instead of drifting.
  [[nodiscard]] Output update(const RobotState& state,
                              const ControlCommand& command,
                              bool integrate = true,
                              float balance_pitch_ref_rad = 0.0F,
                              bool hold_station = false);
  [[nodiscard]] DebugState debugState() const noexcept { return last_debug_; }

  // Adapt the internal pendulum model to the live whole-body CoM height (see
  // Config::pendulum_length_per_com_height). Cheap unless the length actually
  // moves, in which case it rebuilds the condensed MPC matrices.
  void setComHeight(float com_height_m);
  [[nodiscard]] float activePendulumLengthM() const noexcept {
    return active_length_m_;
  }

  void setFallPitchRad(float fall_pitch_rad) noexcept {
    config_.fall_pitch_rad = fall_pitch_rad;
  }
  [[nodiscard]] float fallPitchRad() const noexcept {
    return config_.fall_pitch_rad;
  }

  using StateVec = std::array<float, kStateDim>;
  using StateMatrix = std::array<StateVec, kStateDim>;

 private:
  void buildModel();
  [[nodiscard]] float solveHorizon(const StateVec& x0,
                                   float target_ground_velocity_m_s,
                                   float pitch_ref_rad = 0.0F,
                                   float accel_limit_boost_m_s2 = 0.0F);

  Config config_;
  float active_length_m_ = 0.24F;
  int n_ = 30;
  StateMatrix a_d_{};
  StateVec b_d_{};
  // The prediction tensor is Toeplitz: Psi[k][j] = A^(k-1-j) B.
  std::array<StateVec, kMaxHorizon> impulse_{};
  // Condensed linear cost from [position, relative pitch, velocity, pitch rate,
  // position anchor, commanded velocity]; rebuilt only with the plant model.
  std::array<std::array<float, 6>, kMaxHorizon> gradient_map_{};
  std::array<std::array<float, kMaxHorizon>, kMaxHorizon> hessian_{};
  float hessian_row_bound_ = 1.0F;
  std::array<float, kMaxHorizon> u_warm_{};

  StateVec state_cost_{1.0F, 1.0F, 1.0F, 1.0F};  // [pos,pitch,vel,pitch_rate]
  float commanded_ground_velocity_m_s_ = 0.0F;
  float ground_position_target_m_ = 0.0F;
  // floor_contact edge/window bookkeeping for the landing recovery boost. True
  // at reset so bring-up (already grounded) never reads as a spurious touchdown.
  bool prev_floor_contact_ = true;
  float landing_window_remaining_s_ = 0.0F;
  DebugState last_debug_{};
};

}  // namespace robot_control
