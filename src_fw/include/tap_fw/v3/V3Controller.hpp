#pragma once

#include <array>
#include <cstdint>

#include "robot_control/ImuStateEstimator.hpp"
#include "robot_control/LqrBalancer.hpp"

namespace tap_fw::v3 {

// v3: a plain two-wheel inverted pendulum. The four leg joints are locked by
// LegLock; this class only sequences the start and runs the wheel balancer.
//
// Start flow (operator): robot lifted on the stand -> START -> legs lock ->
// Ready (wheels hold still, no balance loop) -> operator carries the robot to
// the floor and holds it at its best standing point -> releases it -> the
// balance loop engages (auto: a move, then >= still_us of stillness arms it,
// then the first tip engages; or the GO command).
class V3Controller {
 public:
  enum class State { Idle, Locking, Ready, Balancing, Fault };

  enum class Algo { Lqr, Pid };

  // Classic two-loop balancer of hobby two-wheel robots (jjrobots B-ROBOT,
  // Balanduino, ESP32 balancing bots): a fast angle PD whose output is wheel
  // acceleration, integrated into the speed command sent to the drivers, and
  // a slow speed PI whose output is the angle to hold (its integral finds the
  // balance point). Units: acceleration m/s^2, angle rad, speed m/s.
  struct Pid {
    float kp = 30.0F;          // m/s^2 per rad of (pitch - pitch_ref)
    // kp 30 / kd 4 / lead 0.12 s: robustness sweep in the physics plant
    // (mass x0.7-1.3, inertia x0.7-1.4, COM height x0.8-1.2, wheel lag
    // 0.05-0.2 s, command delay 4-40 ms): 318/324 stand, no falls; without
    // the lead the old kp 30 / kd 5 stood 53/108 (wheel lag 0.15 s rings).
    float kd = 4.0F;           // m/s^2 per rad/s of pitch rate
    float kv = 0.10F;          // rad of lean per m/s of speed error
    float kx = 0.10F;          // rad of lean per m of position error
    // Slow integral of position error into the balance angle theta_b: it is
    // what finds the true balance angle (rad per m*s). 0.04 already rings.
    float kix = 0.02F;
    float corr_max_rad = 0.20F; // |kv*ev + kx*ex| clamp; integrator frozen there
    float ref_max_rad = 0.20F;  // |theta_b - seed| limit
    float accel_max_m_s2 = 8.0F;
    // Wheel-lag lead: the drivers are speed servos lagging ~0.1-0.15 s, so
    // the sent command leads the integrated speed by accel * lead_s (a
    // first-order servo then realises the commanded acceleration sooner).
    float lead_s = 0.12F;
    // Wheel deadband compensation [rad/s]: the BL4820 wheels barely move
    // below ~1 rad/s (2026-09-27 floor log; a 1 rad/s deadband in the plant
    // reproduces the measured 0.19 deg / 1.6 cm sway at 0.46 Hz). Added
    // smoothly: w + comp * tanh(w / deadband_eps). 0.6/0.3 cuts the plant's
    // sway 0.0032 -> 0.0013 rad; 0.8 over-drives a light plant without a
    // deadband (physics tests T7/T9).
    float deadband_comp_rad_s = 0.6F;
    float deadband_eps_rad_s = 0.3F;
    float speed_lpf_hz = 5.0F; // speed-loop measurement filter
    // Command may lead the measured wheel speed by at most this (stalls).
    float tracking_margin_m_s = 0.30F;
    // Held/lifted detection: over each window, if the speed command changed
    // by more than held_dv but the body never rotated faster than held_rate,
    // the body is not free (hand, stand, wheels in the air). A free pendulum
    // under 1 m/s^2 would reach well over 0.1 rad/s in that time. Instead of a
    // runaway, the loop re-zeroes: command 0, position here, balance angle =
    // current pitch. So wheels answer a tilt on the stand and stop again.
    float held_dv_m_s = 0.25F;
    // ... and the body's horizontal acceleration stayed below this (a free
    // robot follows the wheels: ~dv/window >= 1.6 m/s^2).
    float held_accel_m_s2 = 0.6F;
    float held_rate_rad_s = 0.10F;
    std::uint64_t held_window_us = 150'000U;
  };

  // Ready-state auto engage (IMU pitch rate / pitch only).
  struct Engage {
    // Engage the balance loop as soon as the legs are locked (the held/
    // lifted detection keeps it from running away on the stand). When false,
    // Ready waits for the auto sequence below or GO.
    bool immediate = true;
    bool automatic = true;
    float move_rate_rad_s = 0.40F;    // carrying the robot off the stand
    float move_pitch_rad = 0.08F;
    float still_rate_rad_s = 0.08F;   // held still at the standing point
    std::uint64_t still_us = 700'000U;
    float tip_rate_rad_s = 0.12F;     // released: starts to tip
    float tip_pitch_rad = 0.02F;
  };

  struct Config {
    Algo algo = Algo::Pid;
    Engage engage{};
    Pid pid{};
    robot_control::LqrBalancer::Config lqr{};
    float com_height_m = 0.2145F;         // locked extended legs (CAD FK)
    std::uint64_t lock_settle_us = 1'200'000U;  // leg ramp + settle before wheels
    float fall_pitch_rad = 0.60F;
    // Below the 0.8 m/s ground-speed cap (20 rad/s): a lifted or held robot
    // saturates the wheels at the cap and must still trip this guard.
    float runaway_wheel_rad_s = 15.0F;
    std::uint64_t runaway_us = 500'000U;
    float wheel_radius_m = 0.04F;
  };

  struct Input {
    std::uint64_t now_us = 0U;
    float dt_s = 0.002F;
    float pitch_rad = 0.0F;
    float pitch_rate_rad_s = 0.0F;
    float yaw_rate_rad_s = 0.0F;
    // Body-frame accelerometer specific force (x forward, z up) [m/s^2]:
    // tells a free robot (body accelerates with the wheels) from a held one.
    float accel_x_m_s2 = 0.0F;
    float accel_z_m_s2 = 9.80665F;
    std::array<float, 2> wheel_rad_s{};    // left, right (core sign: + forward)
    std::array<float, 2> wheel_pos_rad{};
    bool sensors_ok = true;                 // IMU, wheels, legs fresh; no axis fault
    bool legs_ready = false;                // leg lock has reached its target
  };

  struct Command {
    bool start = false;       // edge
    bool go = false;          // edge: engage balancing now (from Ready)
    bool disable = false;     // level: stop everything
    float forward_m_s = 0.0F;
    float yaw_rad_s = 0.0F;
  };

  struct Output {
    State state = State::Idle;
    bool engage_legs = false;   // edge: begin the leg lock now
    bool legs_locked = false;   // level: keep the legs held (also through a fault)
    bool wheels_enabled = false;
    std::array<float, 2> wheel_cmd_rad_s{};
    const char* fault = nullptr;  // set on the tick a fault is raised
  };

  V3Controller() : V3Controller(Config{}) {}
  explicit V3Controller(Config config);

  [[nodiscard]] Output update(const Input& input, const Command& command);
  [[nodiscard]] State state() const noexcept { return state_; }
  [[nodiscard]] robot_control::LqrBalancer& balancer() noexcept { return lqr_; }
  [[nodiscard]] const robot_control::LqrBalancer& balancer() const noexcept { return lqr_; }
  [[nodiscard]] const char* lastFault() const noexcept { return last_fault_; }
  [[nodiscard]] Config& config() noexcept { return config_; }
  // Measured balance angle (IMU pitch with the COM over the axle). When set,
  // it seeds the balance reference at release instead of the release pitch.
  void setBalanceAngle(bool valid, float pitch_rad) noexcept {
    balance_set_ = valid;
    balance_rad_ = pitch_rad;
  }
  [[nodiscard]] bool balanceAngleSet() const noexcept { return balance_set_; }
  // Back to Ready (legs stay locked) so the next GO engages the real balance
  // loop, e.g. from the hand-held TILTTEST check.
  void returnToReady() noexcept {
    if (state_ == State::Balancing) {
      state_ = State::Ready;
      ready_phase_ = 0;
      still_since_us_ = 0U;
    }
  }
  [[nodiscard]] float balanceAngle() const noexcept { return balance_rad_; }
  // Balance angle in use: LQR trim, or the PID's learned theta_b.
  [[nodiscard]] float balanceRef() const noexcept {
    return config_.algo == Algo::Pid ? pid_theta_b_ : lqr_.balanceTrim();
  }
  [[nodiscard]] float pitchRef() const noexcept { return pid_ref_; }
  [[nodiscard]] float positionError() const noexcept { return pid_x_ - pid_x_target_; }
  // Ready sub-phase for the log: 0 wait move, 1 wait still, 2 armed.
  [[nodiscard]] int readyPhase() const noexcept { return ready_phase_; }
  // Number of held/lifted re-zeroes since the balance loop engaged.
  [[nodiscard]] int heldResets() const noexcept { return held_resets_; }
  // Ground velocity/position used by the balancer (axle, rolling contact).
  [[nodiscard]] float groundVelocity() const noexcept { return ground_v_; }
  [[nodiscard]] float groundPosition() const noexcept { return ground_x_; }

 private:
  Output fault(const char* reason, const Output& base);

  Config config_{};
  robot_control::LqrBalancer lqr_;
  State state_ = State::Idle;
  std::uint64_t state_since_us_ = 0U;
  std::uint64_t runaway_since_us_ = 0U;
  const char* last_fault_ = "";
  // The legs are structure in v3: once locked they stay locked through a
  // fault; only DISABLE (command.disable) releases them.
  bool legs_held_ = false;
  float ground_v_ = 0.0F;
  float ground_x_ = 0.0F;
  bool balance_set_ = false;
  float balance_rad_ = 0.0F;
  // PID state.
  float pid_seed_ = 0.0F;     // balance angle at engage
  float pid_theta_b_ = 0.0F;  // learned balance angle
  float pid_ref_ = 0.0F;      // pitch to hold
  float pid_v_filt_ = 0.0F;
  float pid_cmd_m_s_ = 0.0F;  // integrated wheel speed command
  float pid_x0_ = 0.0F;
  float pid_x_ = 0.0F;
  float pid_x_target_ = 0.0F;
  bool pid_driving_ = false;
  std::uint64_t held_window_start_us_ = 0U;
  float held_v0_ = 0.0F;
  float held_max_rate_ = 0.0F;
  float held_accel_sum_ = 0.0F;
  int held_accel_n_ = 0;
  float body_accel_f_ = 0.0F;  // filtered horizontal body acceleration
  int held_free_ticks_ = 0;
  int held_resets_ = 0;
  // After a held re-zero: command 0 until the wheels have stopped (or the
  // operator tilts it), so their run-down cannot restart the loop.
  bool held_quiet_ = false;
  // After a held re-zero the speed/position loop stays off (angle loop only)
  // until the body rotates on its own again (released on the floor): with
  // the body held, the outer loop is what regrows the wheel speed.
  bool held_outer_off_ = false;
  std::uint64_t held_quiet_since_us_ = 0U;
  // Ready state.
  int ready_phase_ = 0;
  float ready_entry_pitch_ = 0.0F;
  std::uint64_t still_since_us_ = 0U;
  float still_sum_ = 0.0F;
  int still_n_ = 0;
  float armed_pitch_ = 0.0F;

  void engageBalance(const Input& in, float seed);
  void heldRezero(const Input& in);
  std::array<float, 2> pidUpdate(const Input& in, const Command& cmd);
};

[[nodiscard]] const char* toString(V3Controller::State state) noexcept;

// The profile: the simulator-tuned LQR plus the hardware-proven deltas
// (adaptive balance angle seeded at start, anti-windup, full pitch feedback)
// and a conservative ground speed cap for bring-up.
[[nodiscard]] V3Controller::Config defaultV3Config();

// Pitch estimator while balancing on the floor. The shipped estimator pulls
// pitch toward the accelerometer at 55 /s; with the wheels pushing on the
// floor the specific force follows the contact force, and the accelerometer
// sees only ~17 % of the real tilt (IMU near the centre of percussion), so
// the loop goes blind and falls. Balancing uses a gyro-dominant estimate:
// slow accelerometer correction, no fast Kalman branch, no quiet-time gyro
// bias learning (it acts as a high-pass on the pitch rate). Idle/Locking/
// Ready keep the base config (accelerometer is right when held or still).
[[nodiscard]] robot_control::ImuStateEstimator::Config balancingEstimatorConfig(
    robot_control::ImuStateEstimator::Config base);

}  // namespace tap_fw::v3
