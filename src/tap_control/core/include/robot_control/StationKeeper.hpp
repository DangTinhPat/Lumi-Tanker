#pragma once

namespace robot_control {

// Outer proportional loop for simulated world-pose feedback. It supplies
// bounded velocity corrections; the caller retains balance and safety guards.
class StationKeeper {
 public:
  struct Config {
    double position_gain_hz = 0.8;
    double yaw_gain_hz = 2.0;
    double max_forward = 0.08;  // m/s
    double max_yaw = 0.25;     // rad/s
    double acquire_dwell_s = 0.0;  // optional stable startup before first anchor
  };
  struct Input {
    double x = 0.0;    // world metres
    double y = 0.0;
    double yaw = 0.0;  // world heading, rad
    double dt = 0.0;   // elapsed seconds; nonpositive/nonfinite resets
    bool active = false;  // caller: balance-capable mode and fresh pose
    bool operator_driving = false;  // actual operator cmd_vel, not correction
    bool ready_to_acquire = true;  // ignored after the anchor has been captured
  };
  struct Output {
    double forward_correction_m_s = 0.0;
    double yaw_correction_rad_s = 0.0;
    double ex = 0.0;        // anchor minus current, world metres
    double ey = 0.0;
    double yawerror = 0.0;  // shortest signed anchor-minus-current angle
    bool holding = false;
  };

  StationKeeper();
  explicit StationKeeper(Config config);
  [[nodiscard]] Output update(const Input& input);
  void reset() noexcept;

 private:
  Config config_;
  double anchor_x_ = 0.0;
  double anchor_y_ = 0.0;
  double anchor_yaw_ = 0.0;
  bool have_anchor_ = false;
  double acquire_elapsed_s_ = 0.0;
};

}  // namespace robot_control
