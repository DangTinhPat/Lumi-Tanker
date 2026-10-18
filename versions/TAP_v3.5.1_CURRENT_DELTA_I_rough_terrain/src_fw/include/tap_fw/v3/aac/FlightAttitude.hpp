#pragma once

namespace tap_fw::v3 {

// FlightAttitude: the touchdown attitude in the air (AAC, flight phase).
//
// The leg SHAPE plan (FlightPlan: tuck for the clearance and back to the touchdown pose) is given.
// On top of it this block commands a touchdown pose change d = (dhip, dknee). In the air the only
// attitude actuator is the legs (the wheels are too light to store the angular momentum), and with the
// angular momentum L constant (AirborneModel):
//   body_td  = body  + free rotation (integral of L / I along the plan) + leg turn on the way + S_body  d
//   theta_td = theta + (same)                                  + line(q_td + d) - line(q_now)
// Every control tick it solves (receding horizon, Gauss-Newton on the exact model)
//   min  w_line (theta_td - theta_ref)^2 + w_body (body_td - body_ref)^2 + w_shape |d|^2
// with body_ref the body pitch of the robot standing balanced at the touchdown pose, then moves the
// commanded d toward the solution so that it arrives `arrive_before_s` before the touchdown. When
// the CoM line and the body cannot both be right (large L at the lift-off), the weights decide;
// the body error is kept within body_tilt_max_rad by stiffening w_body.
class FlightAttitude {
 public:
  struct Config {
    float w_line = 1.0F;              // 1/rad^2: virtual pitch at the touchdown (CoM over the axle)
    float w_body = 0.0F;              // 1/rad^2: body pitch at the touchdown (as when standing); >0 trades CoM placement for a level body
    float w_shape = 0.002F;           // 1/rad^2: touchdown pose change (small: the legs are the only actuator)
    float body_tilt_max_rad = 0.35F;  // with w_body > 0: body error at the touchdown kept below this if possible
    float line_bias_rad = -0.0796F;   // theta_v aimed at = balance angle + this (impact dynamics)
    float hip_change_max_rad = 0.6F;
    float knee_change_max_rad = 0.35F;
    float change_rate_max_rad_s = 6.0F;  // the commanded pose change moves at most this fast
    float arrive_before_s = 0.0299F;    // the pose change is complete this long before the touchdown
    float arrive_min_s = 0.02F;
  };
  struct Input {
    float dt_s = 0.002F;
    float time_left_s = 0.0F;         // to the touchdown
    float angular_momentum = 0.0F;    // filtered (AirborneEstimator)
    float body_pitch = 0.0F;          // IMU
    float theta_v = 0.0F;             // body pitch + line(q_meas) - line(hang)
    float hip_meas_cad = 0.0F;        // measured leg pose (mean of both legs)
    float knee_meas = 0.0F;
    float hip_plan_cad = 0.0F;        // the shape plan's command now (without the change d)
    float knee_plan = 0.0F;
    float hip_td_cad = 0.0F;          // the shape plan's touchdown pose
    float knee_td = 0.0F;
    float plan_turn = 0.0F;           // body turn along the shape plan from now to the touchdown pose
    static constexpr int kSamples = 5;
    float plan_hip_cad[kSamples]{};   // the shape plan sampled from now to the touchdown (free rotation)
    float plan_knee[kSamples]{};
    float theta_ref = 0.0F;           // balance angle of the virtual pitch
    float body_com_dx_m = 0.0F;       // controller's body CoM correction (comLineAngle)
  };
  struct Output {
    float hip_change_rad = 0.0F;      // commanded d, add to the shape plan's command
    float knee_change_rad = 0.0F;
    float target_hip_rad = 0.0F;      // solution of this tick
    float target_knee_rad = 0.0F;
    float theta_v_td = 0.0F;          // predicted at the touchdown with the solution
    float body_td = 0.0F;
    float body_ref = 0.0F;
  };

  FlightAttitude() noexcept = default;
  explicit FlightAttitude(Config config) noexcept : config_(config) {}
  // Starts a flight with the commanded change seeded (continuity with the command before the switch).
  void reset(float hip_change_rad, float knee_change_rad) noexcept {
    hip_cmd_ = hip_change_rad;
    knee_cmd_ = knee_change_rad;
  }
  [[nodiscard]] Output update(const Input& in) noexcept;
  [[nodiscard]] Config& config() noexcept { return config_; }

 private:
  Config config_{};
  float hip_cmd_ = 0.0F;
  float knee_cmd_ = 0.0F;
};

}  // namespace tap_fw::v3
