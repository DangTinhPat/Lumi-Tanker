#pragma once

#include <algorithm>
#include <cmath>

namespace tap_fw::v3 {

// AirborneEstimator: contact phase and centroidal state for the airborne attitude control (AAC).
// Contract sensors only (IMU, leg encoders, optional leg driver currents); nothing from the simulator.
//
//  Lift-off  the CoM acceleration from the leg encoders (FK, valid while the wheels push) reaches -g
//            while the CoM rises: the wheels carry no load any more. Only while armed (thrusting).
//  Flight    T = 2 v / g from the lift-off CoM speed (the thrust's peak); time left = T - time in the air.
//  Touchdown first evidence inside [T - early, ...): IMU specific-force spike, the impact folding
//            the knee (at the end of the flight the legs extend or hold, they never fold), or a knee
//            current jump; without any, assumed at T + late.
//  L         angular momentum about the CoM, a 1-state Kalman filter on the gyro/encoder value: a
//            random walk while the wheels push, (almost) constant in the air.
class AirborneEstimator {
 public:
  struct Config {
    float liftoff_accel_ratio = 0.576F;      // lift-off: FK CoM acceleration <= -this x g ...
    float liftoff_min_speed_m_s = 0.3F;      // ... with the CoM rising at least this fast
    float touchdown_accel_m_s2 = 14.0F;      // IMU specific-force spike
    float touchdown_fold_rate_rad_s = 3.0F;  // impact folds the knee at least this fast
    float touchdown_knee_current_a = 0.0F;   // knee current jump (A) that counts as an impact; 0 = unused
    float touchdown_early_s = 0.05F;         // touchdown evidence accepted from T - this
    float touchdown_late_s = 0.06F;          // without evidence, the touchdown is assumed at T + this
    float l_ground_noise = 0.2F;             // N.m.s / sqrt(s): L random walk while the wheels push
    float l_air_noise = 0.002F;              // ... in the air (conserved up to model error)
    float l_measure_noise = 0.003F;          // N.m.s rms of one gyro/encoder L sample
  };
  enum class Contact { Ground, Air };
  struct Input {
    float dt_s = 0.002F;
    bool armed = false;                // a lift-off may be detected (thrust)
    float accel_norm_m_s2 = 9.80665F;  // |IMU specific force|
    bool vz_valid = false;
    float com_vz_m_s = 0.0F;           // CoM vertical speed / acceleration relative to the axle (FK)
    float com_az_m_s2 = 0.0F;
    float com_vx_m_s = 0.0F;           // CoM horizontal ground speed (wheel odometry + FK + gyro), on the ground
    float knee_rate_rad_s = 0.0F;      // measured, mean of both legs (folding = negative)
    bool current_valid = false;
    float knee_current_a = 0.0F;       // mean |knee current|
    float angular_momentum = 0.0F;     // gyro/encoder sample (bodyMomentum)
  };
  struct Output {
    Contact contact = Contact::Ground;
    bool liftoff = false;              // one tick
    bool touchdown = false;            // one tick
    bool touchdown_assumed = false;    // the touchdown came from the timeout, not from evidence
    float liftoff_speed_m_s = 0.0F;
    float horizontal_speed_m_s = 0.0F; // CoM horizontal speed: live on the ground, held from the lift-off in the air
    float air_time_s = 0.0F;
    float flight_time_s = 0.0F;        // predicted
    float time_left_s = 0.0F;          // predicted, >= 0
    float angular_momentum = 0.0F;     // filtered
    float angular_momentum_std = 0.0F;
  };

  AirborneEstimator() noexcept = default;
  explicit AirborneEstimator(Config config) noexcept : config_(config) {}

  void reset() noexcept {
    contact_ = Contact::Ground;
    air_time_ = flight_time_ = speed_ = 0.0F;
    l_ = 0.0F;
    l_var_ = 1.0F;
    l_seeded_ = false;
    current_prev_ = 0.0F;
    vz_peak_ = 0.0F;
  }

  [[nodiscard]] Output update(const Input& in) noexcept {
    Output out{};
    constexpr float kG = 9.80665F;
    // L: predict (random walk), then correct with the sample.
    const float q = contact_ == Contact::Air ? config_.l_air_noise : config_.l_ground_noise;
    const float r = std::max(config_.l_measure_noise * config_.l_measure_noise, 1.0e-12F);
    if (!l_seeded_ && std::isfinite(in.angular_momentum)) {
      l_ = in.angular_momentum;
      l_var_ = r;
      l_seeded_ = true;
    } else if (std::isfinite(in.angular_momentum)) {
      l_var_ += q * q * std::max(in.dt_s, 0.0F);
      const float k = l_var_ / (l_var_ + r);
      l_ += k * (in.angular_momentum - l_);
      l_var_ *= 1.0F - k;
    }
    if (contact_ == Contact::Ground) {
      vz_peak_ = in.armed && in.vz_valid ? std::max(vz_peak_, in.com_vz_m_s) : 0.0F;
      if (in.armed && in.vz_valid && in.com_vz_m_s >= config_.liftoff_min_speed_m_s &&
          in.com_az_m_s2 <= -config_.liftoff_accel_ratio * kG) {
        contact_ = Contact::Air;
        speed_ = std::max(in.com_vz_m_s, vz_peak_);   // the legs brake at the top: the FK speed already falls
        vx_air_ = in.com_vx_m_s;
        flight_time_ = 2.0F * speed_ / kG;
        air_time_ = 0.0F;
        out.liftoff = true;
      }
    } else {
      air_time_ += in.dt_s;
      const bool window = air_time_ >= flight_time_ - config_.touchdown_early_s;
      const bool spike = in.accel_norm_m_s2 > config_.touchdown_accel_m_s2;
      const bool knee_pushed = -in.knee_rate_rad_s > config_.touchdown_fold_rate_rad_s;
      const bool current_jump = config_.touchdown_knee_current_a > 0.0F && in.current_valid &&
          in.knee_current_a - current_prev_ > config_.touchdown_knee_current_a;
      const bool timeout = air_time_ >= flight_time_ + config_.touchdown_late_s;
      if ((window && (spike || knee_pushed || current_jump)) || timeout) {
        contact_ = Contact::Ground;
        out.touchdown = true;
        out.touchdown_assumed = !(window && (spike || knee_pushed || current_jump));
      }
    }
    current_prev_ = in.current_valid ? in.knee_current_a : current_prev_;
    out.contact = contact_;
    out.liftoff_speed_m_s = speed_;
    out.horizontal_speed_m_s = contact_ == Contact::Air || out.touchdown ? vx_air_ : in.com_vx_m_s;
    out.air_time_s = contact_ == Contact::Air || out.touchdown ? air_time_ : 0.0F;
    out.flight_time_s = flight_time_;
    out.time_left_s = contact_ == Contact::Air ? std::max(flight_time_ - air_time_, 0.0F) : 0.0F;
    out.angular_momentum = l_;
    out.angular_momentum_std = std::sqrt(std::max(l_var_, 0.0F));
    return out;
  }

  // Leaves the air without an event (the jump was aborted).
  void forceGround() noexcept { contact_ = Contact::Ground; }
  [[nodiscard]] Contact contact() const noexcept { return contact_; }
  [[nodiscard]] Config& config() noexcept { return config_; }

 private:
  Config config_{};
  Contact contact_ = Contact::Ground;
  float air_time_ = 0.0F;
  float flight_time_ = 0.0F;
  float speed_ = 0.0F;
  float vx_air_ = 0.0F;
  float l_ = 0.0F;
  float l_var_ = 1.0F;
  bool l_seeded_ = false;
  float current_prev_ = 0.0F;
  float vz_peak_ = 0.0F;
};

}  // namespace tap_fw::v3
