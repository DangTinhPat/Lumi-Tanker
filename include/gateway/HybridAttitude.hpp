#pragma once
#include <array>
#include <cmath>
#include <cstdint>

namespace gateway {
struct ImuObservation {
  std::uint64_t stamp_us = 0;
  std::array<float, 3> gyro{}, gravity{0, 0, -1}, accel{};
  bool valid = false;
};

// Body +X forward, +Y left, +Z up. Integrates a gravity vector in the body
// frame, including roll; no yaw reference or magnetometer is needed by HCR1.
class GravityEstimator {
 public:
  // Largest gyro deviation from a stored bias that still counts as "still".
  // Wide enough for temperature drift, narrow enough that a robot turning
  // (yaw) at boot is not absorbed as bias.
  static constexpr float kStoredBiasToleranceRadS = .05F;

  // A fault or data gap drops the attitude but keeps the stored bias.
  void reset() noexcept {
    const auto stored = stored_;
    const bool has_stored = has_stored_;
    *this = GravityEstimator{};
    stored_ = stored; has_stored_ = has_stored;
  }
  // Explicit operator assertion: the robot is held still. Constant yaw cannot
  // be distinguished from bias by a six-axis IMU alone; never enable this at boot.
  void calibrateStationary() noexcept { reset(); manual_ = true; }
  // Bias measured earlier by calibrateStationary() on this board (from flash).
  // It lets boot seed without the operator: the robot still has to show a gyro
  // reading within kStoredBiasToleranceRadS of it, which a turning robot does
  // not. The bias used from then on is re-measured over that window.
  void restoreBias(const std::array<float, 3>& bias) noexcept {
    reset(); stored_ = bias; has_stored_ = true;
  }
  bool hasStoredBias() const noexcept { return has_stored_; }
  // True once after each manual calibration completes; the caller persists bias().
  bool takeManualCalibration() noexcept {
    const bool done = manual_done_; manual_done_ = false; return done;
  }
  unsigned calibrationSamples() const noexcept { return count_; }
  bool manualCalibration() const noexcept { return manual_; }
  const std::array<float,3>& bias() const noexcept { return bias_; }
  ImuObservation update(const std::array<float, 3>& accel,
                        const std::array<float, 3>& gyro,
                        std::uint64_t now) noexcept {
    ImuObservation out{};
    out.stamp_us = now;
    for (unsigned i = 0; i < 3; ++i)
      if (!std::isfinite(accel[i]) || !std::isfinite(gyro[i])) {
        reset(); return out;
      }
    const float dt = last_us_ && now > last_us_ ? (now-last_us_)*1e-6F : 0;
    if (last_us_ && (dt <= 0 || dt > .02F)) reset();
    last_us_ = now;
    out.accel = accel;
    const float norm = length(accel);
    // Manual: the operator vouches for stillness. Automatic: gyro near zero, or
    // near the stored bias when the board has one.
    const bool use_stored = has_stored_ && !manual_;
    std::array<float, 3> rest = gyro;
    if (use_stored) for (unsigned i = 0; i < 3; ++i) rest[i] -= stored_[i];
    const float rest_limit = manual_ ? .35F : (use_stored ? kStoredBiasToleranceRadS : .20F);
    const bool quiet = std::abs(norm-9.80665F) < .8F && length(rest) < rest_limit;
    if (!seeded_) {
      if (!quiet) { clearWindow(); return out; }
      // Bound variation against the first sample so slow drift cannot hide
      // behind a moving average. Applies to automatic and manual calibration.
      if (!count_) { first_a_=accel; first_w_=gyro; started_us_=now; }
      for (unsigned i=0;i<3;++i)
        if (std::abs(accel[i]-first_a_[i])>.20F || std::abs(gyro[i]-first_w_[i])>.025F) {
          clearWindow(); return out;
        }
      for (unsigned i = 0; i < 3; ++i) { sum_a_[i] += accel[i]; sum_w_[i] += gyro[i]; }
      ++count_;
      if (count_ < (manual_ ? 500U : 250U) || now-started_us_ < (manual_ ? 998000U : 498000U))
        return out; // fresh samples AND elapsed time, independent of call rate
      const float n = length(sum_a_);
      if (n < 1) { reset(); return out; }
      for (unsigned i = 0; i < 3; ++i) {
        g_[i] = -sum_a_[i]/n; bias_[i] = sum_w_[i]/count_;
      }
      seeded_ = true;
      if (manual_) { manual_done_ = true; stored_ = bias_; has_stored_ = true; }
      manual_ = false;
    } else {
      std::array<float, 3> w{};
      for (unsigned i = 0; i < 3; ++i) w[i] = gyro[i]-bias_[i];
      const auto old = g_;
      g_[0] += dt*(old[1]*w[2]-old[2]*w[1]);
      g_[1] += dt*(old[2]*w[0]-old[0]*w[2]);
      g_[2] += dt*(old[0]*w[1]-old[1]*w[0]);
      normalize();
      // Reject acceleration/impact corrections inconsistent with attitude.
      if (std::abs(norm-9.80665F) < 1.0F && length(w) < 1.5F) {
        float dot = 0;
        for (unsigned i = 0; i < 3; ++i) dot -= g_[i]*accel[i]/norm;
        if (dot > .97F) {
          const float gain = dt/(.5F+dt);
          for (unsigned i = 0; i < 3; ++i) g_[i] += gain*(-accel[i]/norm-g_[i]);
          normalize();
        }
      }
    }
    for (unsigned i = 0; i < 3; ++i) out.gyro[i] = gyro[i]-bias_[i];
    out.gravity = g_;
    out.valid = true;
    return out;
  }
 private:
  void clearWindow() noexcept { count_=0; sum_a_={}; sum_w_={}; started_us_=0; }
  static float length(const std::array<float, 3>& a) noexcept {
    return std::sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
  }
  void normalize() noexcept {
    const float n = length(g_);
    for (auto& v : g_) v /= n;
  }
  std::uint64_t last_us_ = 0, started_us_ = 0;
  unsigned count_ = 0;
  bool seeded_ = false, manual_ = false, manual_done_ = false, has_stored_ = false;
  std::array<float, 3> g_{0,0,-1}, bias_{}, stored_{}, sum_a_{}, sum_w_{}, first_a_{}, first_w_{};
};
}  // namespace gateway
