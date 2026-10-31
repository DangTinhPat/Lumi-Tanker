#pragma once
#include <array>
#include <cmath>
#include <cstdint>

namespace tap_hw::hybrid {
struct ImuObservation {
  std::uint64_t stamp_us = 0;
  std::array<float, 3> gyro{}, gravity{0, 0, -1}, accel{};
  bool valid = false;
};

// Body +X forward, +Y left, +Z up. Integrates a gravity vector in the body
// frame, including roll; no yaw reference or magnetometer is needed by HCR1.
class GravityEstimator {
 public:
  void reset() noexcept { *this = GravityEstimator{}; }
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
    const bool quiet = std::abs(norm-9.80665F) < .8F && length(gyro) < .20F;
    if (!seeded_) {
      if (!quiet) { count_ = 0; sum_a_ = {}; sum_w_ = {}; return out; }
      for (unsigned i = 0; i < 3; ++i) { sum_a_[i] += accel[i]; sum_w_[i] += gyro[i]; }
      if (++count_ < 250) return out;  // 0.5 s of fresh stationary 500 Hz samples.
      const float n = length(sum_a_);
      if (n < 1) { reset(); return out; }
      for (unsigned i = 0; i < 3; ++i) {
        g_[i] = -sum_a_[i]/n; bias_[i] = sum_w_[i]/count_;
      }
      seeded_ = true;
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
  static float length(const std::array<float, 3>& a) noexcept {
    return std::sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
  }
  void normalize() noexcept {
    const float n = length(g_);
    for (auto& v : g_) v /= n;
  }
  std::uint64_t last_us_ = 0;
  unsigned count_ = 0;
  bool seeded_ = false;
  std::array<float, 3> g_{0,0,-1}, bias_{}, sum_a_{}, sum_w_{};
};
}  // namespace tap_hw::hybrid
