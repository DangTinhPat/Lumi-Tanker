#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace robot_control {

// Offline reference data in native +Y coordinates on both sides. Bounds here
// validate interpolation, not motor torque-speed feasibility or physical landing.
struct JumpPlan {
  struct Sample {
    float time_s = 0;
    float pitch_rad = 0;
    float pitch_rate_rad_s = 0;
    float hip_rad = 0;
    float knee_rad = 0;
    float hip_velocity_rad_s = 0;
    float knee_velocity_rad_s = 0;
    float hip_torque_nm = 0;
    float knee_torque_nm = 0;
    float wheel_velocity_rad_s = 0;
    float wheel_torque_nm = 0;
    float base_x_m = 0;
    float base_z_m = 0;
    float base_vx_m_s = 0;
    float base_vz_m_s = 0;
    float wheel_position_rad = 0;
    std::array<float, 36> feedback_gain{};
  };

  static constexpr std::size_t kCapacity = 256;
  std::array<Sample, kCapacity> samples{};
  std::size_t count = 0;
  float thrust_duration_s = 0;
  float flight_duration_s = 0;
  float touchdown_com_z_m = 0;

  bool valid() const noexcept {
    if (count < 2 || count > kCapacity || samples[0].time_s != 0 ||
        !std::isfinite(thrust_duration_s) || !std::isfinite(flight_duration_s) ||
        !between(touchdown_com_z_m, 0, .6F) ||
        thrust_duration_s <= 0 || flight_duration_s <= 0 ||
        !(double(thrust_duration_s) + flight_duration_s < samples[count - 1].time_s))
      return false;
    for (std::size_t i = 0; i < count; ++i) {
      const auto& s = samples[i];
      if (!between(s.time_s, 0, 2) || !between(s.pitch_rad, -.3F, .3F) ||
          !std::isfinite(s.pitch_rate_rad_s) ||
          !between(s.hip_rad, -.6F, 1.05F) || !between(s.knee_rad, -1.6F, .2F) ||
          !between(s.hip_velocity_rad_s, -kLegSpeed, kLegSpeed) ||
          !between(s.knee_velocity_rad_s, -kLegSpeed, kLegSpeed) ||
          !between(s.hip_torque_nm, -5, 5) || !between(s.knee_torque_nm, -5, 5) ||
          !between(s.wheel_velocity_rad_s, -60, 60) ||
          !between(s.wheel_torque_nm, -.13F, .13F) ||
          !between(s.base_x_m, -2, 2) || !between(s.base_z_m, 0, .6F) ||
          !std::isfinite(s.base_vx_m_s) || !std::isfinite(s.base_vz_m_s) ||
          !between(s.wheel_position_rad, -100, 100)) return false;
      for (float gain : s.feedback_gain)
        if (!between(gain, -1e5, 1e5)) return false;
      if (i == 0) continue;
      const auto& p = samples[i - 1];
      const double dt = double(s.time_s) - p.time_s;
      if (!(dt > 0)) return false;
      if (!Hermite(p.pitch_rad, s.pitch_rad, p.pitch_rate_rad_s, s.pitch_rate_rad_s, dt)
               .bounded(-.3F, .3F, std::numeric_limits<float>::max()) ||
          !Hermite(p.hip_rad, s.hip_rad, p.hip_velocity_rad_s, s.hip_velocity_rad_s, dt)
               .bounded(-.6F, 1.05F, kLegSpeed) ||
          !Hermite(p.knee_rad, s.knee_rad, p.knee_velocity_rad_s, s.knee_velocity_rad_s, dt)
               .bounded(-1.6F, .2F, kLegSpeed) ||
          !Hermite(p.base_x_m, s.base_x_m, p.base_vx_m_s, s.base_vx_m_s, dt)
               .bounded(-2, 2, std::numeric_limits<float>::max()) ||
          !Hermite(p.base_z_m, s.base_z_m, p.base_vz_m_s, s.base_vz_m_s, dt)
               .bounded(0, .6F, std::numeric_limits<float>::max()) ||
          !Hermite(p.wheel_position_rad, s.wheel_position_rad,
                   p.wheel_velocity_rad_s, s.wheel_velocity_rad_s, dt)
               .bounded(-100, 100, 60)) return false;
    }
    return true;
  }

  // Call only after valid(), and do not mutate the plan during playback.
  // Clamping returns the stored endpoint (including its rate). NaN/empty input
  // returns a zero sample defensively; it is not a valid reference or stop gate.
  Sample sample(float time) const noexcept {
    if (count == 0 || count > kCapacity || std::isnan(time)) return {};
    if (time <= samples[0].time_s) return samples[0];
    if (time >= samples[count - 1].time_s) return samples[count - 1];
    std::size_t lo = 0, hi = count - 1;
    while (hi - lo > 1) {
      const auto mid = lo + (hi - lo) / 2;
      if (samples[mid].time_s <= time) lo = mid;
      else hi = mid;
    }
    const auto& a = samples[lo];
    const auto& b = samples[hi];
    if (time == a.time_s) return a;
    const double dt = double(b.time_s) - a.time_s;
    const double u = (double(time) - a.time_s) / dt;
    const Hermite pitch(a.pitch_rad, b.pitch_rad, a.pitch_rate_rad_s, b.pitch_rate_rad_s, dt);
    const Hermite hip(a.hip_rad, b.hip_rad, a.hip_velocity_rad_s, b.hip_velocity_rad_s, dt);
    const Hermite knee(a.knee_rad, b.knee_rad, a.knee_velocity_rad_s, b.knee_velocity_rad_s, dt);
    const Hermite base_x(a.base_x_m, b.base_x_m, a.base_vx_m_s, b.base_vx_m_s, dt);
    const Hermite base_z(a.base_z_m, b.base_z_m, a.base_vz_m_s, b.base_vz_m_s, dt);
    const Hermite wheel(a.wheel_position_rad, b.wheel_position_rad,
                        a.wheel_velocity_rad_s, b.wheel_velocity_rad_s, dt);
    Sample out;
    out.time_s = time;
    out.pitch_rad = static_cast<float>(pitch.position(u));
    out.pitch_rate_rad_s = static_cast<float>(pitch.rate(u));
    out.hip_rad = static_cast<float>(hip.position(u));
    out.knee_rad = static_cast<float>(knee.position(u));
    out.hip_velocity_rad_s = static_cast<float>(hip.rate(u));
    out.knee_velocity_rad_s = static_cast<float>(knee.rate(u));
    out.hip_torque_nm = linear(a.hip_torque_nm, b.hip_torque_nm, u);
    out.knee_torque_nm = linear(a.knee_torque_nm, b.knee_torque_nm, u);
    out.wheel_velocity_rad_s = static_cast<float>(wheel.rate(u));
    out.wheel_torque_nm = linear(a.wheel_torque_nm, b.wheel_torque_nm, u);
    out.base_x_m = static_cast<float>(base_x.position(u));
    out.base_z_m = static_cast<float>(base_z.position(u));
    out.base_vx_m_s = static_cast<float>(base_x.rate(u));
    out.base_vz_m_s = static_cast<float>(base_z.rate(u));
    out.wheel_position_rad = static_cast<float>(wheel.position(u));
    for (std::size_t i = 0; i < out.feedback_gain.size(); ++i)
      out.feedback_gain[i] = linear(a.feedback_gain[i], b.feedback_gain[i], u);
    return out;
  }

 private:
  static constexpr float kLegSpeed = 28.274334F;

  static bool between(double x, double lower, double upper) noexcept {
    return std::isfinite(x) && x >= lower && x <= upper;
  }

  static float linear(float a, float b, double u) noexcept {
    return static_cast<float>(double(a) + u * (double(b) - a));
  }

  struct Hermite {
    double a, b, c, d, dt;
    Hermite(double p0, double p1, double v0, double v1, double duration) noexcept
        : a(2 * (p0 - p1) + duration * (v0 + v1)),
          b(3 * (p1 - p0) - duration * (2 * v0 + v1)),
          c(duration * v0), d(p0), dt(duration) {}

    double position(double u) const noexcept { return ((a * u + b) * u + c) * u + d; }
    double rate(double u) const noexcept { return ((3 * a * u + 2 * b) * u + c) / dt; }

    // Endpoints are checked separately. A cubic has position extrema at roots
    // of its quadratic rate; that rate has at most one interior extremum.
    bool bounded(double lower, double upper, double max_rate) const noexcept {
      // Allow only double evaluation roundoff at interior position extrema.
      // Stored node values and all rate checks retain their hard bounds.
      const double position_tolerance = 32 * std::numeric_limits<double>::epsilon() *
          (std::abs(a) + std::abs(b) + std::abs(c) + std::abs(d) + 1);
      const auto position_ok = [&](double u) {
        return !(u > 0 && u < 1) ||
            between(position(u), lower - position_tolerance, upper + position_tolerance);
      };
      if (a != 0) {
        const double u = -b / (3 * a);
        if (u > 0 && u < 1 && !between(rate(u), -max_rate, max_rate)) return false;
      }
      const double A = 3 * a, B = 2 * b, C = c;
      if (A == 0) return B == 0 || position_ok(-C / B);
      const double discriminant = B * B - 4 * A * C;
      if (discriminant < 0) return true;
      // Stable quadratic roots avoid losing a small root to cancellation.
      const double root = -.5 * (B + std::copysign(std::sqrt(discriminant), B));
      if (root == 0) return position_ok(-B / (2 * A));
      return position_ok(root / A) && position_ok(C / root);
    }
  };
};

}  // namespace robot_control
