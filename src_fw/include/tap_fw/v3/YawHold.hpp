#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace tap_fw::v3 {
// Filter only the yaw feedback branch; pitch and operator turn shaping keep
// their own timing. Smooth near-zero attenuation avoids a hard threshold.
class YawRateFilter {
 public:
  void reset() noexcept { rate_ = 0.0F; }
  float update(float gyro, float dt, float request) noexcept {
    if (!std::isfinite(gyro) || !std::isfinite(dt) || dt <= 0.0F || dt > .01F ||
        !std::isfinite(request)) { reset(); return 0.0F; }
    rate_ += dt / (.025F + dt) * (gyro - rate_);
    constexpr float noise = .015F;
    return std::abs(request) > .01F ? rate_ : rate_ - noise * std::tanh(rate_ / noise);
  }
 private:
  float rate_ = 0.0F;
};

// Relative heading is the integral of gyro yaw-rate error. This is an outer
// heading P / inner rate P cascade (the rate P remains in LqrBalancer).
// Bounded integration and conditional reset follow the anti-windup principle
// of Astrom & Rundqwist, "Integrator Windup and How to Avoid It", ACC 1989.
// No magnetometer, wheel-contact hypothesis or motor-current dependency.
class YawHold {
 public:
  struct Config {
    bool enabled = false;
    float kp = 4.0F;                    // yaw correction / heading error
    float angle_max_rad = .15F;         // bounded relative heading, not compass yaw
    float wheel_max_rad_s = 3.0F;       // extra per wheel, equal and opposite
    float rise_rad_s2 = 4.0F;
    float release_rad_s2 = 12.0F;
    float support_max_s = 1.5F;         // continuous correction budget
  };
  void reset() noexcept { error_ = correction_ = budget_ = 0; blocked_ = false; direction_ = 0; }
  float error() const noexcept { return error_; }
  float correction() const noexcept { return correction_; } // + adds to right
  bool blocked() const noexcept { return blocked_; }
  std::array<float,2> update(const Config& c, std::array<float,2> wheels,
      float dt, float gyro, float yaw_request, float forward, bool safe,
      float wheel_limit, float half_track, float radius) noexcept {
    if (!c.enabled) { reset(); return wheels; }
    if (!std::isfinite(dt) || dt<=0 || dt>.01F || !std::isfinite(gyro) ||
        !std::isfinite(yaw_request) || !std::isfinite(forward) ||
        !std::isfinite(wheels[0]) || !std::isfinite(wheels[1]) ||
        !std::isfinite(wheel_limit) || wheel_limit<=0 || !std::isfinite(radius) || radius<=0 ||
        !std::isfinite(half_track) || half_track<=0 ||
        !std::isfinite(c.kp) || c.kp<0 || !std::isfinite(c.angle_max_rad) || c.angle_max_rad<=0 ||
        !std::isfinite(c.wheel_max_rad_s) || c.wheel_max_rad_s<0 ||
        !std::isfinite(c.rise_rad_s2) || c.rise_rad_s2<=0 ||
        !std::isfinite(c.release_rad_s2) || c.release_rad_s2<=0 ||
        !std::isfinite(c.support_max_s) || c.support_max_s<=0) { reset(); return wheels; }
    const int direction = forward>.04F ? 1 : forward<-.04F ? -1 : 0;
    const bool turning = std::abs(yaw_request)>.01F;
    // No old direction/heading correction may survive a deliberate turn,
    // reversal, stop or loss of pitch authority. Safety beats slew continuity.
    if (!safe || turning || !direction || (direction_ && direction_!=direction)) {
      reset(); direction_=direction; return wheels;
    }
    direction_=direction;
    const float mean=.5F*(wheels[0]+wheels[1]);
    const float diff=.5F*(wheels[1]-wheels[0]);
    // Preserve the pitch controller's mean command. Use only the remaining
    // differential headroom; never increase a wheel ceiling to assist yaw.
    const float headroom=std::max(0.0F,wheel_limit-std::abs(mean));
    const float lo=std::max(-c.wheel_max_rad_s,-headroom-diff);
    const float hi=std::min(c.wheel_max_rad_s,headroom-diff);
    if (lo>hi) { reset(); return wheels; }
    if (!blocked_) {
      const float candidate=std::clamp(error_-gyro*dt,-c.angle_max_rad,c.angle_max_rad);
      const float demand=c.kp*candidate*half_track/radius;
      // Freeze only integration that would push farther into saturation.
      if (!((demand>hi && candidate>error_) || (demand<lo && candidate<error_)))
        error_=candidate;
    }
    float target=blocked_ ? 0 : std::clamp(c.kp*error_*half_track/radius,lo,hi);
    if (std::abs(target)>.1F || std::abs(correction_)>.1F) budget_+=dt;
    if (budget_>=c.support_max_s) { blocked_=true; error_=0; target=0; }
    // A hard obstruction cannot wind up repeatedly: rearm only after an
    // operator stop/turn or a loss of the active/safe state above.
    const float slew=target*correction_>=0 && std::abs(target)>std::abs(correction_)
        ? c.rise_rad_s2 : c.release_rad_s2;
    correction_+=std::clamp(target-correction_,-slew*dt,slew*dt);
    correction_=std::clamp(correction_,lo,hi);
    return {wheels[0]-correction_,wheels[1]+correction_};
  }
 private:
  float error_=0, correction_=0, budget_=0;
  bool blocked_=false;
  int direction_=0;
};
} // namespace tap_fw::v3
