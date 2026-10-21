#pragma once
// Finite-motion API of the firmware. The layer above (console text command, a C++ task, the GUI's
// MotionAPI over the link) asks for a distance or an angle; the robot runs it closed on its own
// sensors (axle odometry, gyro yaw rate), so the end point does not depend on the link latency.
//
//   MotionApi api;
//   api.forward(0.5F);             // 0.5 m at the default speed        (also: backward, rotate)
//   api.rotate(1.57F, 0.5F);       // +90 deg at 0.5 rad/s (+ = the DRIVE yaw sign)
//   ...every control tick:  auto out = api.update(feedback);  command.forward_m_s = out.forward_m_s ...
//   api.stop();                    // cancel, speed request back to zero
//
// Not balancing, out of range, timeout, a turn going the wrong way and a bad odometry value all end
// the move (Status::Refused / Aborted / Timeout). Pure logic, no hardware: host-tested.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "tap_fw/MotionProfile.hpp"

namespace tap_fw {

class MotionApi {
 public:
  enum class Kind : std::uint8_t { None, Move, Turn };
  enum class Status : std::uint8_t { Idle, Running, Done, Timeout, Aborted, Cancelled, Refused };

  struct Limits {
    // The caller picks the speed and the room it needs: the ceilings below are only the
    // controller's own (wheel clamp 4.19 m/s, yaw cap 1.5 rad/s), so nothing is cut twice.
    float max_distance_m = 1000.0F, min_distance_m = 0.02F;
    float max_angle_rad = 62.8F, min_angle_rad = 0.05F;  // 10 turns
    float default_speed_m_s = 0.3F, min_speed_m_s = 0.05F, max_speed_m_s = 4.19F;
    float default_rate_rad_s = 0.5F, min_rate_rad_s = 0.05F, max_rate_rad_s = 12.0F;
    float decel_m_s2 = 2.0F, decel_rad_s2 = 6.0F;
    float tol_m = 0.02F, tol_rad = 0.05F;
    float done_speed_m_s = 0.06F, done_rate_rad_s = 0.15F;
    float wrong_way_rad = 0.15F, wrong_way_after_s = 1.0F;
    // With the controller's stopping distance known: brake this much early, then creep the last bit.
    float stop_margin_m = 0.005F, stop_margin_rad = 0.01F;
    float creep_m_s = 0.12F, creep_min_m_s = 0.04F, creep_gain_m = 1.5F;
    float creep_rad_s = 0.15F, creep_min_rad_s = 0.06F, creep_gain_rad = 2.0F;
  };

  struct Feedback {
    std::uint64_t now_us = 0U;
    float dt_s = 0.002F;
    bool balancing = false;      // moves only run (and are only accepted) while balancing
    float ground_x_m = 0.0F;     // axle odometry
    float ground_v_m_s = 0.0F;
    float yaw_rate_rad_s = 0.0F; // gyro, + = the DRIVE yaw sign
    // What the controller needs to stop from its current speed plan, in the units of ground_x_m / the gyro
    // integral and signed like the motion (V3Controller::planStopDistance / yawStopAngle). < 0 = unknown: the
    // move then brakes on the fixed Limits::decel_* instead (host tests, other callers).
    float stop_m = -1.0F;
    float stop_rad = -1.0F;
    bool stop_known = false;
  };

  struct Output {
    bool active = false;         // false: the caller keeps its own (DRIVE / joystick) request
    float forward_m_s = 0.0F;
    float yaw_rad_s = 0.0F;
  };

  // A state change worth telling the layer above (or a 5 Hz progress tick while running).
  struct Event {
    Kind kind = Kind::None;
    Status status = Status::Idle;
    float target = 0.0F, progress = 0.0F, speed = 0.0F;
    const char* reason = "";
  };

  MotionApi() = default;
  explicit MotionApi(const Limits& limits) : limits_(limits) {}

  // speed <= 0 selects the default. Return false when refused (reason() says why).
  bool forward(float distance_m, float speed_m_s = 0.0F) { return begin(Kind::Move, std::fabs(distance_m), speed_m_s); }
  bool backward(float distance_m, float speed_m_s = 0.0F) { return begin(Kind::Move, -std::fabs(distance_m), speed_m_s); }
  bool move(float signed_distance_m, float speed_m_s = 0.0F) { return begin(Kind::Move, signed_distance_m, speed_m_s); }
  bool rotate(float signed_angle_rad, float rate_rad_s = 0.0F) { return begin(Kind::Turn, signed_angle_rad, rate_rad_s); }

  // Cancel a running move; the speed request returns to zero.
  void stop() { if (kind_ != Kind::None) finish(Status::Cancelled, "stop"); }

  // Call once per control tick with fresh feedback (also when idle: cheap).
  Output update(const Feedback& fb) {
    last_fb_ = fb;
    Output out;
    if (kind_ == Kind::None) return out;
    const bool turn = kind_ == Kind::Turn;
    if (!fb.balancing) { finish(Status::Aborted, "not_balancing"); return out; }
    if (fb.now_us - start_us_ > timeout_us_) { finish(Status::Timeout, "timeout"); return out; }
    progress_ = turn ? progress_ + fb.yaw_rate_rad_s * fb.dt_s : fb.ground_x_m - start_x_;
    if (!std::isfinite(progress_)) { finish(Status::Aborted, "bad_odometry"); return out; }
    const float remaining = target_ - progress_;
    const float elapsed_s = static_cast<float>(fb.now_us - start_us_) * 1.0e-6F;
    if (turn && elapsed_s > limits_.wrong_way_after_s && progress_ * target_ < -limits_.wrong_way_rad) {
      finish(Status::Aborted, "wrong_way");  // never spin away on a sign error
      return out;
    }
    const float tol = turn ? limits_.tol_rad : limits_.tol_m;
    const float rate = std::fabs(turn ? fb.yaw_rate_rad_s : fb.ground_v_m_s);
    if (std::fabs(remaining) <= tol && rate < (turn ? limits_.done_rate_rad_s : limits_.done_speed_m_s)) {
      finish(Status::Done, "done");
      return out;
    }
    float v;
    if (fb.stop_known) {
      // Brake when the distance left equals what the controller needs to stop (plus a small margin), and ask for
      // zero: its own jerk-limited plan then ends on the target. A slow creep closes what the model missed.
      const float dir = target_ >= 0.0F ? 1.0F : -1.0F;
      const float stop = std::max(turn ? fb.stop_rad * dir : fb.stop_m * dir, 0.0F);
      const float margin = turn ? limits_.stop_margin_rad : limits_.stop_margin_m;
      if (!braking_ && remaining * dir <= stop + margin) braking_ = true;
      if (!braking_) {
        v = dir * cruise_;
      } else if (std::fabs(remaining) <= tol) {
        v = 0.0F;
      } else {
        const float creep_max = turn ? limits_.creep_rad_s : limits_.creep_m_s;
        const float creep_min = turn ? limits_.creep_min_rad_s : limits_.creep_min_m_s;
        const float gain = turn ? limits_.creep_gain_rad : limits_.creep_gain_m;
        const float want = std::clamp(std::fabs(remaining) * gain, creep_min, std::min(creep_max, cruise_));
        // Only creep once the plan has nearly stopped; until then the request stays zero and it brakes.
        v = rate < (turn ? 2.0F * limits_.done_rate_rad_s : 2.0F * limits_.done_speed_m_s)
            ? std::copysign(want, remaining) : 0.0F;
      }
    } else {
      v = std::fabs(remaining) <= tol ? 0.0F
          : approachSpeed(remaining, cruise_, turn ? limits_.decel_rad_s2 : limits_.decel_m_s2);
    }
    out.active = true;
    (turn ? out.yaw_rad_s : out.forward_m_s) = v;
    if (fb.now_us - report_us_ >= 200'000U) {  // progress for the layer above, 5 Hz
      report_us_ = fb.now_us;
      push(Status::Running, "");
    }
    return out;
  }

  // Next pending event (state change or progress tick); false when none.
  bool takeEvent(Event& out) {
    if (event_count_ == 0U) return false;
    out = events_[event_head_];
    event_head_ = (event_head_ + 1U) % kEvents;
    --event_count_;
    return true;
  }

  [[nodiscard]] bool active() const { return kind_ != Kind::None; }
  [[nodiscard]] Kind kind() const { return kind_ != Kind::None ? kind_ : last_kind_; }
  [[nodiscard]] Status status() const { return kind_ != Kind::None ? Status::Running : last_status_; }
  [[nodiscard]] float target() const { return target_; }
  [[nodiscard]] float progress() const { return progress_; }
  [[nodiscard]] const char* reason() const { return reason_; }
  [[nodiscard]] const Limits& limits() const { return limits_; }

  static const char* statusName(Status s) {
    switch (s) {
      case Status::Running: return "running";
      case Status::Done: return "done";
      case Status::Timeout: return "timeout";
      case Status::Aborted: return "aborted";
      case Status::Cancelled: return "cancelled";
      case Status::Refused: return "refused";
      default: return "idle";
    }
  }

 private:
  bool begin(Kind kind, float target, float cruise) {
    const bool turn = kind == Kind::Turn;
    const float lo = turn ? limits_.min_angle_rad : limits_.min_distance_m;
    const float hi = turn ? limits_.max_angle_rad : limits_.max_distance_m;
    if (!std::isfinite(target) || !std::isfinite(cruise)) return refuse(kind, "not_finite");
    if (std::fabs(target) < lo || std::fabs(target) > hi) return refuse(kind, "range");
    if (!last_fb_.balancing) return refuse(kind, "not_balancing");
    if (kind_ != Kind::None) finish(Status::Cancelled, "replaced");
    if (cruise <= 0.0F) cruise = turn ? limits_.default_rate_rad_s : limits_.default_speed_m_s;
    cruise_ = turn ? std::clamp(cruise, limits_.min_rate_rad_s, limits_.max_rate_rad_s)
                   : std::clamp(cruise, limits_.min_speed_m_s, limits_.max_speed_m_s);
    kind_ = kind;
    target_ = target;
    progress_ = 0.0F;
    start_x_ = last_fb_.ground_x_m;
    braking_ = false;
    start_us_ = report_us_ = last_fb_.now_us;
    timeout_us_ = static_cast<std::uint64_t>(std::max(3.0F, 2.5F * std::fabs(target) / cruise_ + 2.0F) * 1.0e6F);
    reason_ = "";
    push(Status::Running, "");
    return true;
  }
  bool refuse(Kind kind, const char* why) {
    reason_ = why;
    enqueue(Event{kind, Status::Refused, 0.0F, 0.0F, 0.0F, why});
    return false;
  }
  void finish(Status status, const char* why) {
    reason_ = why;
    push(status, why);
    last_kind_ = kind_;
    last_status_ = status;
    kind_ = Kind::None;
  }
  void push(Status status, const char* why) { enqueue(Event{kind_, status, target_, progress_, cruise_, why}); }
  void enqueue(const Event& e) {
    if (event_count_ == kEvents) {  // full: drop the oldest
      event_head_ = (event_head_ + 1U) % kEvents;
      --event_count_;
    }
    events_[(event_head_ + event_count_) % kEvents] = e;
    ++event_count_;
  }

  Limits limits_{};
  Kind kind_ = Kind::None, last_kind_ = Kind::None;
  Status last_status_ = Status::Idle;
  float target_ = 0.0F, progress_ = 0.0F, start_x_ = 0.0F, cruise_ = 0.0F;
  bool braking_ = false;
  std::uint64_t start_us_ = 0U, timeout_us_ = 0U, report_us_ = 0U;
  const char* reason_ = "";
  Feedback last_fb_{};
  static constexpr std::size_t kEvents = 4U;
  Event events_[kEvents]{};
  std::size_t event_head_ = 0U, event_count_ = 0U;
};

}  // namespace tap_fw
