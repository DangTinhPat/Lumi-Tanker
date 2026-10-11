#pragma once

#include <algorithm>
#include <cmath>

namespace tap_fw::v3 {

// StandUp: "rocking" stand-up from the lying pose (MJCF keyframe `home`: belly shell and both wheels on the
// floor, body level, the CoM ahead of the axle), adapted from the RockerLower of branch tpa (the old RB; its
// numbers are for that geometry and a 0.8 N.m wheel). A pre-balance maneuver: it never balances itself, it
// only brings the CoM over the axle and then V3Controller engages its own balancer at that moment.
//
// Sensors only (IMU pitch, leg encoders, through the CoM line angle of the leg model):
//     lean = pitch + comLine(hip, knee).angle      (+ : the CoM is ahead of the axle; 0 = over the axle)
//
//  Hold   legs still, wheels still: the IMU and the leg encoders settle.
//  Rise   the hips fold to the hip where the CoM line is vertical (the controller recomputes it for the knee
//         now commanded: ~0.2 rad at knee 0), fast to hip_mid, then slowly. The knees may extend (knee_end):
//         the legs, 3.76 N.m each, lift the belly where the wheels (0.13 N.m) cannot drag it on the floor. Both wheels push forward at
//         wheel_push (joint-space speed; the driver torque limit, 0.13 N.m, is what limits the force) plus
//         wheel_lean_gain x lean: the push fades and reverses as the CoM crosses over the axle. The hips alone
//         cannot lift the belly (their reaction just rolls the wheels back); the push puts the axle under the CoM.
//  Caught the CoM is over the axle and the hips are at the goal: the caller engages the V3 balancer.
//  Failed the CoM fell behind the axle, or the time ran out.
class StandUp {
 public:
  enum class Phase { Idle, Hold, Rise, Caught, Failed };

  struct Config {
    bool enabled = false;               // V3Controller refuses Command::standup unless on
    float hold_s = 0.3F;
    float hip_goal_offset_rad = 0.0F;   // added to the hip that puts the CoM line vertical (body level)
    float knee_end_rad = 0.0F;          // the knees extend to this during the rise (0 = they stay at the homing stop)
    float knee_rate_rad_s = 0.5F;
    float knee_delay_s = 0.0F;          // after the hips start
    float hip_rate1_rad_s = 0.8F;       // hips fold at this rate down to hip_mid ...
    float hip_mid_rad = 0.6F;
    float hip_rate2_rad_s = 0.5F;       // ... then at this one to the goal
    bool wheel_free = true;             // the wheels are commanded their own measured speed (no torque): they roll
                                        // with the legs' geometry; a speed command of 0 would brake the axle
    float wheel_push_rad_s = 0.0F;      // forward joint speed of both wheels added during the rise
    float wheel_delay_s = 0.0F;         // push starts this long after the hips start
    float wheel_ramp_s = 0.2F;
    float wheel_lean_gain = 0.0F;       // rad/s per rad of lean added to the push (negative lean brakes)
    float wheel_max_rad_s = 60.0F;
    float catch_lean_rad = 0.12F;       // |lean| below this ...
    float catch_rate_rad_s = 1.5F;      // ... and |lean rate| below this ...
    float catch_hip_tol_rad = 0.15F;    // ... with the hip this close to the goal: hand over
    float catch_hold_s = 0.03F;
    float fall_lean_rad = 0.5F;         // lean below -this (fell backwards): failed
    float timeout_s = 8.0F;
    float grace_s = 4.0F;               // after the hand-over the balancer's fall limits are wider for this long ...
    float grace_fall_pitch_rad = 1.0F;  // ... (the body is still pitched: the tiers level it)
  };

  struct Input {
    float dt_s = 0.002F;
    float lean_rad = 0.0F;              // pitch + CoM line angle
    float lean_rate_rad_s = 0.0F;
    float hip_goal_rad = 0.0F;          // the standing hip at the knee below
    float wheel_rate_rad_s = 0.0F;      // measured joint speed (mean of both wheels)
    bool sensors_ok = true;
  };

  struct Output {
    Phase phase = Phase::Idle;
    float hip_target_rad = 0.0F;
    float knee_target_rad = 0.0F;
    float wheel_rad_s = 0.0F;           // joint-space speed command, common to both wheels
  };

  StandUp() noexcept = default;
  explicit StandUp(Config config) noexcept : config_(config) {}

  void begin(float hip_rad, float knee_rad) noexcept {
    phase_ = Phase::Hold;
    t_ = 0.0F;
    rise_t_ = 0.0F;
    caught_s_ = 0.0F;
    hip_cmd_ = hip_rad;
    knee_cmd_ = knee_rad;
  }
  void reset() noexcept { phase_ = Phase::Idle; }

  [[nodiscard]] Output update(const Input& in) noexcept {
    Output out{};
    if (phase_ == Phase::Idle || phase_ == Phase::Caught || phase_ == Phase::Failed) {
      out.phase = phase_;
      out.hip_target_rad = hip_cmd_;
      out.knee_target_rad = knee_cmd_;
      return out;
    }
    t_ += in.dt_s;
    if (!in.sensors_ok || !std::isfinite(in.lean_rad) || t_ > config_.timeout_s) phase_ = Phase::Failed;
    if (phase_ == Phase::Hold && t_ >= config_.hold_s) phase_ = Phase::Rise;
    float wheel = 0.0F;
    if (phase_ == Phase::Rise) {
      rise_t_ += in.dt_s;
      const float rate = hip_cmd_ > config_.hip_mid_rad ? config_.hip_rate1_rad_s : config_.hip_rate2_rad_s;
      hip_cmd_ = std::max(hip_cmd_ - rate * in.dt_s, in.hip_goal_rad);
      if (rise_t_ >= config_.knee_delay_s) {
        knee_cmd_ = std::min(knee_cmd_ + config_.knee_rate_rad_s * in.dt_s, std::max(config_.knee_end_rad, knee_cmd_));
      }
      const float ramp = std::clamp((rise_t_ - config_.wheel_delay_s) / std::max(config_.wheel_ramp_s, 1.0e-3F),
                                    0.0F, 1.0F);
      wheel = std::clamp(ramp * config_.wheel_push_rad_s + config_.wheel_lean_gain * in.lean_rad,
                         -config_.wheel_max_rad_s, config_.wheel_max_rad_s) +
          (config_.wheel_free ? in.wheel_rate_rad_s : 0.0F);
      if (in.lean_rad < -config_.fall_lean_rad) phase_ = Phase::Failed;
      const bool over_axle = std::abs(in.lean_rad) < config_.catch_lean_rad &&
          std::abs(in.lean_rate_rad_s) < config_.catch_rate_rad_s &&
          std::abs(hip_cmd_ - in.hip_goal_rad) < config_.catch_hip_tol_rad;
      caught_s_ = over_axle ? caught_s_ + in.dt_s : 0.0F;
      if (caught_s_ >= config_.catch_hold_s) phase_ = Phase::Caught;
    }
    out.phase = phase_;
    out.hip_target_rad = hip_cmd_;
    out.knee_target_rad = knee_cmd_;
    out.wheel_rad_s = phase_ == Phase::Rise ? wheel : config_.wheel_free ? in.wheel_rate_rad_s : 0.0F;
    return out;
  }

  [[nodiscard]] Phase phase() const noexcept { return phase_; }
  [[nodiscard]] float time() const noexcept { return t_; }
  [[nodiscard]] Config& config() noexcept { return config_; }
  [[nodiscard]] const Config& config() const noexcept { return config_; }
  static const char* name(Phase p) noexcept {
    switch (p) {
      case Phase::Idle: return "Idle";
      case Phase::Hold: return "Hold";
      case Phase::Rise: return "Rise";
      case Phase::Caught: return "Caught";
      case Phase::Failed: return "Failed";
    }
    return "?";
  }

 private:
  Config config_{};
  Phase phase_ = Phase::Idle;
  float t_ = 0.0F;
  float rise_t_ = 0.0F;
  float caught_s_ = 0.0F;
  float hip_cmd_ = 0.0F;
  float knee_cmd_ = 0.0F;
};

}  // namespace tap_fw::v3
