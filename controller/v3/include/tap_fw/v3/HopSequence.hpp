#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace tap_fw::v3 {

// v3.7 small vertical hop (target: the CoM rises ~2 cm after lift-off, the wheels leave the floor, the robot
// lands on its wheels and keeps balancing). OFF by default (Config::enabled, SET hop 1).
//
//   Dip      a short counter-movement of the knees (CoM down a little, more stroke).
//   Thrust   the knee follows a constant-acceleration path that ends at the planned lift-off speed. The legs are
//            POS-driven (GDS68 position loop, ~22 ms dead time, ~3-4 Hz bandwidth): the path leads the wanted
//            motion by thrust_lead_s and carries the path rate as the driver velocity feed-forward.
//   Flight   the legs hold; the wheels keep their last command (the wheels cannot do anything in the air);
//            shortly before the predicted touchdown the knee flexes a little (the legs touch down ~85 % extended).
//   Land     from the touchdown the wheel balance loop runs again; the legs stay flexed for land_hold_s.
//   Recover  the knee returns to the pre-hop pose, then normal height/tier control resumes.
//
// Sensor contract: only the IMU specific force magnitude (lift-off / touchdown) and the leg geometry the caller
// supplies are read. The plan is open loop in time: the leg feedback is ~96 Hz, a thrust lasts ~0.1 s.
// This class is pure logic (no hardware, no allocation): host-tested.
class HopSequence {
 public:
  enum class Phase : std::uint8_t { Idle, Dip, Thrust, Flight, Land, Recover };
  enum class Refusal : std::uint8_t {
    None, Disabled, Busy, NotBalancing, Settling, Tilted, Moving, Height, Sensors
  };
  enum class Result : std::uint8_t { None, Done, NoLiftOff, Aborted };

  struct Config {
    bool enabled = false;
    float clearance_m = 0.020F;      // planned CoM rise after lift-off (= wheel clearance with the legs held)
    float thrust_gain = 0.90F;        // scale of the planned lift-off speed (SET hopgain)
    bool adapt = true;               // hop to hop: scale the gain by the measured (IMU) flight time
    float adapt_max_step = 0.25F;
    // The balance loop reacts to the CoM-line error that the lagging hip leaves during the push; its saturated wheel
    // torque then builds angular momentum about the CoM (sim: -0.04 N.m.s, the body tumbles in the air). While the
    // legs push, the wheels keep their last command instead.
    // Holding the wheel command (speed relative to the shank) is not holding the ground speed: the shank rotates in
    // the world at pitch rate + hip rate + knee rate (5-10 rad/s in the push) and rolls the axle forward (sim: +0.3 m/s),
    // which tips the CoM line back. The held command carries the carrier term of the planned leg motion.
    float wheel_carrier_gain = 0.095F;
    // Feed-forward of that carrier term: the wheel answers a command ~wheel_delay_s + a first-order wheel_lag_s late (BL4012
    // not yet measured: WHEELID), about as long as the push, so the term leads the plan by both (only the shank rotation
    // of the planned knee path; 0 = off).
    float wheel_ff_gain = 0.0F;
    float wheel_delay_s = 0.05F;
    float wheel_lag_s = 0.075F;
    // Wheels coast (BL4012 torque disabled) from the start of the thrust to the touchdown: the wheel answers a speed
    // command ~110 ms late (as long as the push) and a held speed is a stiff horizontal constraint, so the shank swing
    // of the push rolled the axle forward 0.15-0.3 m/s and tipped the CoM line back. A coasting wheel exerts no
    // horizontal force: only the vertical push acts, through the axle under the CoM.
    bool wheels_limp = true;
    bool wheels_hold_in_thrust = true;
    bool wheels_hold_in_dip = false;
    // The hip follows the MEASURED knee (plus hip_pred_s of its measured rate) instead of the commanded one: in the
    // sim the knee runs at 80-90 % of its current limit and lags the plan by ~0.2 rad, so a hip solved for the
    // commanded knee leaves the CoM line 0.08 rad off at lift-off (angular momentum -0.04 N.m.s, the body
    // tumbles). 0 = off.
    float hip_pred_s = 0.004F;
    // The wheels are held while the legs push, so nothing regulates the virtual pitch there: the hip does (PD on
    // theta_v about its value at the start, through the CoM-line Jacobian), like the tier-2 reflex. 0 = off.
    float tv_kp = 1.11F;
    float tv_kd_s = 0.039F;
    float tv_max_rad = 0.25F;
    // In the air the angular momentum is fixed; the hip is the one actuator that still moves the CoM-axle line in the
    // world (v3.5 sim: 0.35 rad per hip rad). Law: predict theta_v at the planned touchdown (theta_v + rate x time left),
    // swing the hip by -error / flight_sens. 0 = off.
    float flight_sens = 0.09F;
    // CoM-line bias [rad] during the thrust (about 120 mm x rad = mm of CoM ahead of the axle): the push from a standing
    // crouch builds backward angular momentum (v3.5: rest_thrust_lean_rad); a CoM a few mm ahead of the contact cancels it.
    float thrust_lean_rad = 0.041F;
    float flight_hip_max_rad = 0.4F;
    float flight_hip_rate_rad_s = 8.3F;
    // The robot cannot jump from standing: it dips first (like v3.5 / a person) to get joint stroke. The dip goes down
    // to the knee angle dip_knee_rad (0.60 rad = 94 mm hip-axle) if the stand is higher, and at least dip_rad lower
    // than a stand that is already low. Brief (a half-cosine of dip_s), so the low-pose wobble of the balance loop
    // never builds up.
    float dip_knee_rad = 0.70F;
    float dip_rad = 0.10F;           // minimum dip
    float dip_s = 0.288F;
    float stroke_rad = 0.695F;        // knee travel of the thrust (long stroke = low acceleration and torque)
    float knee_max_rad = 1.45F;      // never commanded beyond this (the extension stop has not been measured)
    float knee_min_rad = 0.30F;      // the dip bottom stays above this
    float thrust_lead_s = 0.053F;     // dead time / lag of the POS driver
    // Lift-off: the specific force magnitude at the IMU falls below liftoff_g x g for liftoff_confirm_s.
    float liftoff_g = 0.35F;
    float liftoff_confirm_s = 0.008F;
    float liftoff_wait_s = 0.10F;    // after the planned thrust end; no lift-off by then = give up (NoLiftOff)
    // Touchdown: specific force above touchdown_g x g after min_flight_s, or the predicted time + margin.
    float touchdown_g = 1.3F;
    float min_flight_s = 0.05F;
    float flight_margin_s = 0.06F;
    float land_flex_rad = 0.086F;     // knee flexes by this before touchdown (legs ~85 % extended)
    float land_flex_lead_s = 0.009F;  // ... starting this long before the predicted touchdown
    float land_flex_s = 0.025F;      // ... over this time
    float land_hold_s = 0.15F;
    float recover_s = 0.363F;
    // The normal tiers (hip reflex, level, height) only take over once the wheel loop has the robot quiet again:
    // handed over leaning 0.14 rad the reflex swung the hip 0.4 rad and the robot fell forward (sim).
    // From the touchdown the tier-2 hip reflex (the 15:20 controller's fast hip loop) runs again on top of the hop's hip path.
    // Without it the wheels alone could not stop the lean left by the landing (sim: fell forward in the Recover).
    bool tier2_after_touchdown = false;
    float recover_calm_s = 0.3F;
    float recover_wait_max_s = 2.0F;
    // Start conditions (checked by the owner).
    float start_max_pitch_err_rad = 0.047F;
    float start_max_rate_rad_s = 0.51F;
    // Phase of the balance wobble to start in (sim, 24 start phases at nominal timing): hops started with the CoM
    // line slightly ahead of the balance angle and not moving forward fast cleared the floor and landed; those started
    // behind it and swinging forward (theta_v - ref < 0, rate > 0, wheels commanded backward) all fell, whatever the
    // thrust gain. The push builds backward angular momentum that a forward start absorbs. The request waits for it.
    float start_min_tv_err_rad = 0.008F;
    float start_rate_lo_rad_s = -0.45F;
    float start_rate_hi_rad_s = 0.05F;
    float start_max_speed_m_s = 0.183F;
    float start_min_balance_s = 4.0F;
    float arm_wait_s = 3.0F;         // a request waits this long for a calm moment (wobble, speed) before it is refused
  };

  struct Input {
    float dt_s = 0.002F;
    float accel_norm_m_s2 = 9.80665F;  // |specific force| at the IMU
    bool calm = true;                  // balance quiet (|theta_v - ref|, rates small): the owner decides
  };

  struct Output {
    Phase phase = Phase::Idle;
    float knee_target_rad = 0.0F;     // core knee offset to command
    float knee_rate_rad_s = 0.0F;     // driver velocity feed-forward
    bool wheels_free = false;         // Flight: the balance loop is not driving the wheels
    bool wheels_limp = false;         // the wheel torque is switched off
    bool lift_off_edge = false;
    bool touchdown_edge = false;
    bool finished_edge = false;
    Result result = Result::None;
  };

  // Plan from the pre-hop knee command. dz_dk_end: CoM height gained per knee rad near the end of the stroke
  // [m/rad] (the caller's forward kinematics with the CoM held over the axle).
  bool begin(float knee_start_rad, float dz_dk_end_m_per_rad) noexcept {
    if (!config_.enabled || phase_ != Phase::Idle) return false;
    if (!std::isfinite(knee_start_rad) || !std::isfinite(dz_dk_end_m_per_rad) || dz_dk_end_m_per_rad < 0.01F) return false;
    constexpr float kG = 9.80665F;
    knee_start_ = knee_start_rad;
    knee_bottom_ = std::max(std::min(knee_start_rad - std::max(config_.dip_rad, 0.0F), config_.dip_knee_rad), config_.knee_min_rad);
    stroke_ = std::min(config_.stroke_rad, config_.knee_max_rad - knee_bottom_);
    if (stroke_ < 0.2F) return false;
    knee_top_ = knee_bottom_ + stroke_;
    v_lift_ = std::sqrt(2.0F * kG * std::max(config_.clearance_m, 0.002F)) * std::max(config_.thrust_gain, 0.1F);
    rate_end_ = v_lift_ / dz_dk_end_m_per_rad;
    accel_ = rate_end_ * rate_end_ / (2.0F * stroke_);
    thrust_s_ = 2.0F * stroke_ / rate_end_;
    flight_pred_s_ = 2.0F * v_lift_ / kG * 0.9F;   // 10 % early: the flex must not come late
    dip_s_ = knee_bottom_ < knee_start_ - 1.0e-4F ? std::max(config_.dip_s, 0.04F) : 0.0F;
    phase_ = dip_s_ > 0.0F ? Phase::Dip : Phase::Thrust;
    t_ = 0.0F;
    cmd_ = knee_start_;
    rate_ = 0.0F;
    lowg_s_ = 0.0F;
    flight_s_ = 0.0F;
    result_ = Result::None;
    measured_flight_s_ = 0.0F;
    return true;
  }

  // Abandon the hop from wherever the command is (owner fault/disable).
  void abort() noexcept {
    if (phase_ == Phase::Idle) return;
    result_ = Result::Aborted;
    phase_ = Phase::Idle;
    rate_ = 0.0F;
  }

  [[nodiscard]] Output update(const Input& in) noexcept {
    Output out{};
    if (phase_ == Phase::Idle) {
      out.result = result_;
      return out;
    }
    const float dt = std::clamp(in.dt_s, 0.0F, 0.01F);
    const float g = 9.80665F;
    t_ += dt;
    switch (phase_) {
      case Phase::Dip: {
        const float u = std::min(t_ / dip_s_, 1.0F);
        const float drop = knee_start_ - knee_bottom_;
        cmd_ = knee_start_ - 0.5F * drop * (1.0F - std::cos(3.14159265F * u));
        rate_ = -0.5F * drop * 3.14159265F / dip_s_ * std::sin(3.14159265F * u);
        if (t_ >= dip_s_) { phase_ = Phase::Thrust; t_ = 0.0F; }
        break;
      }
      case Phase::Thrust: {
        const float tl = std::min(t_ + config_.thrust_lead_s, thrust_s_);
        cmd_ = knee_bottom_ + 0.5F * accel_ * tl * tl;
        rate_ = t_ + config_.thrust_lead_s < thrust_s_ ? accel_ * tl : 0.0F;
        // Lift-off: free fall at the IMU, only after the thrust is nearly done.
        const bool low = in.accel_norm_m_s2 < config_.liftoff_g * g;
        lowg_s_ = low && t_ >= thrust_s_ - 0.05F ? lowg_s_ + dt : 0.0F;
        if (lowg_s_ >= config_.liftoff_confirm_s) {
          phase_ = Phase::Flight;
          flight_s_ = lowg_s_;
          out.lift_off_edge = true;
          t_ = 0.0F;
          cmd_ = std::min(cmd_, knee_top_);
          rate_ = 0.0F;
        } else if (t_ > thrust_s_ + config_.liftoff_wait_s) {
          result_ = Result::NoLiftOff;
          startRecover();
        }
        break;
      }
      case Phase::Flight: {
        flight_s_ += dt;
        const float flex_at = std::max(flight_pred_s_ - config_.land_flex_lead_s, 0.02F);
        const float u = std::clamp((flight_s_ - flex_at) / std::max(config_.land_flex_s, 1.0e-3F), 0.0F, 1.0F);
        const float s = u * u * (3.0F - 2.0F * u);
        const float next = knee_top_ - config_.land_flex_rad * s;
        rate_ = dt > 0.0F ? (next - cmd_) / dt : 0.0F;
        cmd_ = next;
        const bool impact = flight_s_ >= config_.min_flight_s && in.accel_norm_m_s2 > config_.touchdown_g * g;
        if (impact || flight_s_ >= flight_pred_s_ + config_.flight_margin_s) {
          measured_flight_s_ = flight_s_;
          phase_ = Phase::Land;
          t_ = 0.0F;
          out.touchdown_edge = true;
          rate_ = 0.0F;
        }
        break;
      }
      case Phase::Land:
        rate_ = 0.0F;
        if (t_ >= config_.land_hold_s) {
          result_ = Result::Done;
          startRecover();
        }
        break;
      case Phase::Recover: {
        const float u = std::min(t_ / std::max(config_.recover_s, 1.0e-3F), 1.0F);
        const float s = u * u * u * (10.0F + u * (-15.0F + 6.0F * u));
        const float next = recover_from_ + (knee_start_ - recover_from_) * s;
        rate_ = dt > 0.0F ? (next - cmd_) / dt : 0.0F;
        cmd_ = next;
        calm_s_ = in.calm ? calm_s_ + dt : 0.0F;
        if (t_ >= config_.recover_s && (calm_s_ >= config_.recover_calm_s || t_ >= config_.recover_s + config_.recover_wait_max_s)) {
          phase_ = Phase::Idle;
          rate_ = 0.0F;
          out.finished_edge = true;
          adaptGain();
        }
        break;
      }
      case Phase::Idle: break;
    }
    out.phase = phase_;
    out.knee_target_rad = cmd_;
    out.knee_rate_rad_s = rate_;
    out.wheels_limp = config_.wheels_limp && (phase_ == Phase::Thrust || phase_ == Phase::Flight);
    out.wheels_free = phase_ == Phase::Flight || (phase_ == Phase::Thrust && config_.wheels_hold_in_thrust) ||
                      (phase_ == Phase::Dip && config_.wheels_hold_in_dip);
    out.result = result_;
    return out;
  }

  // Planned knee rate [rad/s] (and its slope) ahead_s from now, of the nominal path (no driver lead): the caller's
  // wheel feed-forward needs it before the leg has moved.
  [[nodiscard]] float plannedKneeRate(float ahead_s, float* accel) const noexcept {
    float a = 0.0F, r = 0.0F;
    float tt = t_ + ahead_s;
    Phase ph = phase_;
    if (ph == Phase::Dip) {
      if (tt < dip_s_) {
        const float drop = knee_start_ - knee_bottom_;
        const float w = 3.14159265F / dip_s_;
        r = -0.5F * drop * w * std::sin(w * tt);
        a = -0.5F * drop * w * w * std::cos(w * tt);
        if (accel != nullptr) *accel = a;
        return r;
      }
      tt -= dip_s_;
      ph = Phase::Thrust;
    }
    if (ph == Phase::Thrust && tt < thrust_s_) {
      r = accel_ * tt;
      a = accel_;
    }
    if (accel != nullptr) *accel = a;
    return r;
  }

  // Flight time left until the planned touchdown [s] (0 outside the flight).
  [[nodiscard]] float flightRemainingS() const noexcept {
    return phase_ == Phase::Flight ? std::max(flight_pred_s_ + config_.flight_margin_s * 0.5F - flight_s_, 0.0F) : 0.0F;
  }

  // 0..1 weight of the thrust CoM-line bias: ramps in over the second half of the dip, full in the thrust.
  [[nodiscard]] float thrustLeanWeight() const noexcept {
    if (phase_ == Phase::Dip && dip_s_ > 0.0F) return std::clamp((t_ / dip_s_ - 0.5F) * 2.0F, 0.0F, 1.0F);
    return phase_ == Phase::Thrust ? 1.0F : 0.0F;
  }

  [[nodiscard]] bool active() const noexcept { return phase_ != Phase::Idle; }
  [[nodiscard]] Phase phase() const noexcept { return phase_; }
  [[nodiscard]] Result result() const noexcept { return result_; }
  [[nodiscard]] float measuredFlightS() const noexcept { return measured_flight_s_; }
  [[nodiscard]] float plannedLiftSpeed() const noexcept { return v_lift_; }
  [[nodiscard]] float plannedThrustS() const noexcept { return thrust_s_; }
  [[nodiscard]] float kneeTop() const noexcept { return knee_top_; }
  [[nodiscard]] float kneeBottom() const noexcept { return knee_bottom_; }
  [[nodiscard]] float kneeStart() const noexcept { return knee_start_; }
  [[nodiscard]] Config& config() noexcept { return config_; }
  [[nodiscard]] const Config& config() const noexcept { return config_; }
  // Clearance implied by the measured flight time: a free-fall flight of T rises g T^2 / 8.
  [[nodiscard]] float measuredClearanceM() const noexcept {
    return 9.80665F * measured_flight_s_ * measured_flight_s_ / 8.0F;
  }

  static const char* name(Phase p) noexcept {
    switch (p) {
      case Phase::Idle: return "Idle";
      case Phase::Dip: return "Dip";
      case Phase::Thrust: return "Thrust";
      case Phase::Flight: return "Flight";
      case Phase::Land: return "Land";
      case Phase::Recover: return "Recover";
    }
    return "?";
  }
  static const char* refusalName(Refusal r) noexcept {
    switch (r) {
      case Refusal::None: return "none";
      case Refusal::Disabled: return "hop disabled (SET hop 1)";
      case Refusal::Busy: return "hop already running";
      case Refusal::NotBalancing: return "not balancing";
      case Refusal::Settling: return "balance not settled";
      case Refusal::Tilted: return "pitch/rate not calm";
      case Refusal::Moving: return "robot or legs moving";
      case Refusal::Height: return "height outside the hop window";
      case Refusal::Sensors: return "sensors not ready";
    }
    return "?";
  }

 private:
  void startRecover() noexcept {
    phase_ = Phase::Recover;
    t_ = 0.0F;
    calm_s_ = 0.0F;
    recover_from_ = cmd_;
  }
  // Hop to hop: a flight that came out short/long scales the next lift-off speed (clearance ~ v^2).
  void adaptGain() noexcept {
    if (!config_.adapt || result_ != Result::Done || measured_flight_s_ < 0.02F) return;
    const float target_t = 2.0F * std::sqrt(2.0F * std::max(config_.clearance_m, 0.002F) / 9.80665F);
    const float ratio = std::clamp(target_t / measured_flight_s_, 1.0F - config_.adapt_max_step, 1.0F + config_.adapt_max_step);
    config_.thrust_gain = std::clamp(config_.thrust_gain * ratio, 0.5F, 2.0F);
  }

  Config config_{};
  Phase phase_ = Phase::Idle;
  Result result_ = Result::None;
  float t_ = 0.0F, cmd_ = 0.0F, rate_ = 0.0F;
  float knee_start_ = 0.0F, knee_bottom_ = 0.0F, knee_top_ = 0.0F, stroke_ = 0.0F;
  float v_lift_ = 0.0F, rate_end_ = 0.0F, accel_ = 0.0F, thrust_s_ = 0.0F, dip_s_ = 0.0F;
  float flight_pred_s_ = 0.0F, flight_s_ = 0.0F, lowg_s_ = 0.0F;
  float recover_from_ = 0.0F, measured_flight_s_ = 0.0F, calm_s_ = 0.0F;
};

}  // namespace tap_fw::v3
