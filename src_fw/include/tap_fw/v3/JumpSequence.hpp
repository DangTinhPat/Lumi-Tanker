#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "tap_fw/v3/JumpPlanner.hpp"
#include "tap_fw/v3/LegHeight.hpp"

namespace tap_fw::v3 {

// v3.5 vertical jump, planned from the robot's own FK/IK (JumpPlanner.hpp):
// Dip -> Thrust -> Flight -> Land -> Recover, run inside V3Controller::Balancing and
// OFF by default (Config::enabled).
//
//  Dip     a quick, smooth counter-movement from the standing height to the planned
//          dip height (like a person getting ready to jump), then straight into the thrust.
//  Thrust  the leg height follows a constant-acceleration path that ends at the top pose
//          with the planned lift-off speed.
//  Flight  tuck the legs as high as the flight time allows, hold, and extend them again
//          before touchdown (like a person): the wheels clear the floor by the target
//          height while the CoM itself rises much less, so the lift-off speed and the
//          landing impact are small.
//  Land    knees soft (a damped spring), wheels free until the impact, then the balance
//          loop restarts. Recover fades the stiffness back.
//
// Sensor contract: only the IMU specific force and the measured leg joints are read.
// The IMU is too noisy (off-CoM, leg reactions) to call free fall, so the flight follows
// the plan's timeline and the impact spike only ends it early.
class JumpSequence {
 public:
  enum class Phase { Idle, Dip, Thrust, Flight, Land, Recover };

  struct Config {
    bool enabled = false;
    JumpPlannerConfig planner;        // target clearance, top pose, leg speeds, torque budget
    // Knobs on top of the plan (tuned in the headless sim, docs/V3.5_JUMP.md).
    float lift_speed_factor = 0.9894F;   // the thrust ends when the measured CoM speed reaches plan speed x this
    float tuck_start_rate_rad_s = 6.3804F;  // the tuck starts once the knee is slower than this
    float tuck_start_max_s = 0.08F;      // ... or after this long in the air
    float flight_kp_scale = 0.6419F;     // leg impedance in the air: soft/damped legs ring less after the lift-off
    float flight_kd_scale = 0.4177F;
    float lift_off_accel_ratio = 0.5760F; // lift-off = the measured CoM acceleration reaches -this x g
    float thrust_extra_s = 0.06F;     // the thrust may overrun its path by this before it gives up
    float dip_extra_m = 0.0073F;         // extra dip below the planned crouch
    float flight_delay_s = -0.0113F;      // touchdown comes this much later than the ballistic estimate
    float start_wait_s = 8.0F;        // wait this long for the legs to stop before refusing
    // The dip is a half-cosine from the current leg height to the dip height; its length
    // follows from the distance and a comfortable peak acceleration, so it adapts to any start.
    float dip_accel_m_s2 = 9.6523F;
    float dip_time_min_s = 0.15F;
    float dip_time_max_s = 0.40F;
    float thrust_kp_scale = 1.2398F;
    float thrust_kd_scale = 0.8866F;
    float thrust_max_s = 0.40F;       // abort softly beyond this
    float touchdown_accel_m_s2 = 14.0F;  // impact spike ends the flight early
    float flight_fb_delay_s = 0.0489F;  // wheels are off the ground this long after the thrust path ends
    float min_flight_s = 0.15F;       // impact spikes earlier than this are leg motion
    float land_lead_s = 0.0111F;      // compliance starts this much before the plan's touchdown
    float land_kp_scale = 0.3928F;    // soft knees: the landing is a damped spring
    float land_kd_scale = 1.7737F;
    float land_hip_kp_scale = 0.6021F;
    float land_hip_kd_scale = 1.7530F;
    float land_follow = 0.6975F;      // 0..1: hip follows the folding knee to hold the CoM over the axle
    float land_wait_s = 0.0689F;      // legs compliant, wheels free: wait this long for the impact
    float land_s = 0.3801F;           // total compliant time before the impedance recovers
    float recover_s = 0.35F;          // impedance back to nominal
    float max_pitch_rad = 0.12F;      // refuse to start beyond this (error from the balance angle)
    float abort_pitch_rad = 1.0F;     // abort the thrust softly beyond this
    // Hip feedback on the body pitch (rad of hip per rad / per rad/s) while the wheels push (dip, thrust).
    float thrust_pitch_kp = 0.7787F;
    float thrust_pitch_kd = 0.0193F;
    // Angular-momentum regulation through the wheels: while the wheels push, a wheel speed offset
    // k * L (rad/s per N.m.s) changes the horizontal ground force and so dL/dt. Measured: about
    // -0.00085 N.m.s of L per rad/s of offset during the thrust.
    float momentum_wheel_gain = 2032.9542F;
    float momentum_wheel_max_rad_s = 40.0F;
    bool momentum_in_dip = false;
    float wheel_kick_rad_s = 0.0F;    // diagnostic: wheel speed command added during the thrust
    float td_l_tau_s = 0.0282F;         // touchdown law: low-pass of the (constant) angular momentum
    float td_margin_s = 0.03F;        // ... the hip offset is reached this long before the touchdown
    float td_go_min_s = 0.0226F;
    float td_lean_rad = -0.0734F;         // ... virtual pitch aimed at the balance angle + this
    float align_max_rad = 0.6F;       // touchdown law: largest hip offset from the zero-reaction curve
    float pitch_fb_max_rad = 0.6F;
    float thrust_hip_gain = 1.4918F;     // scales the CoM-preserving hip change during the thrust
    bool com_track = true;            // hip solved so the CoM stays over the axle along the path
  };

  struct Input {
    float dt_s = 0.002F;
    float accel_norm_m_s2 = 9.80665F;  // |IMU specific force|
    float pitch_rad = 0.0F;
    bool moving = false;               // the height planner is still moving the legs
    // From the leg encoders through FK (valid while the wheels are on the ground):
    bool vz_valid = false;
    float com_vz_m_s = 0.0F;           // CoM vertical speed relative to the axle
    float com_az_m_s2 = 0.0F;          // ... and its derivative (low-passed)
    float leg_height_m = 0.0F;         // measured hip-to-axle height
    float hip_meas_rad = 0.0F;         // measured hip/knee, core offsets (mean of both legs)
    float knee_meas_rad = 0.0F;
    float knee_rate_meas_rad_s = 0.0F;
    bool balancing = false;            // only starts / continues while balancing
    float nominal_hip_rad = 0.0F;      // what the normal controller would command
    float nominal_knee_rad = 0.0F;
  };

  struct Output {
    bool active = false;               // leg targets / scales below replace the nominal ones
    bool hold_wheels = false;          // wheels: zero speed command, LQR not driving
    bool landed = false;               // one tick: re-seed the balance loop now
    int refused = 0;                   // one tick: 4 = the legs were still moving
    bool adopt = false;                // one tick (same): the last leg pose becomes the standing pose
    float hip_rad = 0.0F;
    float knee_rad = 0.0F;
    bool absolute = false;             // hip_rad / knee_rad are the leg command itself (Flight), not a delta
    float kp_scale = 1.0F;
    float kd_scale = 1.0F;
    float hip_kp_scale = 1.0F;         // hips: normally stiff
    float hip_kd_scale = 1.0F;
    float com_follow = 0.0F;           // Land/Recover: weight of the CoM-holding hip (fades out)
    float align_weight = 0.0F;         // Flight: weight of the touchdown alignment (0 -> 1 over the extension)
    Phase phase = Phase::Idle;
    float height_ref_m = 0.0F;
    float time_to_land_s = 0.0F;       // Flight only: until the legs go compliant
    float flight_time_s = 0.0F;        // time in the Flight phase
    bool airborne = false;             // Flight, past flight_fb_delay_s: the wheels have left the ground
  };

  JumpSequence() noexcept = default;
  explicit JumpSequence(Config config) noexcept : config_(config) {}

  // Arms the jump with a plan (JumpPlanner); the next update() starts the Dip.
  bool begin(const JumpPlan& plan) noexcept {
    if (!config_.enabled || !plan.ok || running()) return false;
    plan_ = plan;
    begin_pending_ = true;
    wait_s_ = 0.0F;
    return true;
  }

  [[nodiscard]] Output update(const Input& in) noexcept {
    Output out{};
    out.hip_rad = in.nominal_hip_rad;
    out.knee_rad = in.nominal_knee_rad;
    if (!config_.enabled || !in.balancing) {
      phase_ = Phase::Idle;
      begin_pending_ = false;
      return out;
    }
    t_ += in.dt_s;
    const float h_now = LegHeight::forKnee(in.nominal_knee_rad).axle_below_hip_m;
    switch (phase_) {
      case Phase::Idle:
        if (begin_pending_) {
          if (in.moving) {                     // a jump starts from a robot standing still
            wait_s_ += in.dt_s;
            if (wait_s_ > config_.start_wait_s) {
              begin_pending_ = false;
              out.refused = 4;
            }
          } else {
            begin_pending_ = false;
            h_start_ = h_now;
            h_dip_ = std::min(h_now, dipHeight());
            h_ref_ = h_now;
            const float dh = h_start_ - h_dip_;
            dip_time_ = dh > 0.002F
                ? std::clamp(std::sqrt(4.9348F * dh / std::max(config_.dip_accel_m_s2, 0.5F)),
                             config_.dip_time_min_s, config_.dip_time_max_s)
                : 0.0F;
            enter(Phase::Dip);
          }
        }
        break;
      case Phase::Dip:
        if (t_ >= dip_time_) {
          if (dip_time_ > 0.0F) h_ref_ = h_dip_;
          startThrust();
        }
        break;
      case Phase::Thrust:
        // Lift-off: the measured CoM acceleration reaches -g (the wheels carry no load) after the speed
        // peak, which the leg deceleration at the end of the min-jerk path produces. Without
        // measurements the path end is the lift-off.
        if (t_ >= thrust_time_ + config_.thrust_extra_s ||
            (in.vz_valid && t_ > 0.45F * thrust_time_ &&
             in.com_az_m_s2 <= -config_.lift_off_accel_ratio * 9.80665F) ||
            (!in.vz_valid && t_ >= thrust_time_)) {
          // Lift-off: from here the flight is planned from what was measured.
          const float v = in.vz_valid ? in.com_vz_m_s : v_target_;
          const float hip = in.vz_valid ? in.hip_meas_rad : in.nominal_hip_rad;
          const float knee = in.vz_valid ? in.knee_meas_rad : in.nominal_knee_rad;
          // The curve through the top pose is built now; the tuck is planned once the knee has slowed
          // enough to turn around inside the acceleration budget (the legs brake to the top pose).
          flight_ = FlightPlan{};
          flight_.buildCurve(config_.planner);
          flight_.vertical_speed_m_s = v;
          flight_.flight_time_s = 2.0F * v / 9.80665F;
          flight_.ok = v > 0.15F;
          flight_.knee_start_rad = knee;
          (void)hip;
          if (flight_.ok) {
            enter(Phase::Flight);
          } else {
            enter(Phase::Land);           // too slow to fly: absorb softly
            adopt_pending_ = true;
            touched_ = false;
          }
        } else if (t_ > config_.thrust_max_s ||
                   std::abs(in.pitch_rad) > config_.abort_pitch_rad) {
          enter(Phase::Land);  // no lift-off: abort softly
          adopt_pending_ = true;
          touched_ = false;
        }
        break;
      case Phase::Flight: {
        if (!flight_.tuck_planned) {
          const float rate = in.vz_valid ? in.knee_rate_meas_rad_s : 0.0F;
          if (std::abs(rate) <= config_.tuck_start_rate_rad_s || t_ >= config_.tuck_start_max_s) {
            flight_.planTuck(config_.planner, flight_.vertical_speed_m_s, t_,
                             LegHeight::hangHipCad() + in.hip_meas_rad, (in.vz_valid ? in.knee_meas_rad : in.nominal_knee_rad), rate);
          }
        }
        const float land_at = flight_.flight_time_s + config_.flight_delay_s - config_.land_lead_s;
        if (t_ >= land_at ||
            (t_ >= config_.min_flight_s && in.accel_norm_m_s2 > config_.touchdown_accel_m_s2)) {
          enter(Phase::Land);
          adopt_pending_ = true;   // the flight pose becomes the standing pose
          touched_ = false;
        }
        break;
      }
      case Phase::Land:
        if (!touched_ && (in.accel_norm_m_s2 > config_.touchdown_accel_m_s2 ||
                          t_ >= config_.land_wait_s)) {
          touched_ = true;
          landed_pending_ = true;  // wheels back to the balance loop, restarted here
        }
        if (t_ >= config_.land_s) enter(Phase::Recover);
        break;
      case Phase::Recover:
        if (t_ >= config_.recover_s) enter(Phase::Idle);
        break;
    }
    out.phase = phase_;
    if (phase_ == Phase::Flight) {
      out.airborne = t_ >= config_.flight_fb_delay_s;
      out.flight_time_s = t_;
    }
    if (landed_pending_) {
      out.landed = true;
      landed_pending_ = false;
    }
    if (adopt_pending_) {
      out.adopt = true;
      adopt_pending_ = false;
    }
    switch (phase_) {
      case Phase::Idle:
        break;
      case Phase::Dip: {
        const float s = std::clamp(t_ / std::max(dip_time_, 0.05F), 0.0F, 1.0F);
        h_ref_ = h_start_ + (h_dip_ - h_start_) * (0.5F - 0.5F * std::cos(3.14159265F * s));
        out.active = true;
        followPath(in, out);
        break;
      }
      case Phase::Thrust:
        h_ref_ = h0_ + (plan_.top_height_m - h0_) * minJerkShape(t_ / std::max(thrust_time_, 1.0e-3F));
        out.active = true;
        followPath(in, out);
        out.kp_scale = config_.thrust_kp_scale;
        out.kd_scale = config_.thrust_kd_scale;
        break;
      case Phase::Flight:
        {
          // Zero-reaction tuck: the knee folds on its schedule, the hip follows so that the
          // body does not turn (FlightPlan); then the way back before the touchdown.
          out.knee_rad = flight_.kneeAt(t_);
          out.hip_rad = flight_.hipAt(t_) - LegHeight::hangHipCad();
          out.absolute = true;
        }
        out.active = true;
        out.hold_wheels = true;
        out.kp_scale = out.hip_kp_scale = config_.flight_kp_scale;
        out.kd_scale = out.hip_kd_scale = config_.flight_kd_scale;
        {
          const float land_at = flight_.flight_time_s + config_.flight_delay_s - config_.land_lead_s;
          out.time_to_land_s = std::max(land_at - t_, 0.0F);
          out.align_weight = flight_.tuck_planned && flight_.extend_time_s > 0.0F
              ? minJerkShape((t_ - flight_.extend_start_s) / std::max(flight_.extend_time_s, 1.0e-3F))
              : minJerkShape(t_ / std::max(land_at, 1.0e-3F));
        }
        break;
      case Phase::Land:
        out.active = true;
        out.hold_wheels = !touched_;
        out.kp_scale = config_.land_kp_scale;
        out.kd_scale = config_.land_kd_scale;
        out.hip_kp_scale = config_.land_hip_kp_scale;
        out.hip_kd_scale = config_.land_hip_kd_scale;
        out.com_follow = config_.land_follow;
        break;
      case Phase::Recover: {
        const float s = std::clamp(t_ / std::max(config_.recover_s, 1.0e-3F), 0.0F, 1.0F);
        out.active = true;
        out.kp_scale = config_.land_kp_scale + s * (1.0F - config_.land_kp_scale);
        out.kd_scale = config_.land_kd_scale + s * (1.0F - config_.land_kd_scale);
        out.hip_kp_scale = config_.land_hip_kp_scale + s * (1.0F - config_.land_hip_kp_scale);
        out.hip_kd_scale = config_.land_hip_kd_scale + s * (1.0F - config_.land_hip_kd_scale);
        out.com_follow = config_.land_follow * (1.0F - s);
        break;
      }
    }
    out.height_ref_m = h_ref_;
    return out;
  }

  void reset() noexcept {
    phase_ = Phase::Idle;
    t_ = 0.0F;
    landed_pending_ = adopt_pending_ = begin_pending_ = false;
  }
  [[nodiscard]] Phase phase() const noexcept { return phase_; }
  [[nodiscard]] bool running() const noexcept { return phase_ != Phase::Idle || begin_pending_; }
  [[nodiscard]] const JumpPlan& plan() const noexcept { return plan_; }
  [[nodiscard]] const FlightPlan& flightPlan() const noexcept { return flight_; }
  [[nodiscard]] Config& config() noexcept { return config_; }
  [[nodiscard]] const Config& config() const noexcept { return config_; }
  static const char* name(Phase phase) noexcept {
    switch (phase) {
      case Phase::Idle: return "Idle";
      case Phase::Dip: return "Dip";
      case Phase::Thrust: return "Thrust";
      case Phase::Flight: return "Flight";
      case Phase::Land: return "Land";
      case Phase::Recover: return "Recover";
    }
    return "?";
  }

 private:
  [[nodiscard]] float dipHeight() const noexcept {
    return std::max(plan_.crouch_height_m - config_.dip_extra_m, LegHeight::minHeight() + 0.005F);
  }
  // End of the dip: the thrust starts from here, at rest, with a constant-acceleration path
  // that ends at the top pose with the lift-off speed.
  void startThrust() noexcept {
    h0_ = h_ref_;
    v_target_ = plan_.takeoff_speed_m_s * config_.lift_speed_factor;
    // Min-jerk leg path: the peak CoM speed is 1.875 x (CoM height change) / time, so the time that
    // gives v_target follows from the CoM height change over the stroke (FK).
    const float d_cab = std::max(LegHeight::forHeight(plan_.top_height_m).com_above_axle_m -
                                 LegHeight::forHeight(h0_).com_above_axle_m, 0.005F);
    thrust_time_ = std::clamp(1.875F * d_cab / std::max(v_target_, 0.2F), 0.04F, 0.5F);
    enter(Phase::Thrust);
  }
  // Nominal command + the hip/knee change from the crouch height to h_ref_.
  void followPath(const Input& in, Output& out) const noexcept {
    const auto now = LegHeight::forHeight(h_ref_);
    const auto start = LegHeight::forHeight(h_start_);
    out.hip_rad = in.nominal_hip_rad + (now.hip_offset_rad - start.hip_offset_rad);
    out.knee_rad = in.nominal_knee_rad + (now.knee_offset_rad - start.knee_offset_rad);
  }
  static float minJerkShape(float u) noexcept {
    u = std::clamp(u, 0.0F, 1.0F);
    return u * u * u * (10.0F + u * (-15.0F + 6.0F * u));
  }
  void enter(Phase phase) noexcept {
    phase_ = phase;
    t_ = 0.0F;
  }

  Config config_{};
  JumpPlan plan_{};
  FlightPlan flight_{};
  float dip_time_ = 0.0F;
  float v_target_ = 0.0F;
  Phase phase_ = Phase::Idle;
  float t_ = 0.0F;
  float h_start_ = 0.0F;   // leg height when the jump began: base of the hip/knee path deltas
  float h_dip_ = 0.0F;
  float h0_ = 0.0F;        // start of the thrust path
  float h_ref_ = 0.0F;
  float thrust_time_ = 0.0F;
  float wait_s_ = 0.0F;
  bool begin_pending_ = false;
  bool landed_pending_ = false;
  bool adopt_pending_ = false;
  bool touched_ = false;
};

}  // namespace tap_fw::v3
