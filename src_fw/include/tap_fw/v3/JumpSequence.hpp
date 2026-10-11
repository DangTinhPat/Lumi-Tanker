#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "tap_fw/v3/JumpPlanner.hpp"
#include "tap_fw/v3/aac/AirborneEstimator.hpp"
#include "tap_fw/v3/aac/FlightAttitude.hpp"
#include "tap_fw/v3/aac/TakeoffMomentum.hpp"
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
  // Approach is numbered last so the telemetry numbers of the other phases stay as they were.
  enum class Phase { Idle, Dip, Thrust, Flight, Land, Recover, Approach };

  struct Config {
    bool enabled = false;
    JumpPlannerConfig planner;        // target clearance, top pose, leg speeds, torque budget
    AirborneEstimator::Config estimator;  // contact phase, flight time, angular momentum (AAC)
    FlightAttitude::Config flight;        // touchdown attitude in the air (AAC)
    TakeoffMomentum::Config takeoff{false};  // momentum at the lift-off (AAC); off until validated
    // Knobs on top of the plan (tuned in the headless sim, docs/V3.5_JUMP.md).
    float lift_speed_factor = 0.9058F;   // the thrust ends when the measured CoM speed reaches plan speed x this
    float tuck_start_rate_rad_s = 8.0109F;   // the tuck starts once the knee is slower than this (default: at the lift-off)
    float tuck_start_max_s = 0.08F;      // ... or after this long in the air
    float flight_kp_scale = 0.4461F;     // leg impedance in the air: soft/damped legs ring less after the lift-off
    float flight_kd_scale = 0.8165F;
    float lift_off_accel_ratio = 0.6382F; // lift-off = the measured CoM acceleration reaches -this x g
    float thrust_extra_s = 0.06F;     // the thrust may overrun its path by this before it gives up
    float dip_extra_m = 0.0081F;         // extra dip below the planned crouch
    float flight_delay_s = -0.0114F;      // touchdown comes this much later than the ballistic estimate
    float start_wait_s = 8.0F;        // wait this long for the legs to stop before refusing
    // Forward hop (like a kangaroo): one motion from standing. The dip leans the body toward the hop
    // (hop_lean_rad, the CoM line kept so the robot stays balanced) while the forward speed builds to
    // distance / planned flight time; the wheels spin at the CoM's horizontal speed in the air; after the
    // touchdown the speed is braked to zero at hop_brake_m_s2.
    // hop_run_up (default): run up to the hop speed first (Approach, in a ready pose: body leaning toward the
    // jump, legs lower), then dip and jump at a steady speed. 0: one motion from standing (experimental:
    // lands only near 8 cm, docs/V3.5_JUMP.md section 9).
    bool hop_run_up = true;
    float approach_tolerance_m_s = 0.0703F;  // ground speed within this of the hop speed ...
    float approach_settle_s = 0.1252F;        // ... for this long (and the legs still) starts the dip
    float approach_timeout_s = 5.0F;       // otherwise refuse (refused = 5)
    // Ready pose (the "load"): the legs go down along the standing manifold (CoM over the axle, body level)
    // to the crouch height the thrust starts from - only if the stand is above it: a low stand already has
    // the stroke and does not dip at all - while the tiers speed up to the hop speed. ready_crouch false:
    // ready_drop_m lower instead. ready_lean_rad leans the body toward the jump (0: level).
    bool ready_crouch = true;
    bool ready_before_drive = true;        // the speed-up starts once the ready pose is reached
    float ready_lean_rad = 0.0F;
    float ready_drop_m = 0.025F;
    float ready_time_s = 0.2933F;
    float hop_default_m = 0.08F;        // JUMP without a distance
    float hop_max_m = 0.12F;
    float hop_speed_scale = 1.0F;       // run-up speed = scale x distance / planned flight time
    float hop_lean_rad = 0.06F;
    float hop_gimbal = 0.5F;            // share of the tier-2 planned lean (atan(a/g) through the hips, body level) used in the hop
    float hop_stop_s = 1.0F;            // the zero-speed command overrides the operator this long after the touchdown
    float hop_accel_m_s2 = 0.6880F;        // forward plan limits while hopping (the operator's plan uses 0.3 / 1.0):
    float hop_accel_per_m_s = 0.0F;     // speeding up in the load: hop_accel + this x hop speed
    float hop_decel_m_s2 = 0.9287F;        // braking after the touchdown
    float hop_jerk_m_s3 = 5.8259F;
    // Hop thrust: the CoM line leans by this x hop speed (rad per m/s) while the legs extend - the wheels a
    // few mm behind (or ahead of) the CoM, so the leg force's moment about the CoM trims the angular
    // momentum the thrust from a moving crouch builds (sim: -0.009 N.m.s at 0.3 m/s; the vertical impulse
    // ~4 N.s needs only ~2 mm of offset).
    float hop_thrust_lean_per_m_s = 0.4589F;
    float forward_speed_max_m_s = 0.6F;
    bool flight_wheel_match = true;     // hop: in the air the wheels spin at the CoM's horizontal speed
    // The dip is a half-cosine from the current leg height to the dip height; its length
    // follows from the distance and a comfortable peak acceleration, so it adapts to any start.
    float dip_accel_m_s2 = 10.0000F;
    float dip_time_min_s = 0.15F;
    float dip_time_max_s = 0.40F;
    float thrust_kp_scale = 1.2379F;
    float thrust_kd_scale = 0.8358F;
    float thrust_max_s = 0.40F;       // abort softly beyond this
    float touchdown_accel_m_s2 = 14.0F;  // impact spike ends the flight early
    float flight_fb_delay_s = 0.0489F;  // wheels are off the ground this long after the thrust path ends
    float min_flight_s = 0.15F;       // impact spikes earlier than this are leg motion
    float land_lead_s = 0.0164F;      // compliance starts this much before the plan's touchdown
    float land_kp_scale = 0.3362F;    // soft knees: the landing is a damped spring
    float land_kd_scale = 1.8836F;
    float land_hip_kp_scale = 0.5876F;
    float land_hip_kd_scale = 1.6584F;
    float land_follow = 0.8415F;      // 0..1: hip follows the folding knee to hold the CoM over the axle
    float land_wait_s = 0.0689F;      // legs compliant, wheels free: wait this long for the impact
    float land_s = 0.3801F;           // total compliant time before the impedance recovers
    float recover_s = 0.35F;          // impedance back to nominal
    float max_pitch_rad = 0.12F;      // refuse to start beyond this (error from the balance angle)
    float abort_pitch_rad = 1.0F;     // abort the thrust softly beyond this
    // Hip feedback on the body pitch (rad of hip per rad / per rad/s) while the wheels push (dip, thrust).
    float thrust_pitch_kp = 0.6926F;
    float thrust_pitch_kd = 0.0137F;
    // Angular-momentum regulation through the wheels: while the wheels push, a wheel speed offset
    // k * L (rad/s per N.m.s) changes the horizontal ground force and so dL/dt. Measured: about
    // -0.00085 N.m.s of L per rad/s of offset during the thrust.
    float momentum_wheel_gain = 1471.82F;
    float momentum_wheel_max_rad_s = 40.0F;
    bool momentum_in_dip = false;
    float wheel_kick_rad_s = 0.0F;    // diagnostic: wheel speed command added during the thrust
    float pitch_fb_max_rad = 0.6F;
    float thrust_hip_gain = 1.4951F;     // scales the CoM-preserving hip change during the thrust
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
    float ground_speed_m_s = 0.0F;     // wheel odometry ground speed (hop run-up)
    float nominal_hip_rad = 0.0F;      // what the normal controller would command
    float nominal_knee_rad = 0.0F;
  };

  struct Output {
    bool active = false;               // leg targets / scales below replace the nominal ones
    bool hold_wheels = false;          // wheels: zero speed command, LQR not driving
    bool landed = false;               // one tick: re-seed the balance loop now
    int refused = 0;                   // one tick: 4 = the legs were still moving, 5 = run-up speed not reached
    bool drive = false;                // hop: drive_m_s replaces the operator's forward command
    float drive_m_s = 0.0F;
    float lean = 0.0F;                 // one-motion hop: 0..1 progress of the forward lean (Dip), then 1 until the Flight
    float thrust_lean_rad = 0.0F;      // hop, Thrust: CoM line lean (hop_thrust_lean_per_m_s)
    float ready = 0.0F;                // run-up: 0..1 progress into the ready pose (Approach)
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
  // forward_m > 0: hop that far (CoM flight distance); 0: vertical jump; < 0: backwards (not supported:
  // the caller refuses it). The forward speed is forward_m / planned flight time.
  bool begin(const JumpPlan& plan, float forward_m = 0.0F) noexcept {
    if (!config_.enabled || !plan.ok || running()) return false;
    plan_ = plan;
    begin_pending_ = true;
    wait_s_ = 0.0F;
    hop_v_ = plan.flight_time_s > 0.0F && std::isfinite(forward_m)
        ? std::clamp(config_.hop_speed_scale * forward_m / plan.flight_time_s, 0.0F, config_.forward_speed_max_m_s)
        : 0.0F;
    if (hop_v_ < 0.01F) hop_v_ = 0.0F;
    drive_cur_ = hop_v_;
    braking_ = false;
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
        if (begin_pending_ && hop_v_ != 0.0F && config_.hop_run_up) {
          begin_pending_ = false;
          settled_s_ = 0.0F;
          enter(Phase::Approach);
        } else if (begin_pending_) {
          if (in.moving) {                     // a jump starts from a robot standing still
            wait_s_ += in.dt_s;
            if (wait_s_ > config_.start_wait_s) {
              begin_pending_ = false;
              out.refused = 4;
            }
          } else {
            begin_pending_ = false;
            startDip(h_now);
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
        if (in.vz_valid) vz_peak_ = std::max(vz_peak_, in.com_vz_m_s);
        // Lift-off: the measured CoM acceleration reaches -g (the wheels carry no load) after the speed
        // peak, which the leg deceleration at the end of the min-jerk path produces. Without
        // measurements the path end is the lift-off.
        if (t_ >= thrust_time_ + config_.thrust_extra_s ||
            (in.vz_valid && t_ > 0.45F * thrust_time_ &&
             in.com_az_m_s2 <= -config_.lift_off_accel_ratio * 9.80665F) ||
            (!in.vz_valid && t_ >= thrust_time_)) {
          // Lift-off: from here the flight is planned from what was measured.
          // The lift-off is seen when the legs brake at the top of the stroke: the FK speed then already
          // falls (1.03 vs 1.28 m/s from a crouch at rest, sim), while the CoM flies on at its peak.
          const float v = in.vz_valid ? std::max(in.com_vz_m_s, vz_peak_) : v_target_;
          const float hip = in.vz_valid ? in.hip_meas_rad : in.nominal_hip_rad;
          const float knee = in.vz_valid ? in.knee_meas_rad : in.nominal_knee_rad;
          // The tuck is planned once the knee has slowed
          // enough to turn around inside the acceleration budget (the legs brake to the top pose).
          // The tuck tables were built with the plan (outside the control loop); copy them.
          flight_ = plan_.curve;
          if (!(flight_.knee_step_rad > 0.0F)) flight_.buildCurve(config_.planner);
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
          braking_ = hop_v_ != 0.0F;   // hop: the speed command goes to zero from the touchdown
          stop_hold_s_ = 0.0F;
          landed_pending_ = true;  // wheels back to the balance loop, restarted here
        }
        if (t_ >= config_.land_s) enter(Phase::Recover);
        break;
      case Phase::Recover:
        if (t_ >= config_.recover_s) enter(Phase::Idle);
        break;
      case Phase::Approach:
        // Run up to the jump speed in the ready pose; dip once the speed is steady and the legs are still.
        settled_s_ = std::abs(in.ground_speed_m_s - hop_v_) < config_.approach_tolerance_m_s && !in.moving
            ? settled_s_ + in.dt_s : 0.0F;
        if (settled_s_ >= config_.approach_settle_s) {
          startDip(h_now);
        } else if (t_ >= config_.approach_timeout_s) {
          out.refused = 5;
          hop_v_ = drive_cur_ = 0.0F;
          enter(Phase::Idle);
        }
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
    // Hop: the forward speed command (held until the touchdown, then zero; the controller's forward plan,
    // limited to hop_decel_m_s2 and restarted at the CoM's speed, shapes the stop).
    if (braking_) {
      drive_cur_ = 0.0F;
      stop_hold_s_ += in.dt_s;
      if (stop_hold_s_ >= config_.hop_stop_s) braking_ = false;
    }
    out.drive = hop_v_ != 0.0F && (phase_ != Phase::Idle || braking_ || begin_pending_);
    out.drive_m_s = out.drive ? drive_cur_ : 0.0F;
    if (phase_ == Phase::Approach) {
      out.ready = minJerkShape(t_ / std::max(config_.ready_time_s, 0.05F));
      // Crouch first, then speed up: going down while speeding up swung the speed +0.26 / -0.19 m/s and
      // the run-up took ~0.3 m (sim).
      if (config_.ready_before_drive && out.ready < 1.0F) out.drive_m_s = 0.0F;
    }
    if (hop_v_ != 0.0F && !config_.hop_run_up) {
      if (phase_ == Phase::Dip) out.lean = minJerkShape(t_ / std::max(dip_time_, 0.05F));
      else if (phase_ == Phase::Thrust || phase_ == Phase::Flight) out.lean = 1.0F;
    }
    if (phase_ == Phase::Thrust) out.thrust_lean_rad = config_.hop_thrust_lean_per_m_s * hop_v_;
    if (phase_ == Phase::Idle && !braking_ && !begin_pending_) hop_v_ = 0.0F;
    return out;
  }

  void reset() noexcept {
    phase_ = Phase::Idle;
    t_ = 0.0F;
    hop_v_ = drive_cur_ = 0.0F;
    braking_ = false;
    landed_pending_ = adopt_pending_ = begin_pending_ = false;
  }
  [[nodiscard]] Phase phase() const noexcept { return phase_; }
  [[nodiscard]] float hopSpeed() const noexcept { return hop_v_; }
  [[nodiscard]] bool running() const noexcept { return phase_ != Phase::Idle || begin_pending_; }
  [[nodiscard]] const JumpPlan& plan() const noexcept { return plan_; }
  // Hip-axle height the thrust starts from (the plan's crouch less dip_extra).
  [[nodiscard]] float crouchHeight() const noexcept { return dipHeight(); }
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
      case Phase::Approach: return "Approach";
    }
    return "?";
  }

 private:
  void startDip(float h_now) noexcept {
    h_start_ = h_now;
    h_dip_ = std::min(h_now, dipHeight());
    h_ref_ = h_now;
    const float dh = h_start_ - h_dip_;
    // A few mm short of the crouch (the hop's load went down there already) is not worth a dip.
    if (dh <= 0.005F) h_dip_ = h_start_;
    dip_time_ = dh > 0.005F
        ? std::clamp(std::sqrt(4.9348F * dh / std::max(config_.dip_accel_m_s2, 0.5F)),
                     config_.dip_time_min_s, config_.dip_time_max_s)
        : 0.0F;
    enter(Phase::Dip);
  }
  [[nodiscard]] float dipHeight() const noexcept {
    return std::max(plan_.crouch_height_m - config_.dip_extra_m, LegHeight::minHeight() + 0.005F);
  }
  // End of the dip: the thrust starts from here, at rest, with a constant-acceleration path
  // that ends at the top pose with the lift-off speed.
  void startThrust() noexcept {
    h0_ = h_ref_;
    v_target_ = plan_.takeoff_speed_m_s * config_.lift_speed_factor;
    thrust_time_ = thrustTime(h0_);
    vz_peak_ = 0.0F;
    enter(Phase::Thrust);
  }
  // Min-jerk leg path: the peak CoM speed is 1.875 x (CoM height change) / time, so the time that
  // gives the lift-off speed follows from the CoM height change over the stroke (FK).
  [[nodiscard]] float thrustTime(float h_from) const noexcept {
    const float v = plan_.takeoff_speed_m_s * config_.lift_speed_factor;
    const float d_cab = std::max(LegHeight::forHeight(plan_.top_height_m).com_above_axle_m -
                                 LegHeight::forHeight(h_from).com_above_axle_m, 0.005F);
    return std::clamp(1.875F * d_cab / std::max(v, 0.2F), 0.04F, 0.5F);
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
  float vz_peak_ = 0.0F;     // peak FK CoM speed in the thrust
  float wait_s_ = 0.0F;
  float hop_v_ = 0.0F;       // hop speed (0 = vertical jump)
  float drive_cur_ = 0.0F;   // the speed command now (hop_v_, then braking to zero)
  bool braking_ = false;
  float stop_hold_s_ = 0.0F;
  float settled_s_ = 0.0F;
  bool begin_pending_ = false;
  bool landed_pending_ = false;
  bool adopt_pending_ = false;
  bool touched_ = false;
};

}  // namespace tap_fw::v3
