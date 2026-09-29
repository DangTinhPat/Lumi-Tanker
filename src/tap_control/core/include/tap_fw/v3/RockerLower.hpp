#pragma once

namespace tap_fw::v3 {

// Rocking stage 1, from the prone homing pose: the knees hold the homing stop,
// the hips fold down slowly while both wheels roll forward, then fling down
// hard and lock (Pivot: wheels held, waiting for the next stage). The wheel
// floor reaction N to whole-body COM P offset comes only from the MPU6500
// pitch and the four leg encoders through the CAD FK (WholeBodyModel);
// simulator contacts never enter it.
//
// Offline MJCF analysis (2026-09-28): both wheels together (2 x 0.13 Nm)
// cannot drag the loaded belly, so only the fling's momentum lifts it, and
// folding the hip past about 0.55 rad rolls the body over its far edge. The
// operator chose a short fling 1.5 -> 1.2 rad (N-P ~45-50 mm, body tilts
// ~0.5 rad further). Angles are native; the encoder frame subtracts 2.274 rad.
class RockerLower {
 public:
  enum class Phase { Lowering, Flinging, Pivot, Aborted };

  struct Config {
    float hip_rate_rad_s = 0.5F;
    float hip_floor_rad = 1.5F;  // slow lowering stops here at the latest
    float fling_rate_rad_s = 10.0F;
    float fling_lock_rad = 1.2F;
    float wheel_roll_rad_s = 2.0F;  // + rolls the axle toward P
    // N-P rise over its running minimum that marks the minimum as passed.
    float rise_margin_m = 0.0015F;
    // pitch + hip below this puts thigh/shin within 10 mm of the floor next
    // to the wheel (knee at the homing stop).
    float leg_angle_min_rad = 0.70F;
  };

  struct Input {
    float dt_s = 0.0F;
    float pitch_rad = 0.0F;  // MPU6500 estimate
    float hip_rad = 0.0F;    // mean of both hip encoders, native frame
    float knee_rad = 0.0F;   // mean of both knee encoders, native frame
    bool sensors_ok = false;
  };

  struct Output {
    Phase phase = Phase::Lowering;
    float hip_target_rad = 0.0F;
    float knee_target_rad = 0.0F;
    float wheel_rad_s = 0.0F;
    float n_to_p_m = 0.0F;  // + : P ahead of N along +X
  };

  RockerLower() = default;
  explicit RockerLower(Config config) : config_(config) {}

  void reset(float hip_rad, float knee_rad) noexcept;
  [[nodiscard]] Output update(const Input& in) noexcept;

  // Horizontal COM-minus-axle offset from joint encoders and IMU pitch.
  [[nodiscard]] static float nToP(float hip_rad, float knee_rad,
                                  float pitch_rad) noexcept;

 private:
  Config config_{};
  Phase phase_ = Phase::Lowering;
  float hip_cmd_rad_ = 0.0F;
  float knee_hold_rad_ = 0.0F;
  float best_n_to_p_m_ = 0.0F;  // running minimum while lowering
};

[[nodiscard]] const char* toString(RockerLower::Phase phase) noexcept;

}  // namespace tap_fw::v3
