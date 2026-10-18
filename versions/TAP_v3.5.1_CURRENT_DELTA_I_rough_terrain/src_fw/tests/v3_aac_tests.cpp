// Airborne attitude control (AAC) blocks on synthetic signals: AirborneEstimator.
#include <cassert>
#include <cmath>
#include <cstdio>

#include "tap_fw/v3/aac/AirborneEstimator.hpp"

using namespace tap_fw::v3;

namespace {

constexpr float kG = 9.80665F;
constexpr float kDt = 0.002F;

AirborneEstimator::Input still(float L = 0.0F) {
  AirborneEstimator::Input in;
  in.dt_s = kDt;
  in.vz_valid = true;
  in.angular_momentum = L;
  return in;
}

void liftOffNeedsArmingRisingCoMAndFreeFall() {
  AirborneEstimator est;
  auto in = still();
  in.com_vz_m_s = 1.0F;
  in.com_az_m_s2 = -kG;
  assert(!est.update(in).liftoff);                 // not armed (not thrusting)
  in.armed = true;
  in.com_vz_m_s = 0.1F;
  assert(!est.update(in).liftoff);                 // CoM not rising fast enough (the dip)
  in.com_vz_m_s = 1.2F;
  in.com_az_m_s2 = -2.0F;
  assert(!est.update(in).liftoff);                 // the wheels still carry load
  in.com_az_m_s2 = -0.7F * kG;
  const auto out = est.update(in);
  assert(out.liftoff && out.contact == AirborneEstimator::Contact::Air);
  assert(std::abs(out.flight_time_s - 2.0F * 1.2F / kG) < 1e-5F);
  assert(std::abs(out.time_left_s - out.flight_time_s) < 1e-5F);
}

// The lift-off is seen when the legs brake at the top of the stroke, when the FK speed already falls;
// the CoM flies at the thrust's peak (sim, from a crouch at rest: 1.03 seen vs 1.28 m/s flown).
void liftOffSpeedIsTheThrustPeak() {
  AirborneEstimator est;
  auto in = still();
  in.armed = true;
  in.com_az_m_s2 = 15.0F;
  for (float v : {0.4F, 0.9F, 1.28F}) {
    in.com_vz_m_s = v;
    assert(!est.update(in).liftoff);
  }
  in.com_vz_m_s = 1.03F;
  in.com_az_m_s2 = -4.5F * kG;
  const auto out = est.update(in);
  assert(out.liftoff && std::abs(out.liftoff_speed_m_s - 1.28F) < 1e-6F);
  assert(std::abs(out.flight_time_s - 2.0F * 1.28F / kG) < 1e-5F);
}

AirborneEstimator airborne(float v = 1.2F) {
  AirborneEstimator est;
  auto in = still();
  in.armed = true;
  in.com_vz_m_s = v;
  in.com_az_m_s2 = -kG;
  assert(est.update(in).liftoff);
  return est;
}

void touchdownEvidenceCountsOnlyNearThePredictedTime() {
  AirborneEstimator est = airborne();
  const float T = 2.0F * 1.2F / kG;
  const float early = est.config().touchdown_early_s;
  float t = 0.0F;
  auto in = still();
  in.accel_norm_m_s2 = 30.0F;                     // leg reactions shake the IMU in the air
  in.knee_rate_rad_s = -8.0F;                     // ... and the tuck folds the knees
  while (t < T - early - 2 * kDt) {
    const auto out = est.update(in);
    assert(!out.touchdown && out.contact == AirborneEstimator::Contact::Air);
    assert(std::abs(out.time_left_s - std::max(T - out.air_time_s, 0.0F)) < 1e-4F);
    t += kDt;
  }
  bool seen = false;
  for (int i = 0; i < 10 && !seen; ++i) seen = est.update(in).touchdown;
  assert(seen && est.contact() == AirborneEstimator::Contact::Ground);
}

void eachEvidenceEndsTheFlightAndTheTimeoutIsLast() {
  const float T = 2.0F * 1.2F / kG;
  for (int kind = 0; kind < 4; ++kind) {
    AirborneEstimator est = airborne();
    est.config().touchdown_knee_current_a = 3.0F;
    auto in = still();
    float t = 0.0F;
    AirborneEstimator::Output out{};
    while (t < T + 0.2F) {
      const bool late = t >= T - 0.01F;
      in.accel_norm_m_s2 = (kind == 0 && late) ? 25.0F : 2.0F;
      in.knee_rate_rad_s = (kind == 1 && late) ? -6.0F : 1.0F;
      in.current_valid = true;
      in.knee_current_a = (kind == 2 && late) ? 9.0F : 2.0F;
      out = est.update(in);
      t += kDt;
      if (out.touchdown) break;
    }
    assert(out.touchdown);
    if (kind < 3) {
      assert(!out.touchdown_assumed && std::abs(t - (T - 0.01F)) < 3 * kDt);
    } else {
      assert(out.touchdown_assumed && std::abs(t - (T + est.config().touchdown_late_s)) < 3 * kDt);
    }
  }
}

void momentumIsAveragedAndHeldInTheAir() {
  AirborneEstimator est;
  // On the ground L follows the samples (they change while the wheels push).
  auto in = still(0.01F);
  AirborneEstimator::Output out{};
  for (int i = 0; i < 50; ++i) out = est.update(in);
  assert(std::abs(out.angular_momentum - 0.01F) < 1e-4F);
  in.angular_momentum = -0.02F;
  for (int i = 0; i < 50; ++i) out = est.update(in);
  assert(std::abs(out.angular_momentum + 0.02F) < 1e-3F);
  // In the air L is conserved: noisy samples average out.
  in.armed = true;
  in.com_vz_m_s = 1.2F;
  in.com_az_m_s2 = -kG;
  assert(est.update(in).liftoff);
  in.armed = false;
  float worst = 0.0F;
  for (int i = 0; i < 100; ++i) {
    in.angular_momentum = -0.02F + ((i % 2) != 0 ? 0.006F : -0.006F);   // gyro/encoder noise
    out = est.update(in);
    if (i > 20) worst = std::max(worst, std::abs(out.angular_momentum + 0.02F));
  }
  std::printf("L in the air with +-0.006 sample noise: worst error %.5f N.m.s (std %.5f)\n", worst,
              out.angular_momentum_std);
  assert(worst < 0.0015F);
}

}  // namespace

int main() {
  liftOffNeedsArmingRisingCoMAndFreeFall();
  liftOffSpeedIsTheThrustPeak();
  touchdownEvidenceCountsOnlyNearThePredictedTime();
  eachEvidenceEndsTheFlightAndTheTimeoutIsLast();
  momentumIsAveragedAndHeldInTheAir();
  std::puts("v3 aac tests passed");
  return 0;
}
