#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>

#include "CurrentTorqueActuator.hpp"
#include "LegPlant.hpp"
#include "tap_fw/v3/LegHeight.hpp"

using tap_fw::sim::CurrentTorqueActuator;

namespace {

double transientError(double correction_gain) {
  CurrentTorqueActuator actuator({.47, 8.0, .02, correction_gain, 2});
  double error = 0.0;
  constexpr double dt = .002;
  for (int i = 0; i < 50; ++i) {
    const auto out = actuator.step(1.0, dt);
    error += std::abs(1.0 - out.applied_torque_nm) * dt;
    assert(std::isfinite(out.measured_current_a));
    assert(std::abs(out.measured_current_a) <= 8.0);
  }
  return error;
}

double cadLegTrackingError(double correction_gain) {
  using tap_fw::sim::LegPlant;
  // Reduced CAD leg dynamics: axle/body coordinates are constrained; hip and
  // knee remain coupled through the CAD-derived 2x2 mass matrix.
  LegPlant plant(0.0, 1.0, 0.0, 0.0, tap_fw::v3::LegHeight::hangHipCad());
  LegPlant::Vec s{}, sd{};
  CurrentTorqueActuator left_hip({.47, 8.0, .02, correction_gain, 2});
  CurrentTorqueActuator right_hip({.47, 8.0, .02, correction_gain, 2});
  CurrentTorqueActuator left_knee({.47, 8.0, .02, correction_gain, 2});
  CurrentTorqueActuator right_knee({.47, 8.0, .02, correction_gain, 2});
  constexpr double dt = .002;
  constexpr double hip_ref = .035, knee_ref = -.025;
  double error_sq = 0.0;
  unsigned samples = 0;
  for (int tick = 0; tick < 1500; ++tick) {
    const double t = tick * dt;
    const double q_ref = t < .1 ? 0.0 : hip_ref;
    const double k_ref = t < .1 ? 0.0 : knee_ref;
    const auto gravity = plant.staticJointTorques(s);
    const double hip_total = gravity[0] + 8.0 * (q_ref - s[2]) - .8 * sd[2];
    const double knee_total = gravity[1] + 8.0 * (k_ref - s[3]) - .8 * sd[3];
    const double hip_tau = left_hip.step(.5 * hip_total, dt).applied_torque_nm +
                           right_hip.step(.5 * hip_total, dt).applied_torque_nm;
    const double knee_tau = left_knee.step(.5 * knee_total, dt).applied_torque_nm +
                            right_knee.step(.5 * knee_total, dt).applied_torque_nm;

    double mass[LegPlant::kDof][LegPlant::kDof];
    double force[LegPlant::kDof];
    plant.generalized(s, sd, 0.0, 0.0, mass, force);
    const double a = mass[2][2], b = mass[2][3], c = mass[3][2], d = mass[3][3];
    const double det = a * d - b * c;
    assert(std::isfinite(det) && std::abs(det) > 1.0e-10);
    const double hip_acc = ((force[2] + hip_tau) * d - b * (force[3] + knee_tau)) / det;
    const double knee_acc = (a * (force[3] + knee_tau) - c * (force[2] + hip_tau)) / det;
    sd[2] += hip_acc * dt;
    sd[3] += knee_acc * dt;
    s[2] += sd[2] * dt;
    s[3] += sd[3] * dt;
    if (t >= .1) {
      error_sq += (q_ref - s[2]) * (q_ref - s[2]) +
                  (k_ref - s[3]) * (k_ref - s[3]);
      ++samples;
    }
  }
  return std::sqrt(error_sq / (2.0 * samples));
}

}  // namespace

int main() {
  // The modeled delta-I correction should reduce transient torque error under
  // the declared lag/delay, while the zero-gain case remains the baseline.
  const double baseline_error = transientError(0.0);
  const double corrected_error = transientError(0.6);
  assert(corrected_error < baseline_error);
  const double cad_baseline_error = cadLegTrackingError(0.0);
  const double cad_corrected_error = cadLegTrackingError(0.6);
  assert(cad_corrected_error < cad_baseline_error);

  CurrentTorqueActuator limited({.47, 3.0, .01, 1.0, 0});
  CurrentTorqueActuator::Output out{};
  for (int i = 0; i < 100; ++i) out = limited.step(10.0, .002);
  assert(std::abs(out.measured_current_a) <= 3.0);
  assert(std::abs(out.applied_torque_nm) <= .47 * 3.0);
  assert(std::abs(out.applied_torque_nm - .47 * 3.0) < 1.0e-6);

  const auto held = limited.step(1.0, 0.0);
  assert(held.applied_torque_nm == out.applied_torque_nm);
  CurrentTorqueActuator::Config dropout_cfg{.47, 3.0, .01, 1.0, 0};
  dropout_cfg.feedback_sample_period_ticks = 1;
  dropout_cfg.feedback_max_age_ticks = 1;
  dropout_cfg.feedback_drop_period_ticks = 4;
  dropout_cfg.feedback_drop_ticks = 3;
  CurrentTorqueActuator dropout(dropout_cfg);
  bool saw_invalid = false;
  for (int i = 0; i < 16; ++i) {
    const auto sample = dropout.step(1.0, .002);
    if (!sample.feedback_valid) {
      saw_invalid = true;
      assert(sample.delta_current_a == 0.0);
    }
  }
  assert(saw_invalid);
  std::printf("v3 current model: torque transient %.5f -> %.5f N.m*s; "
              "CAD leg RMS %.5f -> %.5f rad; limit %.2f A\n",
              baseline_error, corrected_error, cad_baseline_error,
              cad_corrected_error, out.measured_current_a);
}
