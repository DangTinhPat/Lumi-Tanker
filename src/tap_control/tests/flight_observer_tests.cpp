#include <cmath>
#include <iostream>

#include "robot_control/FlightObserver.hpp"

using namespace robot_control;

namespace {
int failures = 0;

void check(bool condition, const char* name) {
  std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
  if (!condition) ++failures;
}

RobotState baseState() {
  RobotState state;
  state.dt_s = 0.001F;
  state.floor_contact = true;
  state.left_floor_contact = true;
  state.right_floor_contact = true;
  state.imu.accel_magnitude_m_s2 = 9.81F;
  return state;
}

void shortStepDoesNotBecomeFlight() {
  FlightObserver observer;
  auto state = baseState();
  observer.reset(state);
  for (int i = 0; i < 5; ++i) {
    // There is no contact switch in the hardware contract. A brief
    // non-ballistic interval is therefore only a normal supported sample.
    state.imu.accel_magnitude_m_s2 = 9.81F;
    const auto out = observer.update(state);
    check(!out.support_lost && !out.airborne,
          "non-ballistic interval remains grounded");
  }
  const auto out = observer.update(state);
  check(out.phase == FlightObserver::Phase::Grounded,
        "supported IMU samples remain grounded");
}

void BallisticFlightRequiresDebouncedLoss() {
  FlightObserver observer;
  auto state = baseState();
  observer.reset(state);
  // Support loss has two sequential debounce windows: free-fall detection
  // followed by support-loss confirmation.
  for (int i = 0; i < 40; ++i) {
    state.imu.accel_magnitude_m_s2 = 0.2F;
    (void)observer.update(state);
  }
  const auto out = observer.update(state);
  check(out.support_lost && out.airborne && out.ballistic,
        "debounced unloading becomes ballistic flight");
  check(out.downward_velocity_m_s > 0.15F,
        "ballistic observer integrates downward velocity");
}

void EdgeShockIsNotTouchdown() {
  FlightObserver observer;
  auto state = baseState();
  observer.reset(state);
  for (int i = 0; i < 20; ++i) {
    state.imu.accel_magnitude_m_s2 = 0.2F;
    (void)observer.update(state);
  }
  bool saw_collision = false;
  for (int i = 0; i < 40; ++i) {
    state.imu.accel_magnitude_m_s2 = 40.0F;
    state.imu.hard_impact_event = true;
    const auto out = observer.update(state);
    saw_collision = saw_collision || out.collision_candidate;
  }
  check(saw_collision, "IMU shock becomes collision candidate");
  state.imu.hard_impact_event = false;
  state.imu.accel_magnitude_m_s2 = 9.81F;
  auto out = observer.update(state);
  check(out.touchdown_candidate && out.airborne,
        "impact recovery starts as touchdown candidate");
  for (int i = 0; i < 20; ++i) out = observer.update(state);
  check(out.phase == FlightObserver::Phase::Grounded,
        "persistent IMU support validates touchdown");
}
}  // namespace

int main() {
  shortStepDoesNotBecomeFlight();
  BallisticFlightRequiresDebouncedLoss();
  EdgeShockIsNotTouchdown();
  return failures == 0 ? 0 : 1;
}
