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
    state.floor_contact = false;
    const auto out = observer.update(state);
    check(!out.support_lost && !out.airborne,
          "short contact gap remains transient unloading");
  }
  state.floor_contact = true;
  const auto out = observer.update(state);
  check(out.phase == FlightObserver::Phase::Grounded,
        "short step returns directly to grounded");
}

void BallisticFlightRequiresDebouncedLoss() {
  FlightObserver observer;
  auto state = baseState();
  observer.reset(state);
  for (int i = 0; i < 20; ++i) {
    state.floor_contact = false;
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
  state.floor_contact = false;
  state.imu.accel_magnitude_m_s2 = 40.0F;
  for (int i = 0; i < 40; ++i) {
    const auto out = observer.update(state);
    if (i == 30) {
      check(out.collision_candidate,
            "airborne shock is collision candidate");
    }
  }
  state.floor_contact = true;
  state.left_floor_contact = true;
  state.right_floor_contact = true;
  auto out = observer.update(state);
  check(out.touchdown_candidate && out.airborne,
        "first re-contact remains touchdown candidate");
  out = observer.update(state);
  check(out.touchdown_candidate && out.airborne,
        "touchdown candidate persists during contact debounce");
  for (int i = 0; i < 20; ++i) out = observer.update(state);
  check(out.phase == FlightObserver::Phase::Grounded,
        "persistent re-contact validates touchdown");
}

void PartialSupportIsNotFlight() {
  FlightObserver observer;
  auto state = baseState();
  state.right_floor_contact = false;
  observer.reset(state);
  const auto out = observer.update(state);
  check(out.phase == FlightObserver::Phase::PartialSupport &&
            !out.airborne && out.support_settling,
        "one-sided support is partial support");

  for (int i = 0; i < 120; ++i) {
    (void)observer.update(state);
  }
  const auto settled = observer.update(state);
  check(settled.phase == FlightObserver::Phase::PartialSupport &&
            !settled.airborne && !settled.support_settling,
        "partial support settling expires after the support contract dwell");
}
}  // namespace

int main() {
  shortStepDoesNotBecomeFlight();
  BallisticFlightRequiresDebouncedLoss();
  EdgeShockIsNotTouchdown();
  PartialSupportIsNotFlight();
  return failures == 0 ? 0 : 1;
}
