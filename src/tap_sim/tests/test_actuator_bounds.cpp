#include <limits>
#include <stdexcept>
#include "tap_sim/actuator_bounds.hpp"
#include "tap_sim/actuator_gate.hpp"
#include "tap_sim/leg_velocity_reference.hpp"

int main()
{
  const auto check = [](bool valid) {if (!valid) {throw std::runtime_error("Actuator bound failed");}};
  using tap_sim::boundedTorque;
  tap_sim::ActuatorGate startup;
  startup.receive(true, 0); check(!startup.enabled(0));
  startup.receive(false, 0); check(!startup.enabled(.5));
  startup.receive(true, 1); check(startup.enabled(1));
  tap_sim::ActuatorGate gate;
  check(!gate.enabled(0));
  gate.receive(true, 0); check(!gate.enabled(0));
  gate.receive(false, 0); gate.receive(true, .01); check(gate.enabled(.02));
  gate.stop(true); check(!gate.enabled(.02));
  gate.stop(false); gate.receive(true, .03); check(!gate.enabled(.03));
  gate.receive(false, .04); gate.receive(true, .05); check(gate.enabled(.06));
  check(!gate.enabled(.16));
  gate.receive(true, .17); check(!gate.enabled(.17));
  tap_sim::ActuatorGate resumed;
  resumed.receive(false, 0);
  resumed.receive(true, .01);
  resumed.receive(true, 1);
  check(!resumed.enabled(1));
  resumed.receive(true, 1.01); check(!resumed.enabled(1.01));
  resumed.receive(false, 1.02); check(!resumed.enabled(1.02));
  resumed.receive(true, 1.03); check(resumed.enabled(1.03));
  resumed.receive(false, 2); check(!resumed.enabled(2));
  resumed.receive(true, 2.01); check(resumed.enabled(2.01));
  check(boundedTorque(false, 100, 0, 12.566) == 5);
  check(boundedTorque(false, -100, 0, 12.566) == -5);
  check(boundedTorque(false, 5, 13, 12.566) == 0);
  check(boundedTorque(false, -5, -13, 12.566) == 0);
  check(boundedTorque(false, -100, 13, 12.566) == -5);
  check(boundedTorque(false, 100, -13, 12.566) == 5);
  check(boundedTorque(true, 100, 0, 12.566) == .13);
  check(boundedTorque(true, -100, 0, 12.566) == -.13);
  check(boundedTorque(true, 0, 1000, 12.566) == -.13);
  check(boundedTorque(false, std::numeric_limits<double>::quiet_NaN(), 0, 12.566) == 0);
  check(boundedTorque(true, 0, std::numeric_limits<double>::infinity(), 12.566) == 0);
  check(boundedTorque(false, 5, 19, 28.274333882308138) == 5);
  check(boundedTorque(false, 5, 24, 28.274333882308138) > 2.5);
  check(boundedTorque(false, 5, 24, 28.274333882308138) < 2.6);
  check(boundedTorque(false, 5, 29, 28.274333882308138) == 0);
  check(boundedTorque(false, 3, 10, 12.566, .1) == 2);
  check(boundedTorque(false, 3, 10, 12.566, .2) == 1);
  check(boundedTorque(false, 0, 100, 12.566, .2) == -5);
  check(boundedTorque(false, 0, -100, 12.566, .2) == 5);
  check(boundedTorque(false, 3, 20, 12.566, .2) == -1);
  check(boundedTorque(false, 5, 20, 12.566, .05) == 0);
  check(boundedTorque(false, 100, 24, 28.274333882308138, .2) < 2.6);
  check(boundedTorque(true, 0, 100, 12.566, .2) == -.13);
  check(boundedTorque(false, 3, 1, 12.566, -1) == 0);
  check(boundedTorque(false, 3, 10, 12.566, .6, 10) == 3);
  check(boundedTorque(false, 3, 0, 12.566, .6, 10) == 5);
  check(boundedTorque(false, 3, 20, 12.566, .6, 10) == -3);
  check(boundedTorque(false, 3, 24, 28.274333882308138, .6, 24) < 2.6);
  tap_sim::LegVelocityReference ref;
  std::array<double, 4> values{};
  check(ref.sample(1, 1, values) && values[0] == 0);
  std::vector<std::string> names(tap_sim::LegVelocityReference::names.begin(),
    tap_sim::LegVelocityReference::names.end());
  ref.receive(names, {1, 2, 3, 100}, 1, 1, 12.566);
  check(ref.sample(1.01, 1.01, values) && values[3] == 12.566);
  check(!ref.sample(1.11, 1.01, values));
  check(!ref.sample(1.01, 1.11, values));
  check(!ref.sample(.99, 1.01, values));
  std::swap(names[0], names[3]);
  ref.receive(names, {4, 2, 3, 1}, 2, 2, 12.566);
  check(ref.sample(2, 2, values) && values[0] == 1 && values[3] == 4);
  names[1] = names[0];
  ref.receive(names, {4, 2, 3, 1}, 2, 2, 12.566);
  check(!ref.sample(2, 2, values));
  names.assign(tap_sim::LegVelocityReference::names.begin(), tap_sim::LegVelocityReference::names.end());
  ref.receive(names, {0, 0, 0, std::numeric_limits<double>::quiet_NaN()}, 2, 2, 12.566);
  check(!ref.sample(2, 2, values));
  ref.receive(names, {0, 0, 0}, 2, 2, 12.566);
  check(!ref.sample(2, 2, values));
  ref.receive(names, {0, 0, 0, 0}, 0, 2, 12.566);
  check(!ref.sample(2, 2, values));
  ref.receive(names, {0, 0, 0, 0}, 3, 3, 12.566);
  check(ref.sample(3, 3, values));
  tap_sim::LegVelocityReference guarded;
  tap_sim::ActuatorGate reference_gate;
  reference_gate.receive(false, 0);
  guarded.receive(names, {0, 0, 0, 0}, 0, 0, 12.566);
  check(!guarded.allowTorque(reference_gate, 0, 0, values));
  guarded.receive(names, {1, 2, 3, 4}, 1, 1, 12.566);
  reference_gate.receive(true, 1);
  check(guarded.allowTorque(reference_gate, 1, 1, values));
  // Enable heartbeat remains fresh, but the reference stream expires.
  reference_gate.receive(true, 1.05);
  reference_gate.receive(true, 1.1);
  check(!guarded.allowTorque(reference_gate, 1.11, 1.11, values));
  guarded.receive(names, {0, 0, 0, 0}, 1.12, 1.12, 12.566);
  reference_gate.receive(true, 1.12);
  check(!guarded.allowTorque(reference_gate, 1.12, 1.12, values));
  reference_gate.receive(false, 1.13); reference_gate.receive(true, 1.14);
  check(guarded.allowTorque(reference_gate, 1.14, 1.14, values));
  // Malformed then fresh callbacks before a write cannot hide a loss.
  guarded.receive(names, {0}, 1.15, 1.15, 12.566);
  guarded.receive(names, {0, 0, 0, 0}, 1.16, 1.16, 12.566);
  check(!guarded.allowTorque(reference_gate, 1.16, 1.16, values));
  reference_gate.receive(false, 1.17); reference_gate.receive(true, 1.18);
  check(guarded.allowTorque(reference_gate, 1.18, 1.18, values));
  reference_gate.receive(true, 1.23); reference_gate.receive(true, 1.28);
  guarded.receive(names, {0, 0, 0, 0}, 1.29, 1.29, 12.566);
  check(!guarded.allowTorque(reference_gate, 1.29, 1.29, values));
}
