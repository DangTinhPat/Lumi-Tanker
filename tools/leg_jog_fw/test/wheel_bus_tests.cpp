// Host tests for the bench wheel logic (boot mode sync, silent-wheel back-off):
//   g++ -std=c++17 -Wall -Wextra -Wpedantic -UNDEBUG
//     -Itools/leg_jog_fw/src -Isrc/tap_control/hal/include
//     tools/leg_jog_fw/test/wheel_bus_tests.cpp -o /tmp/wheel_bus_tests
#include <cassert>
#include <cstdio>
#include <cstring>
#include <string>

#include "LegJog.hpp"

namespace {
namespace frame = robot_control::motor_frame;

// Two fake BL4012 drivers, one per bus: bus 0 = id 1 (right), bus 1 = id 2 (left).
struct Drivers final : legjog::TxSink {
  legjog::LegJog* jog = nullptr;
  std::uint64_t* now = nullptr;
  std::uint8_t mode[2]{1, 1};
  bool answer[2]{true, true};
  int mode_writes[2]{0, 0};
  int transactions[2]{0, 0};
  bool torque[2]{false, false};
  int speed_writes[2]{0, 0};
  legjog::DxlStreamParser parser[2];

  bool send(std::uint16_t, const frame::MitBytes&) override { return true; }
  void reply(std::uint8_t bus, const frame::DxlFrame& f) {
    for (std::uint8_t i = 0U; i < f.len; ++i) {
      frame::DxlFrame out{};
      if (parser[bus].push(f.bytes[i], out)) jog->onUart(bus, out, *now);
    }
  }
  bool sendUart(std::uint8_t bus, const frame::DxlFrame& f) override {
    ++transactions[bus];
    const std::uint8_t id = bus == 0U ? 1U : 2U;
    if (f.bytes[2] != id || !answer[bus]) return true;
    const std::uint8_t ins = f.bytes[4], addr = f.bytes[5];
    if (ins == frame::kDxlInsRead && addr == frame::kDxlVelocityModeAddr) {
      const std::uint8_t p[4] = {1U, ins, addr, mode[bus]};
      reply(bus, frame::detail::buildFrame(id, 0, p, 4));
    } else if (ins == frame::kDxlInsRead && addr == frame::kDxlPresentAddr) {
      reply(bus, frame::buildPresentResponse(id, 100U, 0, 0));
    } else if (ins == frame::kDxlInsWrite) {
      if (addr == frame::kDxlVelocityModeAddr) { mode[bus] = f.bytes[6]; ++mode_writes[bus]; }
      if (addr == frame::kDxlTorqueEnableAddr) torque[bus] = f.bytes[6] != 0U;
      if (addr == frame::kDxlMovingSpeedAddr) ++speed_writes[bus];
      reply(bus, frame::buildWriteAck(id, f.bytes[3] - 2U, addr));
    }
    return true;
  }
};

struct Rig {
  Drivers d;
  legjog::LegJog jog{d};
  std::uint64_t now = 1'000'000U;
  Rig() { d.jog = &jog; d.now = &now; }
  void run(double seconds, const char* wheel_cmd = nullptr) {
    const int steps = static_cast<int>(seconds / 0.01 + 0.5);
    char reply[160];
    for (int i = 0; i < steps; ++i) {
      now += 10'000U;
      jog.onLine("HB", now, reply, sizeof(reply));
      if (wheel_cmd) jog.onLine(wheel_cmd, now, reply, sizeof(reply));
      jog.tick(now);
    }
  }
};

void speedModeDriversAreNeverWritten() {
  Rig r;
  r.run(1.0);
  assert(r.d.mode_writes[0] == 0 && r.d.mode_writes[1] == 0);   // read-only boot sync
  r.run(0.5, "WHEEL 60 60");
  assert(r.d.torque[0] && r.d.torque[1] && r.d.speed_writes[0] > 0 && r.d.speed_writes[1] > 0);
}

void aDriverLeftInPositionModeIsWrittenOnceAndVerified() {
  Rig r;
  r.d.mode[0] = 2;                                              // right wheel stuck in Position mode
  r.run(1.0);
  assert(r.d.mode_writes[0] == 1 && r.d.mode_writes[1] == 0 && r.d.mode[0] == 1);
  r.run(0.5, "WHEEL 60 60");
  assert(r.d.torque[0] && r.d.speed_writes[0] > 0);
}

void aDriverThatDoesNotAnswerIsNeverWrittenNorDriven() {
  Rig r;
  r.d.answer[0] = false;
  r.d.mode[0] = 2;
  r.run(5.0, "WHEEL 60 60");
  assert(r.d.mode_writes[0] == 0);                              // no blind flash write
  assert(!r.d.torque[0] && r.d.speed_writes[0] == 0);           // unknown mode: not driven
  assert(r.d.torque[1] && r.d.speed_writes[1] > 0);             // the other wheel is unaffected
  r.d.answer[0] = true;                                         // it comes back: sync, fix the mode, drive
  r.run(3.0, "WHEEL 60 60");
  assert(r.d.mode_writes[0] == 1 && r.d.mode[0] == 1 && r.d.torque[0] && r.d.speed_writes[0] > 0);
}

void aSilentWheelIsProbedSlowly() {
  Rig r;
  r.run(1.0);
  r.d.answer[1] = false;                                        // left wheel dies after boot
  r.run(0.5, "WHEEL 60 60");                                    // let it be noticed as silent
  const int before0 = r.d.transactions[0], before1 = r.d.transactions[1];
  r.run(2.0, "WHEEL 60 60");
  const int silent = r.d.transactions[1] - before1, healthy = r.d.transactions[0] - before0;
  assert(silent <= 2 * (2000 / 100) + 4);                       // <= 2 per 100 ms probe
  assert(healthy > 4 * silent);                                 // the live wheel keeps its full rate
  r.d.answer[1] = true;                                         // recovery is noticed within a probe
  r.run(0.5, "WHEEL 60 60");
  assert(r.jog.wheel(0).fb_valid);
}

}  // namespace

int main() {
  speedModeDriversAreNeverWritten();
  aDriverLeftInPositionModeIsWrittenOnceAndVerified();
  aDriverThatDoesNotAnswerIsNeverWrittenNorDriven();
  aSilentWheelIsProbedSlowly();
  std::puts("wheel bus tests passed");
  return 0;
}
