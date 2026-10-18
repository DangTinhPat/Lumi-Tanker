// Host tests for the leg-jog DRAFT logic:
//   g++ -std=c++17 -Wall -Wextra -Wpedantic -UNDEBUG
//     -Itools/leg_jog_fw/src -Isrc/tap_control/hal/include
//     tools/leg_jog_fw/test/leg_jog_tests.cpp -o /tmp/leg_jog_tests

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "LegJog.hpp"

namespace {

using legjog::Mode;
namespace frame = robot_control::motor_frame;

struct Frame {
  std::uint16_t id;
  frame::MitBytes data;
};

struct UartFrame {
  std::uint8_t bus;
  frame::DxlFrame frame;
};

struct FakeSink final : legjog::TxSink {
  std::vector<Frame> frames;
  std::vector<UartFrame> uart;
  bool fail = false;
  bool send(std::uint16_t id, const frame::MitBytes& data) override {
    if (fail) return false;
    frames.push_back({id, data});
    return true;
  }
  bool sendUart(std::uint8_t bus, const frame::DxlFrame& f) override {
    uart.push_back({bus, f});
    return true;
  }
};

// Four fake GDS68 drivers: node -> driver-frame position, axis state.
struct Bench {
  FakeSink sink;
  legjog::LegJog jog{sink};
  std::uint64_t now = 1'000'000U;
  float driver_pos[5]{0.0F, 0.10F, -0.20F, 0.30F, -0.40F};  // index = node
  std::uint8_t axis_state[5]{1, 1, 1, 1, 1};
  bool follow = true;       // Drivers track the commanded MIT position.
  bool echo_ok = true;
  float torque = 0.0F;
  std::uint8_t mit_dlc = 6U;  // GDS68 MIT feedback uses bytes 0-5 only.
  // Two fake BL4012 wheels.
  bool wheel_torque[2]{false, false};
  std::int16_t wheel_speed[2]{0, 0};
  std::uint16_t wheel_pos[2]{100U, 900U};
  bool wheel_answer = true;
  legjog::DxlStreamParser parsers[2];

  void feedback(std::uint8_t node) {
    frame::MitBytes b{};
    frame::packMitFeedback(echo_ok ? node : static_cast<std::uint8_t>(node + 1U),
                           driver_pos[node], 0.0F, torque, b);
    jog.onCan(frame::canId(node, frame::kMitControlCmdId), mit_dlc, b.data(), now);
  }
  void heartbeat(std::uint8_t node) {
    frame::MitBytes b{};
    b[4] = axis_state[node];
    jog.onCan(frame::canId(node, frame::kHeartbeatCmdId), 8U, b.data(), now);
  }
  // Drivers react to what was sent, then answer.
  void reactAndAnswer() {
    for (const auto& f : sink.frames) {
      const auto node = static_cast<std::uint8_t>(f.id >> 5U);
      const auto cmd = static_cast<std::uint8_t>(f.id & 0x1FU);
      if (node < 1U || node > 4U) continue;
      if (cmd == frame::kSetAxisStateCmdId) {
        axis_state[node] = static_cast<std::uint8_t>(frame::unpackSetAxisState(f.data));
      } else if (cmd == frame::kMitControlCmdId && follow && axis_state[node] == 8U) {
        driver_pos[node] = frame::unpackMitCommand(f.data).pos_rad;
      }
    }
    sink.frames.clear();
    for (std::uint8_t node = 1U; node <= 4U; ++node) feedback(node);
    for (const auto& u : sink.uart) {
      const auto write = frame::parseDxlWrite(u.frame);
      if (write.ok && write.address == frame::kDxlTorqueEnableAddr) {
        wheel_torque[u.bus] = write.data[0] != 0U;
        if (!wheel_torque[u.bus]) wheel_speed[u.bus] = 0;
      } else if (write.ok && write.address == frame::kDxlMovingSpeedAddr &&
                 wheel_torque[u.bus]) {
        wheel_speed[u.bus] = static_cast<std::int16_t>(
            write.data[0] | (write.data[1] << 8));
      } else if (frame::parseDxlRead(u.frame).ok && wheel_answer) {
        const auto response = frame::buildPresentResponse(
            1U, wheel_pos[u.bus], wheel_speed[u.bus], 0);
        for (std::uint8_t i = 0U; i < response.len; ++i) {
          frame::DxlFrame out{};
          if (parsers[u.bus].push(response.bytes[i], out)) jog.onUart(u.bus, out, now);
        }
      }
    }
    sink.uart.clear();
  }
  void step(std::uint64_t dt_us = 10'000U, bool host = true) {
    now += dt_us;
    if (host) command("HB");
    if (now % 100'000U < dt_us) {
      for (std::uint8_t node = 1U; node <= 4U; ++node) heartbeat(node);
    }
    jog.tick(now);
    reactAndAnswer();
  }
  std::string command(const char* text) {
    char reply[160];
    jog.onLine(text, now, reply, sizeof(reply));
    return reply;
  }
  void run(double seconds, bool host = true) {
    const auto steps = static_cast<int>(seconds / 0.01 + 0.5);
    for (int i = 0; i < steps; ++i) step(10'000U, host);
  }
  std::vector<Frame> lastTickFrames() {
    now += 10'000U;
    command("HB");
    jog.tick(now);
    auto out = sink.frames;
    reactAndAnswer();
    return out;
  }
};

bool near(float a, float b, float eps = 2.0e-3F) { return std::abs(a - b) <= eps; }

bool contains(const std::string& text, const char* part) {
  return text.find(part) != std::string::npos;
}

void engage(Bench& bench) {
  for (std::uint8_t node = 1U; node <= 4U; ++node) {
    bench.feedback(node);
    bench.heartbeat(node);
  }
  const auto reply = bench.command("ENABLE");
  assert(contains(reply, "EV OK ENABLE"));
  bench.run(0.6);
  assert(bench.jog.mode() == Mode::Active);
}

}  // namespace

int main() {
  // Fake wheel drivers see driver raw = core raw * driver_sign.
  const int kS = static_cast<int>(robot_control::motor_mapping::kWheelUartBindings[0].driver_sign);
  // Disabled: IDLE to all 4 nodes plus zero-gain MIT probes.
  {
    Bench bench;
    bench.now += 10'000U;
    bench.jog.tick(bench.now);
    int idle = 0;
    int probes = 0;
    for (const auto& f : bench.sink.frames) {
      const auto cmd = f.id & 0x1FU;
      if (cmd == frame::kSetAxisStateCmdId) {
        assert(frame::unpackSetAxisState(f.data) == frame::kAxisStateIdle);
        ++idle;
      } else if (cmd == frame::kMitControlCmdId) {
        const auto mit = frame::unpackMitCommand(f.data);
        assert(mit.kp == 0.0F && mit.kd == 0.0F && near(mit.tff_nm, 0.0F, 0.03F));
        ++probes;
      }
    }
    assert(idle == 4 && probes == 4);
  }

  // ENABLE refused without feedback, with a too-short frame, and with a
  // wrong node echo. DLC 8 MIT feedback is accepted as well as DLC 6.
  {
    Bench bench;
    assert(contains(bench.command("ENABLE"), "no MIT feedback"));
    bench.mit_dlc = 5U;
    for (std::uint8_t node = 1U; node <= 4U; ++node) bench.feedback(node);
    assert(contains(bench.command("ENABLE"), "no MIT feedback"));
    bench.mit_dlc = 8U;
    bench.feedback(3U);
    assert(bench.jog.joint(0).fb_valid);
    bench.mit_dlc = 6U;
    bench.echo_ok = false;
    for (std::uint8_t node = 1U; node <= 4U; ++node) bench.feedback(node);
    assert(contains(bench.command("ENABLE"), "bad_echo=1"));
    assert(bench.jog.mode() == Mode::Disabled);
  }

  // ENABLE: controller mode + closed loop, start at the measured pose, gains
  // ramp from zero, then ACTIVE at full gains.
  {
    Bench bench;
    for (std::uint8_t node = 1U; node <= 4U; ++node) {
      bench.feedback(node);
      bench.heartbeat(node);
    }
    bench.sink.frames.clear();
    assert(contains(bench.command("ENABLE"), "EV OK ENABLE"));
    assert(bench.jog.mode() == Mode::Engaging);
    int modes = 0;
    int closed = 0;
    for (const auto& f : bench.sink.frames) {
      const auto cmd = f.id & 0x1FU;
      if (cmd == frame::kSetControllerModeCmdId) ++modes;
      if (cmd == frame::kSetAxisStateCmdId &&
          frame::unpackSetAxisState(f.data) == frame::kAxisStateClosedLoop) ++closed;
    }
    assert(modes == 4 && closed == 4);
    bench.reactAndAnswer();

    // LH = node 3, sign +1; RH = node 1, sign -1 (MotorMapping.hpp).
    assert(near(bench.jog.joint(0).start, 0.30F));
    assert(near(bench.jog.joint(2).start, -0.10F));
    const auto first = bench.lastTickFrames();
    for (const auto& f : first) {
      const auto mit = frame::unpackMitCommand(f.data);
      assert(mit.kp < 0.5F);  // Gain ramp starts near zero.
    }
    bench.run(0.6);
    assert(bench.jog.mode() == Mode::Active);
    const auto full = bench.lastTickFrames();
    for (const auto& f : full) {
      const auto node = f.id >> 5U;
      const auto mit = frame::unpackMitCommand(f.data);
      assert(near(mit.kp, 8.0F, 0.15F) && near(mit.kd, 0.6F, 0.01F));
      assert(near(mit.pos_rad, bench.driver_pos[node]));  // Held where it was.
    }
  }

  // JOG in the core frame: +hip on both legs is the same mechanical
  // direction, so the right hip's driver position moves the other way.
  {
    Bench bench;
    engage(bench);
    const float lh_driver = bench.driver_pos[3];
    const float rh_driver = bench.driver_pos[1];
    bench.command("JOG 0.5 0 0.5 0");
    bench.run(0.4);
    bench.command("JOG 0 0 0 0");
    bench.run(0.1);
    assert(near(bench.driver_pos[3] - lh_driver, 0.2F, 0.01F));
    assert(near(bench.driver_pos[1] - rh_driver, -0.2F, 0.01F));
    assert(near(bench.jog.joint(0).target - bench.jog.joint(0).start, 0.2F, 0.01F));

    // Travel limit around the ENABLE pose.
    bench.command("LIMIT 0.3");
    bench.command("JOG 1.5 0 0 0");
    bench.run(1.0);
    assert(near(bench.jog.joint(0).target - bench.jog.joint(0).start, 0.3F, 1.0e-4F));
    bench.command("JOG 0 0 0 0");

    // GOTO ramps at SPEED and is clamped to the travel window. LK is node 4
    // (sign -1, driver -0.40) so its core start is +0.40; with LIMIT 0.3 a
    // goal of -0.3 is clamped to 0.10.
    bench.command("SPEED 0.5");
    assert(near(bench.jog.joint(1).start, 0.40F));
    const float lk_start = bench.jog.joint(1).start;
    assert(contains(bench.command("GOTO 1 -0.3"), "EV OK GOTO LK 0.10"));
    bench.run(0.2);
    assert(near(bench.jog.joint(1).target, lk_start - 0.5F * 0.2F, 0.011F));
    bench.run(1.0);
    assert(near(bench.jog.joint(1).target, lk_start - 0.3F, 1.0e-4F));
    bench.command("GOTO_START");
    bench.run(2.0);
    for (std::size_t i = 0; i < 4; ++i) {
      assert(near(bench.jog.joint(i).target, bench.jog.joint(i).start, 1.0e-4F));
    }
    // Gains are clamped.
    assert(contains(bench.command("GAINS 900 9"), "kp=40.00 kd=3.00"));
    assert(bench.jog.mode() == Mode::Active);
  }

  // Host silence -> FAULT and IDLE.
  {
    Bench bench;
    engage(bench);
    bench.run(0.45, false);
    assert(bench.jog.mode() == Mode::Active);
    bench.run(0.1, false);
    assert(bench.jog.mode() == Mode::Fault);
    char event[128];
    bool saw_fault = false;
    while (bench.jog.popEvent(event, sizeof(event))) {
      if (contains(event, "host timeout")) saw_fault = true;
    }
    assert(saw_fault);
    assert(bench.axis_state[1] == 1 && bench.axis_state[4] == 1);
    assert(contains(bench.command("ENABLE"), "CLEAR first"));
    assert(contains(bench.command("CLEAR"), "EV OK CLEAR"));
    assert(bench.jog.mode() == Mode::Disabled);
  }

  // Joint that does not follow -> tracking-error FAULT.
  {
    Bench bench;
    engage(bench);
    bench.follow = false;
    bench.command("JOG 0 1.0 0 0");
    bench.run(0.8);
    assert(bench.jog.mode() == Mode::Fault);
  }

  // Drivers that never reach CLOSED_LOOP -> FAULT after the grace period.
  {
    Bench bench;
    for (std::uint8_t node = 1U; node <= 4U; ++node) {
      bench.feedback(node);
      bench.heartbeat(node);
    }
    bench.command("ENABLE");
    bench.sink.frames.clear();  // Frames lost: no driver enters closed loop.
    bench.run(0.6);
    assert(bench.jog.mode() == Mode::Fault);
  }

  // Excess torque -> FAULT; ESTOP latches through DISABLE and ENABLE.
  {
    Bench bench;
    engage(bench);
    bench.torque = 7.0F;
    bench.run(0.05);
    assert(bench.jog.mode() == Mode::Fault);
    bench.torque = 0.0F;
    bench.command("CLEAR");
    bench.command("ESTOP");
    assert(bench.jog.mode() == Mode::Estop);
    assert(contains(bench.command("DISABLE"), "ESTOP latched"));
    assert(contains(bench.command("ENABLE"), "CLEAR first"));
  }

  // Parser robustness.
  {
    Bench bench;
    assert(contains(bench.command("JOG 1 2 3"), "EV ERR"));
    assert(contains(bench.command("JOG 1 2 3 4 5"), "EV ERR"));
    assert(contains(bench.command("GOTO 4 0.1"), "EV ERR"));
    assert(contains(bench.command("GOTO 1.5 0.1"), "EV ERR"));
    assert(contains(bench.command("FLY"), "unknown"));
    assert(bench.command("  hb  ").empty());
    assert(contains(bench.command("gains 10 0.5"), "kp=10.00 kd=0.50"));
  }

  // UART framer: garbage and a corrupted frame are skipped, then the next
  // valid Present Data response comes out whole.
  {
    legjog::DxlStreamParser parser;
    const auto good = frame::buildPresentResponse(1U, 512U, -35, 7);
    auto bad = good;
    bad.bytes[bad.len - 1U] ^= 0x5AU;
    std::vector<std::uint8_t> stream{0x00, 0x13, 0xFF, 0x42};
    stream.insert(stream.end(), bad.bytes.begin(), bad.bytes.begin() + bad.len);
    stream.insert(stream.end(), good.bytes.begin(), good.bytes.begin() + good.len);
    int frames = 0;
    frame::DxlFrame out{};
    for (const auto byte : stream) frames += parser.push(byte, out) ? 1 : 0;
    assert(frames == 1);
    const auto present = frame::parsePresentPayload(
        frame::parseDxlResponse(out.bytes.data(), out.len));
    assert(present.valid && present.position_raw == 512U && present.speed_raw == -35);
  }

  // Wheels while the legs stay DISABLED: coast + poll while idle; WHEEL does
  // Torque Enable, waits the 20 ms settle, then Moving Speed; release brakes
  // at speed 0 and then coasts.
  {
    Bench bench;
    bench.step();
    assert(!bench.wheel_torque[0] && !bench.wheel_torque[1]);
    assert(bench.jog.wheel(0).fb_valid && bench.jog.wheel(1).fb_valid);
    assert(bench.jog.wheel(1).pos_raw == 900U);

    bench.command("WHEEL 60 -60");
    bench.now += 10'000U;
    bench.jog.tick(bench.now);
    bool speed_before_settle = false;
    for (const auto& u : bench.sink.uart) {
      const auto write = frame::parseDxlWrite(u.frame);
      if (write.ok && write.address == frame::kDxlMovingSpeedAddr) speed_before_settle = true;
    }
    assert(!speed_before_settle);
    bench.reactAndAnswer();
    assert(bench.wheel_torque[0] && bench.wheel_torque[1] && bench.wheel_speed[0] == 0);
    for (int i = 0; i < 5; ++i) {
      bench.command("WHEEL 60 -60");
      bench.step();
    }
    assert(bench.wheel_speed[0] == kS * 60 && bench.wheel_speed[1] == -kS * 60);
    assert(bench.jog.wheel(0).speed_raw == 60 && bench.jog.wheel(1).speed_raw == -60);
    assert(bench.jog.mode() == Mode::Disabled);  // Legs untouched.

    // Clamp to the protocol range.
    bench.command("WHEEL 900 0");
    bench.step();
    assert(bench.wheel_speed[0] == kS * 400 && bench.wheel_speed[1] == 0);

    // Release: brake at 0, then coast after wheel_brake_us.
    bench.command("WHEEL 0 0");
    bench.step();
    assert(bench.wheel_torque[0] && bench.wheel_speed[0] == 0);
    for (int i = 0; i < 35; ++i) {
      bench.command("WHEEL 0 0");
      bench.step();
    }
    assert(!bench.wheel_torque[0] && !bench.wheel_torque[1]);
  }

  // Wheel lease: WHEEL not repeated -> stop even though HB keeps coming.
  {
    Bench bench;
    for (int i = 0; i < 5; ++i) {
      bench.command("WHEEL 80 80");
      bench.step();
    }
    assert(bench.wheel_speed[0] == kS * 80);
    bench.run(0.35);
    assert(bench.wheel_speed[0] == 0);
    bench.run(0.4);
    assert(!bench.wheel_torque[0] && !bench.wheel_torque[1]);
  }

  // ESTOP stops wheels at once and WHEEL is ignored until CLEAR.
  {
    Bench bench;
    for (int i = 0; i < 5; ++i) {
      bench.command("WHEEL 80 80");
      bench.step();
    }
    bench.command("ESTOP");
    bench.reactAndAnswer();
    assert(!bench.wheel_torque[0] && !bench.wheel_torque[1]);
    for (int i = 0; i < 5; ++i) {
      bench.command("WHEEL 80 80");
      bench.step();
    }
    assert(!bench.wheel_torque[0] && bench.jog.wheel(0).target_raw == 0);
    bench.command("CLEAR");
    for (int i = 0; i < 5; ++i) {
      bench.command("WHEEL 80 80");
      bench.step();
    }
    assert(bench.wheel_speed[0] == kS * 80);
  }

  // Wheel stops answering while running -> torque off and an event.
  {
    Bench bench;
    for (int i = 0; i < 5; ++i) {
      bench.command("WHEEL 50 50");
      bench.step();
    }
    bench.wheel_answer = false;
    for (int i = 0; i < 25; ++i) {
      bench.command("WHEEL 50 50");
      bench.step();
    }
    assert(!bench.wheel_torque[0] && !bench.wheel_torque[1]);
    char event[128];
    bool lost = false;
    while (bench.jog.popEvent(event, sizeof(event))) {
      if (contains(event, "LW feedback lost")) lost = true;
    }
    assert(lost);
    // Latched: holding the button does not re-enable it...
    bench.wheel_answer = true;
    for (int i = 0; i < 5; ++i) {
      bench.command("WHEEL 50 50");
      bench.step();
    }
    assert(!bench.wheel_torque[0] && bench.jog.wheel(0).fault);
    // ...releasing re-arms it.
    bench.command("WHEEL 0 0");
    bench.step();
    for (int i = 0; i < 5; ++i) {
      bench.command("WHEEL 50 50");
      bench.step();
    }
    assert(bench.wheel_speed[0] == kS * 50 && !bench.jog.wheel(0).fault);
  }

  // Leg FAULT also stops the wheels.
  {
    Bench bench;
    engage(bench);
    for (int i = 0; i < 5; ++i) {
      bench.command("WHEEL 50 50");
      bench.step();
    }
    assert(bench.wheel_torque[0]);
    bench.torque = 7.0F;
    bench.command("WHEEL 50 50");
    bench.step();
    bench.torque = 0.0F;
    bench.command("WHEEL 50 50");
    bench.step();
    assert(bench.jog.mode() == Mode::Fault);
    assert(!bench.wheel_torque[0] && !bench.wheel_torque[1]);
  }

  // Status line layout (parsed by tools/leg_jog_gui.py).
  {
    Bench bench;
    engage(bench);
    char status[512];
    bench.jog.formatStatus(bench.now, status, sizeof(status));
    int commas = 0;
    for (const char* p = status; *p; ++p) commas += *p == ',';
    assert(std::strncmp(status, "ST,ACTIVE,", 10) == 0);
    assert(commas == 5 + 4 * 8 + 2 * 6);
    std::printf("%s\n", status);
  }

  std::printf("leg_jog_tests: all checks passed\n");
  return 0;
}
