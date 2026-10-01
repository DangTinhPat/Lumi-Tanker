// v3: leg lock frames, start sequencing and closed-loop two-wheel balance.

#include <cassert>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "robot_control/GeneratedGeometry.hpp"
#include "robot_hal/MotorFrameCodec.hpp"
#include "tap_fw/v3/LegLock.hpp"
#include "tap_fw/FastFormat.hpp"
#include "tap_fw/v3/V3Controller.hpp"

using namespace robot_control::motor_frame;
using namespace robot_control::motor_transport;
using tap_fw::v3::LegLock;
using tap_fw::v3::V3Controller;

namespace {

struct FakeTransport final : Transport {
  std::vector<CanFrame> can;
  bool sendCan(std::uint16_t id, const MitBytes& data) noexcept override {
    CanFrame f;
    f.id = id;
    f.dlc = 8U;
    f.data = data;
    can.push_back(f);
    return true;
  }
  bool receiveCan(CanFrame&) noexcept override { return false; }
  bool sendUart(std::uint8_t, const DxlFrame&) noexcept override { return true; }
  bool receiveUart(std::uint8_t, DxlFrame&) noexcept override { return false; }
  bool isOpen() const noexcept override { return true; }
  void close() noexcept override {}
};

bool near(float a, float b, float eps = 1.0e-4F) { return std::abs(a - b) <= eps; }
unsigned cmdOf(const CanFrame& f) { return f.id & 0x1FU; }

struct Result {
  bool balanced = false;
  float trim = 0.0F;
  float speed = 0.0F;
};

// Wheel joint speed servo (tau) on a rigid leg: axle v = R (wheel + pitch rate).
Result run(V3Controller::Algo algo, float eq, float release, float tau,
           float length = 0.23F) {
  auto config = tap_fw::v3::defaultV3Config();
  config.algo = algo;
  V3Controller controller(config);
  const float g = 9.80665F, dt = 0.002F, radius = robot_control::geometry::wheel_radius_m;
  std::mt19937 rng(11);
  std::normal_distribution<float> n_pitch(0.0F, 0.004F), n_rate(0.0F, 0.01F), n_w(0.0F, 0.3F);
  float th = release, w = 0.0F, wheel = 0.0F, wheel_pos = 0.0F;
  V3Controller::Input in{};
  V3Controller::Command cmd{};
  cmd.start = true;
  Result r;
  for (int i = 0; i < 12500; ++i) {  // 25 s
    in.now_us = static_cast<std::uint64_t>(i) * 2000U;
    in.pitch_rad = th + n_pitch(rng);
    in.pitch_rate_rad_s = w + n_rate(rng);
    in.wheel_rad_s = {wheel + n_w(rng), wheel + n_w(rng)};
    in.wheel_pos_rad = {wheel_pos, wheel_pos};
    in.legs_ready = i > 600;  // lock ramp done after 1.2 s
    cmd.go = controller.state() == V3Controller::State::Ready;
    const auto out = controller.update(in, cmd);
    cmd.start = false;
    if (out.state == V3Controller::State::Fault) return r;
    if (controller.state() != V3Controller::State::Balancing) continue;  // operator holds it
    const float wcmd = 0.5F * (out.wheel_cmd_rad_s[0] + out.wheel_cmd_rad_s[1]);
    float wdot = (wcmd - wheel) / tau;
    const float sn = std::sin(th - eq), cs = std::cos(th);
    float thdd = (g * sn - radius * cs * wdot) / (length + radius * cs);
    float a = radius * (wdot + thdd);
    if (std::abs(a) > 1.5F) {
      a = std::copysign(1.5F, a);
      thdd = (g * sn - a * cs) / length;
      wdot = a / radius - thdd;
    }
    wheel += wdot * dt;
    wheel_pos += wheel * dt;
    w += thdd * dt;
    th += w * dt;
  }
  r.balanced = controller.state() == V3Controller::State::Balancing;
  r.trim = controller.balanceRef();
  r.speed = controller.groundVelocity();
  return r;
}

}  // namespace

int main() {
  // Position-mode lock: set-point starts at the measured pose, closes the
  // loop without a jump, ramps to exactly 0 (the power-up pose).
  {
    FakeTransport bus;
    LegLock lock;
    const std::array<float, 4> start{0.8F, -0.4F, 0.2F, 0.0F};
    assert(lock.engage(start, 1'000'000U, bus));
    assert(bus.can.size() == 12U);
    for (int i = 0; i < 4; ++i) {
      assert(cmdOf(bus.can[i]) == kSetControllerModeCmdId);
      const auto mode = unpackSetControllerMode(bus.can[i].data);
      assert(mode.control_mode == kControlModePosition &&
             mode.input_mode == kInputModePassthrough);
      assert(cmdOf(bus.can[4 + i]) == kSetInputPosCmdId);
      assert(near(unpackSetInputPosTurns(bus.can[4 + i].data), start[i]));
      assert(cmdOf(bus.can[8 + i]) == kSetAxisStateCmdId);
      assert(unpackSetAxisState(bus.can[8 + i].data) == kAxisStateClosedLoop);
    }
    assert(!lock.atZero(1'500'000U));
    const float mid = lock.targetTurns(0, 1'500'000U);
    assert(mid < 0.8F && mid > 0.0F);
    // Resend cadence.
    bus.can.clear();
    assert(lock.update(1'003'000U, bus) && bus.can.empty());  // resend every 4 ms
    assert(lock.update(1'004'000U, bus) && bus.can.size() == 4U);
    assert(lock.atZero(2'000'000U));
    bus.can.clear();
    assert(lock.update(2'100'000U, bus));
    for (const auto& f : bus.can) assert(near(unpackSetInputPosTurns(f.data), 0.0F));
    bus.can.clear();
    assert(lock.release(bus) && bus.can.size() == 4U);
    for (const auto& f : bus.can) {
      assert(cmdOf(f) == kSetAxisStateCmdId && unpackSetAxisState(f.data) == kAxisStateIdle);
    }
    assert(!lock.engaged());
  }

  // v3.1 height offsets: core hip/knee offsets become driver turns per leg
  // (8:1, driver sign); the engage ramp still starts at the measured pose.
  {
    FakeTransport bus;
    LegLock lock;
    lock.setOffsets(0.2F, -0.4F);
    assert(lock.engage({0.0F, 0.0F, 0.0F, 0.0F}, 0U, bus));
    const auto legs = tap_fw::board::MotorIds{}.leg_can;
    for (std::size_t i = 0; i < 4U; ++i) {
      const bool knee = legs[i].joint_index == 1U || legs[i].joint_index == 4U;
      const float expect = (knee ? -0.4F : 0.2F) * legs[i].driver_sign * 8.0F /
          6.28318530718F;
      assert(near(lock.targetTurns(i, 2'000'000U), expect, 1.0e-5F));
    }
  }

  // Roll side offsets: left joints (LH, LK) get side 0, right (RH, RK) side 1,
  // added to the common offsets.
  {
    FakeTransport bus;
    LegLock lock;
    lock.setOffsets(0.2F, -0.4F);
    lock.setSideOffsets({0.01F, -0.01F}, {0.03F, -0.03F});
    assert(lock.engage({0.0F, 0.0F, 0.0F, 0.0F}, 0U, bus));
    const auto legs = tap_fw::board::MotorIds{}.leg_can;
    for (std::size_t i = 0; i < 4U; ++i) {
      const bool knee = legs[i].joint_index == 1U || legs[i].joint_index == 4U;
      const bool left = legs[i].joint_index < 3U;
      const float core = knee ? -0.4F + (left ? 0.03F : -0.03F) : 0.2F + (left ? 0.01F : -0.01F);
      const float expect = core * legs[i].driver_sign * 8.0F / 6.28318530718F;
      assert(near(lock.targetTurns(i, 2'000'000U), expect, 1.0e-5F));
    }
  }

  // Roll side rates go out as Set_Input_Pos vel_ff (milli-turns/s, driver
  // frame through the 8:1 gear); none without them.
  {
    FakeTransport bus;
    LegLock::Config cfg{};
    cfg.ramp_s = 0.0F;
    LegLock lock(cfg);
    lock.setSideOffsets({0.0F, 0.0F}, {0.0F, 0.0F}, {0.1F, -0.1F}, {0.5F, -0.5F});
    assert(lock.engage({0.0F, 0.0F, 0.0F, 0.0F}, 0U, bus));
    bus.can.clear();
    assert(lock.update(1'000'000U, bus) && bus.can.size() == 4U);
    const auto legs = tap_fw::board::MotorIds{}.leg_can;
    for (std::size_t i = 0; i < 4U; ++i) {
      const bool knee = legs[i].joint_index == 1U || legs[i].joint_index == 4U;
      const bool left = legs[i].joint_index < 3U;
      const float rate = (knee ? 0.5F : 0.1F) * (left ? 1.0F : -1.0F);
      const float expect = rate * legs[i].driver_sign * 8.0F / 6.28318530718F * 1000.0F;
      const auto& d = bus.can[i].data;
      const auto vff = static_cast<std::int16_t>(static_cast<std::uint16_t>(d[4] | (d[5] << 8)));
      assert(std::abs(vff - expect) <= 1.0F);
    }
  }

  // FastFormat matches printf's "%+.Nf" / "%.Nf" on telemetry-sized values.
  {
    const float vals[] = {0.0F, -0.0F, 1.0F, -1.0F, 0.12345F, -0.12345F, 0.0005F, -0.0004F, 0.00049F,
                          2.5F, 3.5F, -2.5F, 123.456F, -987.654F, 188.6F, 1e-7F, -1e-7F, 9.99996F,
                          0.31F, -20.0F, 1234567.0F};
    for (float v : vals) {
      for (int d = 0; d <= 4; ++d) {
        for (int plus = 0; plus < 2; ++plus) {
          char ref[64], buf[64];
          char fmt[8] = {'%', plus ? '+' : '.', plus ? '.' : static_cast<char>('0' + d),
                         plus ? static_cast<char>('0' + d) : 'f', plus ? 'f' : '\0', '\0', '\0', '\0'};
          std::snprintf(ref, sizeof ref, fmt, static_cast<double>(v));
          tap_fw::LineBuf lb(buf, sizeof buf);
          lb.fix(v, d, plus != 0);
          if (std::strcmp(ref, lb.c_str()) != 0) {
            std::printf("FastFormat mismatch v=%.9g d=%d plus=%d: printf '%s' fast '%s'\n",
                        static_cast<double>(v), d, plus, ref, lb.c_str());
            std::fflush(stdout);
            assert(false);
          }
        }
      }
    }
    char buf[32];
    tap_fw::LineBuf lb(buf, sizeof buf);
    lb.str("t=").u32(4294967295U).chr(' ').i32(-2147483647);
    assert(std::strcmp(lb.c_str(), "t=4294967295 -2147483647") == 0);
    tap_fw::LineBuf hx(buf, sizeof buf);
    hx.hex(0x40U).chr(' ').hex(0U).chr(' ').hex(0xDEADBEEFU);
    assert(std::strcmp(hx.c_str(), "40 0 deadbeef") == 0);
  }

  // MIT fallback: joint radians, fixed Kp/Kd.
  {
    FakeTransport bus;
    LegLock::Config cfg{};
    cfg.mode = LegLock::Mode::Mit;
    cfg.ramp_s = 0.0F;
    LegLock lock(cfg);
    assert(lock.engage({0.8F, 0.0F, 0.0F, 0.0F}, 0U, bus));
    const auto mit = unpackMitCommand(bus.can[4].data);
    assert(cmdOf(bus.can[4]) == kMitControlCmdId);
    assert(near(mit.pos_rad, 0.0F, 1.0e-3F));  // ramp 0: straight to zero
    assert(near(mit.kp, 15.0F, 0.2F) && near(mit.kd, 0.5F, 0.01F));
  }

  // Sequencing: start -> Locking (engage edge) -> Balancing once the legs are
  // at zero and settled; disable -> Idle; sensors missing -> refused.
  {
    auto seq_cfg = tap_fw::v3::defaultV3Config();
    seq_cfg.engage.immediate = false;  // exercise the Ready wait
    V3Controller c(seq_cfg);
    V3Controller::Input in{};
    V3Controller::Command cmd{};
    in.sensors_ok = false;
    cmd.start = true;
    auto out = c.update(in, cmd);
    assert(out.state == V3Controller::State::Fault && out.fault != nullptr);
    in.sensors_ok = true;
    in.now_us = 10'000U;
    out = c.update(in, cmd);
    assert(out.state == V3Controller::State::Locking && out.engage_legs && out.legs_locked);
    assert(!out.wheels_enabled);
    cmd.start = false;
    in.legs_ready = true;
    in.pitch_rad = 0.05F;
    in.now_us = 600'000U;
    out = c.update(in, cmd);
    assert(out.state == V3Controller::State::Locking);  // still settling
    in.now_us = 1'300'000U;
    out = c.update(in, cmd);
    assert(out.state == V3Controller::State::Ready);
    in.now_us = 1'301'000U;
    out = c.update(in, cmd);
    // Ready: wheels enabled but held still (no balance loop), even while the
    // robot is carried at a large tilt.
    assert(out.state == V3Controller::State::Ready && out.wheels_enabled);
    assert(out.wheel_cmd_rad_s[0] == 0.0F && out.wheel_cmd_rad_s[1] == 0.0F);
    in.pitch_rad = 0.7F;
    in.now_us = 1'302'000U;
    out = c.update(in, cmd);
    assert(out.state == V3Controller::State::Ready);
    in.pitch_rad = 0.05F;
    cmd.go = true;
    in.now_us = 1'304'000U;
    out = c.update(in, cmd);
    cmd.go = false;
    assert(out.state == V3Controller::State::Balancing && out.wheels_enabled);
    assert(near(c.balancer().balanceTrim(), 0.05F));  // seeded at engage
    in.pitch_rad = 0.7F;
    in.now_us = 1'306'000U;
    out = c.update(in, cmd);
    // A fault stops the wheels but the legs stay locked (they are structure).
    assert(out.state == V3Controller::State::Fault && !out.wheels_enabled && out.legs_locked);
    in.now_us = 1'400'000U;
    out = c.update(in, cmd);
    assert(out.state == V3Controller::State::Fault && out.legs_locked);
    // START again from the fault: no re-engage (still holding), locking again.
    in.pitch_rad = 0.0F;
    cmd.start = true;
    out = c.update(in, cmd);
    assert(out.state == V3Controller::State::Locking && !out.engage_legs && out.legs_locked);
    cmd.start = false;
    // Only DISABLE releases the legs.
    cmd.disable = true;
    out = c.update(in, cmd);
    assert(out.state == V3Controller::State::Idle && !out.legs_locked);
  }

  // A measured balance angle seeds the reference instead of the release pitch.
  {
    V3Controller c(tap_fw::v3::defaultV3Config());
    c.setBalanceAngle(true, 0.09F);
    V3Controller::Input in{};
    in.legs_ready = true;
    in.pitch_rad = 0.02F;
    V3Controller::Command cmd{};
    cmd.start = true;
    (void)c.update(in, cmd);
    cmd.start = false;
    in.now_us = 1'300'000U;
    (void)c.update(in, cmd);
    cmd.go = true;
    in.now_us += 2000U;
    const auto out = c.update(in, cmd);
    cmd.go = false;
    assert(out.state == V3Controller::State::Balancing);
    assert(near(c.balanceRef(), 0.09F, 1.0e-3F));
    assert(near(c.balancer().balanceTrim(), 0.09F));
    // Released 4 deg behind the balance angle: wheels drive backward to
    // bring the COM back over the axle (nose-up -> backward).
    in.now_us += 2000U;
    const auto next = c.update(in, cmd);
    assert(next.wheel_cmd_rad_s[0] < 0.0F && next.wheel_cmd_rad_s[1] < 0.0F);
  }

  // Auto engage: still on the stand does nothing; carry (move), hold still
  // at the standing point for 0.7 s (arms, seed = held pitch), release (tip).
  {
    auto auto_cfg = tap_fw::v3::defaultV3Config();
    auto_cfg.engage.immediate = false;
    V3Controller c(auto_cfg);
    V3Controller::Input in{};
    in.legs_ready = true;
    in.pitch_rad = -0.02F;
    V3Controller::Command cmd{};
    cmd.start = true;
    (void)c.update(in, cmd);
    cmd.start = false;
    std::uint64_t t = 1'300'000U;
    auto step = [&](float pitch, float rate, int n) {
      V3Controller::Output o{};
      for (int i = 0; i < n; ++i) {
        t += 2000U;
        in.now_us = t;
        in.pitch_rad = pitch;
        in.pitch_rate_rad_s = rate;
        o = c.update(in, cmd);
      }
      return o;
    };
    auto o = step(-0.02F, 0.0F, 1000);  // 2 s still on the stand
    assert(o.state == V3Controller::State::Ready && c.readyPhase() == 0);
    o = step(0.1F, 0.8F, 100);          // carried to the floor
    assert(o.state == V3Controller::State::Ready && c.readyPhase() == 1);
    o = step(0.09F, 0.02F, 200);        // held 0.4 s: not armed yet
    assert(c.readyPhase() == 1);
    o = step(0.09F, 0.2F, 5);           // a wobble restarts the still timer
    o = step(0.09F, 0.02F, 400);        // held 0.8 s: armed
    assert(o.state == V3Controller::State::Ready && c.readyPhase() == 2);
    o = step(0.095F, 0.05F, 5);         // small drift below the tip thresholds
    assert(o.state == V3Controller::State::Ready);
    o = step(0.12F, 0.2F, 1);           // released: tips
    assert(o.state == V3Controller::State::Balancing);
    assert(near(c.balanceRef(), 0.09F, 1.0e-3F));  // seeded at the held pitch
  }

  // Default flow: engage right after the lock. Held on the stand (body
  // cannot rotate, wheels spin in the air and follow the command): no
  // runaway, the loop re-zeroes; a tilt still makes the wheels answer.
  {
    V3Controller c(tap_fw::v3::defaultV3Config());
    V3Controller::Input in{};
    in.legs_ready = true;
    in.pitch_rad = -0.02F;
    V3Controller::Command cmd{};
    cmd.start = true;
    (void)c.update(in, cmd);
    cmd.start = false;
    std::uint64_t t = 1'300'000U;
    float wheel = 0.0F, pos = 0.0F, max_wheel = 0.0F;
    V3Controller::Output o{};
    auto run_held = [&](float pitch, int n) {
      for (int i = 0; i < n; ++i) {
        t += 2000U;
        in.now_us = t;
        in.pitch_rad = pitch;
        in.pitch_rate_rad_s = 0.0F;
        in.accel_x_m_s2 = -9.80665F * std::sin(pitch);  // held still: gravity only
        in.accel_z_m_s2 = 9.80665F * std::cos(pitch);
        in.wheel_rad_s = {wheel, wheel};
        in.wheel_pos_rad = {pos, pos};
        o = c.update(in, cmd);
        const float target = 0.5F * (o.wheel_cmd_rad_s[0] + o.wheel_cmd_rad_s[1]);
        wheel += (target - wheel) * 0.002F / 0.1F;  // free wheel, 0.1 s servo
        pos += wheel * 0.002F;
        max_wheel = std::max(max_wheel, std::abs(wheel));
        assert(o.state != V3Controller::State::Fault);
      }
    };
    run_held(-0.021F, 2);
    assert(o.state == V3Controller::State::Balancing);  // engaged without GO
    run_held(-0.021F, 5000);   // 10 s on the stand, 1 mrad off
    assert(max_wheel < 15.0F);  // never reaches the runaway guard
    max_wheel = 0.0F;
    run_held(0.08F, 200);      // operator tilts it nose-down on the stand
    assert(max_wheel > 2.0F);   // wheels answer the tilt (forward)
    // Held with a small uncorrected gyro bias (as on the robot): the wheel
    // command must not creep.
    {
      float max_creep = 0.0F;
      for (int k = 0; k < 15000; ++k) {  // 30 s
        t += 2000U;
        in.now_us = t;
        in.pitch_rad = -0.02F;
        in.pitch_rate_rad_s = 0.01F;
        in.accel_x_m_s2 = -9.80665F * std::sin(-0.02F);
        in.accel_z_m_s2 = 9.80665F * std::cos(-0.02F);
        in.wheel_rad_s = {wheel, wheel};
        in.wheel_pos_rad = {pos, pos};
        o = c.update(in, cmd);
        const float target = 0.5F * (o.wheel_cmd_rad_s[0] + o.wheel_cmd_rad_s[1]);
        wheel += (target - wheel) * 0.002F / 0.1F;
        pos += wheel * 0.002F;
        if (k > 5000) max_creep = std::max(max_creep, std::abs(wheel));
      }
      std::printf("held with gyro bias 0.01 rad/s: max wheel %.2f rad/s\n", max_creep);
      assert(max_creep < 1.0F);  // below the ~1 rad/s wheel deadband
    }
    // Held there for 10 s: short bursts are re-zeroed, never a fault, and
    // the wheels are mostly still.
    int spinning = 0;
    for (int k = 0; k < 5000; ++k) {
      run_held(0.08F, 1);
      if (std::abs(wheel) > 5.0F) ++spinning;
    }
    std::printf("held on stand: resets=%d spinning %.0f %% of 10 s\n", c.heldResets(),
                100.0 * spinning / 5000.0);
    assert(c.heldResets() > 0);
    assert(spinning < 5000 / 4);
    // v3.1: lower the legs while held on the stand. The balance feed-forward
    // must not wind the wheels up (robot log 2026-09-27: -19 rad/s).
    V3Controller::Command hcmd{};
    hcmd.height_m = 0.160F;
    float max_after = 0.0F;
    for (int k = 0; k < 3000; ++k) {
      t += 2000U;
      in.now_us = t;
      in.pitch_rad = 0.08F;
      in.pitch_rate_rad_s = 0.0F;
      in.accel_x_m_s2 = -9.80665F * std::sin(0.08F);
      in.accel_z_m_s2 = 9.80665F * std::cos(0.08F);
      in.wheel_rad_s = {wheel, wheel};
      in.wheel_pos_rad = {pos, pos};
      o = c.update(in, hcmd);
      const float target = 0.5F * (o.wheel_cmd_rad_s[0] + o.wheel_cmd_rad_s[1]);
      wheel += (target - wheel) * 0.002F / 0.1F;
      pos += wheel * 0.002F;
      if (k > 1500) max_after = std::max(max_after, std::abs(wheel));
      assert(o.state != V3Controller::State::Fault);
    }
    assert(std::abs(c.legPose().axle_below_hip_m - 0.160F) < 0.001F);  // legs moved
    std::printf("held on stand, lowered to 160 mm: max wheel after %.2f rad/s\n", max_after);
    assert(max_after < 2.0F);
    // Balance angle (at the power-up height) + height feed-forward = held pitch.
    assert(near(c.balanceRef(), 0.08F, 0.01F));  // FF off by default
  }

  // Closed loop: unknown balance point, hand error at release. The cascade
  // PID (default; gains from a sweep, 9/9 over kp 30-45) must also hold with
  // 80/120 ms wheel lag; the LQR keeps its 40 ms case.
  for (auto algo : {V3Controller::Algo::Pid, V3Controller::Algo::Lqr}) {
  for (float tau : {0.04F, 0.08F, 0.12F}) {
  if (algo == V3Controller::Algo::Lqr && tau > 0.05F) continue;
  for (float eq_deg : {0.0F, 5.0F, -6.0F}) {
    for (float hand_deg : {0.0F, 2.0F, -2.0F}) {
      const float deg = 3.14159265F / 180.0F;
      const auto r = run(algo, eq_deg * deg, (eq_deg + hand_deg) * deg, tau);
      std::printf("%s tau %.0f ms eq %+.0f release %+.0f: balanced=%d ref=%+.2f deg v=%+.3f\n",
                  algo == V3Controller::Algo::Pid ? "PID" : "LQR", tau * 1000.0F, eq_deg,
                  hand_deg, r.balanced, r.trim / deg, r.speed);
      assert(r.balanced);
      assert(std::abs(r.trim - eq_deg * deg) < 1.0F * deg);
      assert(std::abs(r.speed) < 0.05F);
    }
  }
  }
  }
  std::puts("v3_tests: all checks passed");
  return 0;
}
