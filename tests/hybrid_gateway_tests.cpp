#include "gateway/HybridGatewayCore.hpp"
#include "gateway/ImuBiasStore.hpp"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

using namespace gateway;

// Test double only. Never linked into the ESP32 firmware.
struct TestMotors final : MotorBackend {
  bool available = true, fail_apply = false, bad_feedback = false, off_confirmed = true;
  unsigned applications = 0, disables = 0;
  HcrTarget last_target{};
  bool ready() const noexcept override { return available; }
  MotorFeedback feedback(std::uint64_t now) noexcept override {
    MotorFeedback f{}; f.stamp_us = now;
    f.q = {2.35706715F,2.35706715F,0,0,0,0};
    f.encoders_valid = f.homed = available;
    if (bad_feedback) f.q[0] = std::numeric_limits<float>::quiet_NaN();
    return f;
  }
  void disable() noexcept override { ++disables; }
  bool disabled() const noexcept override { return off_confirmed; }
  ApplyResult apply(const HcrTarget& t, std::uint32_t seq, std::uint64_t,
                    std::uint64_t, MotorApplied& out) noexcept override {
    ++applications;
    last_target = t;
    out.epoch=t.epoch; out.seq=seq;
    std::copy(t.target, t.target+6, out.target.begin());
    return fail_apply?ApplyResult::Fault:ApplyResult::Applied;
  }
};

HcrFrame decode(const WirePacket& p) {
  HcrParser parser{}; HcrFrame f{}; unsigned count = 0;
  for (unsigned i = 0; i < p.size; ++i) count += hcr_feed(&parser, p.bytes[i], &f);
  assert(count == 1 && parser.errors == 0); return f;
}
HcrFrame hello(unsigned epoch = 7, unsigned seq = 1) {
  HcrFrame f{}; f.type = HCR_HELLO; f.size = 12; f.seq = seq; f.stamp_us = seq;
  hcr_put32(f.payload, epoch); hcr_put16(f.payload+4, 200); hcr_put16(f.payload+6, 100);
  hcr_put32(f.payload+8, HCR_SCHEMA); return f;
}
HcrFrame statusFrame(unsigned mode, unsigned seq = 2, unsigned epoch = 7) {
  HcrFrame f{}; f.type = HCR_STATUS; f.size = 16; f.seq = seq; f.stamp_us = seq;
  hcr_put32(f.payload, epoch); hcr_put32(f.payload+4, mode); return f;
}
HcrFrame target(unsigned source, unsigned seq = 3, unsigned profile = HCR_POLICY_PD) {
  HcrFrame f{}; f.type = HCR_TARGET; f.size = 52; f.seq = seq; f.stamp_us = seq;
  hcr_put32(f.payload, 7); hcr_put32(f.payload+4, source);
  hcr_put16(f.payload+8, profile); hcr_put16(f.payload+10, profile ? 3 : 0);
  const float values[10] = {2.4F,2.3F,.4F,.6F,10,-10,1,2,3,4};
  for (unsigned i = 0; i < 10; ++i) hcr_putfloat(f.payload+12+4*i, values[i]);
  return f;
}
void sensors(GatewayCore& core, std::uint64_t now) {
  ImuObservation imu{}; imu.stamp_us = now; imu.valid = true; imu.accel = {0,0,9.80665F};
  core.setImu(imu); core.tick(now);
}
unsigned sentTelemetry(GatewayCore& core, std::uint64_t now) {
  const auto frame = decode(core.telemetry(now));
  core.telemetrySent(hcr_u32(frame.payload), frame.seq, frame.stamp_us); return frame.seq;
}
unsigned arm(GatewayCore& core, std::uint64_t now) {
  assert(decode(core.receive(hello(), now)).type == HCR_HELLO_ACK);
  core.receive(statusFrame(2), now); sensors(core, now);
  return sentTelemetry(core, now);
}

void tests() {
  { // OTA interrupts active control; neither RESET nor a fresh LF epoch can unlock it.
    TestMotors m; GatewayCore c(m); const auto ref = arm(c,100000);
    c.receive(target(ref),100001); sensors(c,100500);
    assert(m.applications == 1);
    m.off_confirmed = false;
    c.enterMaintenance();
    assert(c.maintenance() && !c.maintenanceReady());
    assert(!c.diagnostics(100501).armed && !c.diagnostics(100501).enabled);
    assert(!c.receive(target(ref,10),100502).size);
    assert(c.maintenance());
    assert(!c.receive(hello(99,10),100504).size);
    assert(!c.receive(target(ref,11),100505).size);
    assert(!c.telemetry(100506).size && !c.requestSession(100506).size);
    sensors(c,101000);
    assert(m.applications == 1 && !c.maintenanceReady());
    m.off_confirmed = true;
    assert(c.maintenanceReady());
    c.estop(); c.stop(); sensors(c,2000000);
    assert(c.maintenanceReady() && m.applications == 1);
    m.off_confirmed = false; assert(!c.maintenanceReady());
    GatewayCore rebooted(m); assert(!rebooted.maintenance() && !rebooted.diagnostics(1).armed);
  }
  { // Complete downlink boundary: 4 angle targets, 2 wheel rates and 4 rate FF.
    TestMotors m; GatewayCore c(m); const auto ref = arm(c,100000);
    const auto wire = target(ref,3,HCR_RECOVERY);
    c.receive(wire,100001);
    assert(m.applications == 0); // receiving a packet is not applying it
    sensors(c,100500);
    assert(m.applications == 1 && m.last_target.profile == HCR_RECOVERY);
    assert(m.last_target.epoch == 7 && m.last_target.source_seq == ref && m.last_target.flags == 3);
    const float expected[10] = {2.4F,2.3F,.4F,.6F,10,-10,1,2,3,4};
    for (unsigned i = 0; i < 6; ++i) assert(m.last_target.target[i] == expected[i]);
    for (unsigned i = 0; i < 4; ++i) assert(m.last_target.feedforward[i] == expected[6+i]);
    const auto telemetry = decode(c.telemetry(100500));
    assert(hcr_u32(telemetry.payload+8) == wire.seq);
    assert(hcr_u32(telemetry.payload+4) & HCR_TARGET_VALID);
    assert(!(hcr_u32(telemetry.payload+4) & HCR_TORQUE_VALID));
    for (unsigned i = 0; i < 6; ++i)
      assert(hcr_float(telemetry.payload+12+4*(HCR_APPLIED_TARGET+i)) == expected[i]);
  }
  { // IMU calibration, full roll/pitch gravity and axis signs.
    GravityEstimator e; ImuObservation out;
    for (unsigned i = 1; i <= 250; ++i) out = e.update({0,0,9.80665F},{0,0,0},i*2000);
    assert(out.valid && std::abs(out.gravity[2]+1) < 1e-5F);
    for (unsigned i = 251; i <= 350; ++i) out = e.update({0,0,0},{1,0,0},i*2000);
    assert(out.gravity[1] < -.19F && out.gravity[2] < -.97F);
    out = e.update({0,0,0},{0,0,0},800000); assert(!out.valid); // gap needs reseed
    e.reset();
    for (unsigned i = 1; i <= 250; ++i) out = e.update({0,0,9.80665F},{0,0,0},i*2000);
    for (unsigned i = 251; i <= 350; ++i) out = e.update({0,0,0},{0,1,0},i*2000);
    assert(out.gravity[0] > .19F);
    e.reset();
    for (unsigned i = 1; i <= 250; ++i)
      out = e.update({-4.903325F,0,8.492809F},{.01F,.02F,.03F},i*2000);
    assert(out.valid && std::abs(out.gravity[0]-.5F)<1e-4F && std::abs(out.gyro[1])<1e-5F);
    out = e.update({NAN,0,0},{0,0,0},502000); assert(!out.valid);
  }
  { // Regression: measured stationary bias cannot seed the conservative boot
    // gate, but an explicitly requested stable calibration can estimate it.
    GravityEstimator e; ImuObservation out;
    const std::array<float,3> a{.28105F,.10703F,9.82711F}, w{.00809F,-.22010F,.01451F};
    for(unsigned i=1;i<=2000;++i) out=e.update(a,w,i*2000);
    assert(!out.valid);
    e.calibrateStationary();
    for(unsigned i=1;i<=499;++i) { out=e.update(a,w,i*2000);assert(!out.valid); }
    out=e.update(a,w,1000000);assert(out.valid && !e.manualCalibration());
    for(unsigned j=0;j<3;++j) assert(std::abs(out.gyro[j])<.00001F);
    assert(std::abs(out.gravity[2]+.99953F)<.0001F);
    out=e.update(a,w,1100000);assert(!out.valid && !e.manualCalibration());
    for(unsigned i=1;i<=1000;++i) out=e.update(a,w,1100000+i*2000);
    // After the gap the stored bias reseeds automatically, but only at the
    // narrow automatic gate: the wide manual gate is not authorized again.
    assert(out.valid && !e.manualCalibration());
    e.calibrateStationary();
    for(unsigned i=1;i<=1000;++i) out=e.update(a,{0,(i%2?-.22F:-.18F),0},i*2000);
    assert(!out.valid); // gyro motion/noise
    e.calibrateStationary();
    for(unsigned i=1;i<=1000;++i) out=e.update({i%2?.3F:0,0,9.81F},w,i*2000);
    assert(!out.valid); // tilt/acceleration changes despite near-1g norm
    e.calibrateStationary();
    for(unsigned i=1;i<=1000;++i) out=e.update(a,{0,.4F,0},i*2000);
    assert(!out.valid); // excessive bias or rotation
    e.calibrateStationary();
    for(unsigned i=1;i<=500;++i) out=e.update(a,w,i*500);
    assert(!out.valid); // calls alone do not count as one second
    out=e.update({NAN,0,0},w,251000);assert(!out.valid && !e.manualCalibration());
  }
  { // Stored bias: boot seeds on its own while the gyro sits at the stored value.
    const std::array<float,3> a{.28105F,.10703F,9.82711F}, w{.00809F,-.22010F,.01451F};
    GravityEstimator e; ImuObservation out;
    assert(!e.hasStoredBias());
    e.restoreBias(w);
    assert(e.hasStoredBias());
    for(unsigned i=1;i<=249;++i) { out=e.update(a,w,i*2000); assert(!out.valid); }
    out=e.update(a,w,250*2000);
    assert(out.valid && !e.takeManualCalibration()); // automatic, nothing to persist
    for(unsigned j=0;j<3;++j) assert(std::abs(out.gyro[j])<.00001F);
    assert(std::abs(out.gravity[2]+.99953F)<.0001F);
    // Drift inside the tolerance is re-measured, not taken from flash.
    GravityEstimator d; d.restoreBias(w);
    const std::array<float,3> drifted{w[0],w[1]+.03F,w[2]};
    for(unsigned i=1;i<=250;++i) out=d.update(a,drifted,i*2000);
    assert(out.valid && std::abs(out.gyro[1])<.00001F && std::abs(d.bias()[1]-drifted[1])<.00001F);
    // A gap resets the attitude but keeps the stored bias.
    out=d.update(a,drifted,2000000); assert(!out.valid && d.hasStoredBias());
    for(unsigned i=1;i<=250;++i) out=d.update(a,drifted,2000000+i*2000);
    assert(out.valid);
  }
  { // Stored bias does not make a moving or drifted robot look still.
    const std::array<float,3> a{.28105F,.10703F,9.82711F}, w{.00809F,-.22010F,.01451F};
    ImuObservation out;
    GravityEstimator turning; turning.restoreBias(w);  // yaw about body Z at .1 rad/s
    for(unsigned i=1;i<=1000;++i) out=turning.update(a,{w[0],w[1],w[2]+.1F},i*2000);
    assert(!out.valid);
    GravityEstimator drifted; drifted.restoreBias(w);  // bias moved past the tolerance
    for(unsigned i=1;i<=1000;++i) out=drifted.update(a,{w[0],w[1]+.08F,w[2]},i*2000);
    assert(!out.valid);
    GravityEstimator zero; zero.restoreBias(w);        // wrong board: gyro reads near zero
    for(unsigned i=1;i<=1000;++i) out=zero.update(a,{0,0,0},i*2000);
    assert(!out.valid);
    GravityEstimator shaken; shaken.restoreBias(w);
    for(unsigned i=1;i<=1000;++i) out=shaken.update({i%2?.3F:0,0,9.81F},w,i*2000);
    assert(!out.valid);                                 // tilt changing during the window
  }
  { // Manual CAL_IMU reports completion once and replaces an old stored bias.
    const std::array<float,3> a{.28105F,.10703F,9.82711F}, w{.00809F,-.22010F,.01451F};
    GravityEstimator e; ImuObservation out;
    e.restoreBias({.02F,.10F,0});                       // stale value from another unit
    for(unsigned i=1;i<=1000;++i) out=e.update(a,w,i*2000);
    assert(!out.valid && !e.takeManualCalibration());   // automatic path refuses it
    e.calibrateStationary();
    for(unsigned i=1;i<=499;++i) { out=e.update(a,w,i*2000); assert(!out.valid); }
    out=e.update(a,w,1000000); assert(out.valid);
    assert(e.takeManualCalibration() && !e.takeManualCalibration());
    for(unsigned j=0;j<3;++j) assert(std::abs(e.bias()[j]-w[j])<.00001F);
    e.reset();                                           // later reboot-like reseed
    for(unsigned i=1;i<=250;++i) out=e.update(a,w,i*2000);
    assert(out.valid);
    GravityEstimator fresh;                              // never calibrated, never stored
    for(unsigned i=1;i<=1000;++i) out=fresh.update(a,w,i*2000);
    assert(!out.valid);
  }
  { // Flash record: a damaged or foreign record is never used.
    const std::array<float,3> bias{.00798F,-.24153F,.00793F};
    auto r = ImuBiasRecord::make(bias);
    assert(r.valid() && r.value()[1] == bias[1]);
    ImuBiasRecord bad = r; bad.bias[1] = -.2F; assert(!bad.valid());           // CRC
    bad = r; bad.magic ^= 1; assert(!bad.valid());
    bad = r; bad.version = 2; assert(!bad.valid());
    assert(!ImuBiasRecord{}.valid());                                          // erased/blank
    assert(!ImuBiasRecord::make({NAN,0,0}).valid());
    assert(!ImuBiasRecord::make({0,.6F,0}).valid());                           // beyond CAL_IMU range
    assert(ImuBiasRecord::make({0,-.35F,0}).valid());
  }
  { // No motors: honest validity, telemetry still works, no ARM bypass.
    UnavailableMotors m; GatewayCore c(m);
    c.receive(hello(), 100000); c.receive(statusFrame(1), 100000); sensors(c, 100000);
    const auto t = decode(c.telemetry(100000));
    assert(hcr_u32(t.payload+4) == HCR_IMU_OK);
    c.receive(target(t.seq), 100001);
    assert(!c.diagnostics(100001).enabled);
  }
  { // Actual application is acknowledged one service tick after reception.
    TestMotors m; GatewayCore c(m); const auto ref = arm(c, 100000);
    c.receive(target(ref),100001);
    assert(c.diagnostics(100001).applied_seq == 0);
    sensors(c,100500);
    auto t = decode(c.telemetry(100500));
    assert(hcr_u32(t.payload+8) == 3 && (hcr_u32(t.payload+4)&HCR_TARGET_VALID));
    assert(std::abs(hcr_float(t.payload+12+23*4)-2.4F)<1e-6F); // no [-1,1] clipping
    assert(m.applications == 1);
    c.receive(target(ref),119999); // duplicate must not renew TTL
    sensors(c,120001);
    assert(c.diagnostics(120001).fault == LocalFault::TargetTimeout);
    c.receive(target(ref,4),120002); assert(!c.diagnostics(120002).enabled);
    c.receive(statusFrame(1,5),120002);
    assert(c.diagnostics(120002).fault == LocalFault::TargetTimeout);
    c.receive(hello(8,1),120003);
    assert(c.diagnostics(120003).fault == LocalFault::None && !c.diagnostics(120003).armed);
  }
  { // A real history entry is required (no arithmetic guess from last seq).
    TestMotors m; GatewayCore c(m); const auto ref = arm(c,100000);
    c.receive(target(ref+1),100001); assert(!c.diagnostics(100001).enabled); // ARM sequence
    c.receive(target(ref,4),131000); assert(!c.diagnostics(131000).enabled); // old source
    auto f = target(ref,5); hcr_putfloat(f.payload+12,NAN);
    c.receive(f,100005); assert(!c.diagnostics(100005).enabled);
    f = target(ref,6); hcr_put32(f.payload,8);
    c.receive(f,100006); assert(!c.diagnostics(100006).enabled);
  }
  { // Queued frame age counts toward TTL; it cannot restart an expired motor.
    TestMotors m; GatewayCore c(m); const auto ref = arm(c,100000);
    c.receive(target(ref),105000,100001); sensors(c,105001);
    c.receive(target(ref,4),120001,119999);
    assert(c.diagnostics(120001).fault == LocalFault::TargetTimeout);
  }
  { // Same-epoch HELLO cannot clear a fault; new epoch disables and requires ARM.
    TestMotors m; GatewayCore c(m); const auto ref = arm(c,100000);
    c.receive(target(ref),100001); sensors(c,120001);
    assert(c.receive(hello(7,8),120002).size);
    assert(c.diagnostics(120002).fault == LocalFault::TargetTimeout);
    c.receive(hello(9,1),120003); assert(!c.diagnostics(120003).armed);
  }
  { // Disarmed cable loss also detects a lost link and requests a new session.
    UnavailableMotors m; GatewayCore c(m);
    c.receive(hello(),100000); sensors(c,600000);
    assert(!c.diagnostics(600000).connected && c.requestSession(600000).size);
    c.receive(hello(8,1),600001);
    assert(c.diagnostics(600001).connected && !c.diagnostics(600001).armed);
  }
  { // Driver rejection / stale or nonfinite feedback never produces an ACK.
    TestMotors m; GatewayCore c(m); const auto ref = arm(c,100000);
    m.fail_apply = true; c.receive(target(ref),100001); sensors(c,100500);
    assert(c.diagnostics(100500).fault == LocalFault::Driver);
    c.receive(hello(8),100501);
    assert(c.diagnostics(100501).fault == LocalFault::Driver);
    assert(c.diagnostics(100500).applied_seq == 0);
    TestMotors m2; GatewayCore c2(m2); const auto ref2=arm(c2,100000);
    c2.receive(target(ref2),100001); // sensor failure must be tested while armed
    m2.bad_feedback = true; sensors(c2,100500);
    assert(c2.diagnostics(100500).fault == LocalFault::Sensors);
    auto t = decode(c2.telemetry(100500));
    for (unsigned i = 0; i < 29; ++i) assert(std::isfinite(hcr_float(t.payload+12+4*i)));
  }
  { // E-stop survives until an explicit local clear; a fresh LF epoch alone cannot arm.
    TestMotors m; GatewayCore c(m); const auto ref = arm(c,100000);
    c.receive(target(ref),100001); c.estop(); sensors(c,100500);
    assert(m.applications == 0 && (c.diagnostics(100500).telemetry_flags&HCR_ESTOP));
    c.receive(hello(8),100501);
    assert(c.diagnostics(100501).fault == LocalFault::Estop);
    assert(c.clearLocalFault()); c.receive(hello(9),100503);
    assert(c.diagnostics(100503).fault == LocalFault::None);
    c.estop(); sensors(c,600504); c.receive(hello(10),600505);
    assert(c.diagnostics(600505).fault == LocalFault::Estop); // link loss isn't operator RESET
  }
  { // Timestamp rollback and seq wrap across all LF frame types.
    TestMotors m; GatewayCore c(m); c.receive(hello(7,0xfffffffeU),100000);
    auto s = statusFrame(2,0xffffffffU); s.stamp_us = 5000000000ULL; c.receive(s,100000);
    sensors(c,100000); auto ref = sentTelemetry(c,100000);
    auto t = target(ref,0); t.stamp_us = 5000000001ULL; c.receive(t,100001); sensors(c,100500);
    assert(m.applications == 1);
    t.seq = 1; t.stamp_us = 1; c.receive(t,119999); sensors(c,120001);
    assert(c.diagnostics(120001).fault == LocalFault::TargetTimeout);
  }
  { // Byte stream fragmentation, corruption and resynchronization.
    std::uint8_t bytes[64], p[12]{};
    const auto size = hcr_encode(bytes,sizeof bytes,HCR_HELLO,1,1,p,12);
    HcrParser parser{}; HcrFrame f{}; unsigned good = 0;
    const auto crc = bytes[size-1]; bytes[size-1] ^= 0x55;
    for (unsigned i = 0; i < size; ++i) good += hcr_feed(&parser,bytes[i],&f);
    bytes[size-1] = crc;
    for (unsigned i = 0; i < size; ++i) good += hcr_feed(&parser,bytes[i],&f);
    assert(good == 1 && parser.errors);
    assert(hcr_newer(0,0xffffffffU) && !hcr_newer(1,1));
    assert((coreToHcr({0,1,2,3,4,5}) == std::array<float,6>{0,3,1,4,2,5}));
  }
  std::puts("PASS: attitude, mapping, missing-driver gate, applied ACK, TTL, replay, stale source, epoch/reset, estop, driver failure, finite data, CRC/resync");
}

// Host-only peer for exercising the exact LF Python Core against this C++ core.
void peer(bool unavailable) {
  TestMotors m; m.available = !unavailable; GatewayCore core(m); HcrParser parser{};
  std::string line;
  while (std::getline(std::cin,line)) {
    std::istringstream in(line); char op; std::uint64_t now; in >> op >> now;
    sensors(core,now); WirePacket reply{};
    if (op == 'F') {
      std::string hex; in >> hex;
      for (std::size_t i = 0; i+1 < hex.size(); i+=2) {
        const auto byte = std::stoul(hex.substr(i,2),nullptr,16); HcrFrame f{};
        if (hcr_feed(&parser,static_cast<std::uint8_t>(byte),&f)) reply = core.receive(f,now);
      }
    } else if (op == 'T') {
      reply = core.telemetry(now);
      if (reply.size) { const auto f = decode(reply); core.telemetrySent(hcr_u32(f.payload),f.seq,f.stamp_us); }
    } else if (op == 'C') {
      unsigned code; in >> code;
      const auto mode=code==1?2U:1U;
      reply=core.receive(statusFrame(mode,2),now);
    } else if (op == 'E') core.estop();
    if (!reply.size) std::cout << '-';
    else for (unsigned i = 0; i < reply.size; ++i) {
      constexpr char hex[] = "0123456789abcdef";
      std::cout << hex[reply.bytes[i]>>4] << hex[reply.bytes[i]&15];
    }
    const auto d = core.diagnostics(now);
    std::cout << ' ' << d.armed << ' ' << d.enabled << ' ' << static_cast<unsigned>(d.fault)
              << ' ' << d.applied_seq << '\n' << std::flush;
  }
}

int main(int argc, char** argv) {
  if (argc > 1 && !std::strcmp(argv[1],"--peer")) peer(argc > 2);
  else tests();
}
