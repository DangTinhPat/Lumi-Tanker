#include <cmath>
#include <cstdint>
#include <iostream>
#include "robot_control/MotorFrameCodec.hpp"

using namespace robot_control::motor_frame;

namespace {
int checks = 0;
int failures = 0;
void check(bool ok, const char* msg) {
  ++checks;
  if (!ok) { ++failures; std::cerr << "FAIL " << msg << '\n'; }
}
bool eqBytes(const MitBytes& b, std::initializer_list<int> expect) {
  if (expect.size() != b.size()) return false;
  std::size_t i = 0;
  for (int v : expect) { if (b[i++] != static_cast<std::uint8_t>(v)) return false; }
  return true;
}
bool eqFrame(const DxlFrame& f, std::initializer_list<int> expect) {
  if (f.len != expect.size()) return false;
  std::size_t i = 0;
  for (int v : expect) { if (f.bytes[i++] != static_cast<std::uint8_t>(v)) return false; }
  return true;
}
}  // namespace

int main() {
  // --- GDS68_DRIVER.md §4 "Ví dụ số cụ thể": node 1, all mid-range values.
  check(canId(1, kMitControlCmdId) == 0x028, "MIT can id = 0x028");
  const MitBytes mit = packMitCommand(0.0F, 0.0F, 250.0F, 2.5F, 0.0F);
  check(eqBytes(mit, {0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 0x7F, 0xF7, 0xFF}),
        "MIT command bit-packing matches doc worked example");

  // Round-trip through unpack should recover the same values within one LSB.
  const auto decoded = unpackMitCommand(mit);
  check(std::abs(decoded.pos_rad - 0.0F) < 1e-3F, "MIT unpack position round-trips");
  check(std::abs(decoded.kp - 250.0F) < 0.2F, "MIT unpack Kp round-trips");
  check(std::abs(decoded.kd - 2.5F) < 0.01F, "MIT unpack Kd round-trips");

  // --- GDS68_DRIVER.md §4 "Quickstart": 3-frame bring-up sequence.
  check(canId(1, kSetControllerModeCmdId) == 0x02B, "Set_Controller_Mode can id = 0x02B");
  check(canId(1, kSetAxisStateCmdId) == 0x027, "Set_Axis_State can id = 0x027");
  check(eqBytes(packSetControllerMode(kControlModePosition, kInputModeMit),
                {0x03, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00}),
        "Set_Controller_Mode(3,9) matches doc quickstart bytes");
  check(eqBytes(packSetAxisState(kAxisStateClosedLoop),
                {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}),
        "Set_Axis_State(8) matches doc quickstart bytes");
  {
    const auto scm = unpackSetControllerMode(packSetControllerMode(3, 9));
    check(scm.control_mode == 3 && scm.input_mode == 9, "Set_Controller_Mode round-trips");
    check(unpackSetAxisState(packSetAxisState(8)) == 8, "Set_Axis_State round-trips");
  }

  // --- MIT feedback: pack then unpack should recover input within one LSB,
  // and a hand-built frame at the same mid-range point should equal the
  // host->motor bytes for the shared position/velocity fields (feedback
  // reuses those two bytes-for-value layouts, GDS68_DRIVER.md §4).
  {
    MitBytes fb{};
    packMitFeedback(1, 0.0F, 0.0F, 0.0F, fb);
    check(fb[0] == 1, "MIT feedback echoes node_id in byte 0");
    const auto round = unpackMitFeedback(fb);
    check(std::abs(round.pos_rad) < 1e-3F && std::abs(round.vel_rad_s) < 0.05F &&
              std::abs(round.torque_nm) < 0.05F,
          "MIT feedback round-trips near zero");
    packMitFeedback(3, 6.0F, -12.5F, 1.5F, fb);
    const auto round2 = unpackMitFeedback(fb);
    check(round2.node_id == 3, "MIT feedback node_id round-trips");
    check(std::abs(round2.pos_rad - 6.0F) < 1e-3F, "MIT feedback position round-trips");
    check(std::abs(round2.vel_rad_s - (-12.5F)) < 0.05F, "MIT feedback velocity round-trips");
    check(std::abs(round2.torque_nm - 1.5F) < 0.05F, "MIT feedback torque round-trips");
  }

  // --- WHEEL_MOTOR_PROTOCOL.md §5: "Ví dụ khung (checksum đã verify)".
  const float rpm200 = 200.0F * 2.0F * 3.14159265358979323846F / 60.0F;
  check(eqFrame(buildMovingSpeed(1, rpm200), {0xFF, 0xFF, 0x01, 0x05, 0x03, 0x20, 0xC8, 0x00, 0x0E}),
        "Moving Speed +200 matches doc frame");
  check(eqFrame(buildMovingSpeed(1, -rpm200), {0xFF, 0xFF, 0x01, 0x05, 0x03, 0x20, 0x38, 0xFF, 0x9F}),
        "Moving Speed -200 matches doc frame (signed LE)");
  check(eqFrame(buildMovingSpeed(1, 0.0F), {0xFF, 0xFF, 0x01, 0x05, 0x03, 0x20, 0x00, 0x00, 0xD6}),
        "Moving Speed 0 matches doc frame");
  check(eqFrame(buildTorqueEnable(1, false), {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x18, 0x00, 0xDF}),
        "Torque Disable matches doc frame");
  check(eqFrame(buildReadPresent(1), {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x24, 0x06, 0xCE}),
        "Read Present matches doc frame");

  // --- Wheel raw<->rad conversions (WHEEL §8): raw==RPM, clamp +-400.
  check(wheelRadPerSecToRawSpeed(rpm200) == 200, "200 rpm converts to raw 200");
  check(wheelRadPerSecToRawSpeed(-rpm200) == -200, "-200 rpm converts to raw -200");
  check(wheelRadPerSecToRawSpeed(1000.0F) == kWheelRawSpeedLimit, "overspeed clamps to +400");
  check(wheelRadPerSecToRawSpeed(-1000.0F) == -kWheelRawSpeedLimit, "overspeed clamps to -400");
  {
    const float back = wheelRawSpeedToRadPerSec(wheelRadPerSecToRawSpeed(rpm200));
    check(std::abs(back - rpm200) < 0.02F, "wheel speed round-trips through raw");
  }
  check(std::abs(wheelRawPositionToRad(0) - 0.0F) < 1e-6F, "wheel position raw 0 = 0 rad");
  check(std::abs(wheelRawPositionToRad(1023) - (2.0F * 3.14159265F * 1023.0F / 1024.0F)) < 1e-3F,
        "wheel position raw 1023 approaches 2*pi");
  check(std::abs(wheelRawPositionToRad(1024) - 0.0F) < 1e-6F, "wheel position wraps at 1024 (single-turn)");
  check(wheelRadToRawPosition(0.0F) == 0, "wheel position 0 rad encodes as raw 0");
  check(wheelRadToRawPosition(2.0F * 3.14159265358979323846F) == 0,
        "wheel position wraps at one revolution when encoding");

  // --- Response parsing, including the quirk 3-byte echo (WHEEL §3).
  {
    const DxlFrame present = buildPresentResponse(1, 512, -150, 7);
    const auto parsed = parseDxlResponse(present.bytes.data(), present.len);
    check(parsed.ok && parsed.id == 1 && parsed.err == 0, "present response parses and is error-free");
    check(parsed.payload_len == kDxlPresentLen, "present response strips the 3-byte quirk echo");
    const auto data = parsePresentPayload(parsed);
    check(data.valid && data.position_raw == 512, "present position round-trips");
    check(data.speed_raw == -150, "present speed round-trips (signed)");
    check(data.load_raw == 7, "present load round-trips");
  }
  {
    const DxlFrame ack = buildWriteAck(1, 3, kDxlMovingSpeedAddr);
    const auto parsed = parseDxlResponse(ack.bytes.data(), ack.len);
    check(parsed.ok && parsed.payload_len == 0, "write ack parses with zero real payload");
  }
  {
    const auto write = parseDxlWrite(buildMovingSpeed(1, rpm200));
    check(write.ok && write.id == 1 && write.address == kDxlMovingSpeedAddr &&
              write.data_len == 2 && write.data[0] == 0xC8 && write.data[1] == 0x00,
          "Moving Speed command parses back at the wire boundary");
    DxlFrame corrupted = buildMovingSpeed(1, rpm200);
    corrupted.bytes[corrupted.len - 1] ^= 0x01U;
    check(!parseDxlWrite(corrupted).ok, "corrupted write checksum is rejected");
  }
  {
    // Corrupted checksum must be rejected, not silently accepted.
    DxlFrame present = buildPresentResponse(1, 1, 1, 1);
    present.bytes[present.len - 1] ^= 0xFFU;
    const auto parsed = parseDxlResponse(present.bytes.data(), present.len);
    check(!parsed.ok, "corrupted checksum is rejected");
  }

  std::cout << checks - failures << '/' << checks << " motor frame codec checks passed\n";
  return failures ? 1 : 0;
}
