#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstddef>

// Pure, allocation-free pack/unpack for the two actuator wire protocols this
// robot uses. No I/O, no state, no MuJoCo/simulator types anywhere in this
// file - it is meant to be the SAME code compiled into the native sim's fake
// driver AND the eventual ESP32 firmware. If a "if (sim) ..." branch ever
// shows up here, the conformance value of the whole HAL layer is gone; see
// docs/MOTOR_HAL_PLAN.md section 5.5.
//
// Source of truth for every constant below:
//   docs/actuatal_driver/GDS68_DRIVER.md          (4x leg joints, CAN "MIT")
//   docs/actuatal_driver/WHEEL_MOTOR_PROTOCOL.md  (2x wheels, Dynamixel-like UART)
namespace robot_control::motor_frame {

// ---------------------------------------------------------------------------
// GDS68 (CAN, "CAN Simple" / MIT_Control, cmd_id 0x008) - 4 leg joints
// ---------------------------------------------------------------------------

// 11-bit CAN arbitration ID: (node_id << 5) | cmd_id.  GDS68_DRIVER.md §3.
constexpr std::uint16_t canId(std::uint8_t node_id, std::uint8_t cmd_id) {
  return static_cast<std::uint16_t>((static_cast<std::uint16_t>(node_id) << 5) |
                                     (cmd_id & 0x1FU));
}

constexpr std::uint8_t kSetControllerModeCmdId = 0x00B;
constexpr std::uint8_t kSetAxisStateCmdId = 0x007;
constexpr std::uint8_t kMitControlCmdId = 0x008;
constexpr std::uint8_t kSetLimitsCmdId = 0x00F;

constexpr std::int32_t kAxisStateIdle = 1;
constexpr std::int32_t kAxisStateClosedLoop = 8;
constexpr std::int32_t kControlModePosition = 3;
constexpr std::int32_t kInputModeMit = 9;

using MitBytes = std::array<std::uint8_t, 8>;

namespace detail {
inline void putI32Le(MitBytes& b, std::size_t offset, std::int32_t v) {
  const auto u = static_cast<std::uint32_t>(v);
  b[offset + 0] = static_cast<std::uint8_t>(u & 0xFFU);
  b[offset + 1] = static_cast<std::uint8_t>((u >> 8) & 0xFFU);
  b[offset + 2] = static_cast<std::uint8_t>((u >> 16) & 0xFFU);
  b[offset + 3] = static_cast<std::uint8_t>((u >> 24) & 0xFFU);
}
inline std::int32_t getI32Le(const MitBytes& b, std::size_t offset) {
  const std::uint32_t u = static_cast<std::uint32_t>(b[offset]) |
                           (static_cast<std::uint32_t>(b[offset + 1]) << 8) |
                           (static_cast<std::uint32_t>(b[offset + 2]) << 16) |
                           (static_cast<std::uint32_t>(b[offset + 3]) << 24);
  return static_cast<std::int32_t>(u);
}
}  // namespace detail

// Set_Controller_Mode (0x00B): two little-endian int32 params. Bring-up
// sequence, GDS68_DRIVER.md §4: sent once before the first CLOSED_LOOP entry
// to select MIT mode (control_mode=3, input_mode=9).
inline MitBytes packSetControllerMode(std::int32_t control_mode, std::int32_t input_mode) {
  MitBytes b{};
  detail::putI32Le(b, 0, control_mode);
  detail::putI32Le(b, 4, input_mode);
  return b;
}
struct SetControllerModeCommand {
  std::int32_t control_mode = 0;
  std::int32_t input_mode = 0;
};
inline SetControllerModeCommand unpackSetControllerMode(const MitBytes& b) {
  return SetControllerModeCommand{detail::getI32Le(b, 0), detail::getI32Le(b, 4)};
}

// Set_Axis_State (0x007): one little-endian int32 requested_state, GDS68_DRIVER.md §4.
inline MitBytes packSetAxisState(std::int32_t requested_state) {
  MitBytes b{};
  detail::putI32Le(b, 0, requested_state);
  return b;
}
inline std::int32_t unpackSetAxisState(const MitBytes& b) { return detail::getI32Le(b, 0); }

// MIT field ranges, GDS68_DRIVER.md §4. Feedback re-uses the SAME ranges for
// position/velocity as the host->motor frame (the doc says so explicitly for
// position, and states all 5 fields share one linear quantization formula).
// The feedback Torque field's numeric range is NOT stated explicitly in the
// doc; treating it as the same +/-50 N.m / 12-bit range as the host T_ff
// field is an inference from that "5 trường dùng chung 1 công thức" note,
// not a verbatim spec - flag this to whoever does hardware bring-up.
constexpr float kPosMinRad = -12.5F;
constexpr float kPosMaxRad = 12.5F;
constexpr float kVelMinRadS = -65.0F;
constexpr float kVelMaxRadS = 65.0F;
constexpr float kKpMin = 0.0F;
constexpr float kKpMax = 500.0F;
constexpr float kKdMin = 0.0F;
constexpr float kKdMax = 5.0F;
constexpr float kTffMinNm = -50.0F;
constexpr float kTffMaxNm = 50.0F;

namespace detail {
// raw = (x - xmin) * (2^bits - 1) / (xmax - xmin), x clamped first. The doc's
// own worked example truncates rather than rounds (e.g. 12.5*65535/25 =
// 32767.5 -> 32767, not 32768) - match that exactly, bit-exact beats
// mathematically-nicer rounding here.
inline std::uint32_t quantize(float x, float xmin, float xmax, unsigned bits) {
  x = std::isfinite(x) ? x : xmin;
  if (x < xmin) x = xmin;
  if (x > xmax) x = xmax;
  const float scale = static_cast<float>((1U << bits) - 1U) / (xmax - xmin);
  const float raw = (x - xmin) * scale;
  const auto max_raw = static_cast<float>((1U << bits) - 1U);
  return static_cast<std::uint32_t>(raw < 0.0F ? 0.0F : (raw > max_raw ? max_raw : raw));
}
inline float dequantize(std::uint32_t raw, float xmin, float xmax, unsigned bits) {
  const float scale = (xmax - xmin) / static_cast<float>((1U << bits) - 1U);
  return xmin + static_cast<float>(raw) * scale;
}
}  // namespace detail

// Host -> motor MIT_Control payload. GDS68_DRIVER.md §4 byte layout, verified
// bit-exact against the doc's worked example (node 1, all mid-range values).
inline MitBytes packMitCommand(float pos_rad, float vel_rad_s, float kp,
                                float kd, float tff_nm) {
  const std::uint32_t p = detail::quantize(pos_rad, kPosMinRad, kPosMaxRad, 16);
  const std::uint32_t v = detail::quantize(vel_rad_s, kVelMinRadS, kVelMaxRadS, 12);
  const std::uint32_t kp_i = detail::quantize(kp, kKpMin, kKpMax, 12);
  const std::uint32_t kd_i = detail::quantize(kd, kKdMin, kKdMax, 12);
  const std::uint32_t t = detail::quantize(tff_nm, kTffMinNm, kTffMaxNm, 12);
  MitBytes b{};
  b[0] = static_cast<std::uint8_t>(p >> 8);
  b[1] = static_cast<std::uint8_t>(p & 0xFFU);
  b[2] = static_cast<std::uint8_t>(v >> 4);
  b[3] = static_cast<std::uint8_t>(((v & 0xFU) << 4) | (kp_i >> 8));
  b[4] = static_cast<std::uint8_t>(kp_i & 0xFFU);
  b[5] = static_cast<std::uint8_t>(kd_i >> 4);
  b[6] = static_cast<std::uint8_t>(((kd_i & 0xFU) << 4) | (t >> 8));
  b[7] = static_cast<std::uint8_t>(t & 0xFFU);
  return b;
}

struct MitCommand {
  float pos_rad = 0.0F;
  float vel_rad_s = 0.0F;
  float kp = 0.0F;
  float kd = 0.0F;
  float tff_nm = 0.0F;
};

// Inverse of packMitCommand - what a driver's firmware would decode.
inline MitCommand unpackMitCommand(const MitBytes& b) {
  const std::uint32_t p = (static_cast<std::uint32_t>(b[0]) << 8) | b[1];
  const std::uint32_t v = (static_cast<std::uint32_t>(b[2]) << 4) | (b[3] >> 4);
  const std::uint32_t kp_i = (static_cast<std::uint32_t>(b[3] & 0x0FU) << 8) | b[4];
  const std::uint32_t kd_i = (static_cast<std::uint32_t>(b[5]) << 4) | (b[6] >> 4);
  const std::uint32_t t = (static_cast<std::uint32_t>(b[6] & 0x0FU) << 8) | b[7];
  return MitCommand{detail::dequantize(p, kPosMinRad, kPosMaxRad, 16),
                     detail::dequantize(v, kVelMinRadS, kVelMaxRadS, 12),
                     detail::dequantize(kp_i, kKpMin, kKpMax, 12),
                     detail::dequantize(kd_i, kKdMin, kKdMax, 12),
                     detail::dequantize(t, kTffMinNm, kTffMaxNm, 12)};
}

struct MitFeedback {
  std::uint8_t node_id = 0;
  float pos_rad = 0.0F;
  float vel_rad_s = 0.0F;
  float torque_nm = 0.0F;
};

// Motor -> host MIT_Control feedback. GDS68_DRIVER.md §4: node_id echo, then
// position/velocity/torque packed the same bit-packed way, bytes 0-5 only
// (bytes 6-7 of the 8-byte CAN frame are unused by this feedback layout).
inline MitFeedback packMitFeedback(std::uint8_t node_id, float pos_rad,
                                    float vel_rad_s, float torque_nm,
                                    MitBytes& out) {
  const std::uint32_t p = detail::quantize(pos_rad, kPosMinRad, kPosMaxRad, 16);
  const std::uint32_t v = detail::quantize(vel_rad_s, kVelMinRadS, kVelMaxRadS, 12);
  const std::uint32_t t = detail::quantize(torque_nm, kTffMinNm, kTffMaxNm, 12);
  out[0] = node_id;
  out[1] = static_cast<std::uint8_t>(p >> 8);
  out[2] = static_cast<std::uint8_t>(p & 0xFFU);
  out[3] = static_cast<std::uint8_t>(v >> 4);
  out[4] = static_cast<std::uint8_t>(((v & 0xFU) << 4) | (t >> 8));
  out[5] = static_cast<std::uint8_t>(t & 0xFFU);
  out[6] = 0;
  out[7] = 0;
  return MitFeedback{node_id, pos_rad, vel_rad_s, torque_nm};
}

inline MitFeedback unpackMitFeedback(const MitBytes& b) {
  const std::uint32_t p = (static_cast<std::uint32_t>(b[1]) << 8) | b[2];
  const std::uint32_t v = (static_cast<std::uint32_t>(b[3]) << 4) | (b[4] >> 4);
  const std::uint32_t t = (static_cast<std::uint32_t>(b[4] & 0x0FU) << 8) | b[5];
  MitFeedback fb;
  fb.node_id = b[0];
  fb.pos_rad = detail::dequantize(p, kPosMinRad, kPosMaxRad, 16);
  fb.vel_rad_s = detail::dequantize(v, kVelMinRadS, kVelMaxRadS, 12);
  fb.torque_nm = detail::dequantize(t, kTffMinNm, kTffMaxNm, 12);
  return fb;
}

// ---------------------------------------------------------------------------
// BL4820M12 (UART, Dynamixel-like packet) - 2 wheel joints
// ---------------------------------------------------------------------------

constexpr std::uint8_t kDxlTorqueEnableAddr = 0x18;
constexpr std::uint8_t kDxlMovingSpeedAddr = 0x20;
constexpr std::uint8_t kDxlPresentAddr = 0x24;
constexpr std::uint8_t kDxlPresentLen = 6;
constexpr std::uint8_t kDxlInsWrite = 0x03;
constexpr std::uint8_t kDxlInsRead = 0x02;
constexpr std::int16_t kWheelRawSpeedLimit = 400;  // +/-400 raw, WHEEL §8, 400 rpm rated under load

// raw = RPM (WHEEL §8: coincidence of 400==400, not a verbatim spec line -
// flagged in the doc as needing a real measurement before hardware bring-up).
inline std::int16_t wheelRadPerSecToRawSpeed(float rad_s) {
  if (!std::isfinite(rad_s)) return 0;
  float rpm = rad_s * 60.0F / (2.0F * 3.14159265358979323846F);
  if (rpm > static_cast<float>(kWheelRawSpeedLimit)) rpm = static_cast<float>(kWheelRawSpeedLimit);
  if (rpm < static_cast<float>(-kWheelRawSpeedLimit)) rpm = static_cast<float>(-kWheelRawSpeedLimit);
  return static_cast<std::int16_t>(rpm + (rpm >= 0.0F ? 0.5F : -0.5F));
}
inline float wheelRawSpeedToRadPerSec(std::int16_t raw) {
  return static_cast<float>(raw) * 2.0F * 3.14159265358979323846F / 60.0F;
}
// Single-turn position, NOT unwrapped (WHEEL §4: 0..1023 = 0..360 deg, one
// revolution). Callers doing odometry must unwrap statefully themselves, or
// (recommended, see docs/MOTOR_HAL_PLAN.md §5.2) use Speed as the primary
// velocity source instead, which is immune to wrap.
inline float wheelRawPositionToRad(std::uint16_t raw) {
  return static_cast<float>(raw & 0x03FFU) * 2.0F * 3.14159265358979323846F / 1024.0F;
}

using DxlBytes = std::array<std::uint8_t, 16>;  // fits the largest frame used (14 bytes)

struct DxlFrame {
  DxlBytes bytes{};
  std::uint8_t len = 0;  // bytes actually used, starting at bytes[0]
};

namespace detail {
// CHK = ~(ID + LEN + INS/ERR + PARAMS...) & 0xFF. WHEEL §3.
inline std::uint8_t dxlChecksum(std::uint8_t id, std::uint8_t len, std::uint8_t ins,
                                 const std::uint8_t* params, std::uint8_t n) {
  std::uint32_t sum = static_cast<std::uint32_t>(id) + len + ins;
  for (std::uint8_t i = 0; i < n; ++i) sum += params[i];
  return static_cast<std::uint8_t>(~sum);
}
inline DxlFrame buildFrame(std::uint8_t id, std::uint8_t ins, const std::uint8_t* params,
                            std::uint8_t n) {
  DxlFrame f;
  const std::uint8_t len = static_cast<std::uint8_t>(n + 2);  // ins + params + chk
  f.bytes[0] = 0xFF;
  f.bytes[1] = 0xFF;
  f.bytes[2] = id;
  f.bytes[3] = len;
  f.bytes[4] = ins;
  for (std::uint8_t i = 0; i < n; ++i) f.bytes[5 + i] = params[i];
  f.bytes[5 + n] = detail::dxlChecksum(id, len, ins, params, n);
  f.len = static_cast<std::uint8_t>(6 + n);
  return f;
}
}  // namespace detail

// WHEEL_MOTOR_PROTOCOL.md §5: "Torque Enable: FF FF 01 04 03 18 00 DF" etc.
inline DxlFrame buildTorqueEnable(std::uint8_t id, bool enable) {
  const std::uint8_t params[2] = {kDxlTorqueEnableAddr, static_cast<std::uint8_t>(enable ? 1 : 0)};
  return detail::buildFrame(id, kDxlInsWrite, params, 2);
}
inline DxlFrame buildMovingSpeed(std::uint8_t id, float rad_s) {
  const std::int16_t raw = wheelRadPerSecToRawSpeed(rad_s);
  const auto u = static_cast<std::uint16_t>(raw);  // 2's complement LE, WHEEL §5 example
  const std::uint8_t params[3] = {kDxlMovingSpeedAddr, static_cast<std::uint8_t>(u & 0xFFU),
                                   static_cast<std::uint8_t>(u >> 8)};
  return detail::buildFrame(id, kDxlInsWrite, params, 3);
}
inline DxlFrame buildReadPresent(std::uint8_t id) {
  const std::uint8_t params[2] = {kDxlPresentAddr, kDxlPresentLen};
  return detail::buildFrame(id, kDxlInsRead, params, 2);
}

struct DxlParsed {
  bool ok = false;
  std::uint8_t id = 0;
  std::uint8_t err = 0;
  std::array<std::uint8_t, kDxlPresentLen> payload{};
  std::uint8_t payload_len = 0;
};

// Parses a motor->host response, INCLUDING the firmware's quirk of echoing
// [LEN, INS, ADDR] of the request before the real data (WHEEL §3). Handles
// both a write ack (no real payload) and the 6-byte present-data read.
inline DxlParsed parseDxlResponse(const std::uint8_t* bytes, std::size_t n) {
  DxlParsed out;
  if (n < 8 || bytes[0] != 0xFF || bytes[1] != 0xFF) return out;
  const std::uint8_t id = bytes[2];
  const std::uint8_t len = bytes[3];
  const std::uint8_t err = bytes[4];
  if (static_cast<std::size_t>(len) + 4 > n) return out;
  const std::uint8_t* params = bytes + 5;
  const std::uint8_t param_n = static_cast<std::uint8_t>(len - 2);  // exclude ins/err slot + chk
  const std::uint8_t chk_index = static_cast<std::uint8_t>(4 + len - 1);
  const std::uint8_t expect_chk = detail::dxlChecksum(id, len, err, params, param_n);
  if (bytes[chk_index] != expect_chk) return out;
  // Quirk: params[0..2] are the echoed [LEN, INS, ADDR] of the request, not
  // real data. Real data (if any) starts right after them.
  if (param_n < 3) return out;
  const std::uint8_t real_n = static_cast<std::uint8_t>(param_n - 3);
  if (real_n > kDxlPresentLen) return out;
  out.ok = true;
  out.id = id;
  out.err = err;
  out.payload_len = real_n;
  for (std::uint8_t i = 0; i < real_n; ++i) out.payload[i] = params[3 + i];
  return out;
}

struct PresentData {
  bool valid = false;
  std::uint16_t position_raw = 0;
  std::int16_t speed_raw = 0;
  std::int16_t load_raw = 0;
};

// WHEEL §4: Position (u16 LE), Speed (s16 LE), Load (s16 LE), in that order.
inline PresentData parsePresentPayload(const DxlParsed& parsed) {
  PresentData out;
  if (!parsed.ok || parsed.payload_len != kDxlPresentLen) return out;
  const auto& p = parsed.payload;
  out.position_raw = static_cast<std::uint16_t>(p[0] | (static_cast<std::uint16_t>(p[1]) << 8));
  out.speed_raw = static_cast<std::int16_t>(p[2] | (static_cast<std::uint16_t>(p[3]) << 8));
  out.load_raw = static_cast<std::int16_t>(p[4] | (static_cast<std::uint16_t>(p[5]) << 8));
  out.valid = true;
  return out;
}

// Builds the motor->host response for a present-data read, including the
// quirk echo, for use by a fake driver / bench simulator.
inline DxlFrame buildPresentResponse(std::uint8_t id, std::uint16_t position_raw,
                                      std::int16_t speed_raw, std::int16_t load_raw) {
  const auto speed_u = static_cast<std::uint16_t>(speed_raw);
  const auto load_u = static_cast<std::uint16_t>(load_raw);
  const std::uint8_t params[9] = {
      kDxlPresentLen, kDxlInsRead, kDxlPresentAddr,  // echoed [LEN, INS, ADDR] quirk
      static_cast<std::uint8_t>(position_raw & 0xFFU),
      static_cast<std::uint8_t>(position_raw >> 8),
      static_cast<std::uint8_t>(speed_u & 0xFFU),
      static_cast<std::uint8_t>(speed_u >> 8),
      static_cast<std::uint8_t>(load_u & 0xFFU),
      static_cast<std::uint8_t>(load_u >> 8)};
  return detail::buildFrame(id, 0 /*err*/, params, 9);
}

// Builds the motor->host ack for a write instruction (Torque Enable /
// Moving Speed): same quirk-echo shape, zero real payload bytes.
inline DxlFrame buildWriteAck(std::uint8_t id, std::uint8_t written_len, std::uint8_t addr) {
  const std::uint8_t params[3] = {written_len, kDxlInsWrite, addr};
  return detail::buildFrame(id, 0 /*err*/, params, 3);
}

}  // namespace robot_control::motor_frame
