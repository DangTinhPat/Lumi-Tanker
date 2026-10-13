#pragma once

// Bench jog logic (DRAFT). Drives the four GDS68 leg joints together with
// low-gain MIT position hold/jog, and the two BL4012 wheels with hold-to-run
// Moving Speed, for operator bring-up on a stand. It is a separate bench tool,
// not the balance controller: no IMU, no controller core. Pure C++ so it can
// be host-tested; main.cpp owns the I/O.
//
// Coordinates: every position/velocity/torque here is in the robot core frame
// (driver value = core value * driver_sign, MotorMapping.hpp), so "+" on both
// hips is the same mechanical direction on both legs. Positions are relative
// to each driver's power-on pose - GDS68 has no absolute output angle - so
// travel limits are taken around the pose captured at ENABLE.

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "robot_hal/MotorFrameCodec.hpp"
#include "robot_hal/MotorMapping.hpp"

namespace legjog {

namespace frame = robot_control::motor_frame;
namespace mapping = robot_control::motor_mapping;

inline constexpr std::size_t kJointCount = 4U;

enum class Mode : std::uint8_t { Disabled, Engaging, Active, Fault, Estop };

inline const char* toString(Mode mode) noexcept {
  switch (mode) {
    case Mode::Disabled: return "DISABLED";
    case Mode::Engaging: return "ENGAGING";
    case Mode::Active: return "ACTIVE";
    case Mode::Fault: return "FAULT";
    case Mode::Estop: return "ESTOP";
  }
  return "?";
}

class TxSink {
 public:
  virtual ~TxSink() = default;
  virtual bool send(std::uint16_t id, const frame::MitBytes& data) = 0;
  // bus 0 = UART1 (right wheel, id 1), bus 1 = UART2 (left wheel, id 2) -
  // two separate half-duplex single-wire buses, one motor each (see
  // main.cpp's HalfDuplexBus).
  virtual bool sendUart(std::uint8_t bus, const frame::DxlFrame& frame) = 0;
};

struct Config {
  std::uint64_t tick_us = 10'000U;             // 100 Hz command stream.
  std::uint64_t idle_resend_us = 50'000U;      // IDLE repeat while not engaged.
  std::uint64_t host_timeout_us = 500'000U;    // PC must send a line this often.
  std::uint64_t feedback_timeout_us = 100'000U;
  std::uint64_t heartbeat_timeout_us = 350'000U;  // Driver heartbeat is 100 ms.
  std::uint64_t closed_loop_grace_us = 400'000U;
  std::uint64_t gain_ramp_us = 500'000U;
  float max_tracking_error_rad = 0.6F;
  float max_torque_nm = 6.0F;
  float kp_max = 40.0F;
  float kd_max = 3.0F;
  float speed_max_rad_s = 1.5F;
  float travel_max_rad = 3.0F;
  float driver_pos_limit_rad = 12.3F;          // Inside the +/-12.5 MIT range.
  // Wheels (WHEEL_MOTOR_PROTOCOL.md). Speeds are protocol raw units; the
  // raw <-> RPM scale is still unverified. Raised to the Kpower BL4012's
  // no-load speed (2026-10-12, user request) - above 400 (rated speed) the
  // motor is no longer in its continuous-duty envelope, so sustained high
  // speed under load risks overheating/overcurrent; this is an operator
  // tradeoff, not a hardware limit fix.
  std::int16_t wheel_raw_max = 950;
  std::uint64_t wheel_lease_us = 300'000U;     // WHEEL must be repeated.
  std::uint64_t wheel_settle_us = 20'000U;     // Torque Enable -> Moving Speed.
  std::uint64_t wheel_brake_us = 300'000U;     // Speed 0 before coasting.
  std::uint64_t wheel_disable_resend_us = 50'000U;
  std::uint64_t wheel_feedback_timeout_us = 150'000U;
  // 2026-10-13: reg 0x11 (Velocity Mode, used by WHOLD to switch to
  // Position mode) persists to FLASH on this driver, unlike Torque
  // Enable/Moving Speed (RAM-only) - measured needing much more than
  // wheel_settle_us to actually take effect on one specific motor unit
  // (left wheel): it kept obeying its last Moving Speed command the whole
  // time WHOLD showed it "holding" (state 3), because the mode switch
  // hadn't really landed yet when we started sending Goal Position writes
  // that a motor still in Speed mode just ignores.
  std::uint64_t mode_switch_settle_us = 150'000U;
};

struct Settings {
  float kp = 8.0F;      // TEST1 slow-spin defaults.
  float kd = 0.6F;
  float speed_rad_s = 0.3F;  // GOTO ramp speed.
  float travel_rad = 1.2F;   // Allowed travel around the ENABLE pose.
};

struct Joint {
  const char* name = "";
  std::uint8_t node = 0U;
  float sign = 1.0F;
  // Latest MIT feedback, core frame.
  float q = 0.0F;
  float dq = 0.0F;
  float tau = 0.0F;
  std::uint64_t fb_us = 0U;
  bool fb_valid = false;
  std::uint32_t mit_ok = 0U;
  std::uint32_t mit_bad_echo = 0U;
  // Latest ODrive heartbeat.
  std::uint32_t axis_error = 0U;
  std::uint8_t axis_state = 0U;
  std::uint64_t hb_us = 0U;
  bool hb_valid = false;
  // Command state, core frame.
  float start = 0.0F;
  float target = 0.0F;
  float goal = 0.0F;
  bool has_goal = false;
  float jog_rad_s = 0.0F;
};

inline constexpr std::size_t kWheelCount = 2U;

// Bench-rig-only wiring: both wheels (Kpower BL4012, same Dynamixel-like
// protocol). 2026-10-13: reverted from a single shared half-duplex bus
// (both wheels in parallel on one GPIO, fragile under load - both wheels
// losing feedback together during a stall, not recoverable without
// widening margins indefinitely) to 2 SEPARATE half-duplex single-wire
// buses, one motor each (main.cpp's HalfDuplexBus) - this PCB's "UART1"/
// "UART2" connectors each carry one combined TX/RX wire per wheel, not
// true 2-pin full-duplex (a same-day first attempt assumed full-duplex;
// scanning IDs on one wheel's "RX" pin got an identical reply for every
// ID 1-16, i.e. its own TX echo, not a real motor). This matches
// MotorMapping.hpp's kWheelUartBindings topology (2 buses) again, just
// with real (non-default) motor ids this time: UART1 (bus 0) = right
// wheel, id 1; UART2 (bus 1) = left wheel, id 2.
// Index order matches Wheel::name below: [0]=LW, [1]=RW.
inline constexpr std::array<std::uint8_t, kWheelCount> kBenchWheelBus{{1U, 0U}};
inline constexpr std::array<std::uint8_t, kWheelCount> kBenchWheelId{{2U, 1U}};
// Confirmed on the assembled robot 2026-10-12: a "drive forward" command
// spun the right wheel (id=1) correctly but the left wheel (id=2)
// backward.
inline constexpr std::array<float, kWheelCount> kBenchWheelSign{{1.0F, -1.0F}};

struct Wheel {
  const char* name = "";
  std::uint8_t bus = 0U;
  std::uint8_t id = 1U;
  float sign = 1.0F;          // Driver raw = core raw * sign; core + = forward.
  // Command state, core frame.
  std::int16_t target_raw = 0;
  bool torque_on = false;
  std::uint64_t torque_on_us = 0U;
  std::uint64_t last_nonzero_us = 0U;
  std::uint64_t last_disable_us = 0U;
  bool disable_sent = false;
  // Latched after lost feedback; re-armed by releasing (WHEEL 0) or CLEAR.
  bool fault = false;
  // WHOLD <left> <right>, per wheel (2026-10-13 - each motor has its own
  // UART bus now, so they hold independently): 1 switches the driver to
  // Position mode (reg 0x11=2) and locks Goal Position at wherever the
  // wheel was when the hold engaged - 2026-10-12, user report: Speed=0
  // "active brake" (the previous implementation) gave almost no
  // resistance to hand-turning, because it only engages the driver's
  // velocity loop, which is soft at zero; the position loop is much
  // stiffer. A nonzero WHEEL command for that wheel, DISABLE, ESTOP or
  // CLEAR releases it and switches back to Speed mode (reg 0x11=1) -
  // required before normal Moving Speed driving works again.
  bool hold = false;
  // Position-mode hold sub-state (see tickWheels). position_mode mirrors
  // what we last told the driver reg 0x11 is; mode_switch_us times out the
  // settle after that write before torque/goal commands go out, since it's
  // a flash write.
  bool position_mode = false;
  std::uint64_t mode_switch_us = 0U;
  std::uint16_t hold_goal_raw = 0U;
  // 2026-10-13 incident: reg 0x11 persists to flash ON THE MOTOR, so a
  // hold left engaged across an ESP32 reset/reflash (or a power loss
  // mid-hold) leaves the motor in Position mode while this struct's
  // position_mode defaults back to false - RAM and the real driver
  // disagree, and Moving Speed writes to a motor actually in Position
  // mode made it stop answering entirely (one wheel silently "lost").
  // Force a real mode write once at boot, regardless of what this
  // struct assumes, so the two can never start out of sync.
  bool boot_synced = false;
  // Latest Present Data, core frame (speed) / raw single-turn position.
  std::uint16_t pos_raw = 0U;
  std::int16_t speed_raw = 0;
  std::int16_t load_raw = 0;
  std::uint8_t err = 0U;
  std::uint64_t fb_us = 0U;
  bool fb_valid = false;
};

// Byte-stream framer for wheel responses; same rules as the robot firmware's
// Esp32UartPort::readFrame. Only checksum-valid responses come out.
class DxlStreamParser {
 public:
  // Drops any partially-collected frame. Call before each new command on a
  // shared bus: a byte left over from a reply that arrived too late to be
  // matched to its own command would otherwise shift-corrupt the NEXT
  // command's response (seen as a silent, 100%-reproducible checksum fail).
  void reset() noexcept {
    size_ = 0U;
    expected_ = 0U;
  }

  bool push(std::uint8_t byte, frame::DxlFrame& out) noexcept {
    if (size_ == 0U) {
      if (byte == 0xFFU) buffer_[size_++] = byte;
      return false;
    }
    if (size_ == 1U && byte != 0xFFU) {
      restart(byte);
      return false;
    }
    if (size_ >= buffer_.size()) {
      restart(byte);
      return false;
    }
    buffer_[size_++] = byte;
    if (size_ == 4U) {
      const std::size_t total = static_cast<std::size_t>(buffer_[3]) + 4U;
      if (total < 8U || total > buffer_.size()) {
        restart(byte);
        return false;
      }
      expected_ = total;
    }
    if (expected_ != 0U && size_ == expected_) {
      frame::DxlFrame candidate{};
      candidate.len = static_cast<std::uint8_t>(size_);
      std::copy_n(buffer_.begin(), size_, candidate.bytes.begin());
      size_ = 0U;
      expected_ = 0U;
      if (frame::parseDxlResponse(candidate.bytes.data(), candidate.len).ok) {
        out = candidate;
        return true;
      }
    }
    return false;
  }

 private:
  void restart(std::uint8_t byte) noexcept {
    size_ = byte == 0xFFU ? 1U : 0U;
    expected_ = 0U;
    if (size_ != 0U) buffer_[0] = 0xFFU;
  }

  std::array<std::uint8_t, 16> buffer_{};
  std::size_t size_ = 0U;
  std::size_t expected_ = 0U;
};

class LegJog {
 public:
  explicit LegJog(TxSink& tx, Config config = {}) noexcept
      : tx_(tx), config_(config) {
    for (std::size_t i = 0U; i < kJointCount; ++i) {
      const auto& binding = mapping::kVerifiedLegCanBindings[i];
      joints_[i].node = binding.node_id;
      joints_[i].sign = binding.driver_sign;
      joints_[i].name = jointName(binding.joint_index);
    }
    for (std::size_t i = 0U; i < kWheelCount; ++i) {
      // MotorMapping.hpp's motor_id defaults to 1 for both (point-to-point,
      // id doesn't need to be unique there); this bench rig has real,
      // distinct ids set on the motors, so override bus/id/sign here - see
      // kBenchWheelBus/kBenchWheelId/kBenchWheelSign above.
      wheels_[i].bus = kBenchWheelBus[i];
      wheels_[i].id = kBenchWheelId[i];
      wheels_[i].sign = kBenchWheelSign[i];
      wheels_[i].name = i == 0U ? "LW" : "RW";
    }
  }

  // One checksum-valid wheel response (DxlStreamParser output). Matched to
  // a wheel by its motor id rather than by `bus` directly, since ids are
  // unique anyway and this works regardless of topology.
  void onUart(std::uint8_t bus, const frame::DxlFrame& input,
              std::uint64_t now_us) noexcept {
    (void)bus;
    const auto parsed = frame::parseDxlResponse(input.bytes.data(), input.len);
    if (!parsed.ok) return;
    Wheel* const wheel = findWheelById(parsed.id);
    if (wheel == nullptr) return;
    wheel->err = parsed.err;
    const auto present = frame::parsePresentPayload(parsed);
    if (!present.valid) return;  // Write ack.
    wheel->pos_raw = static_cast<std::uint16_t>(present.position_raw & 0x03FFU);
    wheel->speed_raw = static_cast<std::int16_t>(wheel->sign * present.speed_raw);
    wheel->load_raw = present.load_raw;
    wheel->fb_us = now_us;
    wheel->fb_valid = true;
  }

  // ---- inputs -------------------------------------------------------------

  void onCan(std::uint16_t id, std::uint8_t dlc, const std::uint8_t* data,
             std::uint64_t now_us) noexcept {
    if (dlc > 8U || data == nullptr) return;
    const auto node = static_cast<std::uint8_t>(id >> 5U);
    const auto command = static_cast<std::uint8_t>(id & 0x1FU);
    Joint* joint = findNode(node);
    if (joint == nullptr) return;
    frame::MitBytes bytes{};
    std::memcpy(bytes.data(), data, dlc);

    // MIT feedback only uses bytes 0-5 (GDS68_DRIVER.md §4), so drivers may
    // send DLC 6; the heartbeat needs its first 5 bytes.
    if (command == frame::kMitControlCmdId) {
      if (dlc < 6U) return;
      const auto fb = frame::unpackMitFeedback(bytes);
      if (fb.node_id != node) {
        ++joint->mit_bad_echo;
        return;
      }
      if (!std::isfinite(fb.pos_rad) || !std::isfinite(fb.vel_rad_s) ||
          !std::isfinite(fb.torque_nm)) return;
      joint->q = joint->sign * fb.pos_rad;
      joint->dq = joint->sign * fb.vel_rad_s;
      joint->tau = joint->sign * fb.torque_nm;
      joint->fb_us = now_us;
      joint->fb_valid = true;
      ++joint->mit_ok;
    } else if (command == frame::kHeartbeatCmdId) {
      if (dlc < 5U) return;
      joint->axis_error = static_cast<std::uint32_t>(bytes[0]) |
          (static_cast<std::uint32_t>(bytes[1]) << 8U) |
          (static_cast<std::uint32_t>(bytes[2]) << 16U) |
          (static_cast<std::uint32_t>(bytes[3]) << 24U);
      joint->axis_state = bytes[4];
      joint->hb_us = now_us;
      joint->hb_valid = true;
    }
  }

  // One text command from the PC. Writes an optional reply line (empty when
  // the command is a quiet stream command such as HB/JOG).
  void onLine(const char* input, std::uint64_t now_us, char* reply,
              std::size_t reply_size) noexcept {
    if (reply != nullptr && reply_size > 0U) reply[0] = '\0';
    char line[128]{};
    std::size_t length = 0U;
    for (; input != nullptr && input[length] != '\0' && length + 1U < sizeof(line);
         ++length) {
      line[length] = static_cast<char>(
          std::toupper(static_cast<unsigned char>(input[length])));
    }
    line[length] = '\0';

    char* cursor = line;
    char word[24]{};
    if (!nextWord(cursor, word, sizeof(word))) return;
    const auto say = [&](const char* format, auto... args) {
      if (reply == nullptr || reply_size == 0U) return;
      if constexpr (sizeof...(args) == 0U) {
        std::snprintf(reply, reply_size, "%s", format);
      } else {
        std::snprintf(reply, reply_size, format, args...);
      }
    };

    if (std::strcmp(word, "HB") == 0) {
      touchHost(now_us);
      return;
    }
    if (std::strcmp(word, "JOG") == 0) {
      float v[kJointCount]{};
      if (!parseFloats(cursor, v, kJointCount)) {
        say("EV ERR JOG needs 4 numbers");
        return;
      }
      touchHost(now_us);
      if (mode_ != Mode::Active) return;  // Held still while engaging.
      for (std::size_t i = 0U; i < kJointCount; ++i) {
        joints_[i].jog_rad_s = std::clamp(v[i], -config_.speed_max_rad_s,
                                          config_.speed_max_rad_s);
        if (joints_[i].jog_rad_s != 0.0F) joints_[i].has_goal = false;
      }
      return;
    }
    if (std::strcmp(word, "WHEEL") == 0) {
      float v[kWheelCount]{};
      if (!parseFloats(cursor, v, kWheelCount)) {
        say("EV ERR WHEEL needs 2 numbers (raw, + = forward)");
        return;
      }
      touchHost(now_us);
      last_wheel_cmd_us_ = now_us;
      const bool latched = mode_ == Mode::Fault || mode_ == Mode::Estop;
      const float limit = static_cast<float>(config_.wheel_raw_max);
      for (std::size_t i = 0U; i < kWheelCount; ++i) {
        const auto raw = static_cast<std::int16_t>(
            std::lround(std::clamp(v[i], -limit, limit)));
        if (raw == 0) wheels_[i].fault = false;
        // A real (nonzero) jog command is explicit intent: release the hold.
        // WHEEL 0 0 is also the GUI's idle heartbeat, sent continuously, so
        // it must NOT cancel a hold.
        if (raw != 0) wheels_[i].hold = false;
        wheels_[i].target_raw = latched || wheels_[i].fault ? 0 : raw;
      }
      return;
    }
    if (std::strcmp(word, "WHOLD") == 0) {
      // Per-wheel (2026-10-13): each motor has its own UART bus now, so
      // one wheel can hold while the other drives/coasts independently.
      float v[kWheelCount]{};
      if (!parseFloats(cursor, v, kWheelCount)) {
        say("EV ERR WHOLD <left 0|1> <right 0|1> (1 = active-brake that wheel at max hold)");
        return;
      }
      touchHost(now_us);
      last_wheel_cmd_us_ = now_us;  // arm the lease even without a WHEEL heartbeat
      for (std::size_t i = 0U; i < kWheelCount; ++i) {
        const bool on = v[i] != 0.0F;
        wheels_[i].hold = on;
        if (on) wheels_[i].fault = false;
      }
      say("EV OK WHOLD %d %d", static_cast<int>(wheels_[0].hold),
          static_cast<int>(wheels_[1].hold));
      return;
    }
    if (std::strcmp(word, "ENABLE") == 0) {
      touchHost(now_us);
      if (mode_ != Mode::Disabled) {
        say("EV ERR ENABLE refused: mode %s (CLEAR first)", toString(mode_));
        return;
      }
      for (const auto& joint : joints_) {
        if (!fresh(joint, now_us)) {
          say("EV ERR ENABLE refused: no MIT feedback from %s (node %u, bad_echo=%lu)",
              joint.name, static_cast<unsigned>(joint.node),
              static_cast<unsigned long>(joint.mit_bad_echo));
          return;
        }
        if (joint.hb_valid && joint.axis_error != 0U) {
          say("EV ERR ENABLE refused: %s axis_error=0x%08lX", joint.name,
              static_cast<unsigned long>(joint.axis_error));
          return;
        }
      }
      engage(now_us);
      say("EV OK ENABLE: engaging, gains ramp over %lu ms",
          static_cast<unsigned long>(config_.gain_ramp_us / 1000U));
      return;
    }
    if (std::strcmp(word, "DISABLE") == 0) {
      touchHost(now_us);
      if (mode_ == Mode::Estop) {
        say("EV ERR ESTOP latched (CLEAR first)");
        return;
      }
      goIdle(Mode::Disabled, now_us);
      say("EV OK DISABLE");
      return;
    }
    if (std::strcmp(word, "ESTOP") == 0) {
      touchHost(now_us);
      goIdle(Mode::Estop, now_us);
      say("EV OK ESTOP latched");
      return;
    }
    if (std::strcmp(word, "CLEAR") == 0) {
      touchHost(now_us);
      if (mode_ == Mode::Fault || mode_ == Mode::Estop) {
        goIdle(Mode::Disabled, now_us);
        say("EV OK CLEAR");
      } else {
        say("EV OK CLEAR (nothing latched)");
      }
      return;
    }
    if (std::strcmp(word, "HOLD") == 0) {
      touchHost(now_us);
      for (auto& joint : joints_) {
        joint.jog_rad_s = 0.0F;
        joint.has_goal = false;
      }
      say("EV OK HOLD");
      return;
    }
    if (std::strcmp(word, "GOTO") == 0) {
      float v[2]{};
      if (!parseFloats(cursor, v, 2U) || v[0] < 0.0F ||
          v[0] >= static_cast<float>(kJointCount) ||
          v[0] != std::floor(v[0])) {
        say("EV ERR GOTO <joint 0-3> <rad>");
        return;
      }
      touchHost(now_us);
      if (mode_ != Mode::Active) {
        say("EV ERR GOTO needs ACTIVE");
        return;
      }
      Joint& joint = joints_[static_cast<std::size_t>(v[0])];
      joint.jog_rad_s = 0.0F;
      joint.goal = clampTravel(joint, v[1]);
      joint.has_goal = true;
      say("EV OK GOTO %s %.4f", joint.name, static_cast<double>(joint.goal));
      return;
    }
    if (std::strcmp(word, "GOTO_START") == 0) {
      touchHost(now_us);
      if (mode_ != Mode::Active) {
        say("EV ERR GOTO_START needs ACTIVE");
        return;
      }
      for (auto& joint : joints_) {
        joint.jog_rad_s = 0.0F;
        joint.goal = joint.start;
        joint.has_goal = true;
      }
      say("EV OK GOTO_START");
      return;
    }
    if (std::strcmp(word, "GAINS") == 0) {
      float v[2]{};
      if (!parseFloats(cursor, v, 2U)) {
        say("EV ERR GAINS <kp> <kd>");
        return;
      }
      touchHost(now_us);
      settings_.kp = std::clamp(v[0], 0.0F, config_.kp_max);
      settings_.kd = std::clamp(v[1], 0.0F, config_.kd_max);
      say("EV OK GAINS kp=%.2f kd=%.2f", static_cast<double>(settings_.kp),
          static_cast<double>(settings_.kd));
      return;
    }
    if (std::strcmp(word, "SPEED") == 0) {
      float v[1]{};
      if (!parseFloats(cursor, v, 1U)) {
        say("EV ERR SPEED <rad/s>");
        return;
      }
      touchHost(now_us);
      settings_.speed_rad_s = std::clamp(v[0], 0.01F, config_.speed_max_rad_s);
      say("EV OK SPEED %.3f", static_cast<double>(settings_.speed_rad_s));
      return;
    }
    if (std::strcmp(word, "LIMIT") == 0) {
      float v[1]{};
      if (!parseFloats(cursor, v, 1U)) {
        say("EV ERR LIMIT <rad>");
        return;
      }
      touchHost(now_us);
      settings_.travel_rad = std::clamp(v[0], 0.05F, config_.travel_max_rad);
      say("EV OK LIMIT %.3f", static_cast<double>(settings_.travel_rad));
      return;
    }
    say("EV ERR unknown command");
  }

  // ---- periodic -----------------------------------------------------------

  void tick(std::uint64_t now_us) noexcept {
    if (ticked_ && now_us - last_tick_us_ < config_.tick_us) return;
    ticked_ = true;
    last_tick_us_ = now_us;
    tickWheels(now_us);

    if (mode_ != Mode::Engaging && mode_ != Mode::Active) {
      if (!idle_sent_ || now_us - last_idle_us_ >= config_.idle_resend_us) {
        sendIdleAll();
        idle_sent_ = true;
        last_idle_us_ = now_us;
      }
      // Zero-gain MIT probe: the driver reports its position without torque.
      const auto probe = frame::packMitCommand(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
      for (const auto& joint : joints_) {
        (void)tx_.send(frame::canId(joint.node, frame::kMitControlCmdId), probe);
      }
      return;
    }

    if (now_us - last_host_us_ > config_.host_timeout_us) {
      fault(now_us, "host timeout (no command from PC)");
      return;
    }
    const std::uint64_t since_engage = now_us - engage_us_;
    for (const auto& joint : joints_) {
      if (!fresh(joint, now_us)) {
        faultJoint(now_us, joint, "MIT feedback timeout");
        return;
      }
      if (since_engage >= config_.closed_loop_grace_us) {
        if (!joint.hb_valid ||
            now_us - joint.hb_us > config_.heartbeat_timeout_us) {
          faultJoint(now_us, joint, "heartbeat lost");
          return;
        }
        if (joint.axis_error != 0U) {
          faultJoint(now_us, joint, "driver axis_error");
          return;
        }
        if (joint.axis_state != frame::kAxisStateClosedLoop) {
          faultJoint(now_us, joint, "not in CLOSED_LOOP");
          return;
        }
      }
      if (std::abs(joint.tau) > config_.max_torque_nm) {
        faultJoint(now_us, joint, "torque limit");
        return;
      }
    }

    const float dt = static_cast<float>(config_.tick_us) * 1.0e-6F;
    const float gain = config_.gain_ramp_us == 0U ? 1.0F
        : std::min(1.0F, static_cast<float>(since_engage) /
                             static_cast<float>(config_.gain_ramp_us));
    for (auto& joint : joints_) {
      if (mode_ == Mode::Active) {
        if (joint.jog_rad_s != 0.0F) {
          joint.target += joint.jog_rad_s * dt;
        } else if (joint.has_goal) {
          const float step = settings_.speed_rad_s * dt;
          const float error = joint.goal - joint.target;
          if (std::abs(error) <= step) {
            joint.target = joint.goal;
            joint.has_goal = false;
          } else {
            joint.target += error > 0.0F ? step : -step;
          }
        }
      }
      joint.target = clampTravel(joint, joint.target);
      if (std::abs(joint.target - joint.q) > config_.max_tracking_error_rad) {
        faultJoint(now_us, joint, "tracking error");
        return;
      }
    }

    bool sent = true;
    for (const auto& joint : joints_) {
      sent = tx_.send(frame::canId(joint.node, frame::kMitControlCmdId),
                      frame::packMitCommand(joint.sign * joint.target, 0.0F,
                                            settings_.kp * gain,
                                            settings_.kd * gain, 0.0F)) && sent;
    }
    tx_failures_ = sent ? 0U : tx_failures_ + 1U;
    if (tx_failures_ >= 5U) {
      fault(now_us, "CAN TX failing");
      return;
    }

    if (mode_ == Mode::Engaging && gain >= 1.0F &&
        since_engage >= config_.closed_loop_grace_us) {
      mode_ = Mode::Active;
      pushEvent("EV ACTIVE: all 4 joints holding; JOG/GOTO accepted");
    }
  }

  // ---- outputs ------------------------------------------------------------

  // CSV status line; see tools/leg_jog_gui.py parse_status().
  void formatStatus(std::uint64_t now_us, char* out, std::size_t size) const noexcept {
    if (out == nullptr || size == 0U) return;
    int used = std::snprintf(out, size, "ST,%s,%.2f,%.2f,%.3f,%.3f",
                             toString(mode_), static_cast<double>(settings_.kp),
                             static_cast<double>(settings_.kd),
                             static_cast<double>(settings_.speed_rad_s),
                             static_cast<double>(settings_.travel_rad));
    for (const auto& joint : joints_) {
      if (used < 0 || static_cast<std::size_t>(used) >= size) return;
      const long age_ms = joint.fb_valid && now_us >= joint.fb_us
          ? static_cast<long>((now_us - joint.fb_us) / 1000U) : -1L;
      used += std::snprintf(
          out + used, size - static_cast<std::size_t>(used),
          ",%.4f,%.4f,%.4f,%.3f,%ld,%u,%lX,%lu",
          static_cast<double>(joint.q), static_cast<double>(joint.target),
          static_cast<double>(joint.start), static_cast<double>(joint.tau),
          age_ms, static_cast<unsigned>(joint.hb_valid ? joint.axis_state : 0U),
          static_cast<unsigned long>(joint.axis_error),
          static_cast<unsigned long>(joint.mit_bad_echo));
    }
    for (const auto& wheel : wheels_) {
      if (used < 0 || static_cast<std::size_t>(used) >= size) return;
      const long age_ms = wheel.fb_valid && now_us >= wheel.fb_us
          ? static_cast<long>((now_us - wheel.fb_us) / 1000U) : -1L;
      used += std::snprintf(
          out + used, size - static_cast<std::size_t>(used), ",%d,%d,%d,%u,%ld,%u",
          wheel.fault ? 2 : (wheel.hold ? 3 : (wheel.torque_on ? 1 : 0)),
          static_cast<int>(wheel.target_raw),
          static_cast<int>(wheel.speed_raw), static_cast<unsigned>(wheel.pos_raw),
          age_ms, static_cast<unsigned>(wheel.err));
    }
  }

  bool popEvent(char* out, std::size_t size) noexcept {
    if (event_count_ == 0U || out == nullptr || size == 0U) return false;
    std::snprintf(out, size, "%s", events_[event_head_].data());
    event_head_ = (event_head_ + 1U) % events_.size();
    --event_count_;
    return true;
  }

  [[nodiscard]] Mode mode() const noexcept { return mode_; }
  [[nodiscard]] const Joint& joint(std::size_t i) const noexcept { return joints_[i]; }
  [[nodiscard]] const Settings& settings() const noexcept { return settings_; }
  [[nodiscard]] const Wheel& wheel(std::size_t i) const noexcept { return wheels_[i]; }

 private:
  static const char* jointName(std::size_t joint_index) noexcept {
    switch (joint_index) {
      case 0U: return "LH";
      case 1U: return "LK";
      case 3U: return "RH";
      case 4U: return "RK";
      default: return "?";
    }
  }

  static bool nextWord(char*& cursor, char* word, std::size_t size) noexcept {
    while (*cursor != '\0' && std::isspace(static_cast<unsigned char>(*cursor))) ++cursor;
    std::size_t n = 0U;
    while (*cursor != '\0' && !std::isspace(static_cast<unsigned char>(*cursor))) {
      if (n + 1U < size) word[n++] = *cursor;
      ++cursor;
    }
    word[n] = '\0';
    return n > 0U;
  }

  // Exactly `count` finite numbers and nothing after them.
  static bool parseFloats(char* cursor, float* out, std::size_t count) noexcept {
    for (std::size_t i = 0U; i < count; ++i) {
      char* end = nullptr;
      out[i] = std::strtof(cursor, &end);
      if (end == cursor || !std::isfinite(out[i])) return false;
      cursor = end;
    }
    while (*cursor != '\0') {
      if (!std::isspace(static_cast<unsigned char>(*cursor))) return false;
      ++cursor;
    }
    return true;
  }

  Joint* findNode(std::uint8_t node) noexcept {
    for (auto& joint : joints_) {
      if (joint.node == node) return &joint;
    }
    return nullptr;
  }

  Wheel* findWheelById(std::uint8_t id) noexcept {
    for (auto& wheel : wheels_) {
      if (wheel.id == id) return &wheel;
    }
    return nullptr;
  }

  bool fresh(const Joint& joint, std::uint64_t now_us) const noexcept {
    return joint.fb_valid && now_us >= joint.fb_us &&
        now_us - joint.fb_us <= config_.feedback_timeout_us;
  }

  float clampTravel(const Joint& joint, float value) const noexcept {
    const float low = std::max(joint.start - settings_.travel_rad,
                               -config_.driver_pos_limit_rad);
    const float high = std::min(joint.start + settings_.travel_rad,
                                config_.driver_pos_limit_rad);
    return std::clamp(value, low, high);
  }

  void touchHost(std::uint64_t now_us) noexcept { last_host_us_ = now_us; }

  void sendIdleAll() noexcept {
    const auto idle = frame::packSetAxisState(frame::kAxisStateIdle);
    for (const auto& joint : joints_) {
      (void)tx_.send(frame::canId(joint.node, frame::kSetAxisStateCmdId), idle);
    }
  }

  void engage(std::uint64_t now_us) noexcept {
    for (auto& joint : joints_) {
      joint.start = joint.q;
      joint.target = joint.q;
      joint.goal = joint.q;
      joint.has_goal = false;
      joint.jog_rad_s = 0.0F;
    }
    bool sent = true;
    const auto mode = frame::packSetControllerMode(frame::kControlModePosition,
                                                   frame::kInputModeMit);
    for (const auto& joint : joints_) {
      sent = tx_.send(frame::canId(joint.node, frame::kSetControllerModeCmdId),
                      mode) && sent;
    }
    const auto closed = frame::packSetAxisState(frame::kAxisStateClosedLoop);
    for (const auto& joint : joints_) {
      sent = tx_.send(frame::canId(joint.node, frame::kSetAxisStateCmdId),
                      closed) && sent;
    }
    mode_ = Mode::Engaging;
    engage_us_ = now_us;
    tx_failures_ = 0U;
    if (!sent) fault(now_us, "CAN TX failed while enabling");
  }

  void goIdle(Mode mode, std::uint64_t now_us) noexcept {
    for (auto& joint : joints_) {
      joint.jog_rad_s = 0.0F;
      joint.has_goal = false;
    }
    mode_ = mode;
    sendIdleAll();
    idle_sent_ = true;
    last_idle_us_ = now_us;
    for (auto& wheel : wheels_) {
      wheel.target_raw = 0;
      wheel.hold = false;
      if (mode == Mode::Disabled) wheel.fault = false;
      wheelTorqueOff(wheel, now_us);
    }
  }

  void wheelTorqueOff(Wheel& wheel, std::uint64_t now_us) noexcept {
    (void)tx_.sendUart(wheel.bus, frame::buildTorqueEnable(wheel.id, false));
    wheel.torque_on = false;
    wheel.disable_sent = true;
    wheel.last_disable_us = now_us;
  }

  void sendWheelSpeed(const Wheel& wheel, std::int16_t core_raw) noexcept {
    const float driver_raw = wheel.sign * static_cast<float>(core_raw);
    (void)tx_.sendUart(wheel.bus, frame::buildMovingSpeed(
        wheel.id, frame::wheelRawSpeedToRadPerSec(
                      static_cast<std::int16_t>(std::lround(driver_raw)))));
  }

  // Hold-to-run: Torque Enable, settle, Moving Speed; on release brake at
  // speed 0 briefly, then coast (Torque Disable, repeated while stopped).
  void tickWheels(std::uint64_t now_us) noexcept {
    const bool allowed = mode_ != Mode::Fault && mode_ != Mode::Estop &&
        now_us - last_host_us_ <= config_.host_timeout_us &&
        now_us - last_wheel_cmd_us_ <= config_.wheel_lease_us;
    for (auto& wheel : wheels_) {
      if (!wheel.boot_synced) {
        // Force Speed mode once, regardless of what this struct assumes -
        // see boot_synced's comment. Takes priority over everything else
        // this wheel might otherwise do this tick.
        (void)tx_.sendUart(wheel.bus,
            frame::buildVelocityMode(wheel.id, frame::kDxlVelModeSpeed));
        wheel.boot_synced = true;
        wheel.position_mode = false;
        wheel.mode_switch_us = now_us;
        (void)tx_.sendUart(wheel.bus, frame::buildReadPresent(wheel.id));
        continue;
      }
      if (wheel.torque_on && (!wheel.fb_valid ||
          now_us - wheel.fb_us > config_.wheel_feedback_timeout_us) &&
          now_us - wheel.torque_on_us > config_.wheel_feedback_timeout_us) {
        wheel.target_raw = 0;
        wheel.fault = true;
        wheelTorqueOff(wheel, now_us);
        char text[80];
        std::snprintf(text, sizeof(text),
                      "EV WHEEL %s feedback lost -> torque off (release to re-arm)",
                      wheel.name);
        pushEvent(text);
      }
      // A mode switch (reg 0x11, see below) is a flash write on this
      // driver - let it settle (mode_switch_settle_us; measured needing
      // much more than wheel_settle_us on at least one motor unit) before
      // sending anything else, in either direction, before checking what
      // else this wheel needs this tick.
      if (wheel.mode_switch_us != 0U &&
          now_us - wheel.mode_switch_us < config_.mode_switch_settle_us) {
        (void)tx_.sendUart(wheel.bus, frame::buildReadPresent(wheel.id));
        continue;
      }
      const bool holding = allowed && !wheel.fault && wheel.hold;
      if (!holding && wheel.position_mode) {
        // Leaving hold (or it was never released cleanly, e.g. after a
        // fault): switch back to Speed mode first - required before normal
        // Moving Speed driving commands take effect - before anything else
        // runs for this wheel this tick.
        (void)tx_.sendUart(wheel.bus,
            frame::buildVelocityMode(wheel.id, frame::kDxlVelModeSpeed));
        wheel.position_mode = false;
        wheel.mode_switch_us = now_us;
        wheel.torque_on = false;  // force a clean re-enable in the new mode.
        wheel.disable_sent = false;
        (void)tx_.sendUart(wheel.bus, frame::buildReadPresent(wheel.id));
        continue;
      }
      if (holding) {
        // Position-mode hold, indefinitely: much stiffer against an
        // external push than the old Speed=0 "active brake" (velocity loop
        // is soft at zero on this driver). Never falls through to the
        // brake-timeout/coast path below.
        if (!wheel.position_mode) {
          // Safety net: if the mode switch below hasn't actually taken
          // effect by the time we start treating this wheel as held (see
          // mode_switch_settle_us), the motor is still in Speed mode and
          // would otherwise keep obeying whatever Moving Speed it last
          // got - stop that first, in the mode it's still actually in.
          sendWheelSpeed(wheel, 0);
          (void)tx_.sendUart(wheel.bus,
              frame::buildVelocityMode(wheel.id, frame::kDxlVelModePosition));
          wheel.position_mode = true;
          wheel.mode_switch_us = now_us;
          wheel.hold_goal_raw = wheel.pos_raw;  // lock wherever it is now.
          wheel.torque_on = false;  // force a clean re-enable in the new mode.
        } else {
          if (!wheel.torque_on) {
            (void)tx_.sendUart(wheel.bus, frame::buildTorqueEnable(wheel.id, true));
            wheel.torque_on = true;
            wheel.torque_on_us = now_us;
          } else {
            (void)tx_.sendUart(wheel.bus,
                frame::buildGoalPosition(wheel.id, wheel.hold_goal_raw));
          }
        }
        wheel.last_nonzero_us = now_us;  // keeps the (unused) brake window fresh
        (void)tx_.sendUart(wheel.bus, frame::buildReadPresent(wheel.id));
        continue;
      }
      const std::int16_t target = allowed && !wheel.fault ? wheel.target_raw : 0;
      if (target != 0) {
        wheel.last_nonzero_us = now_us;
        if (!wheel.torque_on) {
          (void)tx_.sendUart(wheel.bus, frame::buildTorqueEnable(wheel.id, true));
          wheel.torque_on = true;
          wheel.torque_on_us = now_us;
        } else if (now_us - wheel.torque_on_us >= config_.wheel_settle_us) {
          sendWheelSpeed(wheel, target);
        }
      } else {
        if (wheel.torque_on) {
          if (now_us - wheel.last_nonzero_us < config_.wheel_brake_us &&
              now_us - wheel.torque_on_us >= config_.wheel_settle_us) {
            sendWheelSpeed(wheel, 0);
          } else {
            wheelTorqueOff(wheel, now_us);
          }
        } else if (!wheel.disable_sent ||
                   now_us - wheel.last_disable_us >= config_.wheel_disable_resend_us) {
          wheelTorqueOff(wheel, now_us);
        }
      }
      (void)tx_.sendUart(wheel.bus, frame::buildReadPresent(wheel.id));
    }
  }

  void fault(std::uint64_t now_us, const char* reason) noexcept {
    goIdle(Mode::Fault, now_us);
    char text[96];
    std::snprintf(text, sizeof(text), "EV FAULT %s -> all joints IDLE", reason);
    pushEvent(text);
  }

  void faultJoint(std::uint64_t now_us, const Joint& joint, const char* reason) noexcept {
    char text[80];
    std::snprintf(text, sizeof(text), "%s %s (node %u)", joint.name, reason,
                  static_cast<unsigned>(joint.node));
    fault(now_us, text);
  }

  void pushEvent(const char* text) noexcept {
    const std::size_t slot = (event_head_ + event_count_) % events_.size();
    std::snprintf(events_[slot].data(), events_[slot].size(), "%s", text);
    if (event_count_ < events_.size()) {
      ++event_count_;
    } else {
      event_head_ = (event_head_ + 1U) % events_.size();
    }
  }

  TxSink& tx_;
  Config config_;
  Settings settings_{};
  std::array<Joint, kJointCount> joints_{};
  std::array<Wheel, kWheelCount> wheels_{};
  std::uint64_t last_wheel_cmd_us_ = 0U;
  Mode mode_ = Mode::Disabled;
  std::uint64_t last_tick_us_ = 0U;
  std::uint64_t last_idle_us_ = 0U;
  std::uint64_t last_host_us_ = 0U;
  std::uint64_t engage_us_ = 0U;
  std::uint32_t tx_failures_ = 0U;
  bool ticked_ = false;
  bool idle_sent_ = false;
  std::array<std::array<char, 112>, 6> events_{};
  std::size_t event_head_ = 0U;
  std::size_t event_count_ = 0U;
};

}  // namespace legjog
