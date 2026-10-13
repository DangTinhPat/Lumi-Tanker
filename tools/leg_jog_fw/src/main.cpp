// Bench jog firmware (DRAFT) for the TAP ESP32-S3 board.
//
// Replaces the USB-CAN-A adapter of TEST1/gds68_controller.py: the ESP32
// streams MIT hold/jog frames to all four GDS68 leg joints at 100 Hz, drives
// the two EC4012/R1-POC wheels (Dynamixel-like protocol, MotorFrameCodec.hpp)
// over two separate half-duplex single-wire UART buses (hold-to-run), and
// takes line commands from tools/leg_jog_gui.py over the CP2102 serial port
// (115200 baud). Boots with every leg axis IDLE and both wheels torque-off.
// No IMU.
//
// 2026-10-13, second revision: "UART1"/"UART2" on this board's J6-style
// connectors each carry ONE combined TX/RX data wire to their wheel (same
// half-duplex single-wire design as TEST1/esp32s3_r1poc_bridge used for the
// shared bus, just two independent instances of it now instead of one bus
// for both motors). Treating them as plain 2-pin full-duplex UARTs (this
// file's first revision today) looked like it worked for the right wheel by
// luck, but scanning IDs on the left wheel's "RX" pin got an identical reply
// for every single ID 1-16 - a dead giveaway of hearing our own TX echo
// (shorted to RX on the PCB), not a real motor. Both buses now use the
// proper bus-drive/release + echo-filter technique.
// Pins: "UART1" bus = GPIO17, right wheel, id 1. "UART2" bus = GPIO15, left
// wheel, id 2 (LegJog.hpp's kBenchWheelBus/kBenchWheelId).
//
// This is NOT the robot controller firmware. Reflash firmware_esp32 afterwards.

#include <Arduino.h>
#include <driver/twai.h>
#include <driver/uart.h>
#include <esp_rom_gpio.h>
#include <esp_timer.h>
#include <hal/gpio_ll.h>
#include <hal/uart_ll.h>
#include <soc/gpio_sig_map.h>
#include <soc/gpio_struct.h>

#include <cstdio>
#include <cstring>

#include "LegJog.hpp"

namespace {

constexpr gpio_num_t kCanTxGpio = GPIO_NUM_4;
constexpr gpio_num_t kCanRxGpio = GPIO_NUM_5;
constexpr std::uint32_t kWheelBaud = 1000000U;
constexpr std::uint64_t kStatusPeriodUs = 50000U;       // 20 Hz to the GUI.
constexpr std::uint64_t kBusOffRecoveryDelayUs = 200000U;
// Generous vs. the sub-ms unloaded turnaround at 1 Mbps - a wheel driven
// against resistance/held replies slower (busier control loop on the
// driver), and the whole point of waiting per-command is to never start
// the next command while a reply might still be inbound.
constexpr std::int64_t kReplyWindowUs = 6000;

// One half-duplex, single-wire UART bus: TX and RX share one GPIO pin, same
// technique as TEST1/esp32s3_r1poc_bridge - push-pull while sending, release
// to high-Z (input) so the motor can answer, filter our own TX echoed back
// on RX before parsing. Each wheel gets its own instance (independent GPIO,
// independent UART peripheral, independent echo/parser state) - unlike the
// 2026-10-12 shared-bus draft, these two buses never interact.
class HalfDuplexBus {
 public:
  void begin(uart_port_t port, int pin, std::uint32_t tx_signal_idx) {
    port_ = port;
    pin_ = pin;
    tx_signal_idx_ = tx_signal_idx;

    uart_config_t cfg = {};
    cfg.baud_rate = static_cast<int>(kWheelBaud);
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_APB;

    uart_driver_install(port_, 1024, 0, 0, nullptr, 0);
    uart_param_config(port_, &cfg);
    uart_set_pin(port_, pin_, pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // uart_set_pin(TX=RX) leaves the pin read-only; gpio_set_direction()
    // then unhooks UART TX from it, so reconnect it explicitly (drive()).
    gpio_set_direction(static_cast<gpio_num_t>(pin_), GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(static_cast<gpio_num_t>(pin_), 1);
    esp_rom_gpio_connect_out_signal(pin_, tx_signal_idx_, false, false);
    release();  // default: release the bus (high-Z) to listen.
    gpio_pullup_en(static_cast<gpio_num_t>(pin_));

    uart_set_rx_full_threshold(port_, 96);
    uart_set_rx_timeout(port_, 2);
  }

  bool send(const legjog::frame::DxlFrame& frame) {
    if (frame.len == 0U || frame.len > frame.bytes.size()) return false;
    // Drop anything left over from a previous reply that arrived just after
    // we gave up waiting on it: a stray byte here would shift-corrupt this
    // command's response instead.
    uart_flush_input(port_);
    parser_.reset();
    echo_tail_ = echo_head_ = 0U;
    echoPush(frame.bytes.data(), frame.len);
    // Half-duplex turnaround: release the wire the instant the last stop bit has
    // left the shifter. uart_wait_tx_done() is a task wakeup, so under load the
    // pin could stay driven while the motor already starts replying (a
    // collision that corrupts both frames). Write the FIFO directly and poll the
    // TX state machine in a critical section bounded to 500 us (<=16 bytes is
    // 160 us at 1 Mbps).
    auto* hw = UART_LL_GET_HW(port_);
    taskENTER_CRITICAL(&mux_);
    const std::int64_t start = esp_timer_get_time();
    bool ok = uart_ll_is_tx_idle(hw) && uart_ll_get_txfifo_len(hw) >= frame.len;
    if (ok) {
      drive();
      uart_ll_write_txfifo(hw, frame.bytes.data(), frame.len);
      while (!uart_ll_is_tx_idle(hw) && esp_timer_get_time() - start < 500) {}
      ok = uart_ll_is_tx_idle(hw);
    }
    release();
    if (!ok) uart_ll_txfifo_rst(hw);
    taskEXIT_CRITICAL(&mux_);
    echo_deadline_us_ = esp_timer_get_time() + kEchoMarginUs;
    return ok;
  }

  // Waits (up to `window_us`) for one complete, checksum-valid frame and
  // hands it to `out_frame`; returns false if nothing parsed in time.
  bool drainReply(std::int64_t window_us, legjog::frame::DxlFrame& out_frame) {
    const std::int64_t deadline_us = esp_timer_get_time() + window_us;
    std::uint8_t byte = 0U;
    for (;;) {
      const int n = uart_read_bytes(port_, &byte, 1, 0);
      if (n == 1) {
        if (echoFilter(byte)) continue;  // our own TX looped back on RX.
        if (parser_.push(byte, out_frame)) return true;
        continue;
      }
      if (esp_timer_get_time() >= deadline_us) return false;
    }
  }

  // Non-blocking: drains whatever is already buffered, no new waiting.
  bool pollReply(legjog::frame::DxlFrame& out_frame) {
    return drainReply(0, out_frame);
  }

 private:
  static constexpr std::size_t kEchoRing = 64U;  // power of 2; frames <=16B.
  static constexpr std::int64_t kEchoMarginUs = 1000;

  void drive() {
    esp_rom_gpio_connect_out_signal(pin_, tx_signal_idx_, false, false);
  }
  void release() {
    esp_rom_gpio_connect_out_signal(pin_, SIG_GPIO_OUT_IDX, false, false);
    gpio_ll_output_disable(&GPIO, static_cast<gpio_num_t>(pin_));
  }
  void echoPush(const std::uint8_t* d, std::size_t n) {
    for (std::size_t i = 0U; i < n && (echo_head_ - echo_tail_) < kEchoRing; ++i) {
      echo_[echo_head_++ % kEchoRing] = d[i];
    }
    echo_deadline_us_ = esp_timer_get_time() + static_cast<std::int64_t>(n) * 12 + kEchoMarginUs;
  }
  bool echoFilter(std::uint8_t b) {
    if (echo_tail_ == echo_head_) return false;
    if (esp_timer_get_time() > echo_deadline_us_) { echo_tail_ = echo_head_; return false; }
    if (echo_[echo_tail_ % kEchoRing] == b) { ++echo_tail_; return true; }
    echo_tail_ = echo_head_;  // lệch -> không nuốt dữ liệu thật của motor
    return false;
  }

  uart_port_t port_ = UART_NUM_1;
  int pin_ = -1;
  std::uint32_t tx_signal_idx_ = 0U;
  std::uint8_t echo_[kEchoRing]{};
  std::uint32_t echo_head_ = 0U, echo_tail_ = 0U;
  std::int64_t echo_deadline_us_ = 0;
  legjog::DxlStreamParser parser_;
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};

HalfDuplexBus wheel_buses[legjog::kWheelCount];  // [0] = UART1/right, [1] = UART2/left.

class BenchSink final : public legjog::TxSink {
 public:
  bool send(std::uint16_t id, const legjog::frame::MitBytes& data) override {
    if (!ready) return false;
    twai_message_t message{};
    message.identifier = id;
    message.data_length_code = 8U;
    std::memcpy(message.data, data.data(), 8U);
    return twai_transmit(&message, 0U) == ESP_OK;
  }
  bool sendUart(std::uint8_t bus, const legjog::frame::DxlFrame& frame) override {
    if (bus >= legjog::kWheelCount) return false;
    HalfDuplexBus& wb = wheel_buses[bus];
    const bool ok = wb.send(frame);
    // Both drivers run Status Return Level 2 (every instruction is acknowledged, Return Delay 0).
    // The acknowledgement MUST be waited for: skipping it (as when the level was 1) lets the next
    // request collide with the ack on the wire and both wheels stopped answering (2026-10-13).
    legjog::frame::DxlFrame reply{};
    if (wb.drainReply(kReplyWindowUs, reply)) {
      jog_onUart_hook(bus, reply);
    }
    return ok;
  }
  // Set once jog exists (see below); lets sendUart() hand a reply straight
  // to LegJog without a forward-declaration cycle with the global `jog`.
  void (*jog_onUart_hook)(std::uint8_t, const legjog::frame::DxlFrame&) = nullptr;
  bool ready = false;
};

BenchSink sink;
legjog::LegJog jog(sink);

void onUartReply(std::uint8_t bus, const legjog::frame::DxlFrame& frame) {
  jog.onUart(bus, frame, static_cast<std::uint64_t>(esp_timer_get_time()));
}


// ---- Latency characterisation (LATTEST / LATHAND) -------------------------
// Measures how long each wheel driver takes to answer, per drive state, so the
// cause of slow replies can be told apart: torque off vs held at speed 0 vs
// spinning, and sparse (TEST1-style heartbeat) vs flooded Moving Speed writes.
struct LatHist {
  std::uint32_t n = 0U, timeouts = 0U, b[6]{};
  std::uint64_t sum = 0U;
  std::uint32_t max = 0U;
  void add(std::int64_t us) {
    ++n;
    sum += static_cast<std::uint64_t>(us);
    if (static_cast<std::uint32_t>(us) > max) max = static_cast<std::uint32_t>(us);
    static constexpr std::int64_t edges[5] = {1500, 2000, 3000, 5000, 8000};
    std::size_t i = 0U;
    while (i < 5U && us >= edges[i]) ++i;
    ++b[i];
  }
  void print(const char* phase, const char* regime, const char* kind, int id) const {
    char text[200];
    std::snprintf(text, sizeof(text),
        "LAT phase=%s regime=%s %s id=%d n=%lu timeouts=%lu mean_us=%lu max_us=%lu "
        "lt1.5=%lu lt2=%lu lt3=%lu lt5=%lu lt8=%lu ge8=%lu",
        phase, regime, kind, id, static_cast<unsigned long>(n), static_cast<unsigned long>(timeouts),
        static_cast<unsigned long>(n ? sum / n : 0U), static_cast<unsigned long>(max),
        static_cast<unsigned long>(b[0]), static_cast<unsigned long>(b[1]), static_cast<unsigned long>(b[2]),
        static_cast<unsigned long>(b[3]), static_cast<unsigned long>(b[4]), static_cast<unsigned long>(b[5]));
    Serial.println(text);
  }
};

bool timedTransaction(std::uint8_t bus, const legjog::frame::DxlFrame& request, LatHist& hist) {
  HalfDuplexBus& wb = wheel_buses[bus];
  if (!wb.send(request)) { ++hist.timeouts; return false; }
  const std::int64_t t0 = esp_timer_get_time();
  legjog::frame::DxlFrame reply{};
  if (!wb.drainReply(20000, reply)) { ++hist.timeouts; return false; }
  hist.add(esp_timer_get_time() - t0);
  return true;
}

constexpr std::uint8_t kLatId[2] = {1U, 2U};   // bus 0 = right (id 1), bus 1 = left (id 2)

void latSetTorque(bool on) {
  LatHist sink_hist;
  for (std::uint8_t bus = 0U; bus < 2U; ++bus) {
    timedTransaction(bus, legjog::frame::buildTorqueEnable(kLatId[bus], on), sink_hist);
  }
}

void latSetSpeed(int raw) {
  LatHist sink_hist;
  for (std::uint8_t bus = 0U; bus < 2U; ++bus) {
    timedTransaction(bus, legjog::frame::buildMovingSpeed(kLatId[bus],
        legjog::frame::wheelRawSpeedToRadPerSec(static_cast<std::int16_t>(raw))), sink_hist);
  }
}

// Runs one measurement phase: reads every 5 ms on both buses; speed writes every
// 5 ms (flood) or every 250 ms (sparse) when `speed_writes` is set.
void latPhase(const char* phase, const char* regime, bool speed_writes, int speed_raw, bool flood,
              double seconds) {
  LatHist reads[2], writes[2];
  const std::int64_t end = esp_timer_get_time() + static_cast<std::int64_t>(seconds * 1e6);
  std::int64_t next_read[2] = {0, 0}, next_write[2] = {0, 0};
  while (esp_timer_get_time() < end) {
    for (std::uint8_t bus = 0U; bus < 2U; ++bus) {
      const std::int64_t now = esp_timer_get_time();
      if (speed_writes && now >= next_write[bus]) {
        next_write[bus] = now + (flood ? 5000 : 250000);
        timedTransaction(bus, legjog::frame::buildMovingSpeed(kLatId[bus],
            legjog::frame::wheelRawSpeedToRadPerSec(static_cast<std::int16_t>(speed_raw))), writes[bus]);
      } else if (now >= next_read[bus]) {
        next_read[bus] = now + 5000;
        timedTransaction(bus, legjog::frame::buildReadPresent(kLatId[bus]), reads[bus]);
      }
    }
    delay(0);
  }
  for (std::uint8_t bus = 0U; bus < 2U; ++bus) {
    reads[bus].print(phase, regime, "read", kLatId[bus]);
    if (speed_writes) writes[bus].print(phase, regime, "write", kLatId[bus]);
  }
}

void latRampDown(int from_raw) {
  for (int v = from_raw; v >= 0; v -= 10) {
    latSetSpeed(v);
    delay(100);
  }
  latSetSpeed(0);
  delay(200);
  latSetTorque(false);
  delay(200);
}

void latCue() {   // a short twitch of both wheels: tells the operator a measuring phase starts now
  latSetTorque(true);
  delay(25);
  latSetSpeed(40);
  delay(700);
  latSetSpeed(0);
  delay(150);
  latSetTorque(false);
  delay(500);
}

void runLatencyTest() {
  Serial.println("LAT begin LATTEST: wheels spin up to 100 RPM, keep them free");
  latSetTorque(false);
  delay(200);
  for (int regime = 0; regime < 2; ++regime) {
    const bool flood = regime == 1;
    const char* name = flood ? "flood5ms" : "sparse250ms";
    latPhase("A_torque_off", name, false, 0, flood, 3.0);
    latSetTorque(true);
    delay(25);
    latPhase("B_hold_speed0", name, true, 0, flood, 3.0);
    latPhase("C_spin60", name, true, 60, flood, 3.0);
    latPhase("D_spin100", name, true, 100, flood, 3.0);
    latRampDown(100);
  }
  Serial.println("LAT end");
}

void runHandTest() {
  Serial.println("LAT begin LATHAND: after each twitch rotate the LEFT wheel by hand for 6 s (then the RIGHT)");
  latSetTorque(false);
  delay(200);
  for (int side = 0; side < 2; ++side) {
    latCue();
    latPhase(side == 0 ? "H1_coast_hand_LEFT" : "H1_coast_hand_RIGHT", "reads", false, 0, false, 6.0);
    latCue();
    latSetTorque(true);
    delay(25);
    latPhase(side == 0 ? "H2_hold_hand_LEFT" : "H2_hold_hand_RIGHT", "sparse250ms", true, 0, false, 6.0);
    latSetTorque(false);
    delay(300);
  }
  Serial.println("LAT end");
}


// ---- Step response (LATSTEP) ----------------------------------------------
// One wheel at a time: Moving Speed steps, then back-to-back Present Data reads for a
// fixed window; prints (t since the command went out, speed, position) per read so the
// dead time, lag and overshoot of the wheel + driver can be fitted offline.
struct StepSample { std::uint32_t t_us; std::int16_t speed; std::uint16_t pos; };
StepSample g_step[700];

void stepRun(std::uint8_t bus, int to_raw, int window_ms, const char* tag) {
  HalfDuplexBus& wb = wheel_buses[bus];
  const std::uint8_t id = kLatId[bus];
  std::size_t n = 0U;
  const std::int64_t t0 = esp_timer_get_time();
  wb.send(legjog::frame::buildMovingSpeed(id,
      legjog::frame::wheelRawSpeedToRadPerSec(static_cast<std::int16_t>(to_raw))));
  legjog::frame::DxlFrame reply{};
  const bool acked = wb.drainReply(20000, reply);
  const std::int64_t ack_us = esp_timer_get_time() - t0;
  while (esp_timer_get_time() - t0 < static_cast<std::int64_t>(window_ms) * 1000 && n < 700U) {
    if (!wb.send(legjog::frame::buildReadPresent(id))) continue;
    if (!wb.drainReply(20000, reply)) continue;
    const auto parsed = legjog::frame::parseDxlResponse(reply.bytes.data(), reply.len);
    const auto present = legjog::frame::parsePresentPayload(parsed);
    if (!present.valid) continue;
    g_step[n++] = {static_cast<std::uint32_t>(esp_timer_get_time() - t0), present.speed_raw, present.position_raw};
  }
  char text[120];
  std::snprintf(text, sizeof(text), "STEP tag=%s id=%u to=%d ack_us=%ld acked=%d n=%u", tag, id, to_raw,
                static_cast<long>(ack_us), acked ? 1 : 0, static_cast<unsigned>(n));
  Serial.println(text);
  for (std::size_t i = 0U; i < n; ++i) {
    std::snprintf(text, sizeof(text), "S %lu %d %u", static_cast<unsigned long>(g_step[i].t_us),
                  static_cast<int>(g_step[i].speed), static_cast<unsigned>(g_step[i].pos));
    Serial.println(text);
  }
}

void runStepTest() {
  Serial.println("LAT begin LATSTEP: one wheel at a time, speed steps up to 190 RPM - keep the wheels free");
  latSetTorque(false);
  delay(200);
  for (std::uint8_t bus = 0U; bus < 2U; ++bus) {
    const std::uint8_t id = kLatId[bus];
    LatHist sink_hist;
    timedTransaction(bus, legjog::frame::buildTorqueEnable(id, true), sink_hist);
    delay(25);
    stepRun(bus, 0, 300, "hold0");
    stepRun(bus, 100, 700, "0to100");
    stepRun(bus, 190, 700, "100to190");
    stepRun(bus, -190, 900, "rev190");
    stepRun(bus, 100, 600, "m190to100");
    stepRun(bus, 50, 500, "100to50");
    stepRun(bus, 0, 600, "50to0");
    timedTransaction(bus, legjog::frame::buildTorqueEnable(id, false), sink_hist);
    delay(500);
  }
  Serial.println("LAT end");
}


// ---- Control-table dump (WDUMP): read-only ---------------------------------
void runDump() {
  Serial.println("LAT begin WDUMP: read-only, registers 0x00-0x5F, 4 bytes per read");
  latSetTorque(false);
  delay(100);
  for (std::uint8_t bus = 0U; bus < 2U; ++bus) {
    HalfDuplexBus& wb = wheel_buses[bus];
    for (unsigned addr = 0U; addr < 0x60U; addr += 4U) {
      const std::uint8_t params[2] = {static_cast<std::uint8_t>(addr), 4U};
      wb.send(legjog::frame::detail::buildFrame(kLatId[bus], legjog::frame::kDxlInsRead, params, 2U));
      legjog::frame::DxlFrame reply{};
      char text[96];
      if (wb.drainReply(20000, reply) && reply.len >= 10U) {
        std::snprintf(text, sizeof(text), "DUMP id=%u addr=%02X err=%02X data=%02X %02X %02X %02X", kLatId[bus], addr,
                      reply.bytes[4], reply.bytes[8], reply.bytes[9], reply.bytes[10], reply.bytes[11]);
      } else {
        std::snprintf(text, sizeof(text), "DUMP id=%u addr=%02X NOREPLY", kLatId[bus], addr);
      }
      Serial.println(text);
      delay(2);
    }
  }
  Serial.println("LAT end");
}


// ---- Driver configuration + round-trip time (WCFG / WRTT) -------------------
// WCFG <id> <addr> <value>: one-byte write, 200 ms later a read-back. WRTT: 300 reads per
// wheel, mean/min/max reply time and how many answered; also times one Torque Disable write
// (does it still get an acknowledgement?).
void runCfg(unsigned id, unsigned addr, unsigned value) {
  const std::uint8_t bus = id == 1U ? 0U : 1U;
  HalfDuplexBus& wb = wheel_buses[bus];
  const std::uint8_t wp[2] = {static_cast<std::uint8_t>(addr), static_cast<std::uint8_t>(value)};
  legjog::frame::DxlFrame reply{};
  wb.send(legjog::frame::detail::buildFrame(static_cast<std::uint8_t>(id), legjog::frame::kDxlInsWrite, wp, 2U));
  const bool acked = wb.drainReply(20000, reply);
  delay(250);
  const std::uint8_t rp[2] = {static_cast<std::uint8_t>(addr), 1U};
  wb.send(legjog::frame::detail::buildFrame(static_cast<std::uint8_t>(id), legjog::frame::kDxlInsRead, rp, 2U));
  const bool got = wb.drainReply(20000, reply);
  char text[100];
  std::snprintf(text, sizeof(text), "CFG id=%u addr=%02X wrote=%u ack=%d readback=%s%02X", id, addr, value,
                acked ? 1 : 0, got ? "" : "NOREPLY ", got ? reply.bytes[8] : 0U);
  Serial.println(text);
}

void runRtt() {
  latSetTorque(false);
  delay(100);
  for (std::uint8_t bus = 0U; bus < 2U; ++bus) {
    LatHist h;
    for (int i = 0; i < 300; ++i) {
      timedTransaction(bus, legjog::frame::buildReadPresent(kLatId[bus]), h);
      delayMicroseconds(300);
    }
    LatHist w;
    for (int i = 0; i < 30; ++i) {
      timedTransaction(bus, legjog::frame::buildTorqueEnable(kLatId[bus], false), w);
      delay(2);
    }
    char text[160];
    std::snprintf(text, sizeof(text), "RTT id=%u read n=%lu timeouts=%lu mean_us=%lu max_us=%lu | write-ack n=%lu timeouts=%lu mean_us=%lu",
                  kLatId[bus], static_cast<unsigned long>(h.n), static_cast<unsigned long>(h.timeouts),
                  static_cast<unsigned long>(h.n ? h.sum / h.n : 0U), static_cast<unsigned long>(h.max),
                  static_cast<unsigned long>(w.n), static_cast<unsigned long>(w.timeouts),
                  static_cast<unsigned long>(w.n ? w.sum / w.n : 0U));
    Serial.println(text);
  }
  Serial.println("LAT end");
}


// ---- Revive attempts on a driver that stopped answering (WREV <id>) ----------
bool revProbe(HalfDuplexBus& wb, std::uint8_t id) {
  legjog::frame::DxlFrame reply{};
  wb.send(legjog::frame::buildReadPresent(id));
  return wb.drainReply(15000, reply);
}
void runRevive(unsigned id) {
  const std::uint8_t bus = id == 1U ? 0U : 1U;
  HalfDuplexBus& wb = wheel_buses[bus];
  const std::uint8_t wid = static_cast<std::uint8_t>(id);
  char text[100];
  std::snprintf(text, sizeof(text), "REV id=%u start: answers=%d", id, revProbe(wb, wid) ? 1 : 0);
  Serial.println(text);
  // 1) pad with zero bytes: completes a frame the driver is still waiting for
  legjog::frame::DxlFrame zeros{};
  zeros.len = 16U;
  for (int i = 0; i < 20; ++i) { wb.send(zeros); delayMicroseconds(100); }
  delay(5);
  std::snprintf(text, sizeof(text), "REV id=%u after 320 zero bytes: answers=%d", id, revProbe(wb, wid) ? 1 : 0);
  Serial.println(text);
  // 2) a burst of valid torque-disable frames
  for (int i = 0; i < 10; ++i) { wb.send(legjog::frame::buildTorqueEnable(wid, false)); delay(2); }
  std::snprintf(text, sizeof(text), "REV id=%u after 10 torque-disable frames: answers=%d", id, revProbe(wb, wid) ? 1 : 0);
  Serial.println(text);
  // 3) 0xFF padding: a long header, then valid frames
  legjog::frame::DxlFrame ff{};
  ff.len = 16U;
  for (auto& b : ff.bytes) b = 0xFFU;
  for (int i = 0; i < 4; ++i) { wb.send(ff); delayMicroseconds(100); }
  delay(5);
  std::snprintf(text, sizeof(text), "REV id=%u after 0xFF padding: answers=%d", id, revProbe(wb, wid) ? 1 : 0);
  Serial.println(text);
  Serial.println("LAT end");
}


// ---- Hang hunt (WSOAK <seconds> [mode]) -------------------------------------
// Drives both wheels with random speed targets like the balance loop does (a new target every
// 10-120 ms) while reading Present Data at full rate, and stops the moment one driver stops
// answering. Prints the last commands that reached that wheel, so the trigger can be read off.
// mode 0: both wheels, random targets +-190 RPM incl. frequent sign reversals and zeros
// mode 1: only the right wheel (id 1)   mode 2: only the left wheel (id 2)
// mode 3: both wheels, small targets +-40 RPM (balance-like amplitude)
struct SoakCmd { std::uint32_t t_ms; std::int16_t rpm; };
void runSoak(unsigned seconds, unsigned mode) {
  Serial.println("LAT begin WSOAK: wheels must spin freely");
  latSetTorque(false);
  delay(100);
  const bool use[2] = {mode != 2U, mode != 1U};
  for (std::uint8_t b = 0U; b < 2U; ++b) if (use[b]) {
    LatHist h; timedTransaction(b, legjog::frame::buildTorqueEnable(kLatId[b], true), h);
  }
  delay(25);
  static SoakCmd hist[2][32];
  std::size_t hn[2] = {0U, 0U};
  std::int16_t cur[2] = {0, 0};
  std::int64_t next_change[2] = {0, 0};
  std::uint32_t rng = 12345U ^ static_cast<std::uint32_t>(esp_timer_get_time());
  const auto rnd = [&rng]() { rng = rng * 1664525U + 1013904223U; return rng >> 8; };
  const std::int64_t t0 = esp_timer_get_time();
  const std::int64_t end = t0 + static_cast<std::int64_t>(seconds) * 1000000;
  std::uint32_t miss[2] = {0U, 0U}, writes[2] = {0U, 0U}, reads[2] = {0U, 0U};
  int hung = -1;
  const int range = mode == 3U ? 40 : 190;
  while (esp_timer_get_time() < end && hung < 0) {
    for (std::uint8_t b = 0U; b < 2U && hung < 0; ++b) {
      if (!use[b]) continue;
      const std::int64_t now = esp_timer_get_time();
      if (now >= next_change[b]) {
        next_change[b] = now + 10000 + static_cast<std::int64_t>(rnd() % 110000U);
        const unsigned r = rnd() % 100U;
        cur[b] = r < 15U ? 0 : static_cast<std::int16_t>(static_cast<int>(rnd() % (2U * range + 1U)) - range);
        const std::size_t i = hn[b] % 32U;
        hist[b][i] = {static_cast<std::uint32_t>((now - t0) / 1000), cur[b]};
        ++hn[b];
        LatHist wh;
        timedTransaction(b, legjog::frame::buildMovingSpeed(kLatId[b],
            legjog::frame::wheelRawSpeedToRadPerSec(cur[b])), wh);
        ++writes[b];
      }
      LatHist rh;
      if (timedTransaction(b, legjog::frame::buildReadPresent(kLatId[b]), rh)) { miss[b] = 0U; ++reads[b]; }
      else if (++miss[b] >= 5U) hung = b;
    }
  }
  // stop whatever still listens
  for (std::uint8_t b = 0U; b < 2U; ++b) if (use[b]) {
    LatHist h; timedTransaction(b, legjog::frame::buildMovingSpeed(kLatId[b], 0.0F), h);
    delay(30);
    timedTransaction(b, legjog::frame::buildTorqueEnable(kLatId[b], false), h);
  }
  char text[120];
  if (hung >= 0) {
    std::snprintf(text, sizeof(text), "SOAK HUNG id=%u after %lu ms; reads ok=%lu writes=%lu", kLatId[hung],
                  static_cast<unsigned long>((esp_timer_get_time() - t0) / 1000), static_cast<unsigned long>(reads[hung]),
                  static_cast<unsigned long>(writes[hung]));
    Serial.println(text);
    const std::size_t n = hn[hung] < 32U ? hn[hung] : 32U;
    for (std::size_t k = 0U; k < n; ++k) {
      const auto& c = hist[hung][(hn[hung] - n + k) % 32U];
      std::snprintf(text, sizeof(text), "SOAK cmd t=%lu rpm=%d", static_cast<unsigned long>(c.t_ms), static_cast<int>(c.rpm));
      Serial.println(text);
    }
  } else {
    std::snprintf(text, sizeof(text), "SOAK OK %us mode=%u: right reads=%lu writes=%lu | left reads=%lu writes=%lu", seconds, mode,
                  static_cast<unsigned long>(reads[0]), static_cast<unsigned long>(writes[0]),
                  static_cast<unsigned long>(reads[1]), static_cast<unsigned long>(writes[1]));
    Serial.println(text);
  }
  Serial.println("LAT end");
}

char line[128];
std::size_t line_length = 0U;
bool line_overflow = false;
std::uint64_t next_status_us = 0U;
std::uint64_t bus_off_since_us = 0U;

std::uint64_t nowUs() {
  return static_cast<std::uint64_t>(esp_timer_get_time());
}

bool beginTwai() {
  twai_general_config_t general =
      TWAI_GENERAL_CONFIG_DEFAULT(kCanTxGpio, kCanRxGpio, TWAI_MODE_NORMAL);
  general.tx_queue_len = 32U;
  general.rx_queue_len = 64U;
  const twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
  const twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&general, &timing, &filter) != ESP_OK) return false;
  if (twai_start() != ESP_OK) {
    (void)twai_driver_uninstall();
    return false;
  }
  return true;
}

void serviceTwai(std::uint64_t now_us) {
  if (!sink.ready) return;
  twai_status_info_t status{};
  if (twai_get_status_info(&status) != ESP_OK) return;
  if (status.state == TWAI_STATE_BUS_OFF) {
    if (bus_off_since_us == 0U) {
      bus_off_since_us = now_us == 0U ? 1U : now_us;
      Serial.println("EV WARN CAN bus-off");
    } else if (now_us - bus_off_since_us >= kBusOffRecoveryDelayUs) {
      (void)twai_initiate_recovery();
    }
  } else if (status.state == TWAI_STATE_STOPPED && bus_off_since_us != 0U) {
    if (twai_start() == ESP_OK) {
      bus_off_since_us = 0U;
      Serial.println("EV WARN CAN recovered from bus-off");
    }
  }
}

void drainCan(std::uint64_t now_us) {
  if (!sink.ready) return;
  twai_message_t message{};
  unsigned count = 0U;
  while (count < 64U && twai_receive(&message, 0U) == ESP_OK) {
    ++count;
    if (message.extd != 0U || message.rtr != 0U) continue;
    jog.onCan(static_cast<std::uint16_t>(message.identifier),
              message.data_length_code, message.data, now_us);
  }
}

void drainUarts(std::uint64_t now_us) {
  // Backstop for anything left over between ticks; sendUart() already
  // drains each command's own reply synchronously (see HalfDuplexBus).
  for (std::uint8_t bus = 0U; bus < legjog::kWheelCount; ++bus) {
    legjog::frame::DxlFrame frame{};
    if (wheel_buses[bus].pollReply(frame)) jog.onUart(bus, frame, now_us);
  }
}

void readSerial(std::uint64_t now_us) {
  while (Serial.available() > 0) {
    const int value = Serial.read();
    if (value < 0) break;
    const char c = static_cast<char>(value);
    if (c == '\n' || c == '\r') {
      if (line_length > 0U && !line_overflow) {
        line[line_length] = '\0';
        if (std::strcmp(line, "LATTEST") == 0) { runLatencyTest(); line_length = 0U; line_overflow = false; continue; }
        if (std::strcmp(line, "LATSTEP") == 0) { runStepTest(); line_length = 0U; line_overflow = false; continue; }
        if (std::strcmp(line, "WDUMP") == 0) { runDump(); line_length = 0U; line_overflow = false; continue; }
        if (std::strcmp(line, "WRTT") == 0) { runRtt(); line_length = 0U; line_overflow = false; continue; }
        { unsigned cid = 0U, caddr = 0U, cval = 0U;
          if (std::sscanf(line, "WCFG %u %u %u", &cid, &caddr, &cval) == 3) {
            if ((cid == 1U || cid == 2U) && caddr < 0x30U && cval < 256U) runCfg(cid, caddr, cval);
            line_length = 0U; line_overflow = false; continue; } }
        { unsigned rid = 0U;
          if (std::sscanf(line, "WREV %u", &rid) == 1) { if (rid == 1U || rid == 2U) runRevive(rid); line_length = 0U; line_overflow = false; continue; } }
        { unsigned sk = 0U, sm = 0U;
          const int got = std::sscanf(line, "WSOAK %u %u", &sk, &sm);
          if (got >= 1) { runSoak(sk > 600U ? 600U : sk, got >= 2 ? sm : 0U); line_length = 0U; line_overflow = false; continue; } }
        if (std::strcmp(line, "LATHAND") == 0) { runHandTest(); line_length = 0U; line_overflow = false; continue; }
        char reply[128];
        jog.onLine(line, now_us, reply, sizeof(reply));
        if (reply[0] != '\0') Serial.println(reply);
      }
      line_length = 0U;
      line_overflow = false;
    } else if (line_length + 1U < sizeof(line)) {
      line[line_length++] = c;
    } else {
      line_overflow = true;  // Drop the whole over-long line.
    }
  }
}

}  // namespace

void setup() {
  // Status lines must never block the 100 Hz command stream.
  Serial.setTxBufferSize(4096U);
  Serial.begin(115200);
  delay(200);
  wheel_buses[0].begin(UART_NUM_1, 17, U1TXD_OUT_IDX);  // right wheel, id 1.
  wheel_buses[1].begin(UART_NUM_2, 15, U2TXD_OUT_IDX);  // left wheel, id 2.
  sink.jog_onUart_hook = onUartReply;
  sink.ready = beginTwai();
  Serial.println(sink.ready
      ? "EV BOOT leg-jog DRAFT: CAN 500k up, leg axes IDLE, wheels torque-off; ENABLE to engage legs"
      : "EV ERR BOOT leg-jog: TWAI init failed");
}

void loop() {
  const std::uint64_t now_us = nowUs();
  serviceTwai(now_us);
  drainCan(now_us);
  drainUarts(now_us);
  readSerial(now_us);
  jog.tick(now_us);

  char text[128];
  while (jog.popEvent(text, sizeof(text))) Serial.println(text);
  if (now_us >= next_status_us) {
    next_status_us = now_us + kStatusPeriodUs;
    char status[512];
    jog.formatStatus(now_us, status, sizeof(status));
    Serial.println(status);
  }
}
