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
#include <soc/gpio_sig_map.h>
#include <soc/gpio_struct.h>

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
    drive();
    const int written = uart_write_bytes(
        port_, reinterpret_cast<const char*>(frame.bytes.data()), frame.len);
    uart_wait_tx_done(port_, pdMS_TO_TICKS(50));
    release();
    echo_deadline_us_ = esp_timer_get_time() + kEchoMarginUs;
    return written == static_cast<int>(frame.len);
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
