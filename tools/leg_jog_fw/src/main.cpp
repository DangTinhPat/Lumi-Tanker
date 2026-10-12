// Bench jog firmware (DRAFT) for the TAP ESP32-S3 board.
//
// Replaces the USB-CAN-A adapter of TEST1/gds68_controller.py: the ESP32
// streams MIT hold/jog frames to all four GDS68 leg joints at 100 Hz, drives
// the two EC4012/R1-POC wheels (Dynamixel-like protocol, MotorFrameCodec.hpp)
// over ONE shared half-duplex UART bus (hold-to-run), and takes line commands
// from tools/leg_jog_gui.py over the CP2102 serial port (115200 baud). Boots
// with every leg axis IDLE and both wheels torque-off. No IMU.
//
// Wheel bus: both motors are wired in parallel on GPIO17 (the board's old
// "U1" wheel connector; "U2"/IO15-16 is unused now), told apart by motor id
// (right=1, left=2, MotorMapping.hpp). Single GPIO carries TX and RX
// half-duplex, same technique proven in TEST1/esp32s3_r1poc_bridge: push-pull
// while sending, then release to high-Z (input) so the motor can answer;
// bytes echoed back on our own TX are filtered out before parsing.
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
constexpr uart_port_t kWheelUart = UART_NUM_1;
constexpr int kWheelPin = 17;  // Shared half-duplex bus, both wheels.
constexpr std::uint32_t kWheelBaud = 1'000'000U;
constexpr std::uint64_t kStatusPeriodUs = 50'000U;       // 20 Hz to the GUI.
constexpr std::uint64_t kBusOffRecoveryDelayUs = 200'000U;

// ---- Wheel bus: echo filter (own TX looped back on RX via the shared pin) --
constexpr std::size_t kEchoRing = 64U;  // power of 2; wheel frames are <=16B.
constexpr std::int64_t kEchoMarginUs = 1000;
std::uint8_t s_echo[kEchoRing];
std::uint32_t s_echo_head = 0U, s_echo_tail = 0U;
std::int64_t s_echo_deadline_us = 0;

void wheelEchoPush(const std::uint8_t* d, std::size_t n) {
  for (std::size_t i = 0U; i < n && (s_echo_head - s_echo_tail) < kEchoRing; ++i) {
    s_echo[s_echo_head++ % kEchoRing] = d[i];
  }
  s_echo_deadline_us = esp_timer_get_time() + static_cast<std::int64_t>(n) * 12 + kEchoMarginUs;
}

bool wheelEchoFilter(std::uint8_t b) {
  if (s_echo_tail == s_echo_head) return false;
  if (esp_timer_get_time() > s_echo_deadline_us) { s_echo_tail = s_echo_head; return false; }
  if (s_echo[s_echo_tail % kEchoRing] == b) { ++s_echo_tail; return true; }
  s_echo_tail = s_echo_head;  // lệch -> không nuốt dữ liệu thật của motor
  return false;
}

// Same technique as TEST1/esp32s3_r1poc_bridge: swap the pin's output signal
// source instead of toggling GPIO_ENABLE (the latter was measured unreliable
// at 1 Mbps on this board).
void wheelBusDrive() {
  esp_rom_gpio_connect_out_signal(kWheelPin, U1TXD_OUT_IDX, false, false);
}
void wheelBusRelease() {
  esp_rom_gpio_connect_out_signal(kWheelPin, SIG_GPIO_OUT_IDX, false, false);
  gpio_ll_output_disable(&GPIO, static_cast<gpio_num_t>(kWheelPin));
}

// Forward-declared: defined below, after `jog` exists. Two motors share one
// half-duplex bus, so sendUart() must wait for THIS wheel's reply (or give up
// after a short timeout) before returning - otherwise the next wheel's
// command goes out while the first motor is still replying and collides with
// it on the wire. Measured without this wait: the first wheel sent each tick
// (LW) never got valid feedback and latched a fault; the second (RW), with
// nothing queued right behind it, worked fine. Window is generous (the
// EC4012 turnaround is sub-ms at 1 Mbps) but still small next to the 10 ms
// tick budget.
constexpr std::int64_t kWheelReplyWindowUs = 1500;
void drainWheelReply(std::int64_t window_us);
extern legjog::DxlStreamParser wheel_parser;

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
  // `bus` is unused: both wheels share this one physical port, addressed by
  // the motor id already baked into `frame` (MotorFrameCodec.hpp).
  bool sendUart(std::uint8_t /*bus*/,
                const legjog::frame::DxlFrame& frame) override {
    if (frame.len == 0U || frame.len > frame.bytes.size()) return false;
    // Drop anything left over from the previous command's reply (e.g. one
    // that arrived just after we gave up waiting on it): a stray byte here
    // would shift-corrupt THIS command's response instead.
    uart_flush_input(kWheelUart);
    wheel_parser.reset();
    s_echo_tail = s_echo_head = 0U;
    wheelEchoPush(frame.bytes.data(), frame.len);
    wheelBusDrive();
    const int written = uart_write_bytes(
        kWheelUart, reinterpret_cast<const char*>(frame.bytes.data()), frame.len);
    uart_wait_tx_done(kWheelUart, pdMS_TO_TICKS(50));
    wheelBusRelease();
    s_echo_deadline_us = esp_timer_get_time() + kEchoMarginUs;
    drainWheelReply(kWheelReplyWindowUs);
    return written == static_cast<int>(frame.len);
  }
  bool ready = false;
};

BenchSink sink;
legjog::DxlStreamParser wheel_parser;  // one parser: one shared physical bus.
legjog::LegJog jog(sink);

// Reads and parses bytes off the wheel bus until a complete frame comes
// through (handed to `jog` right away) or `window_us` elapses with nothing
// left to read. Called both right after each TX (see sendUart above) and
// opportunistically from the main loop for anything arriving between ticks.
void drainWheelReply(std::int64_t window_us) {
  const std::int64_t deadline_us = esp_timer_get_time() + window_us;
  std::uint8_t byte = 0U;
  for (;;) {
    const int n = uart_read_bytes(kWheelUart, &byte, 1, 0);
    if (n == 1) {
      if (wheelEchoFilter(byte)) continue;  // our own TX looped back on RX.
      legjog::frame::DxlFrame frame{};
      if (wheel_parser.push(byte, frame)) {
        jog.onUart(0, frame, static_cast<std::uint64_t>(esp_timer_get_time()));
        return;  // one response per command; done early.
      }
      continue;
    }
    if (esp_timer_get_time() >= deadline_us) return;
  }
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

void drainUarts(std::uint64_t /*now_us*/) {
  // Backstop for anything left over between ticks; sendUart() already drains
  // each command's own reply synchronously (see drainWheelReply above).
  drainWheelReply(0);
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

void beginWheelBus() {
  uart_config_t cfg = {};
  cfg.baud_rate = static_cast<int>(kWheelBaud);
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_APB;

  uart_driver_install(kWheelUart, 1024, 0, 0, nullptr, 0);
  uart_param_config(kWheelUart, &cfg);
  uart_set_pin(kWheelUart, kWheelPin, kWheelPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  // uart_set_pin(TX=RX) leaves the pin read-only; gpio_set_direction() then
  // unhooks UART TX from it, so reconnect U1TXD explicitly (see wheelBusDrive).
  gpio_set_direction(static_cast<gpio_num_t>(kWheelPin), GPIO_MODE_INPUT_OUTPUT);
  gpio_set_level(static_cast<gpio_num_t>(kWheelPin), 1);
  esp_rom_gpio_connect_out_signal(kWheelPin, U1TXD_OUT_IDX, false, false);
  wheelBusRelease();  // default: release the bus (high-Z) to listen.
  gpio_pullup_en(static_cast<gpio_num_t>(kWheelPin));

  uart_set_rx_full_threshold(kWheelUart, 96);
  uart_set_rx_timeout(kWheelUart, 2);
}

}  // namespace

void setup() {
  // Status lines must never block the 100 Hz command stream.
  Serial.setTxBufferSize(4096U);
  Serial.begin(115200);
  delay(200);
  beginWheelBus();
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
