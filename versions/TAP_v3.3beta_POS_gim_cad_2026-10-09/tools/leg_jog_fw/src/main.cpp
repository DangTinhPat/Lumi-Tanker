// Bench jog firmware (DRAFT) for the TAP ESP32-S3 board.
//
// Replaces the USB-CAN-A adapter of TEST1/gds68_controller.py: the ESP32
// streams MIT hold/jog frames to all four GDS68 leg joints at 100 Hz, drives
// the two BL4820 wheels over UART1/UART2 (hold-to-run), and takes line
// commands from tools/leg_jog_gui.py over the CP2102 serial port (115200
// baud). Boots with every leg axis IDLE and both wheels torque-off. No IMU.
//
// This is NOT the robot controller firmware. Reflash firmware_esp32 afterwards.

#include <Arduino.h>
#include <driver/twai.h>
#include <esp_timer.h>

#include <cstring>

#include "LegJog.hpp"

namespace {

constexpr gpio_num_t kCanTxGpio = GPIO_NUM_4;
constexpr gpio_num_t kCanRxGpio = GPIO_NUM_5;
constexpr int kUart1TxGpio = 17;  // Left wheel.
constexpr int kUart1RxGpio = 18;
constexpr int kUart2TxGpio = 15;  // Right wheel.
constexpr int kUart2RxGpio = 16;
constexpr std::uint32_t kWheelBaud = 1'000'000U;
constexpr std::uint64_t kStatusPeriodUs = 50'000U;       // 20 Hz to the GUI.
constexpr std::uint64_t kBusOffRecoveryDelayUs = 200'000U;

HardwareSerial* const kWheelPorts[legjog::kWheelCount] = {&Serial1, &Serial2};

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
  bool sendUart(std::uint8_t bus,
                const legjog::frame::DxlFrame& frame) override {
    if (bus >= legjog::kWheelCount || frame.len == 0U ||
        frame.len > frame.bytes.size()) return false;
    return kWheelPorts[bus]->write(frame.bytes.data(), frame.len) == frame.len;
  }
  bool ready = false;
};

BenchSink sink;
legjog::DxlStreamParser wheel_parsers[legjog::kWheelCount];
legjog::LegJog jog(sink);
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
  for (std::uint8_t bus = 0U; bus < legjog::kWheelCount; ++bus) {
    HardwareSerial& port = *kWheelPorts[bus];
    int budget = 64;
    while (budget-- > 0 && port.available() > 0) {
      const int value = port.read();
      if (value < 0) break;
      legjog::frame::DxlFrame frame{};
      if (wheel_parsers[bus].push(static_cast<std::uint8_t>(value), frame)) {
        jog.onUart(bus, frame, now_us);
      }
    }
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
  Serial1.begin(kWheelBaud, SERIAL_8N1, kUart1RxGpio, kUart1TxGpio);
  Serial2.begin(kWheelBaud, SERIAL_8N1, kUart2RxGpio, kUart2TxGpio);
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
