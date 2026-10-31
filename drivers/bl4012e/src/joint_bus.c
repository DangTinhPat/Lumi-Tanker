/**
 * @file joint_bus.c
 * @brief Frame building, echo check and reply parsing for the single-wire bus.
 */
#include "joint_bus.h"

#include <stdbool.h>
#include <string.h>

#include "esp_check.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "hal/gpio_ll.h"
#include "hal/uart_ll.h"
#include "soc/gpio_sig_map.h"
#include "soc/uart_periph.h"

static const char *TAG = "joint_bus";

#define BROADCAST_ID 0xFE
#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03

#define MAX_FRAME 48
#define RX_RING 512
// Upper bound for one frame on the wire (MAX_FRAME bytes at 1 Mbps = 480 us).
#define TX_HOLD_MAX_US 1000
// Reply window after our last byte. A joint answers ~500 us later (register
// 0x05); 3 ticks of 1 ms give it 2-3 ms (2 ticks could be barely 1 ms). A free
// BL4012 answers in ~1.3 ms but a held or loaded one much later; Lumi-Tanker
// waits 8 ms for it, 9 ticks give 8-9 ms.
// The echo of our own frame always uses the joint window.
#define RX_TIMEOUT_JOINT pdMS_TO_TICKS(3)
#define RX_TIMEOUT_WHEEL pdMS_TO_TICKS(9)
// Idle time before we drive DATA again. Without it the frame that follows a
// reply was sometimes garbled at its start (echo "23 74 C2 01 2F 06 ..." for
// "FF FF 02 0D 03 1A 10 27 78 00 2F 06 ..."): the motor that had just answered
// had not released the line yet. 100 us = 10 byte times at 1 Mbps.
#define TURNAROUND_US 100
// After a failed exchange, wait until the line has been silent for one tick
// (at most this many ticks) so a late reply cannot collide with our next frame.
#define DRAIN_MAX_TICKS 10

static uint8_t checksum(const uint8_t *p, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum += p[i];
  }
  return (uint8_t)~sum;
}

static void bus_drive(joint_bus_t *bus) {
  esp_rom_gpio_connect_out_signal(bus->pin, bus->tx_signal, false, false);
}

static void bus_release(joint_bus_t *bus) {
  esp_rom_gpio_connect_out_signal(bus->pin, SIG_GPIO_OUT_IDX, false, false);
  gpio_ll_output_disable(&GPIO, bus->pin);
}

esp_err_t joint_bus_init(joint_bus_t *bus, uart_port_t uart, gpio_num_t pin, int baud) {
  memset(bus, 0, sizeof(*bus));
  bus->uart = uart;
  bus->pin = pin;
  bus->tx_signal = UART_PERIPH_SIGNAL(uart, SOC_UART_TX_PIN_IDX);

  const uart_config_t cfg = {
      .baud_rate = baud,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_RETURN_ON_ERROR(uart_driver_install(uart, RX_RING, 0, 0, NULL, 0), TAG, "driver install");
  ESP_RETURN_ON_ERROR(uart_param_config(uart, &cfg), TAG, "param config");
  ESP_RETURN_ON_ERROR(uart_set_pin(uart, pin, pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG,
                      "set pin");
  // gpio_set_direction() detaches UART TX from the pad; bus_drive() reattaches
  // it for every frame and bus_release() lets the motor answer.
  ESP_RETURN_ON_ERROR(gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT), TAG, "direction");
  ESP_RETURN_ON_ERROR(gpio_set_level(pin, 1), TAG, "level");
  ESP_RETURN_ON_ERROR(gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY), TAG, "pull");
  bus_release(bus);
  ESP_RETURN_ON_ERROR(uart_set_rx_timeout(uart, 2), TAG, "rx timeout");
  return ESP_OK;
}

// Called outside the TX critical section. Cancellation never renews a deadline.
static bool permitted(joint_bus_t *bus) {
  if (bus->last_error == JB_ERROR_DEADLINE || bus->last_error == JB_ERROR_CANCELLED) return false;
  if (bus->deadline_us && esp_timer_get_time() >= bus->deadline_us) {
    bus->last_error = JB_ERROR_DEADLINE;
    return false;
  }
  if (bus->permit && !bus->permit(bus->permit_context)) {
    bus->last_error = JB_ERROR_CANCELLED;
    return false;
  }
  return true;
}

static int read_bounded(joint_bus_t *bus, uint8_t *dst, size_t n, TickType_t ticks) {
  if (!permitted(bus)) return -1;
  if (bus->deadline_us) {
    const int64_t remaining = bus->deadline_us - esp_timer_get_time();
    const TickType_t available = remaining > 0 ? remaining / (portTICK_PERIOD_MS * 1000) : 0;
    if (!available && ticks) { bus->last_error = JB_ERROR_DEADLINE; return -1; }
    if (ticks > available) ticks = available;
  }
  return uart_read_bytes(bus->uart, dst, n, ticks);
}

static portMUX_TYPE s_wire_lock = portMUX_INITIALIZER_UNLOCKED;

// Puts the frame straight into the TX FIFO and releases DATA the moment the
// last stop bit is out. Waiting for the driver's TX-done wakeup held DATA for
// 180-309 us after the frame in Lumi-Tanker's logs: too late for the BL4012,
// which starts answering earlier than that, so its reply lost its first bytes.
// The spin lasts one frame time (<= 480 us) with interrupts off on this core.
static bool send_frame(joint_bus_t *bus, const uint8_t *tx, size_t len) {
  uart_dev_t *hw = UART_LL_GET_HW(bus->uart);
  if (!permitted(bus)) return false;
  taskENTER_CRITICAL(&s_wire_lock);
  const int64_t start = esp_timer_get_time();
  const int64_t end = bus->deadline_us && bus->deadline_us < start + TX_HOLD_MAX_US ?
      bus->deadline_us : start + TX_HOLD_MAX_US;
  bool ok = end > start && uart_ll_is_tx_idle(hw) && uart_ll_get_txfifo_len(hw) >= len;
  if (bus->deadline_us && bus->deadline_us-start < (int64_t)len*10+50) {
    bus->last_error = JB_ERROR_DEADLINE; ok = false;
  }
  if (ok) {
    bus_drive(bus);
    uart_ll_write_txfifo(hw, tx, len);
    while (!uart_ll_is_tx_idle(hw) && esp_timer_get_time() < end) {
    }
    ok = uart_ll_is_tx_idle(hw);
  }
  bus_release(bus);
  if (!ok) {
    uart_ll_txfifo_rst(hw);
  }
  taskEXIT_CRITICAL(&s_wire_lock);
  return ok;
}

static void capture(joint_bus_t *bus, char kind, const uint8_t *tx, size_t tx_len,
                    const uint8_t *rx, int rx_len) {
  jb_capture_t *c = &bus->last_bad;
  c->kind = kind;
  c->tx_len = tx_len < JB_CAPTURE_LEN ? tx_len : JB_CAPTURE_LEN;
  c->rx_len = rx_len <= 0 ? 0 : (rx_len < JB_CAPTURE_LEN ? rx_len : JB_CAPTURE_LEN);
  memcpy(c->tx, tx, c->tx_len);
  memcpy(c->rx, rx, c->rx_len);
  c->at_us = esp_timer_get_time();
}

// Bytes found here belong to a reply that came after its window closed.
static bool wait_quiet(joint_bus_t *bus) {
  uint8_t junk[64];
  bool saw_late = false;
  for (int i = 0; i < DRAIN_MAX_TICKS; i++) {
    const int n = read_bounded(bus, junk, sizeof(junk), 1);
    if (n > 0) {
      bus->idle_since_us = esp_timer_get_time(); // Gap also follows drained late bytes.
      if (!saw_late) { ++bus->err_late; saw_late = true; }
    }
    if (!permitted(bus)) return false;
    if (n == 0) { bus->drain = false; return true; }
    if (n < 0) break;
  }
  // Do not start a new frame unless silence was actually observed.
  bus->last_error = JB_ERROR_QUIET;
  ++bus->err_quiet;
  return false;
}

// Send one frame, check that the bus carried it intact (our echo), then read
// exactly rx_len reply bytes. rx_len == 0 means no reply is expected.
static esp_err_t exchange(joint_bus_t *bus, const uint8_t *tx, size_t tx_len, uint8_t *rx,
                          size_t rx_len, TickType_t rx_timeout, int64_t started) {
  uint8_t echo[MAX_FRAME];

  if (bus->drain) {
    if (!wait_quiet(bus)) return ESP_ERR_TIMEOUT;
  }
  const int64_t wait_us = bus->idle_since_us + TURNAROUND_US - esp_timer_get_time();
  if (wait_us > 0) {
    esp_rom_delay_us((uint32_t)wait_us);
  }
  if (!permitted(bus)) return ESP_ERR_TIMEOUT;
  uart_flush_input(bus->uart);
  if (!send_frame(bus, tx, tx_len)) {
    if (permitted(bus)) bus->last_error = JB_ERROR_TX;
    bus->err_timeout++;
    bus->drain = true;
    return ESP_ERR_TIMEOUT;
  }

  bus->trace.tx_us = (uint32_t)(esp_timer_get_time() - started);
  const int echoed = read_bounded(bus, echo, tx_len, RX_TIMEOUT_JOINT);
  bus->trace.echo_count = echoed > 0 ? echoed : 0;
  memcpy(bus->trace.echo, echo, bus->trace.echo_count);
  bus->trace.echo_us = (uint32_t)(esp_timer_get_time() - started);
  if (!permitted(bus)) return ESP_ERR_TIMEOUT;
  if (echoed != (int)tx_len || memcmp(echo, tx, tx_len) != 0) {
    bus->last_error = echoed != (int)tx_len ? JB_ERROR_ECHO_TIMEOUT : JB_ERROR_ECHO_MISMATCH;
    bus->err_echo++;
    capture(bus, 'E', tx, tx_len, echo, echoed);
    bus->drain = true;
    return ESP_ERR_INVALID_RESPONSE;
  }
  if (rx_len == 0) {
    return ESP_OK;
  }
  // The reply's first sync byte can be lost when the motor starts answering
  // while DATA is turning around (seen on the BL4012: "FF 01 0B ..."). The
  // sync bytes are outside the checksum, so a reply missing exactly one of them
  // is restored and still validated in full by request().
  int got = read_bounded(bus, rx, rx_len - 1, rx_timeout);
  bus->trace.rx_count = got > 0 ? got : 0;
  memcpy(bus->trace.rx, rx, bus->trace.rx_count);
  if (!permitted(bus)) return ESP_ERR_TIMEOUT;
  if (got == (int)rx_len - 1) {
    if (rx[0] == 0xFF && rx[1] != 0xFF) {
      memmove(rx + 1, rx, rx_len - 1);
      bus->trace.sync_restored = true;
      got = rx_len;
    } else {
      const int tail = read_bounded(bus, rx + rx_len - 1, 1, RX_TIMEOUT_JOINT);
      if (tail > 0) got += tail;
      bus->trace.rx_count = got;
      memcpy(bus->trace.rx, rx, got);
      if (!permitted(bus)) return ESP_ERR_TIMEOUT;
    }
  }
  if (got != (int)rx_len) {
    bus->last_error = JB_ERROR_REPLY_TIMEOUT;
    bus->err_timeout++;
    if (got > 0) {
      bus->err_partial++;  // the reply started but was cut short
      capture(bus, 'P', tx, tx_len, rx, got);
    }
    bus->drain = true;
    return ESP_ERR_TIMEOUT;
  }
  return ESP_OK;
}

static esp_err_t transact(joint_bus_t *bus, const uint8_t *tx, size_t tx_len, uint8_t *rx,
                          size_t rx_len, TickType_t rx_timeout) {
  const int64_t started = esp_timer_get_time();
  bus->trace.tx_count = tx_len; bus->trace.expected = rx_len;
  memcpy(bus->trace.tx, tx, tx_len);
  const esp_err_t err = exchange(bus, tx, tx_len, rx, rx_len, rx_timeout, started);
  bus->idle_since_us = esp_timer_get_time();
  bus->trace.total_us = (uint32_t)(bus->idle_since_us - started);
  if (bus->last_error == JB_ERROR_DEADLINE || bus->last_error == JB_ERROR_CANCELLED) {
    bus->drain = true; ++bus->aborted;
  }
  return err;
}

static esp_err_t request(joint_bus_t *bus, jb_format_t fmt, uint8_t id, uint8_t inst,
                         const uint8_t *params, uint8_t n_params, uint8_t *data, uint8_t data_len,
                         joint_reply_t *reply) {
  bus->last_error = JB_ERROR_NONE;
  memset(&bus->trace, 0, sizeof(bus->trace));
  uint8_t tx[MAX_FRAME];
  uint8_t rx[MAX_FRAME];
  const bool wheel = fmt == JB_FMT_WHEEL;
  const size_t echo_at = wheel ? 5 : 7;  // PRE_LEN PRE_INST PRE_P1
  const size_t tx_len = (size_t)n_params + 6;
  const size_t rx_len = id == BROADCAST_ID ? 0 : (size_t)data_len + echo_at + 4;
  if (tx_len > sizeof(tx) || rx_len > sizeof(rx)) {
    bus->last_error = JB_ERROR_PACKET;
    return ESP_ERR_INVALID_SIZE;
  }

  tx[0] = 0xFF;
  tx[1] = 0xFF;
  tx[2] = id;
  tx[3] = n_params + 2;
  tx[4] = inst;
  if (n_params) {
    memcpy(&tx[5], params, n_params);
  }
  tx[tx_len - 1] = checksum(&tx[2], tx_len - 3);

  esp_err_t err =
      transact(bus, tx, tx_len, rx, rx_len, wheel ? RX_TIMEOUT_WHEEL : RX_TIMEOUT_JOINT);
  if (err != ESP_OK || rx_len == 0) {
    return err;
  }

  const uint8_t pre_p1 = (inst == INST_READ || inst == INST_WRITE) ? params[0] : 0x00;
  if (rx[0] != 0xFF || rx[1] != 0xFF || rx[2] != id || rx[3] != rx_len - 4 ||
      rx[rx_len - 1] != checksum(&rx[2], rx_len - 3) || rx[echo_at] != tx[3] ||
      rx[echo_at + 1] != inst || rx[echo_at + 2] != pre_p1) {
    bus->last_error = JB_ERROR_REPLY_INVALID;
    bus->err_frame++;
    capture(bus, 'F', tx, tx_len, rx, rx_len);
    bus->drain = true;
    return ESP_ERR_INVALID_RESPONSE;
  }

  if (!permitted(bus)) { ++bus->aborted; bus->drain = true; return ESP_ERR_TIMEOUT; }
  if (bus->trace.sync_restored) ++bus->sync_recovered;
  if (reply) {
    reply->status = rx[4];
    reply->pos = wheel ? 0 : (int16_t)(rx[5] | (rx[6] << 8));
  }
  if (data_len) {
    memcpy(data, &rx[echo_at + 3], data_len);
  }
  return ESP_OK;
}

esp_err_t bus_ping(joint_bus_t *bus, jb_format_t fmt, uint8_t id, joint_reply_t *reply) {
  return request(bus, fmt, id, INST_PING, NULL, 0, NULL, 0, reply);
}

esp_err_t bus_write(joint_bus_t *bus, jb_format_t fmt, uint8_t id, uint8_t addr,
                    const uint8_t *data, uint8_t len, joint_reply_t *reply) {
  uint8_t params[MAX_FRAME];
  if ((size_t)len + 1 > sizeof(params)) {
    return ESP_ERR_INVALID_SIZE;
  }
  params[0] = addr;
  memcpy(&params[1], data, len);
  return request(bus, fmt, id, INST_WRITE, params, len + 1, NULL, 0, reply);
}

esp_err_t bus_read(joint_bus_t *bus, jb_format_t fmt, uint8_t id, uint8_t addr, uint8_t len,
                   uint8_t *out, joint_reply_t *reply) {
  if (id == BROADCAST_ID) {
    return ESP_ERR_INVALID_ARG;
  }
  const uint8_t params[2] = {addr, len};
  return request(bus, fmt, id, INST_READ, params, 2, out, len, reply);
}

esp_err_t bus_write_mit(joint_bus_t *bus, uint8_t id, const joint_mit_cmd_t *cmd,
                        joint_reply_t *reply) {
  const uint8_t block[10] = {
      (uint8_t)cmd->kp,  (uint8_t)(cmd->kp >> 8),
      (uint8_t)cmd->kd,  (uint8_t)(cmd->kd >> 8),
      (uint8_t)cmd->pos, (uint8_t)((uint16_t)cmd->pos >> 8),
      (uint8_t)cmd->rpm, (uint8_t)((uint16_t)cmd->rpm >> 8),
      (uint8_t)cmd->tau, (uint8_t)((uint16_t)cmd->tau >> 8),
  };
  return bus_write(bus, JB_FMT_JOINT, id, JB_REG_KP, block, sizeof(block), reply);
}
