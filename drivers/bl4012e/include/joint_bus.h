/**
 * @file joint_bus.h
 * @brief Single-wire half-duplex UART bus shared by two device families:
 *        - the joint driver ("Robot joint control via 1-wire UART V1.0.0", MIT)
 *        - the Kpower BL4012 wheel (speed control, Dynamixel-like frames)
 *
 * One bus = one UART + one GPIO (TX and RX on the same pad). Every call is a
 * blocking request/response: send, check our own echo, then read the reply.
 * Only one task may use a given bus.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

// Joint driver registers
#define JB_REG_RESPONSE_LEVEL 0x10
#define JB_REG_TORQUE_ENABLE 0x18  // also on the BL4012
#define JB_REG_KP 0x1A             // start of the 10-byte MIT block 0x1A..0x23
#define JB_REG_CONTROL_MODE 0x2D
#define JB_REG_KP_MAX 0x32         // Kp_max (2) + Kd_max (2)
#define JB_MODE_MIT 3
#define JB_COUNTS_PER_REV 16384

// BL4012 wheel registers
#define JB_WHEEL_REG_SPEED 0x20    // int16 RPM, signed
#define JB_WHEEL_REG_PRESENT 0x24  // pos u16 (0..1023/rev), speed s16, load s16

// Status flags that mean the driver cut torque by itself
#define JB_ERR_OVERLOAD 0x20
#define JB_ERR_OVERHEAT 0x04

// Joint reply: FF FF ID LEN STATUS POS_L POS_H PRE_LEN PRE_INST PRE_P1 [data] CS
// Wheel reply: FF FF ID LEN STATUS PRE_LEN PRE_INST PRE_P1 [data] CS
typedef enum { JB_FMT_JOINT, JB_FMT_WHEEL } jb_format_t;

#define JB_CAPTURE_LEN 24

// The last exchange that went wrong, kept byte for byte for diagnosis.
typedef struct {
  char kind;  // 'E' echo differs from what we sent, 'P' reply cut short, 'F' bad reply frame
  uint8_t tx_len, rx_len;
  uint8_t tx[JB_CAPTURE_LEN], rx[JB_CAPTURE_LEN];
  int64_t at_us;
} jb_capture_t;

// HCR additions: per-request deadline/authorization and raw transaction trace.
typedef enum {
  JB_ERROR_NONE, JB_ERROR_PACKET, JB_ERROR_TX, JB_ERROR_ECHO_TIMEOUT,
  JB_ERROR_ECHO_MISMATCH, JB_ERROR_REPLY_TIMEOUT, JB_ERROR_REPLY_INVALID,
  JB_ERROR_DEADLINE, JB_ERROR_CANCELLED, JB_ERROR_QUIET
} jb_error_t;
typedef struct {
  uint32_t tx_us, echo_us, total_us;
  uint8_t tx_count, echo_count, rx_count, expected;
  bool sync_restored;
  uint8_t tx[48], echo[48], rx[48];
} jb_trace_t;

typedef struct {
  uart_port_t uart;
  gpio_num_t pin;
  uint32_t tx_signal;
  uint32_t err_timeout;
  uint32_t err_echo;
  uint32_t err_frame;
  uint32_t err_partial;  // subset of err_timeout: some reply bytes arrived
  uint32_t err_late;     // a reply arrived only after its window had closed
  uint32_t sync_recovered;  // replies accepted after restoring a lost first 0xFF
  bool drain;  // the last exchange failed: a late reply may still be on the wire
  int64_t idle_since_us;  // end of the last exchange (start of the turnaround gap)
  jb_capture_t last_bad;
  // Zero deadline / NULL permit preserves unguarded bench API semantics.
  int64_t deadline_us;
  bool (*permit)(void *);
  void *permit_context;
  jb_error_t last_error;
  jb_trace_t trace;
  uint32_t err_quiet, aborted;
} joint_bus_t;

typedef struct {
  uint8_t status;
  int16_t pos;  // joint only: output-shaft position, 16384 counts/rev
} joint_reply_t;

// Raw MIT block exactly as the joint driver stores it (0x1A..0x23).
typedef struct {
  uint16_t kp;  // 0.001 N·m/rad
  uint16_t kd;  // 0.001 N·m·s/rad
  int16_t pos;  // counts
  int16_t rpm;  // output RPM
  int16_t tau;  // 0.001 N·m
} joint_mit_cmd_t;

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t joint_bus_init(joint_bus_t *bus, uart_port_t uart, gpio_num_t pin, int baud);

esp_err_t bus_ping(joint_bus_t *bus, jb_format_t fmt, uint8_t id, joint_reply_t *reply);
esp_err_t bus_write(joint_bus_t *bus, jb_format_t fmt, uint8_t id, uint8_t addr,
                    const uint8_t *data, uint8_t len, joint_reply_t *reply);
esp_err_t bus_read(joint_bus_t *bus, jb_format_t fmt, uint8_t id, uint8_t addr, uint8_t len,
                   uint8_t *out, joint_reply_t *reply);
esp_err_t bus_write_mit(joint_bus_t *bus, uint8_t id, const joint_mit_cmd_t *cmd,
                        joint_reply_t *reply);

#ifdef __cplusplus
}
#endif
