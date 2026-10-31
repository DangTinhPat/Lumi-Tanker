#pragma once
#ifdef ARDUINO
#include "tap_hw/BlMotor.hpp"
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace tap_hw::hybrid {
class EspMotorCritical final : public MotorCritical {
 public:
  void enter() noexcept override { portENTER_CRITICAL(&mux_); }
  void leave() noexcept override { portEXIT_CRITICAL(&mux_); }
 private:
  portMUX_TYPE mux_=portMUX_INITIALIZER_UNLOCKED;
};
class EspMotorBus final : public MotorBus {
 public:
  bool begin(uart_port_t uart, gpio_num_t data, gpio_num_t unused_rx) noexcept;
  std::uint64_t now() const noexcept override;
  BusError lastError() const noexcept override { return error_; }
  BusTrace trace() const noexcept override { return trace_; }
  bool request(BusFormat, std::uint8_t, std::uint8_t, const std::uint8_t*,
               std::size_t, std::size_t, BusReply&) noexcept override;
 private:
  bool readExact(std::uint8_t*, std::size_t, std::uint64_t deadline, std::uint8_t& received) noexcept;
  void release() noexcept;
  uart_port_t uart_=UART_NUM_1;
  gpio_num_t pin_=GPIO_NUM_17;
  std::uint32_t tx_signal_=0;
  bool initialized_=false;
  BusError error_=BusError::None;
  BusTrace trace_{};
  portMUX_TYPE tx_mux_=portMUX_INITIALIZER_UNLOCKED;
};
class EspBlMotor {
 public:
  EspBlMotor() : backend(lock), left(backend,0,left_bus), right(backend,1,right_bus) {}
  bool begin() noexcept;
  EspMotorCritical lock;
  BlMotorBackend backend;
 private:
  static void workerTask(void* arg);
  EspMotorBus left_bus, right_bus;
  BlMotorWorker left, right;
  TaskHandle_t left_task=nullptr, right_task=nullptr;
};
} // namespace tap_hw::hybrid
#endif
