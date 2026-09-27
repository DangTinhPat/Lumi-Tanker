#pragma once

#include "robot_hal/MotorTransport.hpp"

namespace tap_fw {

namespace motor_frame = robot_control::motor_frame;
namespace motor_transport = robot_control::motor_transport;

// Small ABI-stable seam for ESP-IDF TWAI/UART code. The firmware application
// owns the driver handles and supplies these callbacks; this layer owns no
// heap, queue, task, or ESP-IDF object.
struct CanPort {
  using Send = bool (*)(void*, const motor_transport::CanFrame&) noexcept;
  using Receive = bool (*)(void*, motor_transport::CanFrame&) noexcept;

  void* context = nullptr;
  Send send = nullptr;
  Receive receive = nullptr;
};

struct UartPort {
  using Send = bool (*)(void*, const motor_frame::DxlFrame&) noexcept;
  using Receive = bool (*)(void*, motor_frame::DxlFrame&) noexcept;

  void* context = nullptr;
  Send send = nullptr;
  Receive receive = nullptr;
};

// Implements the existing robot_hal Transport with one TWAI port and two
// independent full-duplex UART ports. ESP-IDF adapter code only needs to
// translate twai_message_t/uart bytes into these callbacks.
class CallbackBusTransport final : public motor_transport::Transport {
 public:
  CallbackBusTransport(CanPort can, UartPort uart1, UartPort uart2) noexcept
      : can_(can), uarts_{uart1, uart2} {}

  bool sendCan(std::uint16_t id, const motor_frame::MitBytes& data) noexcept override;
  bool sendCanFrame(const motor_transport::CanFrame& frame) noexcept override;
  bool receiveCan(motor_transport::CanFrame& frame) noexcept override;
  bool sendUart(std::uint8_t bus, const motor_frame::DxlFrame& frame) noexcept override;
  bool receiveUart(std::uint8_t bus, motor_frame::DxlFrame& frame) noexcept override;
  bool isOpen() const noexcept override;
  void close() noexcept override;

 private:
  CanPort can_{};
  UartPort uarts_[2]{};
  bool open_ = true;
};

}  // namespace tap_fw
