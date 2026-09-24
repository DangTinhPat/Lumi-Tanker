#include "tap_fw/BusTransport.hpp"

namespace tap_fw {

namespace motor_frame = robot_control::motor_frame;
namespace motor_transport = robot_control::motor_transport;

bool CallbackBusTransport::sendCan(
    std::uint16_t id, const motor_frame::MitBytes& data) noexcept {
  if (!open_ || can_.send == nullptr) return false;
  motor_transport::CanFrame frame;
  frame.id = id;
  frame.dlc = static_cast<std::uint8_t>(data.size());
  frame.data = data;
  return can_.send(can_.context, frame);
}

bool CallbackBusTransport::receiveCan(motor_transport::CanFrame& frame) noexcept {
  return open_ && can_.receive != nullptr && can_.receive(can_.context, frame);
}

bool CallbackBusTransport::sendUart(
    std::uint8_t bus, const motor_frame::DxlFrame& frame) noexcept {
  return open_ && bus < 2U && uarts_[bus].send != nullptr &&
      uarts_[bus].send(uarts_[bus].context, frame);
}

bool CallbackBusTransport::receiveUart(
    std::uint8_t bus, motor_frame::DxlFrame& frame) noexcept {
  return open_ && bus < 2U && uarts_[bus].receive != nullptr &&
      uarts_[bus].receive(uarts_[bus].context, frame);
}

bool CallbackBusTransport::isOpen() const noexcept {
  return open_;
}

void CallbackBusTransport::close() noexcept {
  open_ = false;
}

}  // namespace tap_fw
