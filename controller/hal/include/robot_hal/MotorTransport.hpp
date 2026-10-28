#pragma once

#include <cstdint>

#include "robot_hal/MotorFrameCodec.hpp"

namespace robot_control::motor_transport {

struct CanFrame {
  std::uint16_t id = 0;
  std::uint8_t dlc = 0;
  motor_frame::MitBytes data{};
};

// The only boundary between motor-frame logic and a physical or simulated
// bus. It deliberately moves complete wire frames, not MotorCommand structs.
class Transport {
 public:
  virtual ~Transport() = default;

  virtual bool sendCan(std::uint16_t id,
                       const motor_frame::MitBytes& data) noexcept = 0;
  // A frame with its own DLC (e.g. a DLC-0 CAN Simple getter request).
  // Transports that only carry 8-byte frames fall back to sendCan().
  virtual bool sendCanFrame(const CanFrame& frame) noexcept {
    return sendCan(frame.id, frame.data);
  }
  virtual bool receiveCan(CanFrame& frame) noexcept = 0;
  virtual bool sendUart(std::uint8_t bus,
                        const motor_frame::DxlFrame& frame) noexcept = 0;
  virtual bool receiveUart(std::uint8_t bus,
                           motor_frame::DxlFrame& frame) noexcept = 0;
  virtual bool isOpen() const noexcept = 0;
  virtual void close() noexcept = 0;
};

}  // namespace robot_control::motor_transport
