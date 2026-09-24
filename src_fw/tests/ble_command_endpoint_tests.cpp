#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "tap_fw/BleCommandEndpoint.hpp"
#include "tap_fw/DabbleGamepadMapper.hpp"

int main() {
  tap_fw::ble::CommandEndpoint endpoint;
  const auto initial = endpoint.command(0);
  assert(!initial.request_disable);

  const char enable[] = "ENABLE";
  assert(endpoint.onWrite(reinterpret_cast<const std::uint8_t*>(enable),
                          std::strlen(enable), 0) ==
         tap_fw::ble::WriteResult::Accepted);
  auto command = endpoint.command(100'000);
  assert(command.request_enable && !command.request_disable);

  const char drive[] = "DRIVE 0.5 1.0";
  assert(endpoint.onWrite(reinterpret_cast<const std::uint8_t*>(drive),
                          std::strlen(drive), 100'000) ==
         tap_fw::ble::WriteResult::Accepted);
  command = endpoint.command(200'000);
  assert(command.target_forward_velocity_m_s == 0.25F);
  assert(command.target_yaw_rate_rad_s == 0.80F);

  command = endpoint.command(700'001);
  assert(command.request_disable);
  assert(command.target_forward_velocity_m_s == 0.0F);

  const char estop[] = "ESTOP";
  assert(endpoint.onWrite(reinterpret_cast<const std::uint8_t*>(estop),
                          std::strlen(estop), 800'000) ==
         tap_fw::ble::WriteResult::Accepted);
  command = endpoint.command(800'001);
  assert(command.request_emergency_stop && command.request_disable);

  const char clear[] = "CLEAR_ESTOP";
  assert(endpoint.onWrite(reinterpret_cast<const std::uint8_t*>(clear),
                          std::strlen(clear), 900'000) ==
         tap_fw::ble::WriteResult::Accepted);
  const char reset[] = "RESET";
  assert(endpoint.onWrite(reinterpret_cast<const std::uint8_t*>(reset),
                          std::strlen(reset), 1'000'000) ==
         tap_fw::ble::WriteResult::ResetRequested);

  tap_fw::ble::CommandEndpoint gamepad_endpoint;
  tap_fw::ble::DabbleGamepadMapper mapper(gamepad_endpoint);
  tap_fw::ble::GamepadState gamepad;
  gamepad.start = true;
  gamepad.up = true;
  mapper.update(gamepad, 2'000'000);
  command = gamepad_endpoint.command(2'000'001);
  assert(command.request_enable);
  assert(command.target_forward_velocity_m_s == 0.25F);
  gamepad.start = false;
  gamepad.up = false;
  mapper.update(gamepad, 2'100'000);
  command = gamepad_endpoint.command(2'100'001);
  assert(command.target_forward_velocity_m_s == 0.0F);
  gamepad.triangle = true;
  mapper.update(gamepad, 2'200'000);
  command = gamepad_endpoint.command(2'200'001);
  assert(command.request_height_change);
  assert(command.target_body_height_m == 0.245F);
  assert(command.target_forward_velocity_m_s == 0.0F);
  gamepad.triangle = false;
  mapper.update(gamepad, 2'300'000);
  gamepad.circle = true;
  mapper.update(gamepad, 2'400'000);
  command = gamepad_endpoint.command(2'400'001);
  assert(command.target_body_height_m == 0.140F);
  gamepad.circle = false;
  gamepad.cross = true;
  mapper.update(gamepad, 2'500'000);
  command = gamepad_endpoint.command(2'500'001);
  assert(std::fabs(command.target_body_height_m -
                   robot_control::geometry::neutral_spawn_z_m) < 1.0e-5F);
  gamepad.cross = false;
  gamepad.square = true;
  mapper.update(gamepad, 2'600'000);
  command = gamepad_endpoint.command(2'600'001);
  assert(command.request_emergency_stop && command.request_disable);
  return 0;
}
