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

  // A command stamped later than the reader's `now` (USB console parses the
  // line after the control tick sampled its time) is fresh, not timed out.
  {
    tap_fw::ble::CommandEndpoint late;
    const char enable_text[] = "ENABLE";
    assert(late.onWrite(reinterpret_cast<const std::uint8_t*>(enable_text),
                        std::strlen(enable_text), 5'000'100U) ==
           tap_fw::ble::WriteResult::Accepted);
    assert(!late.timedOut(5'000'000U));
    const auto early = late.command(5'000'000U);
    assert(early.request_enable && !early.request_disable);
    assert(late.timedOut(5'000'100U + late.config().heartbeat_timeout_us + 1U));
  }

  tap_fw::ble::CommandEndpoint gamepad_endpoint;
  tap_fw::ble::DabbleGamepadMapper mapper(gamepad_endpoint);
  tap_fw::ble::GamepadState gamepad;
  gamepad.start = true;
  gamepad.up = true;
  mapper.update(gamepad, 2'000'000);
  command = gamepad_endpoint.command(2'000'001);
  assert(command.request_enable);
  assert(command.request_fixture_release);  // Default: Start goes to balance.
  assert(command.target_forward_velocity_m_s == 0.25F);

  // First robot run: Start enables but leaves the fixture hold in place.
  {
    tap_fw::ble::CommandEndpoint hold_endpoint;
    tap_fw::ble::DabbleGamepadMapper::Config hold_config{};
    hold_config.auto_fixture_release = false;
    tap_fw::ble::DabbleGamepadMapper hold_mapper(hold_endpoint, hold_config);
    tap_fw::ble::GamepadState press;
    press.start = true;
    hold_mapper.update(press, 2'000'000);
    const auto hold = hold_endpoint.command(2'000'001);
    assert(hold.request_enable);
    assert(!hold.request_fixture_release);
    // Holding Start does not release; a second press does.
    hold_mapper.update(press, 2'100'000);
    assert(!hold_endpoint.command(2'100'001).request_fixture_release);
    press.start = false;
    hold_mapper.update(press, 2'200'000);
    press.start = true;
    hold_mapper.update(press, 2'300'000);
    const auto released = hold_endpoint.command(2'300'001);
    assert(released.request_enable);
    assert(released.request_fixture_release);
    // After DISABLE the next Start only enables again (back to FixtureHold).
    press.start = false;
    press.select = true;
    hold_mapper.update(press, 2'400'000);
    press.select = false;
    hold_mapper.update(press, 2'500'000);
    press.start = true;
    hold_mapper.update(press, 2'600'000);
    const auto again = hold_endpoint.command(2'600'001);
    assert(again.request_enable);
    assert(!again.request_fixture_release);
    // A lapsed lease also restarts from ENABLE.
    press.start = false;
    hold_mapper.update(press, 2'700'000);
    press.start = true;
    hold_mapper.update(press, 4'000'000);
    assert(!hold_endpoint.command(4'000'001).request_fixture_release);
  }
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
