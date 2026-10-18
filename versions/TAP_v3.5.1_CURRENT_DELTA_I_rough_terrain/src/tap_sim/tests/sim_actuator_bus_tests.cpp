#include <cmath>
#include <iostream>

#include "../drivers/SimActuatorBus.hpp"

namespace {

int checks = 0;
int failures = 0;

void check(bool condition, const char* message) {
  ++checks;
  if (!condition) {
    ++failures;
    std::cerr << "FAIL " << message << '\n';
  }
}

}  // namespace

int main() {
  using namespace robot_control;
  using namespace robot_control::motor_frame;

  SimMotorTransport bus;

  check(bus.sendCan(canId(3, kSetControllerModeCmdId),
                    packSetControllerMode(kControlModePosition, kInputModeMit)),
        "CAN mode command reaches simulated GDS68");
  check(bus.sendCan(canId(3, kSetAxisStateCmdId),
                    packSetAxisState(kAxisStateClosedLoop)),
        "CAN axis-state command reaches simulated GDS68");
  check(bus.leg(0).closedLoop(), "left hip GDS68 enters closed-loop from verified node ID");
  check(bus.sendCan(canId(1, kSetControllerModeCmdId),
                    packSetControllerMode(kControlModePosition, kInputModeMit)) &&
            bus.sendCan(canId(1, kSetAxisStateCmdId),
                        packSetAxisState(kAxisStateClosedLoop)) &&
            bus.leg(2).closedLoop(),
        "right hip GDS68 is addressed by verified node ID");

  check(bus.publishLegFeedback(0, 1.25F, -2.0F, 0.75F),
        "simulated GDS68 publishes a CAN feedback frame");
  motor_transport::CanFrame can_feedback;
  check(bus.receiveCan(can_feedback), "CAN feedback is readable by the host");
  check(can_feedback.id == canId(3, kMitControlCmdId) && can_feedback.dlc == 8,
        "CAN feedback uses the real node/command ID and DLC");
  const auto feedback = unpackMitFeedback(can_feedback.data);
  check(feedback.node_id == 3 && std::abs(feedback.pos_rad - 1.25F) < 0.01F &&
            std::abs(feedback.vel_rad_s + 2.0F) < 0.05F &&
            std::abs(feedback.torque_nm - 0.75F) < 0.05F,
        "CAN feedback payload round-trips through the wire codec");

  bus.setWheelMeasurement(0, 1.25F, 2.0F);
  check(bus.sendUart(0, buildReadPresent(1)),
        "UART present-data read reaches simulated BL4820M12");
  DxlFrame response;
  check(bus.receiveUart(0, response), "UART response is readable by the host");
  const auto parsed = parseDxlResponse(response.bytes.data(), response.len);
  const auto present = parsePresentPayload(parsed);
  check(parsed.ok && parsed.id == 1 && present.valid,
        "UART response passes checksum and quirk parsing");
  check(std::abs(wheelRawPositionToRad(present.position_raw) -
                 wheelRawPositionToRad(wheelRadToRawPosition(1.25F))) < 1.0e-5F &&
            present.speed_raw == wheelRadPerSecToRawSpeed(2.0F),
        "UART present position/speed match the simulated measurement");

  check(bus.sendUart(0, buildTorqueEnable(1, true)),
        "UART torque-enable write reaches the wheel driver");
  check(!bus.sendUart(0, buildMovingSpeed(1, 4.0F)),
        "wheel driver drops speed write during its settle window");
  bus.wheel(0).step(0.021F);
  check(bus.sendUart(0, buildMovingSpeed(1, 4.0F)) &&
            std::abs(bus.wheel(0).targetSpeedRadS() -
                     wheelRawSpeedToRadPerSec(wheelRadPerSecToRawSpeed(4.0F))) <
                1.0e-5F,
        "wheel driver accepts the same speed write after settling");

  std::cout << checks - failures << '/' << checks
            << " simulated motor transport checks passed\n";
  return failures == 0 ? 0 : 1;
}
