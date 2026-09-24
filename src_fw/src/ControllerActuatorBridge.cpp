#include "tap_fw/ControllerActuatorBridge.hpp"

#include <algorithm>
#include <cmath>

namespace tap_fw {

namespace motor_frame = robot_control::motor_frame;
namespace motor_transport = robot_control::motor_transport;
namespace {
constexpr std::array<std::size_t, 4> kLegJoints = {0U, 1U, 3U, 4U};
constexpr std::array<std::size_t, 2> kWheelJoints = {2U, 5U};

float finiteOrZero(float value) noexcept {
  return std::isfinite(value) ? value : 0.0F;
}
}  // namespace

bool ControllerActuatorBridge::isHip(std::size_t joint) noexcept {
  return joint == 0U || joint == 3U;
}

ControllerActuatorBridge::LegStiffness ControllerActuatorBridge::legStiffness(
    float hip_rad, float knee_rad) const noexcept {
  const auto jacobian = robot_control::WholeBodyModel::symmetricComOffsetJacobian(
      hip_rad, knee_rad);
  const auto leg = robot_control::WholeBodyModel::legHeight(hip_rad, knee_rad);
  const float kx = config_.task_stiffness_fore_aft_n_per_m;
  const float kz = config_.task_stiffness_vertical_n_per_m;
  return {
      kx * jacobian.d_com_offset_x_d_hip * jacobian.d_com_offset_x_d_hip +
          kz * leg.d_height_d_hip * leg.d_height_d_hip,
      kx * jacobian.d_com_offset_x_d_knee * jacobian.d_com_offset_x_d_knee +
          kz * leg.d_height_d_knee * leg.d_height_d_knee,
      kx * jacobian.d_com_offset_x_d_hip * jacobian.d_com_offset_x_d_knee +
          kz * leg.d_height_d_hip * leg.d_height_d_knee};
}

bool ControllerActuatorBridge::due(std::uint64_t now_us) const noexcept {
  return !has_sent_ || now_us - last_send_us_ >= config_.command_period_us;
}

bool ControllerActuatorBridge::sendWheelStop(std::size_t wheel) noexcept {
  const std::uint8_t bus = static_cast<std::uint8_t>(wheel);
  return transport_.sendUart(
      bus, motor_frame::buildTorqueEnable(config_.ids.wheel_uart[wheel], false));
}

bool ControllerActuatorBridge::sendWheelCommand(
    std::size_t wheel, float velocity_rad_s) noexcept {
  const std::uint8_t bus = static_cast<std::uint8_t>(wheel);
  const std::uint8_t id = config_.ids.wheel_uart[wheel];
  // Torque Enable is sent once on the disabled->enabled edge. Repeating it
  // here could restart the real driver's 20 ms settle window on every tick.
  return transport_.sendUart(
      bus, motor_frame::buildMovingSpeed(id, finiteOrZero(velocity_rad_s)));
}

bool ControllerActuatorBridge::sendDisabled() noexcept {
  bool ok = true;
  for (std::size_t i = 0; i < config_.ids.leg_can.size(); ++i) {
    ok = transport_.sendCan(
             motor_frame::canId(config_.ids.leg_can[i],
                                motor_frame::kSetAxisStateCmdId),
             motor_frame::packSetAxisState(motor_frame::kAxisStateIdle)) &&
        ok;
  }
  for (std::size_t i = 0; i < config_.ids.wheel_uart.size(); ++i) {
    ok = sendWheelStop(i) && ok;
  }
  return ok;
}

ControllerActuatorBridge::Result ControllerActuatorBridge::apply(
    const robot_control::MotorCommand& command,
    const robot_control::RobotState& state, std::uint64_t now_us) noexcept {
  if (!transport_.isOpen()) return Result::TransportError;
  if (!due(now_us)) return Result::RateLimited;
  last_send_us_ = now_us;
  has_sent_ = true;

  if (!command.enable_motors) {
    const bool ok = sendDisabled();
    enabled_ = false;
    return ok ? Result::Sent : Result::TransportError;
  }

  const bool was_enabled = enabled_;
  bool ok = true;
  if (!was_enabled) {
    for (std::size_t i = 0; i < config_.ids.wheel_uart.size(); ++i) {
      ok = transport_.sendUart(
               static_cast<std::uint8_t>(i),
               motor_frame::buildTorqueEnable(config_.ids.wheel_uart[i], true)) &&
          ok;
    }
  }

  // GDS68 controller-mode/axis-state frames are sent once on the transition.
  if (!was_enabled) {
    for (std::size_t i = 0; i < kLegJoints.size(); ++i) {
      const auto node = config_.ids.leg_can[i];
      ok = transport_.sendCan(
               motor_frame::canId(node, motor_frame::kSetControllerModeCmdId),
               motor_frame::packSetControllerMode(
                   motor_frame::kControlModePosition, motor_frame::kInputModeMit)) &&
          ok;
      ok = transport_.sendCan(
               motor_frame::canId(node, motor_frame::kSetAxisStateCmdId),
               motor_frame::packSetAxisState(motor_frame::kAxisStateClosedLoop)) &&
          ok;
    }
    if (!ok) {
      enabled_ = false;
      (void)sendDisabled();
      return Result::TransportError;
    }
    enabled_ = true;
    wheel_enable_us_ = now_us;
  }

  // The wheel driver requires >=20 ms between Torque Enable and Moving Speed.
  // Leg MIT frames are safe immediately after the GDS68 mode transition.
  for (std::size_t i = 0; i < kLegJoints.size(); ++i) {
    const std::size_t joint = kLegJoints[i];
    const std::size_t hip = joint < 2U ? 0U : 3U;
    const std::size_t knee = hip + 1U;
    const auto stiffness = legStiffness(
        state.joints.position_rad[hip], state.joints.position_rad[knee]);
    const auto send_joint = [&](std::size_t joint, float base_kp, float inertia) {
      const float kp_scale = finiteOrZero(command.kp_scale[joint]);
      const float kd_scale = finiteOrZero(command.kd_scale[joint]);
      const float base_kd = 2.0F * std::sqrt(std::max(base_kp, 0.0F) * inertia);
      const std::size_t other = isHip(joint) ? knee : hip;
      const float cross = stiffness.cross_nm_per_rad *
          (finiteOrZero(command.position_rad[other]) -
           finiteOrZero(state.joints.position_rad[other])) * kp_scale;
      return transport_.sendCan(
          motor_frame::canId(config_.ids.leg_can[i], motor_frame::kMitControlCmdId),
          motor_frame::packMitCommand(
              finiteOrZero(command.position_rad[joint]),
              finiteOrZero(command.velocity_rad_s[joint]),
              base_kp * kp_scale, base_kd * kd_scale,
              finiteOrZero(command.feedforward_nm[joint]) + cross));
    };
    ok = send_joint(
             joint, isHip(joint) ? stiffness.hip_nm_per_rad
                                 : stiffness.knee_nm_per_rad,
             isHip(joint) ? robot_control::geometry::leg_hip_inertia_kgm2
                          : robot_control::geometry::leg_knee_inertia_kgm2) &&
        ok;
  }

  if (now_us - wheel_enable_us_ >= config_.wheel_enable_settle_us) {
    for (std::size_t i = 0; i < kWheelJoints.size(); ++i) {
      ok = sendWheelCommand(i, command.velocity_rad_s[kWheelJoints[i]]) && ok;
    }
  }
  return ok ? Result::Sent : Result::TransportError;
}

}  // namespace tap_fw
