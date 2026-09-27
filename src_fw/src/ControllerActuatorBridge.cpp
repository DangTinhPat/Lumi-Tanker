#include "tap_fw/ControllerActuatorBridge.hpp"

#include <algorithm>
#include <cmath>

namespace tap_fw {

namespace motor_frame = robot_control::motor_frame;
namespace motor_transport = robot_control::motor_transport;
namespace {

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
  const auto& binding = config_.ids.wheel_uart[wheel];
  return transport_.sendUart(
      bus, motor_frame::buildTorqueEnable(binding.motor_id, false));
}

bool ControllerActuatorBridge::sendWheelCommand(
    std::size_t wheel, float velocity_rad_s) noexcept {
  const std::uint8_t bus = static_cast<std::uint8_t>(wheel);
  const auto& binding = config_.ids.wheel_uart[wheel];
  // Torque Enable is sent once on the disabled->enabled edge. Repeating it
  // here could restart the real driver's 20 ms settle window on every tick.
  return transport_.sendUart(
      bus, motor_frame::buildMovingSpeed(
               binding.motor_id,
               binding.driver_sign * finiteOrZero(velocity_rad_s)));
}

void ControllerActuatorBridge::onCanReceived(
    const motor_transport::CanFrame& frame) noexcept {
  if ((frame.id & 0x1FU) != motor_frame::kMitControlCmdId) return;
  const auto node = static_cast<std::uint8_t>(frame.id >> 5U);
  for (std::size_t i = 0; i < config_.ids.leg_can.size() && i < mit_pending_.size(); ++i) {
    if (config_.ids.leg_can[i].node_id == node) mit_pending_[i] = false;
  }
}

bool ControllerActuatorBridge::sendDisabled() noexcept {
  mit_pending_.fill(false);
  leg_target_integral_rad_.fill(0.0F);
  bool ok = true;
  for (const auto& leg : config_.ids.leg_can) {
    ok = transport_.sendCan(
             motor_frame::canId(leg.node_id, motor_frame::kSetAxisStateCmdId),
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
    // The disable edge goes out at once; the repeats that keep a restarted
    // driver stopped are rate-limited to protect the CAN budget.
    if (!enabled_ && disabled_sent_ &&
        now_us - last_disabled_us_ < config_.disabled_repeat_period_us) {
      return Result::RateLimited;
    }
    const bool ok = sendDisabled();
    enabled_ = false;
    disabled_sent_ = true;
    last_disabled_us_ = now_us;
    return ok ? Result::Sent : Result::TransportError;
  }

  const bool was_enabled = enabled_;
  bool ok = true;
  if (!was_enabled) {
    for (std::size_t i = 0; i < config_.ids.wheel_uart.size(); ++i) {
      ok = transport_.sendUart(
               static_cast<std::uint8_t>(i),
               motor_frame::buildTorqueEnable(
                   config_.ids.wheel_uart[i].motor_id, true)) &&
          ok;
    }
  }

  // GDS68 controller-mode/axis-state frames are sent once on the transition.
  if (!was_enabled || limits_dirty_) {
    limits_dirty_ = false;
    for (const auto& leg : config_.ids.leg_can) {
      if (config_.leg_current_limit_a > 0.0F) {
        ok = transport_.sendCan(
                 motor_frame::canId(leg.node_id, motor_frame::kSetLimitsCmdId),
                 motor_frame::packSetLimits(config_.leg_vel_limit_turns_s,
                                            config_.leg_current_limit_a)) &&
            ok;
      }
    }
  }
  if (!was_enabled) {
    for (const auto& leg : config_.ids.leg_can) {
      const auto node = leg.node_id;
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
  // Leg MIT frames are safe immediately after the GDS68 mode transition, then
  // follow the leg cadence (half a control period of jitter is tolerated).
  const bool legs_due = !was_enabled ||
      now_us - last_leg_us_ + config_.command_period_us / 2U >=
          config_.leg_command_period_us;
  if (legs_due) last_leg_us_ = now_us;
  for (std::size_t slot = 0; slot < config_.ids.leg_can.size(); ++slot) {
    if (!legs_due) break;
    const auto& leg = config_.ids.leg_can[slot];
    const bool tracked = slot < mit_pending_.size();
    if (tracked && mit_pending_[slot] &&
        now_us - mit_pending_since_us_[slot] < config_.mit_reply_timeout_us) {
      ++mit_deferred_;  // previous answer still on its way: same CAN id
      continue;
    }
    const std::size_t joint = leg.joint_index;
    const std::size_t hip = joint < 2U ? 0U : 3U;
    const std::size_t knee = hip + 1U;
    const auto stiffness = legStiffness(
        state.joints.position_rad[hip], state.joints.position_rad[knee]);
    const auto send_joint = [&](std::size_t joint, float base_kp, float inertia) {
      const float kp_scale = finiteOrZero(command.kp_scale[joint]);
      const float kd_scale = finiteOrZero(command.kd_scale[joint]);
      const float gain_scale = std::clamp(config_.leg_gain_scale, 0.0F, 1.0F);
      const float joint_kp = config_.leg_fixed_kp_nm_per_rad > 0.0F
          ? config_.leg_fixed_kp_nm_per_rad
          : std::min(std::max(base_kp, 0.0F) * gain_scale,
                     std::max(config_.leg_max_kp_nm_per_rad, 0.0F));
      const std::size_t other = isHip(joint) ? knee : hip;
      const float cross = stiffness.cross_nm_per_rad *
          (finiteOrZero(command.position_rad[other]) -
           finiteOrZero(state.joints.position_rad[other])) * kp_scale;
      // Critically damped at the (scaled, capped) joint stiffness.
      const float kp = std::max(joint_kp * kp_scale, 0.0F);
      const float kd = config_.leg_fixed_kp_nm_per_rad > 0.0F &&
              config_.leg_fixed_kd_nm_s_per_rad > 0.0F
          ? config_.leg_fixed_kd_nm_s_per_rad * kd_scale
          : std::max(2.0F * std::sqrt(joint_kp * inertia) * kd_scale, 0.0F);
      const float velocity = finiteOrZero(command.velocity_rad_s[joint]);
      float target = finiteOrZero(command.position_rad[joint]);

      // Leg torque envelope. The simulated GDS68 clamps its PD output at
      // torque_nm; the real MIT frame has no limit field, so bound T_ff and
      // the position error such that |kp*e + kd*(v - dq) + T_ff| <= limit
      // against the latest measured q/dq.
      const float limit = command.torque_nm[joint] > 0.0F &&
              std::isfinite(command.torque_nm[joint])
          ? command.torque_nm[joint] : config_.leg_torque_limit_nm;
      const float tff = std::clamp(
          finiteOrZero(command.feedforward_nm[joint]) + cross, -limit, limit);
      const float q = state.joints.position_rad[joint];
      const float dq = state.joints.velocity_rad_s[joint];
      // Close the steady error on the target itself (the drivers act on the
      // Kp spring, not on T_ff): integrate command - measured into an offset.
      if (config_.leg_target_integral_per_s > 0.0F && kp > 1.0e-6F && std::isfinite(q)) {
        const float bound = std::max(config_.leg_target_integral_max_rad, 0.0F);
        const float dt = static_cast<float>(config_.leg_command_period_us) * 1.0e-6F;
        leg_target_integral_rad_[joint] = std::clamp(
            leg_target_integral_rad_[joint] +
                config_.leg_target_integral_per_s * (target - q) * dt,
            -bound, bound);
        target += leg_target_integral_rad_[joint];
      }
      // Envelope on the spring alone: T_ff is not realized by the drivers,
      // so counting it here only shrinks or inflates the spring.
      if (kp > 1.0e-6F && std::isfinite(q) && std::isfinite(dq)) {
        const float damping = kd * (velocity - dq);
        target = q + std::clamp(target - q, (-limit - damping) / kp,
                                (limit - damping) / kp);
      }

      // Core -> driver frame: driver = sign * (core - zero offset).
      const float offset = leg_offsets_ != nullptr ? (*leg_offsets_)[joint] : 0.0F;
      return transport_.sendCan(
          motor_frame::canId(leg.node_id, motor_frame::kMitControlCmdId),
          motor_frame::packMitCommand(
              leg.driver_sign * (target - offset), leg.driver_sign * velocity,
              kp, kd, leg.driver_sign * tff));
    };
    const bool sent = send_joint(
        joint, isHip(joint) ? stiffness.hip_nm_per_rad
                            : stiffness.knee_nm_per_rad,
        isHip(joint) ? robot_control::geometry::leg_hip_inertia_kgm2
                     : robot_control::geometry::leg_knee_inertia_kgm2);
    if (sent && tracked) {
      mit_pending_[slot] = true;
      mit_pending_since_us_[slot] = now_us;
    }
    ok = sent && ok;
  }

  if (now_us - wheel_enable_us_ >= config_.wheel_enable_settle_us) {
    for (std::size_t i = 0; i < config_.ids.wheel_uart.size(); ++i) {
      ok = sendWheelCommand(
               i, command.velocity_rad_s[config_.ids.wheel_uart[i].joint_index]) && ok;
    }
  }
  return ok ? Result::Sent : Result::TransportError;
}

}  // namespace tap_fw
