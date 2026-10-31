#include "tap_hw/HybridGatewayCore.hpp"
#include <algorithm>
#include <cmath>

namespace tap_hw::hybrid {
namespace {
bool fresh(std::uint64_t stamp, std::uint64_t now, std::uint64_t age) {
  return stamp != 0 && now >= stamp && now-stamp <= age;
}
template <std::size_t N> bool finite(const std::array<float, N>& a) {
  for (float v : a) if (!std::isfinite(v)) return false;
  return true;
}
}

void GatewayCore::stop(LocalFault reason) noexcept {
  motor_.disable();
  enabled_ = armed_ = applied_valid_ = false;
  disable_ack_pending_ = false;
  if (reason == LocalFault::Estop || (reason != LocalFault::None && fault_ == LocalFault::None))
    fault_ = reason;
}

void GatewayCore::newSession(std::uint32_t epoch) noexcept {
  stop();
  (void)motor_.resetFault();
  epoch_ = epoch; tx_seq_ = rx_seq_ = applied_seq_ = target_seq_ = 0;
  rx_stamp_ = deadline_ = 0;
  connected_ = true; have_rx_ = reset_pending_ = false;
  fault_ = LocalFault::None;
  history_ = {}; history_next_ = 0; target_ = {}; applied_ = {};
  remote_mode_ = remote_fault_ = 0;
}

std::uint32_t GatewayCore::flags(std::uint64_t now) const noexcept {
  std::uint32_t f = 0;
  float norm = 0;
  for (float g : imu_.gravity) norm += g*g;
  if (imu_.valid && fresh(imu_.stamp_us, now, 10000) && finite(imu_.gyro) &&
      finite(imu_.accel) && finite(imu_.gravity) && norm >= .81F && norm <= 1.21F)
    f |= HCR_IMU_OK;
  const bool enc = feedback_.encoders_valid && fresh(feedback_.stamp_us, now, 20000) &&
      finite(feedback_.q) && finite(feedback_.qd);
  if (enc) f |= HCR_ENCODER_OK;
  if (enc && feedback_.homed && (f & HCR_IMU_OK)) f |= HCR_CALIBRATED;
  if (enc && feedback_.torque_valid && finite(feedback_.hip_torque)) f |= HCR_TORQUE_VALID;
  if (applied_valid_ && enabled_) f |= HCR_TARGET_VALID;
  if (feedback_.estop || fault_ == LocalFault::Estop) f |= HCR_ESTOP;
  if (feedback_.driver_fault || (fault_ != LocalFault::None && fault_ != LocalFault::Estop &&
                                fault_ != LocalFault::RemoteFault))
    f |= HCR_DRIVER_FAULT;
  return f;
}

bool GatewayCore::freshSensors(std::uint64_t now) const noexcept {
  return (flags(now) & (7U | HCR_ESTOP | HCR_DRIVER_FAULT)) == 7U;
}

void GatewayCore::tick(std::uint64_t now) noexcept {
  // Deadline checked before any driver or UART work. UART timestamps never
  // postpone this deadline when the service task has stalled.
  if (enabled_ && now >= deadline_) stop(LocalFault::TargetTimeout);
  if (connected_ && !reset_pending_ && now >= last_rx_us_ && now-last_rx_us_ >= 500000) {
    stop(LocalFault::Transport);
    reset_pending_ = true; // request a fresh epoch when the cable/LF comes back
  }
  feedback_ = motor_.feedback(now);
  if (feedback_.estop) stop(LocalFault::Estop);
  if (feedback_.driver_fault) stop(LocalFault::Driver);
  if (armed_ && (!motor_.ready() || !freshSensors(now))) stop(LocalFault::Sensors);
  last_tick_ = now;
  if (!enabled_) {
    if (disable_ack_pending_ && motor_.disabled()) {
      applied_seq_ = target_seq_; disable_ack_pending_ = false;
    }
    return;
  }
  MotorApplied applied{};
  const auto result = motor_.apply(target_, target_seq_, deadline_, now, applied);
  if (result == ApplyResult::Pending) return;
  if (result == ApplyResult::Fault || !finite(applied.target) || applied.epoch != epoch_ ||
      hcr_newer(applied.seq, target_seq_) ||
      (applied_valid_ && applied.seq != applied_seq_ && !hcr_newer(applied.seq, applied_seq_))) {
    stop(LocalFault::Driver); return;
  }
  applied_ = applied.target; applied_seq_ = applied.seq; applied_valid_ = true;
}

bool GatewayCore::sourceValid(std::uint32_t seq, std::uint64_t now) const noexcept {
  for (const auto& h : history_)
    if (h.valid && h.seq == seq && fresh(h.stamp, now, 30000)) return true;
  return false;
}

void GatewayCore::telemetrySent(std::uint32_t epoch, std::uint32_t seq,
                                std::uint64_t sample_us) noexcept {
  if (epoch != epoch_ || !connected_) return;
  history_[history_next_] = {seq, sample_us, true};
  history_next_ = (history_next_+1)%history_.size();
}

WirePacket GatewayCore::receive(const HcrFrame& f, std::uint64_t now,
                                std::uint64_t received_us) noexcept {
  WirePacket reply{};
  if (maintenance_) { ++rejected_; return reply; }
  if (enabled_ && now >= deadline_) stop(LocalFault::TargetTimeout);
  if (!received_us) received_us = now;
  if (received_us > now || now-received_us >= HCR_TARGET_TTL_US) {
    ++rejected_; return reply;
  }
  if (f.type == HCR_HELLO) {
    if (f.size != 12 || hcr_u16(f.payload+4) != 200 || hcr_u16(f.payload+6) != 100 ||
        hcr_u32(f.payload+8) != HCR_SCHEMA || hcr_u32(f.payload) == 0) {
      ++rejected_; return reply;
    }
    const auto epoch = hcr_u32(f.payload);
    if (!connected_ || epoch != epoch_) newSession(epoch);
    else if (have_rx_ && (!hcr_newer(f.seq, rx_seq_) || f.stamp_us < rx_stamp_)) {
      ++rejected_; return reply;
    }
    rx_seq_ = f.seq; rx_stamp_ = f.stamp_us; have_rx_ = true;
    last_rx_us_ = now;
    reply.size = hcr_hello_ack(reply.bytes.data(), reply.bytes.size(), tx_seq_++, now, epoch_);
    return reply;
  }
  if (!connected_ || reset_pending_ || f.size < 4 || hcr_u32(f.payload) != epoch_ ||
      (have_rx_ && (!hcr_newer(f.seq, rx_seq_) || f.stamp_us < rx_stamp_))) {
    ++rejected_; return reply;
  }
  if (f.type == HCR_STATUS && f.size == 16 && hcr_u32(f.payload+4) <= 6) {
    remote_mode_ = hcr_u32(f.payload+4); remote_fault_ = hcr_u32(f.payload+8);
    rx_seq_ = f.seq; rx_stamp_ = f.stamp_us; have_rx_ = true;
    last_rx_us_ = now;
    if (remote_mode_ == 6 || remote_fault_) stop(LocalFault::RemoteFault);
    else if (remote_mode_ <= 1) stop();
    return reply;
  }
  HcrTarget target{};
  if (!hcr_target(&f, &target) ||
      (target.profile == HCR_DISABLED && target.flags != 0)) {
    ++rejected_; return reply;
  }
  if (target.profile != HCR_DISABLED &&
      (fault_ != LocalFault::None || !motor_.ready() || !freshSensors(now) ||
       !sourceValid(target.source_seq, now) || remote_mode_ < 2 || remote_mode_ > 5)) {
    ++rejected_; return reply;
  }
  if (target.profile != HCR_DISABLED && !armed_) {
    constexpr float home[4] = {2.35706715F,2.35706715F,0,0};
    if (!motor_.disabled()) { ++rejected_; return reply; }
    for (unsigned i=0;i<4;++i)
      if (std::abs(feedback_.q[i]-home[i])>.15F) { ++rejected_; return reply; }
    armed_=true;
  }
  rx_seq_ = f.seq; rx_stamp_ = f.stamp_us; have_rx_ = true;
  last_rx_us_ = now;
  ++rx_targets_;
  if (target.profile == HCR_DISABLED) {
    motor_.disable(); enabled_ = applied_valid_ = false;
    armed_ = false;
    target_seq_ = f.seq;
    disable_ack_pending_ = true;
    if (motor_.disabled()) { applied_seq_ = f.seq; disable_ack_pending_ = false; }
  } else {
    target_ = target; target_seq_ = f.seq; enabled_ = true;
    disable_ack_pending_ = false;
    deadline_ = received_us+HCR_TARGET_TTL_US;
  }
  return reply;
}

WirePacket GatewayCore::requestSession(std::uint64_t now) noexcept {
  WirePacket out{};
  if (maintenance_) return out;
  if (connected_ && !reset_pending_) return out;
  std::uint8_t p[12]{};
  hcr_put16(p+4, 200); hcr_put16(p+6, 100); hcr_put32(p+8, HCR_SCHEMA);
  out.size = hcr_encode(out.bytes.data(), out.bytes.size(), HCR_HELLO, 0, now, p, 12);
  return out;
}

bool GatewayCore::clearLocalFault() noexcept {
  if (armed_ || enabled_ || !motor_.disabled() || feedback_.estop) return false;
  if (!motor_.resetFault()) return false;
  fault_=LocalFault::None; reset_pending_=false; return true;
}

WirePacket GatewayCore::telemetry(std::uint64_t now) noexcept {
  WirePacket out{};
  if (maintenance_) return out;
  if (!connected_ || reset_pending_) return out;
  const auto f = flags(now);
  float values[29]{};
  // Invalid groups remain finite placeholders with validity bits cleared.
  values[HCR_GRAVITY+2] = -1;
  if (f & HCR_IMU_OK) {
    std::copy(imu_.gyro.begin(), imu_.gyro.end(), values+HCR_GYRO);
    std::copy(imu_.gravity.begin(), imu_.gravity.end(), values+HCR_GRAVITY);
    std::copy(imu_.accel.begin(), imu_.accel.end(), values+HCR_ACCEL);
  }
  if (f & HCR_ENCODER_OK) {
    std::copy(feedback_.q.begin(), feedback_.q.end(), values+HCR_Q);
    std::copy(feedback_.qd.begin(), feedback_.qd.end(), values+HCR_QD);
  }
  if (f & HCR_TORQUE_VALID)
    std::copy(feedback_.hip_torque.begin(), feedback_.hip_torque.end(), values+HCR_HIP_TORQUE);
  if (f & HCR_TARGET_VALID)
    std::copy(applied_.begin(), applied_.end(), values+HCR_APPLIED_TARGET);
  // Snapshot is assembled on this tick. Individual sensor age is gated above;
  // the packet timestamp is never refreshed later by the UART writer.
  out.size = hcr_telemetry(out.bytes.data(), out.bytes.size(), tx_seq_++, now,
                          epoch_, f, applied_seq_, values);
  return out;
}

Diagnostics GatewayCore::diagnostics(std::uint64_t now) const noexcept {
  return {epoch_,rx_targets_,rejected_,applied_seq_,remote_mode_,remote_fault_,flags(now),
          fault_,connected_ && !reset_pending_,armed_,enabled_,motor_.ready()};
}
}  // namespace tap_hw::hybrid
