#pragma once
#include "tap_hw/HybridMotorBackend.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace tap_hw::hybrid {
// This boundary is also exercised on the host with the real packet codec.
class MotorCritical {
 public:
  virtual ~MotorCritical() = default;
  virtual void enter() noexcept = 0;
  virtual void leave() noexcept = 0;
};
struct MotorGuard {
  MotorCritical& lock;
  explicit MotorGuard(MotorCritical& l) : lock(l) { lock.enter(); }
  ~MotorGuard() { lock.leave(); }
};
enum class BusFormat { Joint, Wheel };
enum class BusError : std::uint8_t { None, Unavailable, Packet, Tx, EchoTimeout, EchoMismatch, ReplyTimeout, ReplyInvalid, ReadbackMismatch };
struct BusReply {
  std::uint8_t status = 0;
  std::int16_t position = 0;
  std::array<std::uint8_t, 16> data{};
};
// Wire contract from BL4012e/3in1/joint_bus.c. A transaction must be bounded.
class MotorBus {
 public:
  virtual ~MotorBus() = default;
  virtual std::uint64_t now() const noexcept = 0;
  virtual BusError lastError() const noexcept { return BusError::Unavailable; }
  virtual bool request(BusFormat, std::uint8_t id, std::uint8_t instruction,
                       const std::uint8_t* params, std::size_t count,
                       std::size_t read_count, BusReply&) noexcept = 0;
};
std::size_t motorPacket(std::uint8_t id, std::uint8_t instruction,
                        const std::uint8_t* params, std::size_t count,
                        std::array<std::uint8_t,48>& out) noexcept;
bool motorReply(BusFormat, std::uint8_t id, std::uint8_t instruction,
                 std::uint8_t address, std::uint8_t request_length,
                 const std::uint8_t* bytes, std::size_t count,
                 std::size_t read_count, BusReply&) noexcept;

// Hardware readback 2026-10-30: hip Kp_max=20, Kd_max=.120 on both buses.
// The bench GUI request .7 was clamped to .120 by its firmware.
struct MotorGains { float kp = 10.F, kd = .12F; };
struct BlMotorConfig {
  // Index by MODEL side: 0=left UART2/GPIO15, 1=right UART1/GPIO17.
  // Order inside each side: hip, crank, wheel. User confirmed side mapping.
  std::array<std::array<int,3>,2> sign{{{{1,-1,1}},{{-1,1,-1}}}};
  std::array<std::uint8_t,3> id{{2,3,1}};
  // Each profile can be characterized independently; these are bench gains,
  // not a claim of equivalence to the MuJoCo recovery PI or policy servo.
  std::array<MotorGains,3> profile{{{}, {}, {}}}; // HOME, RECOVERY, POLICY
  float wheel_max_rpm = 950.F; // existing bench command cap, not rated speed
  float recovery_wheel_tau_s = .075F;
  float hip_min = -1.56817833F, hip_max = 2.35706715F;
  float knee_min = 0.F, knee_max = 1.94778745F; // supplied rb_dynamic.xml
  std::uint32_t feedback_age_us = 20000, wheel_settle_us = 25000;
};
bool validMotorConfig(const BlMotorConfig&) noexcept;
enum class MotorError : std::uint32_t {
  None, Bus, Status, Configuration, Range, Expired, Feedback, Cancelled, Startup
};
struct MotorBatch {
  std::uint64_t token = 0, deadline_us = 0;
  std::uint32_t generation = 0, epoch = 0, seq = 0;
  std::uint16_t flags = 0;
  std::array<std::array<std::uint8_t,10>,2> joint{};
  std::int16_t wheel_rpm = 0;
};
struct MotorRequest {
  bool active = false, wheel_enable = false, maintenance = false;
  std::uint32_t generation = 1, reset_generation = 0;
  std::uint64_t lease_us = 0;
  MotorBatch batch{};
};
struct MotorLegState {
  bool configured = false, off = false, prepared = false, wheel_ready = false;
  std::uint32_t generation = 0, reset_generation = 0;
  std::array<float,3> raw_q{}, raw_qd{};
  std::array<std::uint64_t,3> stamp{};
  std::array<std::uint16_t,2> kp_max{}, kd_max{};
  std::array<std::uint8_t,3> status{};
  std::uint64_t completed = 0;
  std::uint32_t transactions = 0, errors = 0, cycle_max_us = 0;
  std::uint32_t recovered_writes = 0;
  std::uint8_t failed_id = 0, failed_instruction = 0, failed_address = 0;
  BusError bus_error = BusError::None;
  std::array<std::uint8_t,3> torque_enable{{255,255,255}};
  MotorError error = MotorError::None;
};
struct BlMotorDiagnostics {
  bool homed = false, active = false, in_flight = false;
  MotorError error = MotorError::None;
  std::array<MotorLegState,2> leg{};
};

// Shared between the service task and one worker per bus. Only small snapshots
// are copied under lock; no I/O, allocation or wait while holding the lock.
class BlMotorBackend : public MotorBackend {
 public:
  explicit BlMotorBackend(MotorCritical& lock, BlMotorConfig config = {})
      : lock_(lock), config_(config) {
    if (!validMotorConfig(config_)) error_=MotorError::Configuration;
  }
  bool ready() const noexcept override;
  bool disabled() const noexcept override;
  MotorFeedback feedback(std::uint64_t now) noexcept override;
  void disable() noexcept override;
  void emergencyStop() noexcept override;
  void beginMaintenance() noexcept override;
  bool resetFault() noexcept override;
  ApplyResult apply(const HcrTarget&, std::uint32_t seq,
                    std::uint64_t deadline, std::uint64_t now,
                    MotorApplied&) noexcept override;
  // Explicit manual reference capture; never assumes the boot pose is home.
  // The caller must ensure GatewayCore is disarmed. Values are hipL,R,kneeL,R.
  bool captureHome(const std::array<float,4>& model_q, std::uint64_t now) noexcept;
  BlMotorDiagnostics diagnostics() const noexcept;
  MotorRequest request(unsigned side) const noexcept;
  void publish(unsigned side, const MotorLegState&) noexcept;
  void trip(MotorError) noexcept;
  const BlMotorConfig& config() const noexcept { return config_; }
 private:
  bool sensorsFresh(std::uint64_t now) const noexcept; // lock held
  bool makeBatch(const HcrTarget&, std::uint32_t, std::uint64_t,
                 std::uint64_t) noexcept; // lock held
  void cancel() noexcept; // lock held
  MotorCritical& lock_;
  BlMotorConfig config_;
  std::array<MotorRequest,2> request_{};
  std::array<MotorLegState,2> state_{};
  std::array<std::array<float,3>,2> zero_{};
  MotorApplied completed_{}, pending_{};
  std::uint64_t token_ = 0, applied_at_ = 0, pending_at_ = 0;
  std::uint32_t latest_epoch_ = 0, latest_seq_ = 0;
  bool homed_ = false, in_flight_ = false, have_complete_ = false, have_latest_ = false, maintenance_ = false;
  MotorError error_ = MotorError::None;
};

// One instance is owned by exactly one task. step() performs at most a small
// bounded set of transactions; it checks cancellation/expiry between them.
class BlMotorWorker {
 public:
  BlMotorWorker(BlMotorBackend& owner, unsigned side, MotorBus& bus)
      : owner_(owner), side_(side), bus_(bus) {}
  void step() noexcept;
 private:
  bool transact(unsigned motor, std::uint8_t instruction,
                const std::uint8_t* p, std::size_t n, std::size_t read = 0,
                BusReply* out = nullptr) noexcept;
  bool torque(unsigned motor, bool enable) noexcept;
  bool wheelSpeed(std::int16_t rpm) noexcept;
  bool wheelRead() noexcept;
  bool verifyWheelZero() noexcept;
  void jointSample(unsigned motor, const BusReply&) noexcept;
  bool poll(const MotorRequest* active = nullptr) noexcept;
  bool configure() noexcept;
  bool off() noexcept;
  bool allowed(const MotorRequest&, bool batch) noexcept;
  void fail(MotorError) noexcept;
  void recordFailure(unsigned motor, std::uint8_t instruction, std::uint8_t address, BusError) noexcept;
  BlMotorBackend& owner_;
  unsigned side_;
  MotorBus& bus_;
  MotorLegState state_{};
  std::array<bool,3> on_{{true,true,true}}; // unknown at boot: force OFF
  std::array<std::int16_t,2> previous_joint_{};
  std::uint16_t previous_wheel_ = 0;
  std::int64_t wheel_counts_ = 0;
  std::uint64_t wheel_enabled_at_ = 0, last_idle_poll_ = 0, last_off_attempt_ = 0, last_off_verify_ = 0;
  bool wheel_sample_ = false, wheel_zero_verified_ = false;
};
} // namespace tap_hw::hybrid
