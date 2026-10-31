#include "tap_hw/BlMotor.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace tap_hw::hybrid {
namespace {
constexpr float pi = 3.14159265358979323846F;
constexpr float count_rad = 2*pi/16384.F, rpm_rad = 2*pi/60.F;
std::uint8_t checksum(const std::uint8_t* p, std::size_t n) {
  std::uint8_t sum = 0; for (std::size_t i=0;i<n;++i) sum += p[i];
  return static_cast<std::uint8_t>(~sum);
}
std::uint16_t u16(const std::uint8_t* p) { return p[0] | (std::uint16_t(p[1])<<8); }
void put(std::uint8_t* p, std::int32_t v) { p[0]=v&255; p[1]=(v>>8)&255; }
bool fresh(std::uint64_t stamp, std::uint64_t now, std::uint64_t age) {
  return stamp && now>=stamp && now-stamp<=age;
}
bool quantize(float v, std::int32_t lo, std::int32_t hi, std::int32_t& out) {
  if (!std::isfinite(v) || v<lo || v>hi) return false;
  out=static_cast<std::int32_t>(std::lround(v)); return out>=lo && out<=hi;
}
}

std::size_t motorPacket(std::uint8_t id, std::uint8_t instruction,
                        const std::uint8_t* params, std::size_t count,
                        std::array<std::uint8_t,48>& out) noexcept {
  if (!id || id>=0xfe || count+6>out.size() || (count && !params)) return 0;
  out[0]=out[1]=0xff; out[2]=id; out[3]=count+2; out[4]=instruction;
  if (count) std::memcpy(out.data()+5,params,count);
  out[count+5]=checksum(out.data()+2,count+3); return count+6;
}
bool motorReply(BusFormat fmt, std::uint8_t id, std::uint8_t instruction,
                 std::uint8_t address, std::uint8_t request_length,
                 const std::uint8_t* b, std::size_t count,
                 std::size_t read_count, BusReply& reply) noexcept {
  const unsigned echo=fmt==BusFormat::Wheel?5:7;
  if (!b || read_count>reply.data.size() || count!=echo+4+read_count ||
      b[0]!=0xff || b[1]!=0xff || b[2]!=id || b[3]!=count-4 ||
      b[count-1]!=checksum(b+2,count-3) || b[echo]!=request_length ||
      b[echo+1]!=instruction || b[echo+2]!=address) return false;
  reply={}; reply.status=b[4];
  if (fmt==BusFormat::Joint) reply.position=static_cast<std::int16_t>(u16(b+5));
  if (read_count) std::memcpy(reply.data.data(),b+echo+3,read_count);
  return true;
}

bool validMotorConfig(const BlMotorConfig& c) noexcept {
  for (const auto& side:c.sign) for (int s:side) if (s!=1 && s!=-1) return false;
  for (unsigned i=0;i<3;++i) {
    if (!c.id[i] || c.id[i]>=0xfe) return false;
    for (unsigned j=0;j<i;++j) if (c.id[i]==c.id[j]) return false;
  }
  for (const auto& g:c.profile)
    if (!std::isfinite(g.kp) || !std::isfinite(g.kd) || g.kp<0 || g.kd<0 ||
        g.kp>65.535F || g.kd>65.535F) return false;
  return std::isfinite(c.wheel_max_rpm) && c.wheel_max_rpm>0 &&
    c.feedback_age_us>0 && c.feedback_age_us<=20000 &&
    c.wheel_max_rpm*c.feedback_age_us/60000000.F<.5F &&
    std::isfinite(c.recovery_wheel_tau_s) && c.recovery_wheel_tau_s>=0 &&
    std::isfinite(c.hip_min) && std::isfinite(c.hip_max) && c.hip_min<c.hip_max &&
    std::isfinite(c.knee_min) && std::isfinite(c.knee_max) && c.knee_min<c.knee_max &&
    c.wheel_settle_us>=20000;
}

bool BlMotorBackend::sensorsFresh(std::uint64_t now) const noexcept {
  for (const auto& s:state_) {
    if (!s.configured || s.error!=MotorError::None) return false;
    for (unsigned j=0;j<3;++j)
      if (!fresh(s.stamp[j],now,config_.feedback_age_us) ||
          !std::isfinite(s.raw_q[j]) || !std::isfinite(s.raw_qd[j])) return false;
  }
  return true;
}
bool BlMotorBackend::ready() const noexcept {
  MotorGuard guard(lock_);
  return !maintenance_ && homed_ && error_==MotorError::None && state_[0].configured && state_[1].configured;
}
bool BlMotorBackend::disabled() const noexcept {
  MotorGuard guard(lock_);
  return !request_[0].active && !request_[1].active &&
    state_[0].off && state_[1].off &&
    state_[0].generation==request_[0].generation && state_[1].generation==request_[1].generation;
}
MotorFeedback BlMotorBackend::feedback(std::uint64_t now) noexcept {
  MotorGuard guard(lock_);
  MotorFeedback f{}; f.driver_fault=error_!=MotorError::None;
  f.encoders_valid=sensorsFresh(now); f.homed=homed_ && f.encoders_valid;
  if (!f.encoders_valid) return f;
  f.stamp_us=now;
  for (unsigned s=0;s<2;++s) {
    const auto& m=state_[s]; const auto& sign=config_.sign[s];
    const float hip=sign[0]*m.raw_q[0]+zero_[s][0];
    const float phi=sign[1]*m.raw_q[1]+zero_[s][1];
    f.q[s]=hip; f.q[2+s]=phi-hip; f.q[4+s]=sign[2]*m.raw_q[2]+zero_[s][2];
    f.qd[s]=sign[0]*m.raw_qd[0];
    f.qd[2+s]=sign[1]*m.raw_qd[1]-f.qd[s]; f.qd[4+s]=sign[2]*m.raw_qd[2];
    for (auto t:m.stamp) f.stamp_us=std::min(f.stamp_us,t);
  }
  return f; // no current-based torque sensor: TORQUE_VALID stays clear
}
void BlMotorBackend::cancel() noexcept {
  if (request_[0].active || request_[1].active) {
    for (auto& r:request_) { r.active=false; r.batch={}; ++r.generation; }
  }
  in_flight_=have_complete_=have_latest_=false; applied_at_=0;
}
void BlMotorBackend::disable() noexcept { MotorGuard guard(lock_); cancel(); }
void BlMotorBackend::emergencyStop() noexcept {
  MotorGuard guard(lock_); cancel();
  // A new explicit E-stop rechecks all drivers even if our cached state is OFF.
  for (auto& r:request_) ++r.generation;
}
void BlMotorBackend::beginMaintenance() noexcept {
  MotorGuard guard(lock_);
  if (maintenance_) return;
  cancel(); maintenance_=true;
  for (auto& r:request_) { r.maintenance=true; ++r.generation; }
}
bool BlMotorBackend::resetFault() noexcept {
  MotorGuard guard(lock_); cancel();
  if (maintenance_) return false;
  if (!validMotorConfig(config_)) return false;
  if (error_==MotorError::None) return true; // LF RESET must not erase a good manual home
  error_=MotorError::None; homed_=false;
  for (auto& r:request_) { ++r.reset_generation; ++r.generation; }
  state_={}; return true; // workers must force OFF and reconfigure before ready
}
void BlMotorBackend::trip(MotorError e) noexcept {
  MotorGuard guard(lock_);
  if (error_==MotorError::None) error_=e;
  homed_=false; cancel();
}
MotorRequest BlMotorBackend::request(unsigned side) const noexcept {
  MotorGuard guard(lock_); return request_[side];
}
void BlMotorBackend::publish(unsigned side, const MotorLegState& s) noexcept {
  MotorGuard guard(lock_);
  if (s.reset_generation==request_[side].reset_generation) state_[side]=s;
}
BlMotorDiagnostics BlMotorBackend::diagnostics() const noexcept {
  MotorGuard guard(lock_); return {homed_,request_[0].active,in_flight_,error_,state_};
}
bool BlMotorBackend::captureHome(const std::array<float,4>& q, std::uint64_t now) noexcept {
  MotorGuard guard(lock_);
  if (maintenance_ || request_[0].active || request_[1].active || error_!=MotorError::None ||
      !state_[0].off || !state_[1].off ||
      state_[0].generation!=request_[0].generation ||
      state_[1].generation!=request_[1].generation || !sensorsFresh(now)) return false;
  for (unsigned s=0;s<2;++s) {
    if (!std::isfinite(q[s]) || !std::isfinite(q[2+s]) || q[s]<config_.hip_min ||
        q[s]>config_.hip_max || q[2+s]<config_.knee_min || q[2+s]>config_.knee_max) return false;
    for (auto v:state_[s].raw_qd) if (std::abs(v)>.15F) return false;
  }
  for (unsigned s=0;s<2;++s) {
    zero_[s][0]=q[s]-config_.sign[s][0]*state_[s].raw_q[0];
    zero_[s][1]=q[s]+q[2+s]-config_.sign[s][1]*state_[s].raw_q[1];
    zero_[s][2]=-config_.sign[s][2]*state_[s].raw_q[2];
  }
  homed_=true; return true;
}

bool BlMotorBackend::makeBatch(const HcrTarget& t, std::uint32_t seq,
                                std::uint64_t deadline, std::uint64_t now) noexcept {
  std::array<MotorBatch,2> next{};
  MotorApplied actual{}; actual.epoch=t.epoch; actual.seq=seq;
  if (t.profile<HCR_HOME_PD || t.profile>HCR_POLICY_PD || (t.flags&~3U)) return false;
  for (auto v:t.target) if (!std::isfinite(v)) return false;
  for (auto v:t.feedforward) if (!std::isfinite(v)) return false;
  const auto gain=config_.profile[t.profile-1];
  for (unsigned s=0;s<2;++s) {
    auto& b=next[s]; b.flags=t.flags; b.epoch=t.epoch; b.seq=seq; b.deadline_us=deadline;
    b.generation=request_[s].generation; b.token=token_+1;
    const float hip=t.target[s], knee=t.target[2+s]-hip;
    if ((t.flags&1U) && (hip<config_.hip_min || hip>config_.hip_max ||
                        knee<config_.knee_min || knee>config_.knee_max)) return false;
    for (unsigned j=0;j<2;++j) {
      if (!(t.flags&1U)) {
        actual.target[j*2+s]=have_complete_?completed_.target[j*2+s]:
          config_.sign[s][j]*state_[s].raw_q[j]+zero_[s][j];
        continue;
      }
      std::int32_t kp,kd,pos,rpm;
      // HOME/POLICY have zero FF; RECOVERY FF is hip / knee hinge.
      const float ff=t.profile==HCR_RECOVERY ?
        t.feedforward[s]+(j?t.feedforward[2+s]:0.F) : 0.F;
      const float target=t.target[j*2+s];
      if (!quantize(gain.kp*1000.F,0,state_[s].kp_max[j],kp) ||
          !quantize(gain.kd*1000.F,0,state_[s].kd_max[j],kd) ||
          !quantize(config_.sign[s][j]*(target-zero_[s][j])/count_rad,-32768,32767,pos) ||
          !quantize(config_.sign[s][j]*ff/rpm_rad,-32768,32767,rpm)) return false;
      put(b.joint[j].data(),kp); put(b.joint[j].data()+2,kd);
      put(b.joint[j].data()+4,pos); put(b.joint[j].data()+6,rpm);
      put(b.joint[j].data()+8,0); // direct position/velocity interface, tau_ff=0
      actual.target[j*2+s]=config_.sign[s][j]*pos*count_rad+zero_[s][j];
    }
    float omega=t.target[4+s];
    if ((t.flags&2U) && std::abs(omega/rpm_rad)>config_.wheel_max_rpm) return false; // no hidden clamp
    if (!(t.flags&2U)) omega=0;
    if ((t.flags&2U) && t.profile==HCR_RECOVERY && config_.recovery_wheel_tau_s>0) {
      const float dt=applied_at_ && now>applied_at_ ? (now-applied_at_)*1e-6F : .005F;
      const float previous=have_complete_?completed_.target[4+s]:0.F;
      omega=previous+(omega-previous)*(1-std::exp(-dt/config_.recovery_wheel_tau_s));
    }
    std::int32_t rpm;
    if (!quantize(config_.sign[s][2]*omega/rpm_rad,-32768,32767,rpm)) return false;
    b.wheel_rpm=rpm; actual.target[4+s]=config_.sign[s][2]*rpm*rpm_rad;
  }
  ++token_; pending_=actual; pending_at_=now;
  for (unsigned s=0;s<2;++s) request_[s].batch=next[s];
  in_flight_=true; return true;
}
ApplyResult BlMotorBackend::apply(const HcrTarget& t, std::uint32_t seq,
                                   std::uint64_t deadline, std::uint64_t now,
                                   MotorApplied& out) noexcept {
  MotorGuard guard(lock_);
  if (maintenance_) { cancel(); return ApplyResult::Fault; }
  if (error_!=MotorError::None || !homed_ || !sensorsFresh(now) || now>=deadline) {
    if (error_==MotorError::None) error_=now>=deadline?MotorError::Expired:MotorError::Feedback;
    cancel(); return ApplyResult::Fault;
  }
  if (!have_latest_ || seq!=latest_seq_) {
    if (have_latest_ && (t.epoch!=latest_epoch_ || !hcr_newer(seq,latest_seq_))) {
      error_=MotorError::Configuration; cancel(); return ApplyResult::Fault;
    }
    have_latest_=true; latest_epoch_=t.epoch; latest_seq_=seq;
    for (auto& r:request_) { r.lease_us=deadline; r.active=true; r.wheel_enable=t.flags&2U; }
  }
  if (in_flight_ && state_[0].completed==token_ && state_[1].completed==token_ &&
      state_[0].generation==request_[0].generation && state_[1].generation==request_[1].generation) {
    completed_=pending_; have_complete_=true; in_flight_=false; applied_at_=pending_at_;
    for (auto& r:request_) r.batch={};
  }
  if (!in_flight_ && (!have_complete_ || completed_.seq!=seq) &&
      state_[0].prepared && state_[1].prepared &&
      (!(t.flags&2U) || (state_[0].wheel_ready && state_[1].wheel_ready) ||
       (t.target[4]==0.F && t.target[5]==0.F)) &&
      state_[0].generation==request_[0].generation && state_[1].generation==request_[1].generation) {
    if (!makeBatch(t,seq,deadline,now)) {
      error_=MotorError::Range; cancel(); return ApplyResult::Fault;
    }
  }
  if (have_complete_) { out=completed_; return ApplyResult::Applied; }
  return ApplyResult::Pending;
}

void BlMotorWorker::fail(MotorError e) noexcept {
  if (state_.error==MotorError::None) state_.error=e;
  state_.prepared=false; ++state_.errors; owner_.trip(e);
}
void BlMotorWorker::recordFailure(unsigned j, std::uint8_t inst, std::uint8_t addr, BusError error) noexcept {
  // Preserve the first failed transaction; OFF retries must not obscure it.
  if (state_.failed_id) return;
  state_.failed_id=owner_.config().id[j]; state_.failed_instruction=inst;
  state_.failed_address=addr; state_.bus_error=error;
}
void BlMotorWorker::jointSample(unsigned j, const BusReply& r) noexcept {
  const auto now=bus_.now();
  if (state_.stamp[j] && now>state_.stamp[j]) {
    // Joint encoder is signed multi-turn, not a modulo single-turn wheel.
    // Crossing the int16 boundary is not silently unwrapped into a legal target.
    const int delta=int(r.position)-int(previous_joint_[j]);
    if (std::abs(delta)>16384) { fail(MotorError::Feedback); return; }
    state_.raw_qd[j]=delta*count_rad/((now-state_.stamp[j])*1e-6F);
  }
  previous_joint_[j]=r.position; state_.raw_q[j]=r.position*count_rad;
  state_.stamp[j]=now;
}
bool BlMotorWorker::transact(unsigned j, std::uint8_t inst,
                              const std::uint8_t* p, std::size_t n,
                              std::size_t read, BusReply* out) noexcept {
  BusReply reply{}; ++state_.transactions;
  if (!bus_.request(j==2?BusFormat::Wheel:BusFormat::Joint,
                    owner_.config().id[j],inst,p,n,read,reply)) {
    recordFailure(j,inst,n?p[0]:0,bus_.lastError());
    fail(MotorError::Bus); return false;
  }
  state_.status[j]=reply.status;
  if (reply.status) { recordFailure(j,inst,n?p[0]:0,BusError::None); fail(MotorError::Status); return false; }
  if (j<2) jointSample(j,reply);
  if (out) *out=reply;
  return state_.error==MotorError::None;
}
bool BlMotorWorker::torque(unsigned j, bool enable) noexcept {
  const std::uint8_t p[2]={0x18,static_cast<std::uint8_t>(enable)};
  // OFF works after a latched fault. Verify the actual enable register rather
  // than treating a transmitted packet or a write ACK as physical release.
  if (!enable) {
    BusReply reply{}; const auto fmt=j==2?BusFormat::Wheel:BusFormat::Joint;
    const auto id=owner_.config().id[j]; ++state_.transactions;
    const bool wrote=bus_.request(fmt,id,3,p,2,0,reply);
    if (!wrote) recordFailure(j,3,0x18,bus_.lastError());
    const std::uint8_t read[2]={0x18,1}; ++state_.transactions;
    const bool read_ok=bus_.request(fmt,id,2,read,2,1,reply);
    state_.torque_enable[j]=read_ok?reply.data[0]:255;
    if (read_ok) state_.status[j]=reply.status;
    if (read_ok && !reply.data[0]) { on_[j]=false; return true; }
    on_[j]=true;
    recordFailure(j,2,0x18,read_ok?BusError::ReadbackMismatch:bus_.lastError());
    fail(read_ok?MotorError::Configuration:MotorError::Bus); return false;
  }
  on_[j]=true; // an unacknowledged enable may still have reached the driver
  state_.torque_enable[j]=255;
  return transact(j,3,p,2);
}
bool BlMotorWorker::wheelSpeed(std::int16_t rpm) noexcept {
  std::uint8_t p[3]={0x20,0,0}; put(p+1,rpm);
  BusReply reply{}; ++state_.transactions;
  if (bus_.request(BusFormat::Wheel,owner_.config().id[2],3,p,3,0,reply)) {
    state_.status[2]=reply.status;
    if (reply.status) { recordFailure(2,3,0x20,BusError::None); fail(MotorError::Status); return false; }
    return true;
  }
  const auto error=bus_.lastError();
  // BL4012e does not require a speed-write ACK. HCR additionally requires
  // evidence of the latched target: only a missing reply may use readback.
  if (error!=BusError::ReplyTimeout) {
    recordFailure(2,3,0x20,error); fail(MotorError::Bus); return false;
  }
  const std::uint8_t read[2]={0x20,2};
  if (!transact(2,2,read,2,2,&reply)) return false;
  if (static_cast<std::int16_t>(u16(reply.data.data()))!=rpm) {
    recordFailure(2,2,0x20,BusError::ReadbackMismatch); fail(MotorError::Configuration); return false;
  }
  ++state_.recovered_writes; return true;
}
bool BlMotorWorker::verifyWheelZero() noexcept {
  const std::uint8_t p[2]={0x20,2}; BusReply r{};
  if (!transact(2,2,p,2,2,&r)) return false;
  if (u16(r.data.data())!=0) { recordFailure(2,2,0x20,BusError::ReadbackMismatch); fail(MotorError::Configuration); return false; }
  wheel_zero_verified_=true; return true;
}
bool BlMotorWorker::wheelRead() noexcept {
  const std::uint8_t p[2]={0x24,6}; BusReply r{};
  if (!transact(2,2,p,2,6,&r)) return false;
  const unsigned pos=u16(r.data.data()); const auto now=bus_.now();
  if (pos>=1024) { fail(MotorError::Feedback); return false; }
  if (wheel_sample_) {
    if (!fresh(state_.stamp[2],now,owner_.config().feedback_age_us)) {
      fail(MotorError::Feedback); return false;
    }
    int delta=int(pos)-int(previous_wheel_);
    if (delta>512) delta-=1024;
    if (delta<-512) delta+=1024;
    if (std::abs(delta)==512) { fail(MotorError::Feedback); return false; }
    wheel_counts_+=delta;
  } else { wheel_counts_=pos; wheel_sample_=true; }
  previous_wheel_=pos; state_.stamp[2]=now;
  state_.raw_q[2]=wheel_counts_*(2*pi/1024.F);
  state_.raw_qd[2]=static_cast<std::int16_t>(u16(r.data.data()+2))*rpm_rad;
  return true;
}
bool BlMotorWorker::off() noexcept {
  state_.prepared=false; state_.wheel_ready=false; wheel_enabled_at_=0; wheel_zero_verified_=false;
  bool ok=true;
  // Wheel first; every motor is attempted even when the first one is absent.
  for (unsigned j:{2U,0U,1U}) if (on_[j] && !torque(j,false)) ok=false;
  state_.off=ok && !on_[0] && !on_[1] && !on_[2];
  if (state_.off) last_off_verify_=bus_.now();
  return state_.off;
}
bool BlMotorWorker::configure() noexcept {
  if (!off()) return false;
  for (unsigned j=0;j<2;++j) {
    BusReply r{}; const std::uint8_t level[2]={0x10,1}, limits[2]={0x32,4}, mode[2]={0x2d,3};
    if (!transact(j,2,level,2,1,&r)) return false;
    if (r.data[0]!=2) { fail(MotorError::Configuration); return false; }
    if (!transact(j,2,limits,2,4,&r)) return false;
    state_.kp_max[j]=u16(r.data.data()); state_.kd_max[j]=u16(r.data.data()+2);
    for (const auto& g:owner_.config().profile)
      if (!std::isfinite(g.kp) || !std::isfinite(g.kd) || g.kp<0 || g.kd<0 ||
          g.kp*1000>state_.kp_max[j] || g.kd*1000>state_.kd_max[j]) {
        fail(MotorError::Configuration); return false;
      }
    if (!transact(j,3,mode,2)) return false;
  }
  // Like the bench: disarmed setup only reads the wheel, never writes speed.
  if (!wheelRead()) return false;
  state_.configured=true; return true;
}
bool BlMotorWorker::poll(const MotorRequest* active) noexcept {
  for (unsigned j=0;j<2;++j) {
    if (active && !allowed(*active,false)) return false;
    if (!transact(j,1,nullptr,0)) return false;
  }
  if (active && !allowed(*active,false)) return false;
  return wheelRead();
}
bool BlMotorWorker::allowed(const MotorRequest& r, bool batch) noexcept {
  const auto latest=owner_.request(side_); const auto now=bus_.now();
  if (!latest.active || latest.generation!=r.generation) return false;
  if (now>=latest.lease_us || (batch && now>=r.batch.deadline_us)) {
    fail(MotorError::Expired); return false;
  }
  return true;
}
void BlMotorWorker::step() noexcept {
  const auto started=bus_.now(); auto r=owner_.request(side_);
  if (!validMotorConfig(owner_.config())) { fail(MotorError::Configuration); return; }
  if (r.reset_generation!=state_.reset_generation) {
    state_={}; state_.reset_generation=r.reset_generation;
    on_={true,true,true}; wheel_sample_=false; wheel_enabled_at_=0; wheel_zero_verified_=false;
  }
  if (!r.active && state_.generation!=r.generation) { on_={true,true,true}; state_.off=false; }
  state_.generation=r.generation;
  if (!r.active) {
    // Detect a driver reboot/external enable even when cached OFF was true.
    if (!r.maintenance && state_.off && started-last_off_verify_>=100000) { on_={true,true,true}; state_.off=false; }
    if ((!state_.off || on_[0] || on_[1] || on_[2]) &&
        (!last_off_attempt_ || started-last_off_attempt_>=1000)) {
      off(); last_off_attempt_=started;
    }
    // Once maintenance OFF is verified, stop all bus traffic during flash
    // writes. The backend's sticky latch rejects apply/reset until reboot.
    if (!r.maintenance && state_.error==MotorError::None) {
      if (!state_.configured) configure();
      else if (started-last_idle_poll_>=3000) { poll(); last_idle_poll_=started; }
    }
  } else if (allowed(r,false) && state_.configured && state_.error==MotorError::None) {
    state_.off=false;
    // Startup is not a TARGET batch. Keep zero speed while the wheel's known
    // enable-settle delay expires; new TARGET packets renew only the lease.
    const bool want_wheel=r.batch.token && state_.completed!=r.batch.token ?
      (r.batch.flags&2U)!=0 : r.wheel_enable;
    if (want_wheel && !on_[2]) {
      state_.prepared=false;
      wheel_zero_verified_=false;
      if (allowed(r,false) && wheelSpeed(0) && allowed(r,false) && torque(2,true)) {
        wheel_enabled_at_=bus_.now();
        if (allowed(r,false)) verifyWheelZero();
      }
    }
    if (!want_wheel && on_[2]) torque(2,false);
    const bool settled=!want_wheel ||
      (on_[2] && wheel_enabled_at_ && bus_.now()-wheel_enabled_at_>=owner_.config().wheel_settle_us);
    if (!state_.prepared && allowed(r,false)) {
      // HOME can ACK its zero wheel reference during settle, but only after
      // preload-0, enable and a target-register READBACK proving zero latched.
      // Nonzero speed is deferred until settle; it is never falsely ACKed.
      if (!want_wheel || wheel_zero_verified_) state_.prepared=true;
    }
    state_.wheel_ready=want_wheel && settled && state_.prepared;
    if (state_.prepared && r.batch.token && state_.completed!=r.batch.token && allowed(r,true)) {
      bool ok=true;
      for (unsigned j=0;j<2 && ok;++j) {
        if (!allowed(r,true)) { ok=false; break; }
        if (r.batch.flags&1U) {
          std::uint8_t p[11]={0x1a}; std::memcpy(p+1,r.batch.joint[j].data(),10);
          // A full MIT block auto-enables the joint. Mark unknown/on BEFORE
          // writing, so timeout/CRC failure still causes an OFF attempt.
          on_[j]=true; state_.torque_enable[j]=255; ok=transact(j,3,p,11);
        } else if (on_[j]) ok=torque(j,false);
      }
      if (ok && allowed(r,true)) {
        if (r.batch.flags&2U) {
          if (settled) ok=wheelSpeed(r.batch.wheel_rpm);
          else ok=wheel_zero_verified_ && r.batch.wheel_rpm==0;
          if (ok && r.batch.wheel_rpm!=0) wheel_zero_verified_=false;
        } else if (on_[2]) ok=torque(2,false);
      } else ok=false;
      if (ok && allowed(r,true)) ok=wheelRead();
      if (ok && allowed(r,true)) state_.completed=r.batch.token;
    } else if (allowed(r,false)) poll(&r);
  }
  // Cancellation may arrive during any bus wait; disable before publishing.
  const auto after=owner_.request(side_);
  if (!after.active || after.generation!=r.generation || state_.error!=MotorError::None) {
    if (after.generation!=r.generation) on_={true,true,true};
    if (on_[0] || on_[1] || on_[2]) off();
    state_.prepared=false; state_.generation=after.generation;
  }
  const auto duration=bus_.now()-started;
  state_.cycle_max_us=std::max(state_.cycle_max_us,static_cast<std::uint32_t>(duration));
  owner_.publish(side_,state_);
}
} // namespace tap_hw::hybrid
