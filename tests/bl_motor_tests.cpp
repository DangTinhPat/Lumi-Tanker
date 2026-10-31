#include "gateway/BlMotor.hpp"
#include "gateway/HybridGatewayCore.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace gateway;
struct Lock final:MotorCritical { void enter() noexcept override {} void leave() noexcept override {} };
struct FakeBus final:MotorBus {
  std::uint64_t& clock;
  std::array<std::int16_t,2> position{{1000,-2000}};
  std::array<bool,3> enabled{{true,true,true}};
  std::uint16_t wheel_pos=1020;
  std::int16_t wheel_rpm=0;
  std::vector<std::vector<std::uint8_t>> frames;
  std::function<void()> hook;
  int fail_id=0, fail_addr=-1, fail_inst=0;
  bool corrupt=false;
  int fail_remaining=-1;
  bool drop_speed_write=false, ignore_off=false;
  BusError error=BusError::None, injected_error=BusError::ReplyTimeout;
  std::uint8_t status=0;
  std::uint16_t gain_cap=5000;
  std::uint32_t latency=50;
  explicit FakeBus(std::uint64_t& time):clock(time) {}
  std::uint64_t now() const noexcept override { return clock; }
  BusError lastError() const noexcept override { return error; }
  bool request(BusFormat fmt,std::uint8_t id,std::uint8_t inst,const std::uint8_t* p,
               std::size_t n,std::size_t read,BusReply& out) noexcept override {
    error=BusError::None;
    if (!permitted()) {error=permit_.deadline_us && clock>=permit_.deadline_us?BusError::Deadline:BusError::Cancelled;return false;}
    std::array<std::uint8_t,48> tx{};
    const auto size=motorPacket(id,inst,p,n,tx); assert(size);
    frames.emplace_back(tx.begin(),tx.begin()+size); clock+=latency;
    if (hook) { auto f=std::move(hook);hook=nullptr;f(); }
    const unsigned j=id==2?0:id==3?1:2;
    // The physical write can succeed while its reply is lost.
    if (inst==3 && n>=2 && p[0]==0x18 && (p[1] || !ignore_off)) enabled[j]=p[1];
    if (inst==3 && n==11 && p[0]==0x1a) {
      position[j]=static_cast<std::int16_t>(p[5]|(std::uint16_t(p[6])<<8)); enabled[j]=true;
    }
    if (inst==3 && n==3 && p[0]==0x20 && !drop_speed_write) wheel_rpm=static_cast<std::int16_t>(p[1]|(std::uint16_t(p[2])<<8));
    if (fail_remaining!=0 && fail_id==id && (fail_addr<0 || (n && p[0]==fail_addr)) && (!fail_inst || fail_inst==inst)) {
      if (fail_remaining>0) --fail_remaining;
      error=injected_error; return false;
    }
    const unsigned echo=fmt==BusFormat::Wheel?5:7, count=echo+4+read;
    std::array<std::uint8_t,48> rx{};
    rx[0]=rx[1]=255;rx[2]=id;rx[3]=count-4;rx[4]=status;
    if (j<2) { rx[5]=position[j]&255;rx[6]=(std::uint16_t(position[j])>>8)&255; }
    rx[echo]=n+2;rx[echo+1]=inst;rx[echo+2]=n?p[0]:0;
    auto* data=rx.data()+echo+3;
    if (read && p[0]==0x10) data[0]=2;
    if (read && p[0]==0x18) data[0]=enabled[j];
    if (read && p[0]==0x32) { data[0]=0xff;data[1]=0xff;data[2]=gain_cap&255;data[3]=gain_cap>>8; }
    if (read && p[0]==0x20) { data[0]=wheel_rpm&255;data[1]=(std::uint16_t(wheel_rpm)>>8)&255; }
    if (read && p[0]==0x24) {
      data[0]=wheel_pos&255;data[1]=wheel_pos>>8;
      data[2]=wheel_rpm&255;data[3]=(std::uint16_t(wheel_rpm)>>8)&255;
    }
    unsigned sum=0;for(unsigned i=2;i<count-1;++i)sum+=rx[i];
    rx[count-1]=~sum;if(corrupt)rx[count-1]^=1;
    const bool ok=motorReply(fmt,id,inst,n?p[0]:0,n+2,rx.data(),count,read,out);
    if (!ok) error=BusError::ReplyInvalid;
    return ok;
  }
  const std::vector<std::uint8_t>& last(unsigned id,unsigned address) const {
    for(auto i=frames.rbegin();i!=frames.rend();++i)
      if((*i)[2]==id&&(*i)[4]==3&&(*i)[5]==address)return *i;
    assert(false);return frames[0];
  }
};
struct Rig {
  std::uint64_t now=100000;
  Lock lock;
  BlMotorBackend motor;
  FakeBus left,right;
  BlMotorWorker lw,rw;
  Rig():motor(lock),left(now),right(now),lw(motor,0,left),rw(motor,1,right) { idle(); }
  void workers() { lw.step();rw.step(); }
  void idle() { for(unsigned i=0;i<8;++i){workers();now+=1000;} }
  void home() {
    assert(motor.disabled()&&!motor.ready());
    assert(motor.captureHome({2.35706715F,2.35706715F,0,0},now)); assert(motor.ready());
  }
  HcrTarget target(unsigned profile=HCR_HOME_PD) {
    HcrTarget t{};t.epoch=7;t.profile=profile;t.flags=3;
    t.target[0]=t.target[1]=t.target[2]=t.target[3]=2.35706715F;
    return t;
  }
  MotorApplied run(HcrTarget t,unsigned& seq,unsigned iterations=80) {
    MotorApplied out{};std::uint64_t deadline=now+20000,next=now+5000;
    bool applied=false;
    for(unsigned i=0;i<iterations;++i) {
      if(now>=next){++seq;deadline=now+20000;next=now+5000;}
      const auto result=motor.apply(t,seq,deadline,now,out);
      assert(result!=ApplyResult::Fault);
      applied|=result==ApplyResult::Applied;
      workers();now+=500;
    }
    assert(applied);return out;
  }
};
int16_t signed16(const std::vector<std::uint8_t>& f,unsigned i) {
  return static_cast<std::int16_t>(f[i]|(unsigned(f[i+1])<<8));
}
void close(float a,float b,float eps=.001F) { assert(std::abs(a-b)<eps); }
HcrFrame decode(const WirePacket& packet) {
  HcrParser p{};HcrFrame f{};unsigned n=0;
  for(unsigned i=0;i<packet.size;++i)n+=hcr_feed(&p,packet.bytes[i],&f);
  assert(n==1);return f;
}
void gatewayIntegration() {
  Rig r;r.home();GatewayCore core(r.motor);
  HcrFrame f{};f.type=HCR_HELLO;f.size=12;f.seq=1;f.stamp_us=1;
  hcr_put32(f.payload,7);hcr_put16(f.payload+4,200);hcr_put16(f.payload+6,100);hcr_put32(f.payload+8,HCR_SCHEMA);
  assert(core.receive(f,r.now).size);
  f.type=HCR_STATUS;f.size=16;f.seq=2;f.stamp_us=2;hcr_put32(f.payload+4,1);hcr_put32(f.payload+8,0);
  core.receive(f,r.now);
  const auto sensor=[&](){ImuObservation imu{};imu.valid=true;imu.stamp_us=r.now;core.setImu(imu);core.tick(r.now);};
  sensor();f={};f.type=HCR_STATUS;f.size=16;f.seq=3;f.stamp_us=r.now;
  hcr_put32(f.payload,7);hcr_put32(f.payload+4,2);core.receive(f,r.now);
  unsigned seq=3;std::uint64_t next=0;bool saw_pending=false,saw_applied=false;
  const auto start=r.now;std::uint64_t first_ack=0;
  for(unsigned i=0;i<300;++i) {
    sensor();
    if(r.now>=next) {
      auto sample=decode(core.telemetry(r.now));
      core.telemetrySent(7,sample.seq,sample.stamp_us);
      if(hcr_u32(sample.payload+4)&HCR_TARGET_VALID) {
        assert(hcr_u32(sample.payload+8)<seq);saw_applied=true;if(!first_ack)first_ack=r.now;
      } else saw_pending=true;
      f={};f.type=HCR_TARGET;f.size=52;f.seq=seq++;f.stamp_us=r.now;
      hcr_put32(f.payload,7);hcr_put32(f.payload+4,sample.seq);
      hcr_put16(f.payload+8,HCR_HOME_PD);hcr_put16(f.payload+10,3);
      for(unsigned j=0;j<4;++j)hcr_putfloat(f.payload+12+j*4,2.35706715F);
      core.receive(f,r.now);next=r.now+5000;
    }
    r.workers();r.now+=500;
    assert(core.diagnostics(r.now).fault==LocalFault::None);
  }
  assert(saw_pending&&saw_applied);
  assert(first_ack-start<=15000); // zero reference is read back; nonzero still waits 25 ms
  core.estop();assert(!core.diagnostics(r.now).enabled);
  r.workers();assert(r.motor.disabled());
  for(auto on:r.left.enabled)assert(!on);
  for(auto on:r.right.enabled)assert(!on);
  std::printf("PASS: GatewayCore -> async two-bus backend -> applied ACK; nominal first ACK %llu us\n",
              static_cast<unsigned long long>(first_ack-start));
}
void peer() {
  Rig r; r.left.latency=r.right.latency=0;
  r.now=50000; r.motor.trip(MotorError::Bus); r.motor.resetFault(); r.idle();r.home();
  GatewayCore core(r.motor); HcrParser parser{};
  const auto sensor=[&](){ImuObservation imu{};imu.valid=true;imu.stamp_us=r.now;core.setImu(imu);core.tick(r.now);};
  // The reference peer starts at 100000 us; initialization is before that clock.
  std::string line;
  while(std::getline(std::cin,line)) {
    std::istringstream in(line);char op;std::uint64_t now;in>>op>>now;
    while(r.now<now) {
      r.now=std::min(now,r.now+500);r.workers();sensor();
    }
    // LF RESET preserves home, but cannot clear the local E-stop latch.
    sensor();WirePacket reply{};
    if(op=='F') {
      std::string hex;in>>hex;
      for(std::size_t i=0;i+1<hex.size();i+=2) {
        HcrFrame f{};const auto b=std::stoul(hex.substr(i,2),nullptr,16);
        if(hcr_feed(&parser,static_cast<std::uint8_t>(b),&f))reply=core.receive(f,now);
      }
    } else if(op=='T') {
      reply=core.telemetry(now);
      if(reply.size){auto f=decode(reply);core.telemetrySent(hcr_u32(f.payload),f.seq,f.stamp_us);}
    } else if(op=='C') {
      unsigned code;in>>code;HcrFrame f{};f.type=HCR_STATUS;f.size=16;f.seq=2;f.stamp_us=now;
      hcr_put32(f.payload,7);hcr_put32(f.payload+4,code==1?2:1);reply=core.receive(f,now);
    } else if(op=='E')core.estop();
    if(!reply.size)std::cout<<'-';
    else for(unsigned i=0;i<reply.size;++i){constexpr char h[]="0123456789abcdef";std::cout<<h[reply.bytes[i]>>4]<<h[reply.bytes[i]&15];}
    auto d=core.diagnostics(now);
    std::cout<<' '<<d.armed<<' '<<d.enabled<<' '<<static_cast<unsigned>(d.fault)<<' '<<d.applied_seq<<'\n'<<std::flush;
  }
}
int main(int argc,char** argv) {
  if(argc>1 && !std::strcmp(argv[1],"--peer")){peer();return 0;}

  { // Supplier's corrected golden MIT packet, independent of the mock.
    std::array<std::uint8_t,48> out{};
    const std::uint8_t p[]={0x1a,0x10,0x27,0x24,0,0,0x10,0,0,0,0};
    const std::uint8_t expected[]={255,255,1,13,3,0x1a,0x10,0x27,0x24,0,0,0x10,0,0,0,0,0x69};
    assert(motorPacket(1,3,p,sizeof p,out)==sizeof expected&&!std::memcmp(out.data(),expected,sizeof expected));
    const std::uint8_t reply[]={255,255,1,7,0,0,0x10,13,3,0x1a,0xbd};BusReply result{};
    assert(motorReply(BusFormat::Joint,1,3,0x1a,13,reply,sizeof reply,0,result));
    assert(result.position==4096);
    assert(!motorReply(BusFormat::Wheel,1,3,0x1a,13,reply,sizeof reply,0,result));
    assert(!motorPacket(0xfe,3,p,sizeof p,out));
  }
  { Rig r;assert(r.motor.disabled()&&!r.motor.ready());r.home();
    auto f=r.motor.feedback(r.now);close(f.q[0],2.35706715F);close(f.q[2],0);assert(!f.torque_valid);
    auto t=r.target();unsigned seq=1;r.run(t,seq);
    t.profile=HCR_POLICY_PD;t.target[0]=-.5;t.target[1]=-.4;t.target[2]=.6;t.target[3]=.7;
    t.target[4]=10;t.target[5]=-23;++seq;const auto out=r.run(t,seq);
    // Physical LEFT UART2: hip +, crank -, wheel +. RIGHT UART1: -, +, -.
    const auto& lh=r.left.last(2,0x1a);const auto& lc=r.left.last(3,0x1a);
    const auto& rh=r.right.last(2,0x1a);const auto& rc=r.right.last(3,0x1a);
    assert(signed16(lh,10)<1000 && signed16(lc,10)>-2000);
    assert(signed16(rh,10)>1000 && signed16(rc,10)<-2000);
    assert(signed16(r.left.last(1,0x20),6)==95);
    assert(signed16(r.right.last(1,0x20),6)==220);
    close(out.target[0],-.5);close(out.target[2],.6);close(out.target[4],9.948377F);
    close(out.target[5],-23.038347F);
    assert(signed16(lh,14)==0 && signed16(lc,14)==0); // tau bytes, not input torque
    f=r.motor.feedback(r.now);close(f.q[2],1.1);close(f.q[3],1.1);
    t.profile=HCR_RECOVERY;t.feedforward[0]=.1;t.feedforward[1]=-.2;
    t.feedforward[2]=.3;t.feedforward[3]=-.4;++seq;r.run(t,seq);
    assert(signed16(r.left.last(2,0x1a),12)==1);
    assert(signed16(r.left.last(3,0x1a),12)==-4); // phi_dot = .1 + .3, sign -
    assert(signed16(r.right.last(2,0x1a),12)==2);
    assert(signed16(r.right.last(3,0x1a),12)==-6);
    r.motor.disable();assert(!r.motor.disabled());r.workers();assert(r.motor.disabled());
    assert(r.motor.resetFault() && r.motor.ready()); // ordinary LF RESET preserves home
  }
  { // Read timestamps, not wheel write ACKs, establish freshness; unwrap 1020->5.
    Rig r;r.home();auto old=r.motor.feedback(r.now);r.left.wheel_pos=5;r.now+=4000;r.workers();
    auto f=r.motor.feedback(r.now);close(f.q[4]-old.q[4],9*2*3.141592653589793F/1024);
    r.now+=21000;assert(!r.motor.feedback(r.now).encoders_valid);
    assert(!r.motor.captureHome({0,0,0,0},r.now));
  }
  { // An idle scheduling gap is distinguished from a failed bus transaction.
    Rig r; r.now+=21000; r.workers();
    const auto d=r.motor.diagnostics();
    assert(d.error==MotorError::Feedback);
    assert(d.leg[0].feedback_error==FeedbackError::WheelGap);
    assert(d.leg[0].feedback_id==1 && d.leg[0].feedback_gap_us>20000);
    assert(!d.leg[0].feedback_active && d.leg[0].bus_error==BusError::None);
    assert(d.leg[0].feedback_previous==d.leg[0].feedback_received);
    assert(r.motor.resetFault()); r.idle();
    assert(r.motor.diagnostics().leg[0].feedback_error==FeedbackError::None);
  }
  { // A pending right bus cannot be acknowledged as an applied six-channel batch.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    ++seq;MotorApplied a{};r.motor.apply(t,seq,r.now+20000,r.now,a);
    const auto old=a.seq;r.lw.step();
    r.motor.apply(t,seq,r.now+20000,r.now,a);assert(a.seq==old);
    r.rw.step();assert(r.motor.apply(t,seq,r.now+20000,r.now,a)==ApplyResult::Applied&&a.seq==seq);
  }
  { // No clamp: impossible target must fault, disable and never ACK the new seq.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    t.target[4]=100000;++seq;MotorApplied a{};
    assert(r.motor.apply(t,seq,r.now+20000,r.now,a)==ApplyResult::Fault);
    r.workers();assert(r.motor.disabled());
  }
  { // Worker lease is independent of service execution and repeated submit calls.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.now+=21000;r.workers();assert(r.motor.diagnostics().error==MotorError::Expired);
    assert(r.motor.disabled());for(auto on:r.left.enabled)assert(!on);
  }
  { // Lost speed-write reply is accepted ONLY after matching target readback.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.left.fail_id=1;r.left.fail_addr=0x20;r.left.fail_inst=3;
    t.target[4]=3;++seq;
    const auto a=r.run(t,seq);close(a.target[4],3.036873F);
    assert(r.motor.diagnostics().error==MotorError::None);
    assert(r.motor.diagnostics().leg[0].recovered_writes>0);
  }
  { // A dropped write must not be ACKed merely because present-state reads work.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.left.fail_id=1;r.left.fail_addr=0x20;r.left.fail_inst=3;r.left.drop_speed_write=true;
    t.target[4]=3;
    ++seq;MotorApplied a{};r.motor.apply(t,seq,r.now+20000,r.now,a);r.workers();
    assert(r.motor.diagnostics().error==MotorError::Configuration && r.motor.disabled());
    assert(r.motor.diagnostics().leg[0].bus_error==BusError::ReadbackMismatch);
    for(auto on:r.left.enabled)assert(!on);
  for(auto on:r.right.enabled)assert(!on);
    r.left.fail_id=0;assert(r.motor.resetFault());r.idle();assert(!r.motor.ready());r.home();
  }
  { // Boot/disarmed must not write speed, including repeated OFF verification.
    Rig r;for(unsigned i=0;i<200;++i)r.idle(); // >1 s, includes periodic verification
    assert(r.motor.diagnostics().error==MotorError::None && r.motor.feedback(r.now).encoders_valid);
    for (auto* bus:{&r.left,&r.right}) for (const auto& f:bus->frames)
      assert(!(f[2]==1 && f[4]==3 && f[5]==0x20));
    r.left.enabled[2]=true; r.motor.emergencyStop();
    assert(!r.motor.disabled()); r.workers();
    assert(r.motor.disabled() && !r.left.enabled[2]);
    r.right.enabled[0]=true;r.now+=100000;r.workers();
    assert(!r.right.enabled[0]);
  }
  { // An ACKed OFF which the driver did not apply cannot claim disabled.
    Rig r;r.left.enabled[2]=true;r.left.ignore_off=true;r.motor.emergencyStop();r.workers();
    assert(!r.motor.disabled() && r.left.enabled[2]);
    const auto d=r.motor.diagnostics();
    assert(d.leg[0].failed_id==1 && d.leg[0].failed_address==0x18);
    assert(d.leg[0].torque_enable[2]==1 && d.leg[0].bus_error==BusError::ReadbackMismatch);
    r.left.ignore_off=false;r.now+=2000;r.workers();assert(r.motor.disabled());
  }
  { // Boot verifies speed-target readback without writing or executing a retained goal.
    std::uint64_t now=100000;Lock lock;BlMotorBackend motor(lock);FakeBus bus(now);
    bus.wheel_rpm=321;BlMotorWorker worker(motor,0,bus);worker.step();
    const auto d=motor.diagnostics();
    assert(d.leg[0].configured && d.leg[0].startup_wheel_goal_rpm==321);
    assert(!bus.enabled[2] && bus.wheel_rpm==321);
    for(const auto& f:bus.frames)assert(!(f[2]==1 && f[4]==3 && f[5]==0x20));
  }
  { // An unsupported goal READ is found before ARM, not first encountered in motion.
    std::uint64_t now=100000;Lock lock;BlMotorBackend motor(lock);FakeBus bus(now);
    bus.fail_id=1;bus.fail_addr=0x20;bus.fail_inst=2;
    BlMotorWorker worker(motor,0,bus);worker.step();
    const auto d=motor.diagnostics();
    assert(!d.leg[0].configured && d.error==MotorError::Bus && !bus.enabled[2]);
    assert(d.leg[0].failed_id==1 && d.leg[0].failed_address==0x20);
  }
  { // Maintenance verifies OFF once, then keeps bus quiet during flash writes.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.motor.beginMaintenance();assert(!r.motor.disabled());r.workers();
    assert(r.motor.disabled() && !r.motor.ready() && !r.motor.resetFault());
    const auto left_frames=r.left.frames.size(),right_frames=r.right.frames.size();
    for(unsigned i=0;i<200;++i)r.idle();
    assert(r.left.frames.size()==left_frames && r.right.frames.size()==right_frames);
    MotorApplied a{};assert(r.motor.apply(t,++seq,r.now+20000,r.now,a)==ApplyResult::Fault);
    r.workers();assert(r.motor.disabled());
    assert(!r.motor.captureHome({2.35706715F,2.35706715F,0,0},r.now));
  }
  { // Maintenance cannot freeze a false OFF state when readback failed.
    Rig r;r.left.fail_id=1;r.left.fail_addr=0x18;r.left.fail_inst=2;
    r.motor.beginMaintenance();r.workers();assert(!r.motor.disabled());
    r.left.fail_id=0;r.now+=2000;r.workers();assert(r.motor.disabled());
  }
  { // Unknown OFF readback stays unknown even if the OFF write was ACKed.
    Rig r;r.left.fail_id=1;r.left.fail_addr=0x18;r.left.fail_inst=2;
    r.motor.emergencyStop();r.workers();
    assert(!r.motor.disabled() && r.motor.diagnostics().leg[0].torque_enable[2]==255);
  }
  for (auto error:{BusError::EchoTimeout,BusError::EchoMismatch,BusError::ReplyInvalid,BusError::Tx}) {
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.left.fail_id=1;r.left.fail_addr=0x20;r.left.fail_inst=3;r.left.injected_error=error;
    t.target[4]=3;
    ++seq;MotorApplied a{};r.motor.apply(t,seq,r.now+20000,r.now,a);r.workers();
    assert(r.motor.diagnostics().error==MotorError::Bus && r.motor.disabled());
    assert(r.motor.diagnostics().leg[0].bus_error==error);
  }
  { // Missing speed ACK AND missing readback are a bus fault, never an ACK.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.left.fail_id=1;r.left.fail_addr=0x20; // both instructions fail
    t.target[4]=3;
    ++seq;MotorApplied a{};r.motor.apply(t,seq,r.now+20000,r.now,a);r.workers();
    assert(r.motor.diagnostics().error==MotorError::Bus && r.motor.disabled());
    assert(r.motor.diagnostics().leg[0].failed_instruction==2);
  }
  { // Readback recovery cannot extend the original 20 ms batch deadline.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.left.fail_id=1;r.left.fail_addr=0x20;r.left.fail_inst=3;r.left.latency=6000;
    t.target[4]=3;
    ++seq;MotorApplied a{};r.motor.apply(t,seq,r.now+20000,r.now,a);r.workers();
    assert(r.motor.diagnostics().error==MotorError::Expired && r.motor.disabled());
  }
  { // Lost enable reply must still trigger wheel OFF (enable may have arrived).
    Rig r;r.home();auto t=r.target();MotorApplied a{};
    r.left.fail_id=1;r.left.fail_addr=0x18;r.left.fail_inst=3;
    r.motor.apply(t,1,r.now+20000,r.now,a);r.workers();
    assert(r.motor.diagnostics().error==MotorError::Bus);
    assert(!r.left.enabled[2]); // OFF attempted even though its ACK also lost
    assert(r.motor.disabled()); // OFF readback succeeds despite missing write ACK
  }
  { // Stop during a MIT transaction: do not send the next joint, turn off all.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    ++seq;MotorApplied a{};r.motor.apply(t,seq,r.now+20000,r.now,a);
    r.left.hook=[&](){r.motor.disable();
      assert(!r.motor.captureHome({2.35706715F,2.35706715F,0,0},r.now));};r.workers();assert(r.motor.disabled());
    for(auto on:r.left.enabled)assert(!on);
  }
  { // Driver's gain limits are checked, never silently clamped to another servo.
    Rig r;r.left.gain_cap=36;r.motor.trip(MotorError::Bus);r.motor.resetFault();r.workers();
    assert(r.motor.diagnostics().error==MotorError::Configuration&&!r.motor.ready());
  }
  { Rig r;r.home();r.left.corrupt=true;r.now+=4000;r.workers();
    assert(r.motor.diagnostics().error==MotorError::Bus&&!r.motor.feedback(r.now).encoders_valid);
  }
  { // Nonzero startup must wait for wheel settle, even though zero HOME can ACK early.
    Rig r;r.home();auto t=r.target();t.target[4]=1;t.target[5]=-1;
    unsigned seq=1;MotorApplied a{};const auto start=r.now;
    while(r.now-start<23000) {
      const auto result=r.motor.apply(t,seq++,r.now+20000,r.now,a);
      assert(result==ApplyResult::Pending);r.workers();r.now+=500;
    }
    r.run(t,seq);assert(r.left.wheel_rpm==10 && r.right.wheel_rpm==10);
  }
  { // Individual enable bits: disabled wheels report zero, not a decaying filter;
    // disabled legs retain their last latched reference rather than claiming a write.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    t.profile=HCR_RECOVERY;t.flags=1;t.target[4]=100000;++seq;
    auto a=r.run(t,seq);close(a.target[4],0);assert(!r.left.enabled[2]);
    t.flags=2;t.target[0]=999;t.target[4]=0;++seq;
    a=r.run(t,seq);assert(!r.left.enabled[0]&&!r.left.enabled[1]&&r.left.enabled[2]);
    close(a.target[0],2.35706715F);
  }
  { Rig r;r.home();r.left.status=0x20;r.now+=4000;r.workers();
    assert(r.motor.diagnostics().error==MotorError::Status);assert(r.motor.disabled());
  }
  { // Independent numerical recovery filter: first dt=.005, tau=.075,
    // omega=10 -> .64493 rad/s -> 6 RPM -> applied .6283185 rad/s.
    Lock lock;BlMotorBackend motor(lock);MotorLegState st{};
    st.configured=st.off=st.prepared=st.wheel_ready=true;st.generation=1;
    st.stamp={100000,100000,100000};st.kp_max={65535,65535};st.kd_max={5000,5000};
    motor.publish(0,st);motor.publish(1,st);
    assert(motor.captureHome({2.35706715F,2.35706715F,0,0},100000));
    HcrTarget t{};t.epoch=7;t.profile=HCR_RECOVERY;t.flags=3;
    for(unsigned i=0;i<4;++i)t.target[i]=2.35706715F;
    t.target[4]=10;t.target[5]=20;MotorApplied a{};
    assert(motor.apply(t,1,120000,100000,a)==ApplyResult::Pending);
    assert(motor.request(0).batch.wheel_rpm==6 && motor.request(1).batch.wheel_rpm==-12);
    st.off=false;st.completed=motor.request(0).batch.token;
    motor.publish(0,st);motor.publish(1,st);
    assert(motor.apply(t,1,120000,100500,a)==ApplyResult::Applied);
    close(a.target[4],.6283185F);close(a.target[5],1.256637F);
  }
  { BlMotorConfig c;assert(validMotorConfig(c));c.sign[0][0]=0;assert(!validMotorConfig(c));
    c={};c.id[0]=c.id[1];assert(!validMotorConfig(c));
    c={};c.wheel_max_rpm=1600;assert(!validMotorConfig(c));
    c={};c.profile[0].kd=std::numeric_limits<float>::quiet_NaN();assert(!validMotorConfig(c));
  }
  { // Measured driver limits on the real hips; configuration accepts the
    // explicit .120 gain, matching the bench's capped .7 GUI request.
    Rig r;r.left.gain_cap=r.right.gain_cap=120;
    r.motor.trip(MotorError::Bus);r.motor.resetFault();r.idle();r.home();
    auto t=r.target();unsigned seq=1;r.run(t,seq);
    assert(signed16(r.left.last(2,0x1a),8)==120);
    assert(signed16(r.right.last(3,0x1a),8)==120);
  }
  { // OFF stress test writes only wheel zero; it cannot enable or retain HOME.
    Rig r;r.home();assert(!r.motor.startOffTest(0));assert(r.motor.startOffTest(20));
    assert(!r.motor.ready());assert(!r.motor.captureHome({2.35706715F,2.35706715F,0,0},r.now));
    r.left.frames.clear();r.right.frames.clear();
    for(unsigned i=0;i<25;++i){r.workers();r.now+=1000;}
    const auto diag=r.motor.diagnostics();
    assert(diag.leg[0].off_test_done==20 && diag.leg[1].off_test_done==20);
    for(auto* bus:{&r.left,&r.right}) {
      for(bool on:bus->enabled)assert(!on);
      for(const auto& f:bus->frames)if(f[4]==3)
        assert((f[5]==0x18 && f[6]==0) || (f[2]==1 && f[5]==0x20 && f[6]==0 && f[7]==0));
    }
    assert(r.motor.disabled()&&!r.motor.ready());
    r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);assert(!r.motor.startOffTest(20));
  }
  { // A single lost MIT reply is recovered without changing the absolute target.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.left.fail_id=2;r.left.fail_addr=0x1a;r.left.fail_inst=3;
    r.left.fail_remaining=1;r.left.injected_error=BusError::EchoMismatch;
    r.left.frames.clear();++seq;r.run(t,seq);
    const auto d=r.motor.diagnostics();
    assert(d.error==MotorError::None && d.leg[0].retries==1 && d.leg[0].recovered_transactions==1);
    assert(d.leg[0].failed_id==2 && d.leg[0].bus_error==BusError::EchoMismatch);
    assert(r.left.frames.size()>1 && r.left.frames[0]==r.left.frames[1]);
  }
  { // A transient wheel feedback miss is retried, with fresh feedback on success.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.left.fail_id=1;r.left.fail_addr=0x24;r.left.fail_inst=2;r.left.fail_remaining=1;
    ++seq;r.run(t,seq);
    assert(r.motor.diagnostics().leg[0].recovered_transactions==1);
    assert(r.motor.feedback(r.now).encoders_valid);
  }
  { // Cancellation during the failed exchange must suppress the retry, but is
    // a normal DISARM, not a latched driver fault requiring re-homing.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.left.fail_id=2;r.left.fail_addr=0x1a;r.left.fail_inst=3;r.left.fail_remaining=1;
    r.left.hook=[&](){r.motor.disable();};r.left.frames.clear();
    MotorApplied a{};r.motor.apply(t,++seq,r.now+20000,r.now,a);r.workers();
    unsigned mit=0;for(const auto& f:r.left.frames)if(f[4]==3 && f[5]==0x1a)++mit;
    assert(mit==1 && r.motor.disabled() && r.motor.diagnostics().error==MotorError::None);
  }
  { // No retry or next joint TX after the first transaction used the lease.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.left.fail_id=2;r.left.fail_addr=0x1a;r.left.fail_inst=3;r.left.fail_remaining=1;
    r.left.latency=21000;r.left.frames.clear();MotorApplied a{};
    r.motor.apply(t,++seq,r.now+20000,r.now,a);r.workers();
    unsigned mit=0;for(const auto& f:r.left.frames)if(f[4]==3 && f[5]==0x1a)++mit;
    assert(mit==1 && r.motor.disabled() && r.motor.diagnostics().error==MotorError::Expired);
  }
  { // Stable RPM avoids repeated writes but never decimates encoder reads.
    Rig r;r.home();auto t=r.target();unsigned seq=1;r.run(t,seq);
    r.left.frames.clear();r.run(t,seq,400);
    unsigned writes=0,reads=0;
    for(const auto& f:r.left.frames){
      if(f[2]==1 && f[4]==3 && f[5]==0x20)++writes;
      if(f[2]==1 && f[4]==2 && f[5]==0x24)++reads;
    }
    assert(writes>0 && reads>10*writes && r.motor.diagnostics().leg[0].skipped_writes>10);
    assert(r.motor.feedback(r.now).encoders_valid);
    // A changed RPM bypasses the keepalive interval and is actually sent/ACKed.
    t.target[4]=3;++seq;auto a=r.run(t,seq);close(a.target[4],3.036873F);
    assert(signed16(r.left.last(1,0x20),6)==29);
    t.target[4]=4;++seq;a=r.run(t,seq);assert(signed16(r.left.last(1,0x20),6)==38);
    // A new enabled session must preload/verify zero again.
    r.motor.disable();r.workers();r.left.frames.clear();t.target[4]=0;++seq;r.run(t,seq);
    bool zero=false;for(const auto& f:r.left.frames)
      if(f[2]==1 && f[4]==3 && f[5]==0x20 && signed16(f,6)==0)zero=true;
    assert(zero);
  }
  std::puts("PASS: bounded retry, first failure retained, cancellation/expiry and wheel dedup/keepalive/freshness");
  gatewayIntegration();
  std::puts("PASS: golden codec, calibrated side/sign/units, crank FF, quantized applied targets, startup, both-bus ACK, unwrap/freshness, range, expiry, cancellation, write/enable loss, reset, gain/status/CRC faults");
}
