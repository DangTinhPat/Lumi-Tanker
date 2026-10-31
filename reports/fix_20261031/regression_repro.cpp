#include "tap_hw/BlMotor.hpp"
#include "tap_hw/HybridGatewayCore.hpp"
#include <cassert>
#include <cstdio>
using namespace tap_hw::hybrid;
struct Lock:MotorCritical { void enter() noexcept override {} void leave() noexcept override {} };
struct Bus:MotorBus {
  unsigned speed_writes=0; mutable uint64_t clock=100000;
  uint64_t now() const noexcept override { return clock; }
  bool request(BusFormat,uint8_t,uint8_t inst,const uint8_t* p,size_t n,size_t,BusReply& r) noexcept override {
    clock+=50;r={};
    if(inst==3 && n && p[0]==0x20) ++speed_writes;
    if(inst==2 && n && p[0]==0x10)r.data[0]=2;
    if(inst==2 && n && p[0]==0x32)r.data={255,255,255,255};
    return true;
  }
};
int main() {
  Lock l;BlMotorBackend motors(l);Bus bus;BlMotorWorker worker(motors,0,bus);worker.step();
  UnavailableMotors unavailable;GatewayCore core(unavailable);core.estop();
  HcrFrame f{};f.type=HCR_HELLO;f.size=12;f.seq=1;f.stamp_us=1;
  hcr_put32(f.payload,7);hcr_put16(f.payload+4,200);hcr_put16(f.payload+6,100);hcr_put32(f.payload+8,HCR_SCHEMA);
  core.receive(f,100000);
  const bool latched=core.diagnostics(100000).fault==LocalFault::Estop;
  GravityEstimator imu;ImuObservation out;
  for(unsigned i=1;i<=2000;++i)out=imu.update({.28105F,.10703F,9.82711F},{.00809F,-.22010F,.01451F},i*2000);
  assert(!out.valid);
#ifdef FIXED
  imu.calibrateStationary();
  for(unsigned i=1;i<=500;++i)out=imu.update({.28105F,.10703F,9.82711F},{.00809F,-.22010F,.01451F},i*2000);
  assert(out.valid && bus.speed_writes==0 && latched);
#else
  assert(bus.speed_writes==1 && !latched);
#endif
  std::printf("boot_speed_writes=%u estop_survives_hello=%d calibrated_imu_valid=%d\n",bus.speed_writes,latched,out.valid);
}
