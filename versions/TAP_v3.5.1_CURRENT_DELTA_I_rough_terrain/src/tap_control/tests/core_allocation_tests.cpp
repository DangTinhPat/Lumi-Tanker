#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <new>

#include "robot_control/EmbeddedController.hpp"
#include "robot_control/JumpController.hpp"
#include "robot_control/StationKeeper.hpp"

namespace {
// Single-threaded executable: logging and harness setup stay outside each gate.
bool counting = false;
std::size_t allocations = 0;
int failures = 0;
volatile double checksum = 0;

void* allocate(std::size_t size) {
  if (counting) ++allocations;
  if (void* p = std::malloc(size ? size : 1)) return p;
  throw std::bad_alloc();
}

void* allocateAligned(std::size_t size, std::size_t alignment) {
  if (counting) ++allocations;
  size = size ? size : 1;
  if (size > std::numeric_limits<std::size_t>::max() - (alignment - 1))
    throw std::bad_alloc();
  const auto rounded = (size + alignment - 1) / alignment * alignment;
  if (void* p = std::aligned_alloc(alignment, rounded)) return p;
  throw std::bad_alloc();
}

void check(bool ok, const char* name) {
  std::printf("%s %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok) ++failures;
}

template<class Function>
void noHeap(const char* name, Function function) {
  allocations = 0;
  counting = true;
  try {
    function();
  } catch (...) {
    counting = false;
    check(false, name);
    return;
  }
  counting = false;
  std::printf("%s allocations=%zu\n", name, allocations);
  check(allocations == 0, name);
}
}  // namespace

void* operator new(std::size_t n) { return allocate(n); }
void* operator new[](std::size_t n) { return allocate(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void* operator new(std::size_t n, const std::nothrow_t&) noexcept {
  try { return allocate(n); } catch (...) { return nullptr; }
}
void* operator new[](std::size_t n, const std::nothrow_t&) noexcept {
  try { return allocate(n); } catch (...) { return nullptr; }
}
void operator delete(void* p, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete[](void* p, const std::nothrow_t&) noexcept { std::free(p); }
void* operator new(std::size_t n, std::align_val_t a) {
  return allocateAligned(n, static_cast<std::size_t>(a));
}
void* operator new[](std::size_t n, std::align_val_t a) {
  return allocateAligned(n, static_cast<std::size_t>(a));
}
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void* operator new(std::size_t n, std::align_val_t a, const std::nothrow_t&) noexcept {
  try { return allocateAligned(n, static_cast<std::size_t>(a)); }
  catch (...) { return nullptr; }
}
void* operator new[](std::size_t n, std::align_val_t a, const std::nothrow_t&) noexcept {
  try { return allocateAligned(n, static_cast<std::size_t>(a)); }
  catch (...) { return nullptr; }
}
void operator delete(void* p, std::align_val_t, const std::nothrow_t&) noexcept {
  std::free(p);
}
void operator delete[](void* p, std::align_val_t, const std::nothrow_t&) noexcept {
  std::free(p);
}

int main() {
  using namespace robot_control;
  std::printf("sizeof: EmbeddedController=%zu LqrBalancer=%zu "
              "JumpController=%zu StationKeeper=%zu RobotState=%zu MotorCommand=%zu\n",
              sizeof(EmbeddedController), sizeof(LqrBalancer),
              sizeof(JumpController), sizeof(StationKeeper), sizeof(RobotState),
              sizeof(MotorCommand));
  counting = true;
  void* scalar = ::operator new(8);
  void* array = ::operator new[](8);
  void* aligned = ::operator new(64, std::align_val_t{64});
  counting = false;
  ::operator delete(scalar);
  ::operator delete[](array);
  ::operator delete(aligned, std::align_val_t{64});
  check(allocations == 3, "allocation hooks detect scalar, array and aligned new");

  RobotState state;
  state.joints.position_rad = JumpController::neutralPose(0);
  ControlCommand command;
  command.target_joint_position_rad = state.joints.position_rad;

  // Placement construction excludes caller-owned object storage, not constructors.
  alignas(LqrBalancer) unsigned char lqr_storage[sizeof(LqrBalancer)];
  LqrBalancer* lqr = nullptr;
  LqrBalancer::Config lqr_config;
  noHeap("LQR construction", [&] {
    lqr = ::new (lqr_storage) LqrBalancer(lqr_config);
  });
  if (!lqr) return 1;
  noHeap("LQR reset", [&] { for (int i=0; i<32; ++i) lqr->reset(); });
  noHeap("LQR nonzero updates", [&] {
    for (int i=0; i<128; ++i) {
      state.imu.pitch_rad = (i%2 ? .03F : -.02F);
      command.target_forward_velocity_m_s = .05F;
      checksum += lqr->update(state, command).ground_velocity_m_s;
    }
  });
  bool rebuilt = true;
  noHeap("LQR CoM rebuild and update", [&] {
    for (int i=0; i<16; ++i) {
      const float before = lqr->activePendulumLengthM();
      lqr->setComHeight(i%2 ? .24F : .16F);
      rebuilt = rebuilt && std::abs(lqr->activePendulumLengthM()-before) > .003F;
      checksum += lqr->update(state, command).ground_velocity_m_s;
    }
  });
  check(rebuilt, "each scheduled CoM change updates active pendulum length");
  noHeap("LQR airborne and invalid input", [&] {
    state.floor_contact = false;
    checksum += lqr->update(state, command).ground_velocity_m_s;
    state.dt_s = std::numeric_limits<float>::quiet_NaN();
    checksum += lqr->update(state, command).fallen;
  });
  lqr->~LqrBalancer();

  state = RobotState{};
  state.joints.position_rad = JumpController::neutralPose(0);
  command = ControlCommand{};
  command.target_joint_position_rad = state.joints.position_rad;
  alignas(EmbeddedController) unsigned char embedded_storage[sizeof(EmbeddedController)];
  EmbeddedController* embedded = nullptr;
  EmbeddedController::Config embedded_config;
  embedded_config.state_machine.enable_cycles = 1;
  noHeap("EmbeddedController construction", [&] {
    embedded = ::new (embedded_storage) EmbeddedController(embedded_config);
  });
  if (!embedded) return 1;
  noHeap("EmbeddedController reset", [&] { embedded->reset(state); });
  bool balanced = false;
  noHeap("EmbeddedController enable, balance, drive and live-pose rebuild", [&] {
    command.request_enable = true;
    for (int i=0; i<160; ++i) {
      command.target_forward_velocity_m_s = i > 80 ? .04F : 0.F;
      if (i%40 == 0)
        state.joints.position_rad = JumpController::neutralPose(i%80 ? -.8F : 0.F);
      const auto out = embedded->update(state, command);
      checksum += out.velocity_rad_s[2];
      balanced = balanced || embedded->mode() == ControlMode::Balance;
      command.request_enable = false;
    }
  });
  check(balanced, "EmbeddedController reached balance");
  noHeap("EmbeddedController pose, recovery and stop", [&] {
    command.request_pose_change = true;
    checksum += embedded->update(state, command).enable_motors;
    command.request_pose_change = false;
    embedded->reset(state);
    command.request_stand_up = true;
    for (int i=0; i<600; ++i) {
      const auto out = embedded->update(state, command);
      state.joints.position_rad = out.position_rad;
      checksum += out.enable_motors;
      command.request_stand_up = false;
    }
    state.emergency_stop = true;
    checksum += embedded->update(state, command).enable_motors;
  });
  embedded->~EmbeddedController();

  alignas(JumpController) unsigned char jump_storage[sizeof(JumpController)];
  JumpController* jump = nullptr;
  JumpController::Config jump_config;
  jump_config.enabled = true;
  jump_config.landing_world_frame = true;
  jump_config.landing_compression = true;
  jump_config.smooth_phase_transitions = true;
  noHeap("JumpController construction", [&] {
    jump = ::new (jump_storage) JumpController(jump_config);
  });
  if (!jump) return 1;
  noHeap("JumpController reset", [&] { jump->reset(); });
  JumpController::Input input;
  input.position_rad = JumpController::neutralPose(0);
  input.eligible = input.sensors_fresh = input.contacts_fresh = true;
  input.left_contact = input.right_contact = true;
  input.com_z_m = .217;
  unsigned phases = 0;
  int thrust_ticks = 0, flight_ticks = 0;
  noHeap("JumpController full synthetic phase sequence", [&] {
    for (int i=0; i<1800; ++i) {
      input.request = i == 40;
      if (jump->phase() == JumpController::Phase::Thrust && ++thrust_ticks > 20)
        input.left_contact = input.right_contact = false;
      if (jump->phase() == JumpController::Phase::Flight && ++flight_ticks > 30)
        input.left_contact = input.right_contact = true;
      input.airborne = !input.left_contact && !input.right_contact;
      input.ballistic = input.airborne;
      const auto out = jump->update(input);
      phases |= 1U << static_cast<unsigned>(out.phase);
      input.position_rad = out.position_rad;
      checksum += out.position_rad[0];
    }
  });
  check((phases & 0x7fU) == 0x7fU, "jump visited Idle through Complete");
  noHeap("JumpController abort and reset", [&] {
    jump->abort("allocation_test");
    checksum += jump->update(input).stop_motors;
    jump->reset();
  });
  jump->~JumpController();

  alignas(StationKeeper) unsigned char station_storage[sizeof(StationKeeper)];
  StationKeeper* station = nullptr;
  noHeap("StationKeeper construction", [&] {
    station = ::new (station_storage) StationKeeper();
  });
  if (!station) return 1;
  noHeap("StationKeeper reset", [&] { station->reset(); });
  bool held = false;
  noHeap("StationKeeper acquire, hold, drive, stale and reset", [&] {
    StationKeeper::Input in;
    in.dt = .005;
    in.active = true;
    for (int i=0; i<512; ++i) {
      in.operator_driving = i%64 == 0;
      in.x = .001*(i%20);
      in.yaw = .01*(i%10);
      const auto out = station->update(in);
      held = held || out.holding;
      checksum += out.forward_correction_m_s + out.yaw_correction_rad_s;
    }
    in.dt = std::numeric_limits<double>::quiet_NaN();
    checksum += station->update(in).holding;
    station->reset();
  });
  check(held, "StationKeeper acquired an anchor");
  station->~StationKeeper();
  JumpPlan plan;
  plan.count=2;plan.thrust_duration_s=.2F;plan.flight_duration_s=.2F;
  for(std::size_t i=0;i<2;++i) {
    const float knee=i?-.6F:-1.2F;
    plan.samples[i].time_s=i?.8F:0.F;
    plan.samples[i].hip_rad=JumpController::neutralPose(knee)[0];
    plan.samples[i].knee_rad=knee;
    plan.samples[i].base_z_m=.2F;
    plan.samples[i].feedback_gain[2]=1.F;
  }
  check(plan.valid(),"fixed-storage feedback plan validates");
  noHeap("planned Hermite references and 36 feedback gains",[&] {
    for(int i=0;i<10000;++i) {
      const auto ref=plan.sample(float(i%1000)*.001F);
      checksum+=ref.hip_rad+ref.feedback_gain[2];
    }
  });
  JumpController::Config planned_config;
  planned_config.enabled=true;planned_config.plan=&plan;
  planned_config.max_joint_speed_rad_s=28.274334F;
  planned_config.crouch_duration_s=.1F;
  JumpController planned(planned_config);
  bool planned_thrust=false,planned_flight=false,planned_land=false,planned_complete=false;
  bool planned_feedback=false,planned_safe=true;
  noHeap("planned jump controller updates",[&] {
    JumpController::Input in;
    in.dt_s=.005F;in.eligible=in.sensors_fresh=in.contacts_fresh=true;
    in.left_contact=in.right_contact=true;
    in.body_z_m=in.com_z_m=.2;
    in.position_rad=JumpController::neutralPose(0);
    JumpController::Output out;
    for(int i=0;i<1000;++i) {
      in.request=i==30;
      if(i>30)in.position_rad=out.position_rad;
      in.pitch_rad=out.phase==JumpController::Phase::Thrust ||
          out.phase==JumpController::Phase::Flight || out.phase==JumpController::Phase::Land?.02F:0.F;
      const float t=out.plan_elapsed_s+.005F;
      in.left_contact=in.right_contact=!(t>=.2F&&t<.4F);
      in.airborne=!in.left_contact&&!in.right_contact;
      in.ballistic=in.airborne;
      out=planned.update(in);
      planned_safe=planned_safe&&!out.stop_motors;
      planned_thrust=planned_thrust||out.phase==JumpController::Phase::Thrust;
      planned_flight=planned_flight||out.phase==JumpController::Phase::Flight;
      planned_land=planned_land||out.phase==JumpController::Phase::Land;
      planned_complete=planned_complete||out.phase==JumpController::Phase::Complete;
      if(out.phase==JumpController::Phase::Thrust || out.phase==JumpController::Phase::Flight ||
         out.phase==JumpController::Phase::Land) {
        planned_feedback=planned_feedback||(in.pitch_rad==.02F&&out.leg_kp_scale==0.F&&
            out.override_wheel_speed&&out.wheel_speed_is_relative&&
            std::abs(out.feedforward_nm[0]+.02F)<1e-6F&&
            std::abs(out.feedforward_nm[3]+.02F)<1e-6F);
      }
      checksum+=out.feedforward_nm[0];
    }
  });
  check(planned_thrust&&planned_flight&&planned_land&&planned_complete,
        "allocation gate actually traversed planned thrust flight landing and completion");
  check(planned_feedback,"allocation gate exercised nonzero bilateral planned feedback");
  check(planned_safe,"planned allocation sequence stayed fault-free");
  check(std::isfinite(checksum), "observed outputs remain finite");
  std::printf("failures=%d checksum=%g\n", failures, static_cast<double>(checksum));
  return failures ? 1 : 0;
}
