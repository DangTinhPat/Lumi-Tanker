#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>
#include "../tests/mpc_reference/MpcBalancer.hpp"
#include "robot_control/EmbeddedController.hpp"
#include "robot_control/MpcBalancer.hpp"
#include "robot_control/JumpController.hpp"
#include "robot_control/WholeBodyModel.hpp"

template<class Function>
void measure(const char* name, int count, Function function) {
  double checksum=0;
  const auto start=std::chrono::steady_clock::now();
  for(int i=0;i<count;++i) checksum+=function(i);
  const auto elapsed=std::chrono::duration<double,std::micro>(
      std::chrono::steady_clock::now()-start).count();
  std::cout<<name<<" count="<<count<<" mean_us="<<elapsed/count
           <<" checksum="<<checksum<<'\n';
}

template<class Prepare, class Function>
void measureTail(const std::string& name, int count, Prepare prepare,
                 Function function) {
  double checksum = 0.0;
  for (int i = 0; i < 64; ++i) {
    prepare(i);
    checksum += function(i);
  }
  std::vector<double> times(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    prepare(i + 64);
    const auto start = std::chrono::steady_clock::now();
    const double result = function(i + 64);
    const auto end = std::chrono::steady_clock::now();
    times[static_cast<std::size_t>(i)] =
        std::chrono::duration<double, std::micro>(end - start).count();
    checksum += result;
  }
  const double mean = std::accumulate(times.begin(), times.end(), 0.0) / count;
  std::sort(times.begin(), times.end());
  const auto percentile = [&](double p) {
    return times[static_cast<std::size_t>(std::ceil(p * count)) - 1];
  };
  std::cout << name << " count=" << count << " mean_us=" << mean
            << " p50_us=" << percentile(0.5) << " p95_us=" << percentile(0.95)
            << " p99_us=" << percentile(0.99) << " max_us=" << times.back()
            << " checksum=" << checksum << '\n';
}

template<class Balancer>
void benchmarkMpc(const char* implementation, int horizon) {
  typename Balancer::Config config;
  config.nb_timesteps = horizon;
  config.solver_iterations = 120;
  auto mpc = std::make_unique<Balancer>(config);
  robot_control::RobotState state;
  robot_control::ControlCommand command;
  float pitch_ref = 0.0F;
  const auto prepare = [&](int i) {
    const float t = static_cast<float>(i);
    state.dt_s = 0.005F;
    state.floor_contact = true;
    state.wheel.position_m = 0.03F * std::sin(0.031F * t);
    state.wheel.velocity_m_s = 0.04F * std::cos(0.027F * t);
    pitch_ref = 0.02F * std::sin(0.019F * t);
    state.imu.pitch_rad = pitch_ref + 0.035F + 0.01F * std::sin(0.043F * t);
    state.imu.pitch_rate_rad_s = 0.08F * std::cos(0.037F * t);
    command.target_forward_velocity_m_s = 0.06F * std::sin(0.017F * t);
    command.target_yaw_rate_rad_s = 0.2F * std::cos(0.023F * t);
  };
  const auto update = [&](int) {
    const auto output = mpc->update(state, command, true, pitch_ref, false);
    const auto debug = mpc->debugState();
    if (debug.solver_iterations != 120 || output.fallen) {
      std::cerr << "MPC benchmark did not execute the full solver\n";
      std::exit(EXIT_FAILURE);
    }
    return static_cast<double>(output.left_wheel_velocity_rad_s) +
           output.right_wheel_velocity_rad_s + debug.optimal_command_m_s +
           debug.horizon_end_pitch_rad;
  };
  const std::string prefix = std::string(implementation) + "_mpc_n" + std::to_string(horizon);
  mpc->setComHeight(0.20F);
  measureTail(prefix + "_steady_update", 1000, prepare, update);
  // Alternating clamped lengths always exceeds the 3 mm rebuild threshold.
  // These rows exclude construction, allocation, input generation and sorting.
  measureTail(prefix + "_rebuild_only", 200, [](int) {}, [&](int i) {
    mpc->setComHeight(i % 2 == 0 ? 0.16F : 0.23F);
    return mpc->activePendulumLengthM();
  });
  measureTail(prefix + "_rebuild_and_update", 200, prepare, [&](int i) {
    mpc->setComHeight(i % 2 == 0 ? 0.16F : 0.23F);
    return update(i);
  });
}

int main() {
  using namespace robot_control;
  std::cout << std::fixed << std::setprecision(3)
            << "Host steady_clock timings, warm caches, 120 MPC iterations; "
               "not ESP32-S3 WCET or hard-real-time evidence.\n"
            << "sizeof_MpcBalancer_bytes=" << sizeof(MpcBalancer)
            << " compiled_max_horizon=" << MpcBalancer::kMaxHorizon
            << " sizeof_reference_MpcBalancer_bytes=" << sizeof(reference_control::MpcBalancer)
            << " sizeof_EmbeddedController_bytes=" << sizeof(EmbeddedController)
            << " (object storage only; excludes stack/code/allocator)\n";
  for (const int horizon : {4, 20, 30, 80}) {
    if (horizon > MpcBalancer::kMaxHorizon) continue;
    benchmarkMpc<reference_control::MpcBalancer>("reference", horizon);
    benchmarkMpc<MpcBalancer>("current", horizon);
  }
  WholeBodyModel model;
  measure("whole_body_fk",200000,[&](int i) {
    const float knee=-1.6F+1.8F*(i%1000)/999.F;
    return model.compute(.3F,knee,.4F,knee+.01F).com_offset_x_m;
  });
  measure("neutral_pose",100000,[](int i) {
    return JumpController::neutralPose(-1.6F+1.8F*(i%1000)/999.F)[0];
  });
  JumpController jump;
  JumpController::Input input;
  input.position_rad=JumpController::neutralPose(0);
  input.com_z_m=.217;
  input.eligible=input.sensors_fresh=input.contacts_fresh=true;
  input.left_contact=input.right_contact=true;
  measure("jump_idle",100000,[&](int) {return jump.update(input).position_rad[0];});
}
