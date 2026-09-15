#include <cmath>
#include <iostream>
#include "robot_control/EmbeddedController.hpp"

using namespace robot_control;
namespace {
int checks = 0, failures = 0;
void check(bool ok, const char* label) {
  ++checks;
  if (!ok) { ++failures; std::cerr << "FAIL " << label << '\n'; }
}
bool near(float a, float b) { return std::abs(a-b) < 1e-5F; }
bool safe(const MotorCommand& out, const JointArray& q) {
  if (out.enable_motors || out.position_rad != q) return false;
  for (std::size_t i=0; i<kJointCount; ++i) {
    if (!std::isfinite(out.position_rad[i]) || out.velocity_rad_s[i] != 0 ||
        out.torque_nm[i] != 0 || out.kp_scale[i] != 0 || out.kd_scale[i] != 0) return false;
  }
  return true;
}
EmbeddedController::Config profile() {
  EmbeddedController::Config c;
  c.hip_min_rad = -.6F; c.hip_max_rad = .8F;
  c.knee_min_rad = -1.2F; c.knee_max_rad = .2F;
  c.state_machine.enable_cycles = 2;
  c.mpc.leg_length_m = .234F;
  c.mpc.pendulum_length_per_com_height = 1.077F;
  c.mpc.nb_timesteps = 20;
  return c;
}
}
int main() {
  const auto expanded = profile();
  const EmbeddedController::Config defaults;
  check(near(defaults.hip_min_rad, -.2F) && near(defaults.hip_max_rad, 1.F) &&
        near(defaults.knee_min_rad, -1.2F) && near(defaults.knee_max_rad, 0.F),
        "standalone default caps unchanged by candidate profile");
  auto standard = expanded;
  standard.hip_max_rad = .6F; standard.knee_min_rad = -.8F;
  const JointArray deep{.75838357F, -1.2F, 0, .75838357F, -1.2F, 0};
  RobotState state; state.joints.position_rad = deep;
  ControlCommand command; command.target_joint_position_rad = deep; command.request_enable = true;
  EmbeddedController normal(standard), candidate(expanded);
  normal.reset(state); candidate.reset(state);
  const auto limited = normal.update(state, command);
  auto out = candidate.update(state, command);
  check(near(limited.position_rad[0], .6F) && near(limited.position_rad[1], -.8F),
        "ordinary runtime profile still clamps deep targets");
  check(out.position_rad == deep, "candidate accepts symmetric deepest endpoint without clipping");
  command.request_enable = false;
  for (int i=0; i<5; ++i) out = candidate.update(state, command);
  check(candidate.mode() == ControlMode::Balance && out.enable_motors,
        "candidate enables at deep synthetic measured pose");
  check(std::abs(candidate.wholeBodyState().com_offset_x_m) < 1e-6F &&
        near(candidate.wholeBodyState().com_height_m, .157862115F),
        "deep endpoint has expected neutral COM and height");
  MpcBalancer mpc(expanded.mpc);
  mpc.setComHeight(candidate.wholeBodyState().com_height_m);
  check(near(mpc.activePendulumLengthM(), 1.077F*.157862115F) &&
        mpc.activePendulumLengthM() > expanded.mpc.min_pendulum_length_m,
        "candidate deepest pose does not hit unchanged .15 pendulum floor");
  command.target_joint_position_rad = {.9F, -1.4F, 0, .9F, -1.4F, 0};
  out = candidate.update(state, command);
  check(near(out.position_rad[0], .8F) && near(out.position_rad[1], -1.2F) &&
        near(out.position_rad[3], .8F) && near(out.position_rad[4], -1.2F),
        "candidate still clamps targets outside profile");

  // Exercise reset and e-stop at endpoint and intermediate deep native poses.
  for (const JointArray q : {JointArray{.65023F,-1.F,0,.65023F,-1.F,0}, deep}) {
    state = {}; state.joints.position_rad = q;
    command = {}; command.target_joint_position_rad = q;
    candidate.reset(state);
    check(safe(candidate.update(state, command), q), "deep reset is disabled and preserves measured position");
    command.request_enable = true;
    out = candidate.update(state, command);
    command.request_enable = false;
    for (int i=0; i<4; ++i) out = candidate.update(state, command);
    check(candidate.mode() == ControlMode::Balance && out.position_rad == q, "explicit deep enable maintains pose");
    command.request_pose_change = true;
    command.target_joint_position_rad = expanded.stand_up.stand_pose_rad;
    (void)candidate.update(state, command);
    command.request_pose_change = false;
    state.emergency_stop = true;
    check(safe(candidate.update(state, command), q), "estop interrupts deep-to-neutral transition");
    state.emergency_stop = false;
    for (int i=0; i<4; ++i) out = candidate.update(state, command);
    check(candidate.mode() == ControlMode::EmergencyStop && safe(out,q), "clearing deep estop does not auto-resume");
    candidate.reset(state);
    check(safe(candidate.update(state, command), q), "reset cancels deep transition without replay");
    command.target_joint_position_rad = q; command.request_enable = true;
    (void)candidate.update(state, command); command.request_enable = false;
    for (int i=0; i<4; ++i) out = candidate.update(state, command);
    check(out.enable_motors && out.position_rad == q, "explicit restart has no stale neutral trajectory");
  }
  auto deepest_config = profile();
  deepest_config.hip_max_rad = 1.05F;
  deepest_config.knee_min_rad = -1.6F;
  deepest_config.mpc.min_pendulum_length_m = .125F;
  EmbeddedController deepest(deepest_config);
  const JointArray deepest_q{.97921F,-1.6F,0,.97921F,-1.6F,0};
  state = {}; state.joints.position_rad = deepest_q;
  command = {}; command.target_joint_position_rad = deepest_q;
  deepest.reset(state);
  check(safe(deepest.update(state,command), deepest_q), "1.6 profile reset is motor-off");
  command.request_enable = true;
  (void)deepest.update(state,command); command.request_enable = false;
  for (int i=0;i<4;++i) out = deepest.update(state,command);
  check(out.enable_motors && out.position_rad == deepest_q && deepest.mode() == ControlMode::Balance,
        "1.6 profile supports endpoint without clipping");
  const auto wb = deepest.wholeBodyState();
  check(std::abs(wb.com_offset_x_m)<1e-6F && near(wb.com_height_m,.12761F), "1.6 endpoint FK neutral and finite");
  MpcBalancer deepest_mpc(deepest_config.mpc);
  deepest_mpc.setComHeight(wb.com_height_m);
  check(near(deepest_mpc.activePendulumLengthM(),1.077F*wb.com_height_m) &&
        deepest_mpc.activePendulumLengthM()>.125F && deepest_mpc.activePendulumLengthM()<.15F,
        "1.6 profile schedules below old floor but above explicit .125 floor");
  command.target_joint_position_rad = {1.2F,-1.8F,0,1.2F,-1.8F,0};
  out = deepest.update(state,command);
  check(near(out.position_rad[0],1.05F) && near(out.position_rad[1],-1.6F), "1.6 profile bounds enforced");
  command.target_joint_position_rad = expanded.stand_up.stand_pose_rad;
  command.request_pose_change = true;
  (void)deepest.update(state,command); command.request_pose_change = false;
  state.emergency_stop = true;
  check(safe(deepest.update(state,command),deepest_q), "1.6 estop interrupts return");
  state.emergency_stop = false;
  for (int i=0;i<4;++i) out = deepest.update(state,command);
  check(safe(out,deepest_q) && deepest.mode()==ControlMode::EmergencyStop, "1.6 no spontaneous restart");
  deepest.reset(state);
  check(safe(deepest.update(state,command),deepest_q), "1.6 reset clears trajectory while disabled");
  command.target_joint_position_rad=deepest_q; command.request_enable=true;
  (void)deepest.update(state,command); command.request_enable=false;
  for (int i=0;i<4;++i) out=deepest.update(state,command);
  check(out.enable_motors && out.position_rad==deepest_q, "1.6 explicit restart has no stale return");
  EmbeddedController original_candidate(profile());
  original_candidate.reset(state); command.request_enable=true;
  out=original_candidate.update(state,command);
  check(near(out.position_rad[0],.8F) && near(out.position_rad[1],-1.2F) &&
        near(EmbeddedController::Config{}.mpc.min_pendulum_length_m,.15F),
        "1.2 profile and production default floor remain unchanged");
  std::cout << checks-failures << '/' << checks << " deep squat checks passed (synthetic inputs only)\n";
  return failures ? 1 : 0;
}
