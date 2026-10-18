#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "robot_control/StationKeeper.hpp"

using robot_control::StationKeeper;
namespace {
int failures = 0;
int checks = 0;
constexpr double pi = 3.14159265358979323846;
void check(bool ok, const char* label) {
  ++checks;
  if (!ok) { ++failures; std::cerr << "FAIL " << label << '\n'; }
}
bool near(double a, double b) { return std::abs(a-b) < 1e-10; }
bool cleared(const StationKeeper::Output& out) {
  return !out.holding && out.forward_correction_m_s == 0 &&
      out.yaw_correction_rad_s == 0 && out.ex == 0 && out.ey == 0 && out.yawerror == 0;
}
void poseTests() {
  StationKeeper keeper;
  StationKeeper::Input in{1, 2, 0, .005, true, false};
  auto out = keeper.update(in);
  check(out.holding && near(out.ex, 0) && near(out.forward_correction_m_s, 0), "first activation captures anchor");
  in.x += .05;
  out = keeper.update(in);
  check(near(out.ex, -.05) && near(out.forward_correction_m_s, -.04), "forward displacement commands return");
  in.x = 1; in.y = 2.05;
  out = keeper.update(in);
  check(near(out.ey, -.05) && near(out.forward_correction_m_s, 0), "lateral error reported without fictitious sideways command");
  keeper.reset(); in = {0, 0, pi/2, .005, true, false};
  (void)keeper.update(in);
  in.y = -.05;
  out = keeper.update(in);
  check(near(out.forward_correction_m_s, .04), "forward correction projects along current world heading");
  in.y = 10;
  out = keeper.update(in);
  check(near(out.forward_correction_m_s, -.08), "negative forward saturation");
  in.y = -10;
  check(near(keeper.update(in).forward_correction_m_s, .08), "positive forward saturation");
  in.yaw = pi/2+.5;
  check(near(keeper.update(in).yaw_correction_rad_s, -.25), "negative yaw saturation");
  in.yaw = pi/2-.5;
  check(near(keeper.update(in).yaw_correction_rad_s, .25), "positive yaw saturation");
  keeper.reset(); in = {0, 0, pi-.02, .005, true, false};
  (void)keeper.update(in); in.yaw = -pi+.02;
  out = keeper.update(in);
  check(near(out.yawerror, -.04) && near(out.yaw_correction_rad_s, -.08), "yaw wrap crosses pi by shortest path");
  keeper.reset(); in.yaw = -pi+.02; (void)keeper.update(in); in.yaw = pi-.02;
  check(near(keeper.update(in).yawerror, .04), "opposite yaw wrap sign");
}
void lifecycleTests() {
  StationKeeper keeper;
  StationKeeper::Input in{0, 0, 0, .005, true, false};
  (void)keeper.update(in);
  in.operator_driving = true; in.x = 1; in.yaw = .3;
  check(cleared(keeper.update(in)), "drive suppresses station correction");
  in.x = 2; in.y = .4; in.yaw = .5;
  check(cleared(keeper.update(in)), "held drive continuously recaptures pose");
  in.operator_driving = false; in.x = 2.02; in.yaw = .52;
  auto out = keeper.update(in);
  check(out.holding && near(out.ex, -.02) && near(out.ey, 0) && near(out.yawerror, -.02),
        "release preserves last driving anchor including last-step motion");
  in.dt = .02;
  const auto later = keeper.update(in);
  check(near(later.forward_correction_m_s, out.forward_correction_m_s), "pure P is independent of valid dt");
  for (int i=0; i<100; ++i) out = keeper.update(in);
  check(near(out.ex, -.02), "stationary updates do not recapture drifting pose");
  in.active = false;
  check(cleared(keeper.update(in)), "inactive clears corrections and anchor");
  in.active = true; in.x = 100;
  check(near(keeper.update(in).ex, 0), "reactivation captures current location");
  keeper.reset(); in.x = -50;
  check(near(keeper.update(in).ex, 0), "explicit reset forgets old location");
}
void invalidTests() {
  for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity(),
                         -std::numeric_limits<double>::infinity()}) {
    for (auto member : {&StationKeeper::Input::x, &StationKeeper::Input::y,
                        &StationKeeper::Input::yaw, &StationKeeper::Input::dt}) {
      StationKeeper keeper;
      StationKeeper::Input in{1, 2, .3, .005, true, false};
      (void)keeper.update(in);
      in.*member = invalid;
      check(cleared(keeper.update(in)), "nonfinite field resets with finite zero output");
      in = {5, 6, .4, .005, true, false};
      const auto out = keeper.update(in);
      check(out.holding && near(out.ex, 0) && near(out.ey, 0) && near(out.yawerror, 0), "valid recovery captures fresh anchor");
    }
  }
  StationKeeper keeper;
  for (double dt : {0., -.005}) {
    check(cleared(keeper.update({0, 0, 0, dt, true, false})), "nonpositive time resets");
  }
  const double huge = std::numeric_limits<double>::max();
  (void)keeper.update({huge, 0, 0, .005, true, false});
  check(cleared(keeper.update({-huge, 0, 0, .005, true, false})), "arithmetic overflow also resets");
  for (auto member : {&StationKeeper::Config::position_gain_hz, &StationKeeper::Config::yaw_gain_hz,
                      &StationKeeper::Config::max_forward, &StationKeeper::Config::max_yaw,
                      &StationKeeper::Config::acquire_dwell_s}) {
    StationKeeper::Config cfg;
    cfg.*member = -1;
    bool rejected = false;
    try { StationKeeper invalid(cfg); } catch (const std::invalid_argument&) { rejected = true; }
    check(rejected, "negative configuration rejected");
  }
}
void returnTest() {
  StationKeeper keeper;
  StationKeeper::Input in{0, 0, 0, .01, true, false};
  (void)keeper.update(in);
  in.x = .2;
  // Kinematic scalar response only, not a simulator or balancing-plant test.
  for (int i=0; i<1200; ++i) {
    const auto out = keeper.update(in);
    in.x += out.forward_correction_m_s*in.dt;
  }
  check(std::abs(in.x) < .0001, "bounded proportional loop returns longitudinal displacement to anchor");
  keeper.reset(); in = {0, 0, .2, .01, true, false};
  (void)keeper.update(in); in.yaw = -.2;
  for (int i=0; i<500; ++i) in.yaw += keeper.update(in).yaw_correction_rad_s*in.dt;
  check(std::abs(in.yaw-.2) < .0001, "bounded yaw loop returns to anchor heading");
}
void acquisitionTests() {
  StationKeeper::Config config;config.acquire_dwell_s=.05;
  StationKeeper keeper(config);
  StationKeeper::Input in{0,0,0,.01,true,false,false};
  for(int n=0;n<20;++n) check(cleared(keeper.update(in)),"unstable startup cannot acquire anchor");
  in.ready_to_acquire=true;
  for(int n=0;n<4;++n) check(cleared(keeper.update(in)),"acquisition requires full stable dwell");
  in.ready_to_acquire=false;(void)keeper.update(in);in.ready_to_acquire=true;
  for(int n=0;n<4;++n) check(cleared(keeper.update(in)),"interrupted dwell starts again");
  in.x=1.;auto out=keeper.update(in);
  check(out.holding&&near(out.ex,0),"anchor captured at actual settled position");
  in.x=1.01;in.ready_to_acquire=false;out=keeper.update(in);
  check(out.holding&&near(out.ex,-.01),"later pose motion cannot relocate the captured anchor");
  keeper.reset();check(cleared(keeper.update(in)),"explicit reset restores acquisition gate");
  in.operator_driving=true;check(cleared(keeper.update(in)),"operator drive still captures without correction");
  in.operator_driving=false;in.x+=.01;
  check(near(keeper.update(in).ex,-.01),"drive release retains established anchor");
}
}
int main() {
  poseTests(); lifecycleTests(); invalidTests(); returnTest(); acquisitionTests();
  std::cout << checks-failures << '/' << checks << " StationKeeper checks passed\n";
  return failures ? 1 : 0;
}
