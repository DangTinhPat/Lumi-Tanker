#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <new>
#include <type_traits>

#include "robot_control/JumpPlan.hpp"

using robot_control::JumpPlan;
namespace {
int checks = 0, failures = 0;
bool counting = false;
std::size_t allocations = 0;
void check(bool ok, const char* name) {
  ++checks;
  if (!ok) { ++failures; std::fprintf(stderr, "FAIL %s\n", name); }
}
bool near(float a, float b, float tolerance = 2e-5F) { return std::abs(a - b) <= tolerance; }
JumpPlan plan() {
  JumpPlan p;
  p.count = 3;
  p.thrust_duration_s = .2F;
  p.flight_duration_s = .3F;
  for (std::size_t i = 0; i < p.count; ++i) {
    auto& s = p.samples[i];
    s.time_s = .5F * static_cast<float>(i);
    const float t = s.time_s;
    s.pitch_rad = .05F * t*t*t;
    s.pitch_rate_rad_s = .15F * t*t;
    s.hip_rad = .1F + .2F*t*t*t;
    s.hip_velocity_rad_s = .6F*t*t;
    s.knee_rad = -.4F - .1F*t*t*t;
    s.knee_velocity_rad_s = -.3F*t*t;
    s.hip_torque_nm = -2 + 4*t;
    s.knee_torque_nm = 3 - 2*t;
    s.wheel_velocity_rad_s = -20 + 40*t;
    s.wheel_torque_nm = -.1F + .2F*t;
    s.wheel_position_rad = -20*t + 20*t*t;
    s.base_x_m = -.1F + .2F*t*t*t;
    s.base_vx_m_s = .6F*t*t;
    s.base_z_m = .2F + .1F*t*t*t;
    s.base_vz_m_s = .3F*t*t;
    for (std::size_t k = 0; k < s.feedback_gain.size(); ++k)
      s.feedback_gain[k] = static_cast<float>(k) * (2*t-1);
  }
  return p;
}
JumpPlan flat() {
  auto p = plan();
  for (std::size_t i = 0; i < p.count; ++i) {
    p.samples[i] = {};
    p.samples[i].time_s = .5F * static_cast<float>(i);
  }
  return p;
}
void* allocate(std::size_t size) {
  if (counting) ++allocations;
  if (void* p = std::malloc(size ? size : 1)) return p;
  throw std::bad_alloc();
}
}  // namespace

void* operator new(std::size_t n) { return allocate(n); }
void* operator new[](std::size_t n) { return allocate(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

int main() {
  static_assert(std::is_trivially_copyable<JumpPlan>::value, "Fixed POD-like plan storage");
  static_assert(noexcept(JumpPlan{}.sample(0)), "Sampling must not throw");
  constexpr JumpPlan::Sample legacy{0, .1F, .2F, .3F, -.4F, .5F, .6F, 1, 2, 3, .1F};
  static_assert(legacy.wheel_torque_nm == .1F && legacy.base_x_m == 0 &&
                legacy.base_z_m == 0 && legacy.base_vx_m_s == 0 && legacy.base_vz_m_s == 0 &&
                legacy.wheel_position_rad == 0 && legacy.feedback_gain[35] == 0,
                "Existing eleven-field aggregates retain their layout and zero defaults");
  const auto p = plan();
  check(p.valid(), "reference cubic valid");
  check(!JumpPlan{}.valid(), "empty invalid");
  check(JumpPlan{}.sample(1).time_s == 0, "empty sample defensive");
  for (int i = 0; i <= 100; ++i) {
    const float t = static_cast<float>(i) / 100;
    const auto s = p.sample(t);
    check(near(s.pitch_rad, .05F*t*t*t) && near(s.pitch_rate_rad_s, .15F*t*t), "pitch cubic and derivative");
    check(near(s.hip_rad, .1F+.2F*t*t*t) && near(s.hip_velocity_rad_s, .6F*t*t), "hip cubic and derivative");
    check(near(s.knee_rad, -.4F-.1F*t*t*t) && near(s.knee_velocity_rad_s, -.3F*t*t), "knee cubic and derivative");
    check(near(s.hip_torque_nm, -2+4*t) && near(s.knee_torque_nm, 3-2*t) &&
          near(s.wheel_velocity_rad_s, -20+40*t) && near(s.wheel_torque_nm, -.1F+.2F*t), "linear efforts and wheel speed");
    check(near(s.base_x_m, -.1F+.2F*t*t*t) && near(s.base_vx_m_s, .6F*t*t) &&
          near(s.base_z_m, .2F+.1F*t*t*t) && near(s.base_vz_m_s, .3F*t*t), "base Hermite position and rate");
    check(near(s.wheel_position_rad, -20*t+20*t*t), "wheel Hermite position");
    for (std::size_t k = 0; k < s.feedback_gain.size(); ++k)
      check(near(s.feedback_gain[k], static_cast<float>(k)*(2*t-1)), "gain linear interpolation");
  }
  const auto before = p.sample(.5F - 1e-5F), after = p.sample(.5F + 1e-5F);
  check(near(before.hip_rad, after.hip_rad) && near(before.hip_velocity_rad_s, after.hip_velocity_rad_s), "C1 at internal knot");
  for (float t : {.13F, .37F, .69F, .83F}) {
    constexpr float h = .0005F;
    const auto a = p.sample(t-h), b = p.sample(t+h), s = p.sample(t);
    check(near((b.hip_rad-a.hip_rad)/(2*h), s.hip_velocity_rad_s, 5e-5F) &&
          near((b.knee_rad-a.knee_rad)/(2*h), s.knee_velocity_rad_s, 5e-5F) &&
          near((b.pitch_rad-a.pitch_rad)/(2*h), s.pitch_rate_rad_s, 5e-5F), "finite difference derivatives");
  }
  const float inf = std::numeric_limits<float>::infinity();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  check(p.touchdown_com_z_m == 0 && p.valid(), "default touchdown height remains valid");
  for (float height : {0.F, .2F, .6F}) {
    auto candidate = p; candidate.touchdown_com_z_m = height;
    check(candidate.valid(), "touchdown height inclusive bounds");
  }
  for (float height : {nan, inf, -inf, -.001F, std::nextafter(.6F, inf)}) {
    auto candidate = p; candidate.touchdown_com_z_m = height;
    check(!candidate.valid(), "reject invalid touchdown height");
  }
  check(p.sample(-1).time_s == 0 && p.sample(-inf).time_s == 0, "clamp before start");
  check(p.sample(2).time_s == 1 && p.sample(inf).time_s == 1 &&
        p.sample(2).hip_velocity_rad_s == p.samples[2].hip_velocity_rad_s, "clamp after end retains rate");
  check(p.sample(nan).time_s == 0, "NaN query defensive");
  check(p.sample(.5F).hip_rad == p.samples[1].hip_rad &&
        p.sample(.5F).hip_velocity_rad_s == p.samples[1].hip_velocity_rad_s, "exact internal endpoint");

  const std::array<float JumpPlan::Sample::*, 16> fields{{
    &JumpPlan::Sample::time_s, &JumpPlan::Sample::pitch_rad, &JumpPlan::Sample::pitch_rate_rad_s,
    &JumpPlan::Sample::hip_rad, &JumpPlan::Sample::knee_rad, &JumpPlan::Sample::hip_velocity_rad_s,
    &JumpPlan::Sample::knee_velocity_rad_s, &JumpPlan::Sample::hip_torque_nm,
    &JumpPlan::Sample::knee_torque_nm, &JumpPlan::Sample::wheel_velocity_rad_s, &JumpPlan::Sample::wheel_torque_nm,
    &JumpPlan::Sample::base_x_m, &JumpPlan::Sample::base_z_m, &JumpPlan::Sample::base_vx_m_s,
    &JumpPlan::Sample::base_vz_m_s, &JumpPlan::Sample::wheel_position_rad}};
  for (auto member : fields) for (float invalid : {nan, inf, -inf}) {
    auto bad = p; bad.samples[1].*member = invalid;
    check(!bad.valid(), "reject nonfinite sample field");
  }
  for (float invalid : {nan, inf, -inf, 0.F, -.1F, 1.F}) {
    auto bad = p; bad.thrust_duration_s = invalid;
    check(!bad.valid(), "reject thrust duration");
    bad = p; bad.flight_duration_s = invalid;
    check(!bad.valid(), "reject flight duration");
  }
  for (std::size_t k = 0; k < 36; ++k) for (float invalid : {nan, inf, -inf, 100001.F, -100001.F}) {
    auto bad = p; bad.samples[1].feedback_gain[k] = invalid;
    check(!bad.valid(), "gain finite and magnitude bounds");
  }
  {
    auto legacy_plan = p;
    for (auto& s : legacy_plan.samples) {
      s.base_x_m = s.base_z_m = s.base_vx_m_s = s.base_vz_m_s = s.wheel_position_rad = 0;
      s.feedback_gain = {};
    }
    check(legacy_plan.valid(), "default appended zero fields remain valid");
    for (float bound : {-100000.F, 100000.F}) {
      auto good = p; good.samples[1].feedback_gain.fill(bound);
      check(good.valid(), "inclusive gain bounds");
    }
    const auto start = p.sample(-1), end = p.sample(2), knot = p.sample(.5F);
    check(start.base_x_m == p.samples[0].base_x_m && start.feedback_gain == p.samples[0].feedback_gain &&
          end.wheel_position_rad == p.samples[2].wheel_position_rad && end.base_vz_m_s == p.samples[2].base_vz_m_s &&
          end.feedback_gain == p.samples[2].feedback_gain && knot.feedback_gain == p.samples[1].feedback_gain,
          "new fields preserve clamped and knot values");
    check(near(before.base_x_m, after.base_x_m) && near(before.base_vx_m_s, after.base_vx_m_s) &&
          near(before.base_z_m, after.base_z_m) && near(before.base_vz_m_s, after.base_vz_m_s) &&
          near(before.wheel_velocity_rad_s, after.wheel_velocity_rad_s, .001F), "base and wheel C1 continuity");
    for (float t : {.13F, .37F, .69F, .83F}) {
      constexpr float h = .0005F;
      const auto a = p.sample(t-h), b = p.sample(t+h), s = p.sample(t);
      check(near((b.base_x_m-a.base_x_m)/(2*h), s.base_vx_m_s, 5e-5F) &&
            near((b.base_z_m-a.base_z_m)/(2*h), s.base_vz_m_s, 5e-5F) &&
            near((b.wheel_position_rad-a.wheel_position_rad)/(2*h), s.wheel_velocity_rad_s, .002F),
            "new Hermite finite difference rates");
    }
    auto curved = flat(); curved.samples[1].wheel_position_rad = 1;
    check(curved.valid() && near(curved.sample(.25F).wheel_velocity_rad_s, 3),
          "wheel rate is Hermite derivative rather than endpoint rate interpolation");
  }
  {
    for (float value : {-2.001F, 2.001F}) {
      auto bad = flat(); bad.samples[1].base_x_m = value; check(!bad.valid(), "base x endpoint bound");
    }
    for (float value : {-.001F, .601F}) {
      auto bad = flat(); bad.samples[1].base_z_m = value; check(!bad.valid(), "base z endpoint bound");
    }
    for (float value : {-100.001F, 100.001F}) {
      auto bad = flat(); bad.samples[1].wheel_position_rad = value; check(!bad.valid(), "wheel position endpoint bound");
    }
    auto bad = flat(); bad.samples[0].base_x_m = bad.samples[1].base_x_m = 1.99F;
    bad.samples[0].base_vx_m_s = 1; bad.samples[1].base_vx_m_s = -1;
    check(!bad.valid(), "base x interior overshoot");
    bad = flat(); bad.samples[0].base_z_m = bad.samples[1].base_z_m = .59F;
    bad.samples[0].base_vz_m_s = 1; bad.samples[1].base_vz_m_s = -1;
    check(!bad.valid(), "base z interior upper overshoot");
    bad = flat(); bad.samples[0].base_vz_m_s = -1; bad.samples[1].base_vz_m_s = 1;
    check(!bad.valid(), "base z interior penetration");
    bad = flat(); bad.samples[0].wheel_position_rad = bad.samples[1].wheel_position_rad = 99.99F;
    bad.samples[0].wheel_velocity_rad_s = 1; bad.samples[1].wheel_velocity_rad_s = -1;
    check(!bad.valid(), "wheel position interior overshoot");
    bad = flat(); bad.samples[1].time_s = .01F; bad.samples[1].wheel_position_rad = 1;
    check(!bad.valid(), "wheel interior rate bound");
    auto good = flat();
    for (auto& s : good.samples) { s.base_x_m = -2; s.base_z_m = .6F; s.wheel_position_rad = 100; }
    check(good.valid(), "inclusive new position bounds");
  }
  {
    auto bad = p; bad.count = 1; check(!bad.valid(), "one sample invalid");
    bad.count = 257; check(!bad.valid() && bad.sample(.4F).time_s == 0, "capacity overflow rejected");
    bad = p; bad.samples[0].time_s = .001F; check(!bad.valid(), "start must be zero");
    bad = p; bad.samples[1].time_s = 0; check(!bad.valid(), "duplicate timestamp");
    bad = p; bad.samples[1].time_s = -.1F; check(!bad.valid(), "reversed timestamp");
    bad = p; bad.samples[2].time_s = 2.001F; check(!bad.valid(), "duration over two seconds");
    bad = p; bad.thrust_duration_s = .5F; bad.flight_duration_s = .5F;
    check(!bad.valid(), "phases leave positive recovery time");
  }
  for (auto field : {&JumpPlan::Sample::hip_rad, &JumpPlan::Sample::knee_rad,
                     &JumpPlan::Sample::pitch_rad}) {
    auto bad = flat();
    bad.samples[1].*field = 2;
    check(!bad.valid(), "position endpoint bound");
    bad.samples[1].*field = -2;
    check(!bad.valid(), "negative position endpoint bound");
  }
  for (auto field : {&JumpPlan::Sample::hip_velocity_rad_s, &JumpPlan::Sample::knee_velocity_rad_s}) {
    for (float sign : {-1.F, 1.F}) {
      auto bad = flat(); bad.samples[1].*field = sign*28.275F;
      check(!bad.valid(), "joint rate endpoint bound");
    }
  }
  for (auto field : {&JumpPlan::Sample::hip_torque_nm, &JumpPlan::Sample::knee_torque_nm}) {
    auto bad = p; bad.samples[1].*field = 5.001F; check(!bad.valid(), "leg torque bound");
  }
  {
    auto bad = p; bad.samples[1].wheel_torque_nm = .1301F; check(!bad.valid(), "wheel torque bound");
    bad = p; bad.samples[1].wheel_velocity_rad_s = 60.001F; check(!bad.valid(), "wheel speed bound");
  }
  {
    auto bad = flat();
    bad.samples[0].knee_rad = -.06177078187465668F;
    bad.samples[0].knee_velocity_rad_s = -.1491149514913559F;
    bad.samples[1].time_s = .4149957001209259F;
    bad.samples[1].knee_rad = bad.samples[2].knee_rad = -1.6F;
    // The near-endpoint stationary root evaluates one double ULP below -1.6F.
    check(bad.valid(), "interior position evaluation tolerates double roundoff at bound");
    bad.samples[1].knee_rad = std::nextafter(-1.6F, -inf);
    check(!bad.valid(), "roundoff tolerance does not relax node bounds");
    bad = flat();
    bad.samples[0].knee_rad = bad.samples[1].knee_rad = -1.6F;
    bad.samples[0].knee_velocity_rad_s = -1e-10F;
    bad.samples[1].knee_velocity_rad_s = 1e-10F;
    check(!bad.valid(), "true interior overshoot beyond evaluation roundoff is rejected");
    bad = flat();
    bad.samples[0].hip_velocity_rad_s = std::nextafter(28.274334F, inf);
    check(!bad.valid(), "roundoff tolerance does not relax rate bounds");
  }
  {
    auto bad = flat();
    bad.samples[0].hip_rad = bad.samples[1].hip_rad = 1.F;
    bad.samples[0].hip_velocity_rad_s = 2;
    bad.samples[1].hip_velocity_rad_s = -2;
    check(!bad.valid(), "quadratic-position overshoot despite valid knots");
    bad = flat(); bad.samples[0].knee_velocity_rad_s = -20; bad.samples[1].knee_velocity_rad_s = 20;
    check(!bad.valid(), "knee interior negative overshoot");
    bad = flat(); bad.samples[0].pitch_rate_rad_s = 4; bad.samples[1].pitch_rate_rad_s = -4;
    check(!bad.valid(), "pitch interior overshoot");
    bad = flat(); bad.samples[1].time_s = .01F; bad.samples[1].hip_rad = .4F;
    check(!bad.valid(), "interior rate exceeds limit with zero endpoint rates");
    bad = flat(); bad.samples[1].time_s = .01F; bad.samples[1].knee_rad = -.4F;
    check(!bad.valid(), "interior negative knee rate exceeds limit");
    bad = flat(); bad.samples[0].hip_rad = bad.samples[1].hip_rad = 1.049F;
    bad.samples[0].hip_velocity_rad_s = 1; bad.samples[1].hip_velocity_rad_s = -3;
    check(!bad.valid(), "cubic-position root overshoot");
    bad = flat(); bad.samples[0].pitch_rate_rad_s = std::numeric_limits<float>::max();
    check(!bad.valid(), "extreme finite rate rejected without numeric overflow");
  }
  {
    auto bounds = flat(); bounds.samples[2].time_s = 2;
    for (auto& s : bounds.samples) {
      s.hip_rad = 1.05F; s.knee_rad = -1.6F; s.pitch_rad = .3F;
      s.hip_torque_nm = 5; s.knee_torque_nm = -5;
      s.wheel_velocity_rad_s = 60; s.wheel_torque_nm = -.13F;
    }
    check(bounds.valid(), "inclusive position effort time bounds");
    bounds.samples[3].time_s = nan;
    check(bounds.valid(), "unused capacity ignored");
    auto full = flat(); full.count = JumpPlan::kCapacity;
    for (std::size_t i = 0; i < full.count; ++i) {
      full.samples[i] = {}; full.samples[i].time_s = 2.F*static_cast<float>(i)/255;
    }
    check(full.valid() && full.sample(1).time_s == 1, "full capacity interpolation");
    counting = true;
    volatile bool accepted = full.valid();
    volatile float value = 0;
    for (int i = 0; i < 1000; ++i) value = full.sample(.001F*static_cast<float>(i)).hip_rad;
    counting = false;
    (void)accepted; (void)value;
    check(allocations == 0, "validation and sampling allocate no heap");
  }
  std::printf("jump plan: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
