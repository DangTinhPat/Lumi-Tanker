// Dabble GamePad, declared only: FirmwareApp.cpp (same build) defines the
// GamePad object with CUSTOM_SETTINGS; without it the header emits extern.
#define INCLUDE_GAMEPAD_MODULE
#include "tap_hw/AppV3.hpp"

#include <atomic>

#include <BLEDevice.h>
#include <DabbleESP32.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <esp_timer.h>

#include "robot_hal/MotorFrameCodec.hpp"
#include "tap_fw/ImuMount.hpp"

namespace tap_hw {
namespace {

namespace frame = robot_control::motor_frame;
using tap_fw::v3::LegLock;
using tap_fw::v3::V3Controller;

constexpr float kGravity = 9.80665F;
constexpr float kTwoPi = 6.28318530717958647692F;
constexpr std::uint64_t kReportUs = 200'000U;
constexpr std::uint64_t kTraceUs = 40'000U;
constexpr std::uint64_t kLeaseUs = 500'000U;        // serial silence while active
constexpr std::uint64_t kLegFreshUs = 100'000U;     // 0x009 at ~100 Hz
constexpr std::uint64_t kClosedLoopGraceUs = 500'000U;
constexpr std::uint64_t kLockTimeoutUs = 5'000'000U;
constexpr std::uint64_t kWheelSettleUs = 20'000U;   // Torque Enable -> Moving Speed
// A leg further than this from its power-up pose at START is refused: the
// ramp would otherwise swing it back a long way.
constexpr float kMaxStartOffsetRad = 0.35F;
constexpr std::uint64_t kHeightHoldUs = 300'000U;  // HMOVE refresh window
// BLE gamepad.
constexpr std::uint64_t kBlePollUs = 10'000U;
constexpr std::uint64_t kBleAdvertiseDelayUs = 300'000U;
// Full stick. Forward stays 0.25 m/s under the 0.8 m/s balancer cap: catching
// a forward lean needs the wheels to run faster than the drive speed.
constexpr float kBleDriveSpeedMS = 0.55F;   // full stick forward
constexpr float kBleYawRateRadS = 1.5F;     // full stick sideways

std::atomic<int> g_v3_ble_links{0};
std::atomic<bool> g_v3_ble_readvertise{false};

void onV3GattsEvent(esp_gatts_cb_event_t event, esp_gatt_if_t, esp_ble_gatts_cb_param_t*) {
  if (event == ESP_GATTS_CONNECT_EVT) {
    g_v3_ble_links.fetch_add(1);
  } else if (event == ESP_GATTS_DISCONNECT_EVT) {
    int links = g_v3_ble_links.load();
    while (links > 0 && !g_v3_ble_links.compare_exchange_weak(links, links - 1)) {}
    g_v3_ble_readvertise.store(true);
  }
}
constexpr float kPitchAvgTauS = 0.5F;       // balance-angle measurement average
constexpr float kTiltTestGain = 80.0F;      // rad/s wheel per rad pitch (0.1 rad -> 8 rad/s)
constexpr float kTiltTestMaxRadS = 12.0F;   // below the 15 rad/s runaway guard
// Legs count as "at zero" once within this of the target after the ramp.
constexpr float kLegReadyRad = 0.08F;
constexpr std::array<const char*, 4> kLegNames{"LH", "LK", "RH", "RK"};
// HIPTEST: 0-2 s steps (+A, 0, -A, 0), 2-6 s sines at 1/2/4/8 Hz (1 s each),
// 6-6.5 s back at 0. One sample per control tick.
constexpr float kHipTestDurationS = 6.5F;
constexpr std::size_t kHipTestMaxSamples = 3400U;
struct HipTestSample {
  std::uint32_t t_us;
  float cmd_rad;
  float hip_l_rad;
  float hip_r_rad;
  std::uint32_t fb_age_us;
};
HipTestSample g_hip_test[kHipTestMaxSamples];

std::uint64_t nowUs() { return static_cast<std::uint64_t>(esp_timer_get_time()); }

}  // namespace

AppV3::AppV3() noexcept
    : transport_(can_.port(), uart1_.port(), uart2_.port()),
      ctrl_(tap_fw::v3::defaultV3Config()) {}

bool AppV3::begin() noexcept {
  Serial.setTxBufferSize(4096U);
  Serial.begin(115200);
  delay(300);
  pinMode(tap_fw::board::kLed1Gpio, OUTPUT);
  pinMode(tap_fw::board::kLed2Gpio, OUTPUT);
  pinMode(tap_fw::board::kLed3Gpio, OUTPUT);
  Serial.println("TAP firmware v3: two-wheel inverted pendulum, legs locked at the "
                 "power-up pose (driver position mode), USB serial control");
  can_ready_ = can_.begin();
  uart_ready_ = uart1_.begin(tap_fw::board::kMotorUartBaud) &&
      uart2_.begin(tap_fw::board::kMotorUartBaud);
  imu_ready_ = mpu_.begin();
  Serial.printf("TWAI=%s UART=%s MPU6500 WHO_AM_I=0x%02X (%s)\n",
                can_ready_ ? "PASS" : "FAIL", uart_ready_ ? "READY" : "FAIL",
                mpu_.whoAmI(), imu_ready_ ? "PASS" : "FAIL");
  Serial.println("V3 procedure: arrange the legs BEFORE switching the motor supply on "
                 "(that pose is zero). Stand the robot on its wheels, press START, "
                 "let go (loose tether). DISABLE / ESTOP stop everything.");
  Dabble.begin("TAP-RB");
  BLEDevice::setCustomGattsHandler(onV3GattsEvent);
  Serial.println("BLE: Dabble GamePad as TAP-RB (joystick drive, Start=START, Select=GO, "
                 "Triangle/Cross hold = raise/lower); USB serial GUI also works");
  printTunables();
  const std::uint64_t now_us = nowUs();
  next_control_us_ = now_us;
  last_rx_us_ = now_us;
  return can_ready_ && uart_ready_ && imu_ready_;
}

void AppV3::loop() noexcept {
  const std::uint64_t now_us = nowUs();
  pollSerial(now_us);
  const std::uint64_t t_serial = nowUs();
  pollBle(now_us);
  const std::uint64_t t_ble = nowUs();
  can_.service(now_us);
  drainFeedback(now_us);
  const std::uint64_t t_bus = nowUs();
  pre_serial_max_us_ = std::max(pre_serial_max_us_, static_cast<std::uint32_t>(t_serial - now_us));
  pre_ble_max_us_ = std::max(pre_ble_max_us_, static_cast<std::uint32_t>(t_ble - t_serial));
  pre_bus_max_us_ = std::max(pre_bus_max_us_, static_cast<std::uint32_t>(t_bus - t_ble));
  if (now_us >= next_control_us_) {
    // Fixed-rate schedule. `now + period` added each tick's own processing
    // time to the period: measured 2026-09-30 (HIPTEST) the loop ran at
    // ~440 Hz (2.27 ms mean, 3.5 ms max) while the controller integrated
    // with dt = 2 ms, i.e. ~12 % slow. A long stall resyncs instead of
    // bursting through the missed ticks.
    constexpr std::uint64_t kPeriod = tap_fw::board::kControlPeriodUs;
    if (now_us - next_control_us_ > kPeriod) ++tick_late_;
    next_control_us_ += kPeriod;
    if (now_us >= next_control_us_ + 4U * kPeriod) next_control_us_ = now_us + kPeriod;
    tick_dt_s_ = last_tick_us_ == 0U ? static_cast<float>(kPeriod) * 1.0e-6F
        : std::clamp(static_cast<float>(now_us - last_tick_us_) * 1.0e-6F,
                     0.5e-6F * static_cast<float>(kPeriod),
                     3.0e-6F * static_cast<float>(kPeriod));
    last_tick_us_ = now_us;
    updateImu(now_us);
    const std::uint64_t t_imu = nowUs();
    controlTick(now_us);
    const std::uint64_t t_ctrl = nowUs();
    report(now_us);
    const std::uint64_t t_end = nowUs();
    tick_imu_max_us_ = std::max(tick_imu_max_us_, static_cast<std::uint32_t>(t_imu - t_bus));
    tick_ctrl_max_us_ = std::max(tick_ctrl_max_us_, static_cast<std::uint32_t>(t_ctrl - t_imu));
    tick_exec_max_us_ = std::max(tick_exec_max_us_, static_cast<std::uint32_t>(t_end - now_us));
  }
  delayMicroseconds(100U);
}

void AppV3::pollSerial(std::uint64_t now_us) noexcept {
  for (int budget = 32; budget > 0 && Serial.available() > 0; --budget) {
    const int c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (line_length_ + 1U < sizeof(line_)) line_[line_length_++] = static_cast<char>(c);
      continue;
    }
    line_[line_length_] = '\0';
    line_length_ = 0U;
    if (line_[0] != '\0') handleLine(line_, now_us);
  }
}

void AppV3::handleLine(const char* line, std::uint64_t now_us) noexcept {
  last_rx_us_ = now_us;  // any line keeps the session lease alive
  if (std::strcmp(line, "HEARTBEAT") == 0) return;
  auto& lqr = ctrl_.balancer();
  if (std::strcmp(line, "START") == 0 || std::strcmp(line, "TILTTEST") == 0) {
    if (estop_) {
      Serial.println("V3 START refused: E-STOP latched (CLEAR_ESTOP)");
      return;
    }
    start_pending_ = true;
    tilt_test_ = line[0] == 'T';
    if (!ctrl_.legsHeld()) height_request_m_ = 0.0F;  // fresh lock: hang height
    Serial.println(tilt_test_ ? "V3 START (TILTTEST: wheel = k*(pitch - pitch at release), "
                                "max 12 rad/s, no balance loop)"
                              : "V3 START");
  } else if (std::strcmp(line, "GO") == 0) {
    // Operator flow: GO is pressed with the robot on the stand. It only arms:
    // legs lock, wheels hold, and the balance loop engages by itself once the
    // robot has been carried to the floor, held still and let go. Balancing
    // on the stand would spin the wheels in the air into a runaway.
    const auto st = ctrl_.state();
    if (st == V3Controller::State::Ready) {
      Serial.println("V3 GO: armed. Carry the robot to the floor, hold it still ~1 s at its "
                     "standing point, let go -> balancing starts by itself");
    } else if (st == V3Controller::State::Balancing && tilt_test_) {
      go_pending_ = true;  // leave TILTTEST for the armed Ready state
    } else if (st == V3Controller::State::Balancing || st == V3Controller::State::Locking) {
      Serial.println("V3 GO: already running");
    } else if (estop_) {
      Serial.println("V3 GO refused: E-STOP latched (CLEAR_ESTOP)");
    } else {
      start_pending_ = true;
      tilt_test_ = false;
      if (!ctrl_.legsHeld()) height_request_m_ = 0.0F;
      Serial.println("V3 GO: locking legs, then wheels hold (armed; balances after the robot "
                     "is put on the floor and let go)");
    }
  } else if (std::strncmp(line, "HMOVE ", 6U) == 0) {
    int dir = 0;
    if (std::sscanf(line + 6, "%d", &dir) == 1) setHeightMove(dir, now_us);
  } else if (std::strcmp(line, "GO NOW") == 0) {
    if (ctrl_.state() == V3Controller::State::Ready) {
      go_pending_ = true;
      Serial.println("V3 GO NOW: balance loop engaged immediately");
    }
  } else if (std::strcmp(line, "DISABLE") == 0) {
    disable_pending_ = true;
    tilt_test_ = false;
    go_after_ready_ = false;
    height_request_m_ = 0.0F;
    hmove_dir_ = 0;
    Serial.println("V3 DISABLE");
  } else if (std::strcmp(line, "ESTOP") == 0) {
    estop_ = true;
    disable_pending_ = true;
    Serial.println("V3 E-STOP latched");
  } else if (std::strcmp(line, "CLEAR_ESTOP") == 0) {
    estop_ = false;
    Serial.println("V3 E-STOP cleared");
  } else if (std::strcmp(line, "STOP") == 0) {
    drive_forward_ = drive_yaw_ = 0.0F;
  } else if (std::strncmp(line, "DRIVE ", 6U) == 0) {
    float f = 0.0F, y = 0.0F;
    if (std::sscanf(line + 6, "%f %f", &f, &y) == 2) {
      drive_forward_ = std::clamp(f, -kBleDriveSpeedMS, kBleDriveSpeedMS);
      drive_yaw_ = std::clamp(y, -kBleYawRateRadS, kBleYawRateRadS);
    }
  } else if (std::strcmp(line, "LEGMODE POS") == 0 || std::strcmp(line, "LEGMODE MIT") == 0) {
    if (ctrl_.state() != V3Controller::State::Idle &&
        ctrl_.state() != V3Controller::State::Fault) {
      Serial.println("V3 LEGMODE refused: DISABLE first");
      return;
    }
    legs_.setMode(line[8] == 'P' ? LegLock::Mode::Position : LegLock::Mode::Mit);
    printTunables();
  } else if (std::strncmp(line, "SET ", 4U) == 0) {
    char key[20]{};
    float v = 0.0F;
    if (std::sscanf(line + 4, "%19s %f", key, &v) != 2 || !std::isfinite(v)) {
      Serial.println("V3 SET usage: SET <key> <value>");
      return;
    }
    if (!std::strcmp(key, "trim_gain")) lqr.setBalanceTrimGain(std::clamp(v, 0.0F, 2.0F));
    else if (!std::strcmp(key, "trim_max")) lqr.setBalanceTrimMax(std::clamp(v, 0.0F, 0.35F));
    else if (!std::strcmp(key, "trim")) lqr.setBalanceTrim(v);
    else if (!std::strcmp(key, "margin")) lqr.setCommandTrackingMargin(std::clamp(v, 0.0F, 1.0F));
    // 0.2-10: the default is 8 m/s^2 (defaultV3Config); the old 2.0 cap
    // silently cut the LQR's push-recovery authority 4x.
    else if (!std::strcmp(key, "accel")) lqr.setMaxGroundAccel(std::clamp(v, 0.2F, 10.0F));
    else if (!std::strcmp(key, "pitch_scale")) lqr.setWheelPitchFeedbackScale(std::clamp(v, 0.3F, 1.0F));
    else if (!std::strcmp(key, "balance")) ctrl_.setBalanceAngle(true, std::clamp(v, -0.5F, 0.5F));
    else if (!std::strcmp(key, "kp")) ctrl_.config().pid.kp = std::clamp(v, 0.0F, 200.0F);
    else if (!std::strcmp(key, "kd")) ctrl_.config().pid.kd = std::clamp(v, 0.0F, 30.0F);
    else if (!std::strcmp(key, "kv")) ctrl_.config().pid.kv = std::clamp(v, 0.0F, 1.0F);
    else if (!std::strcmp(key, "height")) {
      // Hip-to-axle height in mm (v3.1); clamped to the planner's range.
      height_request_m_ = std::clamp(v * 0.001F, tap_fw::v3::LegHeight::minHeight(),
                                     tap_fw::v3::LegHeight::maxHeight());
      if (ctrl_.state() != V3Controller::State::Balancing) {
        Serial.println("V3 SET height: stored; the legs move once balancing is settled");
      }
    }
    else if (!std::strcmp(key, "hff")) ctrl_.config().height.balance_ff_scale = std::clamp(v, -2.0F, 2.0F);
    else if (!std::strcmp(key, "hrate")) ctrl_.config().height.rate_m_s = std::clamp(v * 0.001F, 0.002F, 0.06F);
    else if (!std::strcmp(key, "dbc")) ctrl_.config().pid.deadband_comp_rad_s = std::clamp(v, 0.0F, 2.0F);
    else if (!std::strcmp(key, "dbe")) ctrl_.config().pid.deadband_eps_rad_s = std::clamp(v, 0.05F, 2.0F);
    else if (!std::strcmp(key, "lead")) ctrl_.config().pid.lead_s = std::clamp(v, 0.0F, 0.3F);
    else if (!std::strcmp(key, "kx")) ctrl_.config().pid.kx = std::clamp(v, 0.0F, 1.0F);
    else if (!std::strcmp(key, "kix")) ctrl_.config().pid.kix = std::clamp(v, 0.0F, 0.2F);
    else if (!std::strcmp(key, "auto")) ctrl_.config().engage.automatic = v >= 0.5F;
    else if (!std::strcmp(key, "amax")) ctrl_.config().pid.accel_max_m_s2 = std::clamp(v, 0.5F, 30.0F);
    else if (!std::strcmp(key, "algo")) {
      if (ctrl_.state() == V3Controller::State::Balancing ||
          ctrl_.state() == V3Controller::State::Ready ||
          ctrl_.state() == V3Controller::State::Locking) {
        Serial.println("V3 SET algo: only while Idle/Fault");
        return;
      }
      ctrl_.config().algo = v >= 0.5F ? V3Controller::Algo::Pid : V3Controller::Algo::Lqr;
    }
    // v3.3 tiers (V3Controller::Tier). vpitch changes the balancer's state,
    // so it only switches outside Balancing; trunk ramps in and out anytime.
    else if (!std::strcmp(key, "vpitch")) {
      if (ctrl_.state() == V3Controller::State::Balancing) {
        Serial.println("V3 SET vpitch: not while balancing (DISABLE or wait for a fault)");
        return;
      }
      ctrl_.config().tier.virtual_pitch = v >= 0.5F;
      if (v < 0.5F) ctrl_.config().tier.trunk = false;
    }
    else if (!std::strcmp(key, "trunk")) {
      if (v >= 0.5F && !ctrl_.config().tier.virtual_pitch) {
        Serial.println("V3 SET trunk: needs SET vpitch 1 first");
        return;
      }
      ctrl_.config().tier.trunk = v >= 0.5F;
    }
    else if (!std::strcmp(key, "tshare")) ctrl_.config().tier.lean_share = std::clamp(v, 0.0F, 1.0F);
    else if (!std::strcmp(key, "tlead")) ctrl_.config().tier.lead_s = std::clamp(v, 0.0F, 0.15F);
    else if (!std::strcmp(key, "tmax")) ctrl_.config().tier.max_rad = std::clamp(v, 0.0F, 0.30F);
    else if (!std::strcmp(key, "trate")) ctrl_.config().tier.rate_rad_s = std::clamp(v, 0.1F, 6.0F);
    else if (!std::strcmp(key, "thpf")) ctrl_.config().tier.hpf_s = std::clamp(v, 0.0F, 30.0F);
    else if (!std::strcmp(key, "texp")) ctrl_.config().tier.height_exp = std::clamp(v, 0.0F, 4.0F);
    else if (!std::strcmp(key, "texpt")) ctrl_.config().tier.height_exp_tall = std::clamp(v, 0.0F, 4.0F);
    else if (!std::strcmp(key, "cdx")) ctrl_.config().tier.body_com_dx_m = std::clamp(v * 0.001F, -0.06F, 0.06F);
    else if (!std::strcmp(key, "timu")) ctrl_.config().tier.imu_kinematic = v >= 0.5F;
    else if (!std::strcmp(key, "tvg")) ctrl_.config().tier.joint_rate_gain = std::clamp(v, 0.0F, 1.0F);
    // Maneuver preview (needs vpitch): planned, previewed speed changes.
    else if (!std::strcmp(key, "tprev")) ctrl_.config().tier.preview = v >= 0.5F;
    else if (!std::strcmp(key, "tpacc")) ctrl_.config().tier.plan_accel_m_s2 = std::clamp(v, 0.05F, 1.0F);
    else if (!std::strcmp(key, "tpjerk")) ctrl_.config().tier.plan_jerk_m_s3 = std::clamp(v, 0.2F, 5.0F);
    else if (!std::strcmp(key, "tpvmax")) ctrl_.config().tier.plan_vmax_m_s = std::clamp(v, 0.05F, 0.8F);
    else if (!std::strcmp(key, "tpt")) ctrl_.config().tier.preview_s = std::clamp(v, 0.0F, 0.25F);
    // Static level loop (needs trunk): body pitch target and its limits.
    else if (!std::strcmp(key, "tlvl")) ctrl_.config().tier.level = v >= 0.5F;
    else if (!std::strcmp(key, "tlevel")) ctrl_.config().tier.level_pitch_rad = std::clamp(v, -0.30F, 0.30F);
    else if (!std::strcmp(key, "tltau")) ctrl_.config().tier.level_tau_s = std::clamp(v, 0.2F, 10.0F);
    else if (!std::strcmp(key, "tlvmax")) ctrl_.config().tier.level_max_rad = std::clamp(v, 0.0F, 0.25F);
    else if (!std::strcmp(key, "hcap")) ctrl_.config().held_wheel_cap_m_s = std::clamp(v, 0.0F, 0.8F);
    else if (!std::strcmp(key, "leg_kp")) legs_.setMitGains(std::clamp(v, 0.0F, 60.0F), legs_.config().mit_kd_nm_s_per_rad);
    else if (!std::strcmp(key, "leg_kd")) legs_.setMitGains(legs_.config().mit_kp_nm_per_rad, std::clamp(v, 0.0F, 3.0F));
    else {
      Serial.printf("V3 SET unknown key '%s'\n", key);
      return;
    }
    printTunables();
  } else if (std::strcmp(line, "BALANCE") == 0) {
    if (!imuFresh(now_us)) {
      Serial.println("V3 BALANCE refused: IMU not ready");
      return;
    }
    // Measurement only: a reading taken on the stand is not the balance
    // angle, so it is not stored (use SET balance <rad> to store one).
    Serial.printf("V3 BALANCE measured pitch_avg=%+.4f rad (%+.2f deg) spread=%.4f rad "
                  "(not stored; SET balance %.4f to use it)\n",
                  pitch_avg_, pitch_avg_ * 57.29578F, pitch_spread_, pitch_avg_);
  } else if (std::strcmp(line, "BALANCE CLEAR") == 0) {
    ctrl_.setBalanceAngle(false, 0.0F);
    Serial.println("V3 BALANCE cleared: START seeds from the release pitch");
  } else if (std::strcmp(line, "GET") == 0) {
    printTunables();
  } else if (std::strncmp(line, "HIPTEST", 7) == 0) {
    float amp = 0.05F;
    float resend_ms = 4.0F;
    (void)std::sscanf(line + 7, "%f %f", &amp, &resend_ms);
    if (hip_test_ || hip_test_dumping_) {
      Serial.println("V3 HIPTEST refused: already running");
      return;
    }
    if (!legs_.engaged() || !legs_.atZero(now_us)) {
      Serial.println("V3 HIPTEST refused: legs not locked (robot on the stand, START first)");
      return;
    }
    hip_test_amp_rad_ = std::clamp(amp, 0.01F, 0.10F);
    hip_test_resend_us_ =
        static_cast<std::uint32_t>(std::clamp(resend_ms, 2.0F, 20.0F) * 1000.0F);
    legs_.setResendPeriod(hip_test_resend_us_);
    hip_test_ = true;
    hip_test_start_us_ = now_us;
    hip_test_n_ = 0U;
    Serial.printf("V3 HIPTEST start: amp=%.3f rad resend=%lu us, wheels off, %.1f s "
                  "(robot must be on the stand)\n", hip_test_amp_rad_,
                  static_cast<unsigned long>(hip_test_resend_us_), kHipTestDurationS);
  } else {
    Serial.printf("V3 unknown command '%s' (START DISABLE ESTOP CLEAR_ESTOP STOP DRIVE "
                  "GO HMOVE LEGMODE POS|MIT SET GET BALANCE [CLEAR] TILTTEST)\n", line);
  }
}

void AppV3::printTunables() noexcept {
  const auto& c = ctrl_.balancer().config();
  const auto& pid = ctrl_.config().pid;
  Serial.printf("V3 TUNE algo=%s (SET algo 1=PID 0=LQR) PID: kp=%.1f kd=%.2f kv=%.3f kx=%.3f "
                "kix=%.3f amax=%.1f lead=%.2f dbc=%.2f dbe=%.2f auto=%d balance=%s%+.4f "
                "height %.0f-%.0f mm hrate=%.0f mm/s\n",
                ctrl_.config().algo == V3Controller::Algo::Pid ? "PID" : "LQR", pid.kp, pid.kd,
                pid.kv, pid.kx, pid.kix, pid.accel_max_m_s2, pid.lead_s, pid.deadband_comp_rad_s,
                pid.deadband_eps_rad_s,
                ctrl_.config().engage.automatic ? 1 : 0,
                ctrl_.balanceAngleSet() ? "" : "(unset, release pitch) ",
                ctrl_.balanceAngle(), tap_fw::v3::LegHeight::minHeight() * 1000.0F,
                tap_fw::v3::LegHeight::maxHeight() * 1000.0F,
                ctrl_.config().height.rate_m_s * 1000.0F);
  const auto& tier = ctrl_.config().tier;
  Serial.printf("V3 TUNE tiers: vpitch=%d trunk=%d (LQR only) tshare=%.2f tlead=%.3f tmax=%.3f "
                "trate=%.1f thpf=%.1f texp=%.2f texpt=%.2f cdx=%.1f mm timu=%d tvg=%.2f\n",
                tier.virtual_pitch ? 1 : 0, tier.trunk ? 1 : 0, tier.lean_share, tier.lead_s,
                tier.max_rad, tier.rate_rad_s, tier.hpf_s, tier.height_exp, tier.height_exp_tall,
                tier.body_com_dx_m * 1000.0F, tier.imu_kinematic ? 1 : 0, tier.joint_rate_gain);
  Serial.printf("V3 TUNE preview: tprev=%d (needs vpitch) tpacc=%.2f tpjerk=%.1f tpvmax=%.2f "
                "tpt=%.3f\n", tier.preview ? 1 : 0, tier.plan_accel_m_s2, tier.plan_jerk_m_s3,
                tier.plan_vmax_m_s, tier.preview_s);
  Serial.printf("V3 TUNE level: tlvl=%d (needs trunk) tlevel=%+.3f rad tltau=%.1f tlvmax=%.3f "
                "hcap=%.2f m/s learned_trim=%s%+.4f\n", tier.level ? 1 : 0, tier.level_pitch_rad,
                tier.level_tau_s, tier.level_max_rad, ctrl_.config().held_wheel_cap_m_s,
                ctrl_.learnedTrimValid() ? "" : "(none) ", ctrl_.learnedTrim());
  Serial.printf("V3 TUNE legmode=%s leg_kp=%.1f leg_kd=%.2f trim_gain=%.2f trim_max=%.2f "
                "trim=%+.4f margin=%.2f accel=%.2f pitch_scale=%.2f vmax=%.2f\n",
                tap_fw::v3::toString(legs_.config().mode), legs_.config().mit_kp_nm_per_rad,
                legs_.config().mit_kd_nm_s_per_rad, c.balance_trim_gain_rad_per_m,
                c.balance_trim_max_rad, ctrl_.balancer().balanceTrim(),
                c.command_velocity_tracking_margin_m_s, c.max_ground_accel_m_s2,
                c.wheel_pitch_feedback_scale, c.max_ground_velocity_m_s);
}

void AppV3::drainFeedback(std::uint64_t now_us) noexcept {
  robot_control::motor_transport::CanFrame can_frame;
  for (unsigned n = 0; n < 64U && transport_.receiveCan(can_frame); ++n) {
    (void)feedback_.ingestCan(can_frame, now_us);
  }
  for (std::uint8_t bus = 0U; bus < 2U; ++bus) {
    frame::DxlFrame uart_frame;
    for (unsigned n = 0; n < 8U && transport_.receiveUart(bus, uart_frame); ++n) {
      (void)feedback_.ingestUart(bus, uart_frame, now_us);
    }
  }
}

void AppV3::updateImu(std::uint64_t now_us) noexcept {
  if (!imu_ready_ || !mpu_.read(chip_)) return;
  body_.accel_m_s2 = tap_fw::imu_mount::toBody(tap_fw::board::kImuAxisMap, chip_.accel_m_s2);
  body_.gyro_rad_s = tap_fw::imu_mount::toBody(tap_fw::board::kImuAxisMap, chip_.gyro_rad_s);
  const float dt_s = last_imu_us_ == 0U
      ? static_cast<float>(tap_fw::board::kControlPeriodUs) * 1.0e-6F
      : static_cast<float>(now_us - last_imu_us_) * 1.0e-6F;
  float accel_sq = 0.0F, gyro_sq = 0.0F;
  for (int i = 0; i < 3; ++i) {
    accel_sq += body_.accel_m_s2[i] * body_.accel_m_s2[i];
    gyro_sq += body_.gyro_rad_s[i] * body_.gyro_rad_s[i];
  }
  const bool quiet = std::abs(std::sqrt(accel_sq) - kGravity) < 0.35F &&
      std::sqrt(gyro_sq) < 0.12F;
  // Pure IMU: no wheel-kinematic acceleration compensation (see BoardConfig).
  // v3.3: while balancing free with the virtual pitch, the controller hands
  // the kinematic velocity of the IMU point (last tick) so a sustained
  // acceleration does not tilt the estimate (Tier::imu_kinematic); invalid
  // (pure IMU, as before) otherwise.
  const auto kv = ctrl_.imuVelocity();
  const auto out = estimator_.update(body_.accel_m_s2, body_.gyro_rad_s, dt_s,
                                     static_cast<float>(now_us) * 1.0e-6F, 0.0F,
                                     false, quiet,
                                     {kv.horizontal_m_s, kv.vertical_m_s, kv.valid});
  if (!out.valid) return;
  imu_ = out;
  last_imu_us_ = now_us;
  const float alpha = std::clamp(dt_s / kPitchAvgTauS, 0.0F, 1.0F);
  pitch_avg_ += alpha * (imu_.pitch_rad - pitch_avg_);
  pitch_spread_ += alpha * (std::abs(imu_.pitch_rad - pitch_avg_) - pitch_spread_);
}

bool AppV3::imuFresh(std::uint64_t now_us) const noexcept {
  return imu_.seeded && last_imu_us_ != 0U && now_us >= last_imu_us_ &&
      now_us - last_imu_us_ <= tap_fw::board::kFeedbackMaxAgeUs;
}

bool AppV3::legsFresh(std::uint64_t now_us) const noexcept {
  for (const std::size_t joint : {0U, 1U, 3U, 4U}) {
    const auto& e = feedback_.legEstimate(joint);
    if (!e.valid || now_us < e.stamp_us || now_us - e.stamp_us > kLegFreshUs) return false;
  }
  return true;
}

float AppV3::legJointRad(std::size_t leg) const noexcept {
  const std::size_t joint = tap_fw::board::MotorIds{}.leg_can[leg].joint_index;
  return feedback_.legEstimate(joint).pos_turns * kTwoPi / 8.0F;
}

float AppV3::legJointRate(std::size_t leg) const noexcept {
  const std::size_t joint = tap_fw::board::MotorIds{}.leg_can[leg].joint_index;
  return feedback_.legEstimate(joint).vel_turns_s * kTwoPi / 8.0F;
}

void AppV3::setHeightMove(int dir, std::uint64_t now_us) noexcept {
  dir = dir > 0 ? 1 : dir < 0 ? -1 : 0;
  if (dir == 0) {
    if (hmove_dir_ != 0) freezeHeight();
  } else {
    height_request_m_ = dir > 0 ? tap_fw::v3::LegHeight::maxHeight()
                                : tap_fw::v3::LegHeight::minHeight();
  }
  hmove_dir_ = dir;
  hmove_last_us_ = now_us;
}

void AppV3::pollBle(std::uint64_t now_us) noexcept {
  if (now_us < next_ble_poll_us_) return;
  next_ble_poll_us_ = now_us + kBlePollUs;
  // Bytes waiting from the phone = a Dabble packet (kept for diagnostics).
  if (esp32ble.available() > 0) last_phone_rx_us_ = now_us;
  Dabble.processInput();

  const bool link_up = g_v3_ble_links.load() > 0;
  if (link_up != ble_link_up_) {
    ble_link_up_ = link_up;
    ble_inputs_armed_ = false;
    Serial.printf("BLE %s\n", link_up ? "connected" : "disconnected");
    if (!link_up) {
      // Phone gone: stop driving and height moves (the session lease then
      // follows the USB rules: no serial either -> disabled after 0.5 s).
      if (ble_driving_) drive_forward_ = drive_yaw_ = 0.0F;
      ble_driving_ = false;
      if (ble_height_dir_ != 0) setHeightMove(0, now_us);
      ble_height_dir_ = 0;
    }
  }
  if (g_v3_ble_readvertise.exchange(false)) ble_advertise_at_us_ = now_us + kBleAdvertiseDelayUs;
  if (ble_advertise_at_us_ != 0U && now_us >= ble_advertise_at_us_) {
    ble_advertise_at_us_ = 0U;
    BLEDevice::startAdvertising();
  }
  if (!link_up) return;
  // A connected phone keeps the session lease alive (Dabble need not send
  // while nothing changes).
  last_rx_us_ = now_us;

  bool start = GamePad.isStartPressed();
  bool select = GamePad.isSelectPressed();
  bool triangle = GamePad.isTrianglePressed();
  bool cross = GamePad.isCrossPressed();
  // Joystick mode: radius 0..7 at a 15 deg angle step (0 = right, 90 = up);
  // D-pad mode reports the arrows instead.
  const float radius = static_cast<float>(GamePad.getRadius()) / 7.0F;
  const float angle = static_cast<float>(GamePad.getAngle()) * 0.01745329F;
  float x = radius * std::cos(angle);
  float y = radius * std::sin(angle);
  if (radius <= 0.0F) {
    x = static_cast<float>(GamePad.isRightPressed()) - static_cast<float>(GamePad.isLeftPressed());
    y = static_cast<float>(GamePad.isUpPressed()) - static_cast<float>(GamePad.isDownPressed());
  }
  // After (re)connect ignore inputs until everything is released once.
  const bool any = start || select || triangle || cross || std::abs(x) > 0.0F || std::abs(y) > 0.0F;
  if (!ble_inputs_armed_) {
    if (!any) ble_inputs_armed_ = true;
    start = select = triangle = cross = false;
    x = y = 0.0F;
  }
  // Dabble sends a packet only when an input changes, not while it is held
  // (2026-09-28: holding Cross moved the legs ~0.5 s per press with a packet
  // lease). So held inputs stay valid while the link is up; a disconnect
  // releases everything (above).

  if (start && !ble_start_prev_) handleLine("START", now_us);
  if (select && !ble_select_prev_) handleLine("GO", now_us);
  ble_start_prev_ = start;
  ble_select_prev_ = select;

  const int height_dir = triangle ? 1 : cross ? -1 : 0;
  if (height_dir != 0) {
    setHeightMove(height_dir, now_us);  // refreshed every poll while held
  } else if (ble_height_dir_ != 0) {
    setHeightMove(0, now_us);
  }
  ble_height_dir_ = height_dir;

  const bool stick = std::abs(x) > 0.05F || std::abs(y) > 0.05F;
  if (stick) {
    // Up = forward; right = turn right. On the robot (2026-09-28) a positive
    // DRIVE yaw turns it RIGHT, so stick right = + yaw.
    drive_forward_ = kBleDriveSpeedMS * y;
    drive_yaw_ = kBleYawRateRadS * x;
    ble_driving_ = true;
  } else if (ble_driving_) {
    drive_forward_ = drive_yaw_ = 0.0F;
    ble_driving_ = false;
  }
}

void AppV3::freezeHeight() noexcept {
  // Hold where the legs can stop smoothly from their current speed (the
  // controller ramps the speed down at Height::accel_m_s2), not where they
  // are this instant: that forced an abrupt stop on every button release.
  const float h = ctrl_.legPose().axle_below_hip_m;
  if (h <= 0.0F) return;
  const float v = ctrl_.heightSpeed();
  const float a = std::max(ctrl_.config().height.accel_m_s2, 1.0e-3F);
  height_request_m_ = std::clamp(h + v * std::abs(v) / (2.0F * a),
                                 tap_fw::v3::LegHeight::minHeight(),
                                 tap_fw::v3::LegHeight::maxHeight());
}

float AppV3::legTargetRad(std::size_t leg) const noexcept {
  const auto binding = tap_fw::board::MotorIds{}.leg_can[leg];
  const auto& pose = ctrl_.legPose();
  const bool knee = binding.joint_index == 1U || binding.joint_index == 4U;
  return (knee ? pose.knee_offset_rad : pose.hip_offset_rad) * binding.driver_sign;
}

float AppV3::legChainRad() const noexcept {
  // Mean over both legs of (hip + knee) in the core frame.
  float sum = 0.0F;
  for (std::size_t leg = 0; leg < LegLock::kLegs; ++leg) {
    sum += legJointRad(leg) * tap_fw::board::MotorIds{}.leg_can[leg].driver_sign;
  }
  return 0.5F * sum;
}

const char* AppV3::legAxisProblem(std::uint64_t now_us, bool need_closed_loop) const noexcept {
  const std::size_t joint = feedback_.firstLegAxisFault(now_us, need_closed_loop);
  if (joint == robot_control::kJointCount) return nullptr;
  static char text[64];
  const auto& axis = feedback_.legAxis(joint);
  std::snprintf(text, sizeof(text), "leg joint %u axis_state=%u axis_error=0x%08lX",
                static_cast<unsigned>(joint), static_cast<unsigned>(axis.state),
                static_cast<unsigned long>(axis.error));
  return text;
}

void AppV3::controlTick(std::uint64_t now_us) noexcept {
  feedback_.writeTo(state_);
  const auto st = ctrl_.state();
  const bool active = st == V3Controller::State::Locking || st == V3Controller::State::Ready ||
      st == V3Controller::State::Balancing;

  V3Controller::Command cmd{};
  cmd.forward_m_s = drive_forward_;
  cmd.yaw_rad_s = drive_yaw_;
  // HMOVE held but no refresh (GUI closed, link lost): stop where it is.
  if (hmove_dir_ != 0 && now_us - hmove_last_us_ > kHeightHoldUs) {
    freezeHeight();
    hmove_dir_ = 0;
  }
  cmd.height_m = height_request_m_;
  cmd.disable = disable_pending_ || estop_;
  if (active && now_us - last_rx_us_ > kLeaseUs) {
    cmd.disable = true;
    Serial.println("V3 serial lease expired (no command for 0.5 s): disabled");
  }

  // START checks that need the hardware: every leg near its power-up pose.
  if (start_pending_ && !cmd.disable) {
    start_pending_ = false;
    const char* refuse = nullptr;
    static char text[96];
    if (!legsFresh(now_us)) {
      refuse = "leg encoder estimates (0x009) missing - motor supply on?";
    } else if (!feedback_.allWheelsFresh(now_us)) {
      refuse = "wheel feedback missing";
    } else if (const char* axis = legAxisProblem(now_us, false)) {
      refuse = axis;
    } else {
      // Legs already held (START after a fault) may sit at a height offset.
      for (std::size_t leg = 0; leg < LegLock::kLegs && refuse == nullptr && !ctrl_.legsHeld();
           ++leg) {
        const float q = legJointRad(leg);
        if (std::abs(q) > kMaxStartOffsetRad) {
          std::snprintf(text, sizeof(text),
                        "%s is %+.2f rad from its power-up pose (max %.2f): re-place the "
                        "legs or power-cycle the motors", kLegNames[leg], q, kMaxStartOffsetRad);
          refuse = text;
        }
      }
    }
    if (refuse != nullptr) {
      Serial.printf("V3 START refused: %s\n", refuse);
    } else {
      cmd.start = true;
    }
  }
  disable_pending_ = false;
  // GO during the TILTTEST check: back to the armed Ready state.
  if (go_pending_ && tilt_test_ && st == V3Controller::State::Balancing) {
    tilt_test_ = false;
    go_pending_ = false;
    ctrl_.returnToReady();
    last_state_ = -1;  // re-run the state-change block: estimator, READY log
    Serial.println("V3 GO: TILTTEST -> armed Ready (balances after put down and let go)");
  }
  // TILTTEST skips the carry-to-floor wait (it is a hand-held check).
  const bool ready_now = ctrl_.state() == V3Controller::State::Ready;
  cmd.go = go_pending_ || (tilt_test_ && ready_now);
  go_pending_ = false;

  V3Controller::Input in{};
  in.now_us = now_us;
  in.dt_s = tick_dt_s_;  // measured tick period (fixed-rate schedule, clamped)
  in.pitch_rad = imu_.pitch_rad;
  // Bias-corrected gyro: the raw rate carried a -0.05 rad/s bias that read as
  // a steady backward rotation and wound the wheels up from the first tick.
  in.pitch_rate_rad_s = imu_.pitch_rate_rad_s;
  in.yaw_rate_rad_s = imu_.yaw_rate_rad_s;
  in.accel_x_m_s2 = body_.accel_m_s2[0];
  in.accel_z_m_s2 = body_.accel_m_s2[2];
  in.wheel_rad_s = {state_.joints.velocity_rad_s[2], state_.joints.velocity_rad_s[5]};
  in.wheel_pos_rad = {state_.joints.position_rad[2], state_.joints.position_rad[5]};
  in.leg_chain_rad = legChainRad();
  // v3.3: measured joints per side, core frame (LegLock order LH LK RH RK).
  {
    const auto legs = tap_fw::board::MotorIds{}.leg_can;
    for (std::size_t side = 0; side < 2U; ++side) {
      const std::size_t hip = 2U * side, knee = hip + 1U;
      in.hip_rad[side] = legJointRad(hip) * legs[hip].driver_sign;
      in.knee_rad[side] = legJointRad(knee) * legs[knee].driver_sign;
      in.hip_rate_rad_s[side] = legJointRate(hip) * legs[hip].driver_sign;
      in.knee_rate_rad_s[side] = legJointRate(knee) * legs[knee].driver_sign;
    }
    in.legs_measured = legsFresh(now_us);
    // Noise of the hip velocity estimates (they feed theta_v_rate): rms per
    // SENS report, meaningful with the legs holding still.
    for (std::size_t side = 0; side < 2U; ++side) {
      hip_vel_sq_ += in.hip_rate_rad_s[side] * in.hip_rate_rad_s[side];
    }
    hip_vel_n_ += 2U;
  }
  const bool closed_loop_due = legs_.engaged() && now_us - engage_us_ > kClosedLoopGraceUs;
  in.sensors_ok = imuFresh(now_us) && feedback_.allWheelsFresh(now_us) && legsFresh(now_us) &&
      legAxisProblem(now_us, closed_loop_due) == nullptr;
  bool legs_ready = legs_.atZero(now_us);
  for (std::size_t leg = 0; leg < LegLock::kLegs; ++leg) {
    legs_ready = legs_ready && std::abs(legJointRad(leg) - legTargetRad(leg)) <= kLegReadyRad;
  }
  in.legs_ready = legs_ready;
  if (st == V3Controller::State::Locking && now_us - state_since_us_ > kLockTimeoutUs) {
    cmd.disable = true;
    Serial.printf("V3 legs did not reach zero within 5 s: [%+.3f %+.3f %+.3f %+.3f] rad; "
                  "disabled\n", legJointRad(0), legJointRad(1), legJointRad(2), legJointRad(3));
  }
  if (!in.sensors_ok && active) {
    Serial.printf("V3 sensors: imu=%d wheels=%d legs=%d axis=%s\n", imuFresh(now_us),
                  feedback_.allWheelsFresh(now_us), legsFresh(now_us),
                  legAxisProblem(now_us, closed_loop_due) ? legAxisProblem(now_us, closed_loop_due)
                                                          : "ok");
  }

  const auto out = ctrl_.update(in, cmd);
  if (out.fault != nullptr) {
    Serial.printf("V3 FAULT: %s (wheels stopped, legs %s; START retries, DISABLE releases)\n",
                  out.fault, out.legs_locked ? "still locked" : "free");
  }

  if (out.engage_legs) {
    std::array<float, LegLock::kLegs> start{};
    for (std::size_t leg = 0; leg < LegLock::kLegs; ++leg) {
      start[leg] = feedback_.legEstimate(tap_fw::board::MotorIds{}.leg_can[leg].joint_index).pos_turns;
    }
    engage_us_ = now_us;
    legs_.setOffsets(ctrl_.legPose().hip_offset_rad, ctrl_.legPose().knee_offset_rad);
    (void)legs_.engage(start, now_us, transport_);
    Serial.printf("V3 legs locking (%s): from [%+.3f %+.3f %+.3f %+.3f] rad to 0\n",
                  tap_fw::v3::toString(legs_.config().mode), legJointRad(0), legJointRad(1),
                  legJointRad(2), legJointRad(3));
  }
  if (hip_test_ && (!out.legs_locked ||
                    now_us - hip_test_start_us_ >=
                        static_cast<std::uint64_t>(kHipTestDurationS * 1.0e6F))) {
    hip_test_ = false;
    hip_test_dumping_ = true;
    hip_test_dump_i_ = 0U;
    legs_.setResendPeriod(LegLock::Config{}.resend_period_us);
    Serial.printf("HT BEGIN n=%u amp=%.3f resend_us=%lu cols=t_ms,cmd,hipL,hipR,fb_age_ms%s\n",
                  static_cast<unsigned>(hip_test_n_), hip_test_amp_rad_,
                  static_cast<unsigned long>(hip_test_resend_us_),
                  out.legs_locked ? "" : " (ABORTED: legs released)");
  }
  if (out.legs_locked) {
    const float hip_test = hip_test_ ? hipTestOffset(now_us) : 0.0F;
    // v3.3: the height pose plus the tier-2 hip offset (0 unless trunk is
    // on and balancing); HIPTEST drives the bare pose.
    const float hip_base = hip_test_ ? ctrl_.legPose().hip_offset_rad : ctrl_.hipCommand();
    legs_.setOffsets(hip_base + hip_test, ctrl_.legPose().knee_offset_rad);
    (void)legs_.update(now_us, transport_);
    if (hip_test_) recordHipTest(now_us, ctrl_.legPose().hip_offset_rad + hip_test);
  } else if (legs_.engaged()) {
    (void)legs_.release(transport_);
  }
  if (hip_test_dumping_) dumpHipTest();
  auto wheel_cmd = out.wheel_cmd_rad_s;
  if (tilt_test_ && out.state == V3Controller::State::Balancing) {
    if (last_state_ != static_cast<int>(out.state)) tilt_ref_rad_ = imu_.pitch_rad;
    // Nose-down (+pitch) must roll the wheels toward the head (+).
    const float w = std::clamp(kTiltTestGain * (imu_.pitch_rad - tilt_ref_rad_),
                               -kTiltTestMaxRadS, kTiltTestMaxRadS);
    wheel_cmd = {w, w};
  }
  driveWheels(out.wheels_enabled && !hip_test_ && !hip_test_dumping_, wheel_cmd, now_us);
  (void)feedback_.poll(transport_, now_us, true);  // wheel reads + bus voltage, no MIT probes

  const int state = static_cast<int>(out.state);
  if (state != last_state_) {
    last_state_ = state;
    state_since_us_ = now_us;
    next_trace_us_ = now_us;
    Serial.printf("CONTROL mode=%s enable=%d result=0\n", tap_fw::v3::toString(out.state),
                  out.legs_locked ? 1 : 0);
    // Gyro-dominant pitch estimate only while balancing on the floor (see
    // balancingEstimatorConfig); TILTTEST is hand-held and keeps the base.
    const bool floor_balance = out.state == V3Controller::State::Balancing && !tilt_test_;
    estimator_.setConfig(floor_balance
        ? tap_fw::v3::balancingEstimatorConfig(robot_control::ImuStateEstimator::Config{})
        : robot_control::ImuStateEstimator::Config{});
    if (out.state == V3Controller::State::Ready) {
      Serial.println(tilt_test_ ? "V3 READY (TILTTEST: engaging at once)"
          : ctrl_.config().engage.automatic
          ? "V3 READY: wheels holding. Carry the robot to the floor, hold it still at its best "
            "standing point (0.7 s), let go: balancing starts when it tips (or send GO)"
          : "V3 READY: wheels holding. Put the robot on the floor, hold it, send GO");
    }
    if (out.state == V3Controller::State::Balancing) {
      Serial.printf("RELEASE pitch=%+.4f seeded_trim=%+.4f estimator=%s\n", imu_.pitch_rad,
                    ctrl_.balanceRef(), floor_balance ? "gyro-dominant" : "base");
    }
  }
  digitalWrite(tap_fw::board::kLed2Gpio, out.legs_locked ? HIGH : LOW);
  digitalWrite(tap_fw::board::kLed3Gpio, out.wheels_enabled ? HIGH : LOW);

  if (out.state != V3Controller::State::Idle && now_us >= next_trace_us_) {
    next_trace_us_ = now_us + kTraceUs;
    Serial.printf("TR t=%lu mode=%s pitch=%+.3f rate=%+.2f wcmd=[%+.2f %+.2f] "
                  "wmeas=[%+.2f %+.2f] axle_v=%+.3f axle_x=%+.3f trim=%+.4f "
                  "legs=[%+.3f %+.3f %+.3f %+.3f] ref=%+.4f ex=%+.3f ready=%d held=%d "
                  "h=%.1f htgt=%.1f knee=%+.3f tv=%+.4f dhip=%+.4f ta=%.2f me=%+.3f "
                  "lvl=%+.4f ho=%d lt=%+.4f/%d\n",
                  static_cast<unsigned long>((now_us - state_since_us_) / 1000U),
                  tap_fw::v3::toString(out.state), imu_.pitch_rad, imu_.pitch_rate_rad_s,
                  last_wheel_cmd_[0], last_wheel_cmd_[1], in.wheel_rad_s[0], in.wheel_rad_s[1],
                  ctrl_.groundVelocity(), ctrl_.groundPosition(), ctrl_.balanceRef(),
                  legJointRad(0), legJointRad(1), legJointRad(2), legJointRad(3),
                  ctrl_.pitchRef(), ctrl_.positionError(), ctrl_.readyPhase(),
                  ctrl_.heldResets(), ctrl_.legPose().axle_below_hip_m * 1000.0F,
                  ctrl_.heightTarget() * 1000.0F, ctrl_.legPose().knee_offset_rad,
                  ctrl_.virtualPitch(), ctrl_.trunkHip(), ctrl_.trunkAuthority(),
                  ctrl_.legModelError(), ctrl_.levelHip(), ctrl_.heldOuterOff() ? 1 : 0,
                  ctrl_.learnedTrim(), ctrl_.learnedTrimValid() ? 1 : 0);
  }
}

float AppV3::hipTestOffset(std::uint64_t now_us) const noexcept {
  const float t = static_cast<float>(now_us - hip_test_start_us_) * 1.0e-6F;
  const float a = hip_test_amp_rad_;
  if (t < 0.5F) return a;
  if (t < 1.0F) return 0.0F;
  if (t < 1.5F) return -a;
  if (t < 2.0F) return 0.0F;
  if (t < 6.0F) {
    const float seg = t - 2.0F;
    const int k = static_cast<int>(seg);  // 0..3 -> 1, 2, 4, 8 Hz
    const float f = static_cast<float>(1 << k);
    return a * std::sin(kTwoPi * f * (seg - static_cast<float>(k)));
  }
  return 0.0F;
}

void AppV3::recordHipTest(std::uint64_t now_us, float cmd_rad) noexcept {
  if (hip_test_n_ >= kHipTestMaxSamples) return;
  const auto legs = tap_fw::board::MotorIds{}.leg_can;
  // Leg order LH, LK, RH, RK: hips are 0 and 2; core frame = driver * sign.
  const float hip_l = legJointRad(0) * legs[0].driver_sign;
  const float hip_r = legJointRad(2) * legs[2].driver_sign;
  const std::uint64_t stamp = feedback_.legEstimate(legs[0].joint_index).stamp_us;
  auto& s = g_hip_test[hip_test_n_++];
  s.t_us = static_cast<std::uint32_t>(now_us - hip_test_start_us_);
  s.cmd_rad = cmd_rad;
  s.hip_l_rad = hip_l;
  s.hip_r_rad = hip_r;
  s.fb_age_us = static_cast<std::uint32_t>(now_us > stamp ? now_us - stamp : 0U);
}

void AppV3::dumpHipTest() noexcept {
  // A few lines per tick, only while the TX buffer is mostly empty: the
  // TR/SENS printf calls block when it is full, which stalled the loop and
  // let the wheel feedback go stale (fault) during the first dump.
  for (int lines = 0; lines < 2 && Serial.availableForWrite() > 2048; ++lines) {
    if (hip_test_dump_i_ >= hip_test_n_) {
      Serial.println("HT END");
      hip_test_dumping_ = false;
      ctrl_.returnToReady();  // fresh engage after the wheels were held off
      return;
    }
    const auto& s = g_hip_test[hip_test_dump_i_++];
    Serial.printf("HT %.1f %+.4f %+.4f %+.4f %.1f\n", s.t_us * 1.0e-3F, s.cmd_rad,
                  s.hip_l_rad, s.hip_r_rad, s.fb_age_us * 1.0e-3F);
  }
}

void AppV3::driveWheels(bool enabled, const std::array<float, 2>& cmd_rad_s,
                        std::uint64_t now_us) noexcept {
  const auto wheels = tap_fw::board::MotorIds{}.wheel_uart;
  if (!enabled) {
    last_wheel_cmd_ = {0.0F, 0.0F};
    if (wheels_on_) {
      for (std::size_t i = 0; i < wheels.size(); ++i) {
        (void)transport_.sendUart(static_cast<std::uint8_t>(i),
                                  frame::buildMovingSpeed(wheels[i].motor_id, 0.0F));
        (void)transport_.sendUart(static_cast<std::uint8_t>(i),
                                  frame::buildTorqueEnable(wheels[i].motor_id, false));
      }
      wheels_on_ = false;
    }
    return;
  }
  if (!wheels_on_) {
    for (std::size_t i = 0; i < wheels.size(); ++i) {
      (void)transport_.sendUart(static_cast<std::uint8_t>(i),
                                frame::buildTorqueEnable(wheels[i].motor_id, true));
    }
    wheels_on_ = true;
    wheels_on_since_us_ = now_us;
    return;
  }
  if (now_us - wheels_on_since_us_ < kWheelSettleUs) return;
  last_wheel_cmd_ = cmd_rad_s;
  for (std::size_t i = 0; i < wheels.size(); ++i) {
    // Wheel i drives the joint its binding names (left = joint 2, right = 5).
    const float core = wheels[i].joint_index == 2U ? cmd_rad_s[0] : cmd_rad_s[1];
    (void)transport_.sendUart(static_cast<std::uint8_t>(i),
                              frame::buildMovingSpeed(wheels[i].motor_id,
                                                      wheels[i].driver_sign * core));
  }
}

void AppV3::report(std::uint64_t now_us) noexcept {
  if (now_us - last_report_us_ < kReportUs) return;
  last_report_us_ = now_us;
  const long imu_age = last_imu_us_ == 0U ? -1L
      : static_cast<long>((now_us - last_imu_us_) / 1000U);
  // Same leading fields as the v1 SENS line, so the GUI status parser works.
  Serial.printf(
      "SENS imu_age=%ldms seed=%d can_legs=%d uart_wheels=%d homed=%d "
      "joint_q=[%.3f %.3f %.3f %.3f %.3f %.3f] pitch=%+.3f pitch_rate=%+.3f "
      "yaw_rate=%+.3f acc=[%+.2f %+.2f %+.2f] chip_acc=[%+.2f %+.2f %+.2f] "
      "gyro_bias=%+.4f vbus_min=%.2f state=%s axis=[%u %u %u %u] axis_err=[%lx %lx %lx %lx] "
      "pitch_avg=%+.4f spread=%.4f balance=%+.4f%s tick_max_us=%lu imu_us=%lu ctrl_us=%lu "
      "late=%lu serial_us=%lu ble_us=%lu bus_us=%lu hipv_rms=%.3f\n",
      imu_age, imu_.seeded, legsFresh(now_us), feedback_.allWheelsFresh(now_us),
      legs_.engaged() ? 1 : 0, legJointRad(0), legJointRad(1), state_.joints.position_rad[2],
      legJointRad(2), legJointRad(3), state_.joints.position_rad[5], imu_.pitch_rad,
      imu_.raw_pitch_rate_rad_s, imu_.yaw_rate_rad_s, body_.accel_m_s2[0],
      body_.accel_m_s2[1], body_.accel_m_s2[2], chip_.accel_m_s2[0], chip_.accel_m_s2[1],
      chip_.accel_m_s2[2], imu_.gyro_bias_rad_s, feedback_.takeMinBusVoltage(),
      tap_fw::v3::toString(ctrl_.state()),
      static_cast<unsigned>(feedback_.legAxis(0).state),
      static_cast<unsigned>(feedback_.legAxis(1).state),
      static_cast<unsigned>(feedback_.legAxis(3).state),
      static_cast<unsigned>(feedback_.legAxis(4).state),
      static_cast<unsigned long>(feedback_.legAxis(0).error),
      static_cast<unsigned long>(feedback_.legAxis(1).error),
      static_cast<unsigned long>(feedback_.legAxis(3).error),
      static_cast<unsigned long>(feedback_.legAxis(4).error), pitch_avg_, pitch_spread_,
      ctrl_.balanceAngle(), ctrl_.balanceAngleSet() ? "" : "(unset)",
      static_cast<unsigned long>(tick_exec_max_us_), static_cast<unsigned long>(tick_imu_max_us_),
      static_cast<unsigned long>(tick_ctrl_max_us_), static_cast<unsigned long>(tick_late_),
      static_cast<unsigned long>(pre_serial_max_us_), static_cast<unsigned long>(pre_ble_max_us_),
      static_cast<unsigned long>(pre_bus_max_us_),
      hip_vel_n_ > 0U ? std::sqrt(hip_vel_sq_ / static_cast<float>(hip_vel_n_)) : 0.0F);
  hip_vel_sq_ = 0.0F;
  hip_vel_n_ = 0U;
  pre_serial_max_us_ = 0U;
  pre_ble_max_us_ = 0U;
  pre_bus_max_us_ = 0U;
  tick_exec_max_us_ = 0U;
  tick_imu_max_us_ = 0U;
  tick_ctrl_max_us_ = 0U;
  tick_late_ = 0U;
}

}  // namespace tap_hw
