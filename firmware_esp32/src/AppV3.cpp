// CUSTOM_SETTINGS makes <DabbleESP32.h> define the GamePad object here; without it the
// header only emits an extern.
#define CUSTOM_SETTINGS
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
#include "robot_control/GeneratedGeometry.hpp"
#include "tap_fw/FastFormat.hpp"
#include "tap_fw/ImuMount.hpp"
#include "tap_fw/v3/JointTestSchedule.hpp"

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
// Joint-test / REC recorder: 250 Hz, 19 int16 channels with fixed scales
// (printed in the RC BEGIN header), 2000 samples = 8 s, ~88 KB.
constexpr std::uint64_t kRecPeriodUs = 4'000U;
constexpr std::size_t kRecMaxSamples = 2000U;
constexpr std::size_t kRecChannels = 19U;
constexpr float kRecScale[kRecChannels] = {
    1e-4F,                                    // cmd: excitation [rad]
    1e-4F, 1e-4F, 1e-4F, 1e-4F,               // hipL hipR kneeL kneeR [rad, core]
    1e-4F, 1e-4F,                             // hipcmd kneecmd [rad, core offsets]
    1e-4F, 1e-3F, 1e-3F, 1e-3F,               // pitch, pitch/roll/yaw rate
    1e-4F,                                    // theta_v
    1e-3F, 1e-3F, 1e-3F,                      // body accel x y z [m/s^2]
    1e-2F, 1e-2F, 1e-2F,                      // wheel L R measured, mean command [rad/s]
    1e-1F};                                   // leg feedback age [ms]
constexpr const char* kRecColumns =
    "t_ms,cmd,hipL,hipR,kneeL,kneeR,hipcmd,kneecmd,pitch,prate,rrate,yrate,tv,ax,ay,az,"
    "wL,wR,wcmd,fbage_ms";
struct RecSample {
  std::uint32_t t_us;
  std::int16_t v[kRecChannels];
};
RecSample g_rec[kRecMaxSamples];
// KNEETEST FLOOR: abort when the CoM line leaves the trim by this much (sim
// V21: +-0.03 rad knee swings it <= 0.05 at 189/136 mm), and refuse below
// this height (sim: a crouched robot already wobbles more).
constexpr float kKneeFloorAbortRad = 0.08F;
constexpr float kKneeFloorMinHeightM = 0.120F;

std::int16_t recQuant(float x, float scale) {
  const float q = std::round(x / scale);
  return static_cast<std::int16_t>(std::clamp(q, -32767.0F, 32767.0F));
}

const char* recName(int kind) {
  switch (kind) {
    case 1: return "HIPTEST";
    case 2: return "KNEETEST";
    case 3: return "KNEETEST_FLOOR";
    case 4: return "REC";
    default: return "?";
  }
}

std::uint64_t nowUs() { return static_cast<std::uint64_t>(esp_timer_get_time()); }

// SET legacy 1: the tier-2 settings of the b4b6d4a build (partial hip share also
// tall, fixed lead, slow level loop, fixed hip rate / level time, no knee hold /
// carrier / body damping), kept for A/B runs against the current ones (0).
void setLegacyTier(tap_fw::v3::V3Controller::Config& c, bool old) noexcept {
  auto& t = c.tier;
  t.height_exp = old ? 1.5F : 0.15F;
  t.height_exp_tall = old ? 0.5F : 0.0F;
  t.lead_exp = old ? 0.0F : 0.5F;
  t.level_tau_s = old ? 1.5F : 0.75F;
  t.adaptive_schedule = !old;
  t.knee_hold = !old;
  t.wheel_carrier_compensation = !old;
  t.wheel_carrier_gain = 0.10F;
  t.body_damping_s = old ? 0.0F : 0.08F;
}

}  // namespace

AppV3::AppV3() noexcept
    : transport_(can_.port(), uart1_.port(), uart2_.port()),
      ctrl_(tap_fw::v3::defaultV3Config()) {
  // Actuation prediction on at boot: the wheel command-to-speed lag is 85-120 ms on the robot. The predictor
  // keeps its command history across the microsecond jitter of the tick period since 18/10 (it used to be emptied
  // every tick, which made it predict as if no command were in flight). SET lqrpredict 0 turns it off.
  ctrl_.config().lqr_prediction.enabled = true;
  // Stand-up from the lying pose is enabled at boot. STANDUP still requires the
  // operator to confirm the encoder reference with SET lying 1 after powering
  // the motor supply on in the home pose.
  ctrl_.config().standup = tap_fw::v3::StandUp::tunedRecovery(ctrl_.config().lqr_prediction.lag_s);
  ctrl_.config().standup.enabled = true;
  // 19/10 REC fit (3 recordings, 8.5 s at 250 Hz, both wheels): wheel command-to-speed response is a ~30 ms
  // delay plus a ~130 ms lag (delay 20-40 ms, lag 100-170 ms within 3 % of the best fit; rmse 4.6-4.8 rad/s
  // against 5.2-5.5 for the previous 50 ms / 75 ms). Applied AFTER tunedRecovery() so the B4 catch lead
  // (which takes the lag as its argument) is unchanged. SET lqrdelay 0.05 / SET lqrlag 0.075 restore the
  // previous prediction; SET lqrlag is clamped to 0.12, so the 0.13 default is only reachable from here.
  ctrl_.config().lqr_prediction.delay_s = 0.03F;
  ctrl_.config().lqr_prediction.lag_s = 0.13F;
}

bool AppV3::begin() noexcept {
  Serial.setTxBufferSize(4096U);
  // 921600: at 115200 the TR + SENS stream alone used ~90 % of the link, and
  // a recorder dump on top stalled the loop on a full TX buffer until the
  // wheel feedback went stale (FAULT after KNEETEST, 2026-10-01).
  Serial.begin(921600);
  delay(300);
  pinMode(tap_fw::board::kLed1Gpio, OUTPUT);
  pinMode(tap_fw::board::kLed2Gpio, OUTPUT);
  pinMode(tap_fw::board::kLed3Gpio, OUTPUT);
  out_.println("TAP firmware v3.3beta: two-wheel inverted pendulum, legs locked at the "
                 "power-up pose (driver position mode), USB serial control");
  can_ready_ = can_.begin();
  uart1_.setSpeedPriority(true);
  uart2_.setSpeedPriority(true);
  uart_ready_ = uart1_.begin(tap_fw::board::kMotorUartBaud) &&
      uart2_.begin(tap_fw::board::kMotorUartBaud);
  imu_ready_ = mpu_.begin();
  out_.printf("TWAI=%s UART=%s MPU6500 WHO_AM_I=0x%02X (%s)\n",
                can_ready_ ? "PASS" : "FAIL", uart_ready_ ? "READY" : "FAIL",
                mpu_.whoAmI(), imu_ready_ ? "PASS" : "FAIL");
  out_.println("V3 procedure: arrange the legs BEFORE switching the motor supply on "
                 "(that pose is zero). Press START (legs lock, READY), put it on the "
                 "floor and hold it roughly still ~0.5 s until ARMED; it balances 0.5 s "
                 "later or as soon as it tips. DISABLE / ESTOP stop everything.");
  Dabble.begin("TAP-RB");
  BLEDevice::setCustomGattsHandler(onV3GattsEvent);
  out_.println("BLE: Dabble GamePad as TAP-RB (joystick drive, Start=START, Select=GO, "
                 "Triangle/Cross hold = raise/lower); USB serial GUI also works");
  // Shipped configuration: struct defaults (V3Controller.hpp) plus the overrides
  // below. Each override names the SET key that reverts it and the HANDOFF
  // section holding the evidence.
  //
  // Start handshake: put the robot on the floor and hold it roughly still 0.5 s
  // (pitch span 0.04 rad, the IMU alone reads up to 0.023 still) -> ARMED ->
  // balances 0.5 s later or as soon as it tips. SET imm 1 = balance right after
  // START; SET armgo 0 = wait for the tip only (with SET stillms / stillspan).
  ctrl_.config().engage.immediate = false;
  ctrl_.config().engage.still_us = 500'000U;
  ctrl_.config().engage.still_span_rad = 0.04F;
  ctrl_.config().engage.armed_go_s = 0.5F;
  // Hip share of the residual lean = (l/l_hang)^0.15 (0.88 at 78 mm): mechanism 1
  // must still act crouched. SET texp 1.5 restores the old crouch cut (HANDOFF 5.46).
  ctrl_.config().tier.height_exp = 0.15F;
  // Roll balance off: the lowering fall first appeared in the build that added it,
  // and it held a -9..-12 mm leg height difference (HANDOFF 5.45). SET roll 1
  // turns it on (and the 12 mm roll start reserve applies). rollvff stays off: it
  // made the side legs hunt (SET rollvff 1).
  ctrl_.config().roll.enabled = false;
  // Layered roll response, used once SET roll 1 (docs/ROLL_REDESIGN_20261005.md): level
  // reference learned in place (not a fixed -0.040), turn lean-in from the commanded yaw,
  // a latch that fades the loop out if it cannot level the body, and the loop belongs to the
  // gimbal (mechanism 2): it opens and freezes with the pitch level loop. rolldamp (gyro
  // damping) stays 0: it changed a 8 mm bump's peak by < 20 % in the sim.
  ctrl_.config().roll.learn_ref = true;
  ctrl_.config().roll.cmd_preview = true;
  ctrl_.config().roll.latch = true;
  ctrl_.config().roll.gimbal_link = true;  // roll = lateral axis of the gimbal (Tier::level)
  // Height-adaptive wheel-loop lead: lead = 0.05 + 0.04 * clamp((0.95 - ratio) /
  // 0.45) s. SET lqrlead <s> pins a fixed value; SET lqradapt 0/1 (HANDOFF 5.46).
  ctrl_.config().lqr_lead_adaptive = true;
  ctrl_.config().lqr_lead_tall_s = 0.05F;
  ctrl_.config().lqr_lead_low_s = 0.09F;
  // Noise-adaptive suspension: deadband = 3 x rms(vz error), engages 6 s after
  // balance starts, sleeps while the robot is not calm. SET tsusad 0 reverts to
  // the fixed deadband, SET tsus 0 turns the suspension off (HANDOFF 5.47).
  ctrl_.config().tier.sus_adaptive = true;
  // The knee follows the axle-height IK through a 1 Hz low-pass instead of copying
  // the hip reflex swing (knee grind when the reflex nears max_rad). SET kfhz 0 reverts.
  ctrl_.config().tier.knee_follow_hz = 1.0F;
  ctrl_.config().tier.knee_follow_rate_rad_s = 0.6F;
  // Balance angle from the measured ground acceleration (trim follows
  // theta_v - atan(a/g) with 3 s) so a height change does not leave the robot
  // drifting for ~25 s. SET ttrimacc 0 reverts, SET ttrimtau <s> retunes (5.46).
  ctrl_.config().trim_acc = true;
  // Gyro damping of the body through the hips, gated by the disturbance weight
  // (ungated 0.08 rings tall, >= 0.15 rings crouched). SET tbdamp <s> (5.46).
  ctrl_.config().tier.body_damping_s = 0.08F;
  // The damping reads the gyro through a 4.5 Hz low-pass: raw, its gate-open gain plus the
  // lead drives a 6-8 Hz hip limit cycle once the hip lags +30 ms or more (robot logs 07/10,
  // sim V42: gyro >5 Hz 0.14 -> 0.05 rad/s at +40 ms, quiet to +60 ms). SET tbdlpf 0 = raw gyro (HANDOFF 5.50).
  ctrl_.config().tier.body_damp_lpf_hz = 4.5F;
  // Height changes at 15 mm/s, 50 mm/s2 with a 200 mm/s3 jerk limit: the biggest
  // body kicks came when the legs STOP. SET hrate 25 / haccel 70 / hjerk 500
  // restores the faster profile.
  ctrl_.config().height.rate_m_s = 0.015F;
  ctrl_.config().height.accel_m_s2 = 0.05F;
  ctrl_.config().height.jerk_m_s3 = 0.20F;
  // Crouched, theta_v_rate takes 0.7 of the joint-rate term (1.0 tall): the
  // rotor-side hip rates lead the link under the crouched load and corrupt the
  // balancer's D channel (HANDOFF 5.36-5.37). SET tvglow -1 = off.
  ctrl_.config().tier.joint_rate_gain_low = 0.7F;
  // Not shipped: SET tlsrc 1 (lead from the body gyro instead of the hip encoder
  // rate) and SET tgate 1 (reflex gate) are sim-validated alternatives for the
  // standing hip chatter, off until A/B-tested on the robot (HANDOFF 5.41-5.42).
  printTunables();
  const std::uint64_t now_us = nowUs();
  next_control_us_ = now_us;
  last_rx_us_ = now_us;
  // FreeRTOS split (see AppV3.hpp). Output switches to the stream buffer
  // only once the control task exists; otherwise the single loop runs on.
  // Build with -DTAP_V3_SINGLE_LOOP=1 to keep the old single loop.
#if !defined(TAP_V3_SINGLE_LOOP) || !TAP_V3_SINGLE_LOOP
  line_q_ = xQueueCreate(16, sizeof(LineItem));
  if (line_q_ != nullptr && out_.begin(16384U)) {
    rtos_ = true;  // set before the task can run: pollSerial queues from now on
    if (xTaskCreatePinnedToCore(&AppV3::controlTaskEntry, "v3ctrl", 12288, this,
                                configMAX_PRIORITIES - 4, &control_task_, 1) != pdPASS) {
      rtos_ = false;
      Serial.println("V3 RTOS: control task not created - single loop");
    } else {
      if (xTaskCreatePinnedToCore(&AppV3::commsTaskEntry, "v3comms", 6144, this, 3,
                                  &comms_task_, 0) != pdPASS) {
        comms_task_ = nullptr;
      }
    }
  }
#endif
  return can_ready_ && uart_ready_ && imu_ready_;
}

void AppV3::loop() noexcept {
  if (!rtos_) {
    step(false);
    out_.drainTo(Serial);  // no-op unless the stream buffer exists without its task
    delayMicroseconds(100U);
    return;
  }
  if (comms_task_ != nullptr) {
    vTaskDelay(pdMS_TO_TICKS(100));  // serial runs in the comms task (core 0)
    return;
  }
  // Comms only: serial input lines to the control task, its output to the UART.
  pollSerial(nowUs());
  out_.drainTo(Serial);
  vTaskDelay(1);
}

void AppV3::commsTaskEntry(void* arg) noexcept {
  auto* app = static_cast<AppV3*>(arg);
  for (;;) {
    app->pollSerial(nowUs());
    app->out_.drainTo(Serial);
    vTaskDelay(1);
  }
}

void AppV3::controlTaskEntry(void* arg) noexcept {
  static_assert(configTICK_RATE_HZ >= 500, "the 2 ms control period needs a >= 500 Hz RTOS tick");
  auto* app = static_cast<AppV3*>(arg);
  const TickType_t period = pdMS_TO_TICKS(tap_fw::board::kControlPeriodUs / 1000U);
  // Once this task starts it is the only OutSink producer, including the
  // boot banner. Setup must not interleave framed records with this task.
  app->out_.println("V3 RTOS: control core 1 / 2 ms; deferred telemetry on serial task");
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, period);
    app->step(true);
  }
}

void AppV3::step(bool task) noexcept {
  const std::uint64_t now_us = nowUs();
  if (task) {
    // Command lines the loop task queued: handled here, so every state
    // change happens in this task.
    LineItem item;
    // Snapshot the bounded queue depth. A producer continuously adding
    // commands must not keep this task here instead of servicing sensors.
    const auto pending = uxQueueMessagesWaiting(line_q_);
    for (UBaseType_t i = 0; i < pending && xQueueReceive(line_q_, &item, 0) == pdTRUE; ++i)
      handleLine(item.text, now_us);
  } else {
    pollSerial(now_us);
  }
  const std::uint64_t t_serial = nowUs();
  pollBle(now_us);
  const std::uint64_t t_ble = nowUs();
  can_.service(now_us);
  drainFeedback(now_us);
  const std::uint64_t t_bus = nowUs();
  pre_serial_max_us_ = std::max(pre_serial_max_us_, static_cast<std::uint32_t>(t_serial - now_us));
  pre_ble_max_us_ = std::max(pre_ble_max_us_, static_cast<std::uint32_t>(t_ble - t_serial));
  pre_bus_max_us_ = std::max(pre_bus_max_us_, static_cast<std::uint32_t>(t_bus - t_ble));
  constexpr std::uint64_t kPeriod = tap_fw::board::kControlPeriodUs;
  if (task) {
    // The RTOS timer already paces the ticks (tick-aligned wake-ups may land
    // a few us before the old us schedule): run every wake, count a wake
    // more than half a period late.
    if (last_tick_us_ != 0U && now_us - last_tick_us_ > kPeriod + kPeriod / 2U) ++tick_late_;
    next_control_us_ = now_us + kPeriod;
    controlPart(now_us);
    return;
  }
  if (now_us >= next_control_us_) {
    // Fixed-rate schedule. `now + period` added each tick's own processing
    // time to the period: measured 2026-09-30 (HIPTEST) the loop ran at
    // ~440 Hz (2.27 ms mean, 3.5 ms max) while the controller integrated
    // with dt = 2 ms, i.e. ~12 % slow. A long stall resyncs instead of
    // bursting through the missed ticks.
    if (now_us - next_control_us_ > kPeriod) ++tick_late_;
    next_control_us_ += kPeriod;
    if (now_us >= next_control_us_ + 4U * kPeriod) next_control_us_ = now_us + kPeriod;
    controlPart(now_us);
  }
}

void AppV3::controlPart(std::uint64_t now_us) noexcept {
  constexpr std::uint64_t kPeriod = tap_fw::board::kControlPeriodUs;
  const std::uint64_t t_bus = nowUs();
  if (last_tick_us_ != 0U) tick_period_max_us_ = std::max(tick_period_max_us_,
      static_cast<std::uint32_t>(now_us - last_tick_us_));
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
  tick_report_max_us_ = std::max(tick_report_max_us_, static_cast<std::uint32_t>(t_end - t_ctrl));
  if (t_end - now_us > kPeriod) ++tick_overrun_;
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
    if (line_[0] == '\0') continue;
    if (rtos_) {
      LineItem item;
      std::strncpy(item.text, line_, sizeof(item.text) - 1U);
      item.text[sizeof(item.text) - 1U] = '\0';
      if (xQueueSend(line_q_, &item, 0) != pdTRUE) ++lines_dropped_;
    } else {
      handleLine(line_, now_us);
    }
  }
}

void AppV3::handleLine(const char* line, std::uint64_t now_us) noexcept {
  last_rx_us_ = now_us;  // any line keeps the session lease alive
  if (std::strcmp(line, "HEARTBEAT") == 0) return;
  auto& lqr = ctrl_.balancer();
  if (lying_start_ && (std::strcmp(line, "START") == 0 ||
                       std::strcmp(line, "TILTTEST") == 0 || std::strcmp(line, "GO") == 0)) {
    out_.println("V3 start refused: lying encoder frame requires STANDUP");
    return;
  }
  if (std::strcmp(line, "STANDUP") == 0) {
    // Rocking stand-up from the lying pose (Idle or Fault only): the legs lock where they are, the hips fold, and
    // the controller hands over to the balancer when the CoM is over the axle.
    if (estop_) {
      out_.println("V3 STANDUP refused: E-STOP latched (CLEAR_ESTOP)");
    } else if (!ctrl_.config().standup.enabled) {
      out_.println("V3 STANDUP refused: SET standup 1 first");
    } else if (!lying_start_) {
      out_.println("V3 STANDUP refused: SET lying 1 first (the supply must have been switched on in the home pose)");
    } else if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
      out_.println("V3 STANDUP refused: not idle (DISABLE first)");
    } else {
      // Match the 130 mm body/root height the sim stand-up uses.
      height_request_m_ = std::clamp(0.130F - robot_control::geometry::wheel_radius_m +
          robot_control::geometry::origins[0].z, tap_fw::v3::LegHeight::minHeight(),
          ctrl_.supportedMaxHeight());
      standup_pending_ = true;
      out_.println("V3 STANDUP");
    }
    return;
  }
  if (std::strcmp(line, "START") == 0 || std::strcmp(line, "TILTTEST") == 0) {
    if (estop_) {
      out_.println("V3 START refused: E-STOP latched (CLEAR_ESTOP)");
      return;
    }
    start_pending_ = true;
    tilt_test_ = line[0] == 'T';
    if (!ctrl_.legsHeld()) height_request_m_ = 0.0F;  // fresh lock: hang height
    out_.println(tilt_test_ ? "V3 START (TILTTEST: wheel = k*(pitch - pitch at release), "
                                "max 12 rad/s, no balance loop)"
                              : "V3 START");
  } else if (std::strcmp(line, "GO") == 0) {
    // Operator flow: GO is pressed with the robot on the stand. It only arms:
    // legs lock, wheels hold, and the balance loop engages by itself once the
    // robot has been carried to the floor, held still and let go. Balancing
    // on the stand would spin the wheels in the air into a runaway.
    const auto st = ctrl_.state();
    if (st == V3Controller::State::Ready) {
      out_.println("V3 GO: still Ready. Carry the robot to the floor, hold it still ~1 s "
                     "until ARMED, let go -> balancing starts by itself");
    } else if (st == V3Controller::State::Balancing && tilt_test_) {
      go_pending_ = true;  // leave TILTTEST for the armed Ready state
    } else if (st == V3Controller::State::Balancing || st == V3Controller::State::Locking) {
      out_.println("V3 GO: already running");
    } else if (estop_) {
      out_.println("V3 GO refused: E-STOP latched (CLEAR_ESTOP)");
    } else {
      start_pending_ = true;
      tilt_test_ = false;
      if (!ctrl_.legsHeld()) height_request_m_ = 0.0F;
      out_.println("V3 GO: locking legs, then wheels hold (armed; balances after the robot "
                     "is put on the floor and let go)");
    }
  } else if (std::strncmp(line, "HMOVE ", 6U) == 0) {
    int dir = 0;
    if (std::sscanf(line + 6, "%d", &dir) == 1) setHeightMove(dir, now_us);
  } else if (std::strcmp(line, "GO NOW") == 0) {
    if (ctrl_.state() == V3Controller::State::Ready) {
      go_pending_ = true;
      out_.println("V3 GO NOW: balance loop engaged immediately");
    }
  } else if (std::strcmp(line, "DISABLE") == 0) {
    disable_pending_ = true;
    tilt_test_ = false;
    go_after_ready_ = false;
    height_request_m_ = 0.0F;
    hmove_dir_ = 0;
    out_.println("V3 DISABLE");
  } else if (std::strcmp(line, "ESTOP") == 0) {
    estop_ = true;
    disable_pending_ = true;
    out_.println("V3 E-STOP latched");
  } else if (std::strcmp(line, "CLEAR_ESTOP") == 0) {
    estop_ = false;
    out_.println("V3 E-STOP cleared");
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
      out_.println("V3 LEGMODE refused: DISABLE first");
      return;
    }
    legs_.setMode(line[8] == 'P' ? LegLock::Mode::Position : LegLock::Mode::Mit);
    printTunables();
  } else if (std::strncmp(line, "SET ", 4U) == 0) {
    char key[20]{};
    float v = 0.0F;
    if (std::sscanf(line + 4, "%19s %f", key, &v) != 2 || !std::isfinite(v)) {
      out_.println("V3 SET usage: SET <key> <value>");
      return;
    }
    if (!std::strcmp(key, "trim_gain")) lqr.setBalanceTrimGain(std::clamp(v, 0.0F, 2.0F));
    else if (!std::strcmp(key, "trim_max")) lqr.setBalanceTrimMax(std::clamp(v, 0.0F, 0.35F));
    else if (!std::strcmp(key, "trim")) lqr.setBalanceTrim(v);
    else if (!std::strcmp(key, "margin")) lqr.setCommandTrackingMargin(std::clamp(v, 0.0F, 1.0F));
    // 0.2-10 m/s^2; the default is 8 (defaultV3Config). A low cap cuts the LQR's
    // push-recovery authority.
    else if (!std::strcmp(key, "accel")) lqr.setMaxGroundAccel(std::clamp(v, 0.2F, 10.0F));
    else if (!std::strcmp(key, "lqrpredict")) ctrl_.config().lqr_prediction.enabled = v >= 0.5F;
    else if (!std::strcmp(key, "lqrdelay")) ctrl_.config().lqr_prediction.delay_s = std::clamp(v, 0.0F, 0.1F);
    else if (!std::strcmp(key, "lqrlag")) ctrl_.config().lqr_prediction.lag_s = std::clamp(v, 0.0F, 0.12F);
    // Growing-oscillation guard (V3Controller::Config::Guard): mode 0 off / 1 measure and log / 2 act.
    else if (!std::strcmp(key, "guard")) ctrl_.config().guard.mode = std::clamp(static_cast<int>(v + 0.5F), 0, 2);
    else if (!std::strcmp(key, "guardcut")) ctrl_.config().guard.gain_cut = std::clamp(v, 0.0F, 0.5F);
    else if (!std::strcmp(key, "guardlag")) ctrl_.config().guard.lag_boost_s = std::clamp(v, 0.0F, 0.08F);
    else if (!std::strcmp(key, "guardraise")) ctrl_.config().guard.raise_max_m = std::clamp(v, 0.0F, 30.0F) * 1.0e-3F;
    else if (!std::strcmp(key, "guardhold")) ctrl_.config().guard.hold_risk = std::clamp(v, 0.05F, 1.0F);
    else if (!std::strcmp(key, "guardrisk")) ctrl_.config().guard.raise_risk = std::clamp(v, 0.1F, 1.0F);
    else if (!std::strcmp(key, "guardgrow")) ctrl_.config().guard.detect.growth_ratio = std::clamp(v, 1.02F, 2.0F);
    else if (!std::strcmp(key, "guardamp")) ctrl_.config().guard.detect.amp_min_rad = std::clamp(v, 0.001F, 0.05F);
    else if (!std::strcmp(key, "lying")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET lying refused: DISABLE first");
        return;
      }
      if (legs_.engaged() && lying_start_ != (v >= 0.5F)) {
        out_.println("V3 SET lying refused: DISABLE before changing the encoder frame");
        return;
      }
      lying_start_ = v >= 0.5F;
      out_.printf("V3 lying start %s (core hip %+.4f, knee %+.4f at the power-up pose)\n",
                  lying_start_ ? "ON" : "off", homeHip(), homeKnee());
    }
    else if (!std::strcmp(key, "standup")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standup refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.enabled = v >= 0.5F;
      out_.printf("V3 standup %s\n", ctrl_.config().standup.enabled ? "ENABLED" : "off");
    }
    else if (!std::strcmp(key, "standguard")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standguard refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.guarded_handoff = v >= 0.5F;
      out_.printf("V3 standguard %s\n", ctrl_.config().standup.guarded_handoff ? "ON" : "off (legacy handoff)");
    }
    else if (!std::strcmp(key, "standimu")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standimu refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.gyro_dominant_imu = v >= 0.5F;
      out_.printf("V3 standimu %s\n", ctrl_.config().standup.gyro_dominant_imu
                  ? "ON (gyro-dominant recovery)" : "off (base estimator during recovery)");
    }
    else if (!std::strcmp(key, "standcapture")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standcapture refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.capture_first = v >= 0.5F;
      out_.printf("V3 standcapture %s\n", ctrl_.config().standup.capture_first
                  ? "ON (capture first, retain leg pose after handoff)" : "off (combined body leveling)");
    }
    else if (!std::strcmp(key, "standcoord")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standcoord refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.coordinated_speed = v >= 0.5F;
      out_.printf("V3 standcoord=%d (1 speed error -> hip CoM target, 0 legacy wheel feedback)\n",
                  ctrl_.config().standup.coordinated_speed ? 1 : 0);
    }
    else if (!std::strcmp(key, "standhip")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standhip refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.support_hip_capture = v >= 0.5F;
      out_.printf("V3 standhip=%d (1 retain CoM hip feedback after handoff, 0 fixed hip)\n",
                  ctrl_.config().standup.support_hip_capture ? 1 : 0);
    }
    else if (!std::strcmp(key, "standlevel")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standlevel refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.catch_body_leveling = v >= 0.5F;
      out_.printf("V3 standlevel=%d (1 gated B4 anti-dive then Support leveling, 0 previous body law; knee/height held by standcapture)\n",
                  ctrl_.config().standup.catch_body_leveling ? 1 : 0);
    }
    else if (!std::strcmp(key, "wheelprio")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET wheelprio refused: DISABLE first");
        return;
      }
      uart1_.setSpeedPriority(v >= 0.5F);
      uart2_.setSpeedPriority(v >= 0.5F);
      out_.printf("V3 wheelprio=%d (two speed writes per read at most; encoder urgency 6 ms retained)\n",
                  v >= 0.5F ? 1 : 0);
    }
    else if (!std::strcmp(key, "standpostguard")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standpostguard refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.catch_posture_capture_guard = v >= 0.5F;
      out_.printf("V3 standpostguard=%d (B4 posture cannot add outward hip motion while COM/CP remain off-centre)\n",
                  ctrl_.config().standup.catch_posture_capture_guard ? 1 : 0);
    }
    else if (!std::strcmp(key, "standarmhip")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standarmhip refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.body_level_arm_hip_rad = std::clamp(v, 0.0F, 0.25F);
      out_.printf("V3 standarmhip=%.3f rad (B4 posture/leveling arms once the hip rose this far; 0.25 = previous)\n",
                  ctrl_.config().standup.body_level_arm_hip_rad);
    }
    else if (!std::strcmp(key, "standarmclr")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standarmclr refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.body_level_arm_clearance_m = std::clamp(v, 0.0F, 8.0F) * 0.001F;
      out_.printf("V3 standarmclr=%.1f mm (B4 posture/leveling also needs this modelled clearance; 8 = previous)\n",
                  ctrl_.config().standup.body_level_arm_clearance_m * 1000.0F);
    }
    else if (!std::strcmp(key, "recbal")) {
      rec_balance_auto_ = v >= 0.5F;
      out_.printf("V3 recbal=%d (1 record 8 s at 250 Hz when Balancing starts, then dump RC lines)\n",
                  rec_balance_auto_ ? 1 : 0);
    }
    else if (!std::strcmp(key, "trimtau")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET trimtau refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.balance_trim_ungated_tau_s = v <= 0.0F ? 0.0F : std::clamp(v, 0.3F, 10.0F);
      out_.printf("V3 trimtau=%.2f s (after a stand-up handoff the trim follows theta_v - atan(a/g) from ALL samples with this tau; 0 = old gated estimate)\n",
                  ctrl_.config().standup.balance_trim_ungated_tau_s);
    }
    else if (!std::strcmp(key, "trimwin")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET trimwin refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.balance_trim_window_rad = std::clamp(v, 0.0F, 0.2F);
      out_.printf("V3 trimwin=%.3f rad (after a stand-up handoff the LQR trim stays within +-this of its seed; 0 = off)\n",
                  ctrl_.config().standup.balance_trim_window_rad);
    }
    else if (!std::strcmp(key, "standspdi")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standspdi refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.catch_speed_integral_gain = std::clamp(v, 0.0F, 4.0F);
      out_.printf("V3 standspdi=%.2f 1/s^2 (B4 ground-speed integral added to the stopping lean target; 0 = off)\n",
                  ctrl_.config().standup.catch_speed_integral_gain);
    }
    else if (!std::strcmp(key, "standspdimax")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standspdimax refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.catch_speed_integral_max_rad = std::clamp(v, 0.0F, 0.12F);
      out_.printf("V3 standspdimax=%.3f rad (bound of that integral)\n",
                  ctrl_.config().standup.catch_speed_integral_max_rad);
    }
    else if (!std::strcmp(key, "standfloorsup")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standfloorsup refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.floor_guard_in_support = v >= 0.5F;
      out_.printf("V3 standfloorsup=%d (1 floor guard also after the LQR handoff, 0 guard only in B4)\n",
                  ctrl_.config().standup.floor_guard_in_support ? 1 : 0);
    }
    else if (!std::strcmp(key, "standbodygain")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standbodygain refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.recovery_body_gain = std::clamp(v, 0.5F, 8.0F);
      out_.printf("V3 standbodygain=%.2f (hip rate per rad of body pitch in the B4 posture term; 2.5 = previous)\n",
                  ctrl_.config().standup.recovery_body_gain);
    }
    else if (!std::strcmp(key, "standbodymax")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standbodymax refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.recovery_body_rate_max_rad_s =
          std::clamp(v, 0.2F, ctrl_.config().standup.catch_hip_rate_max_rad_s);
      out_.printf("V3 standbodymax=%.2f rad/s (cap of the B4 posture hip rate; 1.0 = previous; <= hip cap %.1f)\n",
                  ctrl_.config().standup.recovery_body_rate_max_rad_s,
                  ctrl_.config().standup.catch_hip_rate_max_rad_s);
    }
    else if (!std::strcmp(key, "standhandoff")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standhandoff refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.catch_rolling_handoff = v >= 0.5F;
      out_.printf("V3 standhandoff=%d (1 captured rolling handoff: 80 ms, cap allowed with measured tracking; 0 prior 200 ms/no-cap gate)\n",
                  ctrl_.config().standup.catch_rolling_handoff ? 1 : 0);
    }
    else if (!std::strcmp(key, "standwheelref")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standwheelref refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.catch_wheel_speed_lean = v >= 0.5F;
      out_.printf("V3 standwheelref=%d (1 B4 wheel lean loop follows the stopping lean_ref, 0 previous zero-lean wheel target)\n",
                  ctrl_.config().standup.catch_wheel_speed_lean ? 1 : 0);
    }
    else if (!std::strcmp(key, "standhospeed")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standhospeed refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.rolling_handoff_speed_m_s = v <= 0.0F ? 0.0F : std::clamp(v, 0.15F, 1.0F);
      out_.printf("V3 standhospeed=%.2f m/s (rolling handoff speed bound; 0 previous wheel cap + margin)\n",
                  ctrl_.config().standup.rolling_handoff_speed_m_s);
    }
    else if (!std::strcmp(key, "standtransfer")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standtransfer refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.catch_rolling_transfer = v >= 0.5F;
      out_.printf("V3 standtransfer=%d (1 settle captured hip posture then guarded rolling LQR transfer, 0 prior B4)\n",
                  ctrl_.config().standup.catch_rolling_transfer ? 1 : 0);
    }
    else if (!std::strcmp(key, "standbrake")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standbrake refused: DISABLE first");
        return;
      }
      auto& su = ctrl_.config().standup;
      su.catch_measured_braking = v >= 0.5F;
      if (su.catch_measured_braking) {
        su.catch_wheel_prediction = true;
        su.catch_capture_velocity = true;
      }
      out_.printf("V3 standbrake=%d (1 measured-motion preview + persistent acceleration integral, 0 prior forecast speed servo)\n",
                  su.catch_measured_braking ? 1 : 0);
    }
    else if (!std::strcmp(key, "standservo")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standservo refused: DISABLE first");
        return;
      }
      auto& su = ctrl_.config().standup;
      su.catch_capture_velocity = v >= 0.5F;
      if (su.catch_capture_velocity) su.catch_wheel_prediction = true;
      else su.catch_measured_braking = false;
      out_.printf("V3 standservo=%d standbrake=%d (standservo 0 disables measured braking; SET standbrake 0 restores prior forecast servo)\n",
                  su.catch_capture_velocity ? 1 : 0, su.catch_measured_braking ? 1 : 0);
    }
    else if (!std::strcmp(key, "standpred")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standpred refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.catch_wheel_prediction = v >= 0.5F;
      if (v < 0.5F) {
        ctrl_.config().standup.catch_capture_velocity = false;
        ctrl_.config().standup.catch_measured_braking = false;
      }
      out_.printf("V3 standpred=%d standservo=%d standbrake=%d (standpred 0 disables servo/brake; next recovery resets learned equilibrium)\n",
                  ctrl_.config().standup.catch_wheel_prediction ? 1 : 0,
                  ctrl_.config().standup.catch_capture_velocity ? 1 : 0,
                  ctrl_.config().standup.catch_measured_braking ? 1 : 0);
    }
    else if (!std::strcmp(key, "standcp")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standcp refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.catch_capture_point_hip = v >= 0.5F;
      out_.printf("V3 standcp=%d (1 filtered CoM angle+rate/omega hip feedback, 0 prior angle/damping law)\n",
                  ctrl_.config().standup.catch_capture_point_hip ? 1 : 0);
    }
    else if (!std::strcmp(key, "standbackdeg")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standbackdeg refused: DISABLE first");
        return;
      }
      auto& su = ctrl_.config().standup;
      su.back_pitch_limit_rad = v <= 0.0F ? 0.0F : std::clamp(v, 41.3F, 50.0F) * 0.0174532925F;
      out_.printf("V3 standbackdeg=%.1f (recovery backward body only; 0 symmetric limits; floor and COM limits retained)\n",
                  su.back_pitch_limit_rad * 57.2957795F);
    }
    else if (!std::strcmp(key, "standhipgain")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standhipgain refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.catch_hip_gain = std::clamp(v, 2.0F, 8.0F);
      out_.printf("V3 standhipgain=%.2f (CoM hip allocator, driver current unchanged; 4 prior tuning)\n",
                  ctrl_.config().standup.catch_hip_gain);
    }
    else if (!std::strcmp(key, "standfloor")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standfloor refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.floor_guard = v >= 0.5F;
      out_.printf("V3 standfloor=%d (1 clearance velocity damper, 0 prior recovery law; no pitch target)\n",
                  ctrl_.config().standup.floor_guard ? 1 : 0);
    }
    else if (!std::strcmp(key, "floorgrace")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET floorgrace refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.floor_accel_grace = v >= 0.5F;
      out_.printf("V3 floorgrace=%d (1 bounded accel while predicted clearance > margin, 0 strict Fault)\n",
                  ctrl_.config().standup.floor_accel_grace ? 1 : 0);
    }
    else if (!std::strcmp(key, "floormm")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET floormm refused: DISABLE first");
        return;
      }
      auto& su = ctrl_.config().standup;
      su.floor_clearance_m = std::clamp(v, 2.0F, 15.0F) * 0.001F;
      su.floor_arm_clearance_m = su.floor_clearance_m + 0.002F;
      out_.printf("V3 floormm=%.1f mm arm>=%.1f mm (model estimates)\n",
                  su.floor_clearance_m * 1000.0F, su.floor_arm_clearance_m * 1000.0F);
    }
    else if (!std::strcmp(key, "standwheel")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standwheel refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.catch_wheel_max_rad_s = std::clamp(v, 4.0F, 20.0F);
      out_.printf("V3 standwheel %.1f rad/s (recovery wheel command cap, SET standwheel 16 restores previous cap)\n",
                  ctrl_.config().standup.catch_wheel_max_rad_s);
    }
    else if (!std::strcmp(key, "standaccel")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standaccel refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.catch_accel_max = std::clamp(v, 1.0F, 5.0F);
      out_.printf("V3 standaccel %.2f m/s2 (command limit, not measured motor acceleration; previous=4)\n",
                  ctrl_.config().standup.catch_accel_max);
    }
    else if (!std::strcmp(key, "standwheelms")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standwheelms refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.catch_wheel_blend_s = v <= 0.0F ? 0.0F : std::clamp(v, 60.0F, 300.0F) * 0.001F;
      out_.printf("V3 standwheelms %.0f ms (wheel-only B4 ramp; 0 inherits hip ramp, previous=80 ms)\n",
                  ctrl_.config().standup.catch_wheel_blend_s * 1000.0F);
    }
    else if (!std::strcmp(key, "standspeed")) {
      if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
        out_.println("V3 SET standspeed refused: DISABLE first");
        return;
      }
      ctrl_.config().standup.catch_speed_gain = std::clamp(v, 0.0F, 2.0F);
      out_.printf("V3 standspeed %.2f /s (0 restores angle/rate-only capture)\n",
                  ctrl_.config().standup.catch_speed_gain);
    }
    else if (!std::strcmp(key, "vmax")) {
      // Ground-speed cap of the wheel loop (also the wheel command cap); 4.1888 m/s = the BL4012 no-load 1000 rpm.
      const float cap = std::clamp(v, 0.2F, 4.1888F);
      ctrl_.config().lqr.max_ground_velocity_m_s = cap;
      lqr.setMaxGroundVelocity(cap);
    }
    else if (!std::strcmp(key, "pitch_scale")) lqr.setWheelPitchFeedbackScale(std::clamp(v, 0.3F, 1.0F));
    else if (!std::strcmp(key, "balance")) ctrl_.setBalanceAngle(true, std::clamp(v, -0.5F, 0.5F));
    else if (!std::strcmp(key, "kp")) ctrl_.config().pid.kp = std::clamp(v, 0.0F, 200.0F);
    else if (!std::strcmp(key, "kd")) ctrl_.config().pid.kd = std::clamp(v, 0.0F, 30.0F);
    else if (!std::strcmp(key, "kv")) ctrl_.config().pid.kv = std::clamp(v, 0.0F, 1.0F);
    else if (!std::strcmp(key, "height")) {
      // Hip-to-axle height in mm; clamped to the planner's range.
      height_request_m_ = std::clamp(v * 0.001F, tap_fw::v3::LegHeight::minHeight(),
                                     tap_fw::v3::LegHeight::maxHeight());
      if (ctrl_.state() != V3Controller::State::Balancing) {
        out_.println("V3 SET height: stored; the legs move once balancing is settled");
      }
    }
    else if (!std::strcmp(key, "hff")) ctrl_.config().height.balance_ff_scale = std::clamp(v, -2.0F, 2.0F);
    else if (!std::strcmp(key, "hrate")) ctrl_.config().height.rate_m_s = std::clamp(v * 0.001F, 0.002F, 0.06F);
    else if (!std::strcmp(key, "haccel")) ctrl_.config().height.accel_m_s2 = std::clamp(v * 0.001F, 0.01F, 0.50F);
    else if (!std::strcmp(key, "hjerk")) ctrl_.config().height.jerk_m_s3 = std::clamp(v * 0.001F, 0.05F, 4.0F);
    else if (!std::strcmp(key, "dbc")) ctrl_.config().pid.deadband_comp_rad_s = std::clamp(v, 0.0F, 2.0F);
    else if (!std::strcmp(key, "dbe")) ctrl_.config().pid.deadband_eps_rad_s = std::clamp(v, 0.05F, 2.0F);
    else if (!std::strcmp(key, "lead")) ctrl_.config().pid.lead_s = std::clamp(v, 0.0F, 0.3F);
    else if (!std::strcmp(key, "kx")) ctrl_.config().pid.kx = std::clamp(v, 0.0F, 1.0F);
    else if (!std::strcmp(key, "kix")) ctrl_.config().pid.kix = std::clamp(v, 0.0F, 0.2F);
    else if (!std::strcmp(key, "auto")) ctrl_.config().engage.automatic = v >= 0.5F;
    else if (!std::strcmp(key, "imm")) ctrl_.config().engage.immediate = v >= 0.5F;
    else if (!std::strcmp(key, "armgo")) ctrl_.config().engage.armed_go_s = std::clamp(v, 0.0F, 5.0F);
    else if (!std::strcmp(key, "stillms")) ctrl_.config().engage.still_us = static_cast<std::uint64_t>(std::clamp(v, 200.0F, 3000.0F) * 1000.0F);
    else if (!std::strcmp(key, "stillspan")) ctrl_.config().engage.still_span_rad = std::clamp(v, 0.005F, 0.10F);
    else if (!std::strcmp(key, "wbarm")) ctrl_.config().engage.wb_armed_only = v >= 0.5F;
    else if (!std::strcmp(key, "wbgive")) ctrl_.config().wb_give_from = std::clamp(v, 0.3F, 1.0F);
    else if (!std::strcmp(key, "wbsoft")) ctrl_.config().wb_soft_from = std::clamp(v, 0.2F, 1.0F);
    else if (!std::strcmp(key, "tenv")) ctrl_.config().tier.envelope = v >= 0.5F;
    else if (!std::strcmp(key, "khold")) ctrl_.config().tier.knee_hold = v >= 0.5F;
    else if (!std::strcmp(key, "kfhz")) ctrl_.config().tier.knee_follow_hz = std::clamp(v, 0.0F, 20.0F);
    else if (!std::strcmp(key, "kfrate")) ctrl_.config().tier.knee_follow_rate_rad_s = std::clamp(v, 0.05F, 20.0F);
    else if (!std::strcmp(key, "tplanfull")) ctrl_.config().tier.plan_full = v >= 0.5F;
    else if (!std::strcmp(key, "leash")) ctrl_.config().drive_leash_m = std::clamp(v, 0.0F, 2.0F);
    else if (!std::strcmp(key, "roll")) ctrl_.config().roll.enabled = v >= 0.5F;
    else if (!std::strcmp(key, "rolltau")) ctrl_.config().roll.tau_s = std::clamp(v, 0.05F, 5.0F);
    else if (!std::strcmp(key, "rollmax")) ctrl_.config().roll.max_dh_m = std::clamp(v, 0.0F, 40.0F) * 1.0e-3F;
    else if (!std::strcmp(key, "rollres")) ctrl_.config().roll.reserve_m = std::clamp(v, 0.0F, 40.0F) * 1.0e-3F;
    else if (!std::strcmp(key, "rollff")) ctrl_.config().roll.turn_ff = std::clamp(v, 0.0F, 2.0F);
    else if (!std::strcmp(key, "rolltgt")) {  // pins the level target, no learning
      ctrl_.config().roll.target_rad = std::clamp(v, -0.15F, 0.15F);
      ctrl_.config().roll.learn_ref = false;
    }
    else if (!std::strcmp(key, "rolllearn")) ctrl_.config().roll.learn_ref = v >= 0.5F;
    else if (!std::strcmp(key, "rollcal")) ctrl_.requestRollRelearn();
    else if (!std::strcmp(key, "rollpre")) ctrl_.config().roll.cmd_preview = v >= 0.5F;
    else if (!std::strcmp(key, "rolldamp")) ctrl_.config().roll.damp_s = std::clamp(v, 0.0F, 0.5F);
    else if (!std::strcmp(key, "rolllatch")) ctrl_.config().roll.latch = v >= 0.5F;
    else if (!std::strcmp(key, "rollgimbal")) ctrl_.config().roll.gimbal_link = v >= 0.5F;
    else if (!std::strcmp(key, "rollyaw")) ctrl_.config().roll.yaw_accel_rad_s2 = std::clamp(v, 0.0F, 50.0F);
    else if (!std::strcmp(key, "rollsign")) ctrl_.config().roll.sign = v < 0.0F ? -1.0F : 1.0F;
    else if (!std::strcmp(key, "rollkp")) ctrl_.config().roll.kp = std::clamp(v, 0.0F, 3.0F);
    else if (!std::strcmp(key, "ttrimacc")) ctrl_.config().trim_acc = v >= 0.5F;
    else if (!std::strcmp(key, "ttrimtau")) ctrl_.config().trim_acc_tau_s = std::clamp(v, 0.5F, 20.0F);
    else if (!std::strcmp(key, "lqrlead")) {
      ctrl_.config().lqr_lead_s = std::clamp(v, 0.0F, 0.2F);
      ctrl_.config().lqr_lead_adaptive = false;  // a fixed value pins the lead
    }
    else if (!std::strcmp(key, "lqradapt")) ctrl_.config().lqr_lead_adaptive = v >= 0.5F;
    else if (!std::strcmp(key, "lqrleadtall")) ctrl_.config().lqr_lead_tall_s = std::clamp(v, 0.0F, 0.2F);
    else if (!std::strcmp(key, "lqrleadlow")) ctrl_.config().lqr_lead_low_s = std::clamp(v, 0.0F, 0.2F);
    else if (!std::strcmp(key, "wpred")) ctrl_.config().wheel_pred.enabled = v >= 0.5F;
    else if (!std::strcmp(key, "wbpred")) ctrl_.config().wheel_pred.with_wb = v >= 0.5F;
    else if (!std::strcmp(key, "wpdelay")) ctrl_.config().wheel_pred.act_delay_s = std::clamp(v, 0.0F, 200.0F) * 1.0e-3F;
    else if (!std::strcmp(key, "wptau")) ctrl_.config().wheel_pred.tau_s = std::clamp(v, 1.0F, 300.0F) * 1.0e-3F;
    else if (!std::strcmp(key, "wpmeas")) ctrl_.config().wheel_pred.meas_delay_s = std::clamp(v, 0.0F, 200.0F) * 1.0e-3F;
    else if (!std::strcmp(key, "rollvff")) ctrl_.config().roll.vel_ff = v >= 0.5F;
    else if (!std::strcmp(key, "wbkhold")) ctrl_.config().tier.knee_hold_wb = v >= 0.5F;
    else if (!std::strcmp(key, "wbenv")) ctrl_.config().tier.envelope_wb = v >= 0.5F;
    else if (!std::strcmp(key, "tenvlean")) ctrl_.config().tier.env_lean_rad = std::clamp(v, 0.0F, 0.3F);
    else if (!std::strcmp(key, "amax")) ctrl_.config().pid.accel_max_m_s2 = std::clamp(v, 0.5F, 30.0F);
    else if (!std::strcmp(key, "algo")) {
      if (ctrl_.state() == V3Controller::State::Balancing ||
          ctrl_.state() == V3Controller::State::Ready ||
          ctrl_.state() == V3Controller::State::Locking) {
        out_.println("V3 SET algo: only while Idle/Fault");
        return;
      }
      // 0 = LQR + tiers (default), 1 = PID, 2 = whole-body LQR (wheel + hip + knee).
      ctrl_.config().algo = v >= 1.5F ? V3Controller::Algo::WholeBody
                          : v >= 0.5F ? V3Controller::Algo::Pid
                                      : V3Controller::Algo::Lqr;
    }
    // Tiers (V3Controller::Tier). vpitch changes the balancer's state,
    // so it only switches outside Balancing; trunk ramps in and out anytime.
    else if (!std::strcmp(key, "vpitch")) {
      if (ctrl_.state() == V3Controller::State::Balancing) {
        out_.println("V3 SET vpitch: not while balancing (DISABLE or wait for a fault)");
        return;
      }
      ctrl_.config().tier.virtual_pitch = v >= 0.5F;
      if (v < 0.5F) ctrl_.config().tier.trunk = false;
    }
    else if (!std::strcmp(key, "trunk")) {
      if (v >= 0.5F && !ctrl_.config().tier.virtual_pitch) {
        out_.println("V3 SET trunk: needs SET vpitch 1 first");
        return;
      }
      ctrl_.config().tier.trunk = v >= 0.5F;
    }
    else if (!std::strcmp(key, "tshare")) ctrl_.config().tier.lean_share = std::clamp(v, 0.0F, 1.0F);
    else if (!std::strcmp(key, "tlead")) ctrl_.config().tier.lead_s = std::clamp(v, 0.0F, 0.15F);
    else if (!std::strcmp(key, "tleadexp")) ctrl_.config().tier.lead_exp = std::clamp(v, 0.0F, 2.0F);
    else if (!std::strcmp(key, "tlsrc")) ctrl_.config().tier.lead_gyro_only = v >= 0.5F;
    else if (!std::strcmp(key, "tgate")) ctrl_.config().tier.reflex_gate = v >= 0.5F;
    else if (!std::strcmp(key, "tglo")) ctrl_.config().tier.reflex_rate_lo = std::clamp(v, 0.0F, 2.0F);
    else if (!std::strcmp(key, "tghi")) ctrl_.config().tier.reflex_rate_hi = std::clamp(v, 0.01F, 3.0F);
    else if (!std::strcmp(key, "tgllo")) ctrl_.config().tier.reflex_lean_lo = std::clamp(v, 0.0F, 0.2F);
    else if (!std::strcmp(key, "tglhi")) ctrl_.config().tier.reflex_lean_hi = std::clamp(v, 0.005F, 0.3F);
    else if (!std::strcmp(key, "tgrel")) ctrl_.config().tier.reflex_release_s = std::clamp(v, 0.05F, 5.0F);
    else if (!std::strcmp(key, "tmax")) ctrl_.config().tier.max_rad = std::clamp(v, 0.0F, 0.30F);
    else if (!std::strcmp(key, "trate")) ctrl_.config().tier.rate_rad_s = std::clamp(v, 0.1F, 6.0F);
    else if (!std::strcmp(key, "texp")) ctrl_.config().tier.height_exp = std::clamp(v, 0.0F, 4.0F);
    else if (!std::strcmp(key, "texpt")) ctrl_.config().tier.height_exp_tall = std::clamp(v, 0.0F, 4.0F);
    else if (!std::strcmp(key, "cdx")) ctrl_.config().tier.body_com_dx_m = std::clamp(v * 0.001F, -0.06F, 0.06F);
    else if (!std::strcmp(key, "timu")) ctrl_.config().tier.imu_kinematic = v >= 0.5F;
    else if (!std::strcmp(key, "tvg")) ctrl_.config().tier.joint_rate_gain = std::clamp(v, 0.0F, 1.0F);
    else if (!std::strcmp(key, "tvglow")) ctrl_.config().tier.joint_rate_gain_low = v < 0.0F ? -1.0F : std::clamp(v, 0.0F, 1.0F);
    // Wheel/leg coordination pieces added after the b4b6d4a build, switchable for
    // A/B runs on the robot (SET legacy 1 sets them all back).
    else if (!std::strcmp(key, "tcarrier")) {
      ctrl_.config().tier.wheel_carrier_gain = std::clamp(v, 0.0F, 1.0F);
      ctrl_.config().tier.wheel_carrier_compensation = v > 0.0F;
    }
    else if (!std::strcmp(key, "tbdamp")) ctrl_.config().tier.body_damping_s = std::clamp(v, 0.0F, 0.2F);
    else if (!std::strcmp(key, "tbdlpf")) ctrl_.config().tier.body_damp_lpf_hz = std::clamp(v, 0.0F, 20.0F);
    else if (!std::strcmp(key, "tadapt")) ctrl_.config().tier.adaptive_schedule = v >= 0.5F;
    else if (!std::strcmp(key, "legacy")) setLegacyTier(ctrl_.config(), v >= 0.5F);
    // Push / standing calm (see Config::stand_leash_m, Tier::hip_deadband_rad,
    // Tier::sat_margin). sleash 0 = off; below 8 cm the catch itself fails.
    else if (!std::strcmp(key, "sleash")) ctrl_.config().stand_leash_m = v <= 0.0F ? 0.0F : std::clamp(v, 0.08F, 2.0F);
    else if (!std::strcmp(key, "sret")) ctrl_.config().stand_return_m_s = std::clamp(v, 0.02F, 0.5F);
    else if (!std::strcmp(key, "tdb")) ctrl_.config().tier.hip_deadband_rad = std::clamp(v, 0.0F, 0.02F);
    else if (!std::strcmp(key, "tsatm")) ctrl_.config().tier.sat_margin = std::clamp(v, 0.3F, 0.95F);
    // Maneuver preview (needs vpitch): planned, previewed speed changes.
    else if (!std::strcmp(key, "tprev")) ctrl_.config().tier.preview = v >= 0.5F;
    else if (!std::strcmp(key, "tpacc")) ctrl_.config().tier.plan_accel_m_s2 = std::clamp(v, 0.05F, 1.0F);
    else if (!std::strcmp(key, "tpjerk")) ctrl_.config().tier.plan_jerk_m_s3 = std::clamp(v, 0.2F, 5.0F);
    else if (!std::strcmp(key, "tpvmax")) ctrl_.config().tier.plan_vmax_m_s = std::clamp(v, 0.05F, 4.1888F);
    else if (!std::strcmp(key, "tpt")) ctrl_.config().tier.preview_s = std::clamp(v, 0.0F, 0.25F);
    // Static level loop (needs trunk): body pitch target and its limits.
    else if (!std::strcmp(key, "tlvl")) ctrl_.config().tier.level = v >= 0.5F;
    else if (!std::strcmp(key, "tlevel")) ctrl_.config().tier.level_pitch_rad = std::clamp(v, -0.30F, 0.30F);
    else if (!std::strcmp(key, "tltau")) ctrl_.config().tier.level_tau_s = std::clamp(v, 0.2F, 10.0F);
    else if (!std::strcmp(key, "tlvmax")) ctrl_.config().tier.level_max_rad = std::clamp(v, 0.0F, 0.25F);
    else if (!std::strcmp(key, "hcap")) ctrl_.config().held_wheel_cap_m_s = std::clamp(v, 0.0F, 0.8F);
    // Knee suspension (needs vpitch).
    else if (!std::strcmp(key, "tsus")) ctrl_.config().tier.suspension = v >= 0.5F;
    else if (!std::strcmp(key, "tsusad")) ctrl_.config().tier.sus_adaptive = v >= 0.5F;
    else if (!std::strcmp(key, "tsusk")) ctrl_.config().tier.sus_noise_k = std::clamp(v, 0.0F, 10.0F);
    else if (!std::strcmp(key, "tsusdelay")) ctrl_.config().tier.sus_engage_delay_s = std::clamp(v, 0.0F, 60.0F);
    else if (!std::strcmp(key, "tsusg")) ctrl_.config().tier.sus_gain = std::clamp(v, 0.0F, 5.0F);
    else if (!std::strcmp(key, "tsusmax")) ctrl_.config().tier.sus_max_m = std::clamp(v * 0.001F, 0.0F, 0.015F);
    // Whole-body LQR (SET algo 2).
    else if (!std::strcmp(key, "wbff")) ctrl_.config().wb_lean_ff = v >= 0.5F;
    else if (!std::strcmp(key, "wbslack")) ctrl_.config().wb_drive_slack_m = std::clamp(v, 0.0F, 0.1F);
    else if (!std::strcmp(key, "wbbg")) ctrl_.config().wb_bias_gain = std::clamp(v, 0.0F, 0.3F);
    else if (!std::strcmp(key, "leg_kp")) legs_.setMitGains(std::clamp(v, 0.0F, 60.0F), legs_.config().mit_kd_nm_s_per_rad);
    else if (!std::strcmp(key, "leg_kd")) legs_.setMitGains(legs_.config().mit_kp_nm_per_rad, std::clamp(v, 0.0F, 3.0F));
    else {
      out_.printf("V3 SET unknown key '%s'\n", key);
      return;
    }
    printTunables();
  } else if (std::strcmp(line, "BALANCE") == 0) {
    if (!imuFresh(now_us)) {
      out_.println("V3 BALANCE refused: IMU not ready");
      return;
    }
    // Measurement only: a reading taken on the stand is not the balance
    // angle, so it is not stored (use SET balance <rad> to store one).
    out_.printf("V3 BALANCE measured pitch_avg=%+.4f rad (%+.2f deg) spread=%.4f rad "
                  "(not stored; SET balance %.4f to use it)\n",
                  pitch_avg_, pitch_avg_ * 57.29578F, pitch_spread_, pitch_avg_);
  } else if (std::strcmp(line, "BALANCE CLEAR") == 0) {
    ctrl_.setBalanceAngle(false, 0.0F);
    out_.println("V3 BALANCE cleared: START seeds from the release pitch");
  } else if (std::strcmp(line, "GET") == 0) {
    printTunables();
  } else if (std::strncmp(line, "HIPTEST", 7) == 0 || std::strncmp(line, "KNEETEST", 8) == 0 ||
             std::strncmp(line, "REC", 3) == 0) {
    const bool hip = line[0] == 'H';
    const bool rec = line[0] == 'R';
    const bool floor = !hip && !rec && std::strstr(line, "FLOOR") != nullptr;
    const char* name = hip ? "HIPTEST" : rec ? "REC" : floor ? "KNEETEST FLOOR" : "KNEETEST";
    if (rec_active_ || rec_dumping_) {
      out_.printf("V3 %s refused: a recording is already running\n", name);
      return;
    }
    if (rec) {
      float seconds = 6.0F;
      (void)std::sscanf(line + 3, "%f", &seconds);
      startRecording(RecKind::Free, 0.0F, std::clamp(seconds, 1.0F, 8.0F), now_us);
      out_.printf("V3 REC start: %.1f s at 250 Hz (wheels and legs unchanged)\n",
                    rec_duration_s_);
      return;
    }
    if (floor) {
      float amp = 0.03F;
      (void)std::sscanf(line + 14, "%f", &amp);
      if (ctrl_.state() != V3Controller::State::Balancing || ctrl_.heldOuterOff()) {
        out_.println("V3 KNEETEST FLOOR refused: not balancing free on the floor");
        return;
      }
      if (!ctrl_.config().tier.virtual_pitch) {
        out_.println("V3 KNEETEST FLOOR refused: needs SET vpitch 1 (the v3.2 balancer "
                       "reads every leg motion as a body tilt)");
        return;
      }
      if (ctrl_.legPose().axle_below_hip_m < kKneeFloorMinHeightM) {
        out_.printf("V3 KNEETEST FLOOR refused: below %.0f mm\n",
                      kKneeFloorMinHeightM * 1000.0F);
        return;
      }
      startRecording(RecKind::KneeFloor, std::clamp(amp, 0.01F, 0.05F),
                     tap_fw::v3::kFloorTestDurationS, now_us);
      out_.printf("V3 KNEETEST FLOOR start: knee +-%.3f rad (centre %+.3f) at 0.5/1/2/3 Hz, "
                    "%.1f s, aborts past %.2f rad of CoM lean\n", rec_amp_, rec_centre_,
                    rec_duration_s_, kKneeFloorAbortRad);
      return;
    }
    float amp = 0.05F;
    float resend_ms = 4.0F;
    (void)std::sscanf(line + (hip ? 7 : 8), "%f %f", &amp, &resend_ms);
    if (!legs_.engaged() || !legs_.atZero(now_us)) {
      out_.printf("V3 %s refused: legs not locked (robot on the stand, START first)\n", name);
      return;
    }
    rec_resend_us_ = static_cast<std::uint32_t>(std::clamp(resend_ms, 2.0F, 20.0F) * 1000.0F);
    legs_.setResendPeriod(rec_resend_us_);
    startRecording(hip ? RecKind::HipStand : RecKind::KneeStand, std::clamp(amp, 0.01F, 0.10F),
                   tap_fw::v3::kStandTestDurationS, now_us);
    out_.printf("V3 %s start: amp=%.3f rad resend=%lu us, wheels off, %.1f s "
                  "(robot must be on the stand)\n", name, rec_amp_,
                  static_cast<unsigned long>(rec_resend_us_), rec_duration_s_);
  } else {
    out_.printf("V3 unknown command '%s' (START DISABLE ESTOP CLEAR_ESTOP STOP DRIVE "
                  "GO HMOVE LEGMODE POS|MIT SET GET BALANCE [CLEAR] TILTTEST HIPTEST KNEETEST [FLOOR] "
                  "REC)\n", line);
  }
}

void AppV3::printTunables() noexcept {
  // The full configuration dump uses many printf float conversions (~8 ms
  // in the active log). Apply SET immediately, but print the full dump only
  // after the robot is inactive. GET/SET cannot stall active motor updates.
  if (ctrl_.state() != V3Controller::State::Idle && ctrl_.state() != V3Controller::State::Fault) {
    if (!tunables_pending_) out_.println("V3 TUNE deferred until Idle/Fault to preserve control timing");
    tunables_pending_ = true;
    return;
  }
  tunables_pending_ = false;
  out_.printf("V3 TUNE wheelprio=%d/%d (SET wheelprio): newest speed first, max 2 writes/read; "
              "read urgency 6 ms, transaction timeout 8 ms; torque-on settle 20 ms; "
              "WT=actual TX, WBUS=queue/ACK timing; motor response delay not measured at boot\n",
              uart1_.speedPriority() ? 1 : 0, uart2_.speedPriority() ? 1 : 0);
  const auto& c = ctrl_.balancer().config();
  const auto& pid = ctrl_.config().pid;
  out_.printf("V3 TUNE algo=%s (SET algo 0=LQR 1=PID 2=WB) PID: kp=%.1f kd=%.2f kv=%.3f kx=%.3f "
                "kix=%.3f amax=%.1f lead=%.2f dbc=%.2f dbe=%.2f auto=%d balance=%s%+.4f "
                "height %.0f-%.0f mm hrate=%.0f mm/s haccel=%.0f mm/s2 hjerk=%.0f mm/s3\n",
                ctrl_.config().algo == V3Controller::Algo::Pid       ? "PID"
                : ctrl_.config().algo == V3Controller::Algo::WholeBody ? "WB"
                                                                       : "LQR",
                pid.kp, pid.kd,
                pid.kv, pid.kx, pid.kix, pid.accel_max_m_s2, pid.lead_s, pid.deadband_comp_rad_s,
                pid.deadband_eps_rad_s,
                ctrl_.config().engage.automatic ? 1 : 0,
                ctrl_.balanceAngleSet() ? "" : "(unset, release pitch) ",
                ctrl_.balanceAngle(), tap_fw::v3::LegHeight::minHeight() * 1000.0F,
                tap_fw::v3::LegHeight::maxHeight() * 1000.0F,
                ctrl_.config().height.rate_m_s * 1000.0F,
                ctrl_.config().height.accel_m_s2 * 1000.0F,
                ctrl_.config().height.jerk_m_s3 * 1000.0F);
  const auto& tier = ctrl_.config().tier;
  const auto& schedule = ctrl_.trunkSchedule();
  out_.printf("V3 TUNE coordinated: axle_path=paired height_adapt=encoder lead=%.3f rate=%.2f level_tau=%.3f cap=%.3f leg_send_us=%lu\n",
              schedule.lead_s, schedule.rate_rad_s, schedule.level_tau_s, schedule.max_rad,
              static_cast<unsigned long>(legs_.config().resend_period_us));
  out_.printf("V3 TUNE wheel/leg: tcarrier=%.2f tbdamp=%.3f s khold=%d tltau=%.2f tleadexp=%.2f tadapt=%d height_handover=planned damping_gate=virtual disturbance_cap=travel recovery_lead=height_scaled level_integral=workspace\n",
              ctrl_.config().tier.wheel_carrier_compensation ? ctrl_.config().tier.wheel_carrier_gain : 0.0F,
              ctrl_.config().tier.body_damping_s, ctrl_.config().tier.knee_hold ? 1 : 0,
              ctrl_.config().tier.level_tau_s, ctrl_.config().tier.lead_exp,
              ctrl_.config().tier.adaptive_schedule ? 1 : 0);
  out_.printf("V3 TUNE ready: armgo=%.2f s stillms=%lu stillspan=%.3f rad still_rate=%.2f tip_rate=%.2f tip_pitch=%.3f\n",
              ctrl_.config().engage.armed_go_s,
              static_cast<unsigned long>(ctrl_.config().engage.still_us / 1000U),
              ctrl_.config().engage.still_span_rad, ctrl_.config().engage.still_rate_rad_s,
              ctrl_.config().engage.tip_rate_rad_s, ctrl_.config().engage.tip_pitch_rad);
  out_.printf("V3 TUNE startup: auto=%d imm=%d height_wait=%.2f s height_rate=%.1f mm/s height_accel=%.1f mm/s2 height_jerk=%.0f mm/s3 sus_settle=%.2f s sus_ramp=%.2f s\n",
              ctrl_.config().engage.automatic ? 1 : 0, ctrl_.config().engage.immediate ? 1 : 0,
              ctrl_.config().height.startup_delay_s, ctrl_.config().height.startup_rate_m_s * 1000.0F,
              ctrl_.config().height.startup_accel_m_s2 * 1000.0F,
              ctrl_.config().height.startup_jerk_m_s3 * 1000.0F,
              tier.sus_startup_settle_s, tier.sus_startup_ramp_s);
  out_.printf("V3 TUNE tiers: vpitch=%d trunk=%d (LQR only) tshare=%.2f disturbance_share=%.2f trunk_feedback=virtual thpf=%.1f "
                "tlead=%.3f tleadexp=%.2f tlsrc=%d tplanfull=%d tmax=%.3f trate=%.1f texp=%.2f texpt=%.2f cdx=%.1f mm timu=%d tvg=%.2f tvglow=%.2f\n",
                tier.virtual_pitch ? 1 : 0, tier.trunk ? 1 : 0, tier.lean_share, tier.disturbance_share,
                tier.hpf_s, tier.lead_s, tier.lead_exp, tier.lead_gyro_only ? 1 : 0, tier.plan_full ? 1 : 0, tier.max_rad, tier.rate_rad_s,
                tier.height_exp, tier.height_exp_tall,
                tier.body_com_dx_m * 1000.0F, tier.imu_kinematic ? 1 : 0, tier.joint_rate_gain,
                tier.joint_rate_gain_low);
  out_.printf("V3 TUNE body COM: base_x=%+.2f mm extra_cdx=%+.2f mm effective_x=%+.2f mm (body frame, +X forward)\n",
              robot_control::geometry::parts[0].x * 1000.0F,
              tier.body_com_dx_m * 1000.0F,
              (robot_control::geometry::parts[0].x + tier.body_com_dx_m) * 1000.0F);
  out_.printf("V3 TUNE body damping: tbdamp=%.3f s tbdlpf=%.1f Hz (0 = raw gyro)\n",
              ctrl_.config().tier.body_damping_s, ctrl_.config().tier.body_damp_lpf_hz);
  out_.printf("V3 TUNE knee follow: kfhz=%.2f Hz (0 = instant IK knee) kfrate=%.2f rad/s\n",
              ctrl_.config().tier.knee_follow_hz, ctrl_.config().tier.knee_follow_rate_rad_s);
  out_.printf("V3 TUNE suspension: tsus=%d adaptive=%d (SET tsusad) k=%.1f (tsusk) engage delay %.1f s (tsusdelay) deadband in use %.1f mm/s\n",
              ctrl_.config().tier.suspension ? 1 : 0, ctrl_.config().tier.sus_adaptive ? 1 : 0,
              ctrl_.config().tier.sus_noise_k, ctrl_.config().tier.sus_engage_delay_s,
              ctrl_.suspensionDeadband() * 1000.0F);
  out_.printf("V3 TUNE lqr lead: adaptive=%d (SET lqradapt) tall=%.3f low=%.3f s (lqrleadtall/lqrleadlow) fixed=%.3f s (SET lqrlead pins it)\n",
              ctrl_.config().lqr_lead_adaptive ? 1 : 0, ctrl_.config().lqr_lead_tall_s,
              ctrl_.config().lqr_lead_low_s, ctrl_.config().lqr_lead_s);
  out_.printf("V3 TUNE reflex gate: tgate=%d (SET tgate 0/1) rate %.2f-%.2f rad/s (tglo/tghi) lean %.3f-%.3f rad (tgllo/tglhi) release %.2f s (tgrel)\n",
              tier.reflex_gate ? 1 : 0, tier.reflex_rate_lo, tier.reflex_rate_hi,
              tier.reflex_lean_lo, tier.reflex_lean_hi, tier.reflex_release_s);
  out_.printf("V3 TUNE calm: sleash=%.2f m sfrom=%.2f sret=%.2f m/s srel=%.1f s tdb=%.4f rad tsatm=%.2f\n",
              ctrl_.config().stand_leash_m, ctrl_.config().stand_leash_from,
              ctrl_.config().stand_return_m_s, ctrl_.config().stand_leash_release_s,
              tier.hip_deadband_rad, tier.sat_margin);
  out_.printf("V3 TUNE preview: tprev=%d (needs vpitch) tpacc=%.2f tpjerk=%.1f tpvmax=%.2f "
                "tpt=%.3f\n", tier.preview ? 1 : 0, tier.plan_accel_m_s2, tier.plan_jerk_m_s3,
                tier.plan_vmax_m_s, tier.preview_s);
  out_.printf("V3 TUNE suspension: tsus=%d (needs vpitch) tsusg=%.1f tsusmax=%.0f mm\n",
                tier.suspension ? 1 : 0, tier.sus_gain, tier.sus_max_m * 1000.0F);
  out_.printf("V3 TUNE level: tlvl=%d (needs trunk) tlevel=%+.3f rad tltau=%.1f tlvmax=%.3f "
                "hcap=%.2f m/s learned_trim=%s%+.4f\n", tier.level ? 1 : 0, tier.level_pitch_rad,
                tier.level_tau_s, tier.level_max_rad, ctrl_.config().held_wheel_cap_m_s,
                ctrl_.learnedTrimValid() ? "" : "(none) ", ctrl_.learnedTrim());
  {
    const auto& rl = ctrl_.config().roll;
    out_.printf("V3 TUNE roll: roll=%d rolltau=%.2f rollkp=%.2f rollmax=%.0f mm rollres=%.0f mm rollff=%.2f "
                  "rolltgt=%+.3f rollyaw=%.1f rollsign=%+.0f rollvff=%d est=%+.4f\n", rl.enabled ? 1 : 0, rl.tau_s, rl.kp,
                  rl.max_dh_m * 1000.0F, rl.reserve_m * 1000.0F, rl.turn_ff, rl.target_rad,
                  rl.yaw_accel_rad_s2, rl.sign, rl.vel_ff ? 1 : 0, ctrl_.rollEstimate());
    out_.printf("V3 TUNE roll layers: learn=%d (SET rolllearn; SET rollcal re-learns; SET rolltgt pins) ref=%+.4f "
                  "preview=%d (rollpre) damp=%.3f s (rolldamp) latch=%d (rolllatch) gimbal=%d (rollgimbal) mode=%d\n",
                  rl.learn_ref ? 1 : 0, ctrl_.rollReference(), rl.cmd_preview ? 1 : 0, rl.damp_s,
                  rl.latch ? 1 : 0, rl.gimbal_link ? 1 : 0, ctrl_.rollMode());
  }
  {
    const auto& wp = ctrl_.config().wheel_pred;
    out_.printf("V3 TUNE wheel predictor: wpred=%d wbpred=%d wpdelay=%.0f ms wptau=%.0f ms wpmeas=%.0f ms\n",
                wp.enabled ? 1 : 0, wp.with_wb ? 1 : 0, wp.act_delay_s * 1000.0F, wp.tau_s * 1000.0F,
                wp.meas_delay_s * 1000.0F);
  }
  out_.printf("V3 TUNE standup=%d (SET standup) lying=%d (SET lying) home_hip=%+.4f home_knee=%+.4f\n",
              ctrl_.config().standup.enabled ? 1 : 0, lying_start_ ? 1 : 0, homeHip(), homeKnee());
  {
    const auto& su = ctrl_.config().standup;
    out_.printf("V3 TUNE standguard=%d (SET standguard) standimu=%d (SET standimu) fold=%.2f skip_pitching=%d "
                "settle=%.2f s body<=%.2f rad rate<=%.2f rad/s wheel_error<=%.1f rad/s "
                "clearance>=%.1f mm (model estimate); standspeed=%.2f /s (SET standspeed); wheel_peak_hw=0\n",
                su.guarded_handoff ? 1 : 0, su.gyro_dominant_imu ? 1 : 0,
                su.fold_travel_rad, su.skip_pitching ? 1 : 0,
                su.handoff_settle_s, su.handoff_body_tol_rad, su.handoff_body_rate_tol_rad_s,
                su.handoff_wheel_tracking_tol_rad_s, su.lift_clearance_m * 1000.0F, su.catch_speed_gain);
    out_.printf("V3 TUNE standcapture=%d (SET standcapture): legacy body leveling and pitch-zero handoff gate=%d; "
                "preserve recovery knee/height=%d; hard pitch limits unchanged\n",
                su.capture_first ? 1 : 0, su.capture_first ? 0 : 1, su.capture_first ? 1 : 0);
    out_.printf("V3 TUNE standcoord=%d (SET standcoord) speed_lean<=%.3f rad filter=%.2f s; "
                "knee=fixed (no opening); handoff_speed<=%.2f m/s; "
                "recovery wheel/accel bounds retained after handoff\n",
                su.coordinated_speed ? 1 : 0, su.catch_speed_lean_max_rad, su.catch_speed_filter_s,
                su.catch_speed_tol_m_s);
    out_.printf("V3 TUNE standhip=%d (SET standhip): supported CoM hip feedback; "
                "wheel=LQR knee=fixed hip_rate<=%.2f rad/s hip_travel<=%.2f rad "
                "standhipgain=%.2f (SET standhipgain) legacy_damping=%.2f (standcp=0 only)\n",
                su.support_hip_capture ? 1 : 0,
                su.catch_hip_rate_max_rad_s, su.catch_hip_travel_rad,
                su.catch_hip_gain, su.catch_hip_damping_s);
    out_.printf("V3 TUNE standlevel=%d (SET standlevel): arm hip_progress>=0.25 rad clearance>=8 mm; "
                "B4 anti-dive ramp=80 ms, Support signed leveling ramp=250 ms; "
                "kp=%.2f kd=%.2f gyro_filter=40 ms body_rate<=%.2f rad/s; knee fixed, hip/floor limits retained\n",
                su.catch_body_leveling ? 1 : 0, su.recovery_body_gain,
                su.recovery_body_damping_s, su.recovery_body_rate_max_rad_s);
    out_.printf("V3 TUNE standcp=%d (SET standcp): when 1 hip_error=lean-lean_ref+filtered_rate/omega; "
                "rate_filter=%.0f ms; 0 restores angle/damping; wheel and hard safety rates=raw\n",
                su.catch_capture_point_hip ? 1 : 0, su.catch_capture_rate_filter_s * 1000.0F);
    out_.printf("V3 TUNE standpred=%d (SET standpred): B4 wheel-only delay=%.0f ms actuator_lag=%.0f ms "
                "observer=40 ms ramp=60 ms correction<=0.12 rad/1.0 rad/s; hip/raw safety unchanged; "
                "delay/lag approximate log fits, not contact detection\n",
                su.catch_wheel_prediction ? 1 : 0, su.catch_wheel_delay_s * 1000.0F,
                su.catch_wheel_lag_s * 1000.0F);
    out_.printf("V3 TUNE standservo=%d standbrake=%d (SET standservo/standbrake): brake=1 uses measured angular "
                "preview + persistent acceleration integral + lead>=%.0f ms; brake=0 uses prior queued-model forecast speed servo; "
                "equilibrium learn after quiet 250 ms tau=1 s rate<=0.02 rad/s max=0.06 rad; raw safety retained\n",
                su.catch_capture_velocity ? 1 : 0, su.catch_measured_braking ? 1 : 0,
                su.catch_wheel_lead_s * 1000.0F);
    out_.printf("V3 TUNE standtransfer=%d (SET standtransfer): requires standcapture/standhip/standlevel/standfloor/guard; "
                "capture dwell=%.0f ms ramp=%.0f ms rate<=%.2f rad/s clearance>=%.0f mm reserve>=%.0f mm; "
                "signed posture after capture, rolling handoff<=%.2f m/s; existing body/hip/tracking/dwell guards retained\n",
                su.catch_rolling_transfer ? 1 : 0, su.transfer_confirm_s * 1000.0F,
                su.transfer_ramp_s * 1000.0F, su.transfer_capture_rate_rad_s,
                su.transfer_clearance_m * 1000.0F, su.transfer_floor_reserve_m * 1000.0F,
                su.rolling_handoff_speed_m_s > 0.0F
                    ? std::min(su.rolling_handoff_speed_m_s,
                          su.catch_wheel_max_rad_s * ctrl_.config().wheel_radius_m + su.catch_tracking_margin_m_s)
                    : su.catch_wheel_max_rad_s * ctrl_.config().wheel_radius_m + su.catch_tracking_margin_m_s);
    out_.printf("V3 TUNE standbackdeg=%.1f (SET standbackdeg, 0 symmetric limits) "
                "catch_forward<=%.1f deg; recovery backward body only, floor and COM limits retained\n",
                su.back_pitch_limit_rad * 57.2957795F, su.pitch_limit_rad * 57.2957795F);
    out_.printf("V3 TUNE standhandoff=%d (SET standhandoff): captured rolling dwell=%.0f ms; "
                "software wheel cap alone does not veto handoff; measured tracking/body/hip/COM/floor gates retained\n",
                su.catch_rolling_handoff ? 1 : 0, su.rolling_handoff_settle_s * 1000.0F);
    out_.printf("V3 TUNE standwheelref=%d (SET standwheelref): B4 wheel lean target = lean_ref (speed %.2f /s, |ref|<=%.2f rad); "
                "standhospeed=%.2f m/s (SET standhospeed, 0 = cap+margin)\n",
                su.catch_wheel_speed_lean ? 1 : 0, su.catch_speed_gain, su.catch_speed_lean_max_rad,
                su.rolling_handoff_speed_m_s);
    out_.printf("V3 TUNE standarmhip=%.3f rad (SET standarmhip) standarmclr=%.1f mm (SET standarmclr): "
                "B4 posture/leveling arming (level_weight in SC rises once armed)\n",
                su.body_level_arm_hip_rad, su.body_level_arm_clearance_m * 1000.0F);
    out_.printf("V3 TUNE recbal=%d (SET recbal): auto REC 8 s at 250 Hz on entering Balancing (wL wR wcmd pitch prate)\n",
                rec_balance_auto_ ? 1 : 0);
    out_.printf("V3 TUNE trimtau=%.2f s (SET trimtau): ungated balance-angle observer after a stand-up handoff (0 = gated)\n",
                su.balance_trim_ungated_tau_s);
    out_.printf("V3 TUNE trimwin=%.3f rad (SET trimwin): LQR trim window around its seed after a stand-up handoff\n",
                su.balance_trim_window_rad);
    out_.printf("V3 TUNE standspdi=%.2f 1/s^2 (SET standspdi) standspdimax=%.3f rad (SET standspdimax): "
                "B4 ground-speed integral on the stopping lean target (speed_i in SU)\n",
                su.catch_speed_integral_gain, su.catch_speed_integral_max_rad);
    out_.printf("V3 TUNE standfloorsup=%d (SET standfloorsup): floor guard after the LQR handoff (Support)\n",
                su.floor_guard_in_support ? 1 : 0);
    out_.printf("V3 TUNE standbodygain=%.2f (SET standbodygain) standbodymax=%.2f rad/s (SET standbodymax): "
                "B4 posture term hip_rate=clamp(gain*pitch+damping*gyro_f, +-max) once armed\n",
                su.recovery_body_gain, su.recovery_body_rate_max_rad_s);
    out_.printf("V3 TUNE standpostguard=%d (SET standpostguard): B4 transfer posture projection, "
                "COM/CP same side, lean fade=0.015..0.030 rad; initial lift and Support unchanged\n",
                su.catch_posture_capture_guard ? 1 : 0);
    out_.printf("V3 TUNE standwheel=%.1f rad/s (SET standwheel): recovery wheel command cap; "
                "integrator_speed<=%.2f m/s accel<=%.2f m/s2 (SET standaccel); "
                "standwheelms=%.0f (SET standwheelms, 0 inherits hip); hip_blend=%.0f ms; commands, not measured limits\n",
                su.catch_wheel_max_rad_s, su.catch_wheel_max_rad_s * ctrl_.config().wheel_radius_m,
                su.catch_accel_max, su.catch_wheel_blend_s * 1000.0F, su.catch_blend_s * 1000.0F);
    out_.printf("V3 TUNE standfloor=%d (SET standfloor) floormm=%.1f (SET floormm) "
                "arm>=%.1f mm/%.0f ms approach=%.0f ms lag=%.0f ms; "
                "floorgrace=%d (SET floorgrace); kinematic estimate, no pitch target, knee=fixed\n",
                su.floor_guard ? 1 : 0, su.floor_clearance_m * 1000.0F,
                su.floor_arm_clearance_m * 1000.0F, su.floor_arm_dwell_s * 1000.0F,
                su.floor_approach_time_s * 1000.0F, su.floor_prediction_lag_s * 1000.0F,
                su.floor_accel_grace ? 1 : 0);
  }
  {
    const auto& gd = ctrl_.config().guard;
    out_.printf("V3 TUNE guard: mode=%d (SET guard 0 off/1 log/2 act) cut=%.2f (guardcut) lag+=%.3f s (guardlag) "
                "raise<=%.1f mm (guardraise) hold@%.2f (guardhold) raise@%.2f (guardrisk) grow=%.2f (guardgrow) "
                "amp>=%.3f rad (guardamp)\n", gd.mode, gd.gain_cut, gd.lag_boost_s, gd.raise_max_m * 1000.0F,
                gd.hold_risk, gd.raise_risk, gd.detect.growth_ratio, gd.detect.amp_min_rad);
  }
  out_.printf("V3 TUNE lqr prediction: enabled=%d (SET lqrpredict) delay=%.3f s (lqrdelay) lag=%.3f s (lqrlag)\n",
              ctrl_.config().lqr_prediction.enabled ? 1 : 0,
              ctrl_.config().lqr_prediction.delay_s, ctrl_.config().lqr_prediction.lag_s);
  out_.printf("V3 TUNE legmode=%s leg_kp=%.1f leg_kd=%.2f trim_gain=%.2f trim_max=%.2f "
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
  // While balancing free with the virtual pitch, the controller hands over the
  // kinematic velocity of the IMU point (last tick) so a sustained acceleration
  // does not tilt the estimate (Tier::imu_kinematic); otherwise it is invalid
  // and the estimate is a pure IMU one.
  const auto kv = ctrl_.imuVelocity();
  const bool dynamic_recovery = ctrl_.state() == V3Controller::State::StandUp &&
      ctrl_.config().standup.gyro_dominant_imu;
  const auto out = estimator_.update(body_.accel_m_s2, body_.gyro_rad_s, dt_s,
                                     static_cast<float>(now_us) * 1.0e-6F, 0.0F,
                                     dynamic_recovery, quiet && !dynamic_recovery,
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
    out_.printf("BLE %s\n", link_up ? "connected" : "disconnected");
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
  // Hold where the jerk-limited height profile can stop, not where it is this
  // instant. This also keeps button release smooth during a fast transition.
  const float h = ctrl_.legPose().axle_below_hip_m;
  if (h <= 0.0F) return;
  height_request_m_ = ctrl_.heightStopTarget();
}

float AppV3::legTargetRad(std::size_t leg) const noexcept {
  const auto binding = tap_fw::board::MotorIds{}.leg_can[leg];
  const auto& pose = ctrl_.legPose();
  const bool knee = binding.joint_index == 1U || binding.joint_index == 4U;
  // Raw encoder frame (zero = the pose the motor supply was switched on in), like legJointRad().
  return (knee ? pose.knee_offset_rad - homeKnee() : pose.hip_offset_rad - homeHip()) * binding.driver_sign;
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
      st == V3Controller::State::Balancing || st == V3Controller::State::StandUp;

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
    out_.println("V3 serial lease expired (no command for 0.5 s): disabled");
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
      out_.printf("V3 START refused: %s\n", refuse);
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
    out_.println("V3 GO: TILTTEST -> armed Ready (balances after put down and let go)");
  }
  // TILTTEST skips the carry-to-floor wait (it is a hand-held check).
  const bool ready_now = ctrl_.state() == V3Controller::State::Ready;
  cmd.standup = standup_pending_;
  standup_pending_ = false;
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
  in.roll_rate_rad_s = imu_.roll_rate_rad_s;  // raw: the roll estimate learns its bias
  in.accel_y_m_s2 = body_.accel_m_s2[1];
  in.accel_x_m_s2 = body_.accel_m_s2[0];
  in.accel_z_m_s2 = body_.accel_m_s2[2];
  in.wheel_rad_s = {state_.joints.velocity_rad_s[2], state_.joints.velocity_rad_s[5]};
  in.wheel_pos_rad = {state_.joints.position_rad[2], state_.joints.position_rad[5]};
  in.leg_chain_rad = legChainRad() + homeHip() + homeKnee();
  // Measured joints per side, core frame (LegLock order LH LK RH RK).
  {
    const auto legs = tap_fw::board::MotorIds{}.leg_can;
    for (std::size_t side = 0; side < 2U; ++side) {
      const std::size_t hip = 2U * side, knee = hip + 1U;
      in.hip_rad[side] = legJointRad(hip) * legs[hip].driver_sign + homeHip();
      in.knee_rad[side] = legJointRad(knee) * legs[knee].driver_sign + homeKnee();
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
  const std::uint32_t wheel_age_limit_us = st == V3Controller::State::Balancing
      ? tap_fw::board::kWheelBalanceMaxAgeUs : tap_fw::board::kWheelHoldMaxAgeUs;
  in.sensors_ok = imuFresh(now_us) && feedback_.allWheelsFreshWithin(now_us, wheel_age_limit_us) && legsFresh(now_us) &&
      legAxisProblem(now_us, closed_loop_due) == nullptr;
  bool legs_ready = legs_.atZero(now_us);
  for (std::size_t leg = 0; leg < LegLock::kLegs; ++leg) {
    legs_ready = legs_ready && std::abs(legJointRad(leg) - legTargetRad(leg)) <= kLegReadyRad;
  }
  in.legs_ready = legs_ready;
  if (st == V3Controller::State::Locking && now_us - state_since_us_ > kLockTimeoutUs) {
    cmd.disable = true;
    out_.printf("V3 legs did not reach zero within 5 s: [%+.3f %+.3f %+.3f %+.3f] rad; "
                  "disabled\n", legJointRad(0), legJointRad(1), legJointRad(2), legJointRad(3));
  }
  if (!in.sensors_ok && active) {
    out_.printf("V3 sensors: imu=%d wheels=%d legs=%d axis=%s\n", imuFresh(now_us),
                  feedback_.allWheelsFresh(now_us), legsFresh(now_us),
                  legAxisProblem(now_us, closed_loop_due) ? legAxisProblem(now_us, closed_loop_due)
                                                          : "ok");
  }

  if (rec_active_ && rec_kind_ == RecKind::KneeFloor) {
    const bool lost = ctrl_.state() != V3Controller::State::Balancing || ctrl_.heldOuterOff() ||
        std::abs(ctrl_.virtualPitch() - ctrl_.balanceRef()) > kKneeFloorAbortRad;
    if (lost) {
      stopRecording(true);
    } else {
      rec_cmd_ = recExcitation(now_us);
    }
    ctrl_.setKneeExcitation(rec_active_ ? rec_cmd_ : 0.0F);
  }
  const std::uint64_t p0 = nowUs();
  const auto out = ctrl_.update(in, cmd);
  const std::uint64_t p1 = nowUs();
  prof(0, p1 - p0);
  prof_upd_sum_ += static_cast<std::uint32_t>(p1 - p0);
  ++prof_upd_n_;
  if (out.fault != nullptr) {
    out_.printf("V3 FAULT: %s (wheels stopped, legs %s; START retries, DISABLE releases)\n",
                  out.fault, out.legs_locked ? "still locked" : "free");
  }

  if (out.engage_legs) {
    std::array<float, LegLock::kLegs> start{};
    for (std::size_t leg = 0; leg < LegLock::kLegs; ++leg) {
      start[leg] = feedback_.legEstimate(tap_fw::board::MotorIds{}.leg_can[leg].joint_index).pos_turns;
    }
    engage_us_ = now_us;
    const bool standing_up_engage = out.state == V3Controller::State::StandUp;
    if (standing_up_engage) {
      legs_.setOffsets(out.leg_hip_offset_rad - homeHip(), out.leg_knee_offset_rad - homeKnee());
      legs_.setSideOffsets({0.0F, 0.0F}, {0.0F, 0.0F});
    } else {
      legs_.setOffsets(ctrl_.legPose().hip_offset_rad - homeHip(), ctrl_.legPose().knee_offset_rad - homeKnee());
    }
    if (!legs_.engage(start, now_us, transport_, !standing_up_engage)) {
      // A rejected setup/limits frame must not leave the stand-up wheels
      // running with legs that never entered the requested closed loop.
      V3Controller::Command cancel{};
      cancel.disable = true;
      (void)ctrl_.update(in, cancel);
      (void)legs_.release(transport_);
      driveWheels(false, {}, now_us);
      out_.println("V3 leg engage failed: CAN setup not queued; disabled, START retries");
      return;
    }
    out_.printf("V3 leg enable: Set_Limits queued current=%.1f A velocity=%.1f rotor turns/s (not read back)\n",
                tap_fw::board::kLegCurrentLimitA, tap_fw::board::kLegVelLimitTurnsS);
    out_.printf("V3 legs locking (%s): from [%+.3f %+.3f %+.3f %+.3f] rad to 0\n",
                  tap_fw::v3::toString(legs_.config().mode), legJointRad(0), legJointRad(1),
                  legJointRad(2), legJointRad(3));
  }
  if (rec_active_ &&
      ((recStand() && !out.legs_locked) ||
       now_us - rec_start_us_ >= static_cast<std::uint64_t>(rec_duration_s_ * 1.0e6F))) {
    stopRecording(recStand() && !out.legs_locked);
  }
  // Wheels catch the unstable pendulum: transmit their freshly computed
  // command before building/packing the four leg targets each control tick.
  const std::uint64_t p_wheel = nowUs();
  auto wheel_cmd = out.wheel_cmd_rad_s;
  if (tilt_test_ && out.state == V3Controller::State::Balancing) {
    if (last_state_ != static_cast<int>(out.state)) tilt_ref_rad_ = imu_.pitch_rad;
    const float w = std::clamp(kTiltTestGain * (imu_.pitch_rad - tilt_ref_rad_),
                               -kTiltTestMaxRadS, kTiltTestMaxRadS);
    wheel_cmd = {w, w};
  }
  const bool stand_test = (rec_active_ || rec_dumping_) && recStand();
  driveWheels(out.wheels_enabled && !stand_test, wheel_cmd, now_us);
  const std::uint64_t wheel_send_us = nowUs() - p_wheel;
  const std::uint64_t p2 = nowUs();
  float hip_cmd = 0.0F, knee_cmd = 0.0F;
  if (out.legs_locked) {
    const bool stand = rec_active_ && recStand();
    if (stand) rec_cmd_ = recExcitation(now_us);
    // The height pose plus the tier-2 hip offset (0 unless trunk is on and
    // balancing); the stand tests drive the bare pose.
    // The stand-up output owns the rocking path; the other states use the balance pose.
    const bool standing_up = !stand && out.state == V3Controller::State::StandUp;
    hip_cmd = stand ? ctrl_.legPose().hip_offset_rad : standing_up ? out.leg_hip_offset_rad : ctrl_.hipCommand();
    knee_cmd = stand ? ctrl_.legPose().knee_offset_rad : standing_up ? out.leg_knee_offset_rad : ctrl_.kneeCommand();
    if (stand && rec_kind_ == RecKind::HipStand) hip_cmd += rec_cmd_;
    if (stand && rec_kind_ == RecKind::KneeStand) {
      knee_cmd = std::clamp(knee_cmd + rec_cmd_, tap_fw::v3::LegHeight::kKneeMinRad, 0.0F);
    }
    legs_.setOffsets(hip_cmd - homeHip(), knee_cmd - homeKnee());
    // Continue the recovery ramp feed-forward while its CoM hip allocator
    // remains active after handoff. Knee stays fixed; Fault/held clears rates.
    const bool supported_hip = !stand && out.state == V3Controller::State::Balancing &&
        ctrl_.recoveryPoseHeld() && ctrl_.config().standup.support_hip_capture &&
        !ctrl_.heldOuterOff() && !ctrl_.heldQuiet();
    const bool recovery_path = standing_up || supported_hip;
    if (recovery_path && ff_prev_us_ != 0U && now_us > ff_prev_us_) {
      const float dt_s = static_cast<float>(now_us - ff_prev_us_) * 1.0e-6F;
      const float alpha = dt_s / (0.02F + dt_s);  // 20 ms low-pass
      ff_hip_rate_ += alpha * (std::clamp((hip_cmd - ff_hip_prev_) / dt_s, -6.0F, 6.0F) - ff_hip_rate_);
      ff_knee_rate_ += alpha * (std::clamp((knee_cmd - ff_knee_prev_) / dt_s, -6.0F, 6.0F) - ff_knee_rate_);
    } else if (!recovery_path) {
      ff_hip_rate_ = 0.0F;
      ff_knee_rate_ = 0.0F;
    }
    ff_hip_prev_ = hip_cmd;
    ff_knee_prev_ = knee_cmd;
    ff_prev_us_ = now_us;
    legs_.setCommonRates(ff_hip_rate_, ff_knee_rate_);
    // Roll: per-side offsets (zero unless SET roll 1 and balancing).
    if (stand) legs_.setSideOffsets({0.0F, 0.0F}, {0.0F, 0.0F});
    else legs_.setSideOffsets(out.leg_roll_hip_rad, out.leg_roll_knee_rad,
                              out.leg_roll_hip_rate, out.leg_roll_knee_rate);
    (void)legs_.update(now_us, transport_);
  } else if (legs_.engaged()) {
    (void)legs_.release(transport_);
  }
  const std::uint64_t p3 = nowUs();
  prof(1, p3 - p2);
  if (rec_active_ && now_us >= rec_next_us_) {
    rec_next_us_ += kRecPeriodUs;
    recordSample(now_us, hip_cmd, knee_cmd, 0.5F * (last_wheel_cmd_[0] + last_wheel_cmd_[1]));
  }
  if (rec_dumping_) dumpRecording();
  (void)feedback_.poll(transport_, now_us, true);  // wheel reads + bus voltage, no MIT probes
  const std::uint64_t p4 = nowUs();
  prof(2, p4 - p3 + wheel_send_us);

  const int state = static_cast<int>(out.state);
  const bool recovery_handoff = last_state_ == static_cast<int>(V3Controller::State::StandUp) &&
      out.state == V3Controller::State::Balancing;
  if (state != last_state_) {
    last_state_ = state;
    state_since_us_ = now_us;
    next_trace_us_ = now_us;
    out_.printf("CONTROL mode=%s enable=%d result=0\n", tap_fw::v3::toString(out.state),
                  out.legs_locked ? 1 : 0);
    // Recovery has the same dynamic specific-force problem as floor balance.
    // Previously it kept the 55/s accelerometer correction throughout Catching
    // (18/10 log: pitch drifted +0.44 rad while gyro integrated only +0.08).
    // Switch config without resetting the seeded angle or gyro bias. Keep
    // this config across the recovery->balance handoff; Idle/Fault restore
    // the base. TILTTEST remains hand-held and uses the base.
    const bool floor_balance = out.state == V3Controller::State::Balancing && !tilt_test_;
    const bool recovery_gyro = out.state == V3Controller::State::StandUp &&
        ctrl_.config().standup.gyro_dominant_imu;
    estimator_.setConfig((floor_balance || recovery_gyro)
        ? tap_fw::v3::balancingEstimatorConfig(robot_control::ImuStateEstimator::Config{})
        : robot_control::ImuStateEstimator::Config{});
    if (out.state == V3Controller::State::StandUp) {
      out_.printf("V3 STANDUP estimator=%s\n", recovery_gyro ? "gyro-dominant" : "base");
    }
    if (out.state == V3Controller::State::Ready) {
      out_.println(tilt_test_ ? "V3 READY (TILTTEST: engaging at once)"
          : ctrl_.config().engage.automatic
          ? "V3 READY: wheels holding. Carry the robot to the floor, hold it still at its best "
            "standing point (0.7 s) until ARMED, let go: balancing starts when it tips"
          : "V3 READY: wheels holding. Auto engage is off");
    }
    if (out.state == V3Controller::State::Balancing) {
      out_.printf("RELEASE pitch=%+.4f seeded_trim=%+.4f estimator=%s\n", imu_.pitch_rad,
                    ctrl_.balanceRef(), floor_balance ? "gyro-dominant" : "base");
      if (rec_balance_auto_ && !rec_active_ && !rec_dumping_) {
        startRecording(RecKind::Free, 0.0F, 8.0F, now_us);
        out_.println("V3 REC auto-start at Balancing: 8 s at 250 Hz (SET recbal 0 disables)");
      }
    }
  }
  digitalWrite(tap_fw::board::kLed2Gpio, out.legs_locked ? HIGH : LOW);
  digitalWrite(tap_fw::board::kLed3Gpio, out.wheels_enabled ? HIGH : LOW);

  // The recorder dump has the link while it runs (TR resumes after RC END).
  // Ready (not immediate): tell the operator where the auto engage stands.
  {
    const int phase = out.state == V3Controller::State::Ready ? ctrl_.readyPhase() : -1;
    if (phase != last_ready_phase_) {
      if (phase == 0) out_.println("V3 READY: wheels held still - put it on the floor, hold it still at its standing angle");
      if (phase == 1) out_.println("V3 READY: moved - now hold it still on the floor (0.7 s)");
      if (phase == 2) out_.println("V3 ARMED: balancing starts in 0.5 s or as soon as it tips - let go");
      last_ready_phase_ = phase;
    }
  }
  const std::uint64_t p5 = nowUs();
  if (out.state != V3Controller::State::Idle && now_us >= next_trace_us_ && !rec_dumping_) {
    next_trace_us_ = now_us + kTraceUs;
    // Copy typed fields here; OutSink formats on the communications task.
    // Controller state is never read from that task. Text stays compatible.
    static char line[1536];
    tap_fw::DeferredLine b(line, sizeof line);
    b.str("TR t=").u32(static_cast<std::uint32_t>((now_us - state_since_us_) / 1000U))
        .str(" mode=").str(tap_fw::v3::toString(out.state))
        .str(" pitch=").fix(imu_.pitch_rad, 3, true).str(" rate=").fix(imu_.pitch_rate_rad_s, 2, true)
        .str(" wcmd=[").fix(last_wheel_cmd_[0], 2, true).chr(' ').fix(last_wheel_cmd_[1], 2, true)
        .str("] wmeas=[").fix(in.wheel_rad_s[0], 2, true).chr(' ').fix(in.wheel_rad_s[1], 2, true)
        .str("] axle_v=").fix(ctrl_.groundVelocity(), 3, true).str(" axle_x=").fix(ctrl_.groundPosition(), 3, true)
        .str(" wheel_lx=").fix(state_.wheel.left_position_m, 3, true)
        .str(" wheel_rx=").fix(state_.wheel.right_position_m, 3, true)
        .str(" trim=").fix(ctrl_.balanceRef(), 4, true)
        .str(" legs=[").fix(legJointRad(0), 3, true).chr(' ').fix(legJointRad(1), 3, true).chr(' ')
        .fix(legJointRad(2), 3, true).chr(' ').fix(legJointRad(3), 3, true)
        .str("] ref=").fix(ctrl_.pitchRef(), 4, true).str(" ex=").fix(ctrl_.positionError(), 3, true)
        .str(" ready=").i32(ctrl_.readyPhase()).str(" held=").i32(static_cast<std::int32_t>(ctrl_.heldResets()))
        .str(" h=").fix(ctrl_.legPose().axle_below_hip_m * 1000.0F, 1)
        .str(" htgt=").fix(ctrl_.heightTarget() * 1000.0F, 1)
        .str(" wa=").fix(static_cast<float>(feedback_.oldestWheelAgeUs(now_us)) * 1.0e-3F, 1)
        .str(" gr=").fix(ctrl_.guardRisk(), 2).str(" ga=").fix(ctrl_.guardAmplitude(), 3)
        .str(" gz=").fix(ctrl_.guardRaise() * 1000.0F, 1)
        .str(" knee=").fix(ctrl_.legPose().knee_offset_rad, 3, true)
        .str(" tv=").fix(ctrl_.virtualPitch(), 4, true)
        .str(" tvr=").fix(ctrl_.virtualPitchRate(), 3, true).str(" dhip=").fix(ctrl_.trunkHip(), 4, true)
        .str(" ta=").fix(ctrl_.trunkAuthority(), 2).str(" me=").fix(ctrl_.legModelError(), 3, true)
        .str(" lvl=").fix(ctrl_.levelHip(), 4, true).str(" ho=").i32(ctrl_.heldOuterOff() ? 1 : 0)
        .str(" lt=").fix(ctrl_.learnedTrim(), 4, true).chr('/').i32(ctrl_.learnedTrimValid() ? 1 : 0)
        .str(" dknee=").fix(ctrl_.kneeCommand() - ctrl_.legPose().knee_offset_rad, 4, true)
        .str(" swing=").fix(ctrl_.trunkSwingRequest(), 4, true)
        .str(" axlim=").i32(ctrl_.trunkWorkspaceLimited() ? 1 : 0)
        .str(" wc=[").fix(ctrl_.wheelCarrierCorrection()[0], 3, true).chr(' ')
        .fix(ctrl_.wheelCarrierCorrection()[1], 3, true).chr(']')
        .str(" adpr=").fix(ctrl_.trunkSchedule().com_ratio, 3)
        .str(" adpl=").fix(ctrl_.trunkSchedule().lead_s, 3)
        .str(" adpv=").fix(ctrl_.trunkSchedule().rate_rad_s, 2)
        .str(" adpt=").fix(ctrl_.trunkSchedule().level_tau_s, 3)
        .str(" wbb=").fix(ctrl_.wbDebug().bias, 4, true)
        .str(" sus=").fix(ctrl_.suspensionOffset() * 1000.0F, 1, true)
        .str(" vz=").fix(ctrl_.bodyVerticalSpeed(), 3, true)
        .str(" roll=").fix(ctrl_.rollEstimate(), 4, true)
        .str(" rdh=").fix(ctrl_.rollSideHeight() * 1000.0F, 1, true)
        .str(" anc=").fix(ctrl_.lqrAnchorOffset() * 1000.0F, 0, true)
        .str(" sgv=").fix(ctrl_.standGive() * 1000.0F, 0, true)
        .str(" rg=").fix(ctrl_.reflexGate(), 2)
        .str(" ql=").fix(ctrl_.lqrLeadEffective(), 3)
        .str(" sdb=").fix(ctrl_.suspensionDeadband() * 1000.0F, 1)
        .str(" rref=").fix(ctrl_.rollReference(), 4, true)
        .str(" rm=").i32(ctrl_.rollMode())
        .str(" suhold=").i32(ctrl_.recoveryPoseHeld() ? 1 : 0);
    out_.println(b);
    {
      const auto left = uart1_.diagnostics();
      const auto right = uart2_.diagnostics();
      const auto wheels = tap_fw::board::MotorIds{}.wheel_uart;
      tap_fw::DeferredLine wb(line, sizeof line);
      wb.str("WT t=").u32(static_cast<std::uint32_t>(now_us / 1000U))
          .str(" enabled=").i32(wheels_on_ ? 1 : 0)
          .str(" sent=[").fix(wheels[0].driver_sign * frame::wheelRawSpeedToRadPerSec(left.sent_speed_raw), 2, true)
          .chr(' ').fix(wheels[1].driver_sign * frame::wheelRawSpeedToRadPerSec(right.sent_speed_raw), 2, true)
          .str("] age_us=[").u32(left.speed_tx_age_us).chr(' ').u32(right.speed_tx_age_us)
          .str("] tx_count=[").u32(left.speed_tx).chr(' ').u32(right.speed_tx).chr(']');
      out_.println(wb);
    }
    const auto& su = ctrl_.standupState();
    if (recovery_handoff || out.state == V3Controller::State::StandUp ||
        (out.state == V3Controller::State::Balancing && ctrl_.recoveryPoseHeld() &&
         ctrl_.config().standup.support_hip_capture) ||
        (out.state == V3Controller::State::Fault && su.phase != tap_fw::v3::StandUp::Phase::Idle)) {
      // Separate bounded line: keep TR parser compatibility and its buffer size.
      tap_fw::DeferredLine sb(line, sizeof line);
      sb.str("SU t=").u32(static_cast<std::uint32_t>(now_us / 1000U))
          .str(" state=").str(tap_fw::v3::toString(out.state))
          .str(" snapshot=").i32(out.state == V3Controller::State::Fault ? 1 : 0)
          .str(" phase=").str(tap_fw::v3::StandUp::name(su.phase))
          .str(" capture_first=").i32(ctrl_.config().standup.capture_first ? 1 : 0)
          .str(" hip_active=").i32(out.state == V3Controller::State::StandUp ||
              (out.state == V3Controller::State::Balancing && ctrl_.recoveryPoseHeld() &&
               ctrl_.config().standup.support_hip_capture &&
               !ctrl_.heldOuterOff() && !ctrl_.heldQuiet()) ? 1 : 0)
          .str(" wheel_owner=").str(out.state == V3Controller::State::Balancing ? "LQR" :
              out.state == V3Controller::State::StandUp ? "catch" : "stopped")
          .str(" lean_ref=").fix(su.capture_lean_target_rad, 3, true)
          .str(" wheel_ref=").fix(su.wheel_lean_ref_rad, 3, true)
          .str(" speed_i=").fix(su.speed_integral_rad, 3, true)
          .str(" speed_f=").fix(su.speed_filtered_m_s, 3, true)
          .str(" knee_tgt=").fix(su.knee_target_rad, 3)
          .str(" elapsed=").fix(su.phase_elapsed_s, 3)
          .str(" hip_tgt=").fix(su.hip_target_rad, 3, true)
          .str(" hip_meas=").fix(su.hip_measured_rad, 3, true)
          .str(" hip_rate=").fix(su.hip_rate_rad_s, 3, true)
          .str(" lean=").fix(su.capture_lean_rad, 3, true)
          .str(" com_rate=").fix(su.capture_rate_rad_s, 3, true)
          .str(" cp=").fix(su.capture_point_rad, 3, true)
          .str(" clearance_mm=").fix(su.body_clearance_m * 1000.0F, 1, true)
          .str(" clearance_valid=").i32(su.body_clearance_valid ? 1 : 0)
          .str(" wheel_error=").fix(su.wheel_tracking_error_rad_s, 2)
          .str(" wheel_accel=").fix(su.wheel_accel_m_s2, 3, true)
          .str(" speed_accel=").fix(su.speed_feedback_accel_m_s2, 3, true)
          .str(" hip_limited=").i32(su.hip_limited ? 1 : 0)
          .str(" wheel_limited=").i32(su.wheel_limited ? 1 : 0)
          .str(" ready=").i32(su.handoff_ready ? 1 : 0)
          .str(" rolling_gate=").i32(su.rolling_handoff ? 1 : 0)
          .str(" settle=").fix(su.settle_s, 3)
          .str(" settle_need=").fix(su.handoff_settle_required_s, 3)
          .str(" reason=").str(tap_fw::v3::StandUp::failureName(su.failure))
          .str(" controller_fault=").str(out.state == V3Controller::State::Fault ? ctrl_.lastFault() : "none");
      out_.println(sb);
      {
        // Capture feedback telemetry has its own bounded buffer line. SU and
        // SF retain their existing fields, including raw-rate safety evidence.
        tap_fw::DeferredLine cb(line, sizeof line);
        cb.str("SC t=").u32(static_cast<std::uint32_t>(now_us / 1000U))
            .str(" state=").str(tap_fw::v3::toString(out.state))
            .str(" snapshot=").i32(out.state == V3Controller::State::Fault ? 1 : 0)
            .str(" phase=").str(tap_fw::v3::StandUp::name(su.phase))
            .str(" standcp=").i32(ctrl_.config().standup.catch_capture_point_hip ? 1 : 0)
            .str(" hip_rate_signal=").fix(su.hip_capture_rate_rad_s, 3, true)
            .str(" hip_error=").fix(su.hip_capture_error_rad, 3, true)
            .str(" feedback_rate=").fix(su.hip_feedback_rate_rad_s, 3, true)
            .str(" post_guard=").i32(su.posture_capture_limited ? 1 : 0)
            .str(" standlevel=").i32(ctrl_.config().standup.catch_body_leveling ? 1 : 0)
            .str(" level_weight=").fix(su.body_level_weight, 2)
            .str(" body_rate_f=").fix(su.body_rate_filtered_rad_s, 3, true)
            .str(" level_rate=").fix(su.body_level_rate_rad_s, 3, true)
            .str(" captured=").i32(su.transfer_captured ? 1 : 0)
            .str(" transfer=").fix(su.transfer_weight, 2)
            .str(" capture_dwell=").fix(su.transfer_confirm_s, 3)
            .str(" applied_rate=").fix(su.hip_rate_rad_s, 3, true);
        out_.println(cb);
      }
      if (ctrl_.config().standup.catch_wheel_prediction) {
        tap_fw::DeferredLine wb(line, sizeof line);
        wb.str("SW t=").u32(static_cast<std::uint32_t>(now_us / 1000U))
            .str(" state=").str(tap_fw::v3::toString(out.state))
            .str(" snapshot=").i32(out.state == V3Controller::State::Fault ? 1 : 0)
            .str(" phase=").str(tap_fw::v3::StandUp::name(su.phase))
            .str(" valid=").i32(su.wheel_prediction_valid ? 1 : 0)
            .str(" source=").str(su.wheel_measured_preview ? "measured" : "queued")
            .str(" weight=").fix(su.wheel_prediction_weight, 2)
            .str(" capped=").i32(su.wheel_prediction_capped ? 1 : 0)
            .str(" lean_pred=").fix(su.wheel_predicted_lean_rad, 3, true)
            .str(" rate_pred=").fix(su.wheel_predicted_rate_rad_s, 3, true)
            .str(" speed_pred=").fix(su.wheel_predicted_speed_m_s, 3, true)
            .str(" accel_obs=").fix(su.wheel_observed_accel_m_s2, 3, true)
            .str(" angular_accel_obs=").fix(su.wheel_observed_angular_accel_rad_s2, 2, true)
            .str(" accel_raw=").fix(su.wheel_raw_accel_m_s2, 3, true)
            .str(" accel_used=").fix(su.wheel_accel_m_s2, 3, true);
        out_.println(wb);
      }
      if (ctrl_.config().standup.catch_capture_velocity) {
        tap_fw::DeferredLine vb(line, sizeof line);
        vb.str("SV t=").u32(static_cast<std::uint32_t>(now_us / 1000U))
            .str(" state=").str(tap_fw::v3::toString(out.state))
            .str(" snapshot=").i32(out.state == V3Controller::State::Fault ? 1 : 0)
            .str(" phase=").str(tap_fw::v3::StandUp::name(su.phase))
            .str(" weight=").fix(su.capture_servo_weight, 2)
            .str(" standbrake=").i32(ctrl_.config().standup.catch_measured_braking ? 1 : 0)
            .str(" speed_target=").fix(su.capture_servo_speed_m_s, 3, true)
            .str(" speed_integral=").fix(su.capture_integrated_speed_m_s, 3, true)
            .str(" equilibrium=").fix(su.capture_equilibrium_rad, 4, true)
            .str(" learning=").i32(su.capture_equilibrium_learning ? 1 : 0);
        out_.println(vb);
      }
      if (ctrl_.config().standup.floor_guard) {
        // Separate line to avoid truncating the existing SU buffer/fields.
        tap_fw::DeferredLine fb(line, sizeof line);
        fb.str("SF t=").u32(static_cast<std::uint32_t>(now_us / 1000U))
            .str(" state=").str(tap_fw::v3::toString(out.state))
            .str(" snapshot=").i32(out.state == V3Controller::State::Fault ? 1 : 0)
            .str(" valid=").i32(su.floor_bounds_valid ? 1 : 0)
            .str(" armed=").i32(su.floor_armed ? 1 : 0)
            .str(" limited=").i32(su.floor_limited ? 1 : 0)
            .str(" infeasible=").i32(su.floor_infeasible ? 1 : 0)
            .str(" accel_sat=").i32(su.floor_accel_saturated ? 1 : 0)
            .str(" clearance_mm=").fix(su.body_clearance_m * 1000.0F, 2, true)
            .str(" closing_mm_s=").fix(su.floor_clearance_rate_m_s * 1000.0F, 1, true)
            .str(" predicted_mm=").fix(su.floor_predicted_clearance_m * 1000.0F, 2, true)
            .str(" hip_rate_min=").fix(su.floor_hip_rate_lower_rad_s, 3, true)
            .str(" hip_rate_max=").fix(su.floor_hip_rate_upper_rad_s, 3, true);
        out_.println(fb);
      }
    }
    prof(3, nowUs() - p5);
  }
}

void AppV3::startRecording(RecKind kind, float amp, float duration_s,
                           std::uint64_t now_us) noexcept {
  rec_kind_ = kind;
  rec_amp_ = amp;
  rec_duration_s_ = duration_s;
  rec_start_us_ = now_us;
  rec_next_us_ = now_us;
  rec_n_ = 0U;
  rec_cmd_ = 0.0F;
  rec_aborted_ = false;
  // Knee swings stay inside [kKneeMinRad, 0]: at the hang pose (knee 0) the
  // knee can only fold, so the swing is centred amp folded.
  rec_centre_ = 0.0F;
  if (kind == RecKind::KneeStand || kind == RecKind::KneeFloor) {
    const float knee = ctrl_.legPose().knee_offset_rad;
    if (knee + amp > 0.0F) rec_centre_ = -amp;
    if (knee - amp < tap_fw::v3::LegHeight::kKneeMinRad) rec_centre_ = amp;
  }
  rec_active_ = true;
}

void AppV3::stopRecording(bool aborted) noexcept {
  rec_active_ = false;
  rec_aborted_ = aborted;
  rec_dumping_ = true;
  rec_dump_i_ = 0U;
  rec_cmd_ = 0.0F;
  ctrl_.setKneeExcitation(0.0F);
  if (recStand()) legs_.setResendPeriod(LegLock::Config{}.resend_period_us);
  out_.printf("RC BEGIN kind=%s n=%u rate_hz=250 amp=%.3f centre=%+.3f resend_us=%lu "
                "aborted=%d cols=%s scales=", recName(static_cast<int>(rec_kind_)),
                static_cast<unsigned>(rec_n_), rec_amp_, rec_centre_,
                static_cast<unsigned long>(rec_resend_us_), aborted ? 1 : 0, kRecColumns);
  for (std::size_t i = 0; i < kRecChannels; ++i) {
    out_.printf(i + 1U < kRecChannels ? "%g," : "%g\n", kRecScale[i]);
  }
}

float AppV3::recExcitation(std::uint64_t now_us) const noexcept {
  const float t = static_cast<float>(now_us - rec_start_us_) * 1.0e-6F;
  switch (rec_kind_) {
    case RecKind::HipStand: return tap_fw::v3::standTestOffset(t, rec_amp_);
    case RecKind::KneeStand: return rec_centre_ + tap_fw::v3::standTestOffset(t, rec_amp_);
    case RecKind::KneeFloor: return tap_fw::v3::floorTestOffset(t, rec_amp_, rec_centre_);
    default: return 0.0F;
  }
}

void AppV3::recordSample(std::uint64_t now_us, float hip_cmd, float knee_cmd,
                         float wheel_cmd) noexcept {
  if (rec_n_ >= kRecMaxSamples) return;
  const auto legs = tap_fw::board::MotorIds{}.leg_can;
  const std::uint64_t stamp = feedback_.legEstimate(legs[0].joint_index).stamp_us;
  const float values[kRecChannels] = {
      rec_cmd_,
      legJointRad(0) * legs[0].driver_sign, legJointRad(2) * legs[2].driver_sign,
      legJointRad(1) * legs[1].driver_sign, legJointRad(3) * legs[3].driver_sign,
      hip_cmd, knee_cmd,
      imu_.pitch_rad, imu_.pitch_rate_rad_s, imu_.roll_rate_rad_s, imu_.yaw_rate_rad_s,
      ctrl_.virtualPitch(),
      body_.accel_m_s2[0], body_.accel_m_s2[1], body_.accel_m_s2[2],
      state_.joints.velocity_rad_s[2], state_.joints.velocity_rad_s[5], wheel_cmd,
      static_cast<float>(now_us > stamp ? now_us - stamp : 0U) * 1.0e-3F};
  auto& sample = g_rec[rec_n_++];
  sample.t_us = static_cast<std::uint32_t>(now_us - rec_start_us_);
  for (std::size_t i = 0; i < kRecChannels; ++i) sample.v[i] = recQuant(values[i], kRecScale[i]);
}

void AppV3::dumpRecording() noexcept {
  // A line per tick, only while the TX buffer is mostly empty: the TR/SENS
  // printf calls block when it is full, which stalled the loop and let the
  // wheel feedback go stale (fault) during the first HIPTEST dump. Digits by
  // hand: snprintf for 19 fields cost ~0.6 ms a line and two lines a tick
  // ran the loop at ~3.4 ms during a dump (2026-10-01) - the floor tests
  // dump while balancing.
  if (out_.availableForWrite() <= 2048) return;
  if (rec_dump_i_ >= rec_n_) {
    out_.println("RC END");
    rec_dumping_ = false;
    // Stand tests held the wheels off: engage afresh.
    if (recStand()) ctrl_.returnToReady();
    rec_kind_ = RecKind::None;
    return;
  }
  const auto& sample = g_rec[rec_dump_i_++];
  char text[160];
  std::size_t n = 0;
  auto put_uint = [&](std::uint32_t v) {
    char digits[10];
    std::size_t k = 0;
    do {
      digits[k++] = static_cast<char>('0' + v % 10U);
      v /= 10U;
    } while (v != 0U && k < sizeof(digits));
    while (k > 0U) text[n++] = digits[--k];
  };
  text[n++] = 'R';
  text[n++] = 'C';
  text[n++] = ' ';
  // Time in ms with one decimal, as before.
  put_uint(sample.t_us / 1000U);
  text[n++] = '.';
  text[n++] = static_cast<char>('0' + (sample.t_us % 1000U) / 100U);
  for (std::size_t i = 0; i < kRecChannels; ++i) {
    text[n++] = ' ';
    const std::int32_t v = sample.v[i];
    if (v < 0) text[n++] = '-';
    put_uint(static_cast<std::uint32_t>(v < 0 ? -v : v));
  }
  text[n] = '\0';
  out_.println(text);
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
  if (tunables_pending_ &&
      (ctrl_.state() == V3Controller::State::Idle || ctrl_.state() == V3Controller::State::Fault))
    printTunables();
  const long imu_age = last_imu_us_ == 0U ? -1L
      : static_cast<long>((now_us - last_imu_us_) / 1000U);
  // Same leading fields as the v1 SENS line, so the GUI status parser works
  // (FastFormat: same text as the former printf, without its float cost).
  static char line[1536];
  tap_fw::DeferredLine b(line, sizeof line);
  b.str("SENS imu_age=").i32(static_cast<std::int32_t>(imu_age)).str("ms seed=").i32(imu_.seeded ? 1 : 0)
      .str(" can_legs=").i32(legsFresh(now_us) ? 1 : 0).str(" uart_wheels=").i32(feedback_.allWheelsFresh(now_us) ? 1 : 0)
      .str(" homed=").i32(legs_.engaged() ? 1 : 0)
      .str(" joint_q=[").fix(legJointRad(0), 3).chr(' ').fix(legJointRad(1), 3).chr(' ')
      .fix(state_.joints.position_rad[2], 3).chr(' ').fix(legJointRad(2), 3).chr(' ').fix(legJointRad(3), 3)
      .chr(' ').fix(state_.joints.position_rad[5], 3)
      .str("] pitch=").fix(imu_.pitch_rad, 3, true).str(" pitch_rate=").fix(imu_.raw_pitch_rate_rad_s, 3, true)
      .str(" yaw_rate=").fix(imu_.yaw_rate_rad_s, 3, true)
      .str(" acc=[").fix(body_.accel_m_s2[0], 2, true).chr(' ').fix(body_.accel_m_s2[1], 2, true).chr(' ')
      .fix(body_.accel_m_s2[2], 2, true)
      .str("] chip_acc=[").fix(chip_.accel_m_s2[0], 2, true).chr(' ').fix(chip_.accel_m_s2[1], 2, true).chr(' ')
      .fix(chip_.accel_m_s2[2], 2, true)
      .str("] gyro_bias=").fix(imu_.gyro_bias_rad_s, 4, true).str(" vbus_min=").fix(feedback_.takeMinBusVoltage(), 2)
      .str(" state=").str(tap_fw::v3::toString(ctrl_.state())).str(" axis=[");
  const std::size_t axes[4] = {0U, 1U, 3U, 4U};
  for (int a = 0; a < 4; ++a) {
    if (a) b.chr(' ');
    b.u32(static_cast<std::uint32_t>(feedback_.legAxis(axes[a]).state));
  }
  b.str("] axis_err=[");
  for (int a = 0; a < 4; ++a) {
    if (a) b.chr(' ');
    b.hex(static_cast<std::uint32_t>(feedback_.legAxis(axes[a]).error));
  }
  b.str("] pitch_avg=").fix(pitch_avg_, 4, true).str(" spread=").fix(pitch_spread_, 4)
      .str(" balance=").fix(ctrl_.balanceAngle(), 4, true).str(ctrl_.balanceAngleSet() ? "" : "(unset)")
      .str(" tick_max_us=").u32(tick_exec_max_us_).str(" imu_us=").u32(tick_imu_max_us_)
      .str(" ctrl_us=").u32(tick_ctrl_max_us_).str(" late=").u32(tick_late_)
      .str(" period_max_us=").u32(tick_period_max_us_).str(" overrun=").u32(tick_overrun_)
      .str(" report_us=").u32(tick_report_max_us_)
      .str(" serial_us=").u32(pre_serial_max_us_).str(" ble_us=").u32(pre_ble_max_us_)
      .str(" bus_us=").u32(pre_bus_max_us_)
      .str(" hipv_rms=").fix(hip_vel_n_ > 0U ? std::sqrt(hip_vel_sq_ / static_cast<float>(hip_vel_n_)) : 0.0F, 3)
      .str(" prof=[").u32(prof_max_[0]).chr(' ').u32(prof_max_[1]).chr(' ').u32(prof_max_[2]).chr(' ')
      .u32(prof_max_[3]).str("] upd_avg=").u32(prof_upd_n_ ? prof_upd_sum_ / prof_upd_n_ : 0U)
      .str(" rtos=").i32(rtos_ ? 1 : 0).str(" outdrop=").u32(out_.dropped()).str(" cmddrop=").u32(lines_dropped_);
  out_.println(b);
  // Bounded transport evidence, separate from requested wcmd and controller
  // snapshots. No motor requests or parameter writes are issued here.
  tap_fw::DeferredLine left(line, sizeof line);
  uart1_.format(left);
  out_.println(left);
  tap_fw::DeferredLine right(line, sizeof line);
  uart2_.format(right);
  out_.println(right);
  prof_max_.fill(0U);
  prof_upd_sum_ = 0U;
  prof_upd_n_ = 0U;
  hip_vel_sq_ = 0.0F;
  hip_vel_n_ = 0U;
  pre_serial_max_us_ = 0U;
  pre_ble_max_us_ = 0U;
  pre_bus_max_us_ = 0U;
  tick_exec_max_us_ = 0U;
  tick_imu_max_us_ = 0U;
  tick_ctrl_max_us_ = 0U;
  tick_late_ = 0U;
  tick_period_max_us_ = tick_overrun_ = tick_report_max_us_ = 0U;
}

}  // namespace tap_hw
