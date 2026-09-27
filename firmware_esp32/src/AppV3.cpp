#include "tap_hw/AppV3.hpp"

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
constexpr float kPitchAvgTauS = 0.5F;       // balance-angle measurement average
constexpr float kTiltTestGain = 80.0F;      // rad/s wheel per rad pitch (0.1 rad -> 8 rad/s)
constexpr float kTiltTestMaxRadS = 12.0F;   // below the 15 rad/s runaway guard
// Legs count as "at zero" once within this of the target after the ramp.
constexpr float kLegReadyRad = 0.08F;
constexpr std::array<const char*, 4> kLegNames{"LH", "LK", "RH", "RK"};

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
  Serial.println("BLE off: control over USB serial (tools/tap_serial_control_gui.py)");
  printTunables();
  const std::uint64_t now_us = nowUs();
  next_control_us_ = now_us;
  last_rx_us_ = now_us;
  return can_ready_ && uart_ready_ && imu_ready_;
}

void AppV3::loop() noexcept {
  const std::uint64_t now_us = nowUs();
  pollSerial(now_us);
  can_.service(now_us);
  drainFeedback(now_us);
  if (now_us >= next_control_us_) {
    next_control_us_ = now_us + tap_fw::board::kControlPeriodUs;
    updateImu(now_us);
    controlTick(now_us);
    report(now_us);
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
      drive_forward_ = std::clamp(f, -0.3F, 0.3F);
      drive_yaw_ = std::clamp(y, -1.0F, 1.0F);
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
    else if (!std::strcmp(key, "accel")) lqr.setMaxGroundAccel(std::clamp(v, 0.2F, 2.0F));
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
  } else {
    Serial.printf("V3 unknown command '%s' (START DISABLE ESTOP CLEAR_ESTOP STOP DRIVE "
                  "GO LEGMODE POS|MIT SET GET BALANCE [CLEAR] TILTTEST)\n", line);
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
  const auto out = estimator_.update(body_.accel_m_s2, body_.gyro_rad_s, dt_s,
                                     static_cast<float>(now_us) * 1.0e-6F, 0.0F,
                                     false, quiet, {});
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
  in.dt_s = static_cast<float>(tap_fw::board::kControlPeriodUs) * 1.0e-6F;
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
  if (out.legs_locked) {
    legs_.setOffsets(ctrl_.legPose().hip_offset_rad, ctrl_.legPose().knee_offset_rad);
    (void)legs_.update(now_us, transport_);
  } else if (legs_.engaged()) {
    (void)legs_.release(transport_);
  }
  auto wheel_cmd = out.wheel_cmd_rad_s;
  if (tilt_test_ && out.state == V3Controller::State::Balancing) {
    if (last_state_ != static_cast<int>(out.state)) tilt_ref_rad_ = imu_.pitch_rad;
    // Nose-down (+pitch) must roll the wheels toward the head (+).
    const float w = std::clamp(kTiltTestGain * (imu_.pitch_rad - tilt_ref_rad_),
                               -kTiltTestMaxRadS, kTiltTestMaxRadS);
    wheel_cmd = {w, w};
  }
  driveWheels(out.wheels_enabled, wheel_cmd, now_us);
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
                  "h=%.1f htgt=%.1f knee=%+.3f\n",
                  static_cast<unsigned long>((now_us - state_since_us_) / 1000U),
                  tap_fw::v3::toString(out.state), imu_.pitch_rad, imu_.pitch_rate_rad_s,
                  last_wheel_cmd_[0], last_wheel_cmd_[1], in.wheel_rad_s[0], in.wheel_rad_s[1],
                  ctrl_.groundVelocity(), ctrl_.groundPosition(), ctrl_.balanceRef(),
                  legJointRad(0), legJointRad(1), legJointRad(2), legJointRad(3),
                  ctrl_.pitchRef(), ctrl_.positionError(), ctrl_.readyPhase(),
                  ctrl_.heldResets(), ctrl_.legPose().axle_below_hip_m * 1000.0F,
                  ctrl_.heightTarget() * 1000.0F, ctrl_.legPose().knee_offset_rad);
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
      "pitch_avg=%+.4f spread=%.4f balance=%+.4f%s\n",
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
      ctrl_.balanceAngle(), ctrl_.balanceAngleSet() ? "" : "(unset)");
}

}  // namespace tap_hw
