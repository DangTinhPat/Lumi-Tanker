#pragma once

#include <array>
#include <cstdint>

#include "robot_control/GeneratedGeometry.hpp"
#include "robot_hal/MotorMapping.hpp"
#include "tap_fw/ImuMount.hpp"

namespace tap_fw::board {

// POC hardware mapping from docs/data/POC - Robot version 1.
inline constexpr int kButtonGpio = 6;
inline constexpr int kLed1Gpio = 39;
inline constexpr int kLed2Gpio = 40;
inline constexpr int kLed3Gpio = 41;
inline constexpr int kSpareGpio = 42;
inline constexpr int kUsbDmGpio = 19;
inline constexpr int kUsbDpGpio = 20;
inline constexpr int kDebugUartTxGpio = 43;
inline constexpr int kDebugUartRxGpio = 44;
inline constexpr int kBootGpio = 0;
// EN is the ESP32-S3 reset input, not a normal GPIO number.
inline constexpr int kI2cSdaGpio = 8;
inline constexpr int kI2cSclGpio = 9;
inline constexpr int kCanTxGpio = 4;
inline constexpr int kCanRxGpio = 5;
inline constexpr int kUart1TxGpio = 17;
inline constexpr int kUart1RxGpio = 18;
inline constexpr int kUart2TxGpio = 15;
inline constexpr int kUart2RxGpio = 16;
inline constexpr int kImuCsGpio = 10;
inline constexpr int kImuMosiGpio = 11;
inline constexpr int kImuSclkGpio = 12;
inline constexpr int kImuMisoGpio = 13;
inline constexpr int kImuIntGpio = 14;
inline constexpr std::uint32_t kCanBitrate = 500'000U;
inline constexpr std::uint32_t kMotorUartBaud = 1'000'000U;
inline constexpr std::uint32_t kControlPeriodUs = 2'000U;
inline constexpr std::uint32_t kFeedbackMaxAgeUs = 20'000U;
inline constexpr std::uint32_t kCanProbePeriodUs = 10'000U;
inline constexpr std::uint32_t kWheelFeedbackPeriodUs = 2'000U;
inline constexpr std::uint32_t kBlePollPeriodUs = 10'000U;
inline constexpr bool kWheelDirectionsVerified =
    robot_control::motor_mapping::kWheelDirectionsVerified;
inline constexpr bool kWheelSpeedScaleVerified =
    robot_control::motor_mapping::kWheelSpeedScaleVerified;
inline constexpr bool kImuFrameVerified = true;
// MPU6500 axes as mounted on the PCB, mapped to the body frame before the
// estimator (see ImuMount.hpp): chip Y forward, chip X right, chip Z up.
// Measured on the robot 2026-09-26 with tools/imu_axis_check.py: a nose-down
// tilt drove chip Y to -2.42 m/s^2 and firmware pitch to +0.205 rad with this
// map. The opposite map (kChipXLeftYBack) was tried later and the first wheel
// balance ran away: forward wheel acceleration read as nose-down pitch
// (+0.3 rad with the gyro showing no rotation), a positive feedback loop.
// SENS prints both chip (chip_acc) and body (acc) accelerations.
inline constexpr imu_mount::AxisMap kImuAxisMap = imu_mount::kChipYForwardXRight;
static_assert(imu_mount::isRotation(kImuAxisMap), "IMU axis map must be a rotation");
// CAN budget at 500 kbit/s: leg MIT commands at 250 Hz (plus one reply each)
// keep the bus near 60 %; 500 Hz would exceed it. The controller itself
// still runs every kControlPeriodUs.
inline constexpr std::uint32_t kLegCommandPeriodUs = 4'000U;
// Hardware leg servo gain scale. The model's task-space stiffness gives
// Kp ~40 N.m/rad at the neutral pose; on the real 8:1 GDS68 joints that
// chattered ("buzzing", accelerometer saturating at 2 g). The bench jog ran
// smoothly at Kp 8 / Kd 0.6. Kp is multiplied by this, Kd by its square root
// (same damping ratio); gravity feed-forward still carries the body weight.
inline constexpr float kLegGainScale = 0.3F;
// Cap on that scaled joint Kp (N.m/rad). The task-space mapping gives the hip
// ~43 near the straight (hanging) leg, 3.5x the neutral 12, and the legs
// buzzed while folding from there. The cap equals the neutral-pose value, so
// balancing at the natural stance is unchanged.
inline constexpr float kLegMaxKpNmPerRad = 12.0F;
// GDS68 current limit sent with Set_Limits (0x00F) on every enable edge. On
// the robot 2026-09-26 the legs delivered only ~0.2 N.m (Iq ~0.4 A at
// Kt 0.47 N.m/A) against a 3 N.m command and stalled where the shank weight
// equalled it. 8 A ~ 3.8 N.m at the output, below the 10.5 A rating.
inline constexpr float kLegCurrentLimitA = 8.0F;
// Rotor turns/s (8:1): 20 = 15.7 rad/s at the joint, far above leg motion.
inline constexpr float kLegVelLimitTurnsS = 20.0F;
// Leg position integral, applied by shifting the MIT position target. On the
// robot 2026-09-26 the GDS68 ignored the MIT torque feed-forward (a 2.6 N.m
// T_ff did not lift a 0.25 N.m shank) while the Kp spring worked, so a
// torque integral could not close the gap and, fed into the envelope clamp,
// starved or overdrove the spring. Offset rate [1/s] and bound [rad].
inline constexpr float kLegTargetIntegralPerS = 1.5F;
inline constexpr float kLegTargetIntegralMaxRad = 0.5F;
// LQR gain schedule points, built at boot (~50 ms each on the S3) so no
// double-precision DARE solve runs inside the 2 ms loop. 18 points track the
// exact solve within 1e-4 relative (tuned_config_tests).
inline constexpr unsigned kLqrGainTablePoints = 18U;
// While disabled, IDLE / Torque Disable are repeated at 50 Hz (and sent at
// once on the disable edge) instead of every control tick.
inline constexpr std::uint32_t kDisabledRepeatPeriodUs = 20'000U;
// Leg zero reference taken on Start #1 (or the console's HOME HANG) when the
// legs are not homed yet. GravityHang: body held up, both legs hanging free,
// knees at their extension stop (LegHoming.hpp). ManualExtended: the operator
// lifts the body and presents both extended legs below it before zero capture.
// FixtureNatural is the legacy neutral-pose capture.
enum class LegHomingPose { FixtureNatural, GravityHang, ManualExtended };
// v1 (2026-09-26): the operator presents both legs extended at Start #1 and
// that pose is the zero, as in v2.
inline constexpr LegHomingPose kLegHomingPose = LegHomingPose::ManualExtended;
// v1 two-wheel mode: the four leg joints stay locked at the captured extended
// pose through FixtureHold and balance (no fold to the natural stance); only
// the two wheels move.
inline constexpr bool kLegsLockedAtZeroPose = true;
// Two-wheel lock stiffness: one fixed MIT Kp for all four leg joints (the
// model's task-space stiffness is not used), no target integral (it moved the
// locked legs), and the GIM6010-8 rated torque as the envelope so the locked
// legs carry the body.
// Kp/Kd near the bench jog that ran quietly (Kp 8 / Kd 0.6); higher values
// buzzed. Live-tunable: console SET leg_kp / SET leg_kd.
inline constexpr float kLegLockKpNmPerRad = 15.0F;
inline constexpr float kLegLockKdNmSPerRad = 0.5F;
inline constexpr float kLegLockTorqueNm = 5.0F;
// Wheel/leg kinematic acceleration compensation in the IMU pitch estimator.
// Off: it assumes the wheels roll the body along. With the body held (or the
// wheels slipping) wheel spin became a fake body acceleration and a fake
// +-0.3 rad pitch while accelerometer and gyro both read upright, and the
// wheels chased it into the runaway guard (2026-09-26, both directions).
inline constexpr bool kImuWheelKinematicCompensation = false;
// Dabble BLE off: commands come from the USB serial console / GUI only.
inline constexpr bool kBleEnabled = false;
// Knee angle at the extension stop. CAD straight leg = top of the native
// knee envelope; the physical stop has not been measured against it.
inline constexpr float kKneeExtensionStopRad = robot_control::geometry::native_knee_max_rad;
// Joints must be this still to home, and the zero is dropped when a leg's
// MIT feedback is silent for kLegZeroLossUs (a driver reboot restarts its
// position from zero).
inline constexpr float kHomingMaxJointSpeedRadS = 0.10F;
// GravityHang homing: motors stay limp while legs relax toward the hanging
// pose; the reference is captured only after all joints and the body settle.
// ManualExtended v2 captures the operator-presented straight-leg pose.
inline constexpr std::uint32_t kHangSettleUs = 2'000'000U;
inline constexpr std::uint32_t kHangSettleTimeoutUs = 30'000'000U;
inline constexpr std::uint32_t kManualExtendedSettleUs = 500'000U;
inline constexpr std::uint32_t kManualExtendedTimeoutUs = 30'000'000U;
inline constexpr std::uint32_t kLegZeroLossUs = 300'000U;
// A leg silent longer than kLegZeroLossUs keeps its zero unless, when it
// talks again, it reports ~0 after a clearly non-zero position (the driver
// rebooted and restarted counting), or it stays silent past kLegZeroForgetUs.
// A CAN bus-off recovery (~1 s, drivers still running) no longer forces a
// re-home.
inline constexpr std::uint32_t kLegZeroForgetUs = 10'000'000U;
inline constexpr float kLegRebootZeroRad = 0.01F;
inline constexpr float kLegRebootMinTravelRad = 0.05F;
// GDS68 heartbeats arrive every 100 ms. After an enable edge the drivers must
// report CLOSED_LOOP within kClosedLoopGraceUs, or the controller disables:
// MIT replies keep flowing from a driver that refused closed loop.
inline constexpr std::uint32_t kHeartbeatMaxAgeUs = 350'000U;
inline constexpr std::uint32_t kClosedLoopGraceUs = 500'000U;
// Bring-up runaway guard: both wheels above this speed for longer than
// kWheelRunawayUs while enabled means the balance loop has lost the wheels.
inline constexpr float kWheelRunawayRadS = 25.0F;
inline constexpr std::uint32_t kWheelRunawayUs = 300'000U;
// After enabling and again after the fixture release, trace the balance loop
// (wheel command vs measurement, adaptive trim) at 25 Hz: long enough to see
// the trim settle, light enough for the 115200 baud console.
inline constexpr std::uint32_t kBalanceTraceUs = 10'000'000U;
inline constexpr std::uint32_t kBalanceTracePeriodUs = 40'000U;
// true: Start sends ENABLE + FIXTURE_RELEASE (straight into balance).
// false: Start #1 = ENABLE (home legs, FixtureHold); the operator stands the
// robot at its balance point; Start #2 = FIXTURE_RELEASE, which seeds the
// adaptive balance angle with the pitch at that moment.
#if defined(TAP_IMPULSE_STYLE_V2)
// Two presses: zero capture needs the body lifted (legs unloaded, knees at
// the stop) while balancing needs the wheels on the floor. Start #1 captures
// the extended pose and holds it; the operator sets the wheels down; Start #2
// releases into balance (angle seeded from the pitch at that moment).
inline constexpr bool kAutoFixtureRelease = false;
// Full profile is an explicit opt-in build. It assumes the operator has
// confirmed the manual extended-pose mapping and extended-leg balance setup.
#if defined(TAP_IMPULSE_V2_FULL_ENABLE)
inline constexpr bool kManualExtendedPoseMappingVerified = true;
#else
inline constexpr bool kManualExtendedPoseMappingVerified = false;
#endif
// Do not enable the extended-leg balance path in the default v2 diagnostic build.
#if defined(TAP_IMPULSE_V2_FULL_ENABLE)
inline constexpr bool kImpulseExtendedLegBalanceVerified = true;
#else
inline constexpr bool kImpulseExtendedLegBalanceVerified = false;
#endif
inline constexpr std::uint64_t kImpulsePrepositionTimeoutUs = 15'000'000U;
inline constexpr std::uint64_t kImpulseBalanceWarmupUs = 1'000'000U;
inline constexpr std::uint64_t kImpulseBalanceRampTimeoutUs = 2'000'000U;
#else
// Two-wheel mode: one START captures and locks the legs where they are and
// goes straight into wheel balance (the robot is held on its wheels).
inline constexpr bool kAutoFixtureRelease = kLegsLockedAtZeroPose;
#endif
// D-pad / height motion from the phone needs a phone packet this recent;
// otherwise it becomes STOP. Provisional until the Dabble packet cadence has
// been measured on the robot (logged as BLE rx gap).
inline constexpr std::uint32_t kBleMotionLeaseUs = 500'000U;
// Bench option: open TWAI listen-only so leg feedback can be read while the
// CAN TX path is being debugged. Nothing is transmitted or
// acknowledged on CAN, and enable stays refused while this is true.
inline constexpr bool kCanListenOnly = false;
// Bench option: TWAI in normal mode, but every firmware CAN transmission is
// suppressed. Instead one single-shot frame with an unassigned ID (node 62,
// cmd 0, no GDS68 addresses it) is sent per second, so a TX failure can be
// separated from the firmware's traffic pattern. Enable stays refused.
inline constexpr bool kCanTxSingleFrameTest = false;
inline constexpr std::uint16_t kCanTxTestId = 0x7C0U;
inline constexpr std::uint32_t kCanTxTestPeriodUs = 1'000'000U;
// After a bus-off, wait this long before starting TWAI recovery so a broken
// TX path does not keep disturbing the bus. Short: the enabled legs ride
// through an outage up to kLegRideThroughUs (the GDS68 hold their last MIT
// set-point meanwhile), and a 1 s delay turned every bus-off into a drop.
inline constexpr std::uint32_t kCanBusOffRecoveryDelayUs = 20'000U;
inline constexpr std::uint32_t kLegRideThroughUs = 250'000U;
// Diagnostics: one GDS68 DC-bus voltage request (0x017) per this period,
// round robin; SENS prints the lowest voltage seen per report.
inline constexpr std::uint32_t kBusVoltagePollUs = 50'000U;

struct MotorIds {
  std::array<robot_control::motor_mapping::LegCanBinding, 4> leg_can =
      robot_control::motor_mapping::kVerifiedLegCanBindings;
  std::array<robot_control::motor_mapping::WheelUartBinding, 2> wheel_uart =
      robot_control::motor_mapping::kWheelUartBindings;
};

}  // namespace tap_fw::board
