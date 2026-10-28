"""Bounded base velocity commands and encoder-odometry motion actions.

This module translates caller velocity requests and finite motion actions into
the native runner's DRIVE/STOP IPC. It does not acquire sensor data and is not
a balance controller; the native controller retains those responsibilities.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import time


# Native runner hard clamps. These are software ceilings, not validated
# physical robot operating limits.
NATIVE_MAX_LINEAR_M_S = 4.1888   # the wheel's no-load limit (1000 rpm x 0.04 m); the API defaults stay conservative
NATIVE_MAX_YAW_RAD_S = 12.0  # the controller's own yaw cap (lqr.max_yaw_rate_rad_s)
WHEEL_TRACK_M = 0.2654
MAX_DISTANCE_ACTION_M = 1000.0
MAX_TURN_ACTION_RAD = 100.0 * math.pi


class ActionStatus(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    COMPLETED = "completed"
    CANCELLED = "cancelled"
    TIMED_OUT = "timed_out"
    REJECTED = "rejected"


@dataclass(frozen=True)
class MotionLimits:
    """Explicit velocity bounds for a motion API instance."""

    max_linear_m_s: float = 0.20
    max_yaw_rad_s: float = 0.50

    def __post_init__(self) -> None:
        if not math.isfinite(self.max_linear_m_s) or not (
                0.0 < self.max_linear_m_s <= NATIVE_MAX_LINEAR_M_S):
            raise ValueError(
                f"max_linear_m_s must be in (0, {NATIVE_MAX_LINEAR_M_S}]")
        if not math.isfinite(self.max_yaw_rad_s) or not (
                0.0 < self.max_yaw_rad_s <= NATIVE_MAX_YAW_RAD_S):
            raise ValueError(
                f"max_yaw_rad_s must be in (0, {NATIVE_MAX_YAW_RAD_S}]")


class MotionAPI:
    """Velocity commands plus nonblocking encoder-odometry motion actions."""

    def __init__(self, backend, limits: MotionLimits = MotionLimits(),
                 require_station_capture: bool = True):
        self.backend = backend
        self.limits = limits
        self.require_station_capture = require_station_capture
        self.action_status = ActionStatus.IDLE
        self._action_kind = ""
        self._action_start_s = 0.0
        self._action_timeout_s = 0.0
        self._action_target = 0.0
        self._action_tolerance = 0.0
        self._action_max_rate = 0.0
        self._action_decel = 0.0
        self._action_start_left_m = 0.0
        self._action_start_right_m = 0.0
        self._action_start_position_m = 0.0
        self._fw_seq = 0

    def command_velocity(self, linear_m_s: float, yaw_rad_s: float) -> bool:
        """Set forward velocity and yaw rate; finite requests are clamped.

        A direct velocity request cancels any active distance/turn action.
        """
        if self.action_status == ActionStatus.RUNNING:
            self.action_status = ActionStatus.CANCELLED
            self._action_kind = ""
        if not (math.isfinite(linear_m_s) and math.isfinite(yaw_rad_s)):
            self.stop()
            return False
        return self._send_velocity(linear_m_s, yaw_rad_s)

    def _send_velocity(self, linear_m_s: float, yaw_rad_s: float) -> bool:
        linear = max(-self.limits.max_linear_m_s,
                     min(self.limits.max_linear_m_s, linear_m_s))
        yaw = max(-self.limits.max_yaw_rad_s,
                  min(self.limits.max_yaw_rad_s, yaw_rad_s))
        # Hardware backends return False when the serial write fails. Native
        # simulation backends historically return None, which means success.
        return self.backend.command_drive(linear, yaw) is not False

    def forward(self, speed_m_s: float = 0.10,
                distance_m: float | None = None,
                tolerance_m: float = 0.02,
                timeout_s: float | None = None) -> bool:
        """Move forward continuously, or travel a finite distance if given."""
        if not math.isfinite(speed_m_s) or speed_m_s < 0.0:
            self.stop()
            return False
        if distance_m is not None:
            if not math.isfinite(distance_m) or distance_m < 0.0:
                return self._reject_action()
            if distance_m == 0.0:
                self._end_action(ActionStatus.COMPLETED)
                return True
            return self._start_distance_action(
                distance_m, speed_m_s, tolerance_m, timeout_s, 0.30)
        return self.command_velocity(speed_m_s, 0.0)

    def backward(self, speed_m_s: float = 0.10,
                 distance_m: float | None = None,
                 tolerance_m: float = 0.02,
                 timeout_s: float | None = None) -> bool:
        """Move backward continuously, or travel a finite distance if given."""
        if not math.isfinite(speed_m_s) or speed_m_s < 0.0:
            self.stop()
            return False
        if distance_m is not None:
            if not math.isfinite(distance_m) or distance_m < 0.0:
                return self._reject_action()
            if distance_m == 0.0:
                self._end_action(ActionStatus.COMPLETED)
                return True
            return self._start_distance_action(
                -distance_m, speed_m_s, tolerance_m, timeout_s, 0.30)
        return self.command_velocity(-speed_m_s, 0.0)

    def rotate(self, yaw_rate_rad_s: float = 0.25,
               angle_rad: float | None = None,
               tolerance_rad: float = 0.04,
               timeout_s: float | None = None) -> bool:
        """Rotate continuously, or through a finite angle if ``angle_rad`` is set.

        Finite turns take their direction from ``angle_rad`` and use the
        magnitude of ``yaw_rate_rad_s`` as the speed cap.
        """
        if not math.isfinite(yaw_rate_rad_s):
            self.stop()
            return False
        if angle_rad is not None:
            if not math.isfinite(angle_rad):
                return self._reject_action()
            if angle_rad == 0.0:
                self._end_action(ActionStatus.COMPLETED)
                return True
            return self._start_turn_action(
                angle_rad, abs(yaw_rate_rad_s), tolerance_rad, timeout_s, 0.80)
        return self.command_velocity(0.0, yaw_rate_rad_s)

    def stop(self) -> bool:
        """Request the native runner's stationary stop behavior."""
        if self.action_status == ActionStatus.RUNNING:
            self.action_status = ActionStatus.CANCELLED
        self._action_kind = ""
        return self.backend.command_stop() is not False

    def _start_distance_action(self, distance_m: float, speed_m_s: float,
                               tolerance_m: float,
                               timeout_s: float | None,
                               decel_m_s2: float) -> bool:
        """Start relative travel using encoder odometry."""

        if (not math.isfinite(distance_m) or distance_m == 0.0 or
                abs(distance_m) > MAX_DISTANCE_ACTION_M or
                not math.isfinite(speed_m_s) or speed_m_s <= 0.0 or
                speed_m_s > self.limits.max_linear_m_s or
                not math.isfinite(tolerance_m) or tolerance_m <= 0.0 or
                tolerance_m >= abs(distance_m) or
                not math.isfinite(decel_m_s2) or decel_m_s2 <= 0.0):
            return self._reject_action()
        timeout = (max(3.0, 2.5 * abs(distance_m) / speed_m_s + 2.0)
                   if timeout_s is None else timeout_s)
        if not self._valid_timeout(timeout):
            return self._reject_action()
        self.cancel_action()
        state = self._ready_odometry(
            "wheel_position", "wheel_left_position", "wheel_right_position")
        if state is None:
            return self._reject_action()
        if self._firmware_motion():
            return self._begin_firmware_action("fw_distance", distance_m, speed_m_s, timeout)
        self._begin_action("distance", distance_m, tolerance_m, speed_m_s,
                           decel_m_s2, timeout, state["wheel_position"],
                           state["wheel_left_position"],
                           state["wheel_right_position"])
        return True

    def _start_turn_action(self, angle_rad: float, yaw_rate_rad_s: float,
                           tolerance_rad: float, timeout_s: float | None,
                           angular_decel_rad_s2: float) -> bool:
        """Start a relative turn from differential wheel odometry."""

        if (not math.isfinite(angle_rad) or angle_rad == 0.0 or
                abs(angle_rad) > MAX_TURN_ACTION_RAD or
                not math.isfinite(yaw_rate_rad_s) or yaw_rate_rad_s <= 0.0 or
                yaw_rate_rad_s > self.limits.max_yaw_rad_s or
                not math.isfinite(tolerance_rad) or tolerance_rad <= 0.0 or
                tolerance_rad >= abs(angle_rad) or
                not math.isfinite(angular_decel_rad_s2) or
                angular_decel_rad_s2 <= 0.0):
            return self._reject_action()
        timeout = (max(3.0, 2.5 * abs(angle_rad) / yaw_rate_rad_s + 2.0)
                   if timeout_s is None else timeout_s)
        if not self._valid_timeout(timeout):
            return self._reject_action()
        self.cancel_action()
        state = self._ready_odometry(
            "wheel_left_position", "wheel_right_position")
        if state is None:
            return self._reject_action()
        if self._firmware_motion():
            return self._begin_firmware_action("fw_turn", angle_rad, yaw_rate_rad_s, timeout)
        mean_position = 0.5 * (state["wheel_left_position"] +
                                state["wheel_right_position"])
        self._begin_action("turn", angle_rad, tolerance_rad, yaw_rate_rad_s,
                           angular_decel_rad_s2, timeout, mean_position,
                           state["wheel_left_position"],
                           state["wheel_right_position"])
        return True

    def update_action(self, now_s: float | None = None) -> ActionStatus:
        """Advance an active finite motion action from fresh runner telemetry."""
        if self.action_status != ActionStatus.RUNNING:
            return self.action_status
        now = time.monotonic() if now_s is None else now_s
        if (not math.isfinite(now) or
                now - self._action_start_s >= self._action_timeout_s):
            self._end_action(ActionStatus.TIMED_OUT)
            return self.action_status
        if self._action_kind.startswith("fw_"):
            # The firmware runs the move on its own odometry / gyro, ends it by itself (done, timeout, left
            # balancing) and says so with a MOT line. A late or missing telemetry frame (Wi-Fi) must not make
            # this side send STOP: that cut the move short.
            return self._update_firmware_action(self.backend.telemetry)
        backend_stamp = getattr(self.backend, "telemetry_received_monotonic_s", 0.0)
        if (backend_stamp <= 0.0 or now - backend_stamp < 0.0 or
                now - backend_stamp > 0.25):
            self._end_action(ActionStatus.REJECTED)
            return self.action_status
        fields = self.backend.telemetry
        if fields.get("mode") not in ("Balance", "Balancing", "Drive"):
            self._end_action(ActionStatus.REJECTED)
            return self.action_status
        if self._action_kind.startswith("fw_"):
            return self._update_firmware_action(fields)
        try:
            if self._action_kind == "distance":
                current = float(fields["wheel_position"])
                progress = current - self._action_start_position_m
            else:
                left = float(fields["wheel_left_position"])
                right = float(fields["wheel_right_position"])
                progress = ((right - self._action_start_right_m) -
                            (left - self._action_start_left_m)) / WHEEL_TRACK_M
        except (KeyError, TypeError, ValueError):
            self._end_action(ActionStatus.REJECTED)
            return self.action_status
        if not math.isfinite(progress):
            self._end_action(ActionStatus.REJECTED)
            return self.action_status

        remaining = self._action_target - progress
        if abs(remaining) <= self._action_tolerance:
            self._end_action(ActionStatus.COMPLETED)
            return self.action_status
        speed = min(self._action_max_rate,
                    math.sqrt(2.0 * self._action_decel * abs(remaining)))
        signed_speed = math.copysign(speed, remaining)
        sent = (self._send_velocity(signed_speed, 0.0)
                if self._action_kind == "distance"
                else self._send_velocity(0.0, signed_speed))
        if not sent:
            self._end_action(ActionStatus.REJECTED)
        return self.action_status

    # -- finite moves run by the firmware (MOVE / TURN, progress as MOT lines) ------------------
    def _firmware_motion(self) -> bool:
        return (hasattr(self.backend, "command_move") and
                self.backend.telemetry.get("fw_motion") == "1")

    def _begin_firmware_action(self, kind: str, target: float, rate: float,
                               timeout_s: float) -> bool:
        self._fw_seq = int(self.backend.telemetry.get("motion_seq", "0"))
        sent = (self.backend.command_move(target, rate) if kind == "fw_distance"
                else self.backend.command_turn(target, rate))
        if sent is False:
            return self._reject_action()
        self._action_kind = kind
        self._action_start_s = time.monotonic()
        self._action_timeout_s = timeout_s + 6.0   # the firmware has its own timeout (same formula)
        self._action_target = target
        self.action_status = ActionStatus.RUNNING
        return True

    def _update_firmware_action(self, fields) -> ActionStatus:
        try:
            seq = int(fields.get("motion_seq", "0"))
        except ValueError:
            seq = 0
        if seq <= self._fw_seq:
            return self.action_status   # no answer to our command yet
        state = fields.get("motion_state", "")
        if state == "done":
            self._end_action(ActionStatus.COMPLETED)
        elif state == "timeout":
            self._end_action(ActionStatus.TIMED_OUT)
        elif state == "cancelled":
            self._action_kind = ""
            self.action_status = ActionStatus.CANCELLED
        elif state in ("aborted", "refused"):
            self._end_action(ActionStatus.REJECTED)
        return self.action_status

    def cancel_action(self) -> bool:
        """Cancel an active finite action and request a stationary stop."""
        if self.action_status == ActionStatus.RUNNING:
            self.action_status = ActionStatus.CANCELLED
            self._action_kind = ""
            return self.backend.command_stop() is not False
        return True

    def _begin_action(self, kind: str, target: float, tolerance: float,
                      max_rate: float, decel: float, timeout: float,
                      position: float, left: float, right: float) -> None:
        self.cancel_action()
        self._action_kind = kind
        self._action_start_s = time.monotonic()
        self._action_timeout_s = timeout
        self._action_target = target
        self._action_tolerance = tolerance
        self._action_max_rate = max_rate
        self._action_decel = decel
        self._action_start_position_m = position
        self._action_start_left_m = left
        self._action_start_right_m = right
        self.action_status = ActionStatus.RUNNING

    def _ready_odometry(self, *keys: str) -> dict[str, float] | None:
        if not getattr(self.backend, "running", False):
            return None
        fields = self.backend.telemetry
        stamp = getattr(self.backend, "telemetry_received_monotonic_s", 0.0)
        now = time.monotonic()
        if stamp <= 0.0 or now - stamp > 0.25:
            return None
        if fields.get("mode") not in ("Balance", "Balancing"):
            return None
        if self.require_station_capture:
            if "drive_ready" in fields:
                if fields["drive_ready"] != "1":
                    return None
            elif (fields.get("station_eligible") != "1" or
                  fields.get("station_capture_ready") != "1"):
                return None
        try:
            values = {key: float(fields[key]) for key in keys}
        except (KeyError, TypeError, ValueError):
            return None
        return values if all(math.isfinite(value) for value in values.values()) else None

    def _valid_timeout(self, timeout_s: float) -> bool:
        return (math.isfinite(timeout_s) and 0.1 <= timeout_s <= 3600.0)

    def _reject_action(self) -> bool:
        if self.action_status == ActionStatus.RUNNING:
            self.cancel_action()
        self.action_status = ActionStatus.REJECTED
        self._action_kind = ""
        return False

    def _end_action(self, status: ActionStatus) -> None:
        stopped = self.backend.command_stop() is not False
        if status == ActionStatus.COMPLETED and not stopped:
            status = ActionStatus.REJECTED
        self._action_kind = ""
        self.action_status = status
