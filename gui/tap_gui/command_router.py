"""Validate and arbitrate high-level robot commands from all input sources."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import time
from typing import Callable, Optional

from .motion_api import ActionStatus, MAX_DISTANCE_ACTION_M, MotionAPI


@dataclass(frozen=True)
class CommandResult:
    command_id: str
    source: str
    action: str
    status: str
    message: str
    effective_args: dict

    def as_dict(self) -> dict:
        return asdict(self)


class CommandMotionSource:
    """MotionAPI-shaped source handle routed through the priority arbiter."""

    def __init__(self, router: "RobotCommandRouter", source: str):
        self.router = router
        self.source = source
        self.limits = router.motion.limits

    def command_velocity(self, linear_m_s: float, yaw_rad_s: float) -> bool:
        return self.router.command_velocity(self.source, linear_m_s, yaw_rad_s)

    def stop(self) -> None:
        self.router.stop_source(self.source)


class RobotCommandRouter:
    """Single command gateway implementing the checked-in command-tree policy.

    Finite move and turn actions use MotionAPI's encoder odometry. The voice
    schema's duration is a timeout estimate only; it is not used as a distance
    measurement. ``set_height`` uses the firmware's SET height command and
    completes when fresh telemetry reaches the requested height.
    """

    SOURCES = {"manual_ui", "luvox", "system"}
    HEIGHT_MIN_M = 0.078
    HEIGHT_MAX_M = 0.1886
    HEIGHT_TOLERANCE_M = 0.005
    HEIGHT_TIMEOUT_S = 30.0

    def __init__(
        self,
        motion: MotionAPI,
        *,
        can_control: Callable[[], bool],
        send_console: Callable[[str], Optional[bool]],
        emergency_stop: Callable[[], Optional[bool]],
        schema_path: Optional[Path] = None,
        on_result: Optional[Callable[[CommandResult], None]] = None,
    ):
        self.motion = motion
        self.can_control = can_control
        self.send_console = send_console
        self.emergency_stop_callback = emergency_stop
        self.on_result = on_result
        path = schema_path or Path(__file__).resolve().parents[1] / "config/robot_command_tree.json"
        with open(path, encoding="utf-8") as schema_file:
            self.schema = json.load(schema_file)
        self.active_level = 0
        self.active_action = ""
        self.active_command_id = ""
        self.active_deadline_s = 0.0
        self.active_height_m: Optional[float] = None
        self.last_results: dict[str, CommandResult] = {}
        self.command_fingerprints: dict[str, str] = {}

    def motion_source(self, source: str) -> CommandMotionSource:
        if source not in self.SOURCES:
            raise ValueError(f"unknown command source: {source}")
        return CommandMotionSource(self, source)

    def submit_json_line(self, payload: str, source_override: Optional[str] = None) -> dict:
        """Entry point for a voice/tool JSON line; returns a status envelope."""
        try:
            command = json.loads(payload)
        except (TypeError, json.JSONDecodeError) as exc:
            return self._reject("", source_override or "luvox", "", str(exc)).as_dict()
        if not isinstance(command, dict):
            return self._reject("", source_override or "luvox", "",
                                "command envelope must be a JSON object").as_dict()
        if source_override is not None:
            command = {**command, "source": source_override}
        return self.execute(command).as_dict()

    def execute(self, envelope: dict) -> CommandResult:
        if not isinstance(envelope, dict):
            return self._reject("", "", "", "command envelope must be an object")
        command_id = envelope.get("command_id", "")
        source = envelope.get("source", "")
        action = envelope.get("action", "")
        if not isinstance(command_id, str) or not command_id.strip():
            return self._reject("", str(source), str(action), "command_id must be a non-empty string")
        if not isinstance(source, str) or not isinstance(action, str):
            return self._reject(command_id, "", "", "source and action must be strings")
        if source not in self.SOURCES:
            return self._reject(command_id, source, action, "unsupported command source")
        if action not in self.schema["commands"]:
            return self._reject(command_id, source, action, "unsupported command action")
        args = envelope.get("args", {})
        if not isinstance(args, dict):
            return self._reject(command_id, source, action, "args must be an object")
        try:
            args = self._validate_args(action, args)
        except ValueError as exc:
            return self._reject(command_id, source, action, str(exc))

        fingerprint = json.dumps(
            {"source": source, "action": action, "args": args},
            sort_keys=True, separators=(",", ":"), allow_nan=False)
        previous_fingerprint = self.command_fingerprints.get(command_id)
        if previous_fingerprint is not None:
            if previous_fingerprint != fingerprint:
                return CommandResult(
                    command_id, source, action,
                    "rejected", "command_id was already used for a different command", args)
            previous_result = self.last_results.get(command_id)
            if previous_result is not None:
                return previous_result
        else:
            self.command_fingerprints[command_id] = fingerprint

        if action == "emergency_stop":
            self._cancel_active(stop=True)
            self.active_level = 0
            self.active_action = ""
            self.active_command_id = ""
            sent = self.emergency_stop_callback()
            return self._record(CommandResult(
                command_id, source, action,
                "completed" if sent is not False else "rejected",
                "ESTOP sent; all behaviors cancelled" if sent is not False
                else "behaviors cancelled, but ESTOP could not be written to serial",
                args))


        if not self.can_control():
            return self._reject(command_id, source, action,
                                "robot must be connected and Balancing before motion")

        if not self._cancel_active(stop=True):
            return self._reject(command_id, source, action,
                                "could not stop the previous command over serial", args)
        if action == "move":
            return self._execute_move(command_id, source, args)
        if action == "turn":
            return self._execute_turn(command_id, source, args)
        if action == "set_height":
            return self._execute_height(command_id, source, args)
        return self._reject(command_id, source, action, "unsupported command")

    def status(self, command_id: str) -> Optional[dict]:
        """Return the latest lifecycle status for a previously submitted command."""
        result = self.last_results.get(command_id)
        return None if result is None else result.as_dict()

    def _validate_args(self, action: str, supplied: dict) -> dict:
        spec = self.schema["commands"][action]
        params = spec["params"]
        unknown = set(supplied) - set(params)
        if unknown:
            raise ValueError(f"unknown {action} argument(s): {', '.join(sorted(unknown))}")
        missing = set(spec.get("required", ())) - set(supplied)
        if missing:
            raise ValueError(f"missing required argument(s): {', '.join(sorted(missing))}")
        values = {name: definition.get("default") for name, definition in params.items()}
        values.update(supplied)
        for name, definition in params.items():
            value = values[name]
            if value is None:
                continue
            kind = definition["type"]
            if kind == "boolean":
                valid_type = type(value) is bool
            elif kind == "number":
                valid_type = isinstance(value, (int, float)) and not isinstance(value, bool)
            else:
                valid_type = isinstance(value, str)
            if not valid_type:
                raise ValueError(f"{name} must be {kind}")
            if "enum" in definition and value not in definition["enum"]:
                raise ValueError(f"{name} must be one of {definition['enum']}")
            if kind == "number":
                if not math.isfinite(float(value)):
                    raise ValueError(f"{name} must be finite")
                if "min" in definition and value < definition["min"]:
                    raise ValueError(f"{name} must be >= {definition['min']}")
                if "max" in definition and value > definition["max"]:
                    raise ValueError(f"{name} must be <= {definition['max']}")
        if action == "set_height":
            absolute = values["height_m"] is not None
            relative = values["direction"] is not None
            if absolute == relative:
                raise ValueError("set_height needs either height_m or direction (+ optional delta_m)")
        return values

    def _execute_move(self, command_id: str, source: str, args: dict) -> CommandResult:
        speed = min(float(args["speed_mps"]), self.motion.limits.max_linear_m_s)
        distance = float(args["distance_m"])
        if distance > MAX_DISTANCE_ACTION_M:
            return self._reject(command_id, source, "move",
                                f"distance exceeds MotionAPI limit ({MAX_DISTANCE_ACTION_M:g} m)")
        signed_speed = speed if args["direction"] == "forward" else -speed
        timeout_s = max(3.0, 2.5 * distance / speed + 2.0) if distance > 0.0 else 0.0
        if timeout_s > 60.0:
            return self._reject(command_id, source, "move", "estimated action timeout exceeds 60 s", args)
        accepted = self.motion.forward(speed, distance_m=distance) if signed_speed >= 0.0 \
            else self.motion.backward(speed, distance_m=distance)
        effective = {**args, "speed_mps": speed}
        if not accepted:
            return self._reject(command_id, source, "move",
                                "fresh encoder odometry is unavailable or action parameters were rejected",
                                effective)
        return self._start_result(command_id, source, "move", effective, timeout_s)

    def _execute_turn(self, command_id: str, source: str, args: dict) -> CommandResult:
        requested_rate = math.radians(float(args["speed_dps"]))
        yaw_rate = min(requested_rate, self.motion.limits.max_yaw_rad_s)
        angle = math.radians(float(args["angle_deg"]))
        if args["direction"] == "left":
            angle = -angle
        timeout_s = max(3.0, 2.5 * abs(angle) / yaw_rate + 2.0) if angle != 0.0 else 0.0
        if timeout_s > 60.0:
            return self._reject(command_id, source, "turn", "estimated action timeout exceeds 60 s", args)
        accepted = self.motion.rotate(yaw_rate, angle_rad=angle)
        effective = {**args, "speed_dps": math.degrees(yaw_rate), "angle_deg": abs(math.degrees(angle))}
        if not accepted:
            return self._reject(command_id, source, "turn",
                                "fresh encoder odometry is unavailable or action parameters were rejected",
                                effective)
        return self._start_result(command_id, source, "turn", effective, timeout_s)

    def _execute_height(self, command_id: str, source: str, args: dict) -> CommandResult:
        telemetry = getattr(self.motion.backend, "telemetry", {})
        if args["height_m"] is not None:
            requested = float(args["height_m"])
        else:
            current = telemetry.get("height_m")
            try:
                current = float(current)
            except (TypeError, ValueError):
                current = math.nan
            stamp = getattr(self.motion.backend, "telemetry_received_monotonic_s", 0.0)
            if not math.isfinite(current) or time.monotonic() - stamp > 0.25:
                return self._reject(command_id, source, "set_height", "fresh height telemetry is unavailable", args)
            delta = float(args["delta_m"])
            requested = current + (delta if args["direction"] == "up" else -delta)
        if requested < self.HEIGHT_MIN_M or requested > self.HEIGHT_MAX_M:
            return self._reject(
                command_id, source, "set_height",
                f"target height must be within {self.HEIGHT_MIN_M:.4f}..{self.HEIGHT_MAX_M:.4f} m",
                args)
        target = requested
        if self.send_console(f"SET height {target * 1000.0:.1f}") is False:
            return self._reject(command_id, source, "set_height",
                                "failed to write height target to serial", args)
        self.active_level = 1
        self.active_action = "set_height"
        self.active_command_id = command_id
        self.active_height_m = target
        self.active_deadline_s = time.monotonic() + self.HEIGHT_TIMEOUT_S
        effective = {**args, "target_height_m": target}
        return self._record(CommandResult(
            command_id, source, "set_height", "running",
            "firmware height target sent; waiting for fresh height telemetry", effective))

    def _start_result(self, command_id: str, source: str, action: str,
                      args: dict, timeout_s: float) -> CommandResult:
        status = "running" if self.motion.action_status == ActionStatus.RUNNING else "completed"
        self.active_level = 1 if status == "running" else 0
        self.active_action = action if status == "running" else ""
        self.active_command_id = command_id if status == "running" else ""
        self.active_deadline_s = time.monotonic() + timeout_s if timeout_s else 0.0
        return self._record(CommandResult(
            command_id, source, action, status,
            "encoder-based action started" if status == "running" else "zero-distance action completed",
            args))

    def update(self, now_s: Optional[float] = None) -> Optional[CommandResult]:
        now = time.monotonic() if now_s is None else now_s
        if self.active_action in ("move", "turn"):
            previous = self.motion.action_status
            status = self.motion.update_action(now)
            if status != ActionStatus.RUNNING:
                label = "completed" if status == ActionStatus.COMPLETED else "cancelled"
                if status in (ActionStatus.REJECTED, ActionStatus.TIMED_OUT):
                    label = "rejected"
                return self._finish_active(label, f"encoder action {status.value}")
            if now >= self.active_deadline_s:
                self.motion.cancel_action()
                return self._finish_active("rejected", "encoder action exceeded its safety timeout")
            return None
        if self.active_action == "set_height":
            telemetry = getattr(self.motion.backend, "telemetry", {})
            stamp = getattr(self.motion.backend, "telemetry_received_monotonic_s", 0.0)
            try:
                height = float(telemetry.get("height_m"))
            except (TypeError, ValueError):
                height = math.nan
            if stamp > 0.0 and now - stamp <= 0.25 and math.isfinite(height):
                if abs(height - float(self.active_height_m)) <= self.HEIGHT_TOLERANCE_M:
                    return self._finish_active("completed", "height target reached")
            if now >= self.active_deadline_s:
                return self._finish_active("rejected", "height target was not reached before timeout")
        return None

    def command_velocity(self, source: str, linear_m_s: float, yaw_rad_s: float) -> bool:
        if source not in self.SOURCES or not self.can_control():
            return False
        if (self.active_action and self.active_action != "manual_drive" and
                not self._cancel_active(stop=True)):
            return False
        sent = self.motion.command_velocity(linear_m_s, yaw_rad_s)
        self.active_level = (1 if sent and
                             (linear_m_s != 0.0 or yaw_rad_s != 0.0) else 0)
        self.active_action = "manual_drive" if self.active_level else ""
        self.active_command_id = ""
        return sent

    def stop_source(self, source: str) -> None:
        if source not in self.SOURCES:
            return
        if self.active_action in ("move", "turn"):
            self.motion.cancel_action()
        else:
            self.motion.stop()
        self.active_level = 0
        self.active_action = ""
        self.active_command_id = ""

    def height_hold(self, direction: int) -> bool:
        if direction not in (-1, 0, 1):
            return False
        if direction == 0:
            sent = self.send_console("HMOVE 0") is not False
            self.active_level = 0
            self.active_action = ""
            self.active_command_id = ""
            return sent
        if not self.can_control():
            return False
        if self.active_action != "manual_height_hold":
            if not self._cancel_active(stop=True):
                return False
        if self.send_console(f"HMOVE {direction}") is False:
            return False
        self.active_level = 1 if direction else 0
        self.active_action = "manual_height_hold" if direction else ""
        self.active_command_id = ""
        return True

    def stop_all(self) -> None:
        self._cancel_active(stop=True)
        self.active_level = 0
        self.active_action = ""
        self.active_command_id = ""

    def _cancel_active(self, stop: bool) -> bool:
        stopped = True
        if self.active_action in ("move", "turn"):
            stopped = self.motion.cancel_action()
        elif self.active_action == "set_height":
            telemetry = getattr(self.motion.backend, "telemetry", {})
            try:
                height = float(telemetry.get("height_m"))
            except (TypeError, ValueError):
                height = math.nan
            if math.isfinite(height):
                stopped = self.send_console(f"SET height {height * 1000.0:.1f}") is not False
            else:
                stopped = False
        elif self.active_action == "manual_height_hold":
            stopped = self.send_console("HMOVE 0") is not False
        elif stop:
            stopped = self.motion.stop()
        if self.active_command_id:
            previous = self.last_results.get(self.active_command_id)
            if previous is not None:
                self._record(CommandResult(
                    previous.command_id, previous.source, previous.action,
                    "cancelled" if stopped else "rejected",
                    "replaced by a higher-priority or newer command" if stopped
                    else "could not cancel over serial; robot stop was not confirmed",
                    previous.effective_args))
        self.active_level = 0
        self.active_action = ""
        self.active_command_id = ""
        self.active_deadline_s = 0.0
        self.active_height_m = None
        return stopped

    def _finish_active(self, status: str, message: str) -> CommandResult:
        command_id = self.active_command_id
        previous = self.last_results.get(command_id)
        self.active_level = 0
        self.active_action = ""
        self.active_command_id = ""
        self.active_deadline_s = 0.0
        self.active_height_m = None
        if previous is None:
            return self._record(CommandResult(command_id, "system", "", status, message, {}))
        return self._record(CommandResult(
            previous.command_id, previous.source, previous.action, status,
            message, previous.effective_args))

    def _reject(self, command_id: str, source: str, action: str, message: str,
                args: Optional[dict] = None) -> CommandResult:
        return self._record(CommandResult(
            command_id, source, action, "rejected", message, args or {}))

    def _record(self, result: CommandResult) -> CommandResult:
        if result.command_id:
            self.last_results[result.command_id] = result
            while len(self.last_results) > 64:
                expired_id = next(iter(self.last_results))
                self.last_results.pop(expired_id)
                self.command_fingerprints.pop(expired_id, None)
        if self.on_result is not None:
            self.on_result(result)
        return result
