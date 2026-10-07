"""Stateful image-centering behavior between vision and base motion APIs."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

from .motion_api import MotionAPI


@dataclass(frozen=True)
class VisionTarget:
    """Vision output for one frame, relative to the image center.

    Positive ``horizontal_error_px`` means the target is to the right of image
    center. Timestamp uses ``time.monotonic()`` seconds.
    """

    horizontal_error_px: float
    image_width_px: float
    observed_at_s: float
    horizontal_fov_rad: Optional[float] = None
    depth_trend: str = "UNKNOWN"
    area_change_fraction: Optional[float] = None
    area_fraction: Optional[float] = None


@dataclass(frozen=True)
class VisionBehaviorConfig:
    """Image servo, backend yaw polarity and limits; FOV must be calibrated."""

    horizontal_fov_rad: float
    max_observation_age_s: float = 0.25
    filter_time_constant_s: float = 0.12
    center_enter_rad: float = 0.025
    center_exit_rad: float = 0.055
    yaw_gain_s_inv: float = 1.8
    max_yaw_rad_s: float = 0.35
    max_yaw_accel_rad_s2: float = 1.0
    approach_accel_m_s2: float = 0.5
    yaw_sign: float = -1.0
    approach_sector_rad: float = 0.20
    approach_speed_m_s: float = 0.0
    range_filter_time_constant_s: float = 0.35
    range_enter_error: float = 0.10
    range_exit_error: float = 0.04
    range_gain_m_s_per_log_error: float = 0.7

    def __post_init__(self) -> None:
        if (not math.isfinite(self.horizontal_fov_rad) or
                not 0.0 < self.horizontal_fov_rad < math.pi):
            raise ValueError("horizontal_fov_rad must be calibrated in (0, pi)")
        positive = {
            "max_observation_age_s": self.max_observation_age_s,
            "filter_time_constant_s": self.filter_time_constant_s,
            "center_enter_rad": self.center_enter_rad,
            "center_exit_rad": self.center_exit_rad,
            "yaw_gain_s_inv": self.yaw_gain_s_inv,
            "max_yaw_rad_s": self.max_yaw_rad_s,
            "max_yaw_accel_rad_s2": self.max_yaw_accel_rad_s2,
            "approach_accel_m_s2": self.approach_accel_m_s2,
            "approach_sector_rad": self.approach_sector_rad,
            "range_filter_time_constant_s": self.range_filter_time_constant_s,
            "range_enter_error": self.range_enter_error,
            "range_exit_error": self.range_exit_error,
            "range_gain_m_s_per_log_error": self.range_gain_m_s_per_log_error,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")
        if self.center_exit_rad <= self.center_enter_rad:
            raise ValueError("center_exit_rad must exceed center_enter_rad")
        if self.range_exit_error >= self.range_enter_error:
            raise ValueError("range_exit_error must be less than range_enter_error")
        if self.yaw_sign not in (-1.0, 1.0):
            raise ValueError("yaw_sign must be -1.0 or +1.0")
        if (not math.isfinite(self.approach_speed_m_s) or
                self.approach_speed_m_s < 0.0):
            raise ValueError("approach_speed_m_s must be finite and >= 0")
        if self.max_yaw_rad_s > 0.8 or self.approach_speed_m_s > 1.0:
            raise ValueError("vision behavior exceeds native runner hard limits")


class Vbc:
    """Convert repeated vision observations into smooth motion setpoints.

    This is a stateful image-based centering controller, not the path-tracking
    Stanley controller: a single target pixel offset provides bearing but no
    path cross-track error or target range. It filters observations, applies a
    hysteretic center gate, limits yaw acceleration, and stops on stale/lost
    targets. Call ``update`` on every behavior tick, including with ``None``
    when the detector has lost the target.
    """

    def __init__(self, motion: MotionAPI, config: VisionBehaviorConfig):
        if config.max_yaw_rad_s > motion.limits.max_yaw_rad_s:
            raise ValueError("vision yaw cap exceeds MotionAPI limit")
        if config.approach_speed_m_s > motion.limits.max_linear_m_s:
            raise ValueError("vision approach speed exceeds MotionAPI limit")
        self.motion = motion
        self.config = config
        self._filtered_bearing_rad: Optional[float] = None
        self._last_observed_at_s: Optional[float] = None
        self._last_update_at_s: Optional[float] = None
        self._yaw_command_rad_s = 0.0
        self._linear_command_m_s = 0.0
        self._filtered_area_fraction: Optional[float] = None
        self._reference_area_fraction: Optional[float] = None
        self._last_area_observed_at_s: Optional[float] = None
        self._range_motion_state = 0
        self._centered = False

    @property
    def bearing_estimate_rad(self) -> Optional[float]:
        return self._filtered_bearing_rad

    @property
    def linear_command_m_s(self) -> float:
        return self._linear_command_m_s

    @property
    def reference_area_fraction(self) -> Optional[float]:
        return self._reference_area_fraction

    @property
    def range_motion_state(self) -> int:
        """Current visual range intent: +1 forward, -1 backward, 0 hold."""
        return self._range_motion_state

    def reset(self) -> None:
        self._filtered_bearing_rad = None
        self._last_observed_at_s = None
        self._last_update_at_s = None
        self._yaw_command_rad_s = 0.0
        self._linear_command_m_s = 0.0
        self._filtered_area_fraction = None
        self._reference_area_fraction = None
        self._last_area_observed_at_s = None
        self._range_motion_state = 0
        self._centered = False

    def update(self, target: Optional[VisionTarget],
               now_s: Optional[float] = None) -> bool:
        """Update once per vision/control tick; return whether target is usable."""
        now = self._clock() if now_s is None else now_s
        if not math.isfinite(now):
            self._stop_and_reset()
            return False
        if target is None:
            self._stop_and_reset()
            return False

        values = (target.horizontal_error_px, target.image_width_px,
                  target.observed_at_s)
        if (not all(math.isfinite(value) for value in values) or
                target.image_width_px <= 0.0):
            self._stop_and_reset()
            return False
        fov = (self.config.horizontal_fov_rad
               if target.horizontal_fov_rad is None
               else target.horizontal_fov_rad)
        if not math.isfinite(fov) or not 0.0 < fov < math.pi:
            self._stop_and_reset()
            return False
        age_s = now - target.observed_at_s
        if age_s < 0.0 or age_s > self.config.max_observation_age_s:
            self._stop_and_reset()
            return False
        if abs(target.horizontal_error_px) > target.image_width_px * 0.5:
            self._stop_and_reset()
            return False

        bearing = self._pixel_error_to_bearing(
            target.horizontal_error_px, target.image_width_px, fov)
        self._filter_bearing(bearing, target.observed_at_s)
        self._update_area_measurement(target.area_fraction, target.observed_at_s)
        self._update_command(now)
        return True

    def _update_area_measurement(self, area_fraction: Optional[float],
                                 observed_at_s: float) -> None:
        if (area_fraction is None or not math.isfinite(area_fraction) or
                not 0.0 < area_fraction <= 1.0):
            return
        previous_stamp = self._last_area_observed_at_s
        if self._filtered_area_fraction is None or previous_stamp is None:
            self._filtered_area_fraction = area_fraction
        elif observed_at_s > previous_stamp:
            dt_s = observed_at_s - previous_stamp
            alpha = dt_s / (self.config.range_filter_time_constant_s + dt_s)
            self._filtered_area_fraction += alpha * (
                area_fraction - self._filtered_area_fraction)
        self._last_area_observed_at_s = max(
            observed_at_s, previous_stamp or observed_at_s)
        if (self._reference_area_fraction is None and
                abs(self._filtered_bearing_rad or 0.0) <=
                self.config.approach_sector_rad):
            self._reference_area_fraction = self._filtered_area_fraction

    def _pixel_error_to_bearing(self, error_px: float, width_px: float,
                                horizontal_fov_rad: float) -> float:
        # Pinhole-camera relation from normalized image coordinate and calibrated
        # horizontal FOV. Positive pixel offset maps to positive (right) yaw.
        normalized = 2.0 * error_px / width_px
        return math.atan(normalized * math.tan(horizontal_fov_rad / 2.0))

    def _filter_bearing(self, measured: float, observed_at_s: float) -> None:
        previous_stamp = self._last_observed_at_s
        if self._filtered_bearing_rad is None or previous_stamp is None:
            self._filtered_bearing_rad = measured
        elif observed_at_s > previous_stamp:
            dt_s = observed_at_s - previous_stamp
            alpha = dt_s / (self.config.filter_time_constant_s + dt_s)
            self._filtered_bearing_rad += alpha * (
                measured - self._filtered_bearing_rad)
        self._last_observed_at_s = max(
            observed_at_s, previous_stamp or observed_at_s)

    def _update_command(self, now_s: float) -> None:
        cfg = self.config
        bearing = self._filtered_bearing_rad or 0.0
        magnitude = abs(bearing)
        if self._centered:
            if magnitude >= cfg.center_exit_rad:
                self._centered = False
        elif magnitude <= cfg.center_enter_rad:
            self._centered = True

        # Positive camera bearing means the target is right of image center.
        # The backend-specific sign maps that bearing into the robot's yaw
        # convention (native MuJoCo defaults to -1; ESP mode must be explicit).
        desired_yaw = 0.0 if self._centered else max(
            -cfg.max_yaw_rad_s,
            min(cfg.max_yaw_rad_s,
                cfg.yaw_sign * cfg.yaw_gain_s_inv * bearing))
        dt_s = (0.0 if self._last_update_at_s is None else
                max(0.0, now_s - self._last_update_at_s))
        max_yaw_step = cfg.max_yaw_accel_rad_s2 * dt_s
        self._yaw_command_rad_s += max(
            -max_yaw_step,
            min(max_yaw_step, desired_yaw - self._yaw_command_rad_s))
        self._last_update_at_s = now_s

        # Hold the apparent target size captured when centered. A pure approach /
        # recede trend command would feed the robot's own motion back into the
        # trend label and cause it to hunt forward and backward.
        range_error = 0.0
        if (self._reference_area_fraction is not None and
                self._filtered_area_fraction is not None):
            range_error = 0.5 * math.log(
                self._reference_area_fraction / self._filtered_area_fraction)
        if magnitude > cfg.approach_sector_rad:
            self._range_motion_state = 0
        elif self._range_motion_state == 0:
            if range_error >= cfg.range_enter_error:
                self._range_motion_state = 1
            elif range_error <= -cfg.range_enter_error:
                self._range_motion_state = -1
        elif self._range_motion_state > 0:
            if range_error <= -cfg.range_enter_error:
                self._range_motion_state = -1
            elif range_error <= cfg.range_exit_error:
                self._range_motion_state = 0
        else:
            if range_error >= cfg.range_enter_error:
                self._range_motion_state = 1
            elif range_error >= -cfg.range_exit_error:
                self._range_motion_state = 0

        linear = 0.0
        if self._range_motion_state > 0:
            linear = min(cfg.approach_speed_m_s,
                         cfg.range_gain_m_s_per_log_error *
                         max(range_error - cfg.range_exit_error, 0.0))
        elif self._range_motion_state < 0:
            linear = -min(cfg.approach_speed_m_s,
                          cfg.range_gain_m_s_per_log_error *
                          max(-range_error - cfg.range_exit_error, 0.0))
        max_linear_step = cfg.approach_accel_m_s2 * dt_s
        self._linear_command_m_s += max(
            -max_linear_step,
            min(max_linear_step, linear - self._linear_command_m_s))
        self.motion.command_velocity(self._linear_command_m_s,
                                     self._yaw_command_rad_s)

    def _stop_and_reset(self) -> None:
        self.motion.stop()
        self.reset()

    @staticmethod
    def _clock() -> float:
        import time
        return time.monotonic()
