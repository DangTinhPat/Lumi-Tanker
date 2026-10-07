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
    max_observation_age_s: float = 0.18
    filter_time_constant_s: float = 0.12
    center_enter_rad: float = 0.025
    center_exit_rad: float = 0.055
    center_reacquire_confirm_s: float = 0.20
    yaw_gain_s_inv: float = 1.8
    yaw_derivative_time_s: float = 0.18
    bearing_rate_filter_time_constant_s: float = 0.12
    max_bearing_rate_rad_s: float = 1.5
    max_yaw_rad_s: float = 0.35
    max_yaw_accel_rad_s2: float = 1.0
    approach_accel_m_s2: float = 0.5
    yaw_sign: float = -1.0
    approach_sector_rad: float = 0.20
    approach_speed_m_s: float = 0.0
    range_filter_time_constant_s: float = 0.08
    range_rate_filter_time_constant_s: float = 0.10
    range_predict_time_s: float = 0.18
    max_range_rate_log_s: float = 0.8
    range_enter_error: float = 0.035
    range_exit_error: float = 0.012
    range_confirm_s: float = 0.08
    range_stop_dwell_s: float = 0.10
    range_min_speed_m_s: float = 0.0
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
            "center_reacquire_confirm_s": self.center_reacquire_confirm_s,
            "yaw_gain_s_inv": self.yaw_gain_s_inv,
            "yaw_derivative_time_s": self.yaw_derivative_time_s,
            "bearing_rate_filter_time_constant_s": self.bearing_rate_filter_time_constant_s,
            "max_bearing_rate_rad_s": self.max_bearing_rate_rad_s,
            "max_yaw_rad_s": self.max_yaw_rad_s,
            "max_yaw_accel_rad_s2": self.max_yaw_accel_rad_s2,
            "approach_accel_m_s2": self.approach_accel_m_s2,
            "approach_sector_rad": self.approach_sector_rad,
            "range_filter_time_constant_s": self.range_filter_time_constant_s,
            "range_rate_filter_time_constant_s": self.range_rate_filter_time_constant_s,
            "range_predict_time_s": self.range_predict_time_s,
            "max_range_rate_log_s": self.max_range_rate_log_s,
            "range_enter_error": self.range_enter_error,
            "range_exit_error": self.range_exit_error,
            "range_confirm_s": self.range_confirm_s,
            "range_gain_m_s_per_log_error": self.range_gain_m_s_per_log_error,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")
        if self.center_exit_rad <= self.center_enter_rad:
            raise ValueError("center_exit_rad must exceed center_enter_rad")
        if self.range_exit_error >= self.range_enter_error:
            raise ValueError("range_exit_error must be less than range_enter_error")
        if (not math.isfinite(self.range_stop_dwell_s) or
                self.range_stop_dwell_s < 0.0):
            raise ValueError("range_stop_dwell_s must be finite and >= 0")
        if self.yaw_sign not in (-1.0, 1.0):
            raise ValueError("yaw_sign must be -1.0 or +1.0")
        if (not math.isfinite(self.approach_speed_m_s) or
                self.approach_speed_m_s < 0.0):
            raise ValueError("approach_speed_m_s must be finite and >= 0")
        if (not math.isfinite(self.range_min_speed_m_s) or
                self.range_min_speed_m_s < 0.0 or
                self.range_min_speed_m_s > self.approach_speed_m_s):
            raise ValueError(
                "range_min_speed_m_s must be between zero and approach_speed_m_s")
        if self.max_yaw_rad_s > 0.8 or self.approach_speed_m_s > 1.0:
            raise ValueError("vision behavior exceeds native runner hard limits")


class Vbc:
    """Convert repeated vision observations into smooth motion setpoints.

    This is a stateful image-based centering controller, not the path-tracking
    Stanley controller: a single target pixel offset provides bearing but no
    path cross-track error or target range. Its yaw controller uses
    ``yaw_sign * Kp * (e + Td * de/dt)``: proportional bearing correction plus
    filtered bearing-rate damping. The hysteretic center band removes the
    position term near center while keeping derivative braking active. It also
    uses a confirmed approach/hold/recede state machine with a stop dwell before
    reversals, limits command acceleration, and stops on stale/lost targets.
    Call ``update`` on every behavior tick, including with ``None`` when the
    detector has lost the target.
    """

    def __init__(self, motion: MotionAPI, config: VisionBehaviorConfig):
        if config.max_yaw_rad_s > motion.limits.max_yaw_rad_s:
            raise ValueError("vision yaw cap exceeds MotionAPI limit")
        if config.approach_speed_m_s > motion.limits.max_linear_m_s:
            raise ValueError("vision approach speed exceeds MotionAPI limit")
        self.motion = motion
        self.config = config
        self._filtered_bearing_rad: Optional[float] = None
        self._bearing_rate_rad_s = 0.0
        self._last_observed_at_s: Optional[float] = None
        self._last_update_at_s: Optional[float] = None
        self._yaw_command_rad_s = 0.0
        self._linear_command_m_s = 0.0
        self._filtered_log_area: Optional[float] = None
        self._reference_log_area: Optional[float] = None
        self._last_area_observed_at_s: Optional[float] = None
        self._range_error = 0.0
        self._range_rate_log_s = 0.0
        self._last_range_error_at_s: Optional[float] = None
        self._range_motion_state = 0
        self._range_candidate_state = 0
        self._range_candidate_since_s: Optional[float] = None
        self._range_hold_until_s = 0.0
        self._centered = False
        self._center_exit_candidate_since_s: Optional[float] = None

    @property
    def bearing_estimate_rad(self) -> Optional[float]:
        return self._filtered_bearing_rad

    @property
    def linear_command_m_s(self) -> float:
        return self._linear_command_m_s

    @property
    def yaw_command_rad_s(self) -> float:
        return self._yaw_command_rad_s

    @property
    def bearing_rate_rad_s(self) -> float:
        """Filtered image bearing velocity, updated only on a fresh frame."""
        return self._bearing_rate_rad_s

    @property
    def reference_area_fraction(self) -> Optional[float]:
        if self._reference_log_area is None:
            return None
        return math.exp(self._reference_log_area)

    @property
    def range_error_log(self) -> float:
        """Relative log-distance error; positive means the target is too far."""
        return self._range_error

    @property
    def range_rate_log_s(self) -> float:
        """Filtered rate of relative log-distance error per second."""
        return self._range_rate_log_s

    @property
    def predicted_range_error_log(self) -> float:
        return (self._range_error +
                self.config.range_predict_time_s * self._range_rate_log_s)

    @property
    def range_motion_state(self) -> int:
        """Current visual range intent: +1 forward, -1 backward, 0 hold."""
        return self._range_motion_state

    def reset(self) -> None:
        self._filtered_bearing_rad = None
        self._bearing_rate_rad_s = 0.0
        self._last_observed_at_s = None
        self._last_update_at_s = None
        self._yaw_command_rad_s = 0.0
        self._linear_command_m_s = 0.0
        self._filtered_log_area = None
        self._reference_log_area = None
        self._last_area_observed_at_s = None
        self._range_error = 0.0
        self._range_rate_log_s = 0.0
        self._last_range_error_at_s = None
        self._range_motion_state = 0
        self._range_candidate_state = 0
        self._range_candidate_since_s = None
        self._range_hold_until_s = 0.0
        self._centered = False
        self._center_exit_candidate_since_s = None

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
        range_sample_updated = self._update_area_measurement(
            target.area_fraction, target.observed_at_s)
        self._update_command(now, target.observed_at_s, range_sample_updated)
        return True

    def _update_area_measurement(self, area_fraction: Optional[float],
                                 observed_at_s: float) -> bool:
        if (area_fraction is None or not math.isfinite(area_fraction) or
                not 0.0 < area_fraction <= 1.0):
            return False
        previous_stamp = self._last_area_observed_at_s
        if previous_stamp is not None and observed_at_s <= previous_stamp:
            return False

        # The vision tracker already applies a three-frame median to box size.
        # Avoid another median here: it delays the range response a second time.
        measured_log_area = math.log(area_fraction)
        if self._filtered_log_area is None or previous_stamp is None:
            self._filtered_log_area = measured_log_area
        else:
            dt_s = observed_at_s - previous_stamp
            alpha = dt_s / (self.config.range_filter_time_constant_s + dt_s)
            self._filtered_log_area += alpha * (
                measured_log_area - self._filtered_log_area)
        self._last_area_observed_at_s = max(
            observed_at_s, previous_stamp or observed_at_s)
        if (self._reference_log_area is None and
                abs(self._filtered_bearing_rad or 0.0) <=
                self.config.approach_sector_rad):
            self._reference_log_area = self._filtered_log_area
        if self._reference_log_area is not None:
            range_error = 0.5 * (
                self._reference_log_area - self._filtered_log_area)
            if self._last_range_error_at_s is not None:
                dt_s = observed_at_s - self._last_range_error_at_s
                if dt_s > 0.0:
                    measured_rate = (range_error - self._range_error) / dt_s
                    measured_rate = max(
                        -self.config.max_range_rate_log_s,
                        min(self.config.max_range_rate_log_s, measured_rate))
                    alpha = dt_s / (
                        self.config.range_rate_filter_time_constant_s + dt_s)
                    self._range_rate_log_s += alpha * (
                        measured_rate - self._range_rate_log_s)
            else:
                self._range_rate_log_s = 0.0
            self._range_error = range_error
            self._last_range_error_at_s = observed_at_s
        return True

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
            previous_bearing = self._filtered_bearing_rad
            alpha = dt_s / (self.config.filter_time_constant_s + dt_s)
            self._filtered_bearing_rad += alpha * (
                measured - self._filtered_bearing_rad)
            # Estimate image-space angular velocity from distinct camera
            # frames. Repeated GUI ticks of the same JSON record must not be
            # interpreted as extra measurements or as zero velocity samples.
            if dt_s <= self.config.max_observation_age_s:
                raw_rate = (self._filtered_bearing_rad - previous_bearing) / dt_s
                raw_rate = max(-self.config.max_bearing_rate_rad_s,
                               min(self.config.max_bearing_rate_rad_s, raw_rate))
                rate_alpha = dt_s / (
                    self.config.bearing_rate_filter_time_constant_s + dt_s)
                self._bearing_rate_rad_s += rate_alpha * (
                    raw_rate - self._bearing_rate_rad_s)
            else:
                self._bearing_rate_rad_s = 0.0
        self._last_observed_at_s = max(
            observed_at_s, previous_stamp or observed_at_s)

    def _update_command(self, now_s: float, observed_at_s: float,
                        range_sample_updated: bool) -> None:
        cfg = self.config
        bearing = self._filtered_bearing_rad or 0.0
        magnitude = abs(bearing)
        if self._centered:
            if magnitude >= cfg.center_exit_rad:
                if self._center_exit_candidate_since_s is None:
                    self._center_exit_candidate_since_s = now_s
                elif (now_s - self._center_exit_candidate_since_s >=
                      cfg.center_reacquire_confirm_s):
                    self._centered = False
                    self._center_exit_candidate_since_s = None
            else:
                # Ignore brief bbox excursions while the target is centered.
                self._center_exit_candidate_since_s = None
        elif magnitude <= cfg.center_enter_rad:
            self._centered = True
            self._center_exit_candidate_since_s = None

        # Image-based PD servo. P turns toward the target; D reduces yaw while
        # the measured bearing is already closing and can command counter-yaw
        # inside the center band to brake body momentum. D is refreshed only
        # from distinct image timestamps in _filter_bearing.
        active_error = math.copysign(
            max(magnitude - cfg.center_enter_rad, 0.0), bearing)
        position_error = 0.0 if self._centered else active_error
        pd_error = (position_error +
                    cfg.yaw_derivative_time_s * self._bearing_rate_rad_s)
        desired_yaw = max(
            -cfg.max_yaw_rad_s,
            min(cfg.max_yaw_rad_s,
                cfg.yaw_sign * cfg.yaw_gain_s_inv * pd_error))
        dt_s = (0.0 if self._last_update_at_s is None else
                max(0.0, now_s - self._last_update_at_s))
        max_yaw_step = cfg.max_yaw_accel_rad_s2 * dt_s
        self._yaw_command_rad_s += max(
            -max_yaw_step,
            min(max_yaw_step, desired_yaw - self._yaw_command_rad_s))
        self._last_update_at_s = now_s

        # Hold the apparent target size captured when centered. Predict a short
        # distance error from filtered log-area rate so motion starts before the
        # person has moved a large fraction of the target distance.
        predicted_range_error = self.predicted_range_error_log
        if range_sample_updated:
            if (magnitude > cfg.approach_sector_rad or
                    self._reference_log_area is None):
                self._clear_range_candidate()
                if self._range_motion_state != 0:
                    self._stop_range_motion(now_s)
            else:
                self._update_range_state(
                    predicted_range_error, now_s, observed_at_s)

        linear = 0.0
        if self._range_motion_state > 0:
            linear = min(
                cfg.approach_speed_m_s,
                cfg.range_min_speed_m_s + cfg.range_gain_m_s_per_log_error *
                max(abs(predicted_range_error) - cfg.range_enter_error, 0.0))
        elif self._range_motion_state < 0:
            linear = -min(
                cfg.approach_speed_m_s,
                cfg.range_min_speed_m_s + cfg.range_gain_m_s_per_log_error *
                max(abs(predicted_range_error) - cfg.range_enter_error, 0.0))
        max_linear_step = cfg.approach_accel_m_s2 * dt_s
        self._linear_command_m_s += max(
            -max_linear_step,
            min(max_linear_step, linear - self._linear_command_m_s))
        self.motion.command_velocity(self._linear_command_m_s,
                                     self._yaw_command_rad_s)

    def _update_range_state(self, predicted_range_error: float, now_s: float,
                            observed_at_s: float) -> None:
        cfg = self.config
        active = self._range_motion_state
        # Stop in the target-size band. A reverse command can only start after
        # braking, a settle dwell, and a fresh confirmed crossing of the other
        # enter threshold.
        if active > 0 and predicted_range_error <= cfg.range_exit_error:
            self._stop_range_motion(now_s)
            return
        if active < 0 and predicted_range_error >= -cfg.range_exit_error:
            self._stop_range_motion(now_s)
            return
        if active != 0:
            return

        candidate = (1 if predicted_range_error >= cfg.range_enter_error else
                     -1 if predicted_range_error <= -cfg.range_enter_error else 0)
        if candidate == 0:
            self._clear_range_candidate()
            return
        if candidate != self._range_candidate_state:
            self._range_candidate_state = candidate
            self._range_candidate_since_s = observed_at_s
            return
        if self._range_candidate_since_s is None:
            self._range_candidate_since_s = observed_at_s
            return
        confirmed = observed_at_s - self._range_candidate_since_s >= cfg.range_confirm_s
        if confirmed and now_s >= self._range_hold_until_s:
            self._range_motion_state = candidate
            self._clear_range_candidate()

    def _stop_range_motion(self, now_s: float) -> None:
        braking_s = abs(self._linear_command_m_s) / max(
            self.config.approach_accel_m_s2, 1.0e-3)
        self._range_hold_until_s = max(
            self._range_hold_until_s,
            now_s + braking_s + self.config.range_stop_dwell_s)
        self._range_motion_state = 0
        self._clear_range_candidate()

    def _clear_range_candidate(self) -> None:
        self._range_candidate_state = 0
        self._range_candidate_since_s = None

    def _stop_and_reset(self) -> None:
        self.motion.stop()
        self.reset()

    @staticmethod
    def _clock() -> float:
        import time
        return time.monotonic()
