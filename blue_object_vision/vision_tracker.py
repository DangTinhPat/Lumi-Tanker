"""Low-cost single-target association and prediction for detector results."""

import math
from collections import deque

import cv2
import numpy as np


class ApparentAreaTrendFilter:
    """Filter noisy size-change estimates into a stable depth trend."""

    def __init__(self, time_constant_s=0.30, enter_fraction=0.14,
                 release_fraction=0.05):
        self.time_constant_s = time_constant_s
        self.enter_fraction = enter_fraction
        self.release_fraction = release_fraction
        self.reset()

    def reset(self):
        self.filtered_change = None
        self.last_timestamp = None
        self.state = "STABLE"

    def update(self, area_change_fraction, timestamp):
        if not math.isfinite(area_change_fraction) or not math.isfinite(timestamp):
            self.reset()
            return "MEASURING"
        if self.filtered_change is None or self.last_timestamp is None:
            self.filtered_change = area_change_fraction
        elif timestamp > self.last_timestamp:
            dt_s = timestamp - self.last_timestamp
            alpha = dt_s / (self.time_constant_s + dt_s)
            self.filtered_change += alpha * (area_change_fraction - self.filtered_change)
        self.last_timestamp = timestamp

        value = self.filtered_change
        if self.state == "APPROACHING":
            if value <= -self.enter_fraction:
                self.state = "RECEDING"
            elif value <= self.release_fraction:
                self.state = "STABLE"
        elif self.state == "RECEDING":
            if value >= self.enter_fraction:
                self.state = "APPROACHING"
            elif value >= -self.release_fraction:
                self.state = "STABLE"
        elif value >= self.enter_fraction:
            self.state = "APPROACHING"
        elif value <= -self.enter_fraction:
            self.state = "RECEDING"
        return self.state


class OneEuroFilter:
    """Smooth one value while raising the cutoff during fast motion."""

    def __init__(self, min_cutoff=1.0, beta=0.005, derivative_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.derivative_cutoff = derivative_cutoff
        self.value = None
        self.raw_value = None
        self.derivative = 0.0
        self.timestamp = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def update(self, value, timestamp):
        if self.value is None or self.timestamp is None:
            self.value = float(value)
            self.raw_value = float(value)
            self.timestamp = timestamp
            return self.value

        dt = max(timestamp - self.timestamp, 1e-6)
        derivative = (value - self.raw_value) / dt
        alpha_d = self._alpha(self.derivative_cutoff, dt)
        self.derivative += alpha_d * (derivative - self.derivative)
        cutoff = self.min_cutoff + self.beta * abs(self.derivative)
        alpha = self._alpha(cutoff, dt)
        self.value += alpha * (value - self.value)
        self.raw_value = float(value)
        self.timestamp = timestamp
        return self.value


class SingleTargetTracker:
    """Track one detection with a Kalman center and an adaptive One Euro box filter.

    Candidate association uses predicted image position, apparent-area
    consistency, and detector confidence. The One Euro filter suppresses box
    jitter at low speed and raises its cutoff during fast motion. A short coast
    period avoids switching to another detection during occlusion, while
    ``visible`` stays true only during a bounded 0.15-second prediction window.
    """

    def __init__(self, max_lost_s=0.8, max_coast_s=0.15,
                 base_gate_px=75.0):
        self.max_lost_s = max_lost_s
        self.max_coast_s = max_coast_s
        self.base_gate_px = base_gate_px
        self.track_confidence = 0.08
        self.filter = cv2.KalmanFilter(4, 2, 0, cv2.CV_32F)
        self.filter.measurementMatrix = np.array(
            [[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float32
        )
        self.filter.measurementNoiseCov = np.eye(2, dtype=np.float32) * 36.0
        self.filter.processNoiseCov = np.diag([2.0, 2.0, 120.0, 120.0]).astype(np.float32)
        self.filter.errorCovPost = np.diag([20.0, 20.0, 400.0, 400.0]).astype(np.float32)
        self.active = False
        self.track_id = 0
        self.last_seen = 0.0
        self.last_update = 0.0
        self.area = 0.0
        self.misses = 0
        self.hits = 0
        self.confirmed = False
        self.last_measurement = None
        self.last_measurement_time = 0.0
        self.box_filters = None
        self.last_detection = None
        self.box_history = deque(maxlen=3)

    @staticmethod
    def _box_values(detection):
        x, y, width, height = detection["box"]
        return (x + width * 0.5, y + height * 0.5,
                math.log(max(width, 1)), math.log(max(height, 1)))

    def _filter_box(self, detection, timestamp):
        values = self._box_values(detection)
        self.box_history.append(values)
        # A three-detection median rejects a one-frame box jump before the
        # adaptive low-pass filter, without averaging it into the target.
        values = np.median(np.asarray(self.box_history), axis=0)
        if self.box_filters is None:
            # Use a responsive cutoff for both centering and range cues; the
            # median above still rejects isolated detector jumps.
            self.box_filters = [
                OneEuroFilter(min_cutoff=2.0),
                OneEuroFilter(min_cutoff=2.0),
                # Box dimensions drive the approach/recede controller. A 1 Hz
                # cutoff made range response noticeably slower than centering;
                # the three-frame median still removes isolated detector jumps.
                OneEuroFilter(min_cutoff=2.0),
                OneEuroFilter(min_cutoff=2.0),
            ]
        filtered = [flt.update(value, timestamp)
                    for flt, value in zip(self.box_filters, values)]
        center_x, center_y, log_width, log_height = filtered
        width, height = max(1, round(math.exp(log_width))), max(1, round(math.exp(log_height)))
        x, y = round(center_x - width * 0.5), round(center_y - height * 0.5)
        result = detection.copy()
        result["box"] = (x, y, width, height)
        result["center"] = (round(center_x), round(center_y))
        result["area"] = width * height
        return result

    def _start(self, detection, timestamp):
        x, y = detection["center"]
        self.filter.statePost = np.array([[x], [y], [0.0], [0.0]], dtype=np.float32)
        self.filter.statePre = self.filter.statePost.copy()
        self.filter.errorCovPost = np.diag([20.0, 20.0, 400.0, 400.0]).astype(np.float32)
        self.track_id += 1
        self.active = True
        self.last_seen = self.last_update = timestamp
        self.area = float(detection["area"])
        self.misses = 0
        self.hits = 1
        self.confirmed = False
        self.last_measurement = (float(x), float(y))
        self.last_measurement_time = timestamp
        self.box_filters = None
        self.box_history.clear()
        detection = self._filter_box(detection, timestamp)
        self.last_detection = detection
        return self._result(detection, timestamp, visible=False, state="TENTATIVE")

    def _result(self, detection, timestamp, visible, state):
        # Visible outputs should follow the current detector measurement directly.
        # Keep the Kalman position for prediction/association and for COASTING.
        point = (detection["center"] if visible and detection is not None
                 else self.filter.statePost[:2, 0])
        return {
            "track_id": self.track_id,
            "state": state,
            "visible": visible,
            "center": (int(point[0]), int(point[1])) if visible and detection is not None
            else (int(round(float(point[0]))), int(round(float(point[1])))),
            "detection": detection,
            "area": float(detection["area"]) if detection is not None else None,
            "area_smooth": (float(detection["area"]) if detection is not None
                            else self.area if self.active else None),
            "confidence": float(detection["confidence"]) if detection is not None else 0.0,
            "age_since_seen_s": max(0.0, timestamp - self.last_seen) if self.active else None,
        }

    def update(self, detections, timestamp):
        """Update with detector results and return a target state."""
        if not self.active:
            if not detections:
                return self._result(None, timestamp, visible=False, state="SEARCHING")
            # Start only from a stronger detection, choosing the largest person
            # box so a distant background person does not steal the target.
            eligible = [d for d in detections
                        if d["confidence"] >= self.track_confidence]
            if not eligible:
                return self._result(None, timestamp, visible=False, state="SEARCHING")
            best = max(eligible, key=lambda d: (d["area"], d["confidence"]))
            return self._start(best, timestamp)

        dt = min(max(timestamp - self.last_update, 1.0 / 120.0), 0.25)
        self.filter.transitionMatrix = np.array(
            [[1, 0, dt, 0], [0, 1, 0, dt],
             [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float32
        )
        prediction = self.filter.predict()
        predicted_center = prediction[:2, 0]
        predicted_speed = float(np.linalg.norm(prediction[2:4, 0]))
        # Keep association local: an overly generous gate lets the tracker jump
        # to unrelated detections when the target is briefly missed.
        # Bound how far one uncertain detection can pull a stable track. The
        # previous wide gate admitted occasional person-box jumps as velocity.
        gate = self.base_gate_px + min(predicted_speed * dt, 35.0)
        gate += min(math.sqrt(max(self.area, 0.0)) * 0.12, 15.0)

        associated = []
        for detection in detections:
            dx = detection["center"][0] - float(predicted_center[0])
            dy = detection["center"][1] - float(predicted_center[1])
            distance = math.hypot(dx, dy)
            if distance > gate:
                continue
            area_ratio = max(float(detection["area"]), 1.0) / max(self.area, 1.0)
            area_penalty = min(abs(math.log(area_ratio)), 2.0)
            score = distance / gate + 0.55 * area_penalty - 0.10 * detection["confidence"]
            associated.append((score, detection))

        self.last_update = timestamp
        if associated:
            # ByteTrack-style two-stage association: detections above the
            # calibrated track threshold get first choice; lower-confidence
            # boxes may preserve an existing track but cannot start one.
            high_confidence = [item for item in associated
                               if item[1]["confidence"] >= self.track_confidence]
            detection = min(high_confidence or associated, key=lambda item: item[0])[1]
            measurement = np.array([[detection["center"][0]], [detection["center"][1]]], dtype=np.float32)
            self.filter.correct(measurement)
            measured_at = self.last_measurement_time
            measured_center = self.last_measurement
            elapsed = timestamp - measured_at
            if measured_center is not None and elapsed > 0.0:
                observed_velocity = np.array([
                    (detection["center"][0] - measured_center[0]) / elapsed,
                    (detection["center"][1] - measured_center[1]) / elapsed,
                ], dtype=np.float32)
                speed = float(np.linalg.norm(observed_velocity))
                if speed > 1000.0:
                    observed_velocity *= 1000.0 / speed
                # A constant-velocity Kalman estimate alone responds slowly to
                # sudden hand motion. Blend in measured image-space velocity.
                self.filter.statePost[2:4, 0] = (
                    0.75 * self.filter.statePost[2:4, 0] + 0.25 * observed_velocity
                )
            self.last_seen = timestamp
            self.last_measurement = (float(detection["center"][0]),
                                     float(detection["center"][1]))
            self.last_measurement_time = timestamp
            self.misses = 0
            self.hits += 1
            self.confirmed = self.confirmed or self.hits >= 2
            self.area = 0.75 * self.area + 0.25 * float(detection["area"])
            detection = self._filter_box(detection, timestamp)
            self.last_detection = detection
            return self._result(detection, timestamp, visible=self.confirmed,
                                state="TRACKING" if self.confirmed else "TENTATIVE")

        # Treat a missed inference as a missing measurement, not as a new
        # stationary state. This lets the Kalman estimate advance through gaps.
        self.filter.statePost = prediction.copy()
        self.filter.errorCovPost = self.filter.errorCovPre.copy()
        self.misses += 1
        age = timestamp - self.last_seen
        if age > self.max_lost_s:
            self.active = False
            self.misses = 0
            self.hits = 0
            self.confirmed = False
            return self._result(None, timestamp, visible=False, state="SEARCHING")
        if not self.confirmed:
            # A single YOLO miss should not discard a tentative track before its
            # second matching detection can confirm it.
            return self._result(None, timestamp, visible=False, state="TENTATIVE")
        if age <= self.max_coast_s and self.last_detection is not None:
            x_center, y_center = self.filter.statePost[:2, 0]
            _, _, width, height = self.last_detection["box"]
            detection = self.last_detection.copy()
            detection["box"] = (
                round(float(x_center) - width * 0.5),
                round(float(y_center) - height * 0.5), width, height)
            detection["center"] = (round(float(x_center)), round(float(y_center)))
            detection["confidence"] *= max(0.0, 1.0 - age / self.max_coast_s)
            return self._result(detection, timestamp, visible=True, state="COASTING")
        return self._result(None, timestamp, visible=False, state="COASTING")
