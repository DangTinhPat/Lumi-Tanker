"""Low-cost single-target association and prediction for color detections."""

import math

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


class BlueTargetTracker:
    """Track one detector-agnostic image point with a constant-velocity Kalman filter.

    Candidate association uses predicted image position, apparent-area
    consistency, and detector confidence. A short coast period avoids switching
    to an unrelated blue region during brief occlusion, while ``visible`` stays
    false whenever there is no fresh color detection.
    """

    def __init__(self, max_lost_s=0.6, base_gate_px=75.0):
        self.max_lost_s = max_lost_s
        self.base_gate_px = base_gate_px
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
        return self._result(detection, timestamp, visible=False, state="TENTATIVE")

    def _result(self, detection, timestamp, visible, state):
        # Visible outputs should follow the current color measurement directly.
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
            "area_smooth": self.area if self.active else None,
            "confidence": float(detection["confidence"]) if detection is not None else 0.0,
            "age_since_seen_s": max(0.0, timestamp - self.last_seen) if self.active else None,
        }

    def update(self, detections, timestamp):
        """Update with color-component detections and return a target state."""
        if not self.active:
            if not detections:
                return self._result(None, timestamp, visible=False, state="SEARCHING")
            best = max(detections, key=lambda d: (d["confidence"], d["area"]))
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
        # to unrelated blue regions when the target is briefly missed.
        gate = self.base_gate_px + min(predicted_speed * dt, 80.0)
        gate += min(math.sqrt(max(self.area, 0.0)) * 0.2, 25.0)

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
            detection = min(associated, key=lambda item: item[0])[1]
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
                if speed > 5000.0:
                    observed_velocity *= 5000.0 / speed
                # A constant-velocity Kalman estimate alone responds slowly to
                # sudden hand motion. Blend in measured image-space velocity.
                self.filter.statePost[2:4, 0] = (
                    0.35 * self.filter.statePost[2:4, 0] + 0.65 * observed_velocity
                )
            self.last_seen = timestamp
            self.last_measurement = (float(detection["center"][0]),
                                     float(detection["center"][1]))
            self.last_measurement_time = timestamp
            self.misses = 0
            self.hits += 1
            self.confirmed = self.confirmed or self.hits >= 2
            self.area = 0.75 * self.area + 0.25 * float(detection["area"])
            return self._result(detection, timestamp, visible=self.confirmed,
                                state="TRACKING" if self.confirmed else "TENTATIVE")

        self.misses += 1
        age = timestamp - self.last_seen
        if not self.confirmed or age > self.max_lost_s:
            self.active = False
            self.misses = 0
            self.hits = 0
            self.confirmed = False
            return self._result(None, timestamp, visible=False, state="SEARCHING")
        return self._result(None, timestamp, visible=False, state="COASTING")
