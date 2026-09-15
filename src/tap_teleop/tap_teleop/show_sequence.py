"""Finite CAD pose performance, with explicit hold/resume and no ROS I/O."""
from copy import deepcopy
import math
from typing import NamedTuple

import numpy as np

from .pose_curves import JOINTS, sample


class SequenceResult(NamedTuple):
    targets: tuple[float, float, float, float]
    finished: bool
    stage: str


def _time(value):
    value = float(value)
    if not math.isfinite(value) or value < 0:
        raise ValueError('Elapsed time must be finite and nonnegative')
    return value


def quintic(value):
    """Unit interval blend with zero first/second endpoint derivatives."""
    return value ** 3 * (10 + value * (-15 + 6 * value))


class ShowSequence:
    """Evaluate elapsed wall time in seconds from a caller-owned epoch.

    Defaults: initial neutral dwell 2 s, then six 3 s transitions each
    followed by a 2 s dwell (32 s total). Amplitude must be in [0, 1];
    the caller owns any stricter experimental cap. Only curve amplitude
    changes, never an independent interpolation of the four joint targets.

    stop(elapsed) holds that exact pose indefinitely. resume(elapsed) is
    explicit and excludes held wall time from the performance clock. Use
    monotonically increasing elapsed values during playback; evaluate is
    otherwise deterministic and permits offline sampling in any order.
    A completed sequence never loops or restarts; construct a new instance.
    """

    def __init__(self, curves, amplitude=.5, transition_s=3., dwell_s=2.):
        self.amplitude = float(amplitude)
        if not math.isfinite(self.amplitude) or not 0 <= self.amplitude <= 1:
            raise ValueError('Amplitude must be finite and in [0, 1]')
        self.transition_s, self.dwell_s = _time(transition_s), _time(dwell_s)
        if self.transition_s == 0 or self.dwell_s == 0:
            raise ValueError('Transition and dwell durations must be positive')
        self.curves = deepcopy(curves)
        if tuple(curves['joint_order']) != JOINTS:
            raise ValueError('Unexpected native joint order')
        self.neutral = tuple(float(v) for v in curves['rest_pose'])
        self.bounds = np.array([curves['hip_bounds'], curves['knee_bounds']] * 2)
        if self.bounds.shape != (4, 2) or not np.isfinite(self.bounds).all():
            raise ValueError('Invalid joint bounds')
        if np.any(self.bounds[:, 0] > self.bounds[:, 1]):
            raise ValueError('Reversed joint bounds')
        self._check(self.neutral)
        for mode in ('squat', 'roll_left', 'roll_right'):
            table = np.asarray(curves['tables'][mode], dtype=float)
            if table.ndim != 2 or table.shape[1] != 4 or len(table) < 2:
                raise ValueError('Invalid pose curve table')
            for pose in table:
                self._check(pose)
            if not np.allclose(table[0], self.neutral, atol=1e-10, rtol=0):
                raise ValueError('Pose curves must share the neutral endpoint')
        self._stages = [('initial_neutral', None, 0., 0., self.dwell_s)]
        for mode in ('squat', 'roll_left', 'roll_right'):
            self._stages.extend([
                (mode + '_out', mode, 0., self.amplitude, self.transition_s),
                (mode + '_hold', mode, self.amplitude, self.amplitude, self.dwell_s),
                (mode + '_return', mode, self.amplitude, 0., self.transition_s),
                (mode + '_neutral', None, 0., 0., self.dwell_s),
            ])
        self.duration_s = sum(stage[4] for stage in self._stages)
        if not math.isfinite(self.duration_s):
            raise ValueError('Sequence duration overflow')
        self.stage_names = tuple(s[0] for s in self._stages) + ('finished',)
        self._held_at = None
        self._held_result = None
        self._paused_s = 0.

    def _check(self, pose):
        pose = np.asarray(pose)
        if (pose.shape != (4,) or not np.isfinite(pose).all()
                or np.any(pose < self.bounds[:, 0])
                or np.any(pose > self.bounds[:, 1])):
            raise ValueError('Pose outside finite native joint bounds')

    @property
    def stopped(self):
        return self._held_at is not None

    def evaluate(self, elapsed):
        elapsed = _time(elapsed)
        if self.stopped:
            return self._held_result
        t = max(0., elapsed - self._paused_s)
        for name, mode, start, end, duration in self._stages:
            if t < duration:
                if mode is None or (start == 0 and end == 0):
                    pose = self.neutral
                else:
                    amount = start + (end - start) * quintic(t / duration)
                    pose = tuple(sample(self.curves, mode, amount))
                self._check(pose)
                return SequenceResult(pose, False, name)
            t -= duration
        return SequenceResult(self.neutral, True, 'finished')

    def stop(self, elapsed):
        elapsed = _time(elapsed)
        if not self.stopped:
            self._held_result = self.evaluate(elapsed)
            self._held_at = elapsed
        return self._held_result

    def resume(self, elapsed):
        elapsed = _time(elapsed)
        if self.stopped:
            if elapsed < self._held_at:
                raise ValueError('Resume time precedes stop time')
            self._paused_s += elapsed - self._held_at
            self._held_at = None
            self._held_result = None
        return self.evaluate(elapsed)
