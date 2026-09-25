#!/usr/bin/env python3
"""Check TAP ESP32 MPU6500, estimator and Dabble bring-up logs.

Usage:
  python3 tools/check_imu_ble_log.py serial.log
  pio device monitor -e esp32-s3-poc | tee serial.log
  python3 tools/check_imu_ble_log.py --require-filter --require-ble serial.log
"""

from __future__ import annotations

import argparse
import math
import re
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, TextIO


WHO_RE = re.compile(r"WHO_AM_I\s*=\s*0x([0-9a-fA-F]+)\s*\((PASS|FAIL)\)")
BLE_RE = re.compile(r"\bBLE\s+(connected|disconnected)\b", re.IGNORECASE)
IMU_RE = re.compile(
    r"IMU\s+"
    r"ax=(?P<ax>[+-]?\d+(?:\.\d+)?)\s+"
    r"ay=(?P<ay>[+-]?\d+(?:\.\d+)?)\s+"
    r"az=(?P<az>[+-]?\d+(?:\.\d+)?)\s+"
    r"gx=(?P<gx>[+-]?\d+(?:\.\d+)?)\s+"
    r"gy=(?P<gy>[+-]?\d+(?:\.\d+)?)\s+"
    r"gz=(?P<gz>[+-]?\d+(?:\.\d+)?)\s+"
    r"seed=(?P<seed>[01])\s+"
    r"pitch=(?P<pitch>[+-]?\d+(?:\.\d+)?)\s+"
    r"rate=(?P<rate>[+-]?\d+(?:\.\d+)?)\s+"
    r"accel_valid=(?P<accel_valid>[01])\s+"
    r"trust=(?P<trust>[+-]?\d+(?:\.\d+)?)"
)


@dataclass(frozen=True)
class Sample:
    accel: tuple[float, float, float]
    gyro: tuple[float, float, float]
    seed: bool
    pitch: float
    rate: float
    accel_valid: bool
    trust: float

    @property
    def accel_magnitude(self) -> float:
        return math.sqrt(sum(value * value for value in self.accel))


def parse(lines: Iterable[str]) -> tuple[list[int], list[bool], list[Sample]]:
    who_values: list[int] = []
    ble_events: list[bool] = []
    samples: list[Sample] = []
    for line in lines:
        if match := WHO_RE.search(line):
            who_values.append(int(match.group(1), 16))
        if match := BLE_RE.search(line):
            ble_events.append(match.group(1).lower() == "connected")
        if match := IMU_RE.search(line):
            value = match.groupdict()
            samples.append(
                Sample(
                    accel=tuple(float(value[f"a{axis}"]) for axis in "xyz"),
                    gyro=tuple(float(value[f"g{axis}"]) for axis in "xyz"),
                    seed=value["seed"] == "1",
                    pitch=float(value["pitch"]),
                    rate=float(value["rate"]),
                    accel_valid=value["accel_valid"] == "1",
                    trust=float(value["trust"]),
                )
            )
    return who_values, ble_events, samples


def report(
    who_values: list[int],
    ble_events: list[bool],
    samples: list[Sample],
    *,
    require_filter: bool,
    require_ble: bool,
    stationary: bool,
) -> int:
    failures: list[str] = []
    warnings: list[str] = []

    if not who_values:
        failures.append("WHO_AM_I line is missing")
    elif any(value not in (0x70, 0x71) for value in who_values):
        failures.append(
            "unexpected WHO_AM_I: " + ", ".join(f"0x{value:02X}" for value in who_values)
        )
    else:
        print(f"PASS WHO_AM_I: 0x{who_values[-1]:02X}")

    if not samples:
        failures.append("no IMU samples found")
    else:
        magnitudes = [sample.accel_magnitude for sample in samples]
        abs_gy = [abs(sample.gyro[1]) for sample in samples]
        seeded = sum(sample.seed for sample in samples)
        accel_valid = sum(sample.accel_valid for sample in samples)
        trusts = [sample.trust for sample in samples]
        pitches = [sample.pitch for sample in samples]
        print(
            f"PASS IMU samples={len(samples)} "
            f"|accel| mean={statistics.fmean(magnitudes):.3f} "
            f"range={min(magnitudes):.3f}..{max(magnitudes):.3f} m/s^2"
        )
        print(
            f"INFO filter seed={seeded}/{len(samples)} "
            f"accel_valid={accel_valid}/{len(samples)} "
            f"trust_mean={statistics.fmean(trusts):.3f} "
            f"pitch_range={min(pitches):+.4f}..{max(pitches):+.4f} rad "
            f"max|gy|={max(abs_gy):.3f} rad/s"
        )
        if seeded == 0:
            message = "filter never reached seed=1"
            (failures if require_filter else warnings).append(message)
        if accel_valid == 0:
            warnings.append("accelerometer tilt was never valid")
        if max(magnitudes) < 8.0 or min(magnitudes) > 11.8:
            warnings.append("acceleration magnitude is far from 1 g")
        if stationary and (min(magnitudes) < 8.5 or max(magnitudes) > 11.2):
            failures.append("stationary check: acceleration is not near 1 g")
        if stationary and max(abs_gy) > 0.30:
            failures.append("stationary check: pitch gyro rate exceeds 0.30 rad/s")

    if any(ble_events):
        print("PASS BLE: connected event found")
    elif require_ble:
        failures.append("BLE connected event is missing")
    else:
        warnings.append("BLE connected event is missing")

    for message in warnings:
        print(f"WARN {message}")
    for message in failures:
        print(f"FAIL {message}")
    if failures:
        print(f"RESULT FAIL ({len(failures)} failure(s))")
        return 1
    print("RESULT PASS")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", help="serial log path, or '-' for stdin")
    parser.add_argument(
        "--require-filter", action="store_true", help="fail unless estimator reports seed=1"
    )
    parser.add_argument(
        "--require-ble", action="store_true", help="fail unless BLE connected is logged"
    )
    parser.add_argument(
        "--stationary",
        action="store_true",
        help="also require near-1g accel and low pitch gyro rate",
    )
    args = parser.parse_args()

    stream: TextIO
    if args.log == "-":
        stream = sys.stdin
        return report(*parse(stream), require_filter=args.require_filter,
                      require_ble=args.require_ble, stationary=args.stationary)
    try:
        with Path(args.log).open(encoding="utf-8", errors="replace") as stream:
            return report(*parse(stream), require_filter=args.require_filter,
                          require_ble=args.require_ble, stationary=args.stationary)
    except OSError as exc:
        print(f"FAIL cannot read {args.log}: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
