from __future__ import annotations

import ctypes
import os
from pathlib import Path


class V3BalanceBridge:
    IDLE = 0
    BALANCING = 3
    FAULT = 4

    def __init__(self, timestep_s: float):
        root = Path(__file__).resolve().parents[4]
        library = Path(os.environ.get(
            "TAP_V3_BALANCE_LIBRARY",
            root / "build" / "lib" / "libtap_v3_balance_bridge.so",
        )).resolve()
        if not library.is_file():
            raise RuntimeError(
                f"V3 balance bridge not found at {library}; build target "
                "tap_v3_balance_bridge with TAP_BUILD_SIMULATOR=ON"
            )
        self.lib = ctypes.CDLL(str(library))
        float_ptr = ctypes.POINTER(ctypes.c_float)
        int_ptr = ctypes.POINTER(ctypes.c_int)
        self.lib.tap_v3_balance_create.argtypes = [ctypes.c_float]
        self.lib.tap_v3_balance_create.restype = ctypes.c_void_p
        self.lib.tap_v3_balance_destroy.argtypes = [ctypes.c_void_p]
        self.lib.tap_v3_balance_step.argtypes = [
            ctypes.c_void_p, float_ptr, float_ptr, float_ptr, float_ptr,
            float_ptr, ctypes.c_double, ctypes.c_int, float_ptr, float_ptr,
            int_ptr, int_ptr, int_ptr, int_ptr, float_ptr,
        ]
        self.lib.tap_v3_balance_step.restype = ctypes.c_int
        self.lib.tap_v3_leg_gravity.argtypes = [float_ptr, float_ptr]
        self.lib.tap_v3_leg_gravity.restype = ctypes.c_int
        self.handle = self.lib.tap_v3_balance_create(timestep_s)
        if not self.handle:
            raise RuntimeError("could not create the V3 balance controller")

    def close(self):
        if self.handle:
            self.lib.tap_v3_balance_destroy(self.handle)
            self.handle = None

    def step(self, accel, gyro, q, qd, measured_current, now_s: float,
             request_balance: bool):
        arrays = [
            (ctypes.c_float * len(values))(*values)
            for values in (accel, gyro, q, qd, measured_current)
        ]
        target_current = (ctypes.c_float * 4)()
        wheel_speed = (ctypes.c_float * 2)()
        state, wheels_enabled, legs_locked, imu_valid = (ctypes.c_int() for _ in range(4))
        estimated_pitch = ctypes.c_float()
        result = self.lib.tap_v3_balance_step(
            self.handle, *arrays, now_s, int(request_balance), target_current,
            wheel_speed, ctypes.byref(state), ctypes.byref(wheels_enabled),
            ctypes.byref(legs_locked), ctypes.byref(imu_valid), ctypes.byref(estimated_pitch),
        )
        if result != 0:
            raise RuntimeError(f"V3 balance bridge failed ({result})")
        return (
            tuple(target_current), tuple(wheel_speed), state.value,
            bool(wheels_enabled.value), bool(legs_locked.value), bool(imu_valid.value),
            estimated_pitch.value,
        )

    def gravity_torque(self, q):
        joints = (ctypes.c_float * 4)(*q)
        torque = (ctypes.c_float * 4)()
        if self.lib.tap_v3_leg_gravity(joints, torque) != 0:
            raise RuntimeError("V3 leg gravity model failed")
        return tuple(torque)
