import math
import os
import select
import struct
import subprocess
import time
from .protocol import TORQUE_VALID

REQUEST = struct.Struct('<24d')
REPLY = struct.Struct('<14d')


class Logic:
    def __init__(self, executable):
        self.process = subprocess.Popen([str(executable)], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, bufsize=0)
        self.reset()

    def exchange(self, values):
        raw = REQUEST.pack(*values)
        self.process.stdin.write(raw)
        response = bytearray()
        deadline = time.monotonic()+.02
        while len(response) < REPLY.size:
            remaining = deadline-time.monotonic()
            if remaining <= 0 or not select.select([self.process.stdout], [], [], remaining)[0]:
                raise RuntimeError('logic helper timeout')
            data = os.read(self.process.stdout.fileno(), REPLY.size-len(response))
            if not data:
                raise RuntimeError('logic helper exited')
            response.extend(data)
        result = REPLY.unpack(response)
        if not all(math.isfinite(v) for v in result) or result[0] < 0:
            raise RuntimeError('stand-up fault code '+str(result[1]))
        return result

    def reset(self):
        self.exchange([0.]*23+[1.])

    def step(self, elapsed, t):
        pitch = math.atan2(t.gravity[0], math.hypot(t.gravity[1], t.gravity[2]))
        s = [elapsed, pitch, t.gyro[1], t.gyro[2], t.accel[0], t.accel[2]]
        s += t.q+t.qd+t.hip_torque+[1., 1., float(bool(t.flags & TORQUE_VALID)), 0.]
        r = self.exchange(s)
        return bool(r[0]), int(r[1]), int(r[2]), list(r[3:9]), list(r[9:13])

    def close(self):
        self.process.stdin.close()
        try:
            self.process.wait(timeout=1)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.process.stdout.close()
