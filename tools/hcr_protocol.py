"""HCR1 wire format. See PROTOCOL.md; all integers/floats little endian."""
import math
import struct
import zlib
from dataclasses import dataclass

MAGIC = b'\xa5\x5a'
VERSION = 1
HEADER = struct.Struct('<2sBBHHIQ')
CRC = struct.Struct('<I')
MAX_PAYLOAD = 256
HELLO, HELLO_ACK, TELEMETRY, TARGET, STATUS = 1, 2, 3, 0x81, 0x82
HELLO_DATA = struct.Struct('<IHHI')
SCHEMA = 0x31524348  # HCR1, fixed interface identifier
TELEMETRY_DATA = struct.Struct('<III29f')
TARGET_DATA = struct.Struct('<IIHH10f')
STATUS_DATA = struct.Struct('<IIII')
IMU_OK, ENCODER_OK, CALIBRATED, ESTOP = 1, 2, 4, 8
TORQUE_VALID, TARGET_VALID, DRIVER_FAULT = 16, 32, 64
DISABLED, HOME_PD, RECOVERY, POLICY_PD = 0, 1, 2, 3
LEGS_ENABLE, WHEELS_ENABLE = 1, 2


@dataclass
class Frame:
    kind: int
    seq: int
    stamp: int
    payload: bytes = b''

    def encode(self):
        if len(self.payload) > MAX_PAYLOAD:
            raise ValueError('payload too large')
        raw = HEADER.pack(MAGIC, VERSION, self.kind, len(self.payload), 0,
                          self.seq & 0xffffffff, self.stamp) + self.payload
        return raw + CRC.pack(zlib.crc32(raw) & 0xffffffff)


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= HEADER.size:
            pos = self.buffer.find(MAGIC)
            if pos < 0:
                self.buffer[:] = self.buffer[-1:]
                break
            if pos:
                del self.buffer[:pos]
            if len(self.buffer) < HEADER.size:
                break
            _, version, kind, size, reserved, seq, stamp = HEADER.unpack_from(self.buffer)
            if version != VERSION or size > MAX_PAYLOAD or reserved:
                self.errors += 1
                del self.buffer[0]
                continue
            total = HEADER.size + size + CRC.size
            if len(self.buffer) < total:
                break
            crc, = CRC.unpack_from(self.buffer, total - 4)
            if zlib.crc32(self.buffer[:total-4]) & 0xffffffff != crc:
                self.errors += 1
                del self.buffer[0]
                continue
            frames.append(Frame(kind, seq, stamp, bytes(self.buffer[HEADER.size:total-4])))
            del self.buffer[:total]
        return frames


def newer(current, previous):
    return previous is None or 0 < ((current-previous) & 0xffffffff) < 0x80000000


@dataclass
class Telemetry:
    epoch: int
    flags: int
    applied_seq: int
    gyro: list
    gravity: list
    accel: list
    q: list
    qd: list
    hip_torque: list
    applied_target: list

    @classmethod
    def decode(cls, payload):
        v = TELEMETRY_DATA.unpack(payload)
        f = v[3:]
        if not all(math.isfinite(x) for x in f):
            raise ValueError('nonfinite telemetry')
        return cls(*v[:3], list(f[:3]), list(f[3:6]), list(f[6:9]),
                   list(f[9:15]), list(f[15:21]), list(f[21:23]), list(f[23:29]))

    def encode(self):
        return TELEMETRY_DATA.pack(self.epoch, self.flags, self.applied_seq,
            *(self.gyro+self.gravity+self.accel+self.q+self.qd+self.hip_torque+self.applied_target))
