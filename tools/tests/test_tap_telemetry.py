"""Frame decoding of the compact telemetry stream (see tools/tap_telemetry.py)."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from tap_telemetry import SYNC, TelemetryDecoder, crc8  # noqa: E402


def frame(ftype: int, payload: bytes) -> bytes:
    body = bytes([ftype, len(payload) & 0xFF, len(payload) >> 8]) + payload
    return bytes([SYNC]) + body + bytes([crc8(body)])


class TelemetryDecoderTest(unittest.TestCase):
    def test_template_then_data(self):
        # "TR p=" + fix(v, 3, plus) + " n=" + i32 ; values: +0.088 -> 88, sign 0 -> code 176; i32 -3 -> zigzag 5
        skeleton = b"TR p=" + bytes([0, 27]) + b" n=" + bytes([0, 2])
        dec = TelemetryDecoder(cache_path=None)
        dec.feed(frame(1, (0x1234).to_bytes(2, "little") + skeleton))
        dec.feed(frame(2, (0x1234).to_bytes(2, "little") + bytes([0xB0, 0x01, 5])))
        self.assertEqual(dec.pop(), "TR p=+0.088 n=-3")

    def test_negative_zero_and_specials(self):
        skeleton = bytes([0, 18]) + b" " + bytes([0, 18])
        dec = TelemetryDecoder(cache_path=None)
        dec.feed(frame(1, (7).to_bytes(2, "little") + skeleton))
        # -0.00 -> code 1; nan -> 2**64-2
        nan = (2 ** 64 - 2).to_bytes(8, "little")
        varint = bytearray()
        value = int.from_bytes(nan, "little")
        while value >= 0x80:
            varint.append((value & 0x7F) | 0x80)
            value >>= 7
        varint.append(value)
        dec.feed(frame(2, (7).to_bytes(2, "little") + bytes([1]) + bytes(varint)))
        self.assertEqual(dec.pop(), "-0.00 nan")

    def test_text_and_resync(self):
        dec = TelemetryDecoder(cache_path=None)
        dec.feed(b"hello\r\n" + bytes([SYNC, 9, 9]) + b"junk\n" + frame(3, b"V3 text\r\n"))
        lines = list(dec.lines)
        self.assertIn("hello", lines)
        self.assertIn("V3 text", lines)

    def test_unknown_id_counted(self):
        dec = TelemetryDecoder(cache_path=None)
        dec.feed(frame(2, (99).to_bytes(2, "little") + bytes([1])))
        self.assertEqual(dec.unknown_id, 1)
        self.assertIsNone(dec.pop())


if __name__ == "__main__":
    unittest.main()
