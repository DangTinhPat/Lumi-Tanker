"""Independent UART bit-wave vectors exercise the passive pulse decoder."""
import importlib.util
from pathlib import Path
import unittest

path = Path(__file__).resolve().parents[1]/'tools/decode_motor_wave.py'
spec = importlib.util.spec_from_file_location('decode_motor_wave', path)
wave = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wave)


def encode(data, lead=None):
    runs = list(lead or [(1, 25)])
    for byte in data:
        for level in [0]+[(byte >> i) & 1 for i in range(8)]+[1]:
            if runs[-1][0] == level:
                runs[-1] = (level, runs[-1][1]+10)
            else:
                runs.append((level, 10))
    runs.append((1, 200))
    words = []
    for i in range(0, len(runs), 2):
        a = runs[i]
        b = runs[i+1] if i+1 < len(runs) else (0, 0)
        words.append(a[1] | a[0] << 15 | b[1] << 16 | b[0] << 31)
    return words+[0]


class WaveTests(unittest.TestCase):
    def test_two_headers_versus_missing_header(self):
        expected = bytes.fromhex('ffff010503200000d6')
        for data in (expected, expected[1:]):
            result = wave.decode(encode(data), 10_000_000)
            self.assertEqual(result['bytes'], data.hex())
            self.assertEqual(result['errors'], [])
            self.assertEqual(result['frames'][0]['start_low_us'], 1)

    def test_narrow_glitch_not_invented_as_header(self):
        result = wave.decode(encode(b'\xff', [(1, 25), (0, 2), (1, 20)]), 10_000_000)
        self.assertEqual(result['bytes'], 'ff')
        self.assertEqual(len(result['short_low_pulses']), 1)

    def test_all_byte_values(self):
        for byte in range(256):
            self.assertEqual(wave.decode(encode(bytes([byte])), 10_000_000)['bytes'], f'{byte:02x}')

    def test_truncated_frame_cannot_pass(self):
        result = wave.decode([0x000a8019, 0], 10_000_000)
        self.assertEqual(result['bytes'], '')
        self.assertEqual(result['errors'][0]['error'], 'capture_truncated')


if __name__ == '__main__':
    unittest.main()
