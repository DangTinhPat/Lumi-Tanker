#!/usr/bin/env python3
"""Decode passive ESP32-S3 RMT pad samples. Never opens a device or sends commands."""
import argparse
import bisect
import json
import re
from pathlib import Path


def decode(items, tick_hz, baud=1_000_000):
    if tick_hz <= 0 or baud <= 0:
        raise ValueError('positive capture clock and baud required')
    runs, time = [], 0
    ended = False
    for word in items:
        for shift in (0, 16):
            duration = (word >> shift) & 0x7fff
            level = (word >> (shift+15)) & 1
            if not duration:
                ended = True
                break
            if runs and runs[-1][2] == level:
                runs[-1][1] += duration
            else:
                runs.append([time, time+duration, level])
            time += duration
        if ended:
            break
    starts = [r[0] for r in runs]

    def at(t):
        i = bisect.bisect_right(starts, t)-1
        return runs[i][2] if 0 <= i < len(runs) and t < runs[i][1] else None

    bit = tick_hz/baud
    next_start, frames, glitches, errors = 0, [], [], []
    for start, end, level in runs:
        if level or start < next_start:
            continue
        # A true UART start must still be low at its centre. Keep rejected
        # narrow pulses visible instead of silently calling them bytes.
        if at(start+bit*.5) != 0:
            glitches.append({'start_us': start*1e6/tick_hz,
                             'low_us': (end-start)*1e6/tick_hz})
            continue
        samples = [at(start+bit*(1.5+i)) for i in range(8)]
        stop = at(start+bit*9.5)
        if None in samples or stop is None:
            errors.append({'start_us': start*1e6/tick_hz, 'error': 'capture_truncated'})
            break
        if stop != 1:
            errors.append({'start_us': start*1e6/tick_hz, 'error': 'stop_bit_low'})
            next_start = start+bit*9.5
            continue
        value = sum(v << i for i, v in enumerate(samples))
        frames.append({'byte': f'{value:02x}', 'start_us': start*1e6/tick_hz,
                       'start_low_us': (end-start)*1e6/tick_hz})
        next_start = start+bit*9.5
    return {'bytes': ''.join(f['byte'] for f in frames), 'frames': frames,
            'short_low_pulses': glitches, 'errors': errors,
            'runs': len(runs), 'captured_us': time*1e6/tick_hz,
            'zero_terminated': ended}


def parse(text):
    captures = {}
    for line in text.splitlines():
        fields = dict(re.findall(r'(\w+)=([^\s]*)', line))
        if line.startswith('PAD_WAVE '):
            if fields.get('available') != '1':
                continue
            key = fields['kind'], fields['serial']
            captures[key] = {'metadata': fields, 'words': {}}
        elif line.startswith('PAD_ITEMS '):
            key = fields['kind'], fields['serial']
            if key not in captures:
                continue
            offset = int(fields['offset'])
            for i, word in enumerate(fields['words'].strip(',').split(',')):
                captures[key]['words'][offset+i] = int(word, 16)
    result = []
    for capture in captures.values():
        if set(capture['words']) != set(range(96)):
            raise ValueError('incomplete MOTOR_WAVE output; read again while stopped')
        meta = capture['metadata']
        decoded = decode([capture['words'][i] for i in range(96)], int(meta['tick_hz']))
        result.append({'metadata': meta, 'decoded': decoded,
                       'pad_starts_with_tx': decoded['bytes'].startswith(meta['tx'])})
    if not result:
        raise ValueError('no available pad capture')
    return result


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('log', type=Path)
    args = ap.parse_args()
    print(json.dumps(parse(args.log.read_text()), indent=2))
