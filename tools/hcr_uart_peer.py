#!/usr/bin/env python3
"""Motorless HCR1 UART peer: acknowledge link, inspect telemetry, send DISABLED only.

Run on LF (Python standard library only) or a PC UART adapter. This is a bench
transport tool, not the hybrid controller. It never sends an enabled target.
"""
import argparse
import fcntl
import json
import os
import secrets
import select
import termios
import time
from pathlib import Path

import hcr_protocol as p


def open_uart(device):
    # Catch an already-running LF runtime/dashboard before opening its port.
    real = os.path.realpath(device)
    for path in Path('/proc').glob('[0-9]*/fd/*'):
        try:
            if os.path.realpath(path) == real:
                raise RuntimeError(f'UART already open by PID {path.parts[2]}')
        except OSError:
            continue
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[4] = attrs[5] = termios.B921600
        attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        return fd
    except Exception:
        os.close(fd)
        raise


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True, help='LF UART path, e.g. /dev/ttyS2')
    ap.add_argument('--seconds', type=float, default=30)
    args = ap.parse_args()
    fd = open_uart(args.port)
    parser = p.Parser()
    pending = bytearray()
    seq, epoch, rx_seq = 0, secrets.randbits(32) or 1, None
    linked = False
    frames = rx_bytes = tx_bytes = hello_frames = hello_acks = 0
    kinds = {}
    next_hello = next_report = 0
    last_telemetry = None
    start = time.monotonic()

    def send(kind, payload):
        nonlocal seq
        pending.extend(p.Frame(kind, seq, time.monotonic_ns()//1000, payload).encode())
        seq = (seq+1) & 0xffffffff
        if len(pending) > 4096:
            raise RuntimeError('UART TX stalled')

    def hello():
        send(p.HELLO, p.HELLO_DATA.pack(epoch,200,100,p.SCHEMA))

    try:
        while time.monotonic()-start < args.seconds:
            now = time.monotonic()
            if not linked and now >= next_hello:
                hello()
                next_hello = now+1
            readable, writable, _ = select.select([fd], [fd] if pending else [], [], .01)
            if writable:
                try:
                    written = os.write(fd,pending)
                    del pending[:written]
                except BlockingIOError:
                    pass
                else:
                    tx_bytes += written
            if not readable:
                continue
            data = os.read(fd,4096)
            rx_bytes += len(data)
            for f in parser.feed(data):
                kinds[str(f.kind)] = kinds.get(str(f.kind),0)+1
                if f.kind == p.HELLO and f.payload == p.HELLO_DATA.pack(0,200,100,p.SCHEMA):
                    hello_frames += 1
                    epoch = secrets.randbits(32) or 1
                    linked, rx_seq = False, None
                    hello()
                    continue
                if len(f.payload) < 4 or int.from_bytes(f.payload[:4],'little') != epoch or not p.newer(f.seq,rx_seq):
                    continue
                rx_seq = f.seq
                if f.kind == p.HELLO_ACK and f.payload == p.HELLO_DATA.pack(epoch,200,100,p.SCHEMA):
                    hello_acks += 1
                    linked = True
                    send(p.STATUS,p.STATUS_DATA.pack(epoch,1,0,0))
                elif linked and f.kind == p.TELEMETRY:
                    t = p.Telemetry.decode(f.payload)
                    frames += 1
                    last_telemetry = now
                    send(p.TARGET,p.TARGET_DATA.pack(epoch,f.seq,p.DISABLED,0,*([0.]*10)))
                    if now >= next_report:
                        print(json.dumps({'link': True,'frames': frames,'flags': t.flags,
                                          'gravity': t.gravity,'gyro': t.gyro,'accel': t.accel,
                                          'q': t.q, 'qd': t.qd,
                                          'encoders_valid': bool(t.flags&p.ENCODER_OK),
                                          'applied_seq': t.applied_seq,'crc_errors':parser.errors}),flush=True)
                        next_report = now+1
        duration = time.monotonic()-start
        print(json.dumps({'link':linked,'lf_rx_frames':kinds,'lf_rx_bytes':rx_bytes,
                          'lf_tx_bytes':tx_bytes,'hello_requests':hello_frames,
                          'hello_acks':hello_acks,'telemetry_frames':frames,
                          'mean_hz':frames/duration,
                          'crc_errors':parser.errors,'enabled_targets_sent':0,
                          'last_sample_age_s':None if last_telemetry is None else time.monotonic()-last_telemetry}))
        if not frames:
            raise SystemExit('FAIL: no telemetry received')
    finally:
        os.close(fd)


if __name__ == '__main__':
    main()
