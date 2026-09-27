#!/usr/bin/env python3
"""Serial console for the robot controller firmware.

Logs every line from the ESP32 (SENS / TR / RELEASE / CONTROL / TUNE) to a
file with host timestamps, and forwards tuning commands to it:

    GET
    SET trim_gain 0.3     SET trim_max 0.2     SET trim 0.05
    SET margin 0.25       SET accel 1.4
    TRIM RESET

Commands come from the terminal (type and press Enter) and, with --fifo,
from a named pipe so another shell can send them:

    python3 tools/controller_console.py --log run10.log --fifo /tmp/tap_console
    echo "SET trim_gain 0.4" > /tmp/tap_console

Opening the port resets the ESP32 (auto-reset circuit), so start the console
before pressing Start, never during a balance run.
"""

import argparse
import glob
import os
import select
import sys
import time

import serial


def default_port():
    ports = sorted(glob.glob("/dev/serial/by-id/*CP210*")) or sorted(
        glob.glob("/dev/ttyUSB*"))
    return ports[0] if ports else None


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=default_port())
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--log", help="append received lines to this file")
    parser.add_argument("--fifo", help="also read commands from this named pipe")
    parser.add_argument("--quiet", action="store_true",
                        help="do not echo SENS lines to the terminal (still logged)")
    args = parser.parse_args()
    if not args.port:
        sys.exit("no serial port found; pass --port")

    link = serial.Serial(args.port, args.baud, timeout=0)
    log = open(args.log, "a", buffering=1) if args.log else None
    fifo_fd = None
    if args.fifo:
        if not os.path.exists(args.fifo):
            os.mkfifo(args.fifo)
        # O_RDWR keeps the pipe open between writers instead of hitting EOF.
        fifo_fd = os.open(args.fifo, os.O_RDWR | os.O_NONBLOCK)
    interactive = sys.stdin.isatty()
    print(f"console on {args.port} @ {args.baud}; log={args.log} fifo={args.fifo}",
          file=sys.stderr)

    rx = b""
    fifo_buf = b""
    start = time.monotonic()

    def send(text):
        text = text.strip()
        if not text:
            return
        link.write((text + "\n").encode())
        stamp = f"{time.monotonic() - start:9.3f}"
        note = f"{stamp} >> {text}"
        print(note, flush=True)
        if log:
            log.write(note + "\n")

    try:
        while True:
            watch = [link.fileno()]
            if interactive:
                watch.append(sys.stdin.fileno())
            if fifo_fd is not None:
                watch.append(fifo_fd)
            ready, _, _ = select.select(watch, [], [], 0.2)
            if link.fileno() in ready:
                rx += link.read(4096)
                *lines, rx = rx.split(b"\n")
                for raw in lines:
                    line = raw.decode(errors="replace").rstrip("\r")
                    stamp = f"{time.monotonic() - start:9.3f}"
                    if log:
                        log.write(f"{stamp} {line}\n")
                    if not (args.quiet and line.startswith("SENS")):
                        print(f"{stamp} {line}", flush=True)
            if interactive and sys.stdin.fileno() in ready:
                entered = sys.stdin.readline()
                if not entered:
                    interactive = False
                else:
                    send(entered)
            if fifo_fd is not None and fifo_fd in ready:
                fifo_buf += os.read(fifo_fd, 1024)
                *commands, fifo_buf = fifo_buf.split(b"\n")
                for command in commands:
                    send(command.decode(errors="replace"))
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
        if log:
            log.close()


if __name__ == "__main__":
    main()
