"""Decoder for the robot's compact telemetry stream (controller/v3/include/tap_fw/TelemetryCodec.hpp).

Frame: 0xA5, type, len u16 LE, payload, crc8 (poly 0x07 over type..payload).
  type 1 template  id u16 + skeleton (literal text; 0x00 op = a value slot)
  type 2 data      id u16 + one varint per value slot
  type 3 text      plain console text
Bytes outside frames are ordinary text lines (the robot's text mode, banners), so one decoder
serves both modes. Templates are cached in ~/.cache/tap_templates.json so the first records after
a reconnect can be decoded before the robot repeats the template (it does, every 2 s).
"""
from __future__ import annotations

import json
import os
from collections import deque

SYNC = 0xA5
MAX_PAYLOAD = 2200  # robot frames are <= ~1.6 KB
CACHE_PATH = os.path.expanduser("~/.cache/tap_templates.json")


def crc8(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def _varint(buf: bytes, pos: int) -> tuple[int, int]:
    value = shift = 0
    while True:
        if pos >= len(buf):
            raise ValueError("truncated varint")
        b = buf[pos]
        pos += 1
        value |= (b & 0x7F) << shift
        if not b & 0x80:
            return value, pos
        shift += 7
        if shift > 70:
            raise ValueError("varint too long")


def _fix_text(code: int, decimals: int, plus: bool) -> str:
    full = (1 << 64) - 1
    if code == full - 1:
        return "nan"
    if code == full - 2:
        return "inf"
    if code == full - 3:
        return "-inf"
    if code == full:
        return "ovf"
    q, neg = code >> 1, code & 1
    sign = "-" if neg else ("+" if plus else "")
    scale = 10 ** decimals
    text = f"{sign}{q // scale}"
    if decimals:
        text += "." + f"{q % scale:0{decimals}d}"
    return text


def render(skeleton: bytes, values: bytes) -> str:
    """Rebuild the text line from a template and the value bytes (after the id)."""
    out: list[str] = []
    i = pos = 0
    n = len(skeleton)
    while i < n:
        c = skeleton[i]
        i += 1
        if c != 0:
            out.append(chr(c))
            continue
        op = skeleton[i]
        i += 1
        v, pos = _varint(values, pos)
        if op == 1:
            out.append(str(v))
        elif op == 2:
            out.append(str((v >> 1) ^ -(v & 1)))
        elif op == 3:
            out.append(f"{v:x}")
        elif 16 <= op <= 21 or 24 <= op <= 29:
            out.append(_fix_text(v, (op - 16) & 7, op >= 24))
        else:
            raise ValueError(f"bad op {op}")
    return "".join(out)


class TelemetryDecoder:
    def __init__(self, cache_path: str | None = CACHE_PATH) -> None:
        self.templates: dict[int, bytes] = {}
        self.cache_path = cache_path
        self.buf = bytearray()
        self.lines: deque[str] = deque()
        self.crc_errors = 0
        self.unknown_id = 0
        self._cache_dirty = False
        self._load()

    # -- cache ----------------------------------------------------------------------------
    def _load(self) -> None:
        if not self.cache_path:
            return
        try:
            with open(self.cache_path, encoding="utf-8") as fh:
                self.templates = {int(k): bytes.fromhex(v) for k, v in json.load(fh).items()}
        except (OSError, ValueError):
            self.templates = {}

    def save(self) -> None:
        if not (self.cache_path and self._cache_dirty):
            return
        try:
            os.makedirs(os.path.dirname(self.cache_path), exist_ok=True)
            with open(self.cache_path, "w", encoding="utf-8") as fh:
                json.dump({str(k): v.hex() for k, v in self.templates.items()}, fh)
            self._cache_dirty = False
        except OSError:
            pass

    # -- stream ---------------------------------------------------------------------------
    def feed(self, data: bytes) -> None:
        self.buf += data
        buf = self.buf
        pos = 0
        text_start = 0
        while pos < len(buf):
            if buf[pos] == SYNC:
                if len(buf) - pos < 4:
                    break  # header not complete yet
                length = buf[pos + 2] | (buf[pos + 3] << 8)
                if length > MAX_PAYLOAD:  # cannot be one of ours: a stray 0xA5
                    self.crc_errors += 1
                    pos += 1
                    continue
                if len(buf) - pos < 5 + length:
                    break  # frame not complete yet
                body = bytes(buf[pos + 1:pos + 4 + length])
                if crc8(body) == buf[pos + 4 + length]:
                    self._flush_text(buf[text_start:pos])
                    self._frame(body[0], body[3:])
                    pos += 5 + length
                    text_start = pos
                    continue
                self.crc_errors += 1  # false sync byte or damaged frame: resynchronise
            pos += 1
        # Keep the unfinished tail: text up to the last newline is emitted now.
        cut = buf.rfind(b"\n", text_start, pos)
        if cut >= 0:
            self._flush_text(buf[text_start:cut + 1])
            text_start = cut + 1
        del buf[:text_start]

    def _flush_text(self, raw: bytes | bytearray) -> None:
        if not raw:
            return
        for line in bytes(raw).decode("utf-8", errors="replace").splitlines():
            if line:
                self.lines.append(line)

    def _frame(self, ftype: int, payload: bytes) -> None:
        if ftype == 1 and len(payload) >= 2:
            tid = payload[0] | (payload[1] << 8)
            if self.templates.get(tid) != payload[2:]:
                self.templates[tid] = bytes(payload[2:])
                self._cache_dirty = True
        elif ftype == 2 and len(payload) >= 2:
            tid = payload[0] | (payload[1] << 8)
            skeleton = self.templates.get(tid)
            if skeleton is None:
                self.unknown_id += 1
                return
            try:
                self.lines.append(render(skeleton, payload[2:]))
            except ValueError:
                self.crc_errors += 1
        elif ftype == 3:
            self._flush_text(payload)

    def pop(self) -> str | None:
        return self.lines.popleft() if self.lines else None
