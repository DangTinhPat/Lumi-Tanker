"""Bounded cleanup of Linux subprocess sessions created by this module.

No process-name matching or unrelated session signals. Children that deliberately
create a new session are outside this contract; use a cgroup for that case.
"""
from dataclasses import dataclass
import ctypes
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from weakref import WeakKeyDictionary


@dataclass(frozen=True)
class Identity:
    pid: int
    session: int
    started: int
    state: str


_OWNED = WeakKeyDictionary()
_LIBC = ctypes.CDLL(None, use_errno=True)


def _pidfd_open(pid):
    if hasattr(os, 'pidfd_open'):
        return os.pidfd_open(pid)
    fd = _LIBC.pidfd_open(ctypes.c_int(pid), ctypes.c_uint(0))
    if fd < 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))
    return fd


def _pidfd_signal(fd, sig):
    if hasattr(signal, 'pidfd_send_signal'):
        signal.pidfd_send_signal(fd, sig)
    elif _LIBC.pidfd_send_signal(ctypes.c_int(fd), ctypes.c_int(sig),
                                ctypes.c_void_p(), ctypes.c_uint(0)) < 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))


def _identity(pid):
    try:
        text = Path(f'/proc/{pid}/stat').read_text()
    except (FileNotFoundError, ProcessLookupError):
        return None
    # comm may contain spaces and parentheses; subsequent fields are fixed.
    fields = text[text.rfind(')')+2:].split()
    return Identity(pid, int(fields[3]), int(fields[19]), fields[0])


def start_owned_process(*args, **kwargs):
    if kwargs.get('start_new_session', True) is not True:
        raise ValueError('Owned processes require a new session')
    if not ((hasattr(os, 'pidfd_open') or hasattr(_LIBC, 'pidfd_open')) and
            (hasattr(signal, 'pidfd_send_signal') or hasattr(_LIBC, 'pidfd_send_signal'))):
        raise RuntimeError('Owned cleanup requires Linux pidfd support')
    kwargs['start_new_session'] = True
    process = subprocess.Popen(*args, **kwargs)
    identity = _identity(process.pid)
    if identity is None or identity.session != process.pid:
        # Never signal a guessed process group when session ownership is unknown.
        process.kill()
        process.wait(timeout=2)
        raise RuntimeError('Could not establish owned subprocess session')
    _OWNED[process] = identity
    return process


def _members(owner):
    leader = _identity(owner.pid)
    if leader is not None and leader.started != owner.started:
        raise RuntimeError('Session leader PID was reused; refusing cleanup signals')
    members = []
    with os.scandir('/proc') as entries:
        for entry in entries:
            if not entry.name.isdecimal():
                continue
            item = _identity(int(entry.name))
            if item is not None and item.session == owner.session and item.started >= owner.started:
                members.append(item)
    return members


def _signal_member(member, sig):
    try:
        fd = _pidfd_open(member.pid)
    except ProcessLookupError:
        return
    try:
        current = _identity(member.pid)
        if current is not None and (current.started, current.session) == (member.started, member.session):
            try:
                _pidfd_signal(fd, sig)
            except ProcessLookupError:
                pass
    finally:
        os.close(fd)


def cleanup_group(process, *, timeouts=(5., 3., 2.), poll_interval=.05):
    """Escalate INT/TERM/KILL, including children after the launcher has exited.

    Returns only once no live owned session members remain; raises on survivors.
    Zombies are reported separately: signals cannot reap another parent's child.
    """
    if process is None:
        return {'live_pids': [], 'zombie_pids': [], 'signals': []}
    if process not in _OWNED:
        raise ValueError('Refusing cleanup of process not created by start_owned_process')
    if len(timeouts) != 3 or any(not math.isfinite(t) or t < 0 for t in timeouts):
        raise ValueError('Provide three finite nonnegative escalation timeouts')
    if not math.isfinite(poll_interval) or poll_interval <= 0:
        raise ValueError('Positive finite poll interval required')
    owner = _OWNED[process]
    if owner.session == os.getsid(0):
        raise ValueError('Refusing cleanup of the caller session')
    report = {'session': owner.session, 'signals': []}

    def remaining():
        process.poll()  # Reap the launcher, but do not stop looking for children.
        members = _members(owner)
        # The launcher can exit between poll() and the /proc snapshot.
        if any(p.pid == process.pid and p.state in ('Z', 'X') for p in members):
            process.wait(timeout=1.)
            members = _members(owner)
        report['zombie_pids'] = [p.pid for p in members if p.state in ('Z', 'X')]
        live = [p for p in members if p.state not in ('Z', 'X')]
        report['live_pids'] = [p.pid for p in live]
        return live

    for sig, timeout in zip((signal.SIGINT, signal.SIGTERM, signal.SIGKILL), timeouts):
        deadline = time.monotonic()+timeout
        sent = set()
        while True:
            live = remaining()
            if not live:
                report['returncode'] = process.poll()
                return report
            for member in live:
                key = (member.pid, member.started)
                if key not in sent:
                    _signal_member(member, sig)
                    sent.add(key)
                    report['signals'].append({'pid': member.pid, 'signal': sig.name})
            delay = deadline-time.monotonic()
            if delay <= 0:
                break
            time.sleep(min(poll_interval, delay))
    if remaining():
        raise TimeoutError(f'Owned session {owner.session} survived cleanup: {report["live_pids"]}')
    report['returncode'] = process.poll()
    return report
