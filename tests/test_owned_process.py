"""Owned lifecycle tests use tiny native Python children."""
import importlib.util
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('tap_owned_process',
    Path(__file__).resolve().parents[1]/'tools/owned_process.py')
M = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = M
spec.loader.exec_module(M)


@unittest.skipUnless(sys.platform == 'linux', 'Linux pidfd required')
class OwnedProcessTest(unittest.TestCase):
    def setUp(self):
        self.processes = []

    def tearDown(self):
        for p in self.processes:
            M.cleanup_group(p, timeouts=(.1, .1, 1.), poll_interval=.01)
            if p.stdout:
                p.stdout.close()
            if p.stdin:
                p.stdin.close()

    def spawn(self, code):
        p = M.start_owned_process([sys.executable, '-u', '-c', code],
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=subprocess.DEVNULL, text=True)
        self.processes.append(p)
        return p

    def ready(self, p):
        self.assertTrue(select.select([p.stdout], [], [], 3.)[0], 'child readiness timed out')
        return p.stdout.readline().strip()

    def test_graceful_and_idempotent_cleanup(self):
        p = self.spawn("import time; print('ready'); time.sleep(60)")
        self.ready(p)
        result = M.cleanup_group(p, timeouts=(1., .1, 1.))
        self.assertEqual(result['live_pids'], [])
        self.assertIsNotNone(p.returncode)
        self.assertEqual(M.cleanup_group(p)['live_pids'], [])

    def test_stubborn_child_escalates_without_killing_unrelated(self):
        unrelated = self.spawn("import time; print('ready'); time.sleep(60)")
        self.ready(unrelated)
        p = self.spawn("import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); "
                       "signal.signal(signal.SIGTERM,signal.SIG_IGN); print('ready'); time.sleep(60)")
        self.ready(p)
        start = time.monotonic()
        result = M.cleanup_group(p, timeouts=(.05, .05, 1.), poll_interval=.01)
        self.assertLess(time.monotonic()-start, 2.)
        self.assertEqual(p.returncode, -signal.SIGKILL)
        self.assertIn('SIGKILL', [x['signal'] for x in result['signals']])
        self.assertIsNone(unrelated.poll())

    def test_launcher_exits_between_poll_and_snapshot(self):
        p = self.spawn("import time; print('ready'); time.sleep(60)")
        self.ready(p)
        real_members = M._members
        injected = False

        def snapshot(owner):
            nonlocal injected
            if not injected:
                injected = True
                p.kill()
                deadline = time.monotonic()+2.
                while M._identity(p.pid).state != 'Z':
                    if time.monotonic() >= deadline:
                        self.fail('launcher did not exit')
                    time.sleep(.001)
            return real_members(owner)

        with patch.object(M, '_members', side_effect=snapshot):
            result = M.cleanup_group(p)
        self.assertEqual(result['zombie_pids'], [])
        self.assertEqual(result['live_pids'], [])
        self.assertEqual(result['returncode'], -signal.SIGKILL)

    def test_child_lingers_after_launcher_exits_and_uses_own_process_group(self):
        p = self.spawn("""import os,signal,sys,time
child=os.fork()
if child == 0:
    os.setpgid(0, 0)
    signal.signal(signal.SIGINT,signal.SIG_IGN)
    signal.signal(signal.SIGTERM,signal.SIG_IGN)
    print(os.getpid())
    time.sleep(60)
else:
    sys.stdin.readline()
    os._exit(0)
""")
        child = int(self.ready(p))
        p.stdin.write('exit\n');p.stdin.flush();p.wait(timeout=2)
        self.assertIsNotNone(M._identity(child))
        result = M.cleanup_group(p, timeouts=(.05, .05, 1.), poll_interval=.01)
        self.assertEqual(result['live_pids'], [])
        identity = M._identity(child)
        self.assertTrue(identity is None or identity.state == 'Z')

    def test_unowned_process_is_refused(self):
        p = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'],
                             start_new_session=True)
        try:
            with self.assertRaises(ValueError):
                M.cleanup_group(p)
            self.assertIsNone(p.poll())
        finally:
            p.kill();p.wait(timeout=2)

    def test_survivor_timeout_is_reported_not_silently_successful(self):
        p = self.spawn("import time; print('ready'); time.sleep(60)")
        self.ready(p)
        with patch.object(M, '_signal_member'):
            with self.assertRaises(TimeoutError):
                M.cleanup_group(p, timeouts=(0., 0., 0.))

    def test_pid_reuse_identity_refuses_signals(self):
        p = self.spawn("import time; print('ready'); time.sleep(60)")
        self.ready(p)
        real = M._identity
        def reused(pid):
            item = real(pid)
            return M.Identity(pid, pid, item.started+1, item.state) if pid == p.pid else item
        with patch.object(M, '_identity', side_effect=reused), patch.object(M, '_signal_member') as send:
            with self.assertRaisesRegex(RuntimeError, 'reused'):
                M.cleanup_group(p)
            send.assert_not_called()


if __name__ == '__main__':
    unittest.main()
