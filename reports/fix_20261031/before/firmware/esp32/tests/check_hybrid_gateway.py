#!/usr/bin/env python3
"""Host-only contract checks; no UART opening, flashing or motor operation."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import pty
import select
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[3]


def check_interop(binary, runtime, absent_cases=(False, True)):
    sys.path.insert(0, str(runtime))
    from hybrid import protocol as p
    from hybrid.core import Core
    spec = importlib.util.spec_from_file_location('hcr_runtime_test', runtime/'tests/test_core.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    for absent in absent_cases:
        args = [str(binary), '--peer'] + (['--absent'] if absent else [])
        with subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True) as proc:
            policy = module.FakePolicy()
            lf = Core(policy, module.FakeLogic(), json.loads((runtime/'config.json').read_text()))
            now = 100000

            def peer(command):
                proc.stdin.write(command+'\n')
                proc.stdin.flush()
                result = proc.stdout.readline().split()
                assert len(result) == 5, result
                frames = [] if result[0] == '-' else p.Parser().feed(bytes.fromhex(result[0]))
                return frames, list(map(int, result[1:]))

            def deliver(frames):
                for frame in frames:
                    replies, _ = peer(f'F {now} {frame.encode().hex()}')
                    for reply in replies:
                        deliver(lf.receive(reply, now/1e6))

            def telemetry():
                frames, state = peer(f'T {now}')
                assert len(frames) == 1
                t = p.Telemetry.decode(frames[0].payload)
                deliver(lf.receive(frames[0], now/1e6))
                return t, state

            deliver([lf.hello()])
            t, _ = telemetry()
            if absent:
                assert t.flags == p.IMU_OK
                assert lf.mode == 'FAULT' and lf.fault == 2
                deliver(lf.command('ARM',now/1e6))
                assert lf.mode=='FAULT'
                print('PASS: absent motors -> honest validity, LF invalid-telemetry FAULT, ARM refused')
            else:
                assert t.flags == 7 and lf.mode == 'DISARMED'
                deliver(lf.command('ARM',now/1e6))
                modes = set()
                for _ in range(2700):  # 13.5 s simulated 200 Hz transport.
                    now += 5000
                    t, state = telemetry()
                    modes.add(lf.mode)
                    assert lf.mode != 'FAULT', (lf.reason, state)
                assert {'HOME','STANDUP','WARMUP','MODEL'} <= modes, modes
                assert policy.hidden >= 50
                print('PASS: real LF Core + MCU core, 2700 telemetry exchanges, HOME/STANDUP/WARMUP/MODEL and applied ACK')
                peer(f'E {now}')
                now += 5000
                telemetry()
                assert lf.mode == 'FAULT'
                old_epoch = lf.epoch
                deliver(lf.command('RESET',now/1e6))
                assert lf.epoch != old_epoch and lf.mode == 'DISARMED'
                now += 5000
                _, state = telemetry()
                assert not state[0] and not state[1]
                print('PASS: estop/reset/reconnect remains disarmed')
            proc.stdin.close()
            assert proc.wait(timeout=5) == 0


def check_bench_peer(binary):
    # An OS pseudoterminal exercises the actual bench CLI without touching
    # any physical UART, LF service or ESP32.
    sys.path.insert(0,str(ROOT/'firmware/esp32/tools'))
    import hcr_protocol as p
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    os.close(slave)
    os.set_blocking(master,False)
    parser = p.Parser()
    with subprocess.Popen([str(binary),'--peer','--absent'],stdin=subprocess.PIPE,
                          stdout=subprocess.PIPE,text=True) as mcu:
        with subprocess.Popen([sys.executable,str(ROOT/'firmware/esp32/tools/hcr_uart_peer.py'),
                               '--port',path,'--seconds','1'],stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE,text=True) as bench:
            def command(text):
                mcu.stdin.write(text+'\n'); mcu.stdin.flush()
                result = mcu.stdout.readline().split()
                assert len(result) == 5, result
                return b'' if result[0] == '-' else bytes.fromhex(result[0])

            next_sample = 0
            deadline = time.monotonic()+5
            disabled = 0
            while bench.poll() is None and time.monotonic() < deadline:
                now = time.monotonic_ns()//1000
                if select.select([master],[],[],.001)[0]:
                    try:
                        data = os.read(master,4096)
                    except OSError:  # slave has not opened yet / just closed
                        data = b''
                        time.sleep(.001)
                    if data:
                        for f in parser.feed(data):
                            if f.kind == p.TARGET:
                                fields = p.TARGET_DATA.unpack(f.payload)
                                assert fields[2:4] == (p.DISABLED,0)
                                disabled += 1
                        reply = command(f'F {now} {data.hex()}')
                        if reply:
                            os.write(master,reply)
                if now >= next_sample:
                    raw = command(f'T {now}')
                    if raw:
                        os.write(master,raw)
                    next_sample = now+5000
            output, error = bench.communicate(timeout=2)
            assert bench.returncode == 0, (output,error)
            report = json.loads(output.splitlines()[-1])
            assert report['telemetry_frames'] > 20 and disabled > 20, report
            assert report['enabled_targets_sent'] == 0 and report['crc_errors'] == 0, report
        mcu.stdin.close()
        assert mcu.wait(timeout=5) == 0
    os.close(master)
    print('PASS: bench UART CLI over virtual serial, telemetry and DISABLED-only targets')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runtime', type=Path, help='LF hybrid_runtime directory for cross-language integration')
    args = ap.parse_args()
    with tempfile.TemporaryDirectory(prefix='hcr-mcu-check-') as directory:
        binary = Path(directory)/'gateway-tests'
        subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-O1','-g',
                        '-fsanitize=address,undefined','-fno-omit-frame-pointer',
                        '-I'+str(ROOT/'firmware/esp32/include'),
                        str(ROOT/'firmware/esp32/src/HybridGatewayCore.cpp'),
                        str(ROOT/'firmware/esp32/tests/hybrid_gateway_tests.cpp'),
                        '-o',str(binary)],check=True)
        subprocess.run([str(binary)],check=True)
        check_bench_peer(binary)
        if args.runtime:
            check_interop(binary,args.runtime.resolve())


if __name__ == '__main__':
    main()
