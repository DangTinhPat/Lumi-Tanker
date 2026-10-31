#!/usr/bin/env python3
"""Compile actual 3in1 C transport + HCR adapter against a deterministic SDK mock.
No UART, board, network or motors. SDK mock cannot prove electrical timing.
"""
from pathlib import Path
import hashlib
import json
import resource
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
FW = ROOT
VENDOR = FW/'drivers/bl4012e'
FIXTURE = FW/'tests/bl4012e_sdk'


def main():
    for name, digest in json.loads((VENDOR/'upstream/SHA256.json').read_text()).items():
        assert hashlib.sha256((VENDOR/'upstream'/name).read_bytes()).hexdigest() == digest, name
    with tempfile.TemporaryDirectory(prefix='hcr-inherited-bus-') as directory:
        build = Path(directory)
        shutil.copy(FIXTURE/'esp_stub.h', build)
        for name in ['driver/gpio.h', 'driver/uart.h', 'esp_err.h', 'esp_check.h',
                     'esp_timer.h', 'esp_idf_version.h', 'esp_rom_gpio.h',
                     'freertos/FreeRTOS.h', 'freertos/task.h', 'hal/gpio_ll.h',
                     'soc/gpio_sig_map.h', 'soc/uart_periph.h', 'hal/uart_ll.h', 'esp_rom_sys.h']:
            path = build/name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('#include "esp_stub.h"\n')
        common = ['-Wall', '-Wextra', '-Werror', '-g', '-O1',
                  '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                  '-fno-pie', '-ffunction-sections', '-fdata-sections', '-I'+str(build)]

        def compile_case(name, source, headers, defines=(), adapter=False):
            obj = build/(name+'.o')
            flags = common + ['-I'+str(headers)]
            subprocess.run(['cc', '-std=c99', *flags, '-c', str(source), '-o', str(obj)], check=True)
            binary = build/name
            inputs = [str(FIXTURE/'probe.cpp'), str(obj)]
            if adapter:
                inputs += [str(FW/'src/EspBlMotor.cpp'), str(FW/'src/BlMotor.cpp')]
            subprocess.run(['g++', '-std=c++17', *flags,
                            '-I'+str(FW/'include'), *defines, *inputs,
                            '-no-pie', '-Wl,--gc-sections', '-o', str(binary)], check=True)
            return binary

        upstream = compile_case('upstream', VENDOR/'upstream/joint_bus.c', VENDOR/'upstream')
        candidate = compile_case('candidate', VENDOR/'src/joint_bus.c', VENDOR/'include')
        old = subprocess.check_output([str(upstream)])
        new = subprocess.check_output([str(candidate)])
        assert old == new, 'SDK calls, pins, frame bytes or stage timeout values changed'
        print('PASS: upstream/candidate identical unguarded SDK call trace and golden packets on covered common paths')
        checked = compile_case('adapter', VENDOR/'src/joint_bus.c', VENDOR/'include',
                               ['-DCANDIDATE', '-DADAPTER', '-DARDUINO', '-DTAP_HYBRID_GATEWAY'], True)
        subprocess.run([str(checked)], check=True, stdout=subprocess.DEVNULL)
        print('PASS: actual C module + C++ adapter; TX/echo/reply, sync validation, late/quiet bus, deadline/cancel, 97-byte full batch')
        # Negative control: the test must detect an omitted TX pad attachment.
        broken = build/'broken.c'
        broken.write_text((VENDOR/'src/joint_bus.c').read_text().replace(
            '  bus_drive(bus);', '  if (bus->pin < 0) bus_drive(bus);'))
        control = compile_case('negative', broken, VENDOR/'include')
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
        result = subprocess.run([str(control)], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        assert result.returncode != 0 and b'Assertion' in result.stderr, result.stderr.decode()
        print('PASS: negative control detects missing pad drive; ASan/UBSan enabled')
        # Targeted control for the upstream quiet-drain defect: ten busy reads
        # must not clear the drain latch and permit another transmission.
        broken.write_text((VENDOR/'src/joint_bus.c').read_text().replace(
            'bus->last_error = JB_ERROR_QUIET;', 'bus->drain = false; return true;'))
        control = compile_case('quiet-negative', broken, VENDOR/'include',
                               ['-DCANDIDATE', '-DADAPTER', '-DARDUINO', '-DTAP_HYBRID_GATEWAY'], True)
        result = subprocess.run([str(control)], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        assert result.returncode != 0 and b'JB_ERROR_QUIET' in result.stderr, result.stderr.decode()
        print('PASS: negative control detects transmitting before the bus becomes quiet')


if __name__ == '__main__':
    main()
