#!/usr/bin/env python3
"""Authenticated hybrid MCU OTA with verified OFF and post-reboot image identity.

First installation requires USB. No ARM, CLEAR_FAULT or motion command is sent.
Use --check for local image validation only, or --status for read-only MCU status.
"""
import argparse
import hashlib
import importlib.util
import json
import logging
from pathlib import Path
import re
import socket
import struct
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
ENV = 'esp32-s3-hybrid-mcu'


def validate_image(binary, credentials):
    data = binary.read_bytes()
    manifest = json.loads(binary.with_suffix('.ota.json').read_text())
    if (manifest.get('environment') != ENV or manifest.get('bytes') != len(data)
            or manifest.get('sha256') != hashlib.sha256(data).hexdigest()):
        raise ValueError('Firmware/manifest mismatch or wrong PlatformIO environment; rebuild')
    if (len(data) < 288 or data[0] != 0xe9 or struct.unpack_from('<H',data,12)[0] != 9
            or struct.unpack_from('<I',data,32)[0] != 0xabcd5432
            or manifest.get('elf_sha256') != data[176:208].hex()):
        raise ValueError('Not an identified ESP32-S3 application image')
    password = credentials.get('password', '')
    if not password or manifest.get('ota_auth_hash') != hashlib.md5(password.encode()).hexdigest():
        raise ValueError('OTA credentials do not match this image; provision and build before USB installation')
    return manifest


def parse_line(line):
    kind, _, body = line.partition(' ')
    return kind, dict(re.findall(r'(\w+)=([^\s]+)', body))


class Console:
    def __init__(self, host, port=8023):
        self.host, self.port = host, port

    def request(self, command='STATUS', timeout=3):
        result = {}
        deadline = time.monotonic()+timeout
        with socket.create_connection((self.host,self.port),timeout=min(2,timeout)) as sock:
            sock.settimeout(.2)
            sock.sendall((command+'\n').encode())
            pending = b''
            while time.monotonic() < deadline:
                try:
                    part = sock.recv(4096)
                except socket.timeout:
                    continue
                if not part:
                    break
                pending += part
                if len(pending)>16384:
                    raise RuntimeError('Console returned an oversized line')
                while b'\n' in pending:
                    raw, pending = pending.split(b'\n',1)
                    line = raw.decode(errors='replace').strip()
                    if line.startswith('ERR '):
                        raise RuntimeError(line)
                    kind, fields = parse_line(line)
                    if kind in ('HCR','OTA','NET'):
                        result[kind] = fields
                if all(k in result for k in ('HCR','OTA','NET')):
                    return result
        if 'HCR' not in result or 'OTA' not in result:
            raise RuntimeError('Incomplete HCR console status')
        return result


def check_identity(state, expected_mac):
    actual = state.get('NET',{}).get('mac','').lower()
    if not expected_mac or actual != expected_mac.lower():
        raise RuntimeError(f'MCU identity mismatch: expected {expected_mac}, received {actual or "missing"}')


def is_ready(state):
    ota, hcr = state.get('OTA',{}), state.get('HCR',{})
    return (all(ota.get(k)=='1' for k in ('supported','maintenance','off_confirmed','ready'))
            and hcr.get('armed')=='0' and hcr.get('enabled')=='0'
            and int(hcr.get('report_age_ms','999999'))<=250)


def update(console, expected_mac, manifest, upload, wait_seconds=20, reboot_seconds=40):
    state=console.request()
    # Reject old/unsupported firmware before changing its operating state.
    if state.get('OTA',{}).get('supported') != '1':
        raise RuntimeError('MCU has no OTA credentials/partition: install this build once via USB')
    check_identity(state,expected_mac)
    if state.get('HCR',{}).get('armed')!='0' or state.get('HCR',{}).get('enabled')!='0':
        raise RuntimeError('Robot is active: DISARM from LF before starting OTA')
    console.request('OTA_BEGIN')
    deadline=time.monotonic()+wait_seconds
    while time.monotonic()<deadline:
        state=console.request()
        check_identity(state,expected_mac)
        if is_ready(state):
            break
        time.sleep(.25)
    else:
        raise RuntimeError('OTA not ready: no fresh confirmation that all motors are OFF; maintenance remains locked')
    print('Maintenance ready; all motors confirmed OFF. Uploading...',flush=True)
    result=upload()
    if result != 0:
        raise RuntimeError('OTA transfer failed; MCU stays in maintenance. Retry; do not ARM')
    deadline=time.monotonic()+reboot_seconds
    last='waiting for MCU reboot'
    while time.monotonic()<deadline:
        try:
            state=console.request()
            check_identity(state,expected_mac)
            hcr,ota=state['HCR'],state['OTA']
            if (ota.get('elf_sha256')==manifest['elf_sha256'] and ota.get('maintenance')=='0'
                    and hcr.get('armed')=='0' and hcr.get('enabled')=='0'):
                print('PASS: expected firmware is running on the same MCU, DISARMED.',flush=True)
                return state
            last='MCU responded but expected firmware/disarmed reboot was not confirmed'
        except (OSError,RuntimeError) as exc:
            last=str(exc)
        time.sleep(.5)
    raise RuntimeError('Transfer finished but reboot verification failed: '+last)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--host',help='ESP IP or lf-mcu.local (laptop must reach LUMI)')
    ap.add_argument('--mac',help='Expected MCU MAC; defaults to the provisioned board')
    ap.add_argument('--build',action='store_true',help='Build hybrid firmware before checking/uploading')
    mode=ap.add_mutually_exclusive_group()
    mode.add_argument('--check',action='store_true',help='Validate local image only; no network or upload')
    mode.add_argument('--status',action='store_true',help='Read MCU status only; no upload')
    ap.add_argument('--binary',type=Path,default=ROOT/'.pio/build'/ENV/'firmware.bin')
    ap.add_argument('--credentials',type=Path,default=ROOT/'.local/hybrid_ota.json')
    ap.add_argument('--platformio-home',type=Path,default=Path.home()/'.platformio')
    ap.add_argument('--listen-port',type=int,default=3233,help='Laptop TCP port for the MCU callback')
    args=ap.parse_args()
    if not 1<=args.listen_port<=65535:
        ap.error('listen-port must be 1..65535')
    credentials=json.loads(args.credentials.read_text())
    host=args.host or credentials.get('host','lf-mcu.local')
    expected_mac=args.mac or credentials.get('mac')
    console=Console(host)
    if args.status:
        print(json.dumps(console.request(),indent=2));return
    if args.build:
        subprocess.run([str(args.platformio_home/'penv/bin/pio'),'run','-e',ENV],cwd=ROOT,check=True)
    manifest=validate_image(args.binary,credentials)
    print(f'Image verified: {manifest["bytes"]} bytes; SHA256 {manifest["sha256"]}',flush=True)
    if args.check:
        return
    vendor=args.platformio_home/'packages/framework-arduinoespressif32/tools/espota.py'
    spec=importlib.util.spec_from_file_location('arduino_espota',vendor)
    ota=importlib.util.module_from_spec(spec);spec.loader.exec_module(ota)
    ota.TIMEOUT=2;ota.PROGRESS=True
    remote=socket.gethostbyname(host)
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as probe:
        probe.connect((remote,3232));local=probe.getsockname()[0]
    # Pass credentials in process memory, never as a shell argument or log line.
    upload=lambda:ota.serve(remote,local,3232,args.listen_port,credentials['password'],str(args.binary),ota.FLASH)
    update(console,expected_mac,manifest,upload)


if __name__=='__main__':
    logging.basicConfig(level=logging.INFO,format='%(message)s')
    try:
        main()
    except (OSError,ValueError,RuntimeError,subprocess.CalledProcessError) as exc:
        print(f'ERROR: {exc}',file=sys.stderr)
        sys.exit(1)
