#!/usr/bin/env python3
"""End-to-end, DISABLED-only UART check between Luckfox /dev/ttyS2 and ESP32."""
import argparse
import getpass
import json
import shlex
import socket
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent


def console_watch(host, port, seconds, result):
    """Collect ESP status and optional UART byte counters without motor commands."""
    try:
        with socket.create_connection((host, port), timeout=5) as sock:
            sock.settimeout(.2)
            buffer = bytearray()
            next_status = 0.0
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                now = time.monotonic()
                if now >= next_status:
                    sock.sendall(b'STATUS\n')
                    next_status = now + 1
                try:
                    data = sock.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    break
                buffer.extend(data)
                while b'\n' in buffer:
                    raw, _, rest = buffer.partition(b'\n')
                    buffer[:] = rest
                    line = raw.decode(errors='replace').strip()
                    if line.startswith('HCR '):
                        result['esp_status'] = line
                    elif line.startswith('IO '):
                        fields = dict(item.split('=',1) for item in line.split()[1:] if '=' in item)
                        result['esp_io'] = fields
    except OSError as exc:
        result['esp_console_error'] = str(exc)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--lf-ip', default='10.10.60.232')
    ap.add_argument('--esp-ip', default='10.10.61.85')
    ap.add_argument('--lf-user', default='root')
    ap.add_argument('--lf-password', help='Omit to enter it without echo')
    ap.add_argument('--lf-port', default='/dev/ttyS2')
    ap.add_argument('--console-port', type=int, default=8023)
    ap.add_argument('--seconds', type=float, default=15)
    ap.add_argument('--report', type=Path)
    args = ap.parse_args()
    if not 5 <= args.seconds <= 300:
        ap.error('--seconds must be between 5 and 300')
    try:
        import paramiko
    except ImportError as exc:
        raise SystemExit('Install Paramiko with: python3 -m pip install paramiko') from exc

    password = args.lf_password or getpass.getpass(f'SSH password for {args.lf_user}@{args.lf_ip}: ')
    report = {'lf': args.lf_ip, 'uart': args.lf_port, 'esp': args.esp_ip,
              'baud': 921600, 'format': '8N1', 'duration_s': args.seconds,
              'enabled_targets_sent': 0}
    console = threading.Thread(target=console_watch,
        args=(args.esp_ip,args.console_port,args.seconds+4,report),daemon=True)
    ssh = paramiko.SSHClient()
    ssh.load_system_host_keys()
    ssh.connect(args.lf_ip,username=args.lf_user,password=password,timeout=8,
                banner_timeout=8,auth_timeout=8)
    remote_dir = f'/tmp/hcr-uart-check-{int(time.time())}'
    sftp = ssh.open_sftp()
    try:
        sftp.mkdir(remote_dir)
        for name in ('hcr_uart_peer.py','hcr_protocol.py'):
            sftp.put(str(HERE/name),remote_dir+'/'+name)
    finally:
        sftp.close()

    command = 'python3 -u {} --port {} --seconds {}'.format(
        shlex.quote(remote_dir+'/hcr_uart_peer.py'),shlex.quote(args.lf_port),args.seconds)
    console.start()
    try:
        stdin, stdout, stderr = ssh.exec_command(command,timeout=args.seconds+20)
        stdin.close()
        output = stdout.read().decode(errors='replace')
        error = stderr.read().decode(errors='replace')
        exit_code = stdout.channel.recv_exit_status()
        console.join(timeout=args.seconds+5)
        report['peer_output'] = output.strip().splitlines()
        report['peer_error'] = error.strip()
        report['peer_exit_code'] = exit_code
        try:
            report['peer'] = json.loads(report['peer_output'][-1])
        except (IndexError,ValueError):
            report['peer'] = None
        report['esp_console_connected'] = 'esp_console_error' not in report
        report['pass'] = bool(report['peer'] and report['peer'].get('link') and
                              report['peer'].get('hello_acks',0)>0 and
                              report['peer'].get('telemetry_frames',0)>0 and
                              report['peer'].get('crc_errors')==0)
        if args.report:
            args.report.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))
        if not report['pass']:
            raise SystemExit(2)
    finally:
        try:
            sftp = ssh.open_sftp(); sftp.remove(remote_dir+'/hcr_uart_peer.py')
            sftp.remove(remote_dir+'/hcr_protocol.py'); sftp.rmdir(remote_dir); sftp.close()
        except OSError:
            pass
        ssh.close()
        console.join(timeout=args.seconds+5)


if __name__ == '__main__':
    main()
