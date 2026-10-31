#!/usr/bin/env python3
"""Run the LF hybrid core with UART or a local MCU emulator connection."""
import argparse
import fcntl
import gc
import json
import os
from pathlib import Path
import select
import signal
import socket
import stat
import sys
import time

from hybrid.core import Core
from hybrid.logic import Logic
from hybrid.policy import Policy
from hybrid.protocol import Parser, TELEMETRY
from hybrid.transport import Writer, open_serial
from hybrid.web import HybridWebServer

ROOT = Path(__file__).resolve().parent


def log(event, **data):
    print(json.dumps({'event':event,**data}),file=sys.stderr,flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config',default=str(ROOT/'config.json'))
    ap.add_argument('--serial',help='physical UART; overrides configured transport')
    ap.add_argument('--socket',help='Unix socket for MCU emulator')
    ap.add_argument('--listen-tcp',type=int,help='loopback-only HIL port, use SSH forwarding')
    ap.add_argument('--backend',choices=['rknn','onnx'])
    ap.add_argument('--logic',default=str(ROOT/'bin/hybrid_logic'))
    ap.add_argument('--status-file')
    ap.add_argument('--command-socket',default='/tmp/rb-hybrid-command.sock',
                    help='local operator command socket; independent of HCR1 UART')
    ap.add_argument('--web-host',default='127.0.0.1',
                    help='dashboard bind address; default is loopback for SSH forwarding')
    ap.add_argument('--web-port',type=int,default=8080)
    ap.add_argument('--web-token-file',default='/tmp/rb-hybrid-web.token')
    ap.add_argument('--no-web',action='store_true',help='disable the operator dashboard')
    ap.add_argument('--realtime-priority',type=int)
    args=ap.parse_args()
    cfg=json.loads(Path(args.config).read_text())
    if args.backend: cfg['backend']=args.backend
    if args.serial: cfg['transport'],cfg['serial_port']='serial',args.serial
    if args.socket: cfg['transport'],cfg['socket']='unix',args.socket
    if args.status_file: cfg['status_file']=args.status_file
    if args.realtime_priority is not None: cfg['realtime_priority']=args.realtime_priority
    if not (0<=cfg['home_hold_s']<cfg['warmup_start_s']<cfg['handoff_earliest_s']):
        raise ValueError('invalid phase timings')
    if cfg['warmup_steps']<50 or cfg['inference_deadline_ms']>10:
        raise ValueError('policy requires at least 50 warm-up ticks and a 10 ms deadline')
    if not 10<=cfg['inference_hard_deadline_ms']<=15 or not 1<=cfg['max_consecutive_deadline_misses']<=3:
        raise ValueError('invalid bounded jitter budget')
    lock=open(cfg['status_file']+'.lock','a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    listener=connection=command_listener=command_connection=web=None
    fd=None
    policy=logic=None
    stop=False
    def terminate(*_):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGTERM,terminate)
    signal.signal(signal.SIGINT,terminate)
    try:
        priority=cfg.get('realtime_priority',0)
        if priority:
            os.sched_setscheduler(0,os.SCHED_FIFO,os.sched_param(priority))
        policy=Policy(ROOT,cfg['backend'])
        logic=Logic(args.logic)
        core=Core(policy,logic,cfg)
        # Control loop owns no growing Python reference cycles; avoid periodic GC stalls.
        gc.collect()
        gc.disable()
        if args.listen_tcp is not None:
            listener=socket.socket()
            listener.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            listener.bind(('127.0.0.1',args.listen_tcp))
        elif cfg['transport']=='unix':
            path=Path(cfg['socket'])
            if path.exists():
                if not stat.S_ISSOCK(path.stat().st_mode):
                    raise RuntimeError('socket path occupied by a regular file')
                probe=socket.socket(socket.AF_UNIX)
                try:
                    probe.connect(str(path))
                except ConnectionRefusedError:
                    path.unlink()
                else:
                    raise RuntimeError('socket is already served')
                finally:
                    probe.close()
            listener=socket.socket(socket.AF_UNIX)
            listener.bind(str(path))
            os.chmod(path,0o600)
        elif cfg['transport']=='serial' and cfg['serial_port']:
            fd=open_serial(cfg['serial_port'],cfg['baud'])
        else:
            raise ValueError('UART port must be configured before selecting serial transport')
        command_path=Path(args.command_socket)
        if command_path.exists():
            if not stat.S_ISSOCK(command_path.stat().st_mode):
                raise RuntimeError('command socket path occupied by a regular file')
            probe=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
            try:
                probe.connect(str(command_path))
            except ConnectionRefusedError:
                command_path.unlink()
            else:
                raise RuntimeError('operator command socket is already served')
            finally:
                probe.close()
        command_listener=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
        command_listener.bind(str(command_path))
        os.chmod(command_path,0o600)
        command_listener.listen(2)
        command_listener.setblocking(False)
        if not args.no_web:
            web=HybridWebServer(args.web_host,args.web_port,args.web_token_file,
                                cfg['status_file'],args.command_socket,
                                ROOT/'web/index.html')
            web.start()
        if listener:
            listener.listen(1)
            listener.setblocking(False)
        writer=Writer(fd) if fd is not None else None
        parser=Parser()
        last_hello=last_status=0.
        last_mode=None
        max_control_ms=0.
        max_telemetry_ms=0.
        log('ready',backend=cfg['backend'],transport='loopback-tcp' if args.listen_tcp else cfg['transport'],
            web=None if args.no_web else f'http://{args.web_host}:{args.web_port}',
            token_file=None if args.no_web else args.web_token_file)
        while not stop:
            now=time.monotonic()
            if writer:
                try:
                    writer.add(core.watchdog(now))
                    if not core.link and now-last_hello>=1:
                        writer.add([core.hello()]);last_hello=now
                    if now-last_status>=1:
                        writer.add([core.status()])
                    writer.flush()
                except (OSError,RuntimeError) as error:
                    core.fail('transport_fault',str(error))
                    if listener:
                        connection.close();connection=None;fd=None;writer=None
                    else:
                        raise
            if now-last_status>=1 or core.mode!=last_mode:
                snapshot=core.snapshot()
                snapshot.update(transport=cfg['transport'],parser_errors=parser.errors,
                                max_handler_ms=max_control_ms,max_telemetry_ms=max_telemetry_ms,updated_monotonic=now)
                temp=Path(cfg['status_file']+'.tmp')
                temp.write_text(json.dumps(snapshot,indent=2)+'\n')
                temp.replace(cfg['status_file'])
                if core.mode!=last_mode:
                    log('mode',**snapshot)
                last_mode=core.mode;last_status=now
            reads=[command_listener]
            if fd is not None: reads.append(fd)
            elif listener: reads.append(listener)
            if command_connection: reads.append(command_connection)
            ready,_,_=select.select(reads,[],[],.005)
            if not ready:
                continue
            for ready_fd in ready:
                if ready_fd is command_listener:
                    command_connection,_=command_listener.accept()
                    command_connection.setblocking(False)
                    continue
                if ready_fd is command_connection:
                    try:
                        data=command_connection.recv(128)
                        if data:
                            line=data.decode('ascii','strict').strip().split()
                            op=line[0].upper() if line else ''
                            vx,wz=(float(line[1]),float(line[2])) if op=='VELOCITY' and len(line)==3 else (0.,0.)
                            if not line or (op=='VELOCITY' and len(line)!=3) or op not in ('ARM','DISARM','RESET','ESTOP','VELOCITY'):
                                raise ValueError('use ARM, DISARM, RESET, ESTOP, or VELOCITY vx wz')
                            rejected=core.counters['rejected']
                            reply=core.estop() if op=='ESTOP' else core.command(op,time.monotonic(),vx,wz)
                            if writer: writer.add(reply);writer.flush()
                            accepted=core.counters['rejected']==rejected
                            command_connection.sendall((json.dumps({'accepted':accepted,'mode':core.mode,
                                'fault':core.reason})+'\n').encode())
                        command_connection.close();command_connection=None
                    except (OSError,UnicodeError,ValueError,IndexError) as error:
                        try: command_connection.sendall((json.dumps({'accepted':False,'error':str(error)})+'\n').encode())
                        except OSError: pass
                        command_connection.close();command_connection=None
                    continue
                if fd is not None and ready_fd == fd:
                    try:
                        data=os.read(fd,4096)
                        if not data: raise ConnectionError('MCU connection closed')
                        for frame in parser.feed(data):
                            begin=time.monotonic()
                            reply=core.receive(frame,begin)
                            duration=(time.monotonic()-begin)*1000
                            max_control_ms=max(max_control_ms,duration)
                            if frame.kind==TELEMETRY: max_telemetry_ms=max(max_telemetry_ms,duration)
                            if frame.kind==TELEMETRY and duration>15 and core.mode in ('HOME','STANDUP','WARMUP','MODEL'):
                                reply=[core.fail('control_deadline'),core.status()]
                            writer.add(reply)
                        writer.flush()
                    except (OSError,RuntimeError) as error:
                        if core.mode in ('DISARMED','WAIT_LINK'): core.new_epoch()
                        else: core.fail('transport_fault',str(error))
                        if listener:
                            connection.close();connection=None;fd=None;writer=None
                        else: raise
                elif fd is None and listener is not None and ready_fd is listener:
                    connection,_=listener.accept()
                    connection.setblocking(False)
                    fd=connection.fileno()
                    writer=Writer(fd);parser=Parser();core.new_epoch();last_hello=0.
        if writer:
            writer.add([core.fail('transport_fault','service stopping')])
            writer.flush()
    finally:
        if web:
            web.close()
        if connection: connection.close()
        elif fd is not None: os.close(fd)
        if listener:
            listener.close()
        if command_connection: command_connection.close()
        if command_listener:
            command_listener.close()
            try: command_path.unlink()
            except FileNotFoundError: pass
        if logic: logic.close()
        if policy: policy.close()
        lock.close()


if __name__=='__main__':
    main()
