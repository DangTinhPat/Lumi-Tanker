"""Run ON LF only after operator approval, HOME and valid sensors.
No retry; disarm after 3.25 s or observed FAULT. Does not reset/clear faults.
"""
import json
from pathlib import Path
import socket
import threading
import time

STATUS = Path('/tmp/rb-hybrid-status.json')

def _command(op):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(.5)
        client.connect('/tmp/rb-hybrid-command.sock')
        client.sendall((op+'\n').encode())
        data=b''
        while b'\n' not in data:
            part=client.recv(1024)
            if not part: raise RuntimeError('command connection closed')
            data+=part
        return json.loads(data)

command_lock=threading.Lock()
def command(op):
    with command_lock:
        return _command(op)

state=json.loads(STATUS.read_text())
assert state['mode']=='DISARMED' and state['fault_code']==0 and state['last_telemetry_flags']==7
assert time.monotonic()-state['updated_monotonic']<2
started=time.monotonic()
deadline=started+3.25
stop_guard=threading.Event()

def guard():
    if not stop_guard.wait(max(0,deadline-time.monotonic())):
        try: print(json.dumps({'guard_disarm':command('DISARM')}),flush=True)
        except Exception as error: print(json.dumps({'guard_error':str(error)}),flush=True)

watcher=threading.Thread(target=guard)
watcher.start()
try:
    response=command('ARM')
    print(json.dumps({'arm':response,'started':started}),flush=True)
    assert response.get('accepted'),response
    previous=None
    while time.monotonic()<deadline:
        state=json.loads(STATUS.read_text())
        signature=(state['mode'],state['fault_code'],state.get('last_telemetry_flags'))
        if signature!=previous:
            print(json.dumps({'elapsed':time.monotonic()-started,'state':state}),flush=True)
            previous=signature
        if state['mode']=='FAULT':break
        time.sleep(.02)
finally:
    try: print(json.dumps({'disarm':command('DISARM'),'elapsed':time.monotonic()-started}),flush=True)
    finally:
        stop_guard.set()
        watcher.join()
