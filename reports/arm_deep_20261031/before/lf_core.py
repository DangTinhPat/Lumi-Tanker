"""Telemetry-driven 200 Hz hybrid state machine; policy ticks at 100 Hz."""
import math
import secrets
import struct
import time
from collections import deque
from . import protocol as p
from .policy import observation, encode_targets, decode_actions

MODES = {'WAIT_LINK':0, 'DISARMED':1, 'HOME':2, 'STANDUP':3, 'WARMUP':4, 'MODEL':5, 'FAULT':6}
PHASES = ['Idle', 'Folding', 'Kneeling', 'Catching', 'Support', 'Failed']
FAULTS = {'telemetry_timeout':1, 'invalid_telemetry':2, 'driver_fault':3,
          'clock_gap':4, 'missing_applied_target':5, 'logic_fault':6,
          'policy_fault':7, 'policy_deadline':8, 'fall':9, 'transport_fault':10,
          'control_deadline':11, 'ack_timeout':12}


class Core:
    def __init__(self, policy, logic, config):
        self.policy, self.logic, self.config = policy, logic, config
        self.tx_seq = 0
        self.counters = {'telemetry':0, 'telemetry_received':0, 'rejected':0, 'inferences':0, 'deadline_misses':0}
        self.new_epoch()

    def new_epoch(self):
        self.policy.reset()
        self.logic.reset()
        self.epoch = secrets.randbits(32) or 1
        self.mode, self.fault, self.reason = 'WAIT_LINK', 0, ''
        self.link = False
        self.rx_seq = None
        self.last_stamp = self.last_rx = self.last_t = self.start_stamp = None
        self.last_infer_stamp = None
        self.next_infer_stamp = None
        self.last_ack_rx = self.last_ack = None
        self.first_target_seq = None
        self.sent_targets = deque(maxlen=32)
        self.support, self.phase = False, 'Idle'
        self.action, self.velocity = [0.]*6, [0., 0.]
        self.target = [0.]*6
        self.handoff_us = None
        self.inference_max_ms = 0.
        self.consecutive_misses = 0
        self.offset_us = None
        self.last_telemetry_flags = None

    def frame(self, kind, payload):
        seq = self.tx_seq
        self.tx_seq = (self.tx_seq+1) & 0xffffffff
        return p.Frame(kind, seq, time.monotonic_ns()//1000, payload)

    def hello(self):
        return self.frame(p.HELLO, p.HELLO_DATA.pack(self.epoch, 200, 100, p.SCHEMA))

    def status(self):
        return self.frame(p.STATUS, p.STATUS_DATA.pack(self.epoch, MODES[self.mode],
                         self.fault, self.counters['rejected']))

    def output(self, seq, profile=0, flags=0, target=None, ff=None):
        """Send 4 absolute hip/crank angles + 2 wheel angular rates.

        ff is hipL/hipR/kneeL/kneeR angular velocity feedforward for recovery.
        Neither target nor ff contains a torque command.
        """
        frame = self.frame(p.TARGET, p.TARGET_DATA.pack(self.epoch, seq, profile, flags,
                          *((target or [0.]*6)+(ff or [0.]*4))))
        if profile and self.first_target_seq is None:
            self.first_target_seq = frame.seq
        if profile:
            self.sent_targets.append(frame.seq)
        return frame

    def fail(self, name, detail=''):
        if self.mode != 'FAULT':
            self.fault, self.reason = FAULTS[name], detail or name
        self.mode = 'FAULT'
        return self.output(self.rx_seq or 0)

    def watchdog(self, now):
        if self.mode in ('HOME','STANDUP','WARMUP','MODEL'):
            if self.last_rx is None or now-self.last_rx > self.config['telemetry_timeout_ms']/1000:
                return [self.fail('telemetry_timeout'), self.status()]
        return []

    def receive(self, frame, now):
        # MCU can reboot while LF retains its old sequence/timestamp baseline.
        if frame.kind == p.HELLO:
            if frame.payload == p.HELLO_DATA.pack(0,200,100,p.SCHEMA):
                disabled=self.output(frame.seq)
                self.new_epoch()
                return [disabled,self.hello(),self.status()]
            self.counters['rejected'] += 1
            return []
        if frame.kind not in (p.HELLO_ACK, p.TELEMETRY):
            self.counters['rejected'] += 1
            return []
        if len(frame.payload) < 4 or struct.unpack_from('<I', frame.payload)[0] != self.epoch:
            self.counters['rejected'] += 1
            return []
        if not p.newer(frame.seq, self.rx_seq):
            self.counters['rejected'] += 1
            return []
        self.rx_seq = frame.seq
        try:
            if frame.kind == p.HELLO_ACK:
                if p.HELLO_DATA.unpack(frame.payload) != (self.epoch, 200, 100, p.SCHEMA):
                    raise ValueError('interface mismatch')
                if not self.link:
                    self.link, self.mode = True, 'DISARMED'
                return [self.status()]
            if not self.link:
                self.counters['rejected'] += 1
                return [self.hello()]
            self.counters['telemetry_received'] += 1
            return self.telemetry(frame, now)
        except (ValueError, struct.error, OverflowError) as error:
            return [self.fail('invalid_telemetry', str(error)), self.status()]

    def command(self, op, now, vx=0., wz=0.):
        """Handle operator intent received locally on LF, outside HCR1 UART."""
        if not math.isfinite(vx) or not math.isfinite(wz):
            raise ValueError('nonfinite command')
        if op not in ('ARM','DISARM','RESET','VELOCITY'):
            self.counters['rejected'] += 1
            return [self.status()]
        if op == 'RESET':
            disabled = self.output(self.rx_seq or 0)
            self.new_epoch()
            return [disabled, self.hello(), self.status()]
        if op == 'DISARM':
            # Faults remain latched until an explicit RESET.
            if self.mode != 'FAULT':
                self.mode = 'DISARMED'
                self.policy.reset()
                self.logic.reset()
            self.velocity = [0.,0.]
            return [self.output(self.rx_seq or 0), self.status()]
        if op == 'ARM' and self.mode == 'DISARMED':
            if self.last_rx is None or now-self.last_rx > self.config['telemetry_timeout_ms']/1000:
                self.counters['rejected'] += 1
                return [self.status()]
            if any(abs(q-h)>self.config['home_tolerance_rad'] for q,h in zip(self.last_t.q[:4],self.config['home_joints'])):
                self.counters['rejected'] += 1
                return [self.status()]
            self.policy.reset()
            self.start_stamp = None
            self.last_infer_stamp = None
            self.next_infer_stamp = None
            self.first_target_seq = self.last_ack = self.last_ack_rx = None
            self.sent_targets.clear()
            self.action, self.velocity = [0.]*6, [0.,0.]
            self.support, self.phase, self.handoff_us = False, 'Idle', None
            self.mode = 'HOME'
            return [self.status()]
        if op == 'VELOCITY' and self.mode in ('HOME','STANDUP','WARMUP','MODEL'):
            if abs(vx)>2 or abs(wz)>math.pi:
                raise ValueError('command outside trained range')
            self.velocity = [vx,wz]
            return [self.status()]
        self.counters['rejected'] += 1
        return [self.status()]

    def estop(self):
        return [self.fail('driver_fault', 'operator E-stop'), self.status()]

    def telemetry(self, frame, now):
        t = p.Telemetry.decode(frame.payload)
        self.last_telemetry_flags = t.flags
        if t.flags & (p.ESTOP | p.DRIVER_FAULT):
            return [self.fail('driver_fault'), self.status()]
        if t.flags & 7 != 7:
            raise ValueError('IMU/encoder/calibration validity missing')
        norm = math.sqrt(sum(x*x for x in t.gravity))
        if not .9 <= norm <= 1.1:
            raise ValueError('gravity must be a body-frame unit vector')
        if self.last_stamp is not None:
            dt = frame.stamp-self.last_stamp
            if dt <= 0 or dt > self.config['telemetry_timeout_ms']*1000:
                return [self.fail('clock_gap'), self.status()]
        # Clock offset estimated without assuming synchronized MCU/Linux clocks.
        offset = now*1e6-frame.stamp
        self.offset_us = offset if self.offset_us is None else min(offset,self.offset_us)
        if offset-self.offset_us > self.config['telemetry_timeout_ms']*1000:
            return [self.fail('clock_gap','queued telemetry exceeds age limit'), self.status()]
        self.last_stamp, self.last_rx, self.last_t = frame.stamp, now, t
        self.counters['telemetry'] += 1
        if self.mode in ('DISARMED','FAULT','WAIT_LINK'):
            return [self.output(frame.seq)]
        if self.start_stamp is None:
            self.start_stamp = frame.stamp
            self.next_infer_stamp = frame.stamp+round(self.config['warmup_start_s']*1e6)
            self.last_ack_rx = now
        elapsed = (frame.stamp-self.start_stamp)/1e6
        if self.first_target_seq is not None:
            valid_ack = t.applied_seq in self.sent_targets
            if valid_ack and (self.last_ack is None or p.newer(t.applied_seq,self.last_ack)):
                self.last_ack, self.last_ack_rx = t.applied_seq, now
            if now-self.last_ack_rx > self.config['target_ack_timeout_ms']/1000:
                return [self.fail('ack_timeout'), self.status()]
        if elapsed >= self.config['warmup_start_s']:
            if not t.flags & p.TARGET_VALID:
                return [self.fail('missing_applied_target'), self.status()]
        # The simulator stops on a fall; keep that behaviour instead of automatic rearming.
        if self.mode == 'MODEL' and -t.gravity[2]/norm < math.cos(math.radians(70)):
            return [self.fail('fall'), self.status()]
        due = frame.stamp >= self.next_infer_stamp
        if elapsed >= self.config['warmup_start_s'] and due:
            if self.mode != 'MODEL':
                self.action = encode_targets(t.applied_target)
            handoff_ready = (elapsed >= self.config['handoff_earliest_s'] and self.support and
                    self.policy.steps >= self.config['warmup_steps'] and self.mode != 'MODEL')
            try:
                prediction = self.policy.step(observation(t,self.action,self.velocity))
            except Exception as error:
                return [self.fail('policy_fault',str(error)),self.status()]
            self.counters['inferences'] += 1
            self.inference_max_ms = max(self.inference_max_ms,self.policy.last_ms)
            self.last_infer_stamp = frame.stamp
            # Keep an absolute 100 Hz schedule despite telemetry timestamp jitter.
            self.next_infer_stamp += 10000
            while self.next_infer_stamp <= frame.stamp:
                self.next_infer_stamp += 10000
            if self.policy.last_ms > self.config['inference_deadline_ms']:
                self.counters['deadline_misses'] += 1
                self.consecutive_misses += 1
                if (self.policy.last_ms > self.config['inference_hard_deadline_ms'] or
                        self.consecutive_misses >= self.config['max_consecutive_deadline_misses']):
                    native_ms=getattr(getattr(self.policy,'model',None),'last_native_ms',None)
                    return [self.fail('policy_deadline',f'inference {self.policy.last_ms:.3f} ms; native {native_ms} ms'),self.status()]
                # Discard a late prediction, retain the last actually commanded action.
                # GRU has observed this sample; do not roll its memory backwards.
            else:
                self.consecutive_misses = 0
                if handoff_ready:
                    self.mode, self.handoff_us = 'MODEL', frame.stamp
                if self.mode == 'MODEL':
                    self.action = prediction
                    self.target = decode_actions(prediction)
        if elapsed < self.config['home_hold_s']:
            self.mode = 'HOME'
            h = self.config['home_joints']
            self.target = [h[0],h[1],h[0]+h[2],h[1]+h[3],0.,0.]
            return [self.output(frame.seq,p.HOME_PD,3,self.target)]
        if self.mode == 'MODEL':
            return [self.output(frame.seq,p.POLICY_PD,3,self.target)]
        try:
            self.support, phase, flags, self.target, ff = self.logic.step(elapsed,t)
            self.phase = PHASES[phase] if 0<=phase<len(PHASES) else 'Unknown'
        except Exception as error:
            return [self.fail('logic_fault',str(error)),self.status()]
        self.mode = 'WARMUP' if elapsed >= self.config['warmup_start_s'] else 'STANDUP'
        return [self.output(frame.seq,p.RECOVERY,flags,self.target,ff)]

    def snapshot(self):
        return {'epoch':self.epoch,'mode':self.mode,'phase':self.phase,'fault_code':self.fault,
                'reason':self.reason,'policy_steps':self.policy.steps,'handoff_us':self.handoff_us,
                'inference_max_ms':self.inference_max_ms,'consecutive_deadline_misses':self.consecutive_misses,
                'counters':self.counters, 'link':self.link,
                'last_telemetry_flags':self.last_telemetry_flags}
