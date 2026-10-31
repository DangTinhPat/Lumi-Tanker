import json
import math
from pathlib import Path
import unittest
from hybrid import protocol as p
from hybrid.core import Core
from hybrid.policy import encode_targets,decode_actions,observation

ROOT=Path(__file__).resolve().parents[1]

class FakePolicy:
    def reset(self): self.steps=0;self.hidden=0;self.last_ms=.1
    def step(self,obs):
        self.last_obs=obs;self.hidden+=1;self.steps+=1
        return [.1,.2,.3,.4,.5,.6]

class FakeLogic:
    def reset(self): pass
    def step(self,elapsed,t): return elapsed>=8,4,3,[-.7,-.7,.2,.2,0.,0.],[0.]*4

class Rig:
    def __init__(self):
        self.policy=FakePolicy()
        self.core=Core(self.policy,FakeLogic(),json.loads((ROOT/'config.json').read_text()))
        self.seq=0;self.stamp=100000;self.ack=0
        self.t=p.Telemetry(self.core.epoch,7|p.TARGET_VALID,0,[0.]*3,[0.,0.,-1.],
            [0.,0.,9.81],[2.35706715,2.35706715,0.,0.,0.,0.],[0.]*6,[0.,0.],[-.7,-.7,.2,.2,0.,0.])
        self.send(p.HELLO_ACK,p.HELLO_DATA.pack(self.core.epoch,200,100,p.SCHEMA))
        self.telemetry()
    def send(self,kind,payload):
        self.seq+=1
        f=p.Frame(kind,self.seq,self.stamp,payload)
        out=self.core.receive(f,self.stamp/1e6)
        for r in out:
            if r.kind==p.TARGET: self.ack=r.seq
        return out
    def command(self,op):
        return self.core.command(op,self.stamp/1e6)
    def telemetry(self,delta=5000):
        self.stamp+=delta;self.t.applied_seq=self.ack
        return self.send(p.TELEMETRY,self.t.encode())

class ProtocolTests(unittest.TestCase):
    def test_fragmentation_corruption_resync(self):
        a=p.Frame(0x55,0xffffffff,123,b'unknown-frame')
        raw=a.encode();bad=bytearray(raw);bad[-1]^=0x55
        parser=p.Parser();got=[]
        for x in b'noise'+bad+raw:
            got+=parser.feed(bytes([x]))
        self.assertEqual(len(got),1);self.assertEqual(got[0],a)
        self.assertGreater(parser.errors,0)
        self.assertTrue(p.newer(0,0xffffffff));self.assertFalse(p.newer(2,3))
    def test_action_and_joint_contract(self):
        a=[2.,-2.,.4,-.2,.3,-.7]
        for x,y in zip(a,encode_targets(decode_actions(a))): self.assertAlmostEqual(x,y)
        r=Rig();r.t.q=[-.6,-.5,1.2,1.3,200.,300.]
        r.t.qd=[1.,2.,3.,4.,5.,6.]
        o=observation(r.t,a,[.5,.8])
        self.assertEqual(len(o),32);self.assertEqual(o[13:19],r.t.qd)
        self.assertEqual(o[19:25],a);self.assertEqual(o[28:32],[0.]*4)

class StateTests(unittest.TestCase):
    def test_faulty_sensor_frames_are_counted_as_received_not_control_valid(self):
        r=Rig();valid=r.core.counters['telemetry'];received=r.core.counters['telemetry_received']
        r.t.flags=p.DRIVER_FAULT
        r.telemetry()
        self.assertEqual(r.core.counters['telemetry_received'],received+1)
        self.assertEqual(r.core.counters['telemetry'],valid)
        self.assertEqual(r.core.snapshot()['last_telemetry_flags'],p.DRIVER_FAULT)
        self.assertEqual(r.core.mode,'FAULT')
        self.assertTrue(r.core.snapshot()['link'])
        r.core.receive(p.Frame(p.TELEMETRY,r.seq,r.stamp,r.t.encode()),r.stamp/1e6)
        self.assertEqual(r.core.counters['telemetry_received'],received+1) # replay excluded

    def test_hybrid_warmup_handoff_preserves_state(self):
        r=Rig();r.command('ARM')
        for _ in range(2500):
            r.telemetry()
            self.assertNotEqual(r.core.mode,'FAULT',r.core.reason)
            self.assertEqual(r.policy.steps,0)
        for _ in range(100):
            r.telemetry()
            self.assertEqual(r.core.mode,'WARMUP')
        self.assertEqual(r.policy.steps,50)
        self.assertEqual(r.policy.hidden,50)
        r.telemetry()
        self.assertEqual(r.core.mode,'MODEL')
        self.assertEqual(r.policy.hidden,51)
        for x,y in zip(r.policy.last_obs[19:25],encode_targets(r.t.applied_target)):
            self.assertAlmostEqual(x,y,places=6)
        r.telemetry();r.telemetry()
        self.assertEqual(r.policy.last_obs[19:25],[.1,.2,.3,.4,.5,.6])
    def test_timeout_fault_latches_until_reset(self):
        r=Rig();r.command('ARM');r.telemetry()
        r.core.watchdog(r.stamp/1e6+.031)
        self.assertEqual(r.core.mode,'FAULT')
        r.command('ARM');r.command('DISARM')
        self.assertEqual(r.core.mode,'FAULT')
        old_epoch=r.core.epoch;r.command('RESET')
        self.assertEqual(r.core.mode,'WAIT_LINK');self.assertNotEqual(r.core.epoch,old_epoch)
    def test_replayed_and_old_epoch_data_do_not_refresh_watchdog(self):
        r=Rig();r.command('ARM');r.telemetry();previous=r.core.last_rx
        f=p.Frame(p.TELEMETRY,r.seq,r.stamp,r.t.encode())
        self.assertFalse(r.core.receive(f,previous+.01))
        r.t.epoch^=1
        f=p.Frame(p.TELEMETRY,r.seq+1,r.stamp+5000,r.t.encode())
        self.assertFalse(r.core.receive(f,previous+.02))
        self.assertEqual(r.core.last_rx,previous)
    def test_invalid_sensor_and_estop(self):
        for change in ('nan','estop','gravity'):
            r=Rig();r.command('ARM')
            if change=='nan':r.t.gyro[0]=float('nan')
            elif change=='estop':r.t.flags|=p.ESTOP
            else:r.t.gravity=[0.,0.,0.]
            r.telemetry();self.assertEqual(r.core.mode,'FAULT')
    def test_future_ack_cannot_keep_motor_link_alive(self):
        r=Rig();r.command('ARM');r.telemetry()
        for _ in range(8):
            r.ack=(r.core.tx_seq+999)&0xffffffff
            r.telemetry()
        self.assertEqual(r.core.reason,'ack_timeout')
    def test_mcu_reboot_starts_new_epoch_despite_old_sequence(self):
        r=Rig();r.command('ARM');r.telemetry();epoch=r.core.epoch
        f=p.Frame(p.HELLO,0,0,p.HELLO_DATA.pack(0,200,100,p.SCHEMA))
        r.core.receive(f,r.stamp/1e6)
        self.assertNotEqual(r.core.epoch,epoch);self.assertEqual(r.core.mode,'WAIT_LINK')
    def test_absolute_policy_schedule_tolerates_timestamp_jitter(self):
        r=Rig();r.command('ARM')
        for _ in range(2500):r.telemetry()
        for i in range(200):r.telemetry(5001 if i%2 else 4999)
        self.assertEqual(r.policy.steps,100)
    def test_missing_target_feedback_and_inference_deadline_fault(self):
        for case in ('feedback','deadline'):
            r=Rig();r.command('ARM')
            for _ in range(2500):r.telemetry()
            if case=='feedback':r.t.flags&=~p.TARGET_VALID
            else:r.policy.last_ms=16.
            r.telemetry();self.assertEqual(r.core.mode,'FAULT')
    def test_late_predictions_are_held_and_repeated_lateness_faults(self):
        r=Rig();r.command('ARM')
        for _ in range(2601):r.telemetry()
        self.assertEqual(r.core.mode,'MODEL')
        target=r.core.target.copy();action=r.core.action.copy()
        r.policy.last_ms=11.
        for _ in range(4):r.telemetry()
        self.assertEqual(r.core.mode,'MODEL')
        self.assertEqual(r.core.target,target);self.assertEqual(r.core.action,action)
        self.assertEqual(r.core.counters['deadline_misses'],2)
        r.telemetry();r.telemetry();self.assertEqual(r.core.mode,'FAULT')
    def test_late_first_policy_prediction_does_not_handoff(self):
        r=Rig();r.command('ARM')
        for _ in range(2600):r.telemetry()
        r.policy.last_ms=11.;r.telemetry()
        self.assertEqual(r.core.mode,'WARMUP');self.assertIsNone(r.core.handoff_us)
        r.policy.last_ms=.1;r.telemetry();r.telemetry()
        self.assertEqual(r.core.mode,'MODEL')

if __name__=='__main__':unittest.main()
