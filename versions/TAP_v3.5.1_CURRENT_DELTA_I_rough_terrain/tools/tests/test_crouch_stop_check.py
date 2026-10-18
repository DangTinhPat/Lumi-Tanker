import importlib.util
import math
import tempfile
from pathlib import Path
import unittest
spec=importlib.util.spec_from_file_location('check',Path(__file__).parents[1]/'crouch_stop_check.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class StopCheckTests(unittest.TestCase):
    def trace(self, end=8):
        return [dict(t=i*.04,h=max(95,150-20*i*.04),hip=.01*math.sin(i*.04*4*math.pi),
                     tvr=.1*math.exp(max(0,i*.04-3)*.4)*math.sin(i*.04*4*math.pi),
                     rate=0.,lean=0.) for i in range(int(end/.04))]
    def test_stop_and_growth(self):
        es=m.events(self.trace())
        self.assertEqual(len(es),1);self.assertTrue(es[0]['complete'])
        self.assertAlmostEqual(es[0]['height_mm'],95)
        self.assertGreater(es[0]['half_cycle_growth'],1)
        self.assertAlmostEqual(es[0]['frequency_hz'],2,delta=.3)
    def test_censored_is_not_pass(self):
        es=m.events(self.trace(4))
        self.assertEqual(len(es),1);self.assertFalse(es[0]['complete'])
    def test_constant_height_no_event(self):
        rows=self.trace()
        for r in rows:r['h']=95
        self.assertEqual(m.events(rows),[])
    def test_monotone_move_no_stop(self):
        rows=self.trace()
        for r in rows:r['h']=150-20*r['t']
        self.assertEqual(m.events(rows),[])
    def test_device_clock_fault_reset_and_drive(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'robot.log'
            path.write_text(
                '1900000000 >> DRIVE 0.2 0\n'
                '1900000001 TR t=1000 h=110 dhip=0 tvr=0.1 tv=0 rate=0\n'
                '1900000002 TR t=1040 h=110 dhip=0 tvr=0.1 tv=0 rate=0\n'
                '1900000003 CONTROL mode=Fault\n'
                '1900000004 >> DRIVE 0 0\n'
                '1900000005 TR t=20 h=100 dhip=0 tvr=0.1 tv=0 rate=0\n')
            segments=m.read_robot(path)
            self.assertEqual(len(segments),2)
            self.assertEqual(segments[0][0]['t'],1.)
            self.assertEqual(segments[1][0]['t'],.02)
            self.assertTrue(segments[0][0]['drive_active'])
            self.assertFalse(segments[1][0]['drive_active'])
if __name__=='__main__':unittest.main()
