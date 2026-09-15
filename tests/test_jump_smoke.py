"""Offline jump evidence contracts; no ROS, Gazebo, or controller process."""
import math
from pathlib import Path
import runpy
from types import SimpleNamespace
import unittest

import numpy as np

H = runpy.run_path(str(Path(__file__).resolve().parents[1]/'tools/jump_smoke.py'))


def row(t, height, contact=(False, False), x=0., pitch=0.):
    return {'sim': t, 'com': [x, 0., height], 'contacts': list(contact), 'pitch': pitch}


class JumpEvidenceTest(unittest.TestCase):
    def test_paused_startup_uses_bounded_wall_deadline(self):
        guard = H['clock_progress_guard']
        guard('startup', 30., 1., 0., 70.)
        with self.assertRaisesRegex(RuntimeError, 'Startup wall timeout'):
            guard('startup', 70.001, 70., 0., 70.)

    def test_clock_two_second_guard_unchanged_after_activation(self):
        guard = H['clock_progress_guard']
        for stage in ('settle', 'baseline', 'jump', 'recovery'):
            guard(stage, 3., 1., 0., 70.)
            with self.assertRaisesRegex(RuntimeError, 'Clock stopped'):
                guard(stage, 3.001, 1., 0., 70.)

    def test_local_damping_shared_model_arguments_and_zero_default(self):
        fn = H['jump_model_arguments']
        for damping in (0., .1, .2):
            self.assertEqual(fn(12., damping), ['jump_model:=true', 'jump_leg_speed:=12.0',
                                               f'jump_leg_velocity_damping:={damping}'])

    def test_local_damping_rejects_invalid_or_duplicate_override(self):
        fn = H['jump_model_arguments']
        for damping in (-.1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):fn(12., damping)
        with self.assertRaises(ValueError):
            fn(12., .1, ['jump_leg_velocity_damping:=.2'])

    def test_settle_allows_early_contact_transient_then_full_stable_second(self):
        window = H['SettlingWindow']()
        for t, contacts, ready in ((0., (True, False), 0), (.2, (True, True), 0),
                                    (7., (True, True), 1)):
            r = row(t, .2, contacts);r['jump_ready'] = ready
            self.assertFalse(window.update(r))
        r = row(8., .2, (True, True));r['jump_ready'] = 1
        self.assertTrue(window.update(r))
        self.assertEqual(len(window.transients), 2)

    def test_settle_late_transient_or_missing_ready_blocks_hold(self):
        for reason in ('contact', 'pitch', 'ready'):
            window = H['SettlingWindow']()
            r = row(0., .2, (True, True));r['jump_ready'] = 1;window.update(r)
            r = row(7.5, .2, (True, reason != 'contact'), pitch=.15 if reason == 'pitch' else 0.)
            r['jump_ready'] = None if reason == 'ready' else 1;window.update(r)
            r = row(8., .2, (True, True));r['jump_ready'] = 1
            with self.assertRaises(RuntimeError):window.update(r)

    def test_settle_does_not_relax_global_hard_pose_guard(self):
        H['hard_pose_guard'](.115, .5)
        for height, pitch in ((.1149, 0.), (.2, .501), (.2, -.501)):
            with self.assertRaises(RuntimeError):H['hard_pose_guard'](height, pitch)

    def test_ros_nested_and_dotted_parameters_are_equivalent(self):
        wrap = lambda p: {'balance_controller_node': {'ros__parameters': p}}
        normalize = H['canonical_ros_parameters']
        self.assertEqual(normalize(wrap({'station_keeper': {'enabled': True}, 'mpc': {'horizon': 20}})),
                         normalize(wrap({'station_keeper.enabled': True, 'mpc.horizon': 20})))

    def test_ros_later_layer_wins_in_both_spelling_directions(self):
        wrap = lambda p: {'balance_controller_node': {'ros__parameters': p}}
        normalize = H['canonical_ros_parameters']
        for base, override in (({'station_keeper': {'enabled': False, 'gain': .8}},
                                {'station_keeper.enabled': True}),
                               ({'station_keeper.enabled': False, 'station_keeper.gain': .8},
                                {'station_keeper': {'enabled': True}})):
            result = normalize(wrap(base), wrap(override))['balance_controller_node']['ros__parameters']
            self.assertEqual(result, {'station_keeper.enabled': True, 'station_keeper.gain': .8})

    def test_ros_parameter_arrays_preserved_and_inputs_not_mutated(self):
        source = {'balance_controller_node': {'ros__parameters': {'jump': {'pose': [1., 2.]}}}}
        result = H['canonical_ros_parameters'](source)
        result['balance_controller_node']['ros__parameters']['jump.pose'].append(3.)
        self.assertEqual(source['balance_controller_node']['ros__parameters']['jump']['pose'], [1., 2.])

    def test_startup_gate_timing_retained_without_assuming_gate_policy(self):
        events = [{'kind': 'actuator_enable', 'stage': 'startup', 'sim': t/100,
                   'wall': t, 'data': value} for t, value in ((0., False), (1., True), (1.01, True))]
        result = H['startup_diagnostics'](events)
        self.assertEqual(result['first_true_previous_false_wall_gap_s'], 1.)
        self.assertEqual(len(result['gate_transitions']), 2)
        self.assertEqual(result['contact_packets'], {'left': 0, 'right': 0})

    def test_startup_raw_commands_survive_zero_coherent_samples(self):
        events = [{'kind': 'legs', 'stage': 'startup', 'data': {'data': [3., -3., 2., -2.]}},
                  {'kind': 'joints', 'stage': 'startup', 'data': {'data': {
                      'positions': {'knee': -1.}, 'efforts': {'knee': 0.}}}}]
        result = H['startup_diagnostics'](events)
        self.assertEqual(result['peak_leg_command_nm'], [3., 3., 2., 2.])
        self.assertEqual(result['peak_applied_effort_nm'], {'knee': 0.})

    def test_applied_effort_caps_and_missing_evidence(self):
        fn = H['check_applied_efforts']
        required = ['cad_left_knee', 'cad_right_wheel']
        fn({'cad_left_knee': 5., 'cad_right_wheel': -.13}, required)
        for data in (None, {}, {'cad_left_knee': 5.01, 'cad_right_wheel': 0.},
                     {'cad_left_knee': 0., 'cad_right_wheel': .131}):
            with self.assertRaises(ValueError):
                fn(data, required)

    def test_empty_airborne_heartbeat_still_must_be_fresh(self):
        cache = {'left': {'stamp': 1., 'wall': 10., 'data': False},
                 'pose': {'stamp': 1., 'wall': 10.}}
        self.assertEqual(H['stale_sources'](cache, 1.06, 10.06, .15, .05), ['left'])
        self.assertEqual(H['stale_sources'](cache, 1.01, 13., .15, .05), ['left', 'pose'])

    def test_baseline_cannot_request_jump_and_edge_is_bounded(self):
        level = H['request_level']
        self.assertFalse(level(1., None))
        self.assertTrue(level(1., 1.))
        self.assertFalse(level(1.11, 1.))
        self.assertFalse(level(1., 1., True))

    def flight(self, peak=.36):
        f = H['FlightEvidence']()
        for r in (row(0, .2, (True, True)), row(.01, .21), row(.05, .25), row(.1, peak)):
            f.update(r)
        return f

    def test_height_requires_actual_flight(self):
        f = H['FlightEvidence']()
        f.update(row(0, .1, (True, True)))
        f.update(row(1, .4, (True, True)))
        self.assertFalse(f.summary()['height_pass'])

    def test_one_wheel_contact_is_not_flight(self):
        f = H['FlightEvidence']()
        f.update(row(0, .1, (True, True)))
        f.update(row(1, .4, (False, True)))
        self.assertIsNone(f.takeoff)

    def test_contact_dropout_does_not_count(self):
        f = H['FlightEvidence']()
        for r in (row(0, .2, (True, True)), row(.01, .21), row(.02, .22, (True, True))):
            f.update(r)
        self.assertIsNone(f.takeoff)

    def test_single_empty_contact_cannot_prove_sustained_flight(self):
        f = H['FlightEvidence']()
        f.update(row(0., .2, (True, True)))
        for t in (.01, .05):
            r = row(t, .3)
            r['contact_stamps'] = [.01, .01]
            f.update(r)
        self.assertIsNone(f.takeoff)

    def test_contact_pulse_between_coherent_samples_cancels_candidate(self):
        f = H['FlightEvidence']()
        f.update(row(0., .2, (True, True)))
        f.update(row(.01, .21))
        r = row(.05, .3)
        r['contact_ground_since_previous'] = [True, False]
        f.update(r)
        self.assertIsNone(f.takeoff)

    def test_ground_history_to_air_transition_detected_with_same_definition(self):
        f = H['FlightEvidence']()
        f.update(row(10.196, .191, (True, True)))
        transitional = row(10.2, .194)
        transitional['contact_ground_since_previous'] = [True, True]
        transitional['contact_stamps'] = [10.199, 10.199]
        f.update(transitional)
        for t, height in ((10.204, .1965), (10.224, .206), (10.24, .2116),
                          (10.264, .2148), (10.332, .1932)):
            airborne = row(t, height)
            airborne['contact_ground_since_previous'] = [False, False]
            airborne['contact_stamps'] = [t-.001, t-.001]
            f.update(airborne)
        self.assertIsNotNone(f.takeoff)
        self.assertEqual(f.summary()['takeoff_bracket_s'], [10.196, 10.204])
        self.assertFalse(f.height_pass)
        self.assertAlmostEqual(f.summary()['height_m'], .2148-.1965)
        f.update(row(10.336, .1906, (True, True)))
        self.assertAlmostEqual(f.summary()['landing_sim_s'], 10.336)


    def test_height_and_landing_separate(self):
        f = self.flight()
        self.assertTrue(f.height_pass)
        self.assertIsNone(f.landing)
        f.update(row(.3, .2, (True, False)))
        self.assertIsNotNone(f.landing)
        self.assertTrue(f.summary()['height_pass'])

    def test_ground_extension_not_credited(self):
        f = H['FlightEvidence']()
        for r in (row(0, .1, (True, True)), row(1, .3, (True, True)),
                  row(1.01, .31), row(1.05, .35)):
            f.update(r)
        self.assertFalse(f.height_pass)

    def test_strict_height_and_conservative_boundary(self):
        f = H['FlightEvidence']()
        for r in (row(0, 0., (True, True)), row(.01, 0.), row(.05, .1)):
            f.update(r)
        self.assertFalse(f.height_pass)
        f.update(row(.06, .1001))
        self.assertTrue(f.height_pass)
        self.assertAlmostEqual(self.flight().summary()['height_m'], .15)

    def test_recovery_fixed_anchor_not_reset(self):
        anchor = row(0, .2)
        self.assertFalse(H['recovery_metrics']([row(1, .2, (True, True), x=.03)], anchor)['passed'])
        self.assertFalse(H['recovery_metrics']([row(1, .2, (True, True), x=.06),
                                                row(2, .2, (True, True))], anchor)['passed'])
        self.assertTrue(H['recovery_metrics']([row(1, .2, (True, True), x=.01)], anchor)['passed'])

    def test_recovery_custom_drift_bounds_and_default_compatibility(self):
        metrics = H['recovery_metrics']
        anchor = row(0., .2)
        boundary = [row(1., .2, (True, True), x=.01), row(2., .2, (True, True), x=.002)]
        self.assertTrue(metrics(boundary, anchor, .01, .002)['passed'])
        for peak, final in ((.010001, .002), (.01, .002001)):
            rows = [row(1., .2, (True, True), x=peak), row(2., .2, (True, True), x=final)]
            self.assertFalse(metrics(rows, anchor, .01, .002)['passed'])
            self.assertTrue(metrics(rows, anchor)['passed'])

    def test_recovery_drift_bounds_must_be_positive_and_finite(self):
        for value in (0., -.01, float('nan'), float('inf')):
            for bounds in ((value, .02), (.05, value)):
                with self.assertRaises(ValueError):
                    H['recovery_metrics']([], row(0., .2), *bounds)

    def test_recovery_pitch_and_both_contacts(self):
        anchor = row(0, .2)
        for r in (row(1, .2, (True, True), pitch=.201), row(1, .2, (True, False))):
            self.assertFalse(H['recovery_metrics']([r], anchor)['passed'])

    def test_shuffled_missing_duplicate_nonfinite_joints(self):
        fn = H['joint_map']
        self.assertEqual(fn(['b', 'a'], [2., 1.], ['a', 'b']), {'a': 1., 'b': 2.})
        for names, values in ((['a'], [1.]), (['a', 'a'], [1., 2.]),
                              (['a', 'b'], [1.]), (['a', 'b'], [1., float('nan')])):
            with self.assertRaises(ValueError):
                fn(names, values, ['a', 'b'])

    def test_phase_contract(self):
        self.assertEqual(H['phase_value']('Flight|enabled=1'), 'Flight')
        self.assertEqual(H['phase_value']('jump|phase=Flight', 'phase'), 'Flight')
        with self.assertRaises(ValueError):
            H['phase_value']('Flight', 'phase')

    def test_actual_contacts_empty_heartbeat_and_wrong_sensor(self):
        fn = H['wheel_contact']
        packet = lambda a,b: SimpleNamespace(contacts=[SimpleNamespace(
            collision1=SimpleNamespace(name=a), collision2=SimpleNamespace(name=b))])
        self.assertFalse(fn(SimpleNamespace(contacts=[]), 'left', 'ground'))
        self.assertTrue(fn(packet('left_collision', 'ground_collision'), 'left', 'ground'))
        with self.assertRaises(ValueError):
            fn(packet('right_collision', 'ground_collision'), 'left', 'ground')
        with self.assertRaises(ValueError):
            fn(packet('left_collision', 'body_collision'), 'left', 'ground')

    def test_saturation_duration_not_sample_fraction(self):
        commands = [{'sim': t, 'values': [v], 'phase': p} for t,v,p in
                    ((0., 1., 'Push'), (.1, 3., 'Push'), (.11, 3., 'Flight'),
                     (.12, 0., 'Flight'), (.2, 0., 'Land'))]
        result = H['saturation_metrics'](commands, [3.])['0']
        self.assertAlmostEqual(result['saturated_s'], .02)
        self.assertAlmostEqual(result['longest_s'], .02)
        self.assertAlmostEqual(result['fraction'], .1)
        self.assertAlmostEqual(result['phases']['Flight']['saturated_s'], .01)


class WorldCOMTest(unittest.TestCase):
    def geometry(self):
        return SimpleNamespace(mass=2., links={'cad_body': (1., np.array([0., 0., 0.])),
                                              'wheel': (1., np.array([1., 0., 0.]))},
                               joints=[('wheel_joint', 'continuous', 'cad_body', 'wheel',
                                        np.eye(4), np.array([0., 1., 0.]))])

    def test_wheel_angle_and_world_rotation_not_zeroed(self):
        fk = H['WorldCOM'](self.geometry())
        np.testing.assert_allclose(fk.evaluate([0.,0.,1.], [0.,0.,0.,1.],
                                               {'wheel_joint': math.pi/2}), [0.,0.,.5], atol=1e-12)
        np.testing.assert_allclose(fk.evaluate([0.,0.,1.], [0.,math.sin(math.pi/4),0.,math.cos(math.pi/4)],
                                               {'wheel_joint': 0.}), [0.,0.,.5], atol=1e-12)

    def test_missing_joint_and_invalid_quaternion_rejected(self):
        fk = H['WorldCOM'](self.geometry())
        with self.assertRaises(ValueError):
            fk.evaluate([0.,0.,1.], [0.,0.,0.,1.], {})
        with self.assertRaises(ValueError):
            fk.evaluate([0.,0.,1.], [0.,0.,0.,2.], {'wheel_joint': 0.})


if __name__ == '__main__':
    unittest.main()
