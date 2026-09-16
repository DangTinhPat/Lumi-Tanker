"""leg_inertia() sanity checks (Track B item 3: critical leg damping derived
from real mass/geometry data instead of a guess)."""
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from generate_control_model import extract, leg_inertia, neutral  # noqa: E402


class LegInertiaTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        xacro = ROOT / 'src/tap_robot_description/description/cad_robot/robot.urdf.xacro'
        xml = subprocess.run(['xacro', str(xacro)], check=True, text=True,
                             capture_output=True).stdout
        cls.parts, cls.origins = extract(xml)

    def test_hip_swings_more_mass_so_has_larger_inertia_than_knee(self):
        hip = neutral(self.parts, self.origins, 0., .03)
        hip_i, knee_i = leg_inertia(self.parts, self.origins, hip, 0.)
        self.assertGreater(hip_i, knee_i)
        self.assertGreater(knee_i, 0.)

    def test_matches_direct_parallel_axis_sum_at_cad_zero(self):
        # Independent computation at hip=knee=0 (no rotation), so each part's
        # COM position is just its own offsets summed along the chain -
        # cross-checks leg_inertia()'s FK against a from-scratch parallel-axis
        # calculation rather than re-testing the same code path.
        hip_i, knee_i = leg_inertia(self.parts, self.origins, 0., 0.)
        side_offsets = self.origins[0]
        hip_x, _, hip_z = side_offsets[0]
        knee_x = hip_x + side_offsets[1][0]
        knee_z = hip_z + side_offsets[1][2]
        expected_hip = expected_knee = 0.
        positions = [(hip_x, hip_z)] * 1  # placeholder, filled below per part
        parts_xz = []
        cur_x, cur_z = hip_x, hip_z
        for j in range(3):
            if j:
                cur_x += side_offsets[j][0]
                cur_z += side_offsets[j][2]
            m, cx, cz, iyy = self.parts[1 + j]
            com_x, com_z = cur_x + cx, cur_z + cz
            parts_xz.append((m, com_x, com_z, iyy))
        for m, com_x, com_z, iyy in parts_xz:
            expected_hip += iyy + m * ((com_x - hip_x) ** 2 + (com_z - hip_z) ** 2)
        for m, com_x, com_z, iyy in parts_xz[1:]:
            expected_knee += iyy + m * ((com_x - knee_x) ** 2 + (com_z - knee_z) ** 2)
        # leg_inertia() averages both (CAD-symmetric) sides.
        self.assertAlmostEqual(hip_i, expected_hip, places=9)
        self.assertAlmostEqual(knee_i, expected_knee, places=9)

    def test_left_and_right_sides_agree_to_cad_symmetry(self):
        # leg_inertia() averages both sides; zeroing out one side's mass/iyy
        # isolates the other side's own contribution (still /2 internally),
        # so comparing the two isolated sides checks CAD left/right symmetry
        # without the average masking a real asymmetry.
        def side_only(side):
            other = 1 - side
            zeroed = list(self.parts)
            for j in range(3):
                _, cx, cz, _ = zeroed[1 + 3 * other + j]
                zeroed[1 + 3 * other + j] = (0., cx, cz, 0.)
            return leg_inertia(zeroed, self.origins, .1, -.2)

        left_hip, left_knee = side_only(0)
        right_hip, right_knee = side_only(1)
        self.assertAlmostEqual(left_hip, right_hip, places=6)
        self.assertAlmostEqual(left_knee, right_knee, places=6)
        self.assertGreater(left_hip, 0.)
        self.assertGreater(left_knee, 0.)


if __name__ == '__main__':
    unittest.main()
