#!/usr/bin/env python3
"""Independent homogeneous-transform URDF oracle for the compact C++ model."""
import math
from pathlib import Path
import random
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]


def matmul(a, b):
    return [[sum(x*y for x, y in zip(row, col)) for col in zip(*b)] for row in a]


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, xyz[0]],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, xyz[1]],
            [-sp, cp*sr, cp*cr, xyz[2]], [0, 0, 0, 1]]


def origin(element):
    if element is None:
        return transform()
    return transform(tuple(map(float, element.get("xyz", "0 0 0").split())),
                     tuple(map(float, element.get("rpy", "0 0 0").split())))


def oracle(root, angles):
    joints = {j.find("child").get("link"): j for j in root.findall("joint")}
    frames = {"cad_body": transform()}

    def frame(link):
        if link not in frames:
            joint = joints[link]
            q = angles.get(joint.get("name"), 0.)
            fixed = matmul(frame(joint.find("parent").get("link")), origin(joint.find("origin")))
            frames[link] = matmul(fixed, transform(rpy=(0, q, 0)))
        return frames[link]

    axles = [frame(f"cad_{s}_wheel_link") for s in ("left", "right")]
    axle = [(axles[0][i][3]+axles[1][i][3])/2 for i in range(3)]
    mass = mx = mz = inertia = 0.
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        t = matmul(frame(link.get("name")), origin(inertial.find("origin")))
        m = float(inertial.find("mass").get("value"))
        dx, dz = t[0][3]-axle[0], t[2][3]-axle[2]
        vals = inertial.find("inertia").attrib
        tensor = [[float(vals["i"+"".join(sorted((a,b)))]) for b in "xyz"] for a in "xyz"]
        rotated_yy = sum(t[1][a]*tensor[a][b]*t[1][b] for a in range(3) for b in range(3))
        mass += m
        mx += m*dx
        mz += m*dz
        inertia += rotated_yy + m*(dx*dx+dz*dz)
    return mx/mass, mz/mass+.03, inertia, axles[0][2][3]-axles[1][2][3]


class UrdfModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not shutil.which("xacro") or not shutil.which("g++"):
            raise RuntimeError("URDF/core validation requires xacro and g++")
        xml = subprocess.check_output(["xacro", str(ROOT/"src/tap_robot_description/description/cad_robot/robot.urdf.xacro")], text=True)
        cls.root = ET.fromstring(xml)
        cls.tmp = tempfile.TemporaryDirectory(prefix="tap-model-test-")
        cls.addClassCleanup(cls.tmp.cleanup)
        cls.probe = Path(cls.tmp.name)/"model_probe"
        subprocess.run(["g++", "-std=c++17", "-O2", "-I"+str(ROOT/"src/tap_control/core/include"),
                        str(ROOT/"src/tap_control/core/src/WholeBodyModel.cpp"),
                        str(ROOT/"src/tap_control/apps/model_probe.cpp"), "-o", str(cls.probe)], check=True)

    def check_pose(self, pose):
        names = [f"cad_{side}_{joint}" for side in ("left", "right") for joint in ("hip", "knee")]
        expected = oracle(self.root, dict(zip(names, pose)))
        actual = list(map(float, subprocess.check_output([str(self.probe), *map(str, pose)], text=True).split()))
        for a, e in zip(actual, expected):
            self.assertAlmostEqual(a, e, delta=2e-7, msg=f"pose={pose}: {actual} != {expected}")

    def test_asymmetric_fk_against_independent_urdf(self):
        rng = random.Random(713)
        for _ in range(80):
            self.check_pose([rng.uniform(-1.3, 1.3), rng.uniform(-1.5, .6),
                             rng.uniform(-1.3, 1.3), rng.uniform(-1.5, .6)])

    def test_reference_poses(self):
        for pose in ([0, 0, 0, 0], [.115483096, 0, .115483096, 0],
                     [.75838357, -1.2, .75838357, -1.2]):
            self.check_pose(pose)

    def test_generated_constants_current(self):
        subprocess.run(["python3", str(ROOT/"tools/generate_control_model.py"), "--check"], check=True)

    def test_candidate_deep_symmetric_squat(self):
        hip, height = self.check_deep_profile(1.2, .8, .15)
        self.assertAlmostEqual(hip, .75838357, delta=1e-7)
        self.assertAlmostEqual(height, .157862115, delta=1e-7)

    def test_candidate_1_6_symmetric_squat(self):
        hip, height = self.check_deep_profile(1.6, 1.05, .125)
        self.assertAlmostEqual(hip, .97921, delta=1e-5)
        self.assertAlmostEqual(height, .12761, delta=1e-5)
        self.assertLess(1.077*height, .15)

    def check_deep_profile(self, depth, hip_max, pendulum_min):
        names = [f"cad_{side}_{joint}" for side in ("left", "right") for joint in ("hip", "knee")]

        def evaluate(hip, knee):
            return oracle(self.root, dict(zip(names, [hip, knee, hip, knee])))

        previous_height = float("inf")
        for step in range(33):
            knee = -depth*step/32
            lo, hi = 0., hip_max
            self.assertLess(evaluate(lo, knee)[0]*evaluate(hi, knee)[0], 0)
            for _ in range(50):
                mid = (lo+hi)/2
                if evaluate(lo, knee)[0]*evaluate(mid, knee)[0] <= 0:
                    hi = mid
                else:
                    lo = mid
            hip = (lo+hi)/2
            result = evaluate(hip, knee)
            self.check_pose([hip, knee, hip, knee])
            self.assertLess(abs(result[0]), 1e-10)
            self.assertLess(abs(result[3]), 1e-10)
            self.assertLess(result[1], previous_height)
            self.assertGreater(result[2], 0)
            self.assertGreater(1.077*result[1], pendulum_min)
            previous_height = result[1]
        return hip, previous_height


if __name__ == "__main__":
    unittest.main()
