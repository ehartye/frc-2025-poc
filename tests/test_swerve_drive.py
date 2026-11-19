import importlib.util
import math
from pathlib import Path
import sys
import unittest
import numpy as np

root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "selected_swerve_kinematics", root / "swerve-drive/kinematics.py"
)
drive = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = drive
spec.loader.exec_module(drive)


class SwerveDrive(unittest.TestCase):
    def setUp(self):
        # An intentionally asymmetric unit fixture exposes swapped module IDs.
        self.centers = np.array(
            [[0.24, 0.27], [0.26, -0.28], [-0.25, 0.26], [-0.26, -0.25]]
        )

    def test_pivot_and_desaturation(self):
        v, a = drive.module_targets(
            [0, 0, 2.0], self.centers, np.zeros(4), 0.55, pivot=self.centers[0]
        )
        vectors = np.c_[v * np.cos(a), v * np.sin(a)]
        self.assertAlmostEqual(v[0], 0)
        self.assertAlmostEqual(max(abs(v)), 0.55)
        p = self.centers - self.centers[0]
        np.testing.assert_allclose(np.sum(p * vectors, axis=1), 0, atol=1e-12)

    def test_drive_reversal_and_steer_wrap(self):
        speeds, angles = drive.module_targets(
            [-1, 0.01, 0], self.centers, np.zeros(4), 2.0
        )
        self.assertTrue((speeds < 0).all())
        self.assertTrue((abs(angles) < 0.02).all())
        self.assertAlmostEqual(
            math.degrees(drive.wrapped(math.radians(-179 - 179))), 2.0
        )

    def test_signed_absolute_and_coupled_drive(self):
        offsets = np.array([0, math.pi / 2, math.pi, -math.pi / 2])
        m = drive.EncoderMap(
            0.0762,
            5.5,
            offsets,
            np.array([1, -1, -1, 1]),
            np.array([-1, -1, 1, 1]),
            np.array([1.0, -0.5, 2.0, -0.75]),
        )
        a = np.deg2rad([179, 45, -50, 0])
        delta = np.deg2rad([6, 15, -25, 90])
        s = np.array([0.013, -0.071, 0.002, 0.110])
        r0, q0 = m.raw(np.zeros(4), a, 1024)
        r1, q1 = m.raw(s, a + delta, 1024)
        da = drive.wrapped(m.angle(q1, 1024) - m.angle(q0, 1024))
        np.testing.assert_allclose(m.delta_distance(r1 - r0, da), s, atol=0.0001)
        self.assertGreater(np.max(abs((r1 - r0) * math.pi * 0.0762 / 5.5 - s)), 0.05)
