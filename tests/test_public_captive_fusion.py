import hashlib
import json
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "cad/swerve-captive"


class PublicCaptureFusionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.receipt = json.loads((STUDY / "student-reopen.json").read_bytes())

    def test_actual_four_wheel_axes_hold_their_physical_centers(self):
        for corner, x, y in (
            ("FL", 282, 277),
            ("FR", 282, -277),
            ("RL", -282, 277),
            ("RR", -282, -277),
        ):
            for joint in ("steer", "roll"):
                sample = self.receipt["joints"][corner + " " + joint]["samples"][1]
                self.assertAlmostEqual(sample["position"], 32.0)
                # The transformations are in cm and express the rotating part in its parent.
                pivot = [x / 10, y / 10, 3.81, 1]
                m = sample["relativeTransform"]
                for i in range(3):
                    self.assertAlmostEqual(
                        sum(m[4 * i + j] * pivot[j] for j in range(4)),
                        pivot[i],
                        places=6,
                    )
                if joint == "steer":
                    self.assertAlmostEqual(m[4], math.sin(math.radians(32)))
                else:
                    self.assertAlmostEqual(m[2], math.sin(math.radians(32)))

    def test_both_yokes_follow_the_outer_rack(self):
        for phase in self.receipt["linkedSweep"]:
            mm = phase["strokeMm"]
            left = phase["leftYokeMm"]
            observed = phase["endpoints"]
            self.assertAlmostEqual(10 * observed["Receiver middle"][3], mm / 2)
            self.assertAlmostEqual(10 * observed["Receiver outer"][3], mm)
            self.assertAlmostEqual(10 * observed["Left yoke close"][7], left)
            self.assertAlmostEqual(10 * observed["Right yoke close"][7], -left)
            self.assertAlmostEqual(10 * observed["Left yoke close"][3], 0)
            self.assertAlmostEqual(10 * observed["Right yoke close"][3], 0)

    def test_supplied_download_and_source_roster(self):
        archive = ROOT / self.receipt["source"]
        self.assertEqual(
            hashlib.sha256(archive.read_bytes()).hexdigest(), self.receipt["sha256"]
        )
        native = self.receipt["components"]
        self.assertEqual(sum(v["bodies"] for v in native.values()), 1002)
        self.assertEqual(sum(v["bodies"] == 0 for v in native.values()), 18)
        self.assertEqual(len(self.receipt["joints"]), 17)
        self.assertTrue(
            all(v["parent"] in native for v in native.values() if v["bodies"])
        )
