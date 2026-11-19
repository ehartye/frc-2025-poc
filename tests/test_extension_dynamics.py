"""Regression for the extension fixture, the changing surface and its unsupported transfer."""

import hashlib
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "extension-support"


class TestExtensionDynamics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = json.loads((HERE / "dynamics.json").read_bytes())

    def test_source_and_motion(self):
        for name, digest in self.report["sources"].items():
            self.assertEqual(hashlib.sha256((ROOT / name).read_bytes()).hexdigest(), digest, name)
        motion = json.loads((HERE / "dynamics-motion.json").read_bytes())
        self.assertGreater(motion["nativeFrameCount"], 21000)
        self.assertLess(motion["maximumStrokeDifferenceM"], 1e-12)
        self.assertAlmostEqual(
            motion["previousReleaseFromDriveS"] - motion["previousExtensionStartFromDriveS"], 3.0
        )

    def test_native_edge_and_two_refined_arrivals(self):
        cases = self.report["cases"]
        self.assertEqual(len(cases), 20)
        native = json.loads((HERE / "dynamics-witness.json").read_bytes())
        for y in (-66.0, 42.0):
            contact = {
                c["strokeMm"]: c["commonWithFreeFlatButtonMm3"]
                for c in native["cases"]
                if c["localCenterMm"][1] == y
            }
            self.assertLess(contact[21.25], 1e-7)
            self.assertGreater(contact[21.375], 0.05)
            self.assertGreater(contact[24.0], 10.0)
        for side in ("leftSmall", "rightSmall"):
            times = []
            for step in (62.5, 31.25, 15.625):
                v = cases[f"{side}-nominal-{step}us"]
                self.assertLess(v["lowestMeanNormal"]["requiredVertForceN"], 0)
                self.assertLess(v["firstPossibleSideStrike"]["strokeMm"], 22)
                self.assertGreater(v["firstMeanTension"]["strokeMm"], 150)
                self.assertLess(v["finalChangeMm"][-1], -6)
                times.append(v["firstMeanTension"]["timeS"])
            self.assertLess(max(times) - min(times), 0.001)
            for variant in ("no-buttons", "retracted-slow", "retracted-slower"):
                r = cases[f"{side}-{variant}-62.5us"]
                self.assertIsNone(r["firstPossibleSideStrike"])
                self.assertIsNotNone(r["firstMeanTension"])
            self.assertIsNone(self.report["achievedPlacement"])
        bad = cases["rightSmall-no-keepers-62.5us"]
        self.assertLess(bad["lastStrokeMm"], 200)
        self.assertIsNotNone(bad["largeRotationStopS"])


if __name__ == "__main__":
    unittest.main()
