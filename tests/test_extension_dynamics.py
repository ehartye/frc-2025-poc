"""Regression for the extension fixture, the changing surface and its unsupported transfer."""

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "extension-support"


class TestExtensionDynamics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = json.loads((HERE / "dynamics.json").read_bytes())

    def test_supplied_motion_duration_and_conflicting_derivatives(self):
        spec = importlib.util.spec_from_file_location(
            "extension_motion_input", HERE / "dynamics_input.py"
        )
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        header = "time_s,stroke_m,velocity_mps,acceleration_mps2\n"
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "outbound.csv"

            def write(rows):
                path.write_bytes(
                    (header + "".join(",".join(str(x) for x in r) + "\n" for r in rows)).encode()
                )
                return hashlib.sha256(path.read_bytes()).hexdigest()

            rows = [[t, 0.01 * t * t, 0.02 * t, 0.02] for t in (0, 0.01, 0.02, 0.03, 0.04)]
            pin = write(rows)
            motion = mod.read_motion(path, pin)
            duration, grid, x, v, a = mod.sample_motion(motion, 0.001)
            self.assertAlmostEqual(duration, 0.04)
            self.assertAlmostEqual(grid[-1], duration)
            self.assertAlmostEqual(x[15], 0.01 * 0.015**2)
            self.assertAlmostEqual(v[15], 0.02 * 0.015)
            self.assertAlmostEqual(a[15], 0.02)
            # Exercise the production receipt on a deliberately short signed
            # trajectory. The original default-duration bug lived at this seam.
            subprocess.run(
                [
                    sys.executable,
                    str(HERE / "dynamics_run.py"),
                    folder,
                    "--motion",
                    str(path),
                    "--motion-sha256",
                    pin,
                ],
                check=True,
                env={**os.environ, "OPENBLAS_NUM_THREADS": "1"},
            )
            result = json.loads((Path(folder) / "leftSmall-nominal-62.5us.json").read_bytes())
            self.assertAlmostEqual(result["durationS"], 0.04)
            self.assertAlmostEqual(result["lastTimeS"], 0.04)
            self.assertEqual(result["motionSha256"], pin)
            # Alternating accelerations integrate to zero under endpoint
            # averaging, despite contradicting the recorded constant speed.
            for bad in (
                [[0, 0, 0.001, 10], [0.001, 0.000001, 0.001, -10], [0.002, 0.000002, 0.001, 10]],
                [[0, 0, 1, 0], [0.00001, 0, 1, 0], [0.00002, 0, 1, 0]],
            ):
                with self.assertRaisesRegex(ValueError, "derivatives"):
                    mod.read_motion(path, write(bad))

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
