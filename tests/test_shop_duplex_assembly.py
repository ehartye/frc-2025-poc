"""Qualification boundaries of the 545-body installed successor."""
import hashlib
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "design/competition-package"
CURRENT = BASE / "shop-duplex"


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class ShopDuplexAssembly(unittest.TestCase):
    def test_sources_and_all_eight_exported_files(self):
        equivalence = read(CURRENT / "alliance-comparison.json")
        self.assertEqual(set(equivalence), {"stowed", "receive", "transport", "place"})
        historical = read(BASE / "part-frames.json")
        self.assertEqual(len(historical), 401)
        for color in ("red", "blue"):
            directory = CURRENT / color
            for source, digest in read(directory / "source-register.json").items():
                self.assertEqual(sha(ROOT / source), digest, source)
            frames = read(directory / "part-frames.json")
            self.assertEqual(len(frames), 545)
            self.assertEqual(frames["tapped-anchor-block-R-140"], "slide")
            self.assertEqual(frames["backup-L-carriage-plate"], "lift")
            self.assertEqual(frames["backup-R-return-line"], "variable-rope")
            for pose, row in read(directory / "review.json").items():
                digest = sha(directory / (pose + ".step"))
                self.assertEqual(digest, row["export_sha256"])
                self.assertEqual(digest, equivalence[pose][color + "_step_sha256"])
                self.assertEqual(equivalence[pose]["identical_native_serializations"], 545)
                self.assertEqual(row["cross_layer_pairs"], 124374)
                self.assertEqual(len(row["service_interferences_mm3"]), 5)
                self.assertFalse(any(row["service_interferences_mm3"].values()))
                self.assertFalse(row["interferences_mm3"])

    def test_normal_travel_and_mass_boundary(self):
        travel = read(CURRENT / "travel.json")
        self.assertEqual(len(travel), 3)
        for row in travel.values():
            self.assertEqual(row["disjoint_swept_boxes"] + row["constant_relative_pose"]
                             + len(row["exact_swept_common_mm3"]), 124374)
            self.assertLessEqual(max(row["exact_swept_common_mm3"].values()), .001)
            self.assertFalse(row["unresolved"])
            self.assertFalse(row["reference_conflicts_mm3"])
        mass = read(CURRENT / "mass.json")
        for path, digest in mass["sourceSha256"].items():
            self.assertEqual(sha(ROOT / path), digest, path)
        self.assertIsNone(mass["weighedKg"])
        carry = mass["poses"]["transport"]
        self.assertAlmostEqual(mass["liftLoadedUpperKg"], 12.204, places=3)
        self.assertAlmostEqual(carry["seated"]["mass_kg"][1], 65.7574, places=4)
        self.assertGreater(carry["R103_margin_at_high_kg"], 0)
        self.assertGreater(carry["R408_margin_at_high_kg"], 0)
        turn = carry["seated"]["drive_demand_at_profile_extrema"]["turn"]
        self.assertGreater(turn["frictionRequired"], .9)

    def test_place_overtravel_reaches_both_service_arms(self):
        # Isolate the package imports from other CAD generators named build.py.
        import subprocess
        import sys
        program = """
from assembly import cad, pose_shapes
parts = pose_shapes(560, 370, 'red')
for side in 'LR':
    plate = parts[f'backup-{side}-carriage-plate']
    arm = parts[f'backup-{side}-service-rail-arm-368']
    for drop, expected in ((5, 0), (10, 234.16)):
        volume = cad.intersection(cad.translated(plate, (0, 0, -drop)), arm)
        assert abs(volume - expected) < .02, (side, drop, volume)
"""
        subprocess.run([sys.executable, "-c", program], cwd=CURRENT, check=True)
