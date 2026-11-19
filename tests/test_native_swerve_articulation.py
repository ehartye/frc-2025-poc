import hashlib
import json
import unittest
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
STUDY = PROJECT / "cad/swerve-articulation"


class NativeArticulation(unittest.TestCase):
    def test_source_registration_and_native_reopen(self):
        result = json.loads((STUDY / "native.json").read_bytes())
        baseline = json.loads((PROJECT / "cad/full-swerve-r4/native-reopen.json").read_bytes())
        self.assertEqual(baseline["archive"]["sha256"], result["baselineSha256"])
        raw = (STUDY / "motion-groups.json").read_bytes()
        self.assertEqual(hashlib.sha256(raw).hexdigest(), result["travelRegisterSha256"])
        groups = json.loads(raw)
        members = [name for stage in groups.values() for name in stage]
        self.assertEqual(len(members), len(set(members)))
        self.assertEqual(groups["stowedOnly"], ["sp1-live-cable"])
        self.assertIn("r6-polymer-facing", groups["slide"])
        self.assertIn("roof-head-0", groups["slide"])
        self.assertEqual(result["groups"], {k: len(v) for k, v in groups.items()})
        self.assertTrue(result["exportAndIndependentReopenMatched"])
        self.assertEqual([p["commandMm"] for p in result["poses"]], [0, 185, 370, 0])
        for pose in result["poses"]:
            self.assertEqual(pose["checked"], len(groups["slide"]) + len(groups["middle"]) + 5)
            self.assertAlmostEqual(pose["commandMm"], pose["slideMm"])
            self.assertAlmostEqual(pose["middleMm"], pose["slideMm"] / 2)
            self.assertLess(pose["maxBoundsErrorMm"], .000001)

    def test_machined_full_assembly_matches_standalone_stock(self):
        native = json.loads((STUDY / "native.json").read_bytes())["nativeFeatures"]
        cams = json.loads((PROJECT / "experiments/swerve-drive/score-stroke/fusion.json").read_bytes())
        self.assertEqual({k: v["features"] for k, v in native.items()},
                         {"r6-polymer-facing": 2, "r6-receiver-insert": 4})
        for name, geometry in native.items():
            self.assertAlmostEqual(geometry["volumeMm3"], cams["originalR4Appearance"][name]["volumeMm3"], places=4)
