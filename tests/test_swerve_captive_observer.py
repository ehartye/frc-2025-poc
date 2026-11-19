"""Bind the scored-film chapters and negative gates to the issued receiver sources."""

import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "experiments/swerve-drive/retention"


def read(path):
    return json.loads(path.read_bytes())


class CaptiveObserverTest(unittest.TestCase):
    def test_loaded_stops_qualify_but_reef_poses_need_opposite_corrections(self):
        held = read(STUDY / "held.json")["cases"]
        reef = read(STUDY / "reef-fix.json")["cases"]
        for case in ("minus", "plus"):
            row = held[case]
            self.assertTrue(row["allContinuationServicesGranted"])
            self.assertGreater(row["postInterlockServices"], 2900)
            self.assertTrue(all(row["source"]["qualifiedAtWheelEndpoint"].values()))
            self.assertGreater(row["source"]["minimumBoreEngagementMm"], 21)
            self.assertFalse(any(row["fusedFaultPermissions"].values()))
            self.assertTrue(
                all(row["firstTwoIndependentGatesUnderFiveMmOffset"].values())
            )
            optical = reef[case]["observation"]
            self.assertTrue(optical["nominal"]["accepted"])
            self.assertFalse(
                any(
                    optical[k]["accepted"]
                    for k in ("missing-eye", "wrong-tag", "ambiguous")
                )
            )
            l, h = optical["nominal"]["positionMmHeadingRad"]
            self.assertTrue(
                all(a <= v <= b for a, v, b in zip(l, reef[case]["finalMmMmRad"], h))
            )
            self.assertTrue(l[0] > -54 if case == "minus" else h[0] < -54)
            self.assertEqual(row["source"]["sourceRoute"], reef[case]["wheel"])
            self.assertEqual(row["source"]["matchDeliveries"], 0)

    def test_native_occluders_and_finish_are_source_bound(self):
        native = read(STUDY / "native-view.json")
        digest = hashlib.sha256((STUDY / "open-yokes.step").read_bytes()).hexdigest()
        self.assertEqual(native["retainerSha256"], digest)
        self.assertEqual(len(read(STUDY / "open-yokes.json")["parts"]), 26)
        for case in native["cases"]:
            v = case["comparisons"]
            self.assertTrue(all(f["accepted"] for f in v["envelope65"]["frames"]))
            for bad in ("envelope215", "excluded"):
                self.assertFalse(any(f["accepted"] for f in v[bad]["frames"]))
            self.assertFalse(any(r["emptyAccepted"] for r in v.values()))

    def test_force_and_length_are_independent(self):
        # The checked force is an upper enclosure; a coherently stuck occupancy
        # pair cannot make empty rim probes satisfy finite tube length.
        path = STUDY / "retained_report.py"
        spec = importlib.util.spec_from_file_location("retention_guard_test", path)
        module = importlib.util.module_from_spec(spec)
        previous = sys.modules.pop("screen", None)
        sys.path.insert(0, str(STUDY))
        try:
            spec.loader.exec_module(module)
        finally:
            sys.path.pop(0)
            sys.modules.pop("screen", None)
            if previous is not None:
                sys.modules["screen"] = previous
        self.assertTrue(module.permission([1.3, 4.6], [True, True], [True, True]))
        for mm in ([0, 0], [1.3, 9.6], [4, 3], [0.5, 5.4]):
            self.assertFalse(module.permission(mm, [True, True], [True, True]))
