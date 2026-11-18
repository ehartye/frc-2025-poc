"""Check the issued station-relative camera design and its physical observability boundary."""
import hashlib
import json
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "station-localization"
sys.path.insert(0, str(HERE))
from attitude import outer, projected
from pose import EYE, SECOND_EYE
from study import edges, gate


class StationLocalizationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.study = json.loads((HERE / "pose-study.json").read_bytes())
        cls.native = json.loads((HERE / "sightlines.json").read_bytes())

    def test_live_sources_and_visibility(self):
        for path, digest in self.study["sourceSha256"].items():
            self.assertEqual(hashlib.sha256((ROOT / path).read_bytes()).hexdigest(), digest)
        saved = self.native["originalReserveStep"]
        self.assertEqual(hashlib.sha256((HERE / "camera-reserve.step").read_bytes()).hexdigest(), saved["sha256"])
        self.assertEqual(saved["solids"], 4)
        self.assertEqual(self.native["robotBodyCount"], 545)
        self.assertEqual(self.native["stationOccurrences"], 47)
        self.assertTrue(all(v < 0 for eye in self.native["legacyPixels"] for u, v in eye))
        rays = [r for view in self.native["candidateRays"].values() for r in view]
        self.assertEqual(len(rays), 78)
        self.assertTrue(all(r["inFrame"] and not r["blockers"] for r in rays))
        self.assertTrue(all(not overlaps for overlaps in self.native["mounts"].values()))
        rejected = json.loads((HERE / "rejected-mount.json").read_bytes())
        self.assertEqual(rejected["occludedBoltRays"], ["rsl-reserve"] * 4)

    def test_six_dimensional_error_and_independent_station_shift(self):
        for key, tight in (("tiltedEligible", True), ("tiltedUnresolved", False), ("adverse", False)):
            case = self.study["cases"][key]
            truth = case["truth"]
            state = truth[:3] + list(map(math.radians, truth[3:]))
            marks = [projected(state, e) for e in (EYE, SECOND_EYE)]
            result = outer(marks, (EYE, SECOND_EYE), case["errorPx"])
            self.assertTrue(all(a <= v <= b for (a, b), v in zip(result, truth)))
            self.assertEqual(edges(result)["tight"], tight)
            for x, (lo, hi) in zip((-100, 100), edges(result)["sideIntervalsAtTwoPlanesMm"]):
                actual = truth[1] + (x - truth[0]) * math.tan(math.radians(truth[3]))
                self.assertLessEqual(lo, actual)
                self.assertLessEqual(actual, hi)
        zero = self.study["controls"]["unsurveyedBoardShiftMm"]
        self.assertEqual(zero["maximumPixelDifference"], 0)
        self.assertGreater(zero["side"], 0.50)
        marks[1][0][1] += 20
        self.assertIsNone(outer(marks, (EYE, SECOND_EYE), .1))

    def test_optical_budget_and_physical_hold(self):
        for name in ("right", "pair"):
            cases = self.study["cases"][name]
            self.assertTrue(cases["0.05"]["alignedExample"]["tight"])
            self.assertFalse(cases["0.05"]["alignedEveryInBudgetReading"]["tight"])
            self.assertTrue(cases["0.1"]["alignedExample"]["wide"])
            self.assertFalse(cases["0.2"]["alignedExample"]["wide"])
        self.assertGreater(self.study["syntheticPatches"]["right"]["corrected15PercentGradientMaxPx"], .50)
        frame = {"camera":"right", "tag":2, "count":4, "clear":True, "epoch":1, "captureMs":100}
        self.assertEqual(gate([frame], 110), "hold-unverified-survey-or-calibration")
        self.assertEqual(gate([frame], 110, validated=True), "eligible-for-pose-estimation")
        for frames, now in (([frame], 122), ([frame], 99), ([frame, frame], 100),
                            ([frame, frame | {"camera":"left", "captureMs":99.5}], 100),
                            ([frame | {"tag":1}], 100), ([frame | {"epoch":0}], 100)):
            self.assertTrue(gate(frames, now, validated=True).startswith("hold-"))
        self.assertFalse(self.study["physicalMeasurementVerified"])
