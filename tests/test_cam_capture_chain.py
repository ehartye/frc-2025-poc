"""Check the published causal handoff and guard the held scoring outcome."""

import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "experiments/swerve-drive/cam-scoring"


class MatchedCaptureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = json.loads((STUDY / "results.json").read_bytes())

    def test_each_same_pipe_uses_the_correct_machined_station(self):
        cases = self.data["developmentCases"]
        self.assertNotEqual(
            cases["plus"]["firstWheelServiceState"],
            cases["minus"]["firstWheelServiceState"],
        )
        for d in cases.values():
            station, travel = d["stages"]["station"], d["stages"]["route"]
            self.assertEqual(station["piece"], travel["piece"])
            self.assertEqual(station["before"], d["firstWheelServiceState"])
            self.assertEqual(station["state"], travel["before"])
            self.assertEqual(station["cam"], travel["cam"])
            self.assertAlmostEqual(station["firstFiveConfirmedS"], 0.255)
            self.assertAlmostEqual(station["endWallS"], travel["startWallS"])
            self.assertEqual(travel["wheel"], d["route"]["trajectory"])
            self.assertTrue(d["carriedToMeasuredRoughStop"])
            self.assertEqual(travel["grantedServices"], travel["services"])
            self.assertGreater(travel["minimumBoreMm"], 21.69)
            self.assertEqual(
                d["carriedToMeasuredRoughStop"],
                travel["allGranted"]
                and d["route"]["stopFeedback"]["confirmed"]
                and abs(travel["endWallS"] - d["route"]["endWallS"]) < 1e-7,
            )

    def test_measured_rough_stop_does_not_authorize_either_target(self):
        for side, d in self.data["developmentCases"].items():
            self.assertEqual(d["stationOptics"], {"65": 6, "215": 0})
            self.assertFalse(any(d["emptyAcceptedByFinish"].values()))
            low, high = d["dock"]["observed"]["positionMmHeadingRad"]
            truth = d["dock"]["trueMmMmRad"]
            self.assertTrue(all(lo <= v <= hi for lo, v, hi in zip(low, truth, high)))
            self.assertFalse(low[0] <= d["positionTargetMmMmRad"][0] <= high[0])
            self.assertFalse(d["releaseAuthorized"])
            self.assertEqual(d["matchDeliveryPoints"], 0)
        self.assertEqual(self.data["falseGrants"], 0)
        self.assertEqual(self.data["holdoutByTreatment"]["wrong-tag"]["accepted"], 0)
        self.assertLessEqual(
            self.data["heldOut"]["maximumAcceptedErrorPx"],
            self.data["heldOut"]["tagErrorPx"],
        )
