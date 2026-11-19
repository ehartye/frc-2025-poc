import hashlib
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "sector-alignment"


def read(name):
    return json.loads((HERE / (name + ".json")).read_bytes())


class SectorAlignmentTest(unittest.TestCase):
    def test_native_and_match_inputs(self):
        scene = read("scene")
        self.assertEqual(scene["robotBodies"], 593)
        self.assertEqual(scene["fasciaWindow"]["intersectMm3"], 0)
        self.assertEqual(len(scene["boltFacesMm"]), 2)
        clock = read("clock")
        for path, sha in clock["sourceSHA256"].items():
            self.assertEqual(hashlib.sha256((ROOT / path).read_bytes()).hexdigest(), sha, path)
        self.assertEqual(scene["bridgeSHA256"], hashlib.sha256((ROOT / "bridge-tolerance/moving-bridge.step").read_bytes()).hexdigest())
        self.assertEqual(len(clock["cases"]), 20)
        for row in clock["cases"]:
            self.assertGreater(row["stowedVoxelStockLowerMm"], 0)
            self.assertGreater(row["allStockToOpticalCorridorMm"], 160)
            self.assertGreater(row["allTrafficToOpticalCorridorMm"], 1500)
            self.assertGreater(row["pairedTrafficLowerMm"], 1200)

    def test_final_pose_requires_real_association_and_fresh_bounds(self):
        healthy = {"nominal", "positive", "negative", "wheel-bias", "lateral-kick"}
        held = {"wrong-face", "ambiguous", "dropout", "stale"}
        for pipe in ("coral-A", "coral-B"):
            rows = {r["mode"]: r for r in read("results") if r["pipe"] == pipe}
            self.assertEqual(set(rows), healthy | held | {"shared-survey-shift"})
            for k in healthy:
                r = rows[k]
                self.assertTrue(r["accepted"])
                self.assertFalse(r["truthOutsideDeclaredCalibration"])
                self.assertGreater(r["decodedTagFrames"], 0)
                self.assertGreater(r["pairedHeadFrames"], 0)
                self.assertLess(r["decisionMs"], 3272)
                self.assertGreaterEqual(r["lastGateCaptureMs"], 3208)
                self.assertLessEqual(r["latestQualifiedInterval"][0], 10)
                self.assertLessEqual(r["latestQualifiedInterval"][1], 1)
                self.assertLessEqual(r["terminalCheckPixelError"], .18)
                self.assertLessEqual(r["peakWheelMmps2"], 1200.00001)
            for k in held:
                self.assertFalse(rows[k]["accepted"])
                self.assertIsNone(rows[k]["decisionMs"])
            false_permit = rows["shared-survey-shift"]
            self.assertTrue(false_permit["accepted"])
            self.assertTrue(false_permit["truthOutsideDeclaredCalibration"])
            self.assertGreater(abs(false_permit["finalBodyErrorMmDeg"][1]), 10)

    def test_measured_state_and_park_credit(self):
        result = read("clock")
        motion = {x["pipe"]: x["motionSHA256"] for x in read("results") if x["mode"] == "nominal"}
        for r in read("pipe"):
            self.assertEqual(motion[r["pipe"]], r["poseSha256"])
            self.assertEqual(r["outcome"], "receiver-recontact")
            flight = r["registeredFreeFlight"]
            self.assertEqual(flight["outcome"], "paired-trough-first-contact")
            self.assertGreater(flight["oneMmShelfClearS"], 0)
            self.assertFalse(r["outOfPlaneVelocityWithinLimit"])
        park = read("park-custody")["stowedPark"]
        self.assertTrue(park["stayedOnShelf"])
        self.assertGreaterEqual(park["assessmentWallS"], 156)
        self.assertEqual(park["finalMaximumDisplacementLastSecondMm"], 0)
        self.assertTrue(result["oneClockPolicy"]["parkOptionClear"])
        self.assertEqual(result["oneClockPolicy"]["conditionalParkRolePoints"], 5)
        self.assertEqual(result["oneClockPolicy"]["blindAbortPoints"], 3)
        self.assertEqual(result["oneClockPolicy"]["verifiedPipeScores"], 0)
        self.assertFalse(result["oneClockPolicy"]["secondPipeDispatched"])
        for c in result["cases"]:
            if not c["controllerAccepted"]:
                self.assertGreater(c["stationaryAbortToBuzzerPairLowerMm"], 1000)
