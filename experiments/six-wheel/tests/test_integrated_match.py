import csv
import hashlib
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
CASE = ROOT / "integrated-match"


def read(name):
    return json.loads((CASE / name).read_bytes())


class IntegratedMatchTest(unittest.TestCase):
    def test_registered_inventory_and_inputs(self):
        optical = read("optics.json")
        hal = read("hal.json")
        for key, count in (("receive", 201), ("place", 550)):
            self.assertEqual(593, optical["poses"][key]["nativeBodies"])
            self.assertEqual(count, optical["poses"][key]["fixedFrames"])
        self.assertEqual(550, hal["fixedReleaseFramesUnchanged"])
        self.assertEqual(84.3, min(p["separationMm"] for p in optical["movingRayPrism"]["baseChanges"]))
        for path, digest in read("match.json")["sourceSha256"].items():
            self.assertEqual(digest, hashlib.sha256((ROOT / path).read_bytes()).hexdigest(), path)
        self.assertEqual(hal["emittedEventsSha256"], hashlib.sha256((CASE / "events.csv").read_bytes()).hexdigest())
        self.assertEqual(hal["sceneRegister"], hashlib.sha256((CASE / "optics.json").read_bytes()).hexdigest())
        self.assertEqual(hal["sourceSha256"]["IntegratedTest.java"], hashlib.sha256((CASE / "IntegratedTest.java").read_bytes()).hexdigest())

    def test_controller_and_contact_gate_both_trips(self):
        with (CASE / "events.csv").open() as stream:
            events = list(csv.DictReader(stream))
        contact = read("contact.json")
        for scenario, count in (("nominal", 2), ("lateB", 1), ("single", 1)):
            completed = [r for r in events if r["scenario"] == scenario and r["event"] == "stow"]
            self.assertEqual(count, len(completed))
            for r in completed:
                pipe = r["piece"]
                physical = contact[scenario][pipe]
                self.assertTrue(physical["admitted"])
                self.assertGreaterEqual(physical["lastObservedWallS"], 156)
                self.assertGreater(physical["wholeFrontLowerMm"], 32.9)
                self.assertGreater(physical["branchLowerMm"], 160)
                steps = [e["event"] for e in events if e["piece"] == pipe and e["scenario"] == scenario]
                self.assertEqual(["feed", "seat", "carry", "release", "withdraw", "motion", "stow"], steps)
        rejected = contact["lateB"]["coral-B"]
        self.assertEqual("receiver-recontact", rejected["stop"])
        self.assertLess(rejected["minShelfGapMm"], 0)
        self.assertFalse(rejected["admitted"])
        b = [e for e in events if e["piece"] == "coral-B" and e["scenario"] == "lateB"]
        self.assertEqual(("98210", "camera-silent"), (b[-1]["wall_ms"], b[-1]["witness"]))
        self.assertNotIn("withdraw", [e["event"] for e in b])

    def test_field_and_score_are_case_specific(self):
        data = read("match.json")
        self.assertEqual(6, len(data["sixStartingPositions"]))
        self.assertTrue(all(s["proxyTapeOverlapMm"] > 0 for s in data["sixStartingPositions"].values()))
        self.assertGreater(data["firstPipeToSecondExtendedRobot"]["combinedLowerMm"], 37)
        for mode, points, count, park in (("nominal", 9, 2, True), ("lateB", 5, 1, False), ("single", 7, 1, True)):
            case = data["cases"][mode]
            self.assertEqual((points, count, park), (case["conditionalPoints"], case["runtimeDeliveries"], case["park"]))
            self.assertIsNone(case["firstOverlap"])
            self.assertGreater(case["pairBound"][0], .105)
            self.assertEqual(24, case["stockPieces"])
            self.assertGreater(case["fieldStockMinimumMm"], 7.8)
            self.assertTrue(all(v > 0 for v in case["stowMarginMs"].values()))
            for edition in case["assessments"].values():
                self.assertEqual(count, sum(r["l1CriteriaAtInstant"] == "yes" for r in edition))
