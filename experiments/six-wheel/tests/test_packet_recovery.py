import csv
import hashlib
import json
from pathlib import Path
import unittest

PROJECT=Path(__file__).resolve().parents[1]
STUDY=PROJECT/"packet-recovery"
SOURCE=PROJECT/"relieved-match"


def load(path):
    return json.loads(path.read_bytes())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class PacketRecoveryTests(unittest.TestCase):
    def test_recovery_uses_registered_evidence_and_finite_timing(self):
        run=load(STUDY/"results.json")
        hal=load(STUDY/"hal.json")
        self.assertEqual(digest(STUDY/"GapWitness.java"),hal["programs"]["GapWitness.java"])
        self.assertEqual(digest(STUDY/"events.csv"),hal["events"])
        self.assertEqual(run["sourceEvents"],digest(SOURCE/"events.csv"))
        self.assertEqual(run["sourceHal"],digest(SOURCE/"hal.json"))
        self.assertEqual(run["sourceMatch"],digest(SOURCE/"match.json"))
        self.assertEqual(run["hal"],digest(STUDY/"hal.json"))
        cases=run["cases"]
        with (SOURCE/"events.csv").open() as source,(STUDY/"events.csv").open() as result:
            historical=list(csv.DictReader(source))
            new=list(csv.DictReader(result))
        for mode in ("nominal","lateB","single"):
            self.assertEqual([r for r in historical if r["scenario"]==mode],[r for r in new if r["scenario"]==mode])
        for delay in range(23,33):
            trial=cases["delay"+str(delay)]
            self.assertTrue(trial["matchesNominalRack"])
            self.assertEqual(trial["BEventsWallMs"]["stow"],100845)
        self.assertEqual(cases["bridge26"]["bridge"],["roof-hold@26255/26231","fixed-reacquired@26260/26231","front-clear@26420/26401"])
        for fault in ("delay33","delay34","bridgeLateSector","wrongEpoch","wrongPiece","roofMismatch","bothLate",
                      "secondGap","roofSilent","fixedSilent","fixedDuplicate","expiredPrior","rackJam"):
            self.assertNotIn("stow",cases[fault]["BEventsWallMs"],fault)
            self.assertIn("hold",cases[fault]["BEventsWallMs"],fault)

    def test_achieved_hull_and_two_unique_supported_pipes(self):
        result=load(STUDY/"geometry.json")
        for name,hash_value in result["sources"].items():
            self.assertEqual(hash_value,digest((STUDY if name in ("events.csv","hal.json") else SOURCE)/name))
        for case in result["cases"].values():
            self.assertEqual(case["contactIntegrations"],4)
            self.assertEqual(case["points"],9)
            self.assertEqual(case["scoredPipes"],["coral-A","coral-B"])
            self.assertEqual(case["landingSHA256"],digest(SOURCE/"landing-observed.csv"))
            self.assertGreater(case["minimumPostContactFrontMm"],32.6)
            self.assertGreater(case["sixRobotMarginMm"],104)
            self.assertGreater(case["stockMarginMm"],1)
            self.assertGreater(case["pipeToPipeMm"],93)
            self.assertLess(case["parkWallS"],111)
