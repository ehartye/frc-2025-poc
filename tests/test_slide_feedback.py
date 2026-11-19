import csv
import hashlib
import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "slide-feedback"


class SlideFeedbackReceipts(unittest.TestCase):
    def setUp(self):
        self.result=json.loads((STUDY/"results.json").read_bytes())

    def test_frozen_inputs(self):
        for name,digest in self.result["sourceSha256"].items():
            self.assertEqual(hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),digest,name)
        self.assertEqual(self.result["poseBodies"],598)
        self.assertEqual(self.result["cases"],6912)

    def test_every_phase_has_motion_and_time_to_stow(self):
        with (STUDY/"phases.csv").open() as fp:
            phases=list(csv.DictReader(fp))
        self.assertEqual(len(phases),34)
        for name,v in self.result["pieces"].items():
            affected=[r for r in phases if r["piece"]==name]
            self.assertEqual(set(int(r["exposure_phase_ms"]) for r in affected),set(range(17)))
            self.assertEqual(set(int(r["accepted_cases"]) for r in affected),{165})
            self.assertTrue(v["allRetainTargetSupport"])
            self.assertGreater(v["minimumWholeFrontMm"],31)
            self.assertGreater(v["minimumShelfGapMm"],37)
            self.assertGreater(v["minimumTimeBeforeIssuedStowMs"],4)
            self.assertGreater(v["stationaryPairedSupportS"],2.2)
            self.assertLessEqual(v["latestAckAfterPriorMs"],180)
            self.assertLess(v["peakStopSpeedMps"],.021)
            self.assertEqual(v["stowAfterCommandMs"],[2470])

    def test_adverse_holds(self):
        for name,v in self.result["pieces"].items():
            for fault,cases in v["faults"].items():
                self.assertEqual(cases,{"FAULT":5},fault)
            for failure in v["faultContact"].values():
                self.assertEqual(failure["physicalShelfRecontacts"],failure["attempts"])
            self.assertEqual(v["dragControl"]["140"],{"FAULT":5})
            self.assertEqual(v["dragControl"]["35"],{"STOWED":5})
            self.assertEqual(self.result["policies"]["short5"][name]["verifiedStow"],0)
            self.assertEqual(self.result["policies"]["extended5"][name]["verifiedStow"],2805)

    def test_no_air_transfer_stays_within_original_front(self):
        receipt=json.loads((STUDY/"no-air.json").read_bytes())
        self.assertFalse(receipt["opticalGateTransferred"])
        self.assertEqual(receipt["newOrChangedBaseFrontMaxMm"],337)
        for pose in receipt["cases"].values():
            self.assertEqual((pose["sourceBodies"],pose["successorBodies"],pose["unchangedNativeBodies"]),(598,593,590))
            self.assertIn(pose["frontMm"],(425,535))
