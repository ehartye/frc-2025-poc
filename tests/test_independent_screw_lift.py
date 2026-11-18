"""Protect the screw replacement's material and loss-of-power counterexamples."""
import hashlib
import json
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
HERE=ROOT/"design"/"lift-screw"
sys.path.insert(0,str(HERE))
from screen import THREADS,shaft,coast

def read(name):
    return json.loads((HERE/name).read_bytes())


class IndependentScrewLift(unittest.TestCase):
    def test_registered_sources_and_failure_bounds(self):
        data=read("screen.json")
        for path,sha in data["sourceSha256"].items():
            self.assertEqual(hashlib.sha256((ROOT/path).read_bytes()).hexdigest(),sha)
        for key in ("half-8","five-eighths-8"):
            self.assertFalse(any(case["schedulingScreen"] for case in data["threads"][key]["drive"]))
        chosen=shaft(*THREADS["three-quarters-6"])
        output=coast(chosen,.10,7.951380767113901,.8,coupled=True)
        self.assertTrue(output["selfHolding"])
        self.assertLess(output["worstPhase"]["marginToBaselineArmMm"],-150)
        self.assertFalse(coast(chosen,.05,12.204,.8,coupled=False)["selfHolding"])
        self.assertFalse(data["qualifiedForInstallation"])

    def test_drag_and_mass_alternatives(self):
        data=read("screen.json")
        drag=data["passiveDragRange"]
        self.assertLess(drag["0.00..0.20"]["best"]["toleranceWindowNm"],0)
        discrete=drag["0.05..0.20"]["discreteRatioExamples"]
        self.assertTrue(0<discrete["18to21"]["toleranceWindowNm"]<.003)
        self.assertLess(discrete["20to23"]["toleranceWindowNm"],0)
        budget=data["mass"]
        self.assertGreater(budget["addOnShaftsKg"],budget["headroomWithLightCarrierKg"])
        self.assertAlmostEqual(budget["fullReplacementRemainderAtUpperKg"]-budget["sameRoomWithLowOriginalDriveCreditKg"],2.1)
        self.assertEqual(data["splitDrive"]["commandTimeMsFromStopped"],298)

    def test_native_pose_and_interference_controls(self):
        bare,fit=read("native-fit.json"),read("proposal-fit.json")
        source=json.loads((ROOT/"design/competition-package/shop-duplex/red/review.json").read_bytes())
        for pose,record in fit.items():
            self.assertEqual(record["base"],source[pose]["export_sha256"])
            self.assertEqual(record["base"],bare[pose]["source"])
            for side in bare[pose]["locationsMm"]["-142,255"].values():
                self.assertFalse(any(side.values()))
            for tag in ("L","R"):
                self.assertGreater(record["rejectedLowerSupportAt365Mm"][tag]["backup-"+tag+"-service-rail-arm-368"],73)
                self.assertNotIn(tag+"-lower-support-reserve",record["fixedAndMovingNativeOverlapsMm3"])
            for contacts in record["fixedAndMovingNativeOverlapsMm3"].values():
                self.assertTrue(all(k.startswith("running-line-") for k in contacts))
        self.assertFalse(any(fit["place"]["simultaneousDropMm"]["9"].values()))
        self.assertTrue(all(fit["place"]["simultaneousDropMm"]["10"].values()))
        bearing=fit["place"]["workingSleeveToLowerSupport"]
        self.assertEqual(max(bearing["5.95"].values()),0)
        self.assertGreater(min(bearing["6.0"].values()),25)
        self.assertEqual(fit["place"]["export"]["namedSolids"],16)
        self.assertEqual(hashlib.sha256((HERE/"placement-reserves.step").read_bytes()).hexdigest(),fit["place"]["export"]["sha"])
