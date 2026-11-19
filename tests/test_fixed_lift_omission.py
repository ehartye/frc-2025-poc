import hashlib
import json
from pathlib import Path
import unittest
from xml.etree import ElementTree

ROOT=Path(__file__).resolve().parents[1]
HERE=ROOT/"fixed-lift-omission"


def read(name):
    return json.loads((HERE/name).read_bytes())


class FixedLiftOmission(unittest.TestCase):
    def test_supplier_seating_and_motion(self):
        r=read("fit.json")
        self.assertEqual(r["capSha256"],hashlib.sha256((HERE/"input-cap.step").read_bytes()).hexdigest())
        for name,digest in r["sources"].items():
            self.assertEqual(digest,hashlib.sha256((ROOT/name).read_bytes()).hexdigest())
        self.assertGreater(r["contactAt0_10mmMm3"],100.)
        self.assertEqual(r["capBlockCommonMm3"],0.)
        self.assertGreater(r["closestOtherSweepBoundsMm"][0][0],8.)
        self.assertGreater(r["nearestAddedSweepMm"][0],90.)
        for bolt in r["hardware"]:
            self.assertEqual(bolt["shaftBlockCommonMm3"]+bolt["headBlockCommonMm3"],0.)
            self.assertGreater(bolt["capEngagementMm"],7)
            self.assertGreater(bolt["longer31_75ProtrusionMm"],4)
        ElementTree.parse(HERE/"shop.svg")

    def test_accounting_does_not_transfer_unproven_credit(self):
        m=read("mass.json")
        self.assertIsNone(m["groupSpecificMinimumMotorKg"])
        self.assertIsNone(m["measuredRemovedKg"])
        self.assertAlmostEqual(m["restrictedNoMotorCreditKg"][1],.054)
        c=read("comparison.json")["jointHarnessAndLightSkin"]["cases"]
        no=c["restrictedNoMotorCreditKg"]
        yes=c["conditionalMatchedArticleDeltaKg"]
        self.assertLess(no["representedMarginKg"][0],-.21)
        self.assertAlmostEqual(yes["representedMarginKg"][0]-no["representedMarginKg"][0],.4)
        self.assertIsNone(yes["physicalMarginKg"])

    def test_locks_and_faults_remain(self):
        fit=read("fit.json")
        for side in ("L","R"):
            self.assertTrue(all(v>0 for v in fit["twoPermanentLockChainsAt1mmMm3"][side].values()))
        mechanics=read("mechanics.json")
        self.assertEqual(mechanics["releasedDeckRemovalCreditKg"],0)
        self.assertLess(mechanics["localRigidityRatio"],.18)
        self.assertGreater(mechanics["oneSideLock"]["10"]["pinBendingMPa"],500)
        self.assertGreater(mechanics["stripAt3g"]["servicePoint"]["sixPockets"]["stressMPa"],400)
        controller=read("controls.json")
        self.assertEqual(controller["recordedEvents"]["hold"],18)
        self.assertEqual(max(controller["fixedRolePeakLiftBranchA"].values()),0)
        self.assertFalse(controller["oldVariableHeightProfileCompatible"])
        self.assertTrue(all(a["visibleWithin0_1mm"]==0 for eye in read("images.json")["eyes"].values() for a in eye))
