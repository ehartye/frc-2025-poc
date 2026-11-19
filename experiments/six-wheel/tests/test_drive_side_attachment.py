"""Regression checks on the selected joint's source-bound native evidence."""
import hashlib
import json
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
HERE=ROOT/"drive-side-attachment"


def read(name):
    return json.loads((HERE/name).read_bytes())


class DriveSideAttachmentTest(unittest.TestCase):
    def test_reopened_stock_both_directions_and_tools(self):
        fit=read("fit.json")
        sha=hashlib.sha256((HERE/"inner-rail-joints.step").read_bytes()).hexdigest()
        self.assertEqual(fit["stepSha256"],sha)
        for path,digest in fit["sourceSha256"].items():
            if path!="603-body shop STEP":
                self.assertEqual(hashlib.sha256((ROOT/path).read_bytes()).hexdigest(),digest,path)
        self.assertEqual(fit["reopenedBodies"],50)
        self.assertEqual(set(fit["ends"]),{end+"-"+side for end in ("front","rear") for side in "LR"})
        for case in fit["ends"].values():
            self.assertEqual(case["originalAgainstNewInboardFilletMm3"],200)
            self.assertGreaterEqual(case["cutRailToInnerFilletMm"],1.49)
            self.assertGreater(sum(case["longSocketControl"]["overlapMm3"].values()),7000)
            for row in case["tools"].values():
                self.assertEqual(len(row),7)
                for witness in row.values():
                    self.assertFalse(witness["overlapMm3"])
        service=read("rail-service.json")
        self.assertEqual(service["candidateSha256"],sha)
        for side,case in service["ends"].items():
            self.assertEqual(len(case["jointHardwareRemoved"]),16)
            self.assertEqual(len(case["attachedTabsTraveling"]),2)
            self.assertEqual(len(case["removedJacks"]),2)
            self.assertEqual(len(case["loweredEnvelope"]),3)
            self.assertEqual(case["withdrawInboardMm"],8)
            self.assertEqual(case["downMm"],180)
            self.assertTrue(all(not shape["overlapMm3"] for shape in case["loweredEnvelope"]))
            self.assertEqual(len(case["dropBeforeUnpinningMm3"]),2)
            self.assertGreater(min(case["dropBeforeUnpinningMm3"].values()),60)

    def test_load_controls_and_bounded_mass(self):
        base,high,thin,narrow=read("demand.json")
        self.assertLess(high["M8EquivalentMPa"],600)
        self.assertGreater(high["M6ControlEquivalentMPa"],600)
        self.assertGreater(thin["weldEquivalentMPa"],high["weldEquivalentMPa"])
        self.assertGreater(narrow["weldEquivalentMPa"],high["weldEquivalentMPa"])
        self.assertLess(base["pilotOnlyRailBearingMPa"],high["pilotOnlyRailBearingMPa"])
        issued=read("accounting.json")
        self.assertEqual(issued["newPhysicalArticles"],40)
        self.assertFalse(any(issued["coverage"].values()))
        self.assertIsNone(issued["physicalMarginKg"])
        baseline=json.loads((ROOT/"axle-retention/accounting.json").read_bytes())
        delta=read("mass-input.json")["deltaKg"]
        self.assertAlmostEqual(issued["representedMarginKg"][0],baseline["representedMarginKg"][0]-delta[1])
        self.assertGreater(issued["representedMarginKg"][0],.25)
