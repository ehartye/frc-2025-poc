import base64
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
import unittest


KIT=Path(__file__).resolve().parents[1]/"guide-mounts"


def read(name):
    return json.loads((KIT/name).read_bytes())


class GuideMounts(unittest.TestCase):
    def test_contact_and_full_stroke(self):
        fit=read("fit.json")
        self.assertEqual(hashlib.sha256((KIT/"guide-mounts.step").read_bytes()).hexdigest(),fit["stepSha256"])
        self.assertEqual(len(fit["volumesMm3"]),58)
        self.assertEqual((len(fit["fit"]),len(fit["tools"])),(56,24))
        for row in [*fit["fit"].values(),*fit["tools"].values()]:
            self.assertFalse(row["overlapMm3"])
            self.assertTrue(all(x>=-.001 for x in row["nearHarnessMm"].values()))
        for key,value in fit["interfaces"].items():
            if "OutsideOriginal" in key:
                self.assertAlmostEqual(value,0,places=3)
            else:
                self.assertGreater(value,0)
        for case in fit["negative"].values():
            self.assertTrue(case["overlapMm3"])
        old=fit["oldCableWitness"]
        self.assertAlmostEqual(old["strokeMm"],327.801,places=3)
        self.assertAlmostEqual(old["insideOldFaceMm3"],4*math.pi*.25**3/3,places=6)

    def test_bolts_do_not_locate_the_throat(self):
        datum=read("datum.json")
        self.assertEqual(datum["stepSha256"],read("fit.json")["stepSha256"])
        self.assertAlmostEqual(datum["alternateEndGapMm"],.015)
        self.assertIsNone(datum["actualMeasuredWidthMm"])
        for side in datum["checked"].values():
            self.assertFalse(side["materialInterferencesMm3"])
            self.assertGreater(min(side["mastSeatsAt0p1Mm3"]),48)

    def test_complete_service_module(self):
        service=read("service.json")
        self.assertEqual(len(service["uncoveredVolumeMm3"]),24)
        self.assertFalse(any(service["uncoveredVolumeMm3"].values()))
        for side in ("L","R"):
            for phase in ("inboard","forward"):
                self.assertFalse(service["collisions"][f"stowed/{side}/{phase}"])
            self.assertTrue(service["collisions"][f"stowed/{side}/unseat"])
        self.assertIn("wire/carrier",service["collisions"]["extended/R/outboard"])

    def test_signed_load_and_mass_controls(self):
        data=read("loads.json")
        self.assertIsNone(data["measuredStiffness"])
        for row in data["cases"].values():
            self.assertEqual(row["screenedStates"],171520)
            self.assertEqual(row["penetratingCellsAboveCut"],0)
            self.assertLess(row["highestPenetratingCellMm"],data["topMm"])
            nominal=row["layouts"]["folded"]["peakTravelMm"]
            self.assertGreater(nominal,.08)
            for adverse in ("flat","noRibs","halfCouple"):
                self.assertGreater(row["layouts"][adverse]["peakTravelMm"],nominal)
        result=read("accounting.json")
        delta=result["bill"]["rangeKg"][1]
        for cases in result["cases"].values():
            for case in cases.values():
                self.assertAlmostEqual(case["upperMarginKg"],case["priorUpperMarginKg"]-delta)
                self.assertEqual(case["count"],1189)
                self.assertIsNone(case["physicalMarginKg"])
            self.assertGreater(cases["completeGroupDeltaKg"]["upperMarginKg"],0)
            self.assertLess(cases["unknownGroupDeltaKg"]["upperMarginKg"],0)
        self.assertAlmostEqual(result["cases"]["noCutCreditKg"]["completeGroupDeltaKg"]["upperMarginKg"],.082,places=3)

    def test_viewer_uses_both_revisions(self):
        html=(KIT/"guide.html").read_text(encoding="utf8")
        blob=re.search(r'id="mesh" type="application/octet-stream">([^<]+)',html)[1]
        scenes=json.loads(gzip.decompress(base64.b64decode(blob)))
        self.assertEqual(set(scenes),{"original","revised","service"})
        self.assertEqual([scenes[k]["height"] for k in ("original","revised")],[850,813])
        for scene in scenes.values():
            self.assertIn("power tail",{part[0] for part in scene["parts"]})
