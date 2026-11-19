import hashlib
import json
from pathlib import Path
import unittest
from xml.etree import ElementTree

ROOT=Path(__file__).resolve().parents[1]
HERE=ROOT/"carriage-support"


def read(name):return json.loads((HERE/name).read_bytes())


class CarriageSupport(unittest.TestCase):
    def test_four_machined_members_and_every_assembled_joint(self):
        row=read("fit.json")
        self.assertEqual(row["stepSha256"],hashlib.sha256((HERE/"carriage-support.step").read_bytes()).hexdigest())
        self.assertEqual(len(row["volumesMm3"]),54)
        self.assertEqual(len(row["installed"]),50)
        for phase in ("installed","tools","service"):
            self.assertTrue(row[phase])
            self.assertFalse(any(row[phase].values()))
        self.assertLess(row["old335mmScrewPowerIntrusionMm"],-4)
        for side in ("L","R"):
            self.assertGreater(min(row["bearingMm3"][side].values()),1)
            self.assertIn(side+"/front0TopExtraction",row["tools"])
            self.assertIn(side+"/rearScrewWithdrawal",row["tools"])
            self.assertIn(side+"/forward/web0",row["service"])
        ElementTree.parse(HERE/"shop.svg")

    def test_coupled_result_exposes_datum_sensitivity(self):
        d=read("reactions.json")
        self.assertIsNone(d["measuredShare"])
        self.assertAlmostEqual(d["pitchArmMm"],254)
        source=json.loads((ROOT/"shelf-structure/coupled-mounts.json").read_bytes())
        self.assertEqual(d["traceRegistrySha256"],hashlib.sha256((ROOT/"shelf-structure/coupled-mounts.json").read_bytes()).hexdigest())
        for case in source:
            r=d["results"][case["case"]]
            self.assertEqual(r["sha256"],case["supportHistorySha256"])
            self.assertEqual(r["samples"],4801)
            self.assertEqual(set(r["cases"]),{f"{yaw:+.0f}Nm/{share:.2f}" for yaw in (-50,50) for share in (.35,.5,.65)})
            values=[q for s in r["cases"].values() for q in s.values()]
            self.assertGreater(max(-q["sampledSpanBendingDeflectionMm"][0] for q in values),1.)
            for half in ("lower","upper"):
                c=max((q["chordBuckling"][half] for q in values),key=lambda e:e["compressionN"])
                self.assertGreater(c["pinPinCriticalN"],c["compressionN"])
                self.assertLess(c["cantileverCriticalN"],c["compressionN"])
            self.assertGreater(max(q["rearResidualIfTappedScrewMissingN"] for q in values),0)

    def test_installed_shelf_alternatives_do_not_reuse_weight_credit(self):
        results=read("accounting.json")
        self.assertEqual(set(results),{"noCutCreditKg","densityMatchedRemovalKg"})
        for case in results.values():
            self.assertEqual(case["configured"],1167)
            self.assertIsNone(case["physicalMarginKg"])
            money=case["bill"]
            self.assertEqual(money["oldStockAndChipCreditKg"],0)
            self.assertEqual(sum(g["installed"] for g in money["groups"].values()),50)
            self.assertAlmostEqual(case["representedMarginKg"][0],case["priorMarginKg"][0]-money["deltaKg"][1])
            self.assertLess(case["representedMarginKg"][0],0)
        self.assertEqual(results["noCutCreditKg"]["bill"],results["densityMatchedRemovalKg"]["bill"])
