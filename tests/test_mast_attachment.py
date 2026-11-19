import hashlib
import json
from pathlib import Path
import unittest
from xml.etree import ElementTree

ROOT=Path(__file__).resolve().parents[1]
HERE=ROOT/"mast-attachment"


def read(name): return json.loads((HERE/name).read_bytes())


class MastAttachment(unittest.TestCase):
    def test_native_identity_bearing_and_reversible_service(self):
        f=read("fit.json")
        for name,key in (("mast-joints.step","stepSha256"),("slotted-ribbed-deck.step","ribbedSlottedSha256")):
            self.assertEqual(hashlib.sha256((HERE/name).read_bytes()).hexdigest(),f[key])
        self.assertEqual(len(f["volumeMm3"]),63)
        self.assertEqual(set(f["extractionJogXmm"]),{"L","R"})
        for group in ("material","tools"):
            for row in f[group].values():
                self.assertFalse(row["overlapMm3"])
                self.assertEqual(row.get("alternativeDeckCommonMm3",0),0)
                if row["closeRoutedLeadMarginMm"]:
                    self.assertGreaterEqual(row["closeRoutedLeadMarginMm"][0],0)
        self.assertTrue(all(min(q.values())>3 for q in f["halfMmBearingChainCommonMm3"].values()))
        self.assertAlmostEqual(f["ribbedDeckRemovedMm3"],f["stockRemovedMm3"]["tie-machined-deck-main"],places=2)
        for side in ("L","R"):
            self.assertGreater(f["serviceControls"][side+"/directOutward"]["overlapMm3"]["backup-"+side+"-bearing-support"],1000)
        ctl=read("controls.json")
        self.assertGreater(ctl["deepLegFromZ60Mm3"]["post--120_64-R"],100)
        self.assertGreater(ctl["unrelievedRibMm3"]["roof-host-trough-01"],20)
        ElementTree.parse(HERE/"shop.svg")

    def test_signed_combination_and_unilateral_failure(self):
        data=read("load.json")
        self.assertEqual(len(data["cases"]),4)
        for cases in data["cases"].values():
            paired=[r for k,r in cases.items() if k.startswith("paired")]
            one=[r for k,r in cases.items() if k.startswith("single")]
            self.assertLess(max(r["spanGrossUpperMPa"] for r in paired),400)
            self.assertGreater(min(r["sectionMPa"] for r in one),800)
            for r in paired:
                self.assertEqual(len(r["group"]["rail"]),5)
        adverse=data["independentAdverseSingleSide"]
        self.assertGreater(min(r["sectionMPa"] for r in adverse.values()),2000)
        d=read("damper.json")
        self.assertTrue(d["includesBothPlungersOnce"])
        for row in d["cases"].values():
            self.assertLess(row["minimumChassisFzN"],-2200)
            self.assertGreater(row["sampleCount"],4500)
            self.assertLess(max(v["spanGrossUpperMPa"] for k,v in row["bounds"].items() if not k.startswith("single")),400)
        self.assertGreater(read("damper-clearance.json")["minimumSeparationMmThroughSlideStroke"],110)

    def test_every_deck_choice_charges_the_same_hardware(self):
        cases=read("comparison.json")
        self.assertEqual(len(cases),6)
        fit=read("fit.json")
        delta=None
        for name,row in cases.items():
            self.assertEqual(row["addedOccurrences"],63)
            self.assertEqual(len(row["replaces"]),5)
            self.assertIn("integral-rib-deck" if name.startswith("ribbed/") else "tie-machined-deck-main",row["replaces"])
            self.assertEqual(row["charge"]["drillingAndSlotCreditKg"],0)
            self.assertEqual(row["charge"]["stepSha256"],fit["stepSha256"])
            if delta is None:delta=row["charge"]["deltaKg"]
            self.assertEqual(row["charge"]["deltaKg"],delta)
            self.assertLess(row["representedMarginKg"][0],0)
            self.assertIsNone(row["physicalMarginKg"])
        self.assertTrue(.63<delta[1]<.65)
