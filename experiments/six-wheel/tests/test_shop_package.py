"""Keep shop coordinates, source identities and proposed joints tied to the checked design."""
import csv
from dataclasses import replace
import json
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
SHOP=ROOT/"design/shop-package"
sys.path.insert(0,str(SHOP))
import profiles
import drawings
import native_check as check
import inventory


class ShopRelease(unittest.TestCase):
    def test_source_and_dxf_operations(self):
        receipt=json.loads((SHOP/"native-review.json").read_text())
        self.assertEqual(receipt["stowedSha256"],check.STOWED_SHA)
        ids=set()
        for p in profiles.profiles():
            saved=receipt["drawingMismatchMm3"][p.key]
            self.assertEqual(set(saved),set(p.ids))
            self.assertLess(max(saved.values()),.01)
            ids.update(p.ids)
            with (SHOP/"patterns"/(p.key+".dxf")).open() as f:
                pairs=list(zip(*[iter(f.read().splitlines())]*2))
            types=[v for k,v in pairs if k=="0"]
            self.assertEqual(types.count("CIRCLE"),sum(not o.travel for o in p.holes))
            self.assertEqual(types.count("ARC"),2*sum(bool(o.travel) for o in p.holes))
            self.assertIn(("9","$INSUNITS"),pairs)
            self.assertEqual((SHOP/"patterns"/(p.key+".dxf")).read_text(),drawings.dxf(p))
        self.assertEqual(len(ids),27)
        self.assertEqual(len(inventory.inventory()),401)
        with (SHOP/"purchasing.csv").open(newline="") as f:
            self.assertTrue(all(len(row)==5 for row in csv.reader(f)))

    def test_upper_nose_and_counterbore_refuse_flat_or_through_substitution(self):
        shelf=next(p for p in profiles.profiles() if p.key=="r6-shelf")
        shape=check.machine(shelf)
        self.assertGreater(check.discrepancy(shape,check.machine(replace(shelf,radius=0))),3000)
        revised=[replace(o,depth=0) if o.depth else o for o in shelf.holes]
        self.assertGreater(check.discrepancy(shape,check.machine(replace(shelf,holes=revised))),2000)

    def test_joints_retain_real_negative_controls(self):
        r=json.loads((SHOP/"native-review.json").read_text())
        self.assertEqual(len(r["outerClamps"]),14)
        for c in r["outerClamps"].values():
            self.assertFalse(c["overlapMm3"] or c["axialToolMm3"])
            self.assertEqual(len(c["ordinaryToolWheelOverlapMm3"]),4)
            self.assertGreater(min(c["ordinaryToolWheelOverlapMm3"].values()),1400)
        for side in ("L","R"):
            q=r["receiverCrossblocks"][side]
            self.assertFalse(q["collisionMm3"])
            self.assertEqual(q["pinVolumeOutsidePreviousMaterialMm3"],[0]*4)
        self.assertAlmostEqual(r["addedNominalKg"],.5921,places=4)
