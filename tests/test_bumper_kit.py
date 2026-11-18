import json
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"design/bumper-kit"))
import bumper
import check_bumpers


class BumperKitReview(unittest.TestCase):
    def test_named_exports_and_both_alliances(self):
        result=json.loads((bumper.HERE/"review.json").read_text())
        for alliance in ("red","blue"):
            rows=bumper.specifications(alliance)
            shapes=bumper.cad.read_named(bumper.HERE/(alliance+".step"),[{"id":k} for k in rows])
            self.assertEqual(len(shapes),44)
            self.assertEqual(set(shapes),set(rows))
            receipt=result["alliances"][alliance]
            self.assertFalse(receipt["internalOverlapMm3"])
            self.assertLess(max(receipt["cornerVoidMm3"].values()),0.001)
            for half,sign in [("front",1),("rear",-1)]:
                b=bumper.cad.bounds(shapes["bumper-cover-"+half+"-"+alliance])
                self.assertAlmostEqual(b[3] if sign>0 else b[0],425*sign,places=5)
                self.assertAlmostEqual(b[1],-425,places=5)
                self.assertAlmostEqual(b[4],425,places=5)
        with self.assertRaises(ValueError):
            bumper.specifications("green")

    def test_driver_hits_old_chain_and_clears_new_datum(self):
        # Recreate the rejected Z119, 58 mm-inboard approach against the named front chain.
        from drive_parts import parts
        cad=bumper.cad
        source=next(p for p in parts() if p["id"]=="chain-1-L")
        chain=cad.brep(source["form"])
        a=next(s for s in bumper.anchors() if s.tag=="front-side-L")
        inside=bumper.FRAME-a.width
        stale=bumper.brick(a.tangent-10,a.tangent+10,inside-68,inside-48,0,127)
        self.assertGreater(cad.intersection(cad.brep(stale),chain),1000)
        for shape in check_bumpers.access(a).values():
            self.assertLess(cad.intersection(shape,chain),.001)

    def test_registered_hull_mounts_and_removal(self):
        self.assertEqual(len(list(bumper.anchors())),8)
        self.assertLessEqual(math.hypot(75,67),101.6)
        result=json.loads((bumper.HERE/"review.json").read_text())
        self.assertEqual(set(result["poses"]),set(bumper.cad.MODEL["poses"]))
        for pose in result["poses"].values():
            self.assertFalse(pose["overlapMm3"])
            self.assertGreater(pose["foreignBodies"],350)
        for group in ["moduleWithdrawalMm3","fastenerAccessMm3"]:
            self.assertTrue(all(not v for v in result["poses"]["stowed"][group].values()))
