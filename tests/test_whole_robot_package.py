import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
PKG=ROOT/"design/station-l1"
sys.path.insert(0,str(PKG))
from mechanical import MODEL,assembly


class PackageTests(unittest.TestCase):
    def test_four_pose_artifact_identities_and_load_budget(self):
        review=json.loads((PKG/"review.json").read_text())
        initial=json.loads((PKG/"first-fit.json").read_text())
        parts=assembly()
        self.assertEqual(len({p["id"] for p in parts}),96)
        with (PKG/"bom.csv").open() as stream:
            bom=list(csv.DictReader(stream))
        self.assertEqual({r["id"] for r in bom},{p["id"] for p in parts}|{"unmodeled-allocation"})
        for pose,obs in review["poses"].items():
            self.assertEqual(obs["stepSha256"],hashlib.sha256((PKG/(pose+".step")).read_bytes()).hexdigest())
            self.assertEqual((obs["shapeCount"],obs["pairCount"]),(96,4560))
            self.assertEqual(obs["interferencesOver0.001mm3"],[])
            self.assertEqual(len(initial["poses"][pose]["interferencesOver0.001mm3"]),6)
        io=json.loads((ROOT/"competition-controls/io-contract.json").read_text())
        self.assertEqual(io["posesMm"]["carry"],[MODEL["poses"]["transport"][k] for k in ["height","stroke"]])
        checks=json.loads((PKG/"checks.json").read_text())
        self.assertEqual(set(checks["identity"].values()),{"matched"})
        self.assertEqual(checks["missingBodyControl"],"mismatch")
        self.assertFalse(checks["powerStudy"]["all_axes_high_negative_control"]["aboveStudyFloor"])

    @unittest.skipUnless(importlib.util.find_spec("OCP"),"OCP required for live STEP measurements")
    def test_real_step_clearance_and_blocked_battery_removal(self):
        from build_assembly import read_named,check_pose,intersection,brep,bounds
        seen=read_named(PKG/"stowed.step")
        test=check_pose("stowed",seen)
        self.assertEqual(test["interferencesOver0.001mm3"],[])
        self.assertEqual(test["service"]["battery-lift"]["overlapsMm3"],{"battery-restraint":16125.0})
        self.assertEqual(test["service"]["panel-cover"]["overlapsMm3"],{})
        old_cross=brep(["box",[325,-325,50,25,650,100]])
        self.assertGreater(intersection(seen["wheel-R-250"],old_cross),600)
        self.assertAlmostEqual(bounds(seen["receiver-R6-ported"])[3],165)
