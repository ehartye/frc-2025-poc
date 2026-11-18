"""Check the selected 545-body make/buy register and separate machining proposal."""
import csv
import hashlib
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT/"design/installed-procurement"
BASE = ROOT/"design/competition-package/shop-duplex"


def read_json(p):
    return json.loads(p.read_bytes())


def read_rows(p):
    with p.open(encoding="utf-8",newline="") as f:
        return list(csv.DictReader(f))


class InstalledProcurementTest(unittest.TestCase):
    def test_both_pose_registrations_and_unique_article_counts(self):
        rows = read_rows(HERE/"installed.csv")
        for color in ("red","blue"):
            frames = read_json(BASE/color/"part-frames.json")
            self.assertEqual({r[color+"Id"]:r["frame"] for r in rows},frames)
        rollup = read_rows(HERE/"item-rollup.csv")
        self.assertEqual(sum(int(r["nativeBodies"]) for r in rollup),545)
        by_item = {r["item"]:int(r["representedUnitsPerAlliance"]) for r in rollup}
        expected = {"REV-21-1650 NEO V1.1":3,"REV-11-2158 SPARK MAX":6,
                    "am-4559 primary line":2,"terminated secondary line":2,
                    "SP1-25 draw wire":0,"outer threaded U saddle":6,
                    "M5 x 16 outer cap screw":24}
        for name,count in expected.items():
            self.assertEqual(by_item[name],count,name)
        for row in rows:
            self.assertTrue((ROOT/row["source"]).is_file(),row["source"])

    def test_candidate_identity_service_and_mass_gate(self):
        heavy = read_json(HERE/"steel-review.json")
        light = read_json(HERE/"clamp-review.json")
        reference = read_json(BASE/"red/review.json")
        digest = hashlib.sha256((HERE/"inner-clamps.step").read_bytes()).hexdigest()
        self.assertEqual(light["candidateStepSha256"],digest)
        for case,qty in ((heavy,70),(light,86)):
            self.assertEqual(case["reopenedNamedBodies"],qty)
            self.assertEqual(len(case["replaces"]),6)
            self.assertEqual(len(case["takeup"]),14)
            self.assertEqual(set(case["poses"]),set(reference))
            for pose,row in case["poses"].items():
                self.assertEqual(row["redStepSha256"],reference[pose]["export_sha256"])
                self.assertFalse(row["intersectionsMm3"])
                self.assertEqual(row["retainedBodies"],539)
            for station,tool in case["toolAccess"].items():
                self.assertEqual((tool["diameterMm"],tool["reachMm"]),(5,25))
                self.assertEqual(bool(tool["requiresRemoval"]),station.endswith("center"))
        margin = read_json(BASE/"mass.json")["poses"]["transport"]["R103_margin_at_high_kg"]
        self.assertGreater(heavy["deltaKg"]-light["deltaKg"],.29)
        self.assertLess(margin-heavy["deltaKg"]-.508,0)
        self.assertTrue(0 < margin-light["deltaKg"]-.508 < .01)

    def test_pocket_operations_and_thread_count(self):
        ops = read_rows(HERE/"machining.csv")
        self.assertEqual(sum(r["operation"]=="square-pocket-10.20" for r in ops),4)
        self.assertEqual(sum(r["operation"]=="corner-relief" for r in ops),16)
        self.assertEqual(sum(r["operation"].startswith("tap-M5") and r["part"]=="center-faceplate" for r in ops),8)
        self.assertEqual(sum(r["operation"]=="window" for r in ops),2)
