"""Regression controls for clearance and source-bound end-stop sizing."""
import hashlib
import importlib.util
import json
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]/"design"/"lift-arrest"
spec = importlib.util.spec_from_file_location("arrest_sizing", HERE/"sizing.py")
sizing = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sizing)
RECEIPT = json.loads((HERE/"study.json").read_bytes())


class ShaftFailureArrest(unittest.TestCase):
    def test_sizing_and_failure_controls(self):
        current = sizing.study()
        self.assertEqual(RECEIPT["size"],current)
        self.assertTrue(current["highBalanced"]["fullStrokeCatalogScreen"])
        self.assertTrue(all(s["fullStrokeCatalogScreen"] for s in current["low7030"]+current["high7030"]))
        self.assertFalse(current["fullLoadOneUnit"]["fullStrokeCatalogScreen"])
        self.assertFalse(current["lowVelocityApproach"]["fullStrokeCatalogScreen"])
        self.assertGreater(current["midtravel"][0]["distanceWithIndexMm"],5)
        self.assertFalse(current["operatingPermission"])

    def test_source_identity_and_native_controls(self):
        red = HERE.parent/"competition-package"/"shop-duplex"/"red"
        registered = json.loads((red/"review.json").read_bytes())
        self.assertEqual(RECEIPT["mounts"]["frames"],hashlib.sha256((red/"part-frames.json").read_bytes()).hexdigest())
        for pose, fit in RECEIPT["bracketFit"].items():
            self.assertEqual(fit["source"],RECEIPT["heightLoss"][pose]["source"])
            self.assertEqual(fit["source"],registered[pose]["export_sha256"])
            self.assertEqual(fit["sourceBodies"],545)
            self.assertEqual(fit["intersectionsMm3"],{})
            self.assertEqual(fit["pinSweepMm3"],0)
        fit = RECEIPT["bracketFit"]["place"]
        self.assertEqual(fit["source"],RECEIPT["mounts"]["source"])
        self.assertEqual(fit["export"]["stepSha256"],hashlib.sha256((HERE/"end-stop-place.step").read_bytes()).hexdigest())
        hits = RECEIPT["mounts"]["singleSidePlateOverlapMm3"]
        self.assertEqual(max(hits["5"].values()),0)
        self.assertGreater(min(hits["5.679"].values()),16)
        candidates = RECEIPT["mounts"]["packagingMm"]
        for side in ("left","right"):
            self.assertFalse(candidates["-60.0,298.0"][side]["barrel"])
            self.assertTrue(candidates["-86.0,257.0"][side]["barrel"])
            self.assertGreater(candidates["-59.0,278.0"][side]["servicePinOverlapMm3"],0)

    def test_isolated_plate_relief_has_a_finite_limit(self):
        relief = json.loads((HERE/"plate-relief.json").read_bytes())
        self.assertEqual(relief["source"],RECEIPT["mounts"]["source"])
        for side in relief["casesBySide"].values():
            self.assertEqual(side["removedMm3"],432)
            for case in side["cases"]:
                if case["plate"] == "relieved":
                    self.assertEqual(bool(case["fixedContactMm3"]),case["dropMm"] == 12)
