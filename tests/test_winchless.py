"""Pin the geometry and distinguish conditional mass credit from the physical scale result."""
import base64
import gzip
import hashlib
import json
import re
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parents[1] / "winchless"


def record(name):
    return json.loads((HERE / name).read_bytes())


class WinchlessStudy(unittest.TestCase):
    def test_support_faces_and_extraction(self):
        fit = record("fit.json")
        pin = hashlib.sha256((HERE / "service-stands.step").read_bytes()).hexdigest()
        self.assertEqual(pin, fit["stepSha256"])
        self.assertEqual(len(fit["parts"]), 16)
        self.assertIsNone(fit["measuredPadReactionsN"])
        self.assertEqual(set(fit["contacts"]), {"rear-L", "rear-R", "front-L", "front-R"})
        for key, contact in fit["contacts"].items():
            self.assertAlmostEqual(contact["areaMm2"], 254.469, places=2)
            checks = contact["checks"]
            self.assertGreater(checks["twoMmOvertravelMm3"], 508)
            for trial in checks.values():
                if isinstance(trial, dict) and "overlapMm3" in trial:
                    self.assertFalse(trial["overlapMm3"])
            if key.startswith("rear"):
                self.assertGreater(checks["aftHeadRailCollisionMm3"], 134)
                self.assertGreater(min(checks["captureOverlapMm3"].values()), 1)
        braces = record("brace-access.json")
        self.assertEqual(braces["standSha256"], pin)
        self.assertEqual(braces["bracketSha256"], fit["carriageSupportSha256"])
        tools = [tool for side in braces["clearances"].values() for tool in side.values()]
        self.assertEqual(len(tools), 54)
        self.assertGreaterEqual(min(tool["nearestMm"] for tool in tools), 8)
        self.assertFalse(any(tool["overlapMm3"] for tool in tools))

    def test_group_credit_and_scale_controls(self):
        for filename, count in (("budget.json", 1083), ("supported-budget.json", 1133)):
            for shelf, row in record(filename)["shelfCases"].items():
                evidence = row["evidence"]
                self.assertEqual(len(evidence["originalOccurrences"]), 34)
                self.assertEqual(evidence["capAndSecondaryKitExtraCreditKg"], 0)
                self.assertFalse(set(evidence["removedIdentities"]) & set(evidence["requiredRetained"]))
                full = row["cases"]["completeGroupDeltaKg"]
                unknown = row["cases"]["unknownGroupDeltaKg"]
                for case in (full, unknown):
                    self.assertEqual(case["installedCount"], count)
                    self.assertIsNone(case["physicalMarginKg"])
                self.assertAlmostEqual(unknown["upperMarginKg"], row["priorUpperMarginKg"])
                self.assertAlmostEqual(full["upperMarginKg"] - unknown["upperMarginKg"], 1.8)
                if count == 1133:
                    target = 1.593 if shelf == "densityMatchedRemovalKg" else .673
                    self.assertAlmostEqual(full["upperMarginKg"], target, places=3)
        loads = record("service-loads.json")
        self.assertIsNone(loads["actualScaleTrial"])
        self.assertTrue(loads["hypotheticalEvenTrial"]["accepted"])
        self.assertFalse(loads["hypotheticalRearOnlyTrial"]["accepted"])

    def test_viewer_contains_full_fixture(self):
        html = (HERE / "service.html").read_text(encoding="utf8")
        packed = re.search(r'id="mesh" type="application/octet-stream">([^<]+)', html)[1]
        scenes = json.loads(gzip.decompress(base64.b64decode(packed)))
        self.assertEqual(set(scenes), {"approach", "supported", "keeper"})
        full = {row[0] for row in scenes["supported"]["parts"]}
        self.assertTrue(set(record("fit.json")["parts"]) <= full)
        self.assertIn("support-machined-bridge", full)
        self.assertIn("rear-L-keeper", {row[0] for row in scenes["keeper"]["parts"]})
