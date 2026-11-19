import hashlib
import json
import unittest
from pathlib import Path


PARTS = Path(__file__).resolve().parents[1] / "cover-retention"


def read(name):
    return json.loads((PARTS / name).read_bytes())


class SupportedChainCoverTests(unittest.TestCase):
    def test_installed_solids_and_service_share_exchange(self):
        fit = read("fit.json")
        service = read("service.json")
        digest = hashlib.sha256((PARTS / "supported-covers.step").read_bytes()).hexdigest()
        self.assertEqual(fit["stepSha256"], digest)
        self.assertEqual(service["kitSha256"], digest)
        self.assertEqual(len(fit["volumeMm3"]), 46)
        self.assertEqual(len(fit["railRemovedMm3"]), 12)
        self.assertEqual(len(fit["fits"]), 44)
        self.assertFalse(any(row["overlapMm3"] for row in fit["fits"].values()))
        self.assertGreater(min(row["continuousLowerMm"] for row in fit["drive"].values()), 1.73)
        for side in ("L", "R"):
            stages = service["stages"]
            for leg in ("lower-down", "upper-lift", "upper-out", "upper-down"):
                self.assertTrue(stages[f"{side}/{leg}"]["clear"])
            self.assertGreater(stages[f"{side}/wheel-in-place-overlapMm3"], 100)

    def test_compliance_budget_and_conditional_credit(self):
        model = read("compliance.json")
        remaining = model["certifiedRigidTranslationMm"] - sum(model["proposedAllowances"].values())
        self.assertAlmostEqual(remaining, model["remainingTrialDeflectionMm"])
        self.assertGreater(model["referencePanel"]["simplySupportedLinearMm"]["0.5"], remaining)
        cases = read("configurations.json")
        strict = cases["restrictedNoMotorCreditKg"]["representedMarginKg"][0]
        conditional = cases["conditionalMatchedArticleDeltaKg"]["representedMarginKg"][0]
        self.assertLess(strict, 0)
        self.assertAlmostEqual(conditional - strict, 0.4)
        for case in cases.values():
            self.assertIn("fixed-lead-install", case["selected"])
            self.assertIn("supported-drive-covers", case["selected"])
            self.assertIsNone(case["physicalMarginKg"])
