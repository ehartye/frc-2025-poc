import hashlib
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "experiments/swerve-drive/mass-closure"
sys.path.insert(0, str(DATA))
from inspection_recipe import CUT_METRES, wire_budget
from r4_receiving import margin_case, tail_span


class InspectionRecipeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.proof = json.loads((DATA / "recipe.json").read_bytes())
        power = DATA.parent / "power"
        cls.wire = json.loads((power / "harness.json").read_bytes())
        cls.paths = json.loads((power / "paths.json").read_bytes())[
            "groupedSupplyTargets"
        ]

    def test_positive_and_adverse_cases_use_same_unidentified_allocation(self):
        cases = self.proof["cases"]
        self.assertEqual(self.proof["construction"]["originalUnidentifiedKg"], [2, 5])
        self.assertAlmostEqual(
            cases["drawingSelected"]["r103MarginKg"], 0.327030342, places=7
        )
        for name in (
            "drawingsWithNoShelfCredit",
            "drawingsWithAllWinchResidue",
            "drawingsWithoutOldClampCorrection",
            "uniformOnePointTwoMetres",
            "longFullSurrender",
        ):
            with self.subTest(name=name):
                self.assertLess(cases[name]["r103MarginKg"], 0)
        self.assertIsNone(self.proof["measuredInspectionKg"])

    def test_native_retirement_and_source_files(self):
        rear = self.proof["rearElectronics"]
        final = self.proof["matchedInstalledChecker"]
        self.assertEqual(
            (rear["drawingKitShapes"], rear["parentCount"], final["installedCount"]),
            (27, 927, 944),
        )
        self.assertFalse(any(final["coverage"].values()))
        self.assertIsNone(final["physicalMarginKg"])
        self.assertAlmostEqual(final["estimatedMarginKg"][0], 0.169928939, places=7)
        joined = self.proof["joinedCases"]
        self.assertAlmostEqual(
            joined["drawingSelected"]["r103UpperKg"], final["representedRangeKg"][1]
        )
        self.assertLess(joined["uniformOneMetre"]["r103MarginKg"], 0)
        self.assertEqual(len(rear["relocatedNativeBodies"]), 5)
        self.assertAlmostEqual(
            sum(rear["positiveNewMaterialKg"].values()) * 1.10 + 0.080,
            rear["upperAllowanceKg"],
        )
        material = self.proof["originalMaterial"]
        self.assertEqual(len(material["formerOuterClampVolumesMm3"]), 54)
        self.assertAlmostEqual(
            sum(material["formerOuterClampVolumesMm3"].values()) * 7.85e-6,
            material["originalShopOuterClampOnlyKg"],
            places=6,
        )
        self.assertEqual(len(material["possibleWinchResidue"]), 7)
        self.assertAlmostEqual(
            material["removedPEVolumeMm3"] * 930e-9,
            material["densityMatchedR6CreditKg"],
        )
        for name, digest in self.proof["sourceEvidence"].items():
            with self.subTest(path=name):
                self.assertEqual(
                    hashlib.sha256((ROOT / name).read_bytes()).hexdigest(), digest
                )

    def test_power_cuts_cover_all_four_and_reject_an_unreachable_route(self):
        mass, slack = wire_budget(self.wire, self.paths, CUT_METRES)
        self.assertAlmostEqual(mass, self.proof["construction"]["issuedWireUpperKg"])
        self.assertGreaterEqual(min(slack.values()), 0.42)
        for bad in (
            dict(CUT_METRES, FR=0.85),
            {k: v for k, v in CUT_METRES.items() if k != "FL"},
            dict(CUT_METRES, FL=1.80),
            dict(CUT_METRES, FL=float("nan")),
        ):
            with self.assertRaises(ValueError):
                wire_budget(self.wire, self.paths, bad)

    def test_numbered_r4_module_and_harness_boundaries(self):
        r4 = json.loads((DATA / "r4-receiving.json").read_bytes())
        source = ROOT / "cad/full-swerve-r4"
        self.assertEqual(hashlib.sha256((source/"full-robot.step").read_bytes()).hexdigest(), r4["sourceStepSha256"])
        self.assertEqual(r4["accounting"]["physicalIds"], 944)
        self.assertTrue(r4["archiveBytesVerified"])
        issued = r4["case"]["issued"]
        cap = self.proof["matchedInstalledChecker"]["ruleLimitKg"]
        receiving = r4["populatedReceivingBudget"]["issuedCuts"]["forWorkingReserveKg"]
        # Unequal modules spend the same aggregate allowance, and additional
        # installed mass is charged once at precisely this boundary.
        result = margin_case(cap, issued["r103UpperKg"], sum(issued["populatedModulesKg"]),
                             issued["fullHarnessUpperKg"], [1.7,1.8,1.72,receiving-5.22],
                             issued["fullHarnessUpperKg"], 0.01)
        self.assertAlmostEqual(result["r103MarginKg"], 0.49)
        self.assertLess(r4["case"]["oneMetreAtReceivingUpper"]["r103MarginKg"], 0)
        self.assertGreater(r4["case"]["oneMetreAtCatalogNominals"]["r103MarginKg"], 0)
        self.assertEqual(len(r4["tailSensitivity"]), 8)
        self.assertEqual(tail_span([0,0,0,1,2,2],[0,0,0]), (0,3))
