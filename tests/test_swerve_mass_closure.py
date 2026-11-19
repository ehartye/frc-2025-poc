import hashlib
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "experiments/swerve-drive/mass-closure"
sys.path.insert(0, str(DATA))
from inspection_recipe import CUT_METRES, wire_budget


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
