import hashlib
import json
import math
from pathlib import Path
import unittest

FOLDER = Path(__file__).resolve().parents[1] / "design/shared-height"


class SharedHeight(unittest.TestCase):
    def test_two_roles_use_the_same_shape(self):
        geometry = json.loads((FOLDER / "native.json").read_bytes())
        self.assertEqual(geometry["mast_step_sha256"], hashlib.sha256((FOLDER / "masts.step").read_bytes()).hexdigest())
        self.assertEqual(len(geometry["common_solids_mm3"]), 38)
        self.assertLess(max(geometry["common_solids_mm3"].values()), .01)
        self.assertLess(max(geometry["reopen_difference_mm3"].values()), .01)
        self.assertEqual(len(geometry["rails"]), 6)
        for rail in geometry["rails"].values():
            self.assertLess(rail["outside_low_stock_mm3"], .01)
            self.assertAlmostEqual(rail["removed_mm3"], 6 * math.pi * 3.1**2, places=3)

    def test_hole_preserves_a_wall_ring_but_reduces_bending_section(self):
        holes = json.loads((FOLDER / "native.json").read_bytes())["critical_holes"]
        for side in "LR":
            for mode in ("low", "high"):
                row = holes[f"{mode}-{side}"]
                self.assertGreater(row["unmodified_sleeve_path_interference_mm3"], 450)
                self.assertLess(row["new_sleeve_path_interference_mm3"], .01)
                initial, final = row["before_section"], row["after_section"]
                self.assertAlmostEqual(final["area_mm2"] / initial["area_mm2"], .830, places=3)
                self.assertAlmostEqual(final["Iy_mm4"] / initial["Iy_mm4"], .690, places=3)
                self.assertGreater(final["stress_MPa_per_100N_at_380mm"], 15)
            for z in (422.5, 802.5):
                for missing in holes[f"{side}-{z}"]["missing_annular_wall_volume_mm3"].values():
                    self.assertLess(abs(missing), .01)

    def test_inspection_union_and_exclusions(self):
        report = json.loads((FOLDER / "inspection.json").read_bytes())
        self.assertEqual(report["cases"]["red"], report["cases"]["blue"])
        case = report["cases"]["red"]
        offered = case["I103"]
        self.assertEqual(case["high"]["physicalIds"], offered["uniqueRepresentedIds"])
        self.assertEqual(case["low"]["physicalIds"] + len(offered["presentedLooseInLow"]), offered["uniqueRepresentedIds"])
        self.assertEqual(len(offered["presentedLooseInLow"]), 6)
        alternate = case["dedicatedMastPairs"]
        self.assertEqual(alternate["uniqueRepresentedIds"], offered["uniqueRepresentedIds"] + 2)
        self.assertGreater(alternate["additionalLowPairUpperKg"], 1)
        self.assertLess(alternate["upperHeadroomKg"], offered["upperHeadroomKg"])
        for mode in ("high", "low"):
            row = case[mode]
            for i in (0, 1):
                self.assertAlmostEqual(row["R408Kg"][i] - row["R103Kg"][i], report["bumperAllowanceKg"][i])
            self.assertGreater(row["R408UpperHeadroomKg"], 2)
            self.assertIsNone(row["finishedMarginKg"])
        self.assertIsNone(offered["finishedMarginKg"])
        self.assertIsNone(alternate["finishedMarginKg"])
