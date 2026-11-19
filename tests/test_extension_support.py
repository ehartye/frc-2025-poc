"""Check the native stroke and the two signed static load paths."""

import json
import unittest
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "extension-support"
sys.path.insert(0, str(HERE))
from balance import balance, digest, reaction
from fixture import support_plane


def record(name):
    return json.loads((HERE / name).read_bytes())


class ExtensionSupportTest(unittest.TestCase):
    def test_stock_joints_open_upward_and_lose_overhung_load(self):
        native = record("geometry.json")
        self.assertEqual(
            record("balance.json")["nativeSha256"], digest(HERE / "geometry.json")
        )
        for side in ("L", "R"):
            r = native["contacts"][side]["370"]
            for key, target in (
                ("innerOnMiddle", (210, 345)),
                ("middleOnOuter", (25, 160)),
            ):
                self.assertGreater(r[key]["floorCommonMm3"], 0)
                self.assertEqual(r[key]["upwardCommonAt1p05Mm3"], 0)
                for actual, expected in zip(r[key]["downwardXMm"], target):
                    self.assertAlmostEqual(actual, expected, places=4)
        for c in record("balance.json")["cases"].values():
            self.assertTrue(
                c["strokes"]["0"]["middleOnOuter"]["compressionOnlyAdmissible"]
            )
            self.assertFalse(
                c["strokes"]["370"]["middleOnOuter"]["compressionOnlyAdmissible"]
            )

    def test_button_support_changes_with_the_actual_underside(self):
        buttons = record("geometry.json")["fixedButtons"]
        for idx in (0, 1):
            self.assertAlmostEqual(buttons["0"][idx]["firstGapMm"], 0.1, places=5)
            self.assertGreater(buttons["10"][idx]["firstGapMm"], 8)
            self.assertLess(buttons["25"][idx]["firstGapMm"], 0.5)
            self.assertTrue(buttons["218"][idx]["overhead"])
            self.assertIsNone(buttons["222"][idx]["firstGapMm"])

    def test_multiple_stroke_positions_have_real_captive_contacts(self):
        pins = record("keepers.json")
        self.assertEqual(pins["stepSha256"], digest(HERE / "rail-fixture.step"))
        self.assertEqual(len(pins["witnesses"]), 6)
        for w in pins["witnesses"].values():
            self.assertGreater(w["-1.05"]["middleFloorToOuterMm3"], 0)
            self.assertEqual(w["0.34"]["innerPinToMiddleMm3"], 0)
            self.assertEqual(max(w["0.34"]["middleToOuterDogsMm3"]), 0)
            self.assertGreater(w["0.36"]["innerPinToMiddleMm3"], 0)
            self.assertGreater(min(w["0.36"]["middleToOuterDogsMm3"]), 0)
        for k in pins["retainingRingWitnesses"].values():
            self.assertEqual(k["installedCommonMm3"], 0)
            self.assertGreater(k["axiallyShiftedClipCommonMm3"], 0)

    def test_equilibrium_preserves_signed_reactions_and_allows_a_floor_only_control(
        self,
    ):
        load = reaction([(120, 30)], (0, 100))
        result = support_plane(load, 10, 10000, 10000)
        self.assertAlmostEqual(result["rearReactionN"] + result["frontReactionN"], 30)
        self.assertAlmostEqual(
            10 * result["rearReactionN"] + 100 * result["frontReactionN"], 3600
        )
        self.assertLess(result["rearReactionN"], 0)
        self.assertEqual(
            support_plane(reaction([(50, 30)], (0, 100)), 10, 10000, 10000)["regime"],
            "two lower lands",
        )
        with self.assertRaises(ValueError):
            support_plane(load, 101, 10000, 10000)
        with self.assertRaises(ValueError):
            balance(record("geometry.json"), 50, float("nan"), 0.8)
        with self.assertRaises(ValueError):
            reaction([(0, 0)], (0, 100))

    def test_selected_cad_and_roster_agree(self):
        mass = record("mass.json")
        clearance = record("clearance.json")
        fixture = record("fixture.json")
        sha = digest(HERE / "rail-fixture.step")
        self.assertEqual(mass["stepSha256"], sha)
        self.assertEqual(clearance["fixtureSha256"], sha)
        self.assertEqual(fixture["keeperSha256"], sha)
        self.assertEqual(
            (len(mass["removed"]), len(mass["added"]), mass["installed"]), (6, 16, 1199)
        )
        self.assertGreater(mass["upperMarginKg"], 0.28)
        for item in clearance["newMaterial"].values():
            for p in item["nearbyShapes"].values():
                self.assertFalse(p["unresolved"])
                self.assertLessEqual(p["commonMm3"], 0.001)
            self.assertTrue(
                all(gap > 0 for gap in item["nearbyCableGapLowerMm"].values())
            )
        for case in fixture["cases"].values():
            for k in ("compliant", "nominal", "stiff"):
                self.assertLess(case[k]["370"]["frontChangeFromStowMm"], -6)
                self.assertGreater(case[k]["370"]["frontChangeFromStowMm"], -8)
