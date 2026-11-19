"""Regression for one-eye calibration, measured dock correction and distinct receiver histories."""

import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
RECEIPT = ROOT / "experiments/swerve-drive/reef-correction/results.json"


class ReefCorrection(unittest.TestCase):
    def test_individual_frame_calibration_and_control_have_independent_evidence(self):
        data = json.loads(RECEIPT.read_bytes())
        for side, n in [("plus", 29), ("minus", 28)]:
            row = data["cases"][side]
            used = [s for s in row["exposures"] if s["residualPx"] <= 0.18]
            self.assertEqual(len(used), n)
            self.assertTrue(all(s["actualMaxCornerErrorPx"] <= 0.25 for s in used))
            self.assertLess(abs(row["terminalMmMmRad"][0] + 54), 1)
            self.assertLess(abs(row["terminalMmMmRad"][1] + 180), 1)
            for v, lo, hi in zip(row["terminalMmMmRad"], *row["enclosure"]):
                self.assertTrue(lo <= v <= hi)
            self.assertTrue(row["physicalStop"]["confirmed"])
            self.assertTrue(row["cargo"]["allGranted"])
            self.assertAlmostEqual(row["endWallS"], row["cargo"]["endWallS"])
            self.assertEqual(row["cargo"]["wheels"], row["route"])
            self.assertGreater(row["conditionalWallClearance"]["minimumMm"], 3)
            for failure in row["adverseControls"]:
                self.assertTrue(failure["stopped"]["confirmed"])
                self.assertLess(failure["travelDuringBrakeMm"], 2.1)
            sources = row["stationSources"]
            self.assertNotEqual(sources["originalTrace"], sources["camTrace"])
            self.assertNotEqual(sources["originalState"], sources["camState"])
            self.assertEqual(row["cargo"]["deliveryCredit"], 0)
