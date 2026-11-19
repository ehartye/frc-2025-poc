import hashlib
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
REPORT = json.loads((ROOT / "fixed-low-power/dual-camera.json").read_bytes())


class DualCameraSupply(unittest.TestCase):
    def test_registered_robot_and_clock(self):
        for file, expected in REPORT["sourceSha256"].items():
            self.assertEqual(hashlib.sha256((ROOT / file).read_bytes()).hexdigest(), expected, file)
        for window in REPORT["rackWindows"]:
            piece = window["piece"]
            self.assertLessEqual(window["startWallS"], REPORT["integratedHal"]["commandWallMs"][piece] / 1000)
            self.assertGreaterEqual(window["endWallS"], REPORT["integratedHal"]["stowWallMs"][piece] / 1000)
        self.assertGreater(REPORT["integratedHal"]["peakSlideInputW"], 10)

    def test_weak_battery_and_hot_connector(self):
        for scenario in ["nominal", "screened", "conversionLoss"]:
            self.assertEqual(REPORT["cases"][scenario]["ticks"], 7800)
            self.assertGreater(REPORT["cases"][scenario]["weakestV"], 9.5)
        self.assertGreater(REPORT["cases"]["weak"]["belowFloorS"], 2.8)
        for name, inlet in REPORT["inlets"].items():
            self.assertAlmostEqual(inlet["conditionalMinV"], 5.12 - inlet["loadA"] * inlet["loopOhm"], places=4)
            self.assertGreaterEqual(inlet["conditionalMinV"], 5.0, name)
            self.assertLessEqual(inlet["conditionalMaxV"], 5.25, name)
        self.assertLess(REPORT["extra50mOhmMovingLoop"]["conditionalMinV"], 5.0)

    def test_screened_case_uses_three_independent_branches(self):
        result = REPORT["integratedScreenedPack"]
        self.assertEqual(set(result["converters"]), {"Host", "FixedCamera", "MovingCamera"})
        self.assertEqual(result["weakestAt"], [90.38, "DriveRight"])
        self.assertGreater(result["minimumSlideTerminalInsideNominalPullsV"], 9.5)
        for row in result["converters"].values():
            self.assertGreater(row["minInputV"], 10.)
            self.assertLess(row["peakInputA"], 2.)

    def test_panel_clearance_and_fixed_lead_budget(self):
        panel = json.loads((ROOT / "design/fixed-no-air/converter-panel.json").read_bytes())
        self.assertEqual(set(panel["poses"]), {"fixed-no-air/pickup", "fixed-no-air/placement", "bridge-tolerance/place"})
        for pose in panel["poses"].values():
            self.assertEqual(pose["retainedParts"], 591)
            self.assertGreater(pose["panelClearance"]["mm"], 3.6)
            self.assertGreater(pose["directLeadCradleInterferenceMm3"], 132)
            for foot in pose["feet"].values():
                self.assertEqual(foot["bearingFraction"], 1.)
                self.assertGreaterEqual(foot["otherClearance"]["mm"], 3.)
            sections = pose["fixedCameraLeadSegments"]
            length = sum(s["lengthMm"] for s in sections) / 1000
            self.assertGreater(length, 1.)
            self.assertLess(length + .24, 1.30)
            self.assertGreaterEqual(min(s["otherClearance"]["mm"] for s in sections), 10.)
            self.assertAlmostEqual(REPORT["inlets"]["FixedCamera"]["loopOhm"],
                                   2 * 1.25 * .008286 * 1.30 + .055, places=5)
