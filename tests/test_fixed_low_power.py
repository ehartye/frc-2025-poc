import hashlib
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "fixed-low-power"


class FixedLowPower(unittest.TestCase):
    def setUp(self):
        self.result = json.loads((STUDY / "results.json").read_text())
        self.cases = self.result["cases"]
        self.protection = json.loads((STUDY / "protection.json").read_text())

    def test_registered_inputs_and_complete_nominal_clock(self):
        for name, expected in self.result["cadAndContactSourceHashes"].items():
            raw = (ROOT / name).read_bytes().replace(b"\r\n", b"\n")
            self.assertEqual(hashlib.sha256(raw).hexdigest(), expected, name)
        route = self.cases["park372-fixed"]
        self.assertEqual(route["durationS"], 156)
        self.assertEqual(route["worstTerminal"], [90.38, "carrying coral-B", "DriveRight"])
        self.assertEqual(route["belowFloorS"] + route["overPhaseCapS"] + route["unsolvedS"], 0)
        self.assertAlmostEqual(route["minDeviceV"], 10.50, places=2)
        self.assertEqual(route["branches"]["Lift"]["peakBranchA"], 0)
        self.assertEqual(set(route["branches"]), set(self.protection["branches"]))

    def test_weak_source_rejects_nominal_motion(self):
        for name, seconds in (("park372-weak", 8.22), ("park372-weakFixed", 2.54)):
            self.assertEqual(self.cases[name]["belowFloorS"], seconds)
        screened = self.cases["park372-screenedPack"]
        self.assertGreater(screened["minDeviceV"], 9.60)
        self.assertLess(screened["minDeviceV"], 9.61)
        self.assertLess(screened["commonOhm"], screened["maxCommonOhmAtThisVoc"])
        self.assertGreater(self.cases["park372-depleted"]["unsolvedS"], 0)

    def test_stall_and_observer_limits(self):
        jam = self.cases["jammedSlide-fixed"]
        self.assertGreater(jam["overPhaseCapS"], 10)
        self.assertGreater(jam["branches"]["Slide"]["peakPhaseA"], 20)
        self.assertLess(jam["branches"]["Slide"]["peakBranchA"], 15)
        self.assertGreater(self.protection["holdJam"]["modeledLeftWindingJ"], 990)
        for device in self.protection["observations"].values():
            self.assertLess(device["catalogWorstTerminalV"], 5.0)
            self.assertGreaterEqual(device["selectedTerminalRangeV"][0], 5.0)
            self.assertLessEqual(device["selectedTerminalRangeV"][1], 5.25)
            self.assertTrue(device["directMeasurementRequired"])
        off = self.result["independentMainOff"]
        self.assertFalse(any(off["branchA"].values()) or off["observerAvailable"] or off["motionAuthorized"])

    def test_retreat_both_identities(self):
        retreat = json.loads((STUDY / "retreat.json").read_text())
        self.assertEqual(set(retreat), {"coral-A", "coral-B"})
        for pipe in retreat.values():
            self.assertEqual(pipe["0.2"]["stop"], "sustained-paired-floor-and-lip")
            self.assertGreater(pipe["0.2"]["gapMm"], 37.3)
            self.assertGreater(pipe["0.2"]["terminalStillS"], 2.2)
            self.assertLess(pipe["0.5"]["gapMm"], pipe["0.2"]["gapMm"])
