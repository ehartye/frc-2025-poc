import hashlib
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
AREA = ROOT / "rack-sensing"


class RackSensingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result = json.loads((AREA / "results.json").read_bytes())
        cls.cases = cls.result["cases"]
        cls.qualified = json.loads((AREA / "qualification.json").read_bytes())

    def test_stationary_fault_and_issued_source(self):
        reference = self.cases["legacy-slip94"]["coral-A"]
        self.assertEqual(reference["maxRackTravelMm"], 0)
        self.assertEqual(reference["motion"] - reference["withdraw"], 5)
        for mode in ("slip94", "ramp-slip94", "ramp-phase-slip94", "ramp-open94", "ramp-drag-adverse94"):
            self.assertNotIn("motion", self.cases[mode]["coral-A"])
            self.assertIn("hold", self.cases[mode]["coral-A"])
            self.assertEqual(self.cases[mode]["coral-B"], {})
        for name in ("RackSignalTrial.java", "CalibratedRack.java", "RackCalibrationCheck.java"):
            self.assertEqual(self.result["hal"]["programs"][name], hashlib.sha256((AREA / name).read_bytes()).hexdigest())
        self.assertEqual(self.result["sourceShop"], hashlib.sha256((ROOT / "selected-drive/rack.json").read_bytes()).hexdigest())

    def test_calibration_and_stroke_on_both_trips(self):
        for mode in ("ramp90", "ramp94", "ramp98", "ramp-reverse94", "ramp-adverse90",
                     "ramp-adverse94", "ramp-adverse98", "ramp-phase-adverse90", "ramp-phase-adverse98"):
            for piece, case in self.cases[mode].items():
                self.assertLessEqual(case["motion"] - case["withdraw"], 40)
                self.assertEqual(case["stow"], self.cases["nominal"][piece]["stow"])
                self.assertEqual(case["traceSha256"], self.result["hal"]["rackPaths"][f"path-{mode}-{piece}.csv"])
        for mode in ("ramp-unscaled90", "ramp-unscaled98"):
            self.assertIn("hold", self.cases[mode]["coral-A"])
            self.assertNotIn("stow", self.cases[mode]["coral-A"])

    def test_load_and_contact_cover_each_completed_revised_case(self):
        accepted = {m for m, pieces in self.cases.items() if m.startswith("ramp") and all("stow" in p for p in pieces.values())}
        self.assertEqual(set(self.qualified["cases"]), accepted)
        for case in self.qualified["cases"].values():
            contacts = case["contacts"]
            self.assertEqual({r["case"] for r in contacts}, {f"{piece}/{step}/observed" for piece in ("coral-A","coral-B") for step in (250,500)})
            self.assertTrue(all(r["outcome"] == "supported" for r in contacts))
            self.assertIsNone(case["power"]["0.027"]["firstReferenceInhibitWallS"])
            self.assertIsNotNone(case["power"]["0.032"]["firstReferenceInhibitWallS"])
