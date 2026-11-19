import hashlib
import json
from pathlib import Path
import unittest

HERE=Path(__file__).resolve().parent

def read(name):
    return json.loads((HERE/name).read_bytes())

class SelectedDriveTests(unittest.TestCase):
    def test_source_and_native_contact(self):
        d=read("results.json")
        for n,val in d["sources"].items():
            self.assertEqual(hashlib.sha256((HERE.parent/n).read_bytes()).hexdigest(),val)
        self.assertGreater(d["emptyFieldKg"][1],65)
        self.assertGreater(d["jointedHighFieldKg"],d["emptyFieldKg"][1])
        self.assertEqual([w["outerRadiusMm"] for w in read("wheels.json")["wheels"].values()],[76.2]*6)
        dock=read("dock.json")["cases"]
        self.assertAlmostEqual(dock["nominal"]["endpoint"]["minimum_native_mm"],10)
        for n in ("bias","track"):
            sample=dock[n]["firstSampleWithPositiveVolume"]
            self.assertGreater(min(sample["intersections_mm3"].values()),8000)
            self.assertLess(sample["wallS"],13.492)

    def test_supply_feedback_and_fallback(self):
        data=read("results.json")["cases"]
        c=data["nominal"]["cases"]
        for mode in ("nominal","bridge26","single","near","nearSlow"):
            self.assertIsNone(data[mode]["pointsAwarded"])
        self.assertGreater(c["screened"]["minTerminalV"],9.5)
        self.assertLess(c["weak"]["minTerminalV"],9.5)
        self.assertLess(c["poorBalance"]["firstReferenceInhibitWallS"],1)
        self.assertGreater(c["poorBalance"]["peakMotorPhaseA"],38)
        self.assertLess(c["continuousMaxGrip"]["minTerminalV"],9.5)
        self.assertLess(c["attachedScreened"]["minTerminalV"],c["screened"]["minTerminalV"])
        self.assertLess(c["attachedPlus50g"]["minTerminalV"],c["attachedScreened"]["minTerminalV"])
        self.assertGreater(data["nearSlow"]["cases"]["depleted"]["minTerminalV"],10.7)
        drift=data["nominal"]["kinematicSensitivity"]
        self.assertGreater(drift["leftSmallRightLarge"]["openingMaxHullDisplacementMm"],100)
        self.assertLess(drift["calibrated"]["openingMaxHullDisplacementMm"],.001)
        self.assertGreater(data["nominal"]["worstFullPathHullDriftAt1ppmMm"],data["nominal"]["motionStockSourceMarginMm"])
        self.assertEqual([v["status"] for v in read("power/review.json")["trials"]],["complete","incomplete"])

    def test_shop_inertia_and_drag_are_independent(self):
        states=read("rack.json")["cases"]
        for name in ("nominal","bridge26","extra50g"):
            for pipe in ("coral-A","coral-B"):
                self.assertEqual(states[name][pipe]["stow"],states["base"][pipe]["stow"])
                self.assertNotEqual(states[name][pipe]["rawSHA256"],states["base"][pipe]["rawSHA256"])
        self.assertEqual(states["heavyDrag"]["coral-A"]["hold"],48335)
        self.assertEqual(states["heavyDrag"]["coral-B"],{})
        for trial in read("results.json")["reference597SurfaceWithShopLoadedRack"].values():
            self.assertEqual(len(trial),4)
            self.assertTrue(all(v["outcome"]=="supported" for v in trial))
