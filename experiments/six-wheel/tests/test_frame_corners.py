import hashlib
import importlib.util
import json
from tempfile import TemporaryDirectory
import unittest
from pathlib import Path


STUDY = Path(__file__).resolve().parents[1]/"frame-corners"


def read(name):
    return json.loads((STUDY/name).read_bytes())


class FrameCornersTest(unittest.TestCase):
    def test_registered_mirror_corners_and_bumper_control(self):
        fit = read("weld-fit.json")
        self.assertEqual(hashlib.sha256((STUDY/"frame-weldment.step").read_bytes()).hexdigest(),fit["stepSha256"])
        self.assertEqual(fit["reopenedBodies"],20)
        self.assertEqual(set(fit["corners"]),{"front-L","front-R","rear-L","rear-R"})
        for tag,corner in fit["corners"].items():
            self.assertGreater(corner["minimumWheelClearanceLowerMm"],19.66)
            self.assertGreater(corner["movingBoxLowerMmAndBody"][0],196)
            self.assertEqual(corner["bothBumperRemovalSweepsMaxOverlapMm3"],0)
            self.assertEqual(corner["maxOtherInterferenceMm3"],0)
            self.assertAlmostEqual(corner["proudFaceControlMm3"]["bumper-contact-"+tag.split("-")[0]+"-end"],48)

    def test_group_sensitivity_and_combined_budget(self):
        spec = importlib.util.spec_from_file_location("frame_loads",STUDY/"loads.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        for case in read("load-demand.json"):
            peak = module.screen(case["forceN"],case["assumedEffectiveThroatMm"])["worst"]["equivalentMPa"]
            self.assertAlmostEqual(peak,case["worst"]["equivalentMPa"])
            self.assertAlmostEqual(peak*case["assumedEffectiveThroatMm"]/case["forceN"],.078450,places=5)
        fit = read("weld-fit.json")
        self.assertAlmostEqual(fit["nominalNetKg"],(fit["grossFillMm3"]-fit["removedMm3"])*2.70e-6)
        mass = read("weld-mass.json")
        self.assertEqual(mass["cornerStudySha256"],hashlib.sha256((STUDY/"weld-fit.json").read_bytes()).hexdigest())
        self.assertLess(mass["nominalNetKg"],mass["estimatedNetRangeKg"][1])
        account = read("accounting.json")
        camera = json.loads((STUDY.parent/"camera-disconnects/accounting.json").read_bytes())
        self.assertAlmostEqual(account["representedMarginKg"][0],
                               camera["represented_margin_kg"][0]-mass["estimatedNetRangeKg"][1])
        self.assertIsNone(account["physicalMarginKg"])
        for control in read("plate-control.json")["corners"].values():
            self.assertGreater(control["outerAdjustmentWheelOverlapMm3"],2.8)
