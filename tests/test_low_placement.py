"""Keep supported reduced-model states and validated installed scores distinct."""

import json
from pathlib import Path
import unittest

STUDY = Path(__file__).resolve().parents[1] / "experiments/swerve-drive/low-placement"


class LowPlacementTests(unittest.TestCase):
    def test_issued_hardware_vetoes_each_apparently_supported_delivery(self):
        data = json.loads((STUDY / "results.json").read_bytes())
        for case in data["cases"].values():
            result = next(r for r in case["coupledTrough"] if r["control"] == "nominal")
            self.assertEqual(result["sourceState"], case["cargo"]["state"])
            self.assertGreaterEqual(result["longestSeparatedSupportS"], 0.25)
            for landing in result["fieldAssessment"]["attained"]:
                self.assertEqual(landing["outcome"], "supported")
            check = result["installedShapeChecks"]
            self.assertEqual(check["trace"], result["trace"])
            self.assertEqual(check["bodies"], 1268)
            foot = check["positiveUndeformedMaterialMm3"]["grip-R-front-support-foot"]
            self.assertLess(foot["firstS"], result["firstTargetContactS"])
            self.assertGreater(foot["peakMm3"], 800)
            self.assertEqual(result["matchCredit"], 0)

    def test_sensor_deadband_and_travel_challenge(self):
        data = json.loads((STUDY / "clearance.json").read_bytes())
        for case in data["cases"].values():
            trials = {
                (r["commandMm"], r["actualPlusMm"]): r["minimumPlaneGapMm"]
                for r in case["trials"]
            }
            self.assertLess(trials[345, 2], 0)
            self.assertGreater(trials[337, 2.5], 2.7)
            self.assertLess(trials[337, 8], 0)

    def test_force_implementations_reach_both_registered_surfaces(self):
        witness = json.loads((STUDY / "force-witness.json").read_bytes())
        self.assertEqual(set(witness["inputs"]), {"plus", "minus"})
        self.assertGreater(min(witness["lipSamples"], witness["floorSamples"]), 0)
        self.assertLess(
            max(witness["maxTorqueErrorNm"] + witness["maxForceErrorN"]), 1e-7
        )
