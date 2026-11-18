import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "design/receiver-lift"
spec = importlib.util.spec_from_file_location("size_lift", HERE / "size_lift.py")
sizing = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sizing)


class LiftDesign(unittest.TestCase):
    def test_bom_and_mission_reproduce_sizing(self):
        self.assertEqual(sizing.run(), json.loads((HERE/"sizing.json").read_text()))

    def test_full_speed_and_clock_counterexample(self):
        results = sizing.run()
        for name, bounds in results["winch"].items():
            for row in bounds:
                self.assertEqual(row["within_9p5V_35A_envelope"], name!="rejected_one_second_return")
        self.assertGreater(results["single_layer"]["margin_mm"], 0)
        self.assertFalse(results["holding"]["power_loss_during_descent_verified"])

    def test_recorded_geometry_covers_original_pose_files(self):
        import hashlib
        receipt = json.loads((HERE/"routing.json").read_text())
        self.assertEqual(set(receipt), {"stowed","receive","transport","place"})
        for pose, row in receipt.items():
            source = HERE.parent / "station-l1" / (pose+".step")
            self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), row["base_step_sha256"])
            self.assertEqual(row["added_routing_bodies"], 34)
            self.assertFalse(row["external_interferences_mm3"])
            self.assertFalse(row["internal_interferences_mm3"])
        self.assertFalse(receipt["stowed"]["service_interferences_mm3"])
