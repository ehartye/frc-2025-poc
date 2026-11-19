import csv
import hashlib
import importlib.util
import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CASE = ROOT / "camera-datum-shop"


def record(filename):
    return json.loads((CASE / filename).read_bytes())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class CameraShopTest(unittest.TestCase):
    def test_released_step_matches_shop_operations_and_complete_robot(self):
        model = record("native.json")
        proof = record("shelf-proof.json")
        with (CASE / "shelf-operations.csv").open(encoding="utf8", newline="") as stream:
            operations = list(csv.DictReader(stream))
        self.assertEqual(proof["step_sha256"], digest(CASE / "camera-shelf.step"))
        self.assertEqual(model["step_sha256"], proof["step_sha256"])
        self.assertEqual(proof["operations_sha256"], digest(CASE / "shelf-operations.csv"))
        self.assertEqual(len(proof["finished_native_faces"]), 24)
        for op, native in zip(operations, proof["finished_native_faces"], strict=True):
            self.assertEqual((op["operation"], float(op["U_mm"]), float(op["V_mm"]), float(op["diameter_mm"])),
                             (native["name"], native["U_mm"], native["V_mm"], native["diameter_mm"]))
        self.assertEqual(proof["forward_radius_mm"], 6)
        self.assertEqual(model["reopened_named_solids"], 32)
        self.assertEqual(model["positive_interference"], [])
        place = record("native-place.json")
        self.assertEqual(place["shop_kit_sha256"], model["step_sha256"])
        self.assertEqual((place["independent_root_solids"], place["reopened_named_solids"]), (603, 603))
        self.assertLess(place["max_volume_error_mm3"], .005)
        self.assertLess(place["max_bounds_error_mm"], .001)

    def test_extraction_and_captivity_cover_old_interferences(self):
        result = record("fit.json")
        self.assertEqual((result["retained_source_bodies"], result["registered_sweep_pairs"]), (571, 17701))
        self.assertEqual(result["unresolved_different_frame"], [])
        self.assertEqual(result["positive_solid_intersections"], [])
        self.assertEqual(result["vertical_removal_unresolved"], [])
        self.assertEqual(result["vertical_removal_mm"], 535)
        self.assertGreater(min(result["issued_foot_dashpot_overlap_at_25mm_mm3"].values()), 35)
        self.assertGreater(min(pair["lower_mm"] for pair in result["refined_vertical_removal"]), .18)
        self.assertEqual(len(result["capture"]), 6)
        for name, screw in result["capture"].items():
            self.assertAlmostEqual(result["tool_access"][name]["gap_mm"], .2)
            self.assertGreater(screw["usable_pin_gap_mm"], .6)
            self.assertGreater(screw["extraction_counterexample_volume_mm3"], 14)

    def test_bend_mass_and_low_preload_counterexample(self):
        m = record("mechanics.json")
        self.assertLess(m["derived_2g"]["summed_rotation_deg"], .12583)
        self.assertGreater(m["prior_1mm_section_under_new_upper_load"]["summed_rotation_deg"], .12583)
        self.assertGreater(m["joint_demand"]["foot_heel_uplift_n_per_fastener"], 33)
        self.assertLess(m["joint_demand"]["trial_preloads"][0]["residual_n"], 0)
        self.assertFalse(m["weighed_robot"])
        mass = record("accounting.json")["mass"]
        self.assertAlmostEqual(mass["representedMarginKg"][0], 1.5772132, places=6)
        self.assertIsNone(mass["configurationMarginKg"])

    def test_observability_and_independent_metrology_controls(self):
        result = record("optical.json")
        self.assertEqual(result["camera_step_sha256"], digest(CASE / "camera-shelf.step"))
        self.assertEqual(sum(r["exposures"] for r in result["nominal_moving"].values()), 2924)
        for r in result["nominal_moving"].values():
            self.assertEqual((r["misses"], r["maximum_truth_outside_mm"]), ({}, 0))
        bad = "misregistered-5mm-2deg"
        self.assertTrue(all(row["status"] == "robot-reference-lost" for row in result["registration_cases"][bad]))
        bounded = [row for k, rows in result["registration_cases"].items() if k != bad for row in rows]
        self.assertEqual(len(bounded), 48)
        self.assertTrue(all(r["status"] == "inside" and r["truth_outside_mm"] == 0 for r in bounded))
        self.assertEqual(result["fixed_view"]["exposure_count"], 550)
        spec = importlib.util.spec_from_file_location("datum_metrology", CASE / "metrology.py")
        sensor = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(sensor)
        sample = record("metrology-example.json")
        fit = sensor.evaluate(sample)
        self.assertTrue(fit["within_project_pose_screen"])
        self.assertFalse(fit["physical_measurement_eligible"])
        self.assertFalse(fit["authorizes_motion"])
        sample["measurement_kind"] = "measured"
        with self.assertRaisesRegex(ValueError, "missing acquisition evidence"):
            sensor.evaluate(sample)
        for k, trial in record("metrology.json").items():
            self.assertEqual(trial["within_project_pose_screen"], k == "source_ideal")


if __name__ == "__main__":
    unittest.main()
