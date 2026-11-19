import hashlib
import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CASE = ROOT / "camera-disconnects"


def get(name):
    return json.loads((CASE / name).read_bytes())


def sha(file):
    return hashlib.sha256(file.read_bytes()).hexdigest()


class CameraDisconnectTest(unittest.TestCase):
    def test_registered_mating_and_failed_factory_case(self):
        r = get("interfaces.json")
        self.assertEqual(r["retained_603_kit"], sha(ROOT / "camera-datum-shop/camera-shelf.step"))
        self.assertAlmostEqual(r["camera_board_mouth_x_mm"], 32.16, places=2)
        self.assertEqual(r["adapter_depths"]["6.2"]["overlap_sample_count"], 86)
        self.assertEqual(r["adapter_depths"]["5.0"]["overlap_sample_count"], 0)
        self.assertGreater(r["adapter_depths"]["6.4"]["relieved_case_plane_gap_mm"], .39)
        self.assertFalse(r["electrical_mating_measured"])
        self.assertAlmostEqual(r["disconnects"]["power-J3"]["mouth_mm"][1] -
                               r["disconnects"]["data-J1"]["mouth_mm"][1], 16, places=4)

    def test_host_travel_cable_curvature_and_mating_service(self):
        source = get("route-proof.json")
        native = json.loads((ROOT / "camera-datum-shop/native-place.json").read_bytes())
        self.assertEqual(source["host_sha256"], native["step_sha256"])
        self.assertGreater(source["data"]["min_radius_certified_mm"], 75.6)
        self.assertGreater(source["power"]["min_radius_certified_mm"], 64.5)
        self.assertGreater(source["data"]["minimum_host_clearance_mm"], 1.5)
        self.assertGreater(source["power"]["minimum_host_clearance_mm"], 2.6)
        self.assertGreater(source["cable_to_cable_clearance_lower_mm"], .94)
        self.assertFalse(source["data_at_1970mm_fits"])
        self.assertTrue(source["data_at_2030mm_fits"])
        self.assertLess(source["data_tip_to_tip_mm"][1], 2017.73)
        for result in get("support-routes.json").values():
            self.assertEqual(result["penetrations_mm"], [])
            self.assertEqual(result["uncertain_intervals"], [])
        service = get("service.json")
        self.assertEqual(service["support_sha256"], sha(CASE / "service-support.step"))
        self.assertGreater(service["rejected_first_support_mm3"]["data/first-rest-support"], 1.20)
        self.assertEqual(service["both_plug_release_material_conflicts_mm3"], {})
        self.assertEqual(service["full_stroke_native_kit_intersections"], {})
        self.assertEqual(service["full_stroke_unresolved_host_boxes"], [])
        self.assertEqual(service["released_support_535mm_lift_unresolved"], [])
        self.assertEqual(service["transition_board_lift_conflicts"], {})
        self.assertTrue(all(not v for v in service["cap_drivers"].values()))
        self.assertTrue(service["lift_requires_both_leads_unplugged_and_parked"])
        for sector in get("target-stock.json").values():
            self.assertEqual(sector["combined_lower_mm"], sector["retained_lower_mm"])

    def test_independent_voltage_limits_and_unknown_physical_evidence(self):
        p = get("power/corners.json")
        self.assertEqual(p["plugin_version"], "0.22.0")
        self.assertFalse(p["physically_measured"])
        self.assertFalse(p["functional_dropout_observed"])
        for k, row in p["conditions"].items():
            if k=="selected-screen":
                self.assertGreater(row["loaded_min_V"], 5.017)
                self.assertEqual(row["no_load_max_V"], 5.21)
                continue
            compare = get(f"power/compare-{k}.json")
            for trial in compare["trials"]:
                self.assertEqual(trial["trace"]["sha256"], sha(CASE / "power" / trial["trace"]["file"]))
            review = get(f"power/review-{k}.json")
            self.assertEqual(review["trials"][-1]["status"], "complete")
            self.assertEqual(row["within_both_limits"], k=="low-resistance-only")
        self.assertGreater(p["conditions"]["overtrim-counterexample"]["no_load_max_V"], 5.25)

    def test_assembly_accounting_keeps_one_complete_loom_and_carrier(self):
        summary = get("accounting.json")
        self.assertEqual(summary["source_sha256"], sha(CASE / "mass-conditions.json"))
        self.assertEqual(summary["native_support_count"], 25)
        self.assertTrue(all(not missing for missing in summary["coverage"].values()))
        self.assertAlmostEqual(summary["represented_margin_kg"][0], .0850773, places=5)
        self.assertIsNone(summary["physical_margin_kg"])
        self.assertAlmostEqual(get("mass-conditions.json")["removed_prior_ranges_kg"][1], .165)


if __name__=="__main__":
    unittest.main()
