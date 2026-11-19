import hashlib
import json
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CASE = ROOT / "bridge-tolerance"


def read(name):
    return json.loads((CASE / name).read_bytes())


class BridgeToleranceTest(unittest.TestCase):
    def test_registered_geometry_and_failed_retrofit(self):
        manifest = read("register.json")
        for key, root in (("source_sha256", ROOT), ("candidate_sha256", CASE)):
            for path, expected in manifest[key].items():
                self.assertEqual(hashlib.sha256((root / path).read_bytes()).hexdigest(), expected, path)
        self.assertEqual(manifest["unchanged_article_count"], 21)
        self.assertAlmostEqual(manifest["neighbor_bore_common_mm3_per_pair"], 52.2700101)
        self.assertLess(manifest["largest_native_translation_error_mm"], .001)
        self.assertEqual(read("mount.json")["positive_or_unresolved_interference"], [])

    def test_real_collision_and_continuous_successor_bounds(self):
        result = read("field.json")
        self.assertEqual(result["collision_bodies"], 593)
        for sector, expected in (("-0.18", 16.48), ("0.22", 24.89)):
            row = result["sectors"][sector]
            self.assertGreater(row["issued"]["native_25_mm_forward_control"]["intersection_mm3"], 2000)
            self.assertEqual(row["candidate"]["native_25_mm_forward_control"]["intersection_mm3"], 0)
            self.assertAlmostEqual(row["whole_target_stock"]["reef-ball"]["after_translation_and_camera_allowance_mm"], expected, places=2)
            self.assertGreater(row["reef_structure"]["after_translation_mount_flex_mm"], 63)
        self.assertLess(result["sectors"]["-0.18"]["candidate"]["translation_35_mm"], 0)
        self.assertGreater(result["sectors"]["0.22"]["settled_first_pipe"]["lower_mm"], 16)

    def test_image_registration_and_fresh_packet_handoff(self):
        result = read("calibration.json")
        self.assertEqual(result["fit_controls"]["forward"], {"issued_decoder":"unbacked-crown","aft_decoder":"inside"})
        self.assertNotEqual(result["fit_controls"]["uncalibrated-low-contrast"]["aft_decoder"], "inside")
        qualified = [r for c in result["cases"][:-1] for r in c["results"]]
        self.assertEqual(len(qualified), 48)
        self.assertTrue(all(r["status"] == "inside" and r["truth_outside_mm"] == 0 for r in qualified))
        self.assertTrue(all(r["status"] == "robot-reference-lost" for r in result["cases"][-1]["results"]))
        timing = read("timing.json")
        self.assertEqual(timing["counts"], {"HAL":1428,"same-consumer":59364})
        self.assertTrue(read("register.json")["hal_all_phase_ticks_and_results_identical"])
        for piece, count in (("coral-A",1472),("coral-B",1452)):
            observed = read("replay.json")["cases"][piece]
            self.assertEqual(observed["counts"], {"inside":count})
            self.assertEqual(observed["maximum_truth_outside_inferred_bounds_mm"], 0)
            self.assertEqual(read("travel-hal.json")["cases"][piece+"/normal"]["hold_ticks"],0)
            self.assertEqual(timing["profiles"]["5ms/constant/"+piece]["18"], [85,85])
            self.assertEqual(timing["profiles"]["5ms/constant/"+piece]["24"], [0,85])
