import json
import unittest

from mission import heavy_field_clock as heavy


class CombinedDepartureTest(unittest.TestCase):
    def test_short_and_long_native_material_controls_and_zero_wait(self):
        directory=heavy.ROOT/"mission"
        long=json.loads((directory/"combined-departure-long.json").read_text())
        short=json.loads((directory/"combined-departure-short.json").read_text())
        for a,b in zip(long["departures"],short["departures"],strict=True):
            first=a["early_native_pipe_wall_snapshots"][0]["forward_endpoint_inward_grip_overlap_mm3"]
            self.assertEqual(set(first),{"grip-L-537-3M-15","grip-R-537-3M-15"})
            self.assertGreater(min(first.values()),143)
            self.assertTrue(all(not v["forward_endpoint_inward_grip_overlap_mm3"] for v in b["early_native_pipe_wall_snapshots"]))
            self.assertGreater(b["early_native_pipe_wall_snapshots"][0]["bodies"]["grip-L-420-3M-15"]["gap_mm"],20)
        import hashlib
        timed=json.loads((directory/"short-vision-clock.json").read_text())
        self.assertEqual(timed["pose_receipt_sha256"],hashlib.sha256((directory/"combined-departure-short.json").read_bytes()).hexdigest())
        self.assertEqual(sum(r["sustained_count"] for r in timed["runs"]),434)
        for r in timed["runs"]:
            tight=r["budgets"]["0.1"]["0"]
            self.assertEqual(tight["phases_within_150ms_prior_contact_bracket"],17)
            self.assertTrue(all(e["whole_front_lower_gap_mm"]>=1 for e in tight["exposures"]))
            self.assertEqual(r["budgets"]["0.1"]["50"]["phases_within_150ms_prior_contact_bracket"],0)
            self.assertEqual(r["late_controls"][-1][1],"receiver-recontact")

    def test_revised_clock_preserves_independent_traffic_and_reproduces(self):
        record=heavy.run()
        expected=json.loads((heavy.ROOT/"mission/67kg-field-clock.json").read_text())
        self.assertEqual(record,expected)
        self.assertEqual(record["focal_drive"]["turnWheelAccelMps2"],.64)
        self.assertEqual(record["other_motion"]["turnWheelAccelMps2"],.7)
        reserved=record["scenarios"]["reserved"]
        self.assertGreater(reserved["min_separating_bound_m"],.10)
        self.assertTrue(133<reserved["planned_empty_finish_s"]<134)
        self.assertFalse(record["scenarios"]["late"]["empty_finish_available"])

    def test_integrated_geometry_contains_adverse_and_clear_cases(self):
        native=json.loads((heavy.ROOT/"mission/combined-departure-345.json").read_text())
        self.assertEqual(native["body_count"],345)
        self.assertEqual(native["pose_bounds_mm"]["transport"][-1],1059)
        self.assertEqual(native["clearance_error_allowances_mm"],{"outer_slide_x_mm":15,"grip_independent_y_mm":1.5})
        self.assertFalse(native["automatic_release_at_shelf_witness_qualified"])
        self.assertTrue(all(d["min_tail_gap_mm"]>37 for d in native["departures"]))
        early,clear=native["departures"][0]["checks_next"]
        self.assertFalse(early["clear_all_modeled_poses"])
        collisions=[body["volume_mm3"] for body in early["native_place_witnesses"]]
        self.assertGreater(max(collisions),436)
        self.assertTrue(clear["clear_all_modeled_poses"])
        self.assertGreater(clear["min_swept_bound_gap_mm"],39)
