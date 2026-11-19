import hashlib
import json
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
HERE=ROOT/"leave-park"
FIELD=json.loads((HERE/"native-park.json").read_bytes())
REVIEW=json.loads((HERE/"body-route-review.json").read_bytes())


class NativeParking(unittest.TestCase):
    def test_original_and_short_stops_keep_bumper_and_route_identity(self):
        path=ROOT/"design/competition-package/shop-duplex/red/stowed.step"
        self.assertEqual(FIELD["robot_step_sha256"],hashlib.sha256(path.read_bytes()).hexdigest())
        a,b=(FIELD["cases"][key] for key in ("issued","shorter"))
        self.assertGreater(b["static_field_lower_bound_mm"],a["static_field_lower_bound_mm"]+80)
        self.assertLess(b["drive_s"],a["drive_s"])
        self.assertGreater(b["tape_clearance_mm"],100)
        self.assertGreater(sum(b["start_cover_mm3"].values()),0)
        self.assertEqual(sum(b["end_tape_cover_mm3"].values()),0)
        self.assertGreater(b["end_park_cover_mm3"]["bumper-cover-front-red"],0)
        self.assertEqual(b["end_park_cover_mm3"]["bumper-cover-rear-red"],0)
        self.assertTrue(b["teleop_clear"])
        self.assertGreater(b["one_percent_wheel_case"]["remaining_field_margin_m"],0)

    def test_held_partner_has_static_clearance_but_remains_inside_cage_exclusion(self):
        shared=FIELD["cases"]["shared-105"]
        self.assertTrue(shared["red1_keeps_south"])
        self.assertGreater(shared["south_partner_gap_mm"],160)
        self.assertGreater(shared["static_field_lower_bound_mm"],100)
        self.assertEqual(shared["planning_field"]["gap_m"],0)
        self.assertGreater(shared["teleop_pair_bound_m"],.16)
        self.assertEqual(sum(shared["initial_overlap_against_south_partner_mm3"].values()),0)

    def test_same_observed_bumper_overlap_on_valid_and_invalid_starts(self):
        rows=REVIEW["cases"]
        self.assertEqual(rows["issued"]["volumeInZoneMm3"],rows["occupied-start"]["volumeInZoneMm3"])
        self.assertEqual(rows["occupied-start"]["routeStatus"],"infeasible")
        self.assertTrue(all(c["timeS"]==0 and c["volumeMm3"]>0 for c in rows["occupied-start"]["contacts"]))
        self.assertEqual(len(rows["occupied-start"]["contacts"]),2)
        for key in ("issued","shorter","retained-partner"):
            self.assertEqual(rows[key]["routeStatus"],"supported")
            self.assertEqual(rows[key]["wholeFieldStatus"],"pending")
