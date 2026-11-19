import hashlib
import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STUDY = json.loads((ROOT/"leave-park/study.json").read_bytes())


class LeavePark(unittest.TestCase):
    def test_issued_body_and_actual_bumper_material(self):
        step = ROOT/"design/competition-package/shop-duplex/red/stowed.step"
        self.assertEqual(hashlib.sha256(step.read_bytes()).hexdigest(),STUDY["stowed_step_sha256"])
        self.assertEqual(STUDY["body_count"],545)
        witness=STUDY["assessment"]
        self.assertGreater(sum(witness["start_tape_mm3"].values()),0)
        self.assertEqual(sum(witness["end_auto_tape_mm3"].values()),0)
        self.assertGreater(min(witness["park_zone_mm3"].values()),0)
        self.assertEqual(sum(witness["hold_zone_mm3"].values()),0)

    def test_reservation_changes_usable_score(self):
        cases=STUDY["scenarios"]
        self.assertEqual((cases["reserved"]["leave_points"],cases["reserved"]["park_points"]),(3,2))
        for name in ("occupied","late"):
            self.assertTrue(cases[name]["route_usable"])
            self.assertFalse(cases[name]["attempt"])
            self.assertEqual(cases[name]["park_points"],0)
        self.assertGreater(cases["late"]["decision_teleop_s"],STUDY["latest_park_start_teleop_s"])
        self.assertFalse(cases["ignore-reservation"]["route_usable"])
        self.assertEqual(set(cases["ignore-reservation"]["first_sampled_overlap"][:2]),{"red1","red2"})

    def test_tracking_error_can_exhaust_the_same_field_gap(self):
        cases=STUDY["tracking_cases"]
        self.assertGreater(cases["10"]["field_margin_m"],0)
        self.assertLess(cases["20"]["field_margin_m"],0)
        self.assertGreater(cases["20"]["park_cover_in_eroded_zone_mm3"],0)
        self.assertEqual(STUDY["shift_30mm_toward_cage"]["gap_m"],0)

    def test_start_assignment_eliminates_turn_demand(self):
        direct=STUDY["preassigned_straight"]
        self.assertEqual(direct["potential_points"],5)
        self.assertGreater(direct["start_overlap_if_partner_keeps_south_slot_m"],0)
        self.assertTrue(direct["drive"]["parameter_cases_clear"])
        self.assertFalse(direct["middle_slot_drive"]["parameter_cases_clear"])
        self.assertGreater(direct["minimum_centerline_clearance_m"],0)
        self.assertLess(direct["drive"]["friction_required"],.27)
        self.assertGreater(direct["middle_slot_drive"]["friction_required"],1.3)
        tight,loose=direct["wheel_error_cases"]
        self.assertGreater(tight["remaining_field_margin_m"],0)
        self.assertLess(loose["remaining_field_margin_m"],0)
        self.assertGreater(tight["remaining_end_auto_tape_clearance_m"],0)
        self.assertGreater(tight["start_cover_in_eroded_tape_mm3"],0)
        self.assertGreater(tight["end_cover_in_eroded_zone_mm3"],0)
