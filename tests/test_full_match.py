"""Regression coverage for source-bound full-match evidence and counterexamples."""
import hashlib
import json
from pathlib import Path
import unittest

HERE=Path(__file__).resolve().parents[1]/"full-match"


def read(name):
    return json.loads((HERE/name).read_text(encoding="utf8"))


class FullMatchTest(unittest.TestCase):
    def test_staged_objects_and_endgame_alternatives(self):
        trial=read("field-trial.json")
        self.assertEqual(len(trial["field_floor_stacks_centers_in"]),6)
        runs=trial["hold_variants"]
        self.assertEqual(runs["0"]["rules"]["transition_disabled_s"],3)
        self.assertEqual([r["id"] for r in runs["0"]["qualified_supports"]],["coral-A","coral-B"])
        self.assertLess(runs["0"]["rules"]["teleop_motion_completion_s"],94)
        self.assertIsNone(runs["partner-holds-start"]["potential_points_after_other_acceptance"])
        self.assertEqual(runs["return-to-station"]["rules"]["red_barge_park_overlap_m"],0)
        self.assertEqual(len(runs["second-inventory-absent"]["qualified_supports"]),1)
        negative=read("stack-negative.json")
        self.assertEqual(len(negative["hits"]),4)
        self.assertGreater(min(h["volumeMm3"] for h in negative["hits"]),220)

    def test_qualified_onsets_and_end_of_assessment(self):
        extended=read("observer-extent.json")
        for name,field in (("red-assembly.json","assembly_sha256"),("field-trial.json","field_trial_sha256"),
                           ("red-withdrawal-clock.json","clock_sha256")):
            self.assertEqual(hashlib.sha256((HERE/name).read_bytes()).hexdigest(),extended[field])
        source=read("red-withdrawal-clock.json")
        for row,window in zip(source["runs"],extended["runs"],strict=True):
            eligible=[p for p in row["budgets"]["0.25"]["0"]["exposures"] if p["within_150ms"]]
            self.assertEqual(window["eligible_phases"],[p["source_phase_mod17"] for p in eligible])
            lo,hi=window["modeled_onset_interval_after_shelf_ms"]
            self.assertTrue(all(lo<=p["onset_after_shelf_ms"][0]<=p["onset_after_shelf_ms"][1]<=hi for p in eligible))
            self.assertLess(window["last_possible_contact_after_shelf_ms"],lo)
            self.assertGreater(window["min_tail_separation_mm"],34)
            if window["shelf_coast_mu"]==.1:
                state=window["static_target_retention"]
                self.assertEqual(state["through_teleop_s"],138)
                self.assertGreater(state["still_paired_contact_s"],57)

    def test_authoritative_floor_inventory_and_model_identity(self):
        rows=read("plugin-review-017.json")
        self.assertEqual(rows["revised"]["status"],"supported")
        self.assertEqual(rows["revised"]["whole_field"],"pending")
        self.assertEqual(rows["prior"]["native_hits"],4)
        self.assertEqual(rows["underdeclared"]["status"],"pending")
        self.assertEqual(len(rows["underdeclared"]["inventory_gap_reasons"]),2)
        for name,result in read("kit-comparison.json").items():
            self.assertEqual(result["identical_native_serializations"],401)
            self.assertEqual(read("red-kit/manifest.json")[name]["red_sha256"],result["red_step_sha256"])
