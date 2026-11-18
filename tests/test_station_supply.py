"""Regression checks for the source to receiver exchange and its independent failure cases."""
import hashlib
import json
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
RECEIPT=json.loads((ROOT/"station-supply/transfer.json").read_bytes())


class StationSupplyTest(unittest.TestCase):
    def test_actual_opening_and_crossed_guide_material(self):
        gate=RECEIPT["opening_sections"]
        self.assertAlmostEqual(gate["slot_mm"],121.99055,places=4)
        self.assertLess(gate["outer_hull_yaw_limit_deg"],1.47)
        probes=gate["native_controls_mm3"]
        self.assertEqual(probes["center"],{})
        self.assertGreater(probes["shifted-toward-field"][gate["front"]],40000)
        self.assertEqual(set(probes["yaw-three-degrees"]),{gate["rear"],gate["front"]})
        self.assertEqual(RECEIPT["issued_match_sha256"],hashlib.sha256((ROOT/"full-match/field-trial.json").read_bytes()).hexdigest())

    def test_centered_path_and_real_receiver_counterexamples(self):
        f=RECEIPT["feed"]
        swept=f["continuous_occupied_hulls"]
        for key in ("vertical-drop","vertical-drop-receiving-robot","tangent-roll","tangent-roll-receiving-robot"):
            self.assertEqual(swept[key],{})
        self.assertTrue(all(r["chord_expansion_mm"]<.05 for r in swept["flight"]))
        self.assertTrue(all(not r["robot"] and not r["station"] for r in swept["flight"][1:-1]))
        rebound=f["first_rebound_alternatives"]
        self.assertEqual([row["assumed_normal_restitution"] for row in rebound],[.2,.4,.6])
        self.assertTrue(all("NAUO1490/GE-25015" in row["first_material_witness"]["material_mm3"] for row in rebound))
        rates=RECEIPT["rates"]
        for v in rates["0.5"]["native_arrival_samples"][-1]["material_mm3"].values():
            self.assertGreater(v,50)
        self.assertEqual(len(rates["0.5"]["native_arrival_samples"][-1]["material_mm3"]),3)
        nominal=rates[str(round(f["lip_mps"],6))]
        self.assertEqual(nominal["centered_flat_model"]["result"],"within-modeled-seat-window")
        self.assertEqual(nominal["supported_motion_occupied_enclosure"],{})
        self.assertEqual(rates["2.5"]["centered_flat_model"]["result"],"recover")

    def test_supply_identity_and_non_focal_clearance(self):
        a,b=RECEIPT["identified_transfers"]
        self.assertEqual([a["id"],b["id"]],["coral-A","coral-B"])
        self.assertEqual([a["alliance_stock_remaining_if_taken"],b["alliance_stock_remaining_if_taken"]],[59,58])
        self.assertGreater(b["human_release_teleop_s"],b["available_teleop_s"])
        for p in (a,b):
            self.assertLess(p["human_release_teleop_s"],p["ramp_lip_teleop_s"])
            self.assertLess(p["ramp_lip_teleop_s"],p["ideal_seat_check_teleop_s"])
            self.assertGreater(p["five_other_programs"]["lower_bound_m"],2.58)
        self.assertEqual(RECEIPT["whole_match_using_new_source"],"pending")
