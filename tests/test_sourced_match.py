import copy
import hashlib
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "sourced-match"
sys.path.insert(0,str(HERE))
import replay_source_match as sourced
import view_data


class SourcedMatch(unittest.TestCase):
    def test_actual_seat_clocks_identity_position_and_velocity(self):
        source=json.loads((HERE/"source-545.json").read_bytes())
        entry=source["identified_transfers"][0]
        field=sourced.corrected()
        route=field.RegisteredMission(-.18)
        state=sourced.checkpoint(source,entry,route)
        with self.assertRaisesRegex(ValueError,"matching attained seat"):
            sourced.mission.resume_from_seat(state,.001,route,sourced.Cartridge(),-.18,"coral-B")
        disturbed=copy.deepcopy(entry)
        disturbed["ideal_seat_check_teleop_s"]+=.020
        with self.assertRaisesRegex(ValueError,"clock discontinuity"):
            sourced.checkpoint(source,disturbed,route)
        disturbed=copy.deepcopy(state)
        disturbed["xyz_m"][1]+=.16
        disturbed["vxyz_mps"][1]+=.12
        continued=sourced.mission.resume_from_seat(disturbed,.001,route,sourced.Cartridge(),-.18,"coral-A")
        self.assertAlmostEqual(.16,continued["trace"][0]["xyz_m"][1])
        self.assertAlmostEqual(.12,continued["trace"][0]["vx_vy_mps"][1])
        self.assertNotEqual("paired-trough-first-contact",continued["outcome"])

    def test_same_piece_qualifications_and_failure_controls(self):
        register=json.loads((HERE/"robot-models.json").read_bytes())
        for model,item in register.items():
            receipt=json.loads((HERE/(model+".json")).read_bytes())
            self.assertEqual(item["poses"],receipt["pose_sha256"])
            self.assertEqual(item["pickup_sha256"],receipt["source_receipt_sha256"])
            self.assertEqual(receipt["source_receipt_sha256"],hashlib.sha256(
                (HERE/"source-545.json" if model=="shop-545" else sourced.SOURCE).read_bytes()).hexdigest())
            for case in receipt["case_studies"]:
                self.assertEqual(case["state"]["piece_id"],case["resume_first_sample"]["piece_id"])
                self.assertEqual(17,sum(e["eligible"] for e in case["images"]["0.1"].values()))
                self.assertTrue(case["native_clearance_samples"])
                for sample in case["native_clearance_samples"]:
                    self.assertFalse(sample["forward_endpoint_inward_grip_overlap_mm3"])
                    self.assertFalse(any(a["overlap_mm3"] for a in sample["bodies"].values()))
                self.assertEqual("receiver-recontact",case["delayed_600ms"]["stop"])
            for edition in receipt["end_assessment"].values():
                self.assertIsNone(edition["officialEarnedPoints"])
                self.assertEqual(["yes","yes"],[p["l1CriteriaAtInstant"] for p in edition["samples"][0]["pieces"]])
            self.assertFalse(receipt["public_hal_authorization"])
            self.assertEqual(["coral-A"],[p["id"] for p in receipt["scenarios"]["second-stock-late"]["support"]])
            self.assertIsNone(receipt["scenarios"]["partner-holds-start"]["model_conditional_point_ceiling"])

    def test_clearance_and_offline_view_follow_the_receipts(self):
        receipt=json.loads((HERE/"carried-stock.json").read_bytes())
        self.assertEqual(24,receipt["stock_count"])
        self.assertLess(receipt["negative_control_m"],0)
        for groups in receipt["nearest_by_group"].values():
            self.assertTrue(all(row["lower_bound_m"]>0 for row in groups.values()))
        self.assertEqual("coral-A",receipt["nearest_by_group"]["coral-B"]["previously-supported-pipe"]["occurrence"])
        timing=json.loads((HERE/"onset-sweep.json").read_bytes())
        for rows in timing["cases"].values():
            self.assertEqual(list(range(62,120)),[r["delay_ms"] for r in rows])
            self.assertTrue(all(r["result"]=="sustained-paired-floor-and-lip" and r["min_shelf_gap_mm"]>41 for r in rows))
        self.assertEqual(view_data.render(),(HERE/"timeline-data.js").read_text())
