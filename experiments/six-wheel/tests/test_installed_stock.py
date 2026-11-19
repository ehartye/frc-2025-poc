import hashlib
import json
from pathlib import Path
import unittest


HERE = Path(__file__).resolve().parents[1] / "full-match"


class InstalledStock(unittest.TestCase):
    def test_all_occurrences_and_all_focal_stages(self):
        evidence = json.loads((HERE/"installed-stock.json").read_text())
        self.assertEqual(evidence["field_trial_sha256"], hashlib.sha256((HERE/"field-trial.json").read_bytes()).hexdigest())
        self.assertEqual(evidence["body_count"], 401)
        self.assertEqual(evidence["robot_pose_step_sha256"], json.loads((HERE/"red-assembly.json").read_text())["pose_step_sha256"])
        stock = list(evidence["stock"].values())
        self.assertEqual([sum(o["group"] == g for o in stock) for g in ("reef-ball","floor-ball","floor-pipe")], [12,6,6])
        self.assertEqual(len(evidence["stages"]), 13)
        self.assertTrue(all(r["lower_bound_mm"] > 0 for stage in evidence["stages"].values() for r in stage.values()))
        for key in ("A","B"):
            row = evidence["stages"]["target-extension-coral-"+key]["reef-ball"]
            self.assertEqual(row["occurrence"], "NAUO201")
            self.assertGreater(row["lower_bound_mm"], 150)

    def test_contact_from_alternative_layout(self):
        earlier = json.loads((HERE/"installed-stock.json").read_text())["earlier_route_native_check"]
        self.assertEqual(earlier["hits"], [])
        self.assertGreater(earlier["minimum_nominal_native_gap_mm"], 67)
        self.assertGreater(earlier["displaced_pipe_control"]["foam_overlap_mm3"], 100000)
        before, after = [json.loads((HERE/("field-stock-plugin-"+v+".json")).read_text()) for v in ("017","018")]
        self.assertEqual(before["route"], "infeasible")
        self.assertEqual(after["declared"], "infeasible")
        self.assertEqual(after["route"], "pending")
        self.assertEqual(len(after["reported_contacts"]), 4)
        self.assertTrue(all(row["registrationIssue"] for row in after["reported_contacts"]))
