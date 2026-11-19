import csv
import hashlib
import json
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
STUDY=ROOT/"moving-observer"

def load(name):
    return json.loads((STUDY/name).read_bytes())

class MovingObserver(unittest.TestCase):
    def test_measurements_and_same_clock(self):
        record=load("replay.json"); hal=load("hal.json")
        for name,digest in hal["inputs"].items():
            self.assertEqual(hashlib.sha256((STUDY/name).read_bytes()).hexdigest(),digest)
        self.assertEqual(record["frame_sha256"],hal["inputs"]["frames.csv"])
        self.assertEqual(record["primary_sha256"],hashlib.sha256((ROOT/"low-observation/results.json").read_bytes()).hexdigest())
        for k,n in (("coral-A",1472),("coral-B",1452)):
            self.assertEqual(record["cases"][k]["counts"],{"inside":n})
            self.assertEqual(record["cases"][k]["maximum_truth_outside_inferred_bounds_mm"],0)
            self.assertEqual(hal["cases"][k+"/normal"]["hold_ticks"],0)
            for mode in ("stale","empty","wrong-epoch","uncalibrated"):
                self.assertGreater(hal["cases"][k+"/"+mode]["hold_ticks"],0)
            hand=load("handoff.json")[k]
            self.assertEqual(sum(r["timely"] for r in hand),11 if k=="coral-A" else 12)
            self.assertTrue(all(r["overlap_joined_ms"] is not None and not r["fixed_track_lost"] for r in hand))
        for k,r in record["counterexamples"].items():
            self.assertEqual(r["status"]=="inside",k=="seated",k)
        self.assertIn("flat-matched-rectangle",record["counterexamples"])

    def test_geometry_and_mass_negative_controls(self):
        model=load("mount.json"); field=load("clearance.json"); account=load("accounting.json")
        self.assertEqual((598,22),(model["combined_count"],model["step_parts"]))
        self.assertEqual([],model["positive_or_unresolved_interference"])
        self.assertEqual([],model["fixed_or_middle_optical_candidates"])
        self.assertGreater(field["rejected_raised_plate"]["native_intersection_mm3"],430)
        for poses in field["issued_start_stock"].values():
            self.assertGreater(min(cat["lower_bound_mm"] for stage in poses.values() for cat in stage.values()),5)
        self.assertGreater(field["station"]["minimum_mm"],330)
        self.assertTrue(0<account["margin_kg"][0]<.012)
        self.assertIsNone(account["configuration_margin_kg"])
