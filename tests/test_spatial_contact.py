"""Cross-check signed source state, non-ideal timing, scoring and the second-station hold."""
import csv
import hashlib
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT/"spatial-contact"


def read(name):
    return json.loads((STUDY/name).read_bytes())


class SpatialContactTest(unittest.TestCase):
    def test_gate_and_physics_share_identified_motion(self):
        src=read("source.json")
        self.assertEqual(len(src),10)
        self.assertEqual(len({row["poseSHA256"] for row in src}),10)
        lateral=[s["velocityMps"][1] for s in src]
        self.assertLess(min(lateral),-.00015)
        self.assertGreater(max(lateral),.00015)
        self.assertEqual(sum(abs(v)>1e-5 for v in lateral),9)
        physics=read("physics.json")
        self.assertEqual(physics["attainedSHA256"],hashlib.sha256((STUDY/"source.json").read_bytes()).hexdigest())
        self.assertEqual(physics["clockSHA256"],hashlib.sha256((STUDY/"events.csv").read_bytes()).hexdigest())
        self.assertEqual(set(read("hal.json")["pathSHA256"].values()),{physics["rackSHA256"]})
        for s in src:
            self.assertAlmostEqual(sum(a*a for a in s["axis"]),1)
            self.assertAlmostEqual(sum(a*w for a,w in zip(s["axis"],s["angularRadps"])),-s["rollXZRadps"])

    def test_support_and_timing_controls(self):
        rows={r["case"]:r for r in read("physics.json")["cases"]}
        for source in read("source.json"):
            key=source["pipe"]+"-"+source["mode"]
            live=rows[key+"-scheduled"]
            self.assertEqual(live["outcome"],"supported")
            self.assertGreater(live["final_dwell_s"],50)
            self.assertGreater(live["min_front_m"],.035)
            self.assertGreater(live["min_riser_m"],.015)
            self.assertLess(live["energy_gain_J"],.0011)
            self.assertNotEqual(rows[key+"-held"]["outcome"],"supported")
        for pipe in ("coral-A","coral-B"):
            key=pipe+"-nominal-"
            self.assertNotEqual(rows[key+"late"]["outcome"],"supported")
            for variation in ("fine","finer","low-friction","high-friction"):
                self.assertEqual(rows[key+variation]["outcome"],"supported")
            a,b=rows[key+"fine"],rows[key+"finer"]
            self.assertLess(abs(a["first_lip_s"]-b["first_lip_s"]),.0005)

    def test_distinguishes_station_reentry_and_camera_failure(self):
        with (STUDY/"events.csv").open(newline="") as f:events=list(csv.DictReader(f))
        for r in read("match.json")["cases"]:
            key=r["mode"]
            if r["pipe"]=="coral-A":
                self.assertEqual(r["conditionalRolePoints"],7)
                self.assertEqual(set(r["scoring"].values()),{"yes"})
                self.assertGreater(r["parkRobotToScoredPipeMm"],70)
                self.assertGreater(abs(r["returnStationRobotDxDyYawMmDeg"][1]),.1)
            else:
                self.assertNotIn("conditionalRolePoints",r)
                seq=[e["event"] for e in events if e["approach"]==key and e["scenario"]=="lateB" and e["piece"]==r["pipe"]]
                self.assertIn("hold",seq)
                self.assertNotIn("stow",seq)
