import csv
from hashlib import sha256
import json
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
DESIGN=ROOT/"design"/"receiver-drives"


class ReceiverDriveTests(unittest.TestCase):
    def test_reopened_layer_matches_selected_body_inventory(self):
        checks=json.loads((DESIGN/"checks.json").read_bytes())
        with (DESIGN/"bom.csv").open(encoding="utf-8") as f:rows=list(csv.DictReader(f))
        names={r["id"] for r in rows}
        self.assertEqual(len(names),len(rows))
        self.assertIn("slide-power-NEO550",names)
        self.assertIn("slide-position-SP1-25",names)
        for pose in ("stowed","receive","transport","place"):
            entry=checks[pose]
            self.assertEqual(len(names),entry["parts"])
            self.assertEqual({},entry["exterior"]);self.assertEqual({},entry["interior"])
            original=ROOT/"design"/"station-l1"/(pose+".step")
            self.assertEqual(sha256(original.read_bytes()).hexdigest(),entry["source_sha256"])
        self.assertEqual(sha256((DESIGN/"grip-slide-place.step").read_bytes()).hexdigest(),checks["place_step_sha256"])

    def test_fractional_mesh_phases_and_independent_grip_limits(self):
        r=json.loads((DESIGN/"checks.json").read_bytes())
        for phase,gears in r["rack_phase_sweep_mm3"].items():
            self.assertEqual(3,len(gears))
            self.assertTrue(all(v==0 for v in gears.values()),phase)
        for key in ("opposed_limit_1.5","opposed_limit_-1.5"):
            self.assertEqual({"base":{},"internal":{}},r[key])
        self.assertTrue(any("UP5" in s for s in r["M5_vertical_service_blockers"]["L-rear"]))
        self.assertEqual([],r["M5_vertical_service_blockers"]["R-front"])

    def test_current_limits_surface_failures_and_source_pins(self):
        r=json.loads((DESIGN/"sizing.json").read_bytes())
        for p,digest in r["source_sha256"].items(): self.assertEqual(sha256((ROOT/p).read_bytes()).hexdigest(),digest)
        dry=[s for s in r["transfers"] if s["case"]=="dry nominal"]
        self.assertEqual(2,len({s["piece_id"] for s in dry}))
        self.assertEqual({-.18,.18},{s["sector_m"] for s in dry})
        for s in dry:
            self.assertEqual("sustained-paired-floor-and-lip",s["post"])
            self.assertTrue(s["within_sprung_travel"])
            self.assertEqual(0,s["minimum_intersected_end_rim_width_mm"])
            self.assertTrue(152<s["last_grip_u_mm"]<154)
            self.assertEqual(4,len(s["synthetic_shelf_observer"]))
        for s in r["transfers"]:
            if s["case"]=="low preload and wet":self.assertEqual("recover-rear-edge-departure",s["first_outcome"])
            if s["case"]=="no damping":self.assertFalse(s["within_sprung_travel"])
        grip=r["grip_acceleration_and_peak_force_combined"]
        self.assertTrue(grip["3e-06"]["feasible"])
        self.assertFalse(grip["7e-06"]["feasible"])
        for label,tr in r["slide"].items():
            if "140 N" in label:
                self.assertFalse(tr["feasible"]);self.assertLess(tr["sync_slack_N"],0)
            if "25 N" in label:
                self.assertTrue(tr["feasible"]);self.assertGreater(tr["sync_slack_N"],0)
