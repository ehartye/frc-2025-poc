import csv
import hashlib
import json

import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT/"observable-acquisition"
RESULTS = json.loads((HERE/"results.json").read_bytes())
CASES = RESULTS["cases"]


class ObservableAcquisition(unittest.TestCase):
    def test_native_channel_and_evidence_provenance(self):
        native = json.loads((HERE/"ports.json").read_bytes())
        self.assertEqual(545, native["installed_bodies"])
        step = ROOT/"design/competition-package/shop-duplex/red/receive.step"
        self.assertEqual(hashlib.sha256(step.read_bytes()).hexdigest(), native["receive_sha256"])
        for k,x in (("entry", 340), ("seat", 280)):
            channel = native["channels"][k]
            for name, bottom, top in (("receiver-R6-ported",865,880), ("optical-mount-"+k,858,865)):
                hole = channel[name]
                self.assertEqual([x,0], hole["axis_xy_mm"])
                self.assertEqual(6, hole["radius_mm"])
                self.assertAlmostEqual(bottom, hole["face_bounds_mm"][2])
                self.assertAlmostEqual(top, hole["face_bounds_mm"][5])
        hal = json.loads((HERE/"hal.json").read_bytes())
        with (HERE/"ticks.csv").open() as file:
            rows = list(csv.DictReader(file))
        self.assertEqual(hal["ticks"],len(rows))
        self.assertEqual(hal["trials"],sum(r["ms"]=="0" for r in rows))
        for document in (hal,RESULTS):
            for path,digest in document["source_sha256"].items():
                self.assertEqual(digest,hashlib.sha256((ROOT/path).read_bytes()).hexdigest(),path)

    def test_dual_return_gauge_and_ordinary_wide_optics(self):
        lo, hi = RESULTS["two_true_center_bound_mm"]
        self.assertGreater(lo,72)
        self.assertLess(hi,88)
        # Fully illuminated holes without range discrimination can both see these off-target centers.
        for center in (49,91):
            self.assertTrue(all(abs(center-port)<57.15+6 for port in (40,100)))
            self.assertFalse(72 <= center <= 88)
        for case, levels in CASES.items():
            self.assertEqual(levels["0.00025"]["outcome"],levels["0.0005"]["outcome"],case)
            for result in levels.values():
                if result["outcome"] == "IN_BAND":
                    self.assertTrue(lo <= result["first_in_band"]["u_mm"] <= hi,case)
                    self.assertLess(abs(result["last"]["speed_mps"]),.04,case)
                    self.assertEqual("IN_BAND",result["last"]["state"])
                else:
                    self.assertFalse(result["last"]["active"],case)
                    self.assertIsNone(result["first_in_band"],case)

    def test_scanned_return_and_fault_controls(self):
        phase_cases = ["nominal"] + [f"phase-{p}" for p in range(1,20)] + [
            f"depth-{d}-phase-{p}" for d in (-5,5) for p in range(20)]
        with (HERE/"ticks.csv").open() as file:
            rows = list(csv.DictReader(file))
        aliased = 0
        for name in phase_cases:
            trial = [r for r in rows if r["case"] == name and r["step_us"] == "500"]
            first = next(r for r in trial if r["entry"] == "1" or r["seat"] == "1")
            aliased += first["entry"] == first["seat"] == "1"
            self.assertEqual("IN_BAND",CASES[name]["0.0005"]["outcome"])
            self.assertEqual("BRAKE",first["state"])
            self.assertEqual(.45,float(first["belt_mps"]))
        self.assertEqual(12,aliased)
        self.assertEqual("physical-custody-lost",CASES["low-grip"]["0.0005"]["outcome"])
        for name in ("uncalibrated","disabled","out-of-plane","stale","lost-dock","missing-frame",
                     "drop-seat-in-dwell","aliased-pair-stuck","entry-high-at-wake","seat-stuck-high"):
            self.assertEqual("neutral-fault",CASES[name]["0.0005"]["outcome"],name)
        sweep = json.loads((HERE/"sweep.json").read_bytes())
        flight = json.loads((HERE/"flight.json").read_bytes())
        self.assertEqual({},sweep["selected_continuous_overlap_mm3"])
        self.assertEqual({"grip-L-drive-22T","grip-R-drive-22T"},
                         set(sweep["annular_material_at_selected_u_mm"]["-33.32312"]))
        self.assertEqual(6,len(flight["inbound_groups"]))
        for group in flight["inbound_groups"]:
            self.assertLessEqual(set(group["padded_outer_overlap_mm3"]),{"receiver-R6-ported"})
        for artifact in (sweep,flight):
            self.assertEqual(hashlib.sha256((HERE/"results.json").read_bytes()).hexdigest(),artifact["results_sha256"])
        self.assertFalse(RESULTS["carry_permitted"])
        self.assertEqual("IN_BAND",RESULTS["empty_receiver_identical_electronics"]["terminal_state"])
