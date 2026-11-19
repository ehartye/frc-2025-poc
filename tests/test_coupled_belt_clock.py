import hashlib
import importlib.util
import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "coupled-belt-clock"
spec = importlib.util.spec_from_file_location("held_belt_replay", HERE / "replay.py")
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)
STUDY = json.loads((HERE / "study.json").read_bytes())


class HeldBeltClock(unittest.TestCase):
    def test_recorded_inputs_match_current_sources(self):
        for source, sha in STUDY["sourceSha256"].items():
            with self.subTest(source=source):
                self.assertEqual(sha, hashlib.sha256((ROOT / source).read_bytes()).hexdigest())

    def test_held_edges_and_limits(self):
        shaped = replay.Trace("RAMP_090")
        original = replay.Trace("HISTORICAL")
        self.assertEqual([shaped.held(t) for t in (0, .0199, .020, .0399, .040, .3999, .400)],
                         [0, 0, .45, .45, .9, .9, 0])
        self.assertEqual(1.05, original.held(0))
        self.assertEqual(-.2, shaped.held(0, acquisition=True))
        with self.assertRaises(ValueError):
            shaped.held(.75)

    def test_model_cannot_claim_acquisition_or_ordinary_withdrawal(self):
        for run in STUDY["acquisition"].values():
            self.assertEqual("hold-unconfirmed-seat", run["status"])
            self.assertLess(run["seat_u_m"], -.03)
            contradiction = run["impossible_scripted_blockages"][0]
            self.assertEqual(("entry", 120), (contradiction["port"], contradiction["scripted_ms_after_lip"]))
            self.assertGreater(contradiction["projected_gap_mm"], 16)
        for profile, ids in STUDY["releases"].items():
            self.assertEqual(set(ids), {"coral-A", "coral-B"})
            outcomes = ("inner-riser-bound", "inner-riser-bound") if profile == "HISTORICAL" else (
                "sustained-paired-floor-and-lip", "receiver-recontact")
            for steps in ids.values():
                self.assertEqual(set(steps), {"0.00025", "0.0005"})
                for row in steps.values():
                    self.assertEqual(outcomes, (row["status"], row["desktop_fixed_slide"]["status"]))
                    self.assertLess(abs(row["angular_momentum_residual_Nms"]), 1e-10)
                    self.assertLessEqual(row["force_fraction"], 1 + 1e-12)
                    self.assertEqual(row["release_spin_rad_s"], row["target"]["input_spin_rad_s"])
