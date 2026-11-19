"""Finite-bore, retained mass and independent observation regressions."""

import importlib.util
import json
from pathlib import Path
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "experiments/swerve-drive/retention"


def module(name):
    spec = importlib.util.spec_from_file_location(name, STUDY / f"{name}.py")
    obj = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(obj)
    return obj


class SwerveRetainerTest(unittest.TestCase):
    def test_ring_is_observed_independently_at_both_ends(self):
        fingers = module("fingers")
        rows = fingers.geometry(
            np.array([75.0, 0.0, 557.0]), np.array([0.0, 1.0, 0.0]), 35.0
        )
        self.assertTrue(
            all(
                pin["clearance"] > 0 and pin["overlap"] > 20
                for hand in rows
                for pin in hand["pins"]
            )
        )
        good = fingers.observations(rows, 35.0, "nominal")
        self.assertFalse(fingers.permits([good] * 4))
        self.assertTrue(fingers.permits([good] * 5))
        for fault in ("empty", "missing-probe", "stuck-open"):
            with self.subTest(fault=fault):
                self.assertFalse(
                    fingers.permits(
                        [good] * 4 + [fingers.observations(rows, 35.0, fault)]
                    )
                )
        shifted = fingers.geometry(
            np.array([75.0, 0.0, 570.0]), np.array([0.0, 1.0, 0.0]), 35.0
        )
        self.assertFalse(
            fingers.permits([fingers.observations(shifted, 35.0, "nominal")] * 5)
        )

    def test_complete_finite_geometry_and_two_voltage_controls(self):
        screen = module("screen")
        data = json.loads((STUDY / "screen.json").read_bytes())
        self.assertEqual(data["robot"], screen.package())
        self.assertFalse(any(data["robot"]["positiveVolumeBoxOverlaps"].values()))
        self.assertEqual(len(data["station"]), 4)
        self.assertGreater(min(r["minimumToOpenTipsMm"] for r in data["station"]), 9.5)
        power = screen.servo(max(r["peakProbeN"] for r in data["stationary"].values()))
        self.assertEqual(power, data["servo"])
        self.assertEqual([r["passes"] for r in power], [True, False])
