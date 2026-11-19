import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import unittest

import numpy as np

MOTION = Path(__file__).resolve().parents[1] / "experiments/swerve-drive/motion"
sys.path.insert(0, str(MOTION))
try:
    spec = importlib.util.spec_from_file_location(
        "reciprocal_motor_test", MOTION / "plant.py"
    )
    plant = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(plant)
finally:
    sys.path.remove(str(MOTION))


class SwerveEnergy(unittest.TestCase):
    def test_free_run_stall_and_empirical_mismatch(self):
        for axis in ("drive", "steer"):
            motor = plant.SETTINGS[axis]
            free = motor["freeRpm"] * 2 * np.pi / 60
            torque, current, volts = plant.available_torque(0.0, free, motor, 12.0)
            np.testing.assert_allclose(
                [torque, current, volts], [0.0, motor["freeCurrentA"], 12.0], atol=1e-12
            )
            torque, current, _ = plant.available_torque(99.0, 0.0, motor, 12.0)
            self.assertAlmostEqual(current, motor["phaseLimitA"])
            self.assertAlmostEqual(torque, plant.motor_constants(motor)[1] * current)
            with self.assertRaisesRegex(ValueError, "No current"):
                plant.available_torque(0.0, free * 3, motor, 0.0)

    def test_four_quadrants_and_disabled_short(self):
        for axis in ("drive", "steer"):
            motor = plant.SETTINGS[axis]
            resistance, _, viscous = plant.motor_constants(motor)
            for speed in (-100.0, 100.0):
                for requested in (-0.2, 0.0, 0.2):
                    for supply in (0.0, 9.5):
                        torque, amps, volts = plant.available_torque(
                            requested, speed, motor, supply
                        )
                        self.assertAlmostEqual(
                            volts * amps - torque * speed - amps**2 * resistance,
                            viscous * speed**2,
                        )
                        self.assertLessEqual(abs(volts), supply + 1e-10)
                        if supply == 0.0:
                            self.assertLess(torque * speed, 0.0)
        torque, current, v = plant.historical_torque(
            0.2, 100.0, plant.SETTINGS["drive"], 9.5
        )
        self.assertAlmostEqual(
            v * current - torque * 100 - current**2 * 12 / 105, -3.97320852, places=6
        )

    def test_coupled_backdrive_and_opposed_shaft(self):
        gear = plant.SETTINGS["drive"]
        roll, yaw = 10.0, -2.0
        for polarity in (-1.0, 1.0):
            rate = gear["ratio"] * roll + polarity * yaw
            for torque in (-0.5, 0.5):
                equivalent = torque * plant.transmission_factor(
                    torque * rate, gear["efficiency"]
                )
                wheels = equivalent * gear["ratio"] * roll
                coupling = equivalent * polarity * yaw
                joint = wheels + coupling
                self.assertGreaterEqual(torque * rate - joint, 0.0)
                self.assertAlmostEqual(joint, equivalent * rate)
                self.assertNotAlmostEqual(wheels - coupling, equivalent * rate)

    def test_registered_paths(self):
        data = json.loads((MOTION / "energy.json").read_bytes())
        for source, expected in data["sourceSha256"].items():
            self.assertEqual(
                hashlib.sha256((MOTION / source).read_bytes()).hexdigest(),
                expected,
                source,
            )
        self.assertLess(data["ports"]["drive-historical"][0]["otherLossW"], 0.0)
        for case, row in data["cases"].items():
            self.assertEqual(row["energy"]["observations"], 168800)
            self.assertLess(row["energy"]["maxMotorIdentityErrorW"], 1e-10)
            self.assertGreaterEqual(row["energy"]["minGearLossW"], -1e-10)
            self.assertGreater(row["energy"]["reverseGearInputJ"], 0)
            self.assertNotEqual(row["csvSha256"], row["historicalCsvSha256"])
            self.assertEqual(
                row["stationLockWallS"] is not None,
                case in ("plus", "minus", "heavy", "upper-minus"),
            )
            self.assertTrue(all(p["supplied"] for p in row["battery"].values()))
