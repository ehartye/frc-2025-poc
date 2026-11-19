from pathlib import Path
import importlib.util
import unittest

MOTION = Path(__file__).resolve().parents[1] / "experiments/swerve-drive/motion"
spec = importlib.util.spec_from_file_location(
    "departure_service_policy", MOTION / "station_clock.py"
)
policy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(policy)


class LoadedSwerve(unittest.TestCase):
    def setUp(self):
        self.issued = {
            "gate": {"accepted": True},
            "epoch": 21.571672635314284,
            "frames": [
                {"captureJavaMs": t, "accepted": True}
                for t in (5372, 5389, 5406, 5423, 5440, 5457)
            ],
        }
        self.first = 27.035
        self.stop = 27.635

    def test_freshness_and_arrival_are_causal(self):
        tick, _ = policy.first_hold(self.issued, [], self.first, self.stop)
        self.assertAlmostEqual(tick, 27.070)
        frame = {"captureWallS": self.issued["epoch"] + 5.474, "accepted": False}
        tick, _ = policy.first_hold(self.issued, [frame], self.first, self.stop)
        self.assertAlmostEqual(tick, 27.065)
        frame["accepted"] = True
        tick, _ = policy.first_hold(self.issued, [frame], self.first, self.stop)
        self.assertAlmostEqual(tick, 27.090)
        self.issued["gate"]["accepted"] = False
        self.assertEqual(
            policy.first_hold(self.issued, [frame], self.first, self.stop)[0],
            self.first,
        )

    def test_eligible_observations_cannot_extend_command(self):
        rows = [
            {"captureWallS": self.issued["epoch"] + ms / 1000, "accepted": True}
            for ms in range(5474, 6150, 17)
        ]
        self.assertAlmostEqual(
            policy.first_hold(self.issued, rows, self.first, self.stop)[0], self.stop
        )
