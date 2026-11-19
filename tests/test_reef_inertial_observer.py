import importlib.util
import json
import math
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parents[1] / "reef-inertial-observer"
loader = importlib.util.spec_from_file_location("reef_rate", HERE / "rate.py")
rate = importlib.util.module_from_spec(loader)
loader.loader.exec_module(rate)


def results(name):
    return json.loads((HERE / (name + ".json")).read_bytes())


class ReefInertialObserver(unittest.TestCase):
    def test_gyro_latency_and_physical_integral(self):
        for accel in (-rate.ANGULAR_ACCEL, rate.ANGULAR_ACCEL):
            sensor = rate.RatePackets(0, 0, "nominal")
            for t in range(0, 500, 10):
                sensor.sample(t, 1.2 + accel * (t / 1000) ** 2 / 2)
            self.assertIsNone(sensor.at_capture(0))
            for stamp in range(35, 480, 35):
                estimate, error = sensor.at_capture(stamp)
                self.assertLessEqual(
                    abs(estimate - accel * (stamp / 1000) ** 2 / 2), error + 1e-12
                )
            del sensor.packets[200]
            self.assertIsNone(sensor.at_capture(210))

    def test_signed_decisions_and_controlled_faults(self):
        rows = {(r["wheelCase"], r["fieldInput"]): r for r in results("decisions")}
        self.assertEqual(len(rows), 16)
        for side in ("leftSmall", "rightSmall"):
            normal = rows[side, "nominal"]
            self.assertTrue(
                normal["fieldControllerAccepted"]
                and normal["fixtureEnclosesTrueFieldPose"]
            )
            self.assertAlmostEqual(normal["wallStoppedS"], 48.1621726353, places=6)
            for mode in ("dropout", "gyro-dropout", "wrong-face", "stale", "ambiguous"):
                self.assertFalse(rows[side, mode]["fieldControllerAccepted"])
            false = rows[side, "shared-survey-shift"]
            self.assertTrue(false["fieldControllerAccepted"])
            self.assertFalse(false["fixtureEnclosesTrueFieldPose"])
            self.assertEqual(normal["stationGuardBothRefinements"], side == "leftSmall")
            self.assertEqual(
                rows[side, "lateral-kick"]["fieldControllerAccepted"],
                side == "rightSmall",
            )
        self.assertTrue(
            all(
                not r["scoringPermission"] and r["supportedPipePoints"] == 0
                for r in rows.values()
            )
        )

    def test_true_signed_paths_and_stock(self):
        trials = {(r["side"], r["mode"]): r for r in results("results")}
        for field in results("clearance"):
            r = trials[field["side"], field["mode"]]
            self.assertEqual(field["motionSha256"], r["motionSHA256"])
            self.assertLess(
                field["maximumAngularAccelerationRadS2"], rate.ANGULAR_ACCEL
            )
            if field["mode"] != "nominal":
                continue
            self.assertEqual(field["guideCount"], 58)
            self.assertGreater(field["installedBallMm"], 134)
            self.assertGreater(field["barrierMm"], 41)
            self.assertGreater(field["peerMm"], 1290)
            self.assertLessEqual(r["terminalCheckPixelError"], 0.18)
            self.assertLess(r["acceptanceDiskMmYawDeg"][0], 10)
            self.assertLess(r["acceptanceDiskMmYawDeg"][1], 1)
            captures = [
                row for row in r["observations"] if row[0] <= r["lastGateCaptureMs"]
            ]
            self.assertTrue(any(row[1] == 2 for row in captures))
            self.assertEqual(captures[-1][1], 1)
