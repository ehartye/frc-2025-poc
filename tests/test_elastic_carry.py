"""Check the identified station-to-drive handoff and the separate arrested forecasts."""

import hashlib
import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "elastic-carry"


def read(name):
    return json.loads((STUDY / (name + ".json")).read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class ElasticCarryTest(unittest.TestCase):
    def test_exact_modal_departure(self):
        d = read("departure")
        self.assertEqual(
            d["poweredStation"], sha(ROOT / "conditioned-match/elastic.json")
        )
        upstream = json.loads((ROOT / "conditioned-match/elastic.json").read_bytes())
        inputs = {
            r["case"]: r
            for r in upstream["cases"]
            if r.get("machinedTray") and r["stepS"] == 0.00003125
        }
        m = read("mode-frame")
        self.assertEqual(m["shape"], [145, 107, 3, 13])
        self.assertEqual(m["coupledSha256"], sha(ROOT / "shelf-structure/coupled.py"))
        for r in d["cases"]:
            self.assertEqual(
                r["stationTrajectorySha256"], inputs[r["case"]]["trajectorySha256"]
            )
            self.assertEqual(r["javaMs"], 5460)
            self.assertEqual(r["nextBeltMps"], 0)
            self.assertEqual(len(r["modalPosition"]), 13)
            self.assertEqual(len(r["modalVelocity"]), 13)
            self.assertGreater(max(map(abs, r["modalPosition"])), 0)
            self.assertEqual(r["plungerMm"], [0, 0])

    def test_two_time_steps_lose_forward_custody(self):
        traces = read("drive")
        self.assertEqual(len(traces), 4)
        for side in ("leftSmall", "rightSmall"):
            matching = [r for r in traces if r["case"] == side]
            self.assertEqual({r["stepS"] for r in matching}, {0.000125, 0.0000625})
            a, b = matching
            self.assertLessEqual(abs(a["endWallS"] - b["endWallS"]), 0.000125)
            self.assertEqual(a["stationStateSha256"], b["stationStateSha256"])
        for r in traces:
            self.assertEqual(r["loss"], "outside-flat-shelf")
            self.assertGreater(r["lastPipeXyzMm"][0], 165)
            self.assertLess(r["caseEndS"], r["routeEndS"])
        for r in read("deflection"):
            self.assertLess(r["globalVerticalUpperMm"], 0.05)

    def test_backing_and_actual_camera_cadence(self):
        applied = read("finish")
        self.assertTrue(all(abs(v - 1) < 1e-6 for v in applied["markAreaBacking"]))
        self.assertEqual(len(applied["markAreaBacking"]), 12)
        self.assertEqual(applied["fixedLeadBounds"], 629)
        self.assertEqual(read("native")["nativeBodies"], 998)
        for source in ("roof", "roof-front", "roof-rear"):
            for r in read(source):
                self.assertEqual(r["firstSampleJavaMs"], 5457)
                self.assertGreater(r["tested"], 100)
                self.assertEqual(
                    r["firstUnresolvedJavaMs"], 5457 + 17 * (r["tested"] - 1)
                )
        for source in (
            "fixed-native",
            "fixed-marked",
            "fixed-same",
            "roof-050",
            "roof-guard",
            "roof-same",
        ):
            self.assertEqual([r["tested"] for r in read(source)], [1, 1])

    def test_stop_order_and_pupil_control(self):
        stopped = read("stop") + read("stop-late")
        self.assertEqual(len(stopped), 16)
        for r in stopped:
            self.assertAlmostEqual(
                r["actuationWallS"] - r["alarmWallS"],
                r["assumedActuationLatencyMs"] / 1000,
            )
            self.assertAlmostEqual(
                r["stopDurationS"],
                max(map(abs, r["initialWheelMps"])) / r["wheelLimitMps2"],
            )
            if r["monitor"] == "ports":
                self.assertEqual(r["loss"], "outside-flat-shelf")
                self.assertIsNone(r["wheelStopPipeXyzMm"])
                self.assertLess(r["endWallS"], r["wheelStopWallS"])
            else:
                self.assertIsNone(r["loss"])
                self.assertEqual(r["endPorts"], [True, True])
                self.assertAlmostEqual(r["endWallS"] - r["wheelStopWallS"], 1.0)
        self.assertEqual(
            {r["firstMissingFromDepartureMs"] for r in read("ports")}, {9060}
        )

    def test_stopped_wheel_path_and_resource_limits(self):
        forecasts = read("stop-route")
        self.assertEqual(len(forecasts), 8)
        for r in forecasts:
            self.assertGreater(r["minBrakeBarrierMm"], 39)
            self.assertGreater(r["minBrakeStockMm"], 0)
            self.assertGreater(r["minParkedPeerMm"], 0)
            for source in r["sourceCases"].values():
                self.assertIsNone(source["firstReferenceInhibitWallS"])
                self.assertGreater(source["minTerminalV"], 9.59)

    def test_placement_remains_a_separate_fixture(self):
        check = read("placement")
        self.assertIsNone(check["actualScoredPipe"])
        self.assertTrue(check["sameEventsAsReference"])
        self.assertEqual(check["scenarioCount"], 28)
        self.assertEqual(
            check["packetEventSha256"], sha(ROOT / "packet-recovery/events.csv")
        )
        self.assertEqual(check["historicalNominal"]["withdrawWallMs"], 98375)
        self.assertEqual(check["BDesktopFrames"]["26231"]["moving"][0], "inside")
        self.assertEqual(
            check["BDesktopFrames"]["26248"]["moving"][0], "unresolved-end-curvature"
        )

    @unittest.skipUnless(
        importlib.util.find_spec("numpy"), "NumPy required for the brake trajectory"
    )
    def test_constant_curvature_stop_law(self):
        spec = importlib.util.spec_from_file_location("elastic_stop", STUDY / "stop.py")
        stop = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(stop)
        np = stop.np
        start = np.array([2.0, 3.0, 0.0])
        np.testing.assert_allclose(
            stop.point(start, np.array([1.0, 0.0]), 0.0, 2.0, 2.0), [3.0, 3.0, 0.0]
        )
        np.testing.assert_allclose(
            stop.point(start, np.zeros(2), 2.0, 2.0, 1.0), [2.0, 3.0, 1.0]
        )
        pose = stop.point(start, np.array([1.0, 0.0]), 1.0, 4.0, 2.0)
        np.testing.assert_allclose(
            pose, [2.0 + np.sin(1.0), 3.0 + 1.0 - np.cos(1.0), 1.0]
        )

    def test_stopped_pipe_needs_reacquisition(self):
        branches = read("stop-observations")
        self.assertEqual(len(branches), 4)
        self.assertTrue(
            all(r["loss"] is None and r["endPorts"] == [True, True] for r in branches)
        )
        images = read("stopped-images")
        self.assertEqual(len(images), 16)
        self.assertFalse(any(r["bounded"] for r in images))
