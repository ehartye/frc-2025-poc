"""Regression checks for the observed opening, retained geometry and issued wheel path."""

import importlib.util
import json
from pathlib import Path
import unittest
import numpy as np

HERE = Path(__file__).resolve().parents[1] / "straight-guide-match"
spec = importlib.util.spec_from_file_location("straight_edge", HERE / "edge.py")
edge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(edge)


def read(name):
    return json.loads((HERE / name).read_bytes())


class StraightGuideMatch(unittest.TestCase):
    def test_hidden_peak_with_pixel_error(self):
        rows = np.r_[np.arange(-16, -4), np.arange(5, 17)]
        for phase in (0.0, 0.25, 0.5):
            for sign in (-1, 1):
                values = 47 - 0.045 * (rows - phase) ** 2 + sign * (-1.0) ** rows
                upper = edge.peak(list(zip(rows, values)), -5, 5, 1.0)
                self.assertGreaterEqual(upper, 47.0)
                self.assertLess(upper, 57.0)

    def test_stale_station_packets_inhibit_both_feeds(self):
        nominal = read("revised-opening.json")
        for side in ("leftSmall", "rightSmall"):
            self.assertEqual(nominal[side]["status"], "observed stationary")
            self.assertLess(nominal[side]["stationWallS"], 21.1)
            for fault in ("noContrast", "late", "interrupt"):
                row = read(f"revised-opening-{fault}.json")[side]
                self.assertNotEqual(row["status"], "observed stationary")

    def test_anchoring_uses_camera_and_preserves_shared_survey_failure(self):
        cases = read("anchored-route.json")
        for side in ("leftSmall", "rightSmall"):
            accepted = next(
                r for r in cases if r["side"] == side and r["mode"] == "nominal"
            )
            self.assertTrue(accepted["accepted"])
            self.assertNotEqual(accepted["driveSeedMmDeg"], [0, 0, 0])
            stale = next(r for r in cases if r["side"] == side and r["mode"] == "stale")
            self.assertFalse(stale["accepted"])
        self.assertTrue(
            any(r["accepted"] and r["truthOutsideDeclaredCalibration"] for r in cases)
        )

    def test_both_feeds_transfer_distinct_native_and_observed_states(self):
        for side in ("leftSmall", "rightSmall"):
            hashes = set()
            for step in (31.25, 15.625):
                contact = read(f"feed-{side}-{step}.json")
                sensor = read(f"station-{side}-{step}.json")
                opening = read("revised-opening.json")[side]
                x, y, h = opening["reachedMmDeg"]
                self.assertEqual(contact["poseMmDeg"], [-x, -y, h])
                self.assertEqual(sensor["stateSha256"], contact["stateSha256"])
                self.assertEqual(sensor["pipeSha256"], contact["trajectorySha256"])
                hashes.add(contact["stateSha256"])
                self.assertTrue(sensor["guardedStationHandoff"])
                self.assertEqual(
                    [
                        f["serviceMs"]
                        for f in sensor["frames"]
                        if f["fixed"] and f["roof"] and f["serviceMs"] <= 5460
                    ],
                    [5390, 5405, 5425, 5440, 5455],
                )
            self.assertEqual(len(hashes), 2)

    def test_unobservable_common_station_shift_can_fail_acquisition(self):
        control = read("offset--3mm-leftSmall-31.25.json")
        self.assertEqual(control["lastState"], "FAULT")
        self.assertIsNone(control["firstBandMs"])
        self.assertEqual(control["reason"], "incoming-edges-unresolved")
        geometry = read("station-uncertainty.json")
        self.assertEqual(len(geometry["cases"]), 4)
        self.assertLess(
            max(x["rasterHalfDifferencePx"] for x in geometry["cases"]), 0.23
        )

    def test_jammed_wheels_stop_before_the_reef_route(self):
        for side in ("leftSmall", "rightSmall"):
            run = read(f"jam-{side}.json")
            self.assertTrue(run["surfaceStuck"])
            self.assertFalse(run["plannedEndpoint"])
            self.assertIn("idler", run["firstFault"][1])
            self.assertLess(run["finalWallS"], 27.17)

    def test_full_clock_has_one_identity_and_no_unobserved_score(self):
        for side in ("leftSmall", "rightSmall"):
            run = read(f"match-{side}-31.25-nominal.json")
            loaded = read(f"cargo-{side}-31.25.json")
            self.assertEqual(run["cargoTrace"], loaded["trajectorySha256"])
            self.assertTrue(loaded["plannedEndpoint"])
            self.assertEqual(run["field"]["stockId"], "coral-A")
            self.assertEqual(run["remainingStock"], ["coral-B"])
            self.assertEqual(run["scoredPieceIds"], [])
            self.assertGreater(run["bargeZoneSeparationMm"], 0)
            self.assertEqual(run["points"], {"leave": 3, "l1": 0, "park": 0})
            self.assertEqual([e[0] for e in run["events"]][-2:], [153, 156])
        traffic = read("peer-traffic.json")
        self.assertEqual(len(traffic["otherPairLowerBoundsMm"]), 10)
        self.assertGreater(min(traffic["otherPairLowerBoundsMm"].values()), 1300)
