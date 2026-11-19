import hashlib
import importlib.util
import json
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
DIRECTORY=ROOT/"station-approach"
spec=importlib.util.spec_from_file_location("station_trial",DIRECTORY/"station_trial.py")
trial=importlib.util.module_from_spec(spec)
spec.loader.exec_module(trial)


class StationApproachTests(unittest.TestCase):
    def test_fresh_rotations_stop_at_target_across_opposed_ground_errors(self):
        for scale in ((.998,1.002),(1.002,.998)):
            r=trial.rollout(scale,.582,delay_ms=25)
            self.assertTrue(r["goalSensorConfirmed"])
            self.assertLess(abs(r["measuredWheelTravelM"]-trial.GOAL),.001)
            self.assertLess(r["settledS"],3)
            self.assertGreater(abs(r["lateralM"]),0)

    def test_missing_start_and_midrun_fault_latch_through_braking(self):
        cold=trial.rollout(fault="drop",fault_at=0)
        self.assertTrue(cold["latchedFault"])
        self.assertEqual(cold["forwardM"],0)
        for kind in ("late","drop","disagree"):
            r=trial.rollout(fault=kind,fault_at=.75)
            self.assertTrue(r["latchedFault"])
            self.assertFalse(r["goalSensorConfirmed"])
            self.assertGreater(r["postInhibitTravelM"],.10)
            self.assertLess(r["forwardM"],trial.GOAL)

    def test_geometry_and_clocks_share_current_inputs(self):
        path=DIRECTORY/"results.json"
        results=json.loads(path.read_bytes())
        field=json.loads((DIRECTORY/"field.json").read_bytes())
        self.assertEqual(field["sourceResultsSha256"],hashlib.sha256(path.read_bytes()).hexdigest())
        shop=ROOT/"selected-match/corrected-clearance.json"
        self.assertEqual(field["shopStowedSourceSha256"],hashlib.sha256(shop.read_bytes()).hexdigest())
        self.assertAlmostEqual(results["short"]["upperHeightAllowanceMm"],7.8,places=6)
        for key,expected in results["sources"].items():
            self.assertEqual(hashlib.sha256((ROOT/key).read_bytes()).hexdigest(),expected,key)
        for key in ("nominal","radius","track"):
            row=results["station"][key]
            self.assertEqual(row["allBoltsStereoFrameCount"],0)
            self.assertLess(row["lastHalfSecondLargestPanelRowPx"],0)
        edge=results["station"]["lowerEdgeOnly"]
        self.assertEqual(edge["-20"]["pixelY"],edge["20"]["pixelY"])
        for row in edge.values():
            self.assertLess(min(row["pixelX"]),0)
            self.assertGreater(max(row["pixelX"]),640)
        self.assertGreater(min(field["fullCorridorTeleopPeerGapMm"].values()),0)
        self.assertGreater(sum(field["parkErodedZoneCommonMm3"].values()),0)
        self.assertGreater(results["short"]["cageMarginAt3DegMm"],0)
        self.assertGreater(results["short"]["tapeLeaveMarginMm"],0)
