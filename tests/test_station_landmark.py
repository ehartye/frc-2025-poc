import hashlib
import json
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
HERE=ROOT/"station-landmark"


def read(file):
    return json.loads((HERE/file).read_bytes())


class StationLandmarkTests(unittest.TestCase):
    def test_image_lock_and_pipe_contact_are_separate_gates(self):
        routes=read("routes.json")
        feed=read("feed.json")
        self.assertEqual(hashlib.sha256((HERE/"routes.json").read_bytes()).hexdigest(),feed["routesSha256"])
        self.assertEqual(routes["equalChassisAndCameraShiftPixelDifference"],0)
        expected={"nominal","trackOnly","bothLarge","bothSmall","crossedDrift"}
        observed={k for k,r in feed["outcomes"].items() if r["lastPortState"]=="IN_BAND" and
                  r["hardStop"] is None and not r["loss"]}
        self.assertEqual(observed,expected)
        for name in ("leftSmall","rightSmall","badRegistration"):
            r=routes["cases"][name]
            self.assertTrue(r["eligibleInRehearsal"])
            d=r["observer"]["finalStationMmDeg"]
            self.assertEqual([-d[0],-d[1],d[2]],feed["outcomes"][name]["actualRobotMmDeg"])
            self.assertIsNotNone(feed["outcomes"][name]["hardStop"])
        for name in expected|{"leftSmall","rightSmall"}:
            r=routes["cases"][name]
            self.assertLess(r["observer"]["wallS"],feed["humanFeedWallS"])
            self.assertEqual(r["observer"]["maxTransitionWheelMmS"],0)
            self.assertGreater(r["occupancy"]["minimumStockMm"],19)

    def test_stock_source_and_fault_holds_cannot_inherit_nominal_feed(self):
        routes=read("routes.json")
        for key,d in routes["sources"].items():
            if key=="native.json":path=HERE/key
            elif key.startswith(("selected-optics/","full-match/","relieved-match/")):path=ROOT/key
            else:continue
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),d)
        no=routes["cases"]["stockAtFarStage"]
        self.assertEqual(no["observer"]["status"],"observed stationary")
        self.assertLess(no["occupancy"]["minimumStockMm"],0)
        self.assertFalse(no["eligibleInRehearsal"])
        for name in ("noGyro","lostGyro","noContrast","lateStereo","lostStereo"):
            r=routes["cases"][name]
            self.assertFalse(r["eligibleInRehearsal"])
            self.assertGreater(r["occupancy"]["minimumPeerMm"],0)
            self.assertGreater(min(r["occupancy"]["holdThroughTeleopPeerMm"].values()),0)
        recovery=routes["cases"]["lostGyro"]["inertial"]["gyroLossRecovery"]
        self.assertLess(recovery["fieldReturnErrorMm"],20)
        self.assertEqual(recovery["shaftResidualMm"],[0,0])

    def test_distinct_native_datums_and_finite_image_ambiguity(self):
        n=read("native.json")
        self.assertEqual(len(n["frontWireCenterYzAndSizeMm"]),12)
        self.assertGreater(min(n["startTapeCommonMm3"].values()),5700)
        c=read("ambiguity.json")[-1]
        self.assertEqual(c["sideSeparationMm"],3)
        self.assertLess(c["sharedObservationErrorPx"],.205)
