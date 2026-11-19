import csv
import hashlib
import json
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
STUDY=ROOT/"return-station"


def read(name):
    return json.loads((STUDY/name).read_bytes())


class ReturnStationTest(unittest.TestCase):
    def test_bias_preserves_clock_and_actual_station(self):
        d=read("return.json")
        self.assertAlmostEqual(d["fixedStationLeftBiasMm"],-1.3)
        for k,r in d["cases"].items():
            self.assertLess(abs(r["stationRobotDxDyYawMmDeg"][1]),1.24)
            self.assertGreater(r["minStaticMm"],2.8)
            self.assertLess(r["unrestrictedRotationScreenMm"],0)
            self.assertGreater(r["minRobotMm"],100)
            self.assertGreater(r["settleBeforeFeedS"],5.7)
        self.assertGreater(len(read("dock.json")["poses"]["toward-wall"]["intersections_mm3"]),0)

    def test_real_collision_invalidates_an_apparent_port_witness(self):
        raw=next(v for v in read("feed.json") if v["name"]=="negative-250")
        self.assertGreater(raw["hardStop"][2],2.5)
        with_hits=[v for v in read("rigid.json")["cases"] if v["case"].startswith("bias-") and v["unloadedOutwardTravelMm"]==-1.5 and v["earliestSampledHit"]]
        self.assertEqual(len(with_hits),4)
        self.assertTrue(all(v["earliestSampledHit"]["beforeModeledWorkingBeltMs"]>0 for v in with_hits))
        old=[r for r in read("relief-feed.json") if r["name"].endswith("-250")]
        probe=[r for r in read("probe-feed.json") if r["name"].endswith("-250")]
        self.assertEqual(sum(r["firstPortBandMs"] is not None for r in old),2)
        self.assertTrue(all(r["firstPortBandMs"] is not None for r in probe))
        self.assertEqual(len(probe),5)

    def test_clearance_and_live_departure_are_conditioned(self):
        bound=read("coupled-clearance.json")["cases"]
        self.assertGreater(min(r["at025mmIndentationMm"] for r in bound),.06)
        self.assertLess(min(r["at050mmIndentationMm"] for r in bound),0)
        self.assertGreater(min(r["continuousLowerMm"] for r in read("pads.json")["results"]),.035)
        d=read("handoff.json")
        self.assertEqual(len(d["cases"]),5)
        for k,v in d["pins"].items():
            self.assertEqual(hashlib.sha256((ROOT/k).read_bytes()).hexdigest(),v)
        for r in d["cases"]:
            self.assertTrue(r["desktopDomainAtLastTick"] and r["expiredAtLastTick"])
            self.assertFalse(r["competitionPermission"])
            self.assertEqual(r["pipe"],"coral-B")
            self.assertEqual(r["nativeBodies"],597)
            self.assertAlmostEqual(r["scheduledDriveWallS"],77.43453010)
            self.assertAlmostEqual(sum(x*x for x in r["axisRobot"]),1)
