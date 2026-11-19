import csv
import hashlib
import json
from pathlib import Path
import unittest

STUDY=Path(__file__).resolve().parents[1]/"relieved-match"


def read(name):
    return json.loads((STUDY/name).read_bytes())


class RelievedMatchTest(unittest.TestCase):
    def test_pressure_gate_rejects_material_contradiction(self):
        native=read("native-register.json")
        self.assertEqual(native["nativeBodies"],597)
        for case in native["flatModelIntersections"]:
            self.assertEqual(len(case["positiveMaterialMm3"]),3)
            self.assertGreater(min(case["positiveMaterialMm3"].values()),43)
        for row in read("free-flight-clearance.json"):
            self.assertTrue(row["clear"])
            self.assertGreater(row["continuousLowerMm"],0)
        reg=read("controller-register.json")
        for k,hs in reg["inputs"].items():
            if k.endswith((".json",".csv")):
                self.assertEqual(hashlib.sha256((STUDY/k).read_bytes()).hexdigest(),hs,k)
        for part,record in reg["eligible"].items():
            self.assertTrue(record["eligibleCaptureMs"])
            for sensitivity in record["forceThresholdsMs"].values():
                self.assertIsNotNone(sensitivity["1.0"])
                self.assertIsNone(sensitivity["8.0"])
        self.assertFalse(reg["padThresholdMeasured"])

    def test_distinct_source_and_fresh_achieved_states(self):
        motions=read("navigation.json")["cases"]
        self.assertFalse(next(r for r in motions if r["mode"]=="dropout")["accepted"])
        self.assertEqual(len([r for r in motions if r["accepted"]]),2)
        for label,piece in (("first","coral-A"),("return","coral-B")):
            feed=read(label+"-feed.json");self.assertEqual(feed["identifiedPipe"],piece)
            failed=next(r for r in feed["cases"] if r["name"].endswith("displaced-source"))
            self.assertIsNotNone(failed["hardStop"]);self.assertIsNone(failed["stateAtDrive"])
            registration=next(r for r in motions if r["pipe"]==piece and r["mode"]=="nominal")
            self.assertEqual(feed["stationChassisMmDeg"],registration["actualStationMmDeg"])
            for dt in (250,500):
                path=read(f"{piece}-pads-edge-{dt}.json")
                self.assertEqual(path["feedTrajectorySha256"],feed["cases"][0]["trajectorySHA256"])
                self.assertEqual(path["poseSha256"],registration["motionSHA256"])
                self.assertGreater(path["edgeImpulseNs"],0)
        self.assertTrue(all(x["enclosed"] for x in read("roof-challenge.json")))
        self.assertEqual(len(read("roof-challenge.json")),36)

    def test_same_mechanism_clock_and_fault_scoring(self):
        with (STUDY/"events.csv").open() as f:events=list(csv.DictReader(f))
        with (STUDY/"pre-contact-control/events.csv").open() as f:cutoff=list(csv.DictReader(f))
        self.assertFalse(any(r["event"]=="feed" and r["piece"]=="coral-B" for r in cutoff))
        def time(piece,name):
            return int(next(r["wall_ms"] for r in events if (r["piece"],r["event"],r["scenario"])==(piece,name,"nominal")))
        self.assertLess(time("coral-A","stow"),time("coral-B","feed"))
        for key in ("coral-A","coral-B"):
            self.assertLess(time(key,"seat"),time(key,"carry"))
            self.assertLess(time(key,"withdraw"),time(key,"stow"))
        m=read("match.json")
        self.assertEqual({k:v["points"] for k,v in m["cases"].items()},{"nominal":9,"single":7,"lateB":3})
        self.assertEqual(m["cases"]["nominal"]["scoredPieces"],["coral-A","coral-B"])
        self.assertLess(m["cases"]["nominal"]["parkWallS"],111)
        self.assertGreater(m["successiveTubeLowerMm"],90)
        self.assertFalse(m["cases"]["lateB"]["parked"])
        self.assertEqual(m["cases"]["lateB"]["scoredPieces"],[])
        self.assertGreater(m["cases"]["nominal"]["stowedStockLowerMm"],1)
        self.assertGreater(min(r["branchLowerMm"] for r in m["sourceToFinal"].values()),0)
