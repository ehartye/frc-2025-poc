"""Check the two sources of truth for receiver fit: installed geometry and bought articles."""

import csv
import hashlib
import json
import unittest
from pathlib import Path

HERE=Path(__file__).resolve().parents[1]/"cad/assembly-engineering/receiver-service"


def record(path):
    return json.loads((HERE/path).read_bytes())


def digest(path):
    return hashlib.sha256((HERE/path).read_bytes()).hexdigest()


class ReceiverService(unittest.TestCase):
    def test_fastener_interference_and_travel(self):
        grip=record("grip-fit.json")
        self.assertEqual(grip["drawingSha256"],digest("grip-carriers.step"))
        self.assertEqual(len(grip["observedIntersectionsMm3"]),16)
        self.assertTrue(all("tie-" in pair for pair in grip["observedIntersectionsMm3"]))
        self.assertTrue(all(not h for h in grip["installedToolHits"].values()))
        for s in grip["supplier"].values():
            self.assertGreater(s["inheritedPlateInterferenceMm3"],470)
            self.assertEqual(s["trueRotorToPlateMm3"],0)
            self.assertGreater(s["housingPressBearingMm2"],379)
        travel=record("travel.json")
        self.assertEqual(travel["adverse"],[])
        self.assertEqual(travel["source"]["switches"],digest("switch-mounts.step"))
        self.assertEqual(travel["rackTravelMm"],[0,370])
        self.assertTrue(any("jaw-closing" in v for v in travel["checked"].values()))

    def test_actual_end_and_wrong_assembly(self):
        r=record("switch-controls.json")
        fit=record("switch-fit.json")
        self.assertEqual(r["drawing"],digest("switch-mounts.step"))
        self.assertEqual(len(fit["installedIntersectionsMm3"]),2)
        for label,s in r["stations"].items():
            tip=s["surfaceByRackDistanceFromEndMm"]
            self.assertAlmostEqual(tip["0"],456.15,places=2)
            self.assertGreater(tip["14"],457.0)
            self.assertIsNone(tip["40"])
            self.assertTrue(all(not h for h in s["detachedSwitchDriverHits"].values()))
            self.assertIn("slide-"+label+"-stop",fit["installedTools"]["slide-"+label+"-switch-driver-1"])
        self.assertGreater(r["calibration"]["unadjustedLowOPMissesByMm"],1)
        self.assertGreater(r["calibration"]["unadjustedHighOPOverdrivesToMm"],1)
        self.assertEqual(record("stop-bearings.json")["drawing"],digest("stop-bearings.step"))

    def test_whole_bill_and_positive_mounts(self):
        d=record("battery-disconnect.json")
        self.assertEqual(d["publicDrawingSha256"],digest("battery-disconnect.step"))
        self.assertGreater(d["fitAtCatalogPairLengthMm3"],17)
        self.assertEqual(d["fitAtQuarterMmLongerMm3"],0)
        self.assertEqual(d["fullWithdrawal"]["adverse"],[])
        self.assertEqual(d["nominalIntersectionsMm3"],{})
        self.assertTrue(all(v["capPressMm2"]>13 and v["spacerPressMm2"]>17 for v in d["ringSeating"].values()))
        r=record("issue/interfaces.json")
        m=record("issue/mass.json")
        self.assertEqual(r["outputSha256"],digest("issue/station-and-jack-interfaces.step"))
        self.assertEqual(r["robotSolids"],1268)
        self.assertEqual(record("roof/saddle-receiver-motion.json")["overlappingBroadPairs"],0)
        with (HERE/"issue/material-register.csv").open(encoding="utf8",newline="") as f:
            solids={v["id"]:v for v in csv.DictReader(f)}
        self.assertEqual(len(solids),r["robotSolids"]+1)
        self.assertEqual(solids["SB50-fixed-housing"]["instance"],solids["SB50-battery-housing"]["instance"])
        self.assertEqual(solids["slide-retracted-switch-lever"]["instance"],"slide-retracted-switch")
        with (HERE/"issue/article-register.csv").open(encoding="utf8",newline="") as f:
            articles={v["instance"]:v for v in csv.DictReader(f)}
        self.assertEqual(float(articles["base-harness"]["upperKg"]),1.65)
        upper=sum(float(v["inspectionStockUpperKg"]) for v in solids.values() if v["scope"]=="inspection")+sum(
            float(v["upperKg"]) for k,v in articles.items() if k!="np18-battery")
        self.assertAlmostEqual(m["inspectionUpperKg"],upper)
        self.assertTrue(.020<m["R103MarginKg"]<.024)
