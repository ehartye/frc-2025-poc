"""Check the two sources of truth for receiver fit: installed geometry and bought articles."""

import csv
import hashlib
import json
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parents[1] / "cad/assembly-engineering/receiver-service"


def record(path):
    return json.loads((HERE / path).read_bytes())


def digest(path):
    return hashlib.sha256((HERE / path).read_bytes()).hexdigest()


class ReceiverService(unittest.TestCase):
    def test_fastener_interference_and_travel(self):
        grip = record("grip-fit.json")
        self.assertEqual(grip["drawingSha256"], digest("grip-carriers.step"))
        self.assertEqual(len(grip["observedIntersectionsMm3"]), 16)
        self.assertTrue(
            all("tie-" in pair for pair in grip["observedIntersectionsMm3"])
        )
        self.assertTrue(all(not h for h in grip["installedToolHits"].values()))
        for s in grip["supplier"].values():
            self.assertGreater(s["inheritedPlateInterferenceMm3"], 470)
            self.assertEqual(s["trueRotorToPlateMm3"], 0)
            self.assertGreater(s["housingPressBearingMm2"], 379)
        travel = record("travel.json")
        self.assertEqual(travel["adverse"], [])
        self.assertEqual(travel["source"]["switches"], digest("switch-mounts.step"))
        self.assertEqual(travel["rackTravelMm"], [0, 370])
        self.assertTrue(any("jaw-closing" in v for v in travel["checked"].values()))

    def test_actual_end_and_wrong_assembly(self):
        r = record("switch-controls.json")
        fit = record("switch-fit.json")
        self.assertEqual(r["drawing"], digest("switch-mounts.step"))
        self.assertEqual(len(fit["installedIntersectionsMm3"]), 2)
        for label, s in r["stations"].items():
            tip = s["surfaceByRackDistanceFromEndMm"]
            self.assertAlmostEqual(tip["0"], 456.15, places=2)
            self.assertGreater(tip["14"], 457.0)
            self.assertIsNone(tip["40"])
            self.assertTrue(all(not h for h in s["detachedSwitchDriverHits"].values()))
            self.assertIn(
                "slide-" + label + "-stop",
                fit["installedTools"]["slide-" + label + "-switch-driver-1"],
            )
        self.assertGreater(r["calibration"]["unadjustedLowOPMissesByMm"], 1)
        self.assertGreater(r["calibration"]["unadjustedHighOPOverdrivesToMm"], 1)
        self.assertEqual(
            record("stop-bearings.json")["drawing"], digest("stop-bearings.step")
        )

    def test_whole_bill_and_positive_mounts(self):
        d = record("battery-disconnect.json")
        self.assertEqual(d["publicDrawingSha256"], digest("battery-disconnect.step"))
        self.assertGreater(d["fitAtCatalogPairLengthMm3"], 17)
        self.assertEqual(d["fitAtQuarterMmLongerMm3"], 0)
        self.assertEqual(d["fullWithdrawal"]["adverse"], [])
        self.assertEqual(d["nominalIntersectionsMm3"], {})
        self.assertTrue(
            all(
                v["capPressMm2"] > 13 and v["spacerPressMm2"] > 17
                for v in d["ringSeating"].values()
            )
        )
        r = record("issue/interfaces.json")
        m = record("issue/mass.json")
        self.assertEqual(
            r["outputSha256"], digest("issue/station-and-jack-interfaces.step")
        )
        self.assertEqual(r["robotSolids"], 1268)
        self.assertEqual(
            record("roof/saddle-receiver-motion.json")["overlappingBroadPairs"], 0
        )
        with (HERE / "issue/material-register.csv").open(
            encoding="utf8", newline=""
        ) as f:
            solids = {v["id"]: v for v in csv.DictReader(f)}
        self.assertEqual(len(solids), r["robotSolids"] + 1)
        self.assertEqual(
            solids["SB50-fixed-housing"]["instance"],
            solids["SB50-battery-housing"]["instance"],
        )
        self.assertEqual(
            solids["slide-retracted-switch-lever"]["instance"], "slide-retracted-switch"
        )
        with (HERE / "issue/article-register.csv").open(
            encoding="utf8", newline=""
        ) as f:
            articles = {v["instance"]: v for v in csv.DictReader(f)}
        self.assertEqual(float(articles["base-harness"]["upperKg"]), 1.65)
        upper = sum(
            float(v["inspectionStockUpperKg"])
            for v in solids.values()
            if v["scope"] == "inspection"
        ) + sum(float(v["upperKg"]) for k, v in articles.items() if k != "np18-battery")
        self.assertAlmostEqual(m["inspectionUpperKg"], upper)
        self.assertTrue(0.020 < m["R103MarginKg"] < 0.024)

    def test_camera_carrier_and_residual_budget(self):
        r = record("camera-receiving.json")
        m = record("issue/mass.json")
        with (HERE / "issue/article-register.csv").open(encoding="utf8") as source:
            row = {x["instance"]: float(x["upperKg"]) for x in csv.DictReader(source)}
        self.assertAlmostEqual(row["camera-carrier-links"], 0.2806)
        self.assertAlmostEqual(row["camera-carrier-guides"], 0.120)
        self.assertAlmostEqual(sum(r["cableGroupCapsKg"].values()), row["camera-leads"])
        self.assertAlmostEqual(
            sum(v for k, v in row.items() if k.startswith("wire--converter--"))
            + row["camera-panel-fittings"],
            0.750,
        )
        self.assertAlmostEqual(
            row["unitemized-receiving"] + r["reassignedToNamedArticlesKg"], 5
        )
        self.assertAlmostEqual(m["includedUnresolvedReserveKg"], 3.86715)
        self.assertGreater(
            m["additionalFullReserveUpperKg"]
            - m["inspectionUpperKg"]
            - m["R103MarginKg"],
            1.1094,
        )

    def test_corrected_support_moves_with_camera_and_rollout_holds_at_jack(self):
        assignment = record("issue/interfaces.json")["frameAssignments"]
        frames = record("frames.json")
        self.assertEqual(frames["matchesReviewed"], 465)
        self.assertEqual(len(frames["movingSupportIds"]), 25)
        camera = "moving-camera-public-envelope"
        self.assertEqual(
            frames["publicEnvelopeSources"][camera]["nativeComponents"], 13
        )
        self.assertEqual(
            set(frames["corrected"]),
            {
                camera,
                "support--rail-M3--9.5",
                "support--data-M3--1",
                "support--power-M3--1",
            },
        )
        self.assertTrue(
            all(assignment[k] == "Outer slide" for k in frames["corrected"])
        )
        moving = record("issue/orbit.json")["movingGroups"]["Outer slide"]
        self.assertTrue(set(frames["movingSupportIds"] + [camera]) <= set(moving))
        travel = record("support-travel.json")
        self.assertEqual(travel["framesSha256"], digest("frames.json"))
        self.assertEqual(travel["guidePairs"], 7656)
        self.assertGreater(travel["worstGuidePair"][0], 3.43)
        self.assertTrue(
            all(not r["adverse"] for r in travel["correctedBodies"].values())
        )
        control = record("support-fixed-camera-control.json")
        self.assertTrue(control["heldCamera"])
        self.assertGreater(
            control["correctedBodies"]["support--data-M3--1"]["adverse"][0][
                "volumeMm3"
            ],
            160,
        )
        hold = record("carrier-motor.json")
        self.assertEqual(hold["geometry"], travel["issue"])
        self.assertGreater(hold["overlapMm3"]["data/370"], 318)
        self.assertGreater(hold["overlapMm3"]["power/370"], 457)
