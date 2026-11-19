"""Protect the source-matched engineering bounds and distinguish hardware from trials."""

import hashlib
import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SHELF = ROOT / "shelf-structure"


def read(name):
    return json.loads((SHELF / f"{name}.json").read_bytes())


class ReceivingShelfTest(unittest.TestCase):
    def test_machining_and_purchased_parts_stay_separate(self):
        issue = read("installed")
        for name, expected in (
            ("stow-receiver.step", issue["manufacturingSha256"]),
            ("shelf-insert.step", read("native")["kitSha256"]),
        ):
            self.assertEqual(
                hashlib.sha256((SHELF / name).read_bytes()).hexdigest(), expected
            )
        self.assertEqual(len(issue["volumeMm3"]), 3)
        self.assertLess(issue["resultFaceInsertInterferenceMm3"], 0.001)
        self.assertLess(issue["receiverEscapeMm3"], 0.001)
        self.assertEqual(issue["pressureLaminates"]["zShiftMm"], -0.8)
        self.assertIsNone(issue["pressureLaminates"]["newPhysicalPressureThreshold"])
        self.assertEqual(
            [r["xyMm"] for r in issue["supports"]], [[118, -66], [118, 42]]
        )
        for site in issue["supports"]:
            self.assertGreater(site["tipLandingFraction"], 0.999)
            self.assertAlmostEqual(site["freeTipToUndeformedLandMm"], 0.1)
        self.assertIsNone(issue["actualWeighedDelta"])

    def test_nested_rail_contact_and_rebound(self):
        source = read("supports")
        for name, digest in source["sourcePins"].items():
            self.assertEqual(
                hashlib.sha256((ROOT / name).read_bytes()).hexdigest(), digest
            )
        for side, record in source["sides"].items():
            faces = record["interfaces"]
            self.assertEqual(faces["innerFloorFrontLocalSeparationMm"], 20)
            for a, b in (
                (f"drilled-slide-{side}-inner", f"slide-{side}-middle"),
                (f"slide-{side}-middle", f"slide-{side}-outer"),
            ):
                self.assertEqual(faces[f"{a} to {b}"]["initialSeparationMm"], 1)
        for case in read("transfer")["cases"]:
            for row in case["coincident"].values():
                self.assertAlmostEqual(row[1] + row[2], -row[5], places=7)
            self.assertLess(case["coincident"]["leftRailDownN-min"][1], 0)

    def test_distributed_pulse_convergence_and_held_load(self):
        candidates = {
            r["nx"]: r
            for r in read("response")["cases"]
            if (r["hardware"], r["case"], r["law"], r["dampingRatio"])
            == ("insert", "leftSmall", "400k", 0.02)
        }
        converged = candidates[15]
        self.assertAlmostEqual(
            converged["peakMeanTravelMm"],
            candidates[13]["peakMeanTravelMm"],
            delta=0.02,
        )
        self.assertAlmostEqual(converged["pulseImpulseNs"], 2.59, delta=0.02)
        self.assertGreater(max(converged["gauges"]["staticAlMPa"]), 276)
        self.assertLess(max(converged["gauges"]["dynamicAlMPa"]), 200)
        self.assertIsNone(read("response")["physicalDeformationMm"])

    def test_symmetric_rack_interference_and_service_probe(self):
        rejected = read("rejected-symmetric-fit")["results"][0]["sides"]
        self.assertEqual(rejected["-1"]["supplier"], {})
        self.assertGreater(
            rejected["1"]["supplier"]["slide-inner-rack-shaft"]["commonMm3"], 130
        )
        fits = read("damper-fit")
        self.assertEqual(fits["strokeMm"], [0, 370])
        self.assertEqual([r["yMm"] for r in fits["results"]], [-66, 42])
        for site in fits["results"]:
            fit = site["sides"]["1"]
            for name in (
                "supplier",
                "lowerNut",
                "upperNut",
                "axialAdjustmentToolAtStow",
            ):
                self.assertEqual(fit[name], {})
            self.assertGreater(fit["upperNutSweptAabbLowerMm"], 4.6)

    def test_rigid_face_and_gap_controls(self):
        self.assertTrue(all(r["firstBandJavaMs"] is None for r in read("coupled")))
        rigid = [r for r in read("coupled-controls") if r["modeCutoffHz"] == 0]
        self.assertEqual([r["lastState"] for r in rigid], ["IN_BAND"] * 2)
        gap = read("coupled-installed")
        self.assertEqual(
            [r["lastState"] for r in gap if r["runningGapMm"] == 0.2], ["FAULT"] * 2
        )
        self.assertEqual(
            [r["lastState"] for r in gap if r["runningGapMm"] == 0.1], ["IN_BAND"] * 2
        )
        for r in read("coupled-mounts"):
            self.assertEqual(r["damperYmm"], [-66, 42])
            self.assertGreater(min(r["damperDemandJ"]), 0.4)
            self.assertLess(max(r["damperStrokeMm"]), 0.8)
            self.assertEqual(len(r["supportHistorySha256"]), 64)
            self.assertLess(r["lastJavaMs"], 5460)

    def test_mass_coverage_with_and_without_removal_credit(self):
        mass = read("installed-mass")
        for check in mass["installedChecker"].values():
            self.assertTrue(
                all(not problems for problems in check["coverage"].values())
            )
            self.assertIsNone(check["physicalMarginKg"])
        debit = mass["installedChecker"]["noCutCreditKg"]
        self.assertLess(debit["representedMarginKg"][0], -0.82)
        margin = mass["integralDeckAlternatives"]["restrictedNoMotorCreditKg"][
            "withSelectedAndGuideKg"
        ]
        self.assertAlmostEqual(margin, 0.03017, places=4)
        self.assertIsNone(mass["physicalInspectionMarginKg"])
