"""Protect the source-matched engineering bounds and distinguish hardware from trials."""

import hashlib
import importlib.util
import subprocess
import sys
import textwrap
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

    @unittest.skipUnless(
        importlib.util.find_spec("OCP") and importlib.util.find_spec("numpy"),
        "OpenCascade and NumPy required for native geometry and modes",
    )
    def test_live_exchange_and_undamped_held_force(self):
        # The shared project reuses module names across shop studies. Isolate this
        # import exactly as in the documented command-line entry point.
        program = textwrap.dedent("""
            import json
            import numpy as np
            from response import forced
            from tray import cad, volume, HERE
            record=json.loads((HERE/"installed.json").read_bytes())
            bodies=cad.read_named(HERE/"stow-receiver.step",
                                  [{"id":key} for key in record["volumeMm3"]])
            for key,body in bodies.items():
                assert cad.solid_count(body)==1
                assert abs(volume(body)-record["volumeMm3"][key])<0.002
            t=np.arange(21)/100
            w=np.array([8.,15.])
            q=np.tile([3.,5.],(len(t),1))
            z,accel=forced(q,w,0.,0.01)
            np.testing.assert_allclose(z, q*(1-np.cos(t[:,None]*w))/w**2, atol=1e-12)
            np.testing.assert_allclose(accel,q*np.cos(t[:,None]*w),atol=1e-12)
        """)
        subprocess.run([sys.executable, "-c", program], cwd=SHELF, check=True)

    def test_registered_sources(self):
        for name, expected in read("source-register").items():
            with self.subTest(name=name):
                self.assertEqual(
                    hashlib.sha256((ROOT / name).read_bytes()).hexdigest(), expected
                )

    def test_machined_ports_and_stereo_fail_closed(self):
        by_case = {(r["case"], r["stepS"]): r for r in read("coupled-machined")}
        self.assertEqual(len(by_case), 4)
        for r in by_case.values():
            self.assertTrue(r["machinedTray"])
            self.assertEqual(r["lastState"], "IN_BAND")
            self.assertEqual(r["lastJavaMs"], 5460)
            self.assertLess(r["peakCartridgeMm"], 1.4)
        custody = read("custody")
        self.assertEqual(
            custody["fixedBackgroundSha256"],
            read("background")["backgroundSha256"]["stowed-robot-fixed"],
        )
        self.assertEqual(custody["cableBounds"], 629)
        self.assertEqual(len(custody["cases"]), 2)
        for case in custody["cases"]:
            self.assertEqual(
                case["trajectorySha256"],
                by_case[case["case"], 0.00003125]["trajectorySha256"],
            )
            self.assertEqual(
                [f["javaMs"] for f in case["frames"]], [5372, 5389, 5406, 5423, 5440]
            )
            for frame in case["frames"]:
                self.assertLessEqual(frame["first5msTickMs"], 5460)
                self.assertEqual(frame["status"], "position-witness")
                self.assertEqual(frame["undeformedNativeStatus"], "position-witness")
                self.assertEqual(
                    frame["pipeColorOnChangedRays"], "insufficient-stereo-contour"
                )
                self.assertEqual(frame["oneEyeBlank"], case["blankControl"])
