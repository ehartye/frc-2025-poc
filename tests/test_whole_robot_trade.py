"""Preserve configuration identity and signed controls in the competition-design comparison."""

import hashlib
import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "experiments/swerve-drive/whole-robot-trade"


def read(name):
    return json.loads((STUDY / (name + ".json")).read_bytes())


class WholeRobotTradeTest(unittest.TestCase):
    def test_hardware_sources_and_transverse_fault(self):
        report = read("robot-cases")
        for pin in report["sources"].values():
            self.assertEqual(
                hashlib.sha256((ROOT / pin["path"]).read_bytes()).hexdigest(),
                pin["sha256"],
            )
        nominal = report["currentFrontHoist"]["operating-split"]
        m = report["currentFrontHoist"]["single-loaded-screw"]
        self.assertAlmostEqual(nominal["operatingMassKg"], 65.398900971, places=6)
        self.assertGreater(nominal["voltageHeadroomV"], 1.9)
        self.assertGreater(m["heavierScrewForceN"], nominal["heavierScrewForceN"])
        self.assertLess(
            report["currentFrontHoist"]["binding-low-voltage"]["voltageHeadroomV"], -3.5
        )
        self.assertIsNone(nominal["qualifiedPawlTorqueNm"])
        fault = next(
            c
            for c in read("elevation-result")["mechanisms"][0]["cases"]
            if c["id"] == "one-side-missing"
        )
        self.assertLess(fault["unsupportedMomentNm"], -180.0)
        self.assertEqual(fault["status"], "fail")

    def test_longitudinal_offset_survives_two_vertical_screw_reactions(self):
        mechanism = read("spatial-result")["mechanisms"][0]
        cases = {case["id"]: case["balance"] for case in mechanism["cases"]}
        self.assertEqual(len(cases), 16)
        raised = cases["full-travel-upward-minusminus"]
        self.assertAlmostEqual(sum(raised["loads"]), 962.016198, places=5)
        self.assertAlmostEqual(raised["requiredRestraintReactionXYNm"][1], 351.135912, places=5)
        lost = cases["one-side-missing-minusminus"]
        self.assertAlmostEqual(lost["requiredRestraintReactionXYNm"][0], -231.525, places=2)
        self.assertEqual(lost["momentEquilibrium"], "unbalanced")

    def test_real_complete_layouts_and_failed_directions(self):
        d = read("design-review-result")
        self.assertEqual(d["qualifiedForTheseCases"], [])
        for design in d["concepts"]:
            for row in design["cases"]:
                expected = design["id"] == "front-jack-swerve" and row["id"].endswith(
                    "-design-entry"
                )
                self.assertEqual(row["status"], "pass" if expected else "fail")

    def test_stock_traffic_and_independent_cargo(self):
        base = read("throughput-result")
        self.assertEqual(
            [(r["id"], r["accepted"]) for r in base["episodes"]],
            [
                ("clear-ally", 15),
                ("blocked-ally", 0),
                ("one-pipe-left", 1),
                ("empty-after-preloads", 0),
            ],
        )
        self.assertEqual(base["passingRuns"], 1)
        self.assertEqual(read("robot-throughput-result")["observedRuns"], 0)
        refined = read("refinement-result")["cells"]
        correct = [r for r in refined if r["role"] == "upper-chute"]
        self.assertEqual(len(correct), 2)
        for c in correct:
            seq = [
                e
                for e in c["events"]
                if e["robot"] == "R1"
                and e["phase"] == "TELEOP"
                and e["outcome"] == "credited"
            ]
            self.assertEqual(len(seq), 15)
            self.assertEqual(len(set(e["pieces"][0] for e in seq)), 15)
            self.assertTrue(
                all(
                    e["station"] == "red-l1"
                    and e["pieces"][0].startswith("red-chute-high/")
                    for e in seq
                )
            )
        for x in read("refinement-stock"):
            self.assertEqual(
                x["focal"]["tally"].get("attempt/credited", {}).get("pieces", 0),
                x["availableAfterPreload"],
            )
