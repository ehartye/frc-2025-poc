import hashlib
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "compliant-carriage"


def read(p):
    return json.loads(p.read_bytes())


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


class CompliantCarriage(unittest.TestCase):
    def test_native_stiffness_and_relative_damper_coordinates(self):
        b = read(HERE / "boundary.json")
        self.assertEqual(
            b["sourceStepSha256"],
            digest(ROOT / "carriage-support/carriage-support.step"),
        )
        self.assertGreater(b["armFlexureMmPerN"][0], b["armFlexureMmPerN"][1] * 100)
        for i, row in enumerate(b["complianceMmPerN"]):
            for j, val in enumerate(row):
                self.assertAlmostEqual(val, b["complianceMmPerN"][j][i])
        q = read(HERE / "basis-checks.json")
        for k, n in (("1", 19), ("2", 21)):
            row = q["modalChecks"][k]
            self.assertEqual(row["transformedModes"], n)
            self.assertLess(max(row["heavePitchRollFalseStrokeM"]), 1e-12)
            self.assertLess(row["relativeEigenResidual"], 1e-8)
            for v in row["actual1mmGuideTravelAtBothDampersM"]:
                self.assertAlmostEqual(v, 0.001)
        self.assertLess(
            max(
                abs(x)
                for r in q["staticPinDisplacementMmAt9WeightDistributions"]
                for x in r
            ),
            0.1,
        )

    def test_distinct_dock_poses_and_both_stage_clocks(self):
        poses = read(ROOT / "conditioned-match/opening.json")
        for file in ("source", "refine", "limits", "series", "engaged"):
            h = read(HERE / (file + ".json"))
            for k, v in h["pinnedInputs"].items():
                self.assertEqual(digest(ROOT / k), v)
            for r in h["cases"]:
                if r["approach"] == "powered-dock":
                    self.assertEqual(
                        r["sourceRobotMmDeg"], poses[r["case"]]["sourceRobotMmDeg"]
                    )
                self.assertLessEqual(r["lastJavaMs"], r["deadlineMs"] + 20)
                if r["trial"] == "rigid":
                    continue
                self.assertEqual(len(r["maxAbsOldPinForceN"]), 2)
                self.assertEqual(len(r["landHistorySha256"]), 64)
        both = read(HERE / "series.json")["cases"]
        self.assertEqual(len(both), 4)
        self.assertEqual({r["stepS"] for r in both}, {31.25e-6, 15.625e-6})
        self.assertTrue(all(len(r["individualStageRangeMm"][0]) == 4 for r in both))

    def test_stage_resolution_and_pin_seating_change_the_physical_conclusion(self):
        comparison = read(HERE / "comparison.json")
        for path, sha in comparison["mechanismAndControllerInputs"].items():
            self.assertEqual(digest(ROOT / path), sha)
        cases = comparison["cases"]
        split_left = [
            r for r in cases if r["trial"] == "two-stages" and r["case"] == "leftSmall"
        ]
        self.assertEqual(len(split_left), 2)
        for row in split_left:
            self.assertEqual(
                (row["lastState"], row["fault"], row["lastJavaMs"]),
                ("FAULT", "incoming-edges-unresolved", 1020),
            )
            self.assertEqual(row["portTransitions"][-1][2:4], [False, True])
            self.assertEqual(row["pinPeakN"], [0.0, 0.0])
        combined = [
            r
            for r in cases
            if r["trial"] == "bracket" and r["approach"] == "powered-dock"
        ]
        self.assertTrue(all(r["lastState"] == "IN_BAND" for r in combined))
        self.assertEqual({r["firstBandJavaMs"] for r in combined}, {1340, 1360})
        seated = [r for r in cases if r["trial"] == "zero-pin-gap-integrated"]
        self.assertTrue(all(min(r["pinPeakN"]) > 400.0 for r in seated))
        self.assertEqual(
            {r["lastState"] for r in seated if r["stepS"] == 15.625e-6}, {"FAULT"}
        )
        self.assertEqual(
            {r["lastState"] for r in seated if r["stepS"] == 31.25e-6}, {"IN_BAND"}
        )
        self.assertIsNone(comparison["physicalDatum"])

    def test_reject_held_stiff_pin_and_keep_all_published_controls(self):
        rows = read(HERE / "comparison.json")["cases"]
        failures = [r for r in rows if r["trial"] == "zero-pin-gap"]
        corrected = [r for r in rows if r["trial"] == "zero-pin-gap-integrated"]
        self.assertEqual(len(failures), 2)
        self.assertEqual(len(corrected), 4)
        self.assertTrue(all("rejected" in r["classification"] for r in failures))
        self.assertGreater(min(max(r["pinPeakN"]) for r in failures), 100000.0)
        self.assertLess(max(max(r["pinPeakN"]) for r in corrected), 10000.0)
        self.assertTrue(
            any(
                r["trial"] == "bracket" and r["firstBandJavaMs"] is not None
                for r in rows
            )
        )
        self.assertTrue(
            any(
                r["trial"] == "two-stages" and r["firstBandJavaMs"] is None
                for r in rows
            )
        )
        self.assertTrue((HERE / "chart.html").is_file())
