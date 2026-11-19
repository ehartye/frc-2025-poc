"""R4 scoring-end results must keep source custody, physical controls and scope."""

import hashlib
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
LAB = ROOT / "experiments/swerve-drive/score-stroke"


def read(name):
    return json.loads((LAB / name).read_bytes())


class ScoringStroke(unittest.TestCase):
    def test_selected_robot_and_actual_cap(self):
        model = read("native.json")
        selected = json.loads(
            (ROOT / "cad/full-swerve-r4/native-reopen.json").read_bytes()
        )
        self.assertEqual(model["robotSha256"], selected["source"]["publicStepSha256"])
        self.assertEqual(model["railPartsVerified"], 16)
        for rows in model["caps"].values():
            for side in (0, 1):
                by = {r["strokeMm"]: r for r in rows if r["side"] == side}
                self.assertLess(by[21.25]["issuedCommonMm3"], 1e-6)
                self.assertGreater(by[21.375]["issuedCommonMm3"], 0.05)
                self.assertGreater(by[24]["issuedCommonMm3"], 20)
                self.assertLess(
                    max(r["retracted2p5CommonMm3"] for r in by.values()), 1e-6
                )

    def test_pipe_has_independent_compression_and_lift_detector(self):
        suite = read("mechanics.json")
        for sign in ("plus", "minus"):
            matched = suite[f"{sign}-issued-31.25us"]
            self.assertIsNone(matched["stationToDockMotion"])
            self.assertLess(matched["firstPossibleEdgeState"]["strokeMm"], 21.4)
            for step in ("31.25", "62.5"):
                trial = suite[f"{sign}-free-{step}us-3s-400000"]
                self.assertEqual(trial["stateSha256"], matched["stationStateSha256"])
                self.assertEqual(trial["events"], [])
                self.assertGreater(trial["minNormalN"], 0)
        kick = suite["plus-free-31.25us-3s-400000-kick-0.25"]
        self.assertEqual(
            [e["event"] for e in kick["events"][:2]],
            ["all-stations-clear", "first-recontact"],
        )
        self.assertGreater(kick["peakCompleteSeparationMm"], 3)

    def test_both_wall_conditions_use_same_dock(self):
        data = read("native.json")
        for key, rows in data["rails"].items():
            full = {
                r["setbackMm"]: r
                for r in rows
                if r["strokeMm"] == 370 and r["model"] == "loaded"
            }
            self.assertTrue(all(v > 50 for v in full[0]["commonMm3"]))
            self.assertTrue(all(g > 3.2 for g in full[42]["gapMm"]))
            release = next(
                x
                for x in data["release"][key + "-loaded"]["cases"]
                if x["setbackMm"] == 42
            )
            self.assertEqual(release["overflight"], "clear")
            self.assertGreater(release["farFaceMarginMm"], 2.5)
        for piece in ("coral-A", "coral-B"):
            short = next(
                x
                for x in data["release"][piece]["cases"]
                if x["strokeMm"] == 21.25 and x["setbackMm"] == -60
            )
            self.assertEqual(short["overflight"], "falls short")

    def test_candidate_mass_and_cap_sections(self):
        part = read("cam.json")
        digest = hashlib.sha256((LAB / "cam-lands.step").read_bytes()).hexdigest()
        self.assertEqual(part["candidateStepSha256"], digest)
        self.assertAlmostEqual(
            part["upperAddedMassKgIgnoringRemovedStock"], 0.0445261, 6
        )
        self.assertEqual(part["topLayerChangedMm3"], 0)
        self.assertLess(
            max(abs(v) for v in part["originalStowLandVolumeLossMm3"]), 0.002
        )
        for row in part["capChecks"]:
            self.assertLess(row["belowFloorCommonMm3"], 1e-5)
            if row["strokeMm"] <= 115:
                self.assertGreater(row["aboveFloorCommonMm3"], 0)

    def test_native_fusion_candidate_reopens_with_its_editable_features(self):
        fusion = read("fusion.json")
        self.assertEqual(fusion["errors"], [])
        self.assertEqual(
            fusion["archiveSha256"],
            hashlib.sha256((LAB / "cam-lands.f3d").read_bytes()).hexdigest(),
        )
        self.assertEqual(fusion["stepSha256"], read("cam.json")["candidateStepSha256"])
        self.assertEqual(fusion["reopenedLaneWidthMm"], 10)
        self.assertEqual(fusion["nativeFeatures"], fusion["reopened"])
        self.assertEqual(fusion["reopened"]["r6-receiver-insert"]["extrudes"], 4)
        self.assertEqual(fusion["reopened"]["r6-polymer-facing"]["extrudes"], 2)
        for key, part in fusion["originalR4Appearance"].items():
            self.assertEqual(part["rgb"], "#E5AE4C")
            self.assertEqual(part["role"], "shelf")
            self.assertAlmostEqual(
                part["volumeMm3"], fusion["reopened"][key]["volumeMm3"], places=3
            )

    def test_revised_pickup_two_way_motion_and_free_pipe(self):
        trials = read("candidate.json")
        for sign in ("plus", "minus"):
            prefix = "cam-station-" + sign
            for step in ("31.25", "15.625"):
                tag = f"{prefix}-{step}us-0.1mm"
                self.assertEqual(trials["feeds"][tag]["state"], "IN_BAND")
                self.assertEqual(
                    trials["feeds"][tag]["pipeStateSha256"],
                    trials["service"][tag]["incoming"],
                )
                self.assertEqual(trials["service"][tag]["wallS"], 27.035)
            self.assertEqual(
                trials["feeds"][prefix + "-31.25us-0.2mm"]["state"], "IN_BAND"
            )
            self.assertEqual(
                trials["feeds"][prefix + "-31.25us-2.6mm"]["loss"], "outside-flat-shelf"
            )
            for step in ("31.25", "62.5"):
                cycle = trials["cycles"][f"{sign}-cam-cycle-{step}us"]
                self.assertFalse(cycle["issuedFixture"])
                self.assertEqual(
                    cycle["stationStateSha256"],
                    trials["feeds"][prefix + "-31.25us-0.1mm"]["pipeStateSha256"],
                )
                self.assertEqual(
                    (cycle["fullStrokeMm"], cycle["lastStrokeMm"]), (370, 0)
                )
                self.assertEqual(cycle["possibleSharpEdgeInterferenceMm"], [0, 0])
                self.assertEqual(cycle["maxButtonForceBeyondRunnerN"], 0)
                free = trials["freePipe"][f"{sign}-free-{step}us-3s-400000-cam"]
                self.assertEqual(
                    free["stateSha256"],
                    trials["service"][prefix + "-31.25us-0.1mm"]["outgoing"],
                )
                self.assertGreater(free["minNormalN"], 0)
                self.assertGreater(free["peakCapForceN"], 1)
            for sector in trials["dock"][sign]["cases"].values():
                docks = {r["setbackMm"]: r for r in sector["setbackSweep"]}
                self.assertGreater(docks[54]["railGapLowerMm"], 14.9)
                self.assertGreater(docks[54]["flightGapLowerMm"], 16.3)
                self.assertLess(docks[80]["flightGapLowerMm"], 0)

    def test_post_release_native_trough_contacts_and_adverse_riser(self):
        landing = read("landing.json")
        self.assertIsNone(landing["matchCredit"])
        self.assertEqual(landing["addedSetbackMm"], 54)
        self.assertEqual(len(landing["cases"]), 60)
        for row in landing["cases"]:
            if row["case"].endswith("/excess-speed"):
                self.assertEqual(row["outcome"], "inner-riser-separation-unproven")
            else:
                self.assertEqual(row["outcome"], "supported")
                self.assertGreater(row["final_dwell_s"], 2.29)
                self.assertGreater(row["min_front_m"], 0.0155)


if __name__ == "__main__":
    unittest.main()
