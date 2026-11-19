import hashlib
import json
from pathlib import Path
import sys
import subprocess
import unittest

ROOT=Path(__file__).resolve().parents[1]
HERE=ROOT/"shelf-contact"


def data(name):
    return json.loads((HERE/name).read_bytes())


class ShelfContactTests(unittest.TestCase):
    def test_vertical_impact_and_finite_support_are_independent_oracles(self):
        probe = "from contact import analytic_impact, Shelf; import json; print(json.dumps(analytic_impact(2.95, Shelf())))"
        # Legacy case studies use overlapping flat module names. Exercise the normal study entry point in isolation.
        output = subprocess.check_output([sys.executable, "-c", probe], cwd=HERE, text=True)
        depth, rebound = json.loads(output)
        self.assertAlmostEqual(depth*1000,1.536307,places=5)
        self.assertAlmostEqual(rebound,.2499912,places=6)
        rows=data("analytical.json")
        for row in rows["symmetricGravityFreeDrops"]:
            self.assertLess(row["angularImpulseNms"],1.e-12)
            if row["dt"]==.000015625:
                self.assertLess(abs(row["referenceMaxCompressionMm"]-row["modelMaxCompressionMm"]),.013)
                self.assertLess(abs(row["referenceReboundMps"]-row["modelReboundMps"]),.0001)
        controls=rows["static"]
        self.assertEqual(controls[0]["reactionN"],0)
        self.assertEqual(controls[3]["reactionN"],0)
        self.assertAlmostEqual(controls[1]["reactionN"],400)
        self.assertAlmostEqual(controls[2]["reactionN"],200)
        self.assertAlmostEqual(controls[2]["torqueNm"][0],-15.08125)

    def test_fine_range_resolves_limits_and_retains_coarse_counterexamples(self):
        coarse=data("review.json")["cases"]
        fine=data("fine-review.json")["cases"]
        self.assertEqual(set(fine),{"leftSmall","rightSmall","nominal"})
        for name,row in fine.items():
            self.assertEqual((row["status"],row["exitCode"],row["acceptance"]),("within-declared-bounds",0,"blocked"))
            self.assertEqual(len(row["axes"]),5)
            self.assertTrue(all(axis["status"]=="within-declared-bounds" for axis in row["axes"].values()))
            self.assertLess(max(entry["cartridgeMm"] for entry in row["runs"].values()),1.4)
            self.assertEqual(coarse[name]["status"],"unknown")
            self.assertGreater(coarse[name]["runs"]["dt-1"]["peakShelfN"],6000)
        ref=json.loads((ROOT/"station-guide/review.json").read_bytes())
        self.assertTrue(all(ref["cases"][n]["runs"][3]["firstStop"] for n in ("leftSmall","rightSmall")))

    def test_stationary_custody_and_adverse_materials_use_the_source_state(self):
        runs=data("ports.json")["results"]
        good=[r for r in runs if r["shelf"]["stiffness_n_m"]==400000 and r["shelf"]["dissipation_s_m"]==4]
        self.assertEqual(len(good),15)
        self.assertTrue(all(r["state"]=="IN_BAND" and r["lastTickMs"]==5460 for r in good))
        self.assertTrue(all(r["loss"]=="outside-flat-shelf" for r in runs if r["shelf"]["dissipation_s_m"]==0))
        self.assertTrue(all(r["state"]=="FAULT" for r in runs if r["shelf"]["dissipation_s_m"]==1))
        poses=data("custody.json")
        self.assertEqual(poses["guideSha256"],hashlib.sha256((ROOT/"station-guide/narrow-face.step").read_bytes()).hexdigest())
        for name,r in poses["runs"].items():
            self.assertEqual(r["trajectorySha256"],next(v["trajectorySha256"] for v in good if v["case"]==name and v["dtS"]==.0000625))
            self.assertNotEqual(r["emptyControl"],"position-witness")
            self.assertEqual(len(r["frames"]),5)
            for f in r["frames"]:
                self.assertEqual(f["status"],"position-witness")
                self.assertLessEqual(f["first5msTickMs"],poses["lastPermittedTickMs"])
                x,y,z=f["sourceCenterMm"]
                for value,key in ((x,"u_interval_mm"),(y,"y_interval_mm"),(z-557.15,"bottom_relative_to_shelf_mm")):
                    self.assertLessEqual(f[key][0],value)
                    self.assertGreaterEqual(f[key][1],value)

    def test_robot_load_sign_duration_and_location(self):
        for r in data("load.json"):
            self.assertLess(r["peakRobotReactionXYZn"][2],-3300)
            self.assertTrue(2<1000*(r["endFromLipS"]-r["startFromLipS"])<2.3)
            self.assertTrue(-2.7<r["pulseImpulseRobotXYZns"][2]<-2.5)
            self.assertTrue(110<r["peakNormalCenterRobotMm"][0]<120)
            self.assertTrue(498<r["peakNormalCenterRobotMm"][2]<500)
            self.assertTrue(all(len(r[key])==64 for key in ("aggregateSha256","stationsSha256")))

    def test_signed_wheel_cases_retain_a_fine_port_and_independent_late_image(self):
        fine = data("fine-ports.json")["results"]
        self.assertEqual(len(fine), 6)
        for run in fine:
            self.assertEqual((run["guideAngleSectors"], run["shelf"]["stations"], run["beltStrips"]), (128, 16, 10))
            self.assertEqual((run["state"], run["lastTickMs"], run["stop"], run["loss"]), ("IN_BAND", 5460, None, None))
        poses = data("fine-custody.json")
        for name, pair in poses["runs"].items():
            source = next(run for run in fine if run["case"] == name and run["dtS"] == .000015625)
            self.assertEqual(pair["trajectorySha256"], source["trajectorySha256"])
            self.assertNotEqual(pair["emptyControl"], "position-witness")
            self.assertEqual([f["timeMs"] for f in pair["frames"]], [5372, 5389, 5406, 5423, 5440])
            for frame in pair["frames"]:
                self.assertEqual(frame["status"], "position-witness")
                self.assertLessEqual(frame["first5msTickMs"], 5460)
                x, y, z = frame["sourceCenterMm"]
                for value, key in ((x, "u_interval_mm"), (y, "y_interval_mm"), (z - 557.15, "bottom_relative_to_shelf_mm")):
                    self.assertLessEqual(frame[key][0], value)
                    self.assertGreaterEqual(frame[key][1], value)

    def test_lower_effective_stiffness_reduces_peak_and_retains_signed_impulse(self):
        low = data("soft-load.json")
        full = data("soft-ports.json")["results"]
        self.assertEqual(len(low), 6)
        self.assertEqual(len(full), 6)
        for side in ("leftSmall", "rightSmall"):
            group = sorted((r for r in low if r["case"] == side), key=lambda r: r["stiffnessNm"])
            self.assertEqual([r["stiffnessNm"] for r in group], [20000, 40000, 80000])
            force = [-r["peakRobotReactionXYZn"][2] for r in group]
            self.assertTrue(745 < force[0] < 760 and 1050 < force[1] < 1080 and 1490 < force[2] < 1530)
            self.assertTrue(group[0]["peakApproachMm"] > group[1]["peakApproachMm"] > group[2]["peakApproachMm"])
            self.assertTrue(all(-2.63 < r["pulseImpulseRobotXYZns"][2] < -2.58 for r in group))
        self.assertTrue(all(r["state"] == "IN_BAND" and r["lastTickMs"] == 5460 for r in full))
