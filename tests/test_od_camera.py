"""Cross-check the OD receiver's geometry, lighting, clearance and independent power budget."""
import hashlib
import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CASE = ROOT / "od-camera"

def read(name):
    return json.loads((CASE / (name + ".json")).read_bytes())

class OutsideDiameterCameraTests(unittest.TestCase):
    def test_sources_and_native_identity(self):
        register = read("register")
        for category, location in [("source", ROOT), ("evidence", CASE)]:
            for name, expected in register[category].items():
                with self.subTest(name=name):
                    self.assertEqual(hashlib.sha256((location / name).read_bytes()).hexdigest(), expected)
        native = read("native")
        names = [set(native[key]) for key in ("identicalAtPlacement", "retired", "relocated", "added")]
        self.assertEqual(len(set.union(*names)), sum(map(len, names)))
        self.assertEqual([len(n) for n in names], [468, 122, 3, 147])
        self.assertIn("receiver-R6-drilled-for-camera", names[1])
        self.assertIn("roof-camera-envelope", names[2])
        self.assertEqual(native["old"] - len(names[1]) + len(names[3]), native["new"])
        self.assertEqual(native["new"], 618)
        for name in ["ranges", "travel", "departure", "station"]:
            self.assertEqual(read(name)["bridgeSha"], native["nativeSha"])
        self.assertEqual(read("images")["nativeSha"], native["nativeSha"])

    def test_physical_approach_and_mount_controls(self):
        self.assertEqual(len(read("unmodified")["geometryVsRoof"]["0"]), 16)
        self.assertEqual(len(read("unmodified")["geometryVsRoof"]["112"]), 35)
        self.assertEqual(len(read("foot-width-control")["unknownOrTouching"]), 2)
        native = read("native")
        self.assertFalse(any(native["onGate"].values()) or any(native["on593Unchanged"].values()) or native["withinBridge"])
        travel = read("travel")
        self.assertFalse(travel["unresolved"] or travel["intersections"])
        self.assertGreater(travel["worstRefined"][0], 1.24)
        station = read("station")
        self.assertGreater(station["approach"]["gap_lower_bound_mm"], 8.33)
        self.assertTrue(station["nativeAdverseControl"]["intersections_mm3"])
        occupied = read("occupied")
        self.assertEqual(occupied["reefBalls"], 12)
        self.assertEqual(len(occupied["samples"]), 12)
        cases = occupied["samples"]
        self.assertGreater(cases["long/-0.18/open"]["minimumMm"], 19)
        self.assertTrue(cases["long/-0.18/forward-error"]["intersectionsMm3"])
        self.assertGreater(cases["long/0.22/forward-error"]["minimumMm"], 25)

    def test_hood_and_camera_disagreement_at_exit(self):
        for row in read("ranges")["samples"]:
            for side in ("L", "R"):
                self.assertTrue(all(name == "no-return" for _, name in row[side]["empty"]))
                shots = row[side]["filled"]
                self.assertEqual(len(shots), 9)
                if row["stage"] == "late-release":
                    self.assertTrue(all(name == "no-return" for _, name in shots))
                else:
                    self.assertTrue(all(2 < distance < 24 and name == "finite-wall" for distance, name in shots))
        released = read("departure")["cases"]
        self.assertLess(released["center"]["native"]["firstHeldOneMmClearS"], 3.69)
        long = released["long"]
        self.assertGreater(long["native"]["firstHeldOneMmClearS"], long["lastPlaneOverOneMmS"])
        self.assertGreater(long["native"]["firstHeldOneMmClearS"], 4.10)
        self.assertGreater(long["native"]["terminalMinimumMm"], 3.6)
        self.assertEqual(read("images")["installedReefBalls"], 12)
        pictures = read("images")["cases"]
        self.assertEqual(len(pictures), 28)
        for case in ("center", "long"):
            terminal = pictures[case + "-terminal-roof"]
            self.assertTrue(all(eye["pipePixels"] > 44000 and not eye["idealClipped"] for eye in terminal["fov"]))
            self.assertTrue(all(min(pair) > 0 for pair in terminal["twoPixelDiscriminators"].values()))
            self.assertTrue(all(eye["idealClipped"] for eye in pictures[case + "-seat-roof"]["fov"]))
        old = read("rearward-aim-control")["selectedWitnesses"]
        self.assertEqual(old["center-terminal-roof"]["fov"][0]["pipePixels"], 0)

    def test_conservative_and_correlated_outcomes(self):
        b = read("budget")
        m = b["mass"]
        retrofit = m["uninstalledThreeConverterStudy"]
        self.assertTrue(0 < m["remainingConservativeR103Kg"] < .003)
        self.assertLess(retrofit["ODRemainingConservativeR103Kg"], -.74)
        self.assertGreater(retrofit["endBeltRemainingConservativeR103Kg"], 1.06)
        self.assertGreater(m["remainingCorrelatedR103Kg"] - retrofit["grossKg"], .05)
        self.assertLess(retrofit["ODEvenWithoutAnyMovingCameraKg"], 0)
        c = b["threeIndependentConverters"]
        self.assertEqual(c["inputSourceSha256"], read("register")["source"]["fixed-low-power/dual-camera.json"])
        for eff in ("efficiency85", "efficiency75"):
            def margin(scenario):
                return c[eff][scenario]["11.8V-27mOhm"]["worstMarginV"]
            self.assertGreater(margin("modeled-parked-transfer"), .95)
            self.assertLess(margin("carry-hood-hold"), -.07)
            self.assertLess(margin("coincident-drive-and-transfer"), -.91)
