import hashlib
import json
import unittest
from pathlib import Path

BASE = Path(__file__).resolve().parents[1] / "split-guide-acquisition"


def read(name):
    return json.loads((BASE / name).read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class SplitGuideAcquisition(unittest.TestCase):
    def test_undisturbed_and_independent_restart_clocks(self):
        data = read("candidate.json")
        basis = BASE.parent / "compliant-carriage/model.py"
        self.assertEqual(sha(basis), data["sourceBasis"])
        self.assertEqual(len(data["cases"]), 4)
        runs = read("restart.json")["cases"]
        exposures = {5372, 5389, 5406, 5423, 5440, 5457, 5460}
        for r in data["cases"]:
            self.assertEqual(r["state"], "IN_BAND")
            self.assertFalse(r["physicalCaptureQualified"])
            self.assertEqual(set(map(int, r["exposures"])), exposures)
            e = r["terminal"]
            self.assertEqual(
                (e["javaMs"], e["javaState"], e["javaCommandMps"], e["actualBeltMps"]),
                (5460, "IN_BAND", 0, 0),
            )
            self.assertEqual(
                (
                    len(e["z"]),
                    len(e["zdot"]),
                    len(e["landsM"]),
                    len(e["guideStagesM"]),
                    len(e["plungerM"]),
                ),
                (21, 21, 4, 4, 2),
            )
            self.assertTrue(all(g["normalN"] > 0 for g in e["grips"]))
            step = next(
                s for s in runs if s["stepS"] == r["stepS"] and s["side"] == r["side"]
            )
            self.assertLess(max(step["motionReplayMaxAbsByField"].values()), 1e-6)
            self.assertAlmostEqual(step["secondsToNextWheelService"], 0.0005, places=6)
            for ms, frame in r["exposures"].items():
                self.assertLessEqual(frame["javaHeldFromMs"], int(ms))
                self.assertLess(int(ms) - frame["javaHeldFromMs"], 20)

    def test_native_channel_frame_at_every_exposure(self):
        frame = read("guide-bounds.json")
        for key, pin in frame["sourceSha256"].items():
            self.assertEqual(sha(BASE.parent / key), pin)
        self.assertEqual(len(frame["cases"]), 4)
        for row in frame["cases"]:
            self.assertEqual(len(row["exposures"]), 7)
            for channels in row["exposures"].values():
                self.assertEqual(len(channels), 6)
                for name, c in channels.items():
                    self.assertIn("slide-", name)
                    lo, hi = c["zOffsetMm"]
                    self.assertLessEqual(lo, hi)
                    self.assertAlmostEqual(
                        c["conservativeModelZRangeMm"][0], c["unloadedBoundsMm"][2] + lo
                    )
                    self.assertAlmostEqual(
                        c["conservativeModelZRangeMm"][1], c["unloadedBoundsMm"][5] + hi
                    )

    def test_real_disturbances_inhibit_and_continue(self):
        post = read("postfault.json")
        for filename, reason in (
            ("locked.json", "probe-timeout"),
            ("traction.json", "lost-probe-seat"),
        ):
            for r in read(filename)["cases"]:
                self.assertEqual(r["side"], "leftSmall")
                self.assertEqual(r["state"], "FAULT")
                self.assertEqual(r["portTicks"][-1][-1], 0)
                self.assertIsNone(r["firstBandJavaMs"])
                p = next(
                    q
                    for q in post["cases"]
                    if q["disturbance"] == r["id"] and q["stepS"] == r["stepS"]
                )
                self.assertFalse(p["qualifiedForTravel"])
                self.assertGreater(p["observedAfterFaultMs"], 199.0)
                self.assertEqual(p["terminal"]["actualBeltMps"], 0)
        negatives = read("signals.json")["cases"]
        self.assertEqual(negatives["one-dark-flicker"]["positiveProbeTickCount"], 0)
        self.assertFalse(
            negatives["empty-with-replayed-good-bits"]["independentCustody"]
        )
        for key, row in negatives.items():
            if "firstFault" in row:
                self.assertEqual(
                    (row["firstFault"]["state"], row["firstFault"]["command"]),
                    ("FAULT", 0.0),
                )

    def test_replayed_history_preserves_original_right_hand_path(self):
        data = read("comparison.json")
        for dt in ("31.25", "15.625"):
            old = data["issued/leftSmall/" + dt]
            self.assertEqual((old["controller"], old["lastTick"]), ("FAULT", 1020))
            right = data["candidate/rightSmall/" + dt]
            self.assertIsNone(right["firstChangedPhysicalSampleMs"])
            left = data["candidate/leftSmall/" + dt]
            self.assertGreater(left["firstChangedPhysicalSampleMs"], 580)
            self.assertLess(left["firstChangedPhysicalSampleMs"], 581)
        java = read("java-replay.json")["cases"]
        self.assertEqual(len(java), 8)
        for run in java:
            self.assertEqual(run["lastOutput"]["command"], 0)
        self.assertTrue((BASE / "timing.html").is_file())
