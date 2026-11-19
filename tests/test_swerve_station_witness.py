"""Check that optical and mechanical handoffs preserve the signed feed and wall clock."""

import hashlib
import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
MOTION = ROOT / "experiments/swerve-drive/motion"
spec = importlib.util.spec_from_file_location(
    "swerve_station_clock", MOTION / "station_clock.py"
)
clock = importlib.util.module_from_spec(spec)
spec.loader.exec_module(clock)


def read(name):
    return json.loads((MOTION / name).read_bytes())


class StationCustody(unittest.TestCase):
    def test_registered_cohort_and_source_binding(self):
        optics = read("receiver-witness.json")
        service = read("first-service.json")
        feeds = read("first-feed.json")["attempts"]
        self.assertEqual(
            optics["firstFeedSha256"],
            hashlib.sha256((MOTION / "first-feed.json").read_bytes()).hexdigest(),
        )
        self.assertEqual(
            optics["stepSha256"],
            hashlib.sha256(
                (ROOT / "cad/full-swerve-r4/full-robot.step").read_bytes()
            ).hexdigest(),
        )
        self.assertEqual(len(optics["cases"]), 12)
        self.assertEqual(len(optics["controls"]), 20)
        self.assertEqual(len(service), 4)
        for f in feeds:
            key = (f["case"], f["plantStepUs"])
            rows = [r for r in optics["cases"] if (r["case"], r["stepUs"]) == key]
            self.assertEqual(
                {r["finish"] for r in rows}, {"guarded", "nominal", "matched"}
            )
            for r in rows:
                self.assertEqual(
                    (r["piece"], r["trace"], r["terminalStateSha256"]),
                    ("coral-A", f["pipeTrajectorySha256"], f["pipeStateSha256"]),
                )
                self.assertEqual(r["dockMmDeg"], f["trueChassisAtFeedMmMmDeg"])
                self.assertEqual(r["gate"]["accepted"], r["finish"] != "matched")
                self.assertEqual(
                    r["gate"]["accepted"],
                    clock.service_gate(r["frames"], f, 15)["accepted"],
                )
                self.assertFalse(clock.service_gate(r["frames"], f, 45)["accepted"])
            (physical,) = [r for r in service if (r["case"], r["stepUs"]) == key]
            self.assertEqual(physical["incoming"], f["pipeStateSha256"])
            self.assertEqual(
                physical["wallS"], rows[0]["gate"]["firstEligibleWheelServiceWallS"]
            )
            self.assertLess(physical["peakCenterDisplacementMm"], 0.001)
            self.assertIsNone(physical["loss"])
            self.assertIsNone(physical["hardStop"])
            adverse = [c for c in optics["controls"] if (c["case"], c["stepUs"]) == key]
            self.assertEqual(
                {c["challenge"] for c in adverse},
                {
                    "empty",
                    "off-center",
                    "yaw-7-deg",
                    "fixed-left-drop",
                    "roof-right-drop",
                },
            )
            self.assertFalse(any(c["accepted"] for c in adverse))

    def test_all_required_frames_arrive_before_service(self):
        f = read("first-feed.json")["attempts"][0]
        frames = next(
            c["frames"]
            for c in read("receiver-witness.json")["cases"]
            if c["case"] == f["case"] and c["finish"] == "guarded"
        )
        self.assertEqual(
            clock.service_gate(frames, f, 15)["serviceWallS"][5457], 27.045
        )
        self.assertTrue(clock.service_gate(frames, f, 23)["accepted"])
        self.assertFalse(clock.service_gate(frames, f, 24)["accepted"])
        self.assertFalse(clock.service_gate(frames[1:], f, 15)["accepted"])
        with self.assertRaises(ValueError):
            clock.service_gate([frames[0], *frames], f, 15)
        with self.assertRaises(ValueError):
            clock.service_gate(frames, f, -1)
