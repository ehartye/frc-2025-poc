"""Regression of the issued R4 camera receipts and single-article mass change."""

import hashlib
import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
EXP = ROOT / "experiments/swerve-drive"
STUDY = EXP / "receiver-contrast"


def read(path):
    return json.loads(path.read_bytes())


class ReceiverContrast(unittest.TestCase):
    def test_original_r4_images_reproduce_and_both_camera_heads_see_the_facing(self):
        source = EXP / "motion/receiver-witness.json"
        new = read(STUDY / "replay.json")
        self.assertEqual(
            new["sourceObservationSha256"],
            [hashlib.sha256(source.read_bytes()).hexdigest()],
        )
        original = read(source)
        self.assertEqual(new["surfaces"]["robotSha256"], original["stepSha256"])
        self.assertEqual(new["surfaces"]["fieldSha256"], original["fieldSha256"])
        for row in new["cases"]:
            old = next(
                c
                for c in original["cases"]
                if (c["case"], c["stepUs"], c["finish"])
                == (row["case"], row["stepUs"], "matched")
            )
            self.assertEqual(row["trajectorySha256"], old["trace"])
            self.assertEqual(
                [
                    f["fourFramesSha256"]
                    for f in row["finishes"]["issuedMatched"]["frames"]
                ],
                [f["fourFramesSha256"] for f in old["frames"]],
            )
            source = new["surfaces"]["docks"][row["case"]]
            for a, b in zip(
                source["fixedAllPixels"] + source["roofAllPixels"],
                source["fixedTopPixels"] + source["visibleTopPixels"],
            ):
                self.assertGreaterEqual(a, b)
                self.assertGreater(b, 20000)

    def test_artifact_reports_observability_and_timing_separately(self):
        for filename in ("replay.json", "reciprocal-drive.json"):
            result = read(STUDY / filename)
            cases = result["cases"]
            self.assertEqual({c["case"] for c in cases}, {"plus", "minus"})
            for row in cases:
                g = row["finishes"]
                for label in (
                    "face",
                    "topOnly",
                    "delta37",
                    "delta36",
                    "issuedMatched",
                    "matched",
                ):
                    with self.subTest(
                        filename=filename,
                        step=row["stepUs"],
                        case=row["case"],
                        finish=label,
                    ):
                        allow = label in {"face", "topOnly", "delta37"}
                        self.assertEqual(
                            [f["accepted"] for f in g[label]["frames"]], [allow] * 6
                        )
                        self.assertEqual(g[label]["gate15ms"]["accepted"], allow)
                        self.assertFalse(g[label]["gate45ms"]["accepted"])
                        if allow:
                            self.assertEqual(
                                g[label]["gate15ms"]["eligibleCapturesJavaMs"],
                                [5372, 5389, 5406, 5423, 5440],
                            )
                            self.assertTrue(
                                all(f["yawIntervalRad"] for f in g[label]["frames"])
                            )
                        else:
                            self.assertEqual(
                                {f["roof"] for f in g[label]["frames"]},
                                {"missing-rear-edge"},
                            )
                self.assertEqual(
                    set(row["controls"]),
                    {
                        "empty",
                        "off-center",
                        "yaw-7-deg",
                        "fixed-left-drop",
                        "roof-right-drop",
                        "opaque-guard",
                    },
                )
                self.assertFalse(any(f["accepted"] for f in row["controls"].values()))

    def test_all_azimuth_and_full_scanline_guard_outcomes(self):
        revised = read(STUDY / "reciprocal-drive.json")
        masked = read(STUDY / "all-azimuth.json")
        self.assertEqual(
            masked["sourceObservationsSha256"], revised["sourceObservationSha256"]
        )
        self.assertEqual(
            masked["wheelMaskRegisterSha256"],
            "d1d9d052bd54f20a6273e227b59a4bf29f0ebdb36e41d1f39ab379bfe84cd0d7",
        )
        self.assertEqual(len(masked["cases"]), 4)
        for row in masked["cases"]:
            for label, result in row["treatments"].items():
                self.assertEqual(result["gate15ms"]["accepted"], label == "bulk-black")
                self.assertFalse(result["gate45ms"]["accepted"])
                self.assertEqual(
                    [f["accepted"] for f in result["frames"]],
                    [label == "bulk-black"] * 6,
                )

    def test_stock_replacement_is_booked_once_with_adverse_allowance(self):
        spec = importlib.util.spec_from_file_location(
            "receiver_contrast_material", STUDY / "material.py"
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        charge = module.account()
        self.assertEqual(
            charge["candidateBodyKg"],
            [charge["nativeVolumeMm3"] * 1e-9 * r for r in (950, 960)],
        )
        self.assertGreater(charge["headroomUnderExtraIndependentChargeKg"], 0.149)
        self.assertLess(charge["headroomUnderExtraIndependentChargeKg"], 0.151)
        self.assertIsNone(charge["scaleMassKg"])
