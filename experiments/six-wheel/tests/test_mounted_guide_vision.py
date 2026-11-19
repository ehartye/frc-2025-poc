import importlib.util
import json
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parents[1] / "mounted-guide-vision"


def rows(name):
    return json.loads((HERE / f"{name}.json").read_bytes())


class MountedGuideVision(unittest.TestCase):
    def test_real_installations_and_setting_revision(self):
        original = rows("sector")
        revised = rows("setting-sector")
        self.assertEqual(len(original), 24)
        self.assertEqual(len(revised), 24)

        def key(row):
            return (
                row["installation"].removeprefix("set-"),
                row["wheelCase"],
                row["mode"],
            )

        self.assertEqual(
            {key(r): r["motionSHA256"] for r in original},
            {key(r): r["motionSHA256"] for r in revised},
        )
        self.assertFalse(any(r["accepted"] for r in original + revised))
        for name in ("station", "setting-station"):
            for entry in rows(name).values():
                evidence = entry["register"]
                self.assertEqual(
                    (
                        evidence["reopenedParts"],
                        evidence["addedToRaster"],
                        evidence["oldOpaqueMastRetained"],
                    ),
                    (58, 56, 2),
                )
                self.assertEqual(
                    bool(evidence["settingStepSha256"]), name == "setting-station"
                )

    def test_physical_removal_controls(self):
        allowed = [
            (r["installation"], r["wheelCase"])
            for r in rows("ablation-sector")
            if r["accepted"]
        ]
        self.assertCountEqual(
            allowed,
            [
                ("omit-lower-left-arm", "leftSmall"),
                ("omit-lower-left-arm", "rightSmall"),
                ("omit-both", "rightSmall"),
            ],
        )

    def test_actual_native_occluders(self):
        for entry in rows("obstruction"):
            if entry["installation"] != "nominal" or entry["path"] != "new-actual":
                continue
            lost = {k: v for k, v in entry["frames"].items() if v["newlyOccluded"]}
            self.assertEqual(len(lost), 1)
            key, evidence = next(iter(lost.items()))
            self.assertTrue(key.startswith("left/NAUO1192/"))
            self.assertEqual(
                evidence["newlyOccluded"],
                52 if entry["wheelCase"] == "leftSmall" else 46,
            )

    def test_negative_images(self):
        negatives = [r for r in rows("controls") if r["challenge"] != "acquired"]
        self.assertEqual(len(negatives), 40)
        self.assertTrue(all(not r["admitted"] for r in negatives))

    def test_handoff_uses_only_delivered_frames(self):
        evidence = rows("handoff")
        for trials in evidence.values():
            for result in trials:
                self.assertEqual(result["lastAppliedMs"], 5455)
                self.assertEqual(result["nextFrameAppliedMs"], 5475)
                if result["guard"] == "matched-finish":
                    self.assertFalse(result["accepted"])
        joined = [
            r
            for r in evidence["nominal"]
            if r["guard"] == "guarded"
            and r["source"].endswith("serial-carriage-recover")
            and "31.25us" in r["source"]
        ]
        self.assertEqual(
            {r["wheelCase"]: r["accepted"] for r in joined},
            {"leftSmall": True, "rightSmall": False},
        )

    def test_missing_capture_and_invalid_yaw_inhibit(self):
        spec = importlib.util.spec_from_file_location(
            "mounted_guide_handoff", HERE / "handoff.py"
        )
        helper = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(helper)
        case = next(
            c
            for c in rows("station")["nominal"]["cases"]
            if c["source"].endswith("serial-carriage-recover")
            and "31.25us" in c["source"]
            and c["wheelCase"] == "leftSmall"
        )
        self.assertTrue(helper.handoff(case, "guarded")["accepted"])
        guarded = next(row for row in case["boundedGaps"] if row[1] == "guarded")
        guarded[-1] = [-0.15, 0.15]
        self.assertFalse(helper.handoff(case, "guarded")["accepted"])
        case["fixed"].pop()
        self.assertLessEqual(helper.handoff(case, "guarded")["observationsPassing"], 3)
