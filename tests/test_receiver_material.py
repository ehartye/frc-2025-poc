"""Check that the selected contact and camera data describe the same physical pipe."""

import importlib.util
import json
from pathlib import Path
import unittest

import numpy as np

HERE = (
    Path(__file__).resolve().parents[1] / "experiments/swerve-drive/receiver-material"
)


def load(name):
    spec = importlib.util.spec_from_file_location(
        "material_" + name, HERE / (name + ".py")
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Receivers(unittest.TestCase):
    def test_held_out_gain_and_unmeasured_backgrounds(self):
        opt = load("radiometry")
        image = np.repeat(np.array([[45, 90, 150]], dtype=np.uint8), 96, axis=1)
        image = np.repeat(image, 96, axis=0)
        surveyed = opt.samples(image, np.ones_like(image, dtype=bool))
        dim = opt.encode(image, 0.72, 4)
        restored, check = opt.normalize(dim, surveyed)
        self.assertEqual(check["status"], "validated")
        self.assertLessEqual(abs(restored.astype(int) - image.astype(int)).max(), 1)
        for _, _, held in surveyed:
            dim.ravel()[held] = 240
        self.assertIsNone(opt.normalize(dim, surveyed)[0])
        self.assertIsNone(opt.normalize(image, [])[0])

    def test_projected_rims_obey_distinct_known_end_planes_and_radius(self):
        arc = load("model_arc")

        class Camera:
            H = 480
            W = 640
            F = 360
            R = 57.15
            L = 301.625
            ORIGIN = np.array([48, 0, 842])
            EYES = [37.5, -37.5]

        pose = [76.8, -0.2, 557.15, 0, 0]
        rows = np.array([172.5, 205.5, 245.5, 273.5])
        for eye, side in enumerate((1, -1)):
            col, valid, depth = arc.project(pose, rows, eye, Camera)
            self.assertTrue(valid.all())
            reconstructed = np.stack(
                [
                    48 + depth * (Camera.H / 2 - rows) / Camera.F,
                    Camera.EYES[eye] + depth * (Camera.W / 2 - col) / Camera.F,
                    842 - depth,
                ],
                axis=1,
            )
            np.testing.assert_allclose(
                reconstructed[:, 1], pose[1] + side * Camera.L / 2, atol=1e-5
            )
            np.testing.assert_allclose(
                np.hypot(reconstructed[:, 0] - pose[0], reconstructed[:, 2] - pose[2]),
                Camera.R,
                atol=1e-5,
            )

    def test_published_conditions_and_adverse_camera_controls(self):
        record = json.loads((HERE / "results.json").read_bytes())
        self.assertEqual(len(record["cases"]), 12)
        self.assertEqual(
            sum(c["observations"]["nominal"]["originalPeak"] for c in record["cases"]),
            6,
        )
        signs = {}
        for r in record["cases"]:
            self.assertEqual((r["javaState"], r["javaLastMs"]), ("IN_BAND", 5460))
            for mode in ("nominal", "dim", "bright", "mixed-pupils"):
                self.assertTrue(r["observations"][mode]["projectedRims"])
            key = (r["densityKgM3"], r["shelfFriction"], r["stepUs"])
            signs.setdefault(key, set()).add(r["case"])
            for o in r["observations"].values():
                self.assertFalse(o["projected45ms"])
            for name in ("raw-dim", "matched"):
                self.assertFalse(r["observations"][name]["projectedRims"])
        self.assertTrue(all(s == {"minus", "plus"} for s in signs.values()))
        controls = next(
            c["matchedControls"] for c in record["cases"] if "matchedControls" in c
        )
        for name in (
            "flat-roof",
            "flat-right",
            "rotated",
            "off-center",
            "opaque-guard",
            "empty",
            "saturate",
            "pipe-shadow",
            "uneven-reference",
            "left-drop",
        ):
            self.assertEqual(controls[name], [False, False], name)

    def test_identical_images_at_pickup_cannot_establish_future_traction(self):
        observation = json.loads((HERE / "retention.json").read_bytes())
        pair = observation["firstLease"]
        self.assertEqual(
            pair["nominal"]["chassis"], pair["zeroBeltTraction"]["chassis"]
        )
        self.assertEqual(
            pair["nominal"]["serviceState"], pair["zeroBeltTraction"]["serviceState"]
        )
        self.assertLess(pair["nominal"]["peakTravelMm"], 0.26)
        self.assertGreater(pair["zeroBeltTraction"]["peakTravelMm"], 48)
        self.assertLess(pair["shelfOnlyIdealStaticCapN"], pair["peakInertialDemandN"])
        self.assertGreater(
            min(r["minimumForeAftPadGapMm"] for r in observation["padGaps"]), 16
        )
