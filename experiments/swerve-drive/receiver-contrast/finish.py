"""Test a passive finish only on CAD-identified visible robot stock, through the existing estimator."""

import argparse
import json
import math
from pathlib import Path

import numpy as np


from native_scene import inputs, witness

ORIGIN = np.array(
    witness.read(witness.LEGACY / "selected-optics/mount.json")["registrations"][
        "roof"
    ]["assumedRectifiedOriginMm"]
)
CAM = witness.sensor.camera


def select_views(
    path,
    gray,
    *,
    x_span=(-100, 165),
    y_span=(-210, 210),
    guard_pixels=1,
    blocked=False,
    fixed_finish=True,
    whole_part=True,
):
    """Apply the same empty-scene appearance on both render and surveyed backing."""
    witness.sensor.optics.ORIGIN = ORIGIN.copy()
    witness.sensor.optics.configure()
    fixed_key, roof_key = (
        ("fixedAll", "allFaces") if whole_part else ("fixedTarget", "target")
    )
    with np.load(path) as data:
        fixed = [
            (
                np.where(data[f"{fixed_key}{i}"], gray, data[f"fg{i}"]).astype(np.uint8)
                if fixed_finish
                else data[f"fg{i}"],
                data[f"fz{i}"],
            )
            for i in range(2)
        ]
        scene = [(data[f"g{i}"], data[f"z{i}"]) for i in range(2)]
        ref = [(g, data[f"rz{i}"]) for i, (g, _) in enumerate(scene)]
        masks = [data[f"{roof_key}{i}"] for i in range(2)]
    actual, reference = witness.sensor.shelf_guard(scene, ref, ORIGIN[2], True, 0.05)
    counts = []
    for i, (g, depth) in enumerate(actual):
        x = ORIGIN[0] + CAM.Q * scene[i][1]
        y = ORIGIN[1] + CAM.EYES[i] - (CAM.U - CAM.W / 2) / CAM.F * scene[i][1]
        mask = (
            masks[i]
            & (x >= x_span[0])
            & (x <= x_span[1])
            & (y >= y_span[0])
            & (y <= y_span[1])
        )
        for _ in range(guard_pixels):
            padded = np.pad(mask, 1, constant_values=False)
            mask = np.logical_and.reduce(
                [
                    padded[dy : dy + mask.shape[0], dx : dx + mask.shape[1]]
                    for dy in range(3)
                    for dx in range(3)
                ]
            )
        counts.append(int(mask.sum()))
        g = np.where(mask, gray, g).astype(np.uint8)
        known = reference[i][1]
        if blocked:
            # Opaque, nearer obstruction across the full sight area, in actual and empty images.
            obstruction = (x >= -80) & (x <= 160) & (abs(y) <= 200)
            depth = np.where(obstruction, 120.0, depth)
            known = np.where(obstruction, 3500.0, known)
            g = np.where(obstruction, 130, g).astype(np.uint8)
        actual[i] = g, depth
        reference[i] = g, known
    return fixed, (actual, reference), counts


def inspect(views, frame, challenge=None):
    fixed, roof, _ = views
    q = np.array(frame["trueCenterMm"])
    u = np.array(frame["trueAxis"])
    if challenge == "empty":
        q = u = None
    elif challenge == "off-center":
        q = q + [0, 18, 0]
    elif challenge == "yaw-7-deg":
        a = math.radians(7)
        u = np.array([math.sin(a), math.cos(a), 0])
    row = witness.observe(fixed, roof, q, u, challenge)
    return {"captureJavaMs": frame["captureJavaMs"], **row}


def freeze(folder, out, docks=None):
    source = witness.HERE / "receiver-witness.json"
    baseline = witness.read(source)
    feeds, cases, sources = inputs(docks)
    meta = witness.read(folder / "surfaces.json")
    if (
        meta["robotSha256"] != baseline["stepSha256"]
        or meta["fieldSha256"] != baseline["fieldSha256"]
    ):
        raise ValueError("Source robot or field has changed")
    result = []
    for old in cases:
        name = old["case"]
        feed = next(
            f for f in feeds if f["case"] == name and f["plantStepUs"] == old["stepUs"]
        )
        if (
            old["dockMmDeg"] != meta["docks"][name]["dockMmMmDeg"]
            or old["trace"] != feed["pipeTrajectorySha256"]
            or old["dockMmDeg"] != feed["trueChassisAtFeedMmMmDeg"]
        ):
            raise ValueError("Source image or field deployment is for a different dock")
        row = {
            "case": name,
            "stepUs": old["stepUs"],
            "trajectorySha256": old["trace"],
            "finishes": {},
            "controls": {},
        }
        path = folder / (name + ".npz")
        for label, gray in (
            ("issuedMatched", 220),
            ("matched", 220),
            ("face", 80),
            ("topOnly", 80),
            ("delta36", 184),
            ("delta37", 183),
        ):
            views = select_views(
                path,
                gray,
                fixed_finish=label != "issuedMatched",
                whole_part=label not in ("topOnly", "issuedMatched"),
            )
            frames = [inspect(views, frame) for frame in old["frames"]]
            if label == "issuedMatched":
                reference = [
                    f
                    for f in old["frames"]
                    if f["captureJavaMs"] in {r["captureJavaMs"] for r in frames}
                ]
                if any(
                    a["fourFramesSha256"] != b["fourFramesSha256"]
                    for a, b in zip(frames, reference)
                ):
                    raise ValueError("Could not reproduce original matched images")
            row["finishes"][label] = {
                "pixels": views[2],
                "frames": frames,
                "gate15ms": witness.service_gate(frames, feed, 15),
                "gate45ms": witness.service_gate(frames, feed, 45),
            }
            print(name, old["stepUs"], label, [f["roof"] for f in frames], flush=True)
        for challenge in (
            "empty",
            "off-center",
            "yaw-7-deg",
            "fixed-left-drop",
            "roof-right-drop",
            "opaque-guard",
        ):
            views = select_views(path, 80, blocked=challenge == "opaque-guard")
            row["controls"][challenge] = inspect(views, old["frames"][4], challenge)
        result.append(row)
    record = {"surfaces": meta, "sourceObservationSha256": sources, "cases": result}
    out.write_text(
        json.dumps(record, indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("scene", type=Path)
    p.add_argument("result", type=Path)
    p.add_argument("--docks", type=Path)
    a = p.parse_args()
    freeze(a.scene, a.result, a.docks)
