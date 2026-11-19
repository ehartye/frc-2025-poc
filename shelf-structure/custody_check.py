"""Read fixed stereo at the two attained stations after the coupled port trial."""

import argparse
import json
import math
from pathlib import Path
import sys
import numpy as np

from tray import ROOT, digest, cad
from installed import STATIONS, GAP
from damper_fit import envelope

sys.path.insert(0, str(ROOT / "shelf-contact"))
from custody import field_mesh, scene, silhouette, read_pair, camera

sys.path.insert(0, str(ROOT / "fixed-camera-route"))
from visibility import depths
from paths import F, U, L, O


def guard(robot, offset):
    # Bound every changed shelf ray by an entire displaced 265 x 420 x 15 mm
    # envelope, including all three laminates and residual settlement allowance.
    boxes = [[-100, -210, 485 - offset, 165, 210, 500.8 + offset]]
    for x, y in STATIONS:
        boxes.append(list(cad.bounds(envelope(x, y, 485 - GAP))))
        boxes.append([x - 5.78, y - 5.78, 430, x + 5.78, y + 5.78, 441])
    out = []
    masks = []
    for side, (grey, z) in zip(camera.EYES, robot):
        near = depths(boxes, O + side * L, F, U)
        invalid = (near <= z + 0.2) & np.isfinite(near)
        masks.append(invalid)
        out.append((np.where(invalid, 0, grey).astype(np.uint8), np.minimum(z, near)))
    return out, masks


def run(root, field, rows):
    src = json.loads((ROOT / "station-guide/background.json").read_bytes())
    robot_path = root / "station-guide/selected-background/stowed-robot-fixed.npz"
    if digest(robot_path) != src["backgroundSha256"]["stowed-robot-fixed"]:
        raise ValueError("Different fixed head register")
    if digest(field) != src["sourceSha256"]["registered field STEP"]:
        raise ValueError("Different field")
    saved = np.load(robot_path)
    robot = [(saved[k], saved[k + "_z"]) for k in ("left", "right")]
    # A completely opaque, unknown backing over the shelf and both plungers
    # excludes visual credit from machining and from the pressure-tail recesses.
    masked, masks = guard(robot, 2.0)
    parts = field_mesh(field)
    result = []
    for row in json.loads(rows.read_bytes()):
        if row["stepS"] != 0.00003125:
            continue
        out = {"case": row["case"], "portState": row["lastState"], "frames": []}
        result.append(out)
        if row["lastState"] != "IN_BAND" or row["lastJavaMs"] != 5460:
            continue
        trace = root / "shelf-structure/coupled" / (row["id"] + ".npy")
        if digest(trace) != row["trajectorySha256"]:
            raise ValueError("Different pipe trajectory")
        states = np.load(trace)
        bg = scene(parts, row["sourceRobotMmDeg"], masked)
        empty = [g for g, _ in bg]
        # Match the pipe intensity on every novel visible background ray.
        # The nominal contrast comparison uses a dark finish on those same rays.
        same_color = [
            (np.where(mask & (z == robot_z), 220, g).astype(np.uint8), z)
            for (g, z), (_, robot_z), mask in zip(bg, masked, masks)
        ]
        same_empty = [g for g, _ in same_color]
        out["trajectorySha256"] = digest(trace)
        for t in (5372, 5389, 5406, 5423, 5440):
            xy = np.array([np.interp(t, states[:, 0], col) for col in states[:, 1:7].T])
            axis = xy[3:] / np.linalg.norm(xy[3:])
            pair = silhouette(bg, xy[:3], axis)
            fit = read_pair(pair, empty, 560, 0)
            same = read_pair(silhouette(same_color, xy[:3], axis), same_empty, 560, 0)
            fit["pipeColorOnChangedRays"] = same["status"]
            fit["oneEyeBlank"] = read_pair([empty[0], pair[1]], empty, 560, 0)["status"]
            out["frames"].append(
                {"javaMs": t, "first5msTickMs": 5 * math.ceil((t + 15) / 5), **fit}
            )
        out["blankControl"] = read_pair(empty, empty, 560, 0)["status"]
    return {
        "sourceFieldSha256": digest(field),
        "fixedBackgroundSha256": digest(robot_path),
        "guardedPixels": [int(m.sum()) for m in masks],
        "wholeShelfVerticalAllowanceMm": 2.0,
        "nominalPupilOnly": True,
        "cases": result,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for k in ("artifacts", "field", "rows", "out"):
        p.add_argument(k, type=Path)
    a = p.parse_args()
    a.out.write_text(
        json.dumps(run(a.artifacts, a.field, a.rows), indent=2) + "\n",
        encoding="utf8",
        newline="\n",
    )
