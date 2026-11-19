"""Replay the station images with every open retainer reservation treated as an optical occluder."""

import argparse
import itertools
import sys
from pathlib import Path

import cv2
import numpy as np
from screen import HERE, ROOT, reservations, read, sha

sys.path.insert(0, str(HERE.parent / "motion"))
import receiver_witness as camera
from wheel_visibility import load_wheel_mask

TRIANGLES = [
    0,
    2,
    3,
    0,
    3,
    1,
    4,
    5,
    7,
    4,
    7,
    6,
    0,
    1,
    5,
    0,
    5,
    4,
    2,
    6,
    7,
    2,
    7,
    3,
    0,
    4,
    6,
    0,
    6,
    2,
    1,
    3,
    7,
    1,
    7,
    5,
]


def projected(native=None, stroke=0.0):
    pieces = []
    for side in (-1, 1):
        for name, b in reservations(stroke).items():
            corners = []
            for sx, sy, sz in itertools.product(range(2), repeat=3):
                corners.extend(
                    [
                        round(b[3 * sx] * 10),
                        round(side * b[1 + 3 * sy] * 10),
                        round(b[2 + 3 * sz] * 10),
                    ]
                )
            pieces.append([f"{side}/{name}", "retainer", "#ffffff", corners, TRIANGLES])
    if native is not None:
        path = HERE / "open-yokes.step"
        if sha(path) != native["stepSha256"]:
            raise ValueError("Retainer CAD changed before rendering")
        shapes = camera.sensor.mount.cad.read_named(path, native["parts"])
        pieces = []
        for row in native["parts"]:
            points, triangles = camera.sensor.mount.original.triangulate(
                shapes[row["id"]]
            )
            vertices = np.asarray(points).reshape(-1, 3).copy()
            sgn = 1 if row["id"].startswith("retainer-left") else -1
            if row["frame"] in ("probe", "yoke"):
                vertices[:, 1] -= int(round(10 * sgn * stroke))
            pieces.append(
                (
                    row["id"],
                    "mechanism",
                    "#ffffff",
                    vertices.ravel().tolist(),
                    triangles,
                )
            )
            if stroke and row["frame"] == "probe":
                # Enclose the complete 0..8 mm compression; both end positions
                # and the intermediate sweep can hide a target feature.
                bounds = vertices.min(axis=0), vertices.max(axis=0)
                lo, hi = (v.copy() for v in bounds)
                lo[1] += min(0, 80 * sgn)
                hi[1] += max(0, 80 * sgn)
                box = [
                    int((lo, hi)[which[j]][j])
                    for which in itertools.product(range(2), repeat=3)
                    for j in range(3)
                ]
                pieces.append(
                    (row["id"] + "-travel", "mechanism", "#ffffff", box, TRIANGLES)
                )
    register = read(camera.LEGACY / "selected-optics/mount.json")["registrations"][
        "roof"
    ]
    depths = {}
    for name in ("fixed", "roof"):
        if name == "fixed":
            camera.sensor.scenes.fixed_pose(camera.O)
        else:
            camera.sensor.optics.ORIGIN = np.asarray(
                register["assumedRectifiedOriginMm"]
            )
            camera.sensor.optics.configure()
        depths[name] = [
            z for _, z in camera.sensor.camera.images_for_pose({"parts": pieces})
        ]
    return depths


def composite(background, reference, front, gray):
    images = []
    ref = []
    for (old, depth), (_, robot), z in zip(background, reference, front):
        changed = (z < 3490) & (z < depth)
        pixels = np.where(changed, gray, old).astype(np.uint8)
        images.append((pixels, np.where(changed, z, depth)))
        ref.append((pixels, np.where(changed, z, robot)))
    return images, ref


def run(artifacts, native=None):
    paths = artifacts / "receiver-contrast/corrected"
    reg = read(HERE.parent / "motion/loaded-feed.json")
    wheel = load_wheel_mask(artifacts / "loaded-swerve/wheel-visibility.json")
    depths = projected(native)
    result = {
        "inputPublicRobot": sha(ROOT / "cad/full-swerve-r4/full-robot.step"),
        "retainerSha256": native["stepSha256"] if native else None,
        "cases": [],
    }
    for feed in reg["attempts"]:
        scene = paths / (feed["case"] + ".npz")
        with np.load(scene) as stock:
            fixed = [(stock[f"fg{i}"], stock[f"fz{i}"]) for i in range(2)]
            roof = [(stock[f"g{i}"], stock[f"z{i}"]) for i in range(2)]
            ref = [(roof[i][0], stock[f"rz{i}"]) for i in range(2)]
        extra = {}
        for name, background in (("fixed", fixed), ("roof", roof)):
            extra[name] = [
                cv2.dilate(
                    ((z < 3490) & (z <= base[1] + 0.1)).astype("uint8"),
                    np.ones((11, 11), np.uint8),
                ).astype(bool)
                for z, base in zip(depths[name], background)
            ]
        masks = {k: [a | b for a, b in zip(wheel[k], extra[k])] for k in extra}
        states = camera.frames(feed, artifacts / "swerve-energy/contact")
        modes = {
            "excluded": (fixed, (roof, ref), masks),
            "original": (fixed, (roof, ref), wheel),
        }
        for gray in (65, 215):
            modeled_fixed, _ = composite(fixed, fixed, depths["fixed"], gray)
            modeled_roof, modeled_ref = composite(roof, ref, depths["roof"], gray)
            modes["envelope" + str(gray)] = (
                modeled_fixed,
                (modeled_roof, modeled_ref),
                wheel,
            )
        comparisons = {}
        for mode, (f, r, m) in modes.items():
            gated = []
            for stamp, p, axis in states:
                obs = camera.observe(f, r, p, axis, wheel_mask=m)
                gated.append(
                    {
                        "javaMs": stamp,
                        "accepted": obs["accepted"],
                        "fixed": obs["fixed"],
                        "roof": obs["roof"],
                        "sha256": obs["fourFramesSha256"],
                    }
                )
            comparisons[mode] = {
                "frames": gated,
                "emptyAccepted": camera.observe(f, r, None, None, wheel_mask=m)[
                    "accepted"
                ],
            }
        result["cases"].append(
            dict(
                case=feed["case"],
                stepUs=feed["plantStepUs"],
                fieldSceneSha256=sha(scene),
                sourcePipe=feed["pipeTrajectorySha256"],
                pixels={k: [int(a.sum()) for a in v] for k, v in extra.items()},
                comparisons=comparisons,
            )
        )
    return result


if __name__ == "__main__":
    import json

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifacts", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--native", type=Path)
    a = parser.parse_args()
    a.output.write_text(
        json.dumps(run(a.artifacts, read(a.native) if a.native else None), indent=2)
        + "\n",
        encoding="utf-8",
    )
