"""Check the actual two wheel endpoints against the surveyed red-reef features and held pipe."""

import argparse
import importlib.util
import json
import math
from pathlib import Path

import cv2
import numpy as np
from screen import HERE, ROOT, read, sha
from station_view import projected, TRIANGLES, camera
import itertools

SECTOR = ROOT / "experiments/six-wheel/sector-alignment"
spec = importlib.util.spec_from_file_location(
    "retainer_reef_vision", SECTOR / "vision.py"
)
vision = importlib.util.module_from_spec(spec)
spec.loader.exec_module(vision)


def create_scene(artifacts, robot_foreground=None):
    old = read(SECTOR / "scene.json")
    cache = artifacts / "swerve-transit/native-scene"
    if sha(cache / "scene.npz") != read(cache / "scene.json")["bytes"]:
        raise ValueError("Current initial R4 robot-only camera cache changed")
    points = vision.corners(np.array(old["targetFrameCenterMm"]))
    faces = []
    centers = []
    for name, v in sorted(old["boltFacesMm"].items()):
        v = np.asarray(v)
        h = cv2.convexHull(v[:, 1:].astype("float32")).reshape(-1, 2)
        m = cv2.moments(h)
        centers.append([v[:, 0].min(), m["m10"] / m["m00"], m["m01"] / m["m00"]])
        faces.append(np.column_stack((np.full(len(h), centers[-1][0]), h)))
    result = dict(points=points, bolts=np.array(centers), faces=faces)
    depth = projected(read(HERE / "open-yokes.json"), 35.0)["fixed"]
    camera.sensor.scenes.fixed_pose(camera.O)
    b = old["carryBoxMm"]
    v = [
        b[k + 3 * i[k]] * 10
        for i in itertools.product(range(2), repeat=3)
        for k in range(3)
    ]
    box = camera.sensor.camera.images_for_pose(
        {"parts": [["loaded-pipe", "reserve", "#ffffff", v, TRIANGLES]]}
    )
    wheel = camera.load_wheel_mask(artifacts / "loaded-swerve/wheel-visibility.json")
    foreground = cache / "scene.npz" if robot_foreground is None else robot_foreground
    with np.load(foreground) as raw:
        for i, k in enumerate(("left", "right")):
            g, z = raw[f"fixed{i}g"], raw[f"fixed{i}z"]
            front = np.minimum(depth[i], box[i][1])
            result[k] = np.where(front < z, 65, g).astype(np.uint8)
            result[k + "_z"] = np.where(wheel["fixed"][i], 0, np.minimum(front, z))
    return result


def measure(ctx, pose, pred, mode):
    sample = vision.station_frame(
        pose,
        ctx,
        11 if mode == "wrong-tag" else 10,
        mode == "duplicate",
        mode == "occluded",
    )
    tag, heads, diagnostic, quality = sample
    pair = vision.associate(heads, ctx["bolts"], pred)
    if pair is None:
        return {"tag": tag is not None, "boltPair": False}
    if isinstance(pair, tuple) and len(pair) == 2 and isinstance(pair[1], (float, int)):
        pair = pair[0]
    fitted = vision.estimate(pair, ctx["bolts"])
    enclosure = vision.interval(
        pair, ctx["bolts"], np.array([-54.0, -180.0]), epsilon=0.23
    )
    return dict(
        tag=tag is not None,
        boltPair=True,
        fit=fitted.tolist(),
        bounds=None
        if enclosure is None
        else [enclosure[0].tolist(), enclosure[1].tolist()],
    )


def compare(artifacts):
    ctx = create_scene(artifacts)
    result = {}
    for side in ("plus", "minus"):
        wheel = read(artifacts / "retained-coral" / side / "continued/route.json")
        with np.load(artifacts / "retained-coral" / side / "continued/path.npz") as w:
            full = w["worldPose"]
            estimated = w["estimatedPose"]
            times = w["relativeS"]
        x, y, a = wheel["endPose"]
        gx, gy = wheel["goals"][-1]
        pose = np.array([1000 * (x - gx) - 54, 1000 * (y - gy) - 180, a])
        results = {
            mode: measure(ctx, pose, [-54.0, -180.0, 0], mode)
            for mode in ("nominal", "duplicate", "occluded", "wrong-tag")
        }
        ahead = []

        def local(v):
            return np.array([1000 * (v[0] - gx) - 54, 1000 * (v[1] - gy) - 180, v[2]])

        for i in range(0, len(times), 20):
            if (
                full[i, 0] > gx - 0.99
                and abs(full[i, 1] - gy) < 0.06
                and abs(full[i, 2]) < 0.04
            ):
                q = local(full[i])
                estimate = local(estimated[i])
                tag, found, actual, diagnostic = vision.station_frame(q, ctx)
                pair = vision.associate(found, ctx["bolts"], estimate)
                ahead.append(
                    dict(
                        wallS=wheel["startWallS"] + times[i],
                        positionMm=q.tolist(),
                        survey=tag is not None,
                        paired=pair is not None,
                        odoMm=estimate.tolist(),
                        tag=None if tag is None else tag.tolist(),
                        bolt=None
                        if pair is None
                        else vision.estimate(pair, ctx["bolts"]).tolist(),
                    )
                )
        result[side] = dict(
            wheel=wheel["trajectory"],
            trueEndMmMmRad=pose.tolist(),
            imageResults=results,
            approach=ahead,
        )
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("artifacts", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    a.output.write_text(
        json.dumps(compare(a.artifacts), indent=2) + "\n", encoding="utf-8"
    )
