"""Render the changed cam faces and open yokes at the actual signed station poses."""

from pathlib import Path
from argparse import ArgumentParser
import json
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
SW = HERE.parent
sys.path.insert(0, str(SW / "retention"))
from station_view import camera, projected, composite
from screen import read, sha

MATCH = SW / "score-stroke"
KIT = SW / "retention"


def foreground():
    parts = camera.source_parts()
    cam = read(MATCH / "cam.json")
    if sha(MATCH / "cam-lands.step") != cam["candidateStepSha256"]:
        raise ValueError("Changed shelf CAD")
    replaced = camera.sensor.mount.cad.read_named(
        MATCH / "cam-lands.step", [{"id": k} for k in cam["volumeDeltaMm3"]]
    )
    revised = []
    for name, role, color, pts, tri in parts:
        if name in replaced:
            pts, tri = camera.sensor.mount.original.triangulate(replaced[name])
        revised.append((name, role, color, pts, tri))
    del parts
    fixed = camera.render(revised, "fixed")
    roof = camera.render(revised, "roof")
    del revised
    return fixed, roof, cam


def run(artifacts, *, sides=("plus", "minus"), gap_mm=0.1):
    fixed, roof, cam = foreground()
    kit = read(KIT / "open-yokes.json")
    ys = projected(kit)
    mesh = artifacts / "swerve-transit/native-scene"
    field = read(mesh / "scene.json")
    if field["bytes"] != sha(mesh / "scene.npz"):
        raise ValueError("Different field geometry")
    with np.load(mesh / "scene.npz") as data:
        environment = [
            (key, data[f"v{i}"], data[f"f{i}"].tolist())
            for i, key in enumerate(field["keys"])
        ]
    wheel = camera.load_wheel_mask(artifacts / "loaded-swerve/wheel-visibility.json")
    out = {}
    for side in sides:
        name = f"cam-station-{side}-31.25us-{gap_mm:g}mm"
        raw = artifacts / "r4-score-stroke"
        feed = read(raw / (name + ".json"))
        if (
            feed["candidateStepSha256"] != cam["candidateStepSha256"]
            or feed["state"] != "IN_BAND"
        ):
            raise ValueError("No cam station seat")
        x, y, yaw = feed["trueChassisAtFeedMmMmDeg"]
        pose = (-x, -y, yaw)
        camera.sensor.scenes.fixed_pose(camera.O)
        fi = camera.scene(environment, pose, fixed)
        camera.sensor.optics.configure()
        ro = camera.scene(environment, pose, roof)
        rb = [
            (g, np.where(rz <= z + 0.01, rz, 3500.0))
            for (g, z), (_, rz) in zip(ro, roof)
        ]
        states = camera.frames(feed, raw / name)
        modes = {}
        for shade in [65, 215]:
            f, _ = composite(fi, fi, ys["fixed"], shade)
            r, b = composite(ro, rb, ys["roof"], shade)
            angular = camera.sensor.shelf_guard(
                r, b, camera.sensor.optics.ORIGIN[2], shade == 215, 0.05
            )
            checks = [
                dict(javaMs=t, **camera.observe(f, angular, p, axis, wheel_mask=wheel))
                for t, p, axis in states
            ]
            empty = camera.observe(f, angular, None, None, wheel_mask=wheel)
            modes[str(shade)] = {"frames": checks, "empty": empty["accepted"]}
        out[side] = dict(
            case=side,
            cam=cam["candidateStepSha256"],
            kit=kit["stepSha256"],
            piece=feed["physicalPieceId"],
            pipe=feed["pipeStateSha256"],
            trajectory=feed["pipeTrajectorySha256"],
            field=field["sources"]["field"],
            counts={
                name: sum(bool(v["accepted"]) for v in item["frames"])
                for name, item in modes.items()
            },
            observations=modes,
        )
    return out


if __name__ == "__main__":
    p = ArgumentParser(description=__doc__)
    p.add_argument("artifacts", type=Path)
    a = p.parse_args()
    out = run(a.artifacts)
    (a.artifacts / "cam-scoring/optics.json").write_text(
        json.dumps(out, indent=2) + "\n", encoding="utf8"
    )
    print({k: v["counts"] for k, v in out.items()})
