"""Render phase-zero observations for each attained path using both registered 593-body cameras."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "bridge-tolerance"))
import replay_candidate as bridge
import bridge_fit
from design import mount, optics
from optics import camera


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def observe(cache, historical, lamp):
    rows = json.loads((HERE / "source.json").read_bytes())
    ref = json.loads((ROOT / "bridge-tolerance/replay.json").read_bytes())
    scene = historical / ("moving-custody-" + digest(ROOT / "bridge-tolerance/moving-bridge.step")[:16] + ".npz")
    if digest(scene) != ref["registration_sha256"]:
        raise ValueError("Moving camera source changed")
    link = bridge.mod("registered_pair", ROOT / "moving-observer/handoff.py")
    shapes = bridge.model(lamp)
    _, groups, colors, _ = mount.pose_layers(560, 370, "red")
    mesh = {"parts": []}
    for name, shape in sorted(shapes.items()):
        vertices, triangles = mount.original.triangulate(shape)
        mesh["parts"].append([name, groups.get(name, "Mount"), colors.get(name, "#737d85"), vertices, triangles])
    fixed = camera.images_for_pose(mesh)
    moving = link.load_scene(scene)
    trajectories = {}
    for row in rows:
        key = row["pipe"] + "-" + row["mode"]
        file = cache / (key + ".npy")
        if digest(file) != row["trajectorySHA256"]:
            raise ValueError("Unregistered pipe motion")
        trajectories[key] = np.load(file)
    for head in ("fixed", "moving"):
        if head == "moving":
            optics.ORIGIN = np.array([48., 0., 842.])
            optics.configure()
        memo = {}
        for row in rows:
            pipe, mode = row["pipe"], row["mode"]
            key = pipe + "-" + mode
            tr = trajectories[key]
            start = ref["cases"][pipe]["first_ms"] if head == "moving" else int(np.ceil(row["releaseCameraMs"] / 17)) * 17
            with (cache / (key + "-" + head + ".csv")).open("w", newline="") as out:
                writer = csv.writer(out, lineterminator="\n")
                writer.writerow(["piece", "head", "capture_ms", "status", "x_lo", "x_hi", "y_lo", "y_hi", "bottom_lo", "bottom_hi"])
                for tick in range(start, int(row["fieldTouchCameraMs"]) + 1, 17):
                    x, y, z, yaw, stroke = [np.interp(tick, tr[:, 0], col) for col in tr[:, 1:].T]
                    coords = (x - stroke if head == "moving" else x, y, z, yaw)
                    state = tuple(round(float(v), 6) for v in coords)
                    if state not in memo:
                        bg = moving if head == "moving" else fixed
                        frame = camera.exposure(bg, state[:3], state[3])
                        if head == "moving":
                            r = bridge_fit.fit(frame, bg)
                            names = ("x_mm", "y_mm", "bottom_mm")
                        else:
                            r = link.read_pair(frame, [gray for gray, _ in bg], 560, 370)
                            names = ("u_interval_mm", "y_interval_mm", "bottom_relative_to_shelf_mm")
                        memo[state] = [r["status"], *(v for n in names for v in r.get(n, []))]
                    writer.writerow([pipe, head, tick, *memo[state]])
            print(key, head, len(memo), flush=True)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("cache", "historical", "lamp"):
        p.add_argument(name, type=Path)
    a = p.parse_args()
    observe(a.cache, a.historical, a.lamp)
