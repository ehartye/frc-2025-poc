"""Register the issued SP1 and optical assembly in the robot frame."""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "sp1-mount"))
import check as sp1
from clearance import wire_boxes

sys.path.insert(0, str(ROOT / "selected-optics"))
import mechanics as m
from fixed_locator import CAD


def read(path):
    return json.loads(path.read_bytes())


def write(path, value):
    path.parent.mkdir(exist_ok=True, parents=True)
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf8", newline="\n")


def get(artifacts, lamp):
    q = ROOT / "sp1-mount"
    for path, expected in [
        (q / "sp1-mount.step", read(q / "fit.json")["stepSha256"]),
        (
            ROOT / "selected-optics/fixed-datum.step",
            read(ROOT / "selected-optics/mount.json")["stepSha256"],
        ),
    ]:
        if m.digest(path) != expected:
            raise ValueError("Unexpected " + path.name)
    poses = sp1.scene(artifacts, lamp)
    meta = read(ROOT / "selected-optics/mount.json")
    parts = CAD.read_named(
        ROOT / "selected-optics/fixed-datum.step", [{"id": k} for k in meta["reopenedParts"]]
    )
    raw, eyes, bounds, _ = m.source(artifacts / "camera-disconnects")
    matrix, offset = m.pose("fixed-datum-up", bounds, eyes.mean(axis=0))
    fixed, moving = wire_boxes()
    for stroke, pose in zip((0, 370), poses):
        for old in (
            "roof-camera-envelope",
            "observer-camera-proxy",
            "observer-slotted-cradle-weldment",
        ):
            del pose[old]
        pose.update(parts)
        pose.update(
            {f"fixed-head/{i}": m.vendor.transform(s, matrix, offset) for i, s in enumerate(raw)}
        )
        pose.update(
            {
                f"roof-head/{i}": m.vendor.transform(s, m.ROOF_M, m.ROOF_T + [stroke, 0, 0])
                for i, s in enumerate(raw)
            }
        )
        for name, b in {**fixed, **moving}.items():
            if name != "registered-camera-case":
                pose["wire/" + name] = sp1.native_box(
                    np.array(b) + ([stroke, 0, 0, stroke, 0, 0] if name in moving else 0)
                )
    return poses


def run(a):
    start, end = get(a.artifacts, a.lamp)
    rot = read(ROOT / "design/competition-package/shop-duplex/red/rotation-bounds.json")
    frames = read(ROOT / "design/competition-package/shop-duplex/red/part-frames.json")
    bounds = {}
    rest = {}
    for k, s in start.items():
        ab = np.array(CAD.bounds(s))
        cd = np.array(CAD.bounds(end[k]))
        rest[k] = ab.tolist()
        if k in rot:
            ab = np.array(rot[k]["bounds_mm"]) + [0, 0, 560, 0, 0, 560]
            cd = ab + [185, 0, 0, 185, 0, 0] if frames[k] == "middle" else ab
        bounds[k] = np.r_[np.minimum(ab[:3], cd[:3]), np.maximum(ab[3:], cd[3:])].tolist()
    write(
        a.out,
        {
            "bounds": bounds,
            "stowed": rest,
            "registeredBodies": len(start),
            "inputs": {
                p: m.digest(ROOT / p)
                for p in [
                    "sp1-mount/sp1-mount.step",
                    "sp1-mount/sp1_hardware.py",
                    "selected-match/receiver-clearance-kit.step",
                    "selected-optics/fixed-datum.step",
                    "selected-match/clearance.py",
                ]
            },
        },
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    for k in ("artifacts", "lamp", "out"):
        p.add_argument(k, type=Path)
    run(p.parse_args())
