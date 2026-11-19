"""Verify that both native marks remain on stock, and apply the fixed-lead sight guard."""

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

from checkpoint import ROOT, digest

sys.path.insert(0, str(ROOT / "shelf-structure"))
from tray import HERE as SHELF, FACE, box, cad, volume, common

sys.path.insert(0, str(ROOT / "selected-optics"))
import shelf_marks
from scenes import store

sys.path.insert(0, str(ROOT / "fixed-camera-route"))
from visibility import register, depths
from paths import F, U, L


def run(assets, folder):
    raw = folder / "native"
    marked = folder / "original-finish"
    out = folder / "marked"
    issue = json.loads((SHELF / "installed.json").read_bytes())
    step = SHELF / "stow-receiver.step"
    if digest(step) != issue["manufacturingSha256"]:
        raise ValueError("Changed machined shelf")
    face = cad.read_named(step, [{"id": k} for k in issue["volumeMm3"]])[FACE]
    backing = []
    for x, y, xx, yy in shelf_marks.INK:
        probe = box(x, y, 499.95, xx, yy, 500.0)
        fraction = volume(common(probe, face)) / volume(probe)
        if abs(fraction - 1) > 1e-5:
            raise ValueError("Asymmetric mark lost its receiving stock")
        backing.append(fraction)
    subprocess.run(
        [
            sys.executable,
            str(ROOT / "selected-optics/shelf_marks.py"),
            str(raw),
            str(marked),
        ],
        check=True,
    )
    scene = json.loads((marked / "scenes.json").read_bytes())
    sweep = json.loads((marked / "sweep.json").read_bytes())
    mounts = json.loads((ROOT / "selected-optics/mount.json").read_bytes())[
        "registrations"
    ]
    bound = assets / "fixed-camera-route/scene.json"
    if (
        digest(bound)
        != "291d68c43e21f20c33885064e6bbe8c0e47b684d13a8a06e8af61f9fd9db5886"
    ):
        raise ValueError("Different fixed harness")
    boxes = register(json.loads(bound.read_bytes())["stowed"])
    sources = dict(scene["backgroundSha256"])
    sources["barrel-moving-swept"] = sweep["backgroundSha256"]
    sources["barrel-moving-nearest"] = sweep["nearestPossibleSha256"]
    views = {}
    memo = {}
    out.mkdir(exist_ok=True, parents=True)
    for name, expected in sources.items():
        src = marked / (name + ".npz")
        if digest(src) != expected:
            raise ValueError("Changed camera raster " + name)
        fixed = name.endswith("-fixed")
        swept = name.endswith(("-nearest", "-swept", "-reference"))
        axis = F if fixed else np.array([0, 0, -1.0])
        up = U if fixed else np.array([1.0, 0, 0])
        center = np.array(
            mounts["fixed-datum-up" if fixed else "roof"]["assumedRectifiedOriginMm"]
        )
        if not fixed:
            center += [
                370 if name.endswith("-370") else 0,
                0,
                {"barrel": 0.0, "front": -0.75, "rear": 0.75}[name.split("-")[0]],
            ]
        key = tuple(center) + (fixed, swept)
        if key not in memo:
            ob = [b - np.array([370, 0, 0, 0, 0, 0]) for b in boxes] if swept else boxes
            memo[key] = [
                depths(ob, center + eye * L, axis, up) for eye in (37.5, -37.5)
            ]
        result = []
        counts = []
        with np.load(src) as source:
            for eye, close in zip(("left", "right"), memo[key]):
                gray, z = source[eye], source[eye + "_z"]
                visible = (close < z + 0.2) & (z < 3490)
                counts.append(int(visible.sum()))
                if name.endswith("-nearest"):
                    result.append(
                        (gray.copy(), np.where(z < 3490, np.minimum(z, close), z))
                    )
                else:
                    result.append(
                        (
                            np.where(visible, 0, gray).astype(np.uint8),
                            np.where(visible, 3500, z),
                        )
                    )
        h = store(out, name, result)
        views[name] = {"count": counts, "sha256": h}
        if name.endswith("-nearest"):
            sweep["nearestPossibleSha256"] = h
        elif name.endswith("-swept"):
            sweep["backgroundSha256"] = h
        else:
            scene["backgroundSha256"][name] = h
    for name, data in (("scenes", scene), ("sweep", sweep)):
        (out / (name + ".json")).write_text(
            json.dumps(data, indent=2) + "\n", encoding="utf8", newline="\n"
        )
    return {
        "issueStep": digest(step),
        "markAreaBacking": backing,
        "fixedLeadBounds": len(boxes),
        "fixedLeadSource": digest(bound),
        "native": scene["hardware"]["selectedPartsIncludingWire"],
        "views": views,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for k in ("artifacts", "folder", "out"):
        p.add_argument(k, type=Path)
    a = p.parse_args()
    a.out.write_text(
        json.dumps(run(a.artifacts, a.folder), indent=2) + "\n",
        encoding="utf8",
        newline="\n",
    )
