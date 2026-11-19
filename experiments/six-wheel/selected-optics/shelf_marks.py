"""Apply two flush, asymmetric registration marks only to visible native shelf pixels."""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from mechanics import FIXED, FORWARD, UP, digest, vendor
from camera_parts import box

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
# Dimensions on the R6 top surface in shelf coordinates, [X0,Y0,X1,Y1], mm.
# Deliberate unequal lengths and crossbars identify roll and left/right swaps.
INK = [
    [-81, 94, -77, 130],
    [-73, 94, -69, 118],
    [-65, 94, -61, 126],
    [-57, 94, -53, 109],
    [-49, 94, -45, 121],
    [-81, 94, -45, 98],
    [-81, -131, -77, -95],
    [-73, -124, -69, -95],
    [-65, -110, -61, -95],
    [-57, -129, -53, -95],
    [-49, -117, -45, -95],
    [-81, -99, -45, -95],
]
F = 320 / math.tan(math.radians(73 / 2))
V, U = np.mgrid[:480, :640] + 0.5


def apply(scene, origin, axis, up, stroke):
    bg = []
    changed = []
    for eye, (gray, depth) in zip((37.5, -37.5), scene):
        o = origin + [0, eye, 0]
        dx = axis[0] + (240 - V) * up[0] / F
        dy = -(U - 320) / F
        dz = axis[2] + (240 - V) * up[2] / F
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (500 - o[2]) / dz
            x = o[0] + t * dx - stroke
            y = o[1] + t * dy
        mask = np.zeros(depth.shape, bool)
        for x0, y0, x1, y1 in INK:
            mask |= (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
        # Unknown or foreground material is never painted. The mark receives no geometric depth credit.
        mask &= (t > 0) & (abs(depth - t) < 0.20)
        output = gray.copy()
        output[mask] = 36
        bg.append((output, depth))
        changed.append(int(np.count_nonzero(mask)))
    return bg, changed


def run(args):
    import sys

    sys.path.insert(0, str(ROOT / "relieved-match"))
    from sight import background
    from scenes import store

    args.out.mkdir(exist_ok=True, parents=True)
    shelf = ROOT / "selected-match/receiver-clearance-kit.step"
    article = json.loads((shelf.parent / "kit.json").read_bytes())
    if digest(shelf) != article["kitSha256"]:
        raise ValueError("Changed shelf STEP")
    shape = vendor.cad.read_named(shelf, [{"id": k} for k in article["volumeMm3"]])[
        "receiver-R6-drilled-for-camera"
    ]
    contact = []
    for x0, y0, x1, y1 in INK:
        gauge = box(x0, y0, 499.95, x1 - x0, y1 - y0, 0.05)
        ratio = vendor.cad.intersection(gauge, shape) / ((x1 - x0) * (y1 - y0) * 0.05)
        if abs(ratio - 1) > 0.00001:
            raise ValueError("Registration stroke crosses a hole, groove or edge")
        contact.append(ratio)
    scenes = json.loads((args.raw / "scenes.json").read_bytes())
    sweep = json.loads((args.raw / "sweep.json").read_bytes())
    mount = json.loads((HERE / "mount.json").read_bytes())["registrations"]
    tagged = {}
    sources = dict(scenes["backgroundSha256"])
    sources["barrel-moving-swept"] = sweep["backgroundSha256"]
    if "nearestPossibleSha256" in sweep:
        sources["barrel-moving-nearest"] = sweep["nearestPossibleSha256"]
    for name, source in sources.items():
        path = args.raw / (name + ".npz")
        if digest(path) != source:
            raise ValueError("Unregistered native render " + name)
        if name.endswith("-fixed"):
            origin, axis, up = FIXED, FORWARD, UP
            stroke = 370 if name.startswith("coral-") else 0
        else:
            stroke = 0 if not name.endswith("-370") else 370
            dz = {"front": -0.75, "barrel": 0.0, "rear": 0.75}[name.split("-")[0]]
            origin = np.array(mount["roof"]["assumedRectifiedOriginMm"]) + [
                stroke,
                0,
                dz,
            ]
            axis, up = np.array([0.0, 0.0, -1.0]), np.array([1.0, 0.0, 0.0])
        result, pixels = apply(background(path), origin, axis, up, stroke)
        h = store(args.out, name, result)
        if name.endswith("-nearest"):
            sweep["nearestPossibleSha256"] = h
        elif name.endswith("-swept"):
            sweep["backgroundSha256"] = h
        else:
            scenes["backgroundSha256"][name] = h
        tagged[name] = {
            "originalSha256": source,
            "markedSha256": h,
            "visibleMarkPixels": pixels,
        }
    for file, value in (("scenes", scenes), ("sweep", sweep)):
        (args.out / (file + ".json")).write_text(
            json.dumps(value, indent=2) + "\n", encoding="utf-8"
        )
    receipt = {
        "shelfSha256": article["kitSha256"],
        "unmarkedScenesSha256": digest(args.raw / "scenes.json"),
        "unmarkedSweepSha256": digest(args.raw / "sweep.json"),
        "inkRectsShelfXYmm": INK,
        "areaBackingFraction": contact,
        "syntheticInkGray": 36,
        "renderings": tagged,
    }
    (args.out / "marks.json").write_text(
        json.dumps(receipt, indent=2) + "\n", encoding="utf-8"
    )
    print(
        "Native backing verified for",
        len(contact),
        "strokes;",
        tagged["barrel-moving-swept"]["visibleMarkPixels"],
        "guarded pixels",
        flush=True,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("raw", type=Path)
    p.add_argument("out", type=Path)
    run(p.parse_args())
