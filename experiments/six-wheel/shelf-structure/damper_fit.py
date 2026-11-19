"""Register two through-bore shock absorbers and check the telescoping stages."""

import argparse
import json
import math
from pathlib import Path
import sys
import numpy as np
from tray import ROOT, SHELF, common, volume, cad, unite

sys.path.insert(0, str(ROOT / "sp1-mount"))
import check as sp1
from shop_fit import linear_travel, check_translation, swept_box
from clearance import dock, native_box


def cylinder(x, y, z, r, h):
    return cad.brep(["cyl", [x, y, z, 0, 0, 1, r, h]])


def envelope(x, y, tip):
    lower = tip - 61.4
    body = cylinder(x, y, lower, 4, 48.3)
    shaft = cylinder(x, y, lower + 48.3, 1.25, 13.1)
    # Full rod excursion including the inward-moving button.
    head = cylinder(x, y, tip - 2.1 - 8, 3.2, 10.1)
    return unite(unite(body, shaft), head)


def run(artifacts, lamp, stations, selected=False):
    start, end = sp1.scene(artifacts, lamp)
    rot = json.loads(
        (
            ROOT / "design/competition-package/shop-duplex/red/rotation-bounds.json"
        ).read_bytes()
    )
    frames = json.loads(
        (
            ROOT / "design/competition-package/shop-duplex/red/part-frames.json"
        ).read_bytes()
    )
    scan = {}
    for key, sh in start.items():
        if key in (SHELF, "slide-base-bridge"):
            continue
        if key in rot:
            b = np.array(rot[key]["bounds_mm"]) + [0, 0, 560, 0, 0, 560]
            shape = native_box(b)
            travel = 185 if frames[key] == "middle" else 0
        else:
            shape = sh
            travel = linear_travel(cad.bounds(sh), cad.bounds(end[key]))
            if travel is None:
                b, e = np.array(cad.bounds(sh)), np.array(cad.bounds(end[key]))
                shape = native_box(
                    np.r_[np.minimum(b[:3], e[:3]), np.maximum(b[3:], e[3:])]
                )
                travel = 0
        scan[key] = shape, travel, swept_box(shape, travel)
    results = []
    for x, cy in stations:
        records = {}
        for side in (1,) if selected else (-1, 1):
            y = side * cy
            tip = 484.9 if selected else 484.8
            supplier = envelope(x, y, tip)
            # AF10 hex is enclosed by its corner circle in all tool orientations.
            nut1 = cylinder(x, y, 430, 10 / math.sqrt(3), 3)
            nut2 = cylinder(x, y, 438, 10 / math.sqrt(3), 3)
            items = {"supplier": supplier, "lowerNut": nut1, "upperNut": nut2}
            record = {}
            for n, shape in items.items():
                b = cad.bounds(shape)
                pairs = {}
                for k, (obs, travel, sweep) in scan.items():
                    if dock.separation(b, sweep) > 1e-6:
                        continue
                    hit, unresolved = check_translation(obs, shape, travel)
                    if hit > 0.001 or unresolved:
                        pairs[k] = {"commonMm3": hit, "unresolved": unresolved}
                record[n] = pairs
            bore = cylinder(x, y, 432, 4.15, 7)
            record["bridgeDrillMm3"] = volume(common(start["slide-base-bridge"], bore))
            if selected:
                # The full installed stroke of the rod end is already enclosed.
                # Keep the small intentional shelf/bridge interfaces separate.
                for n, shape in items.items():
                    b = cad.bounds(shape)
                    record[n + "SweptAabbLowerMm"] = min(
                        dock.separation(b, sweep) for _, _, sweep in scan.values()
                    )
                below = cylinder(x, y, tip - 61.4 - 60.0, 3.5, 60.0)
                record["axialAdjustmentToolAtStow"] = {
                    k: volume(common(below, ob))
                    for k, (ob, movement, sweep) in scan.items()
                    if cad.occupied(cad.bounds(below), cad.bounds(ob))
                    and volume(common(below, ob)) > 0.001
                }
            # Native body and both rounded application zones stay separate from
            # the shelf/bridge manufacturing exchange.
            records[str(side)] = record
        results.append(
            {"xMm": x, ("yMm" if selected else "absYmm"): cy, "sides": records}
        )
    return {
        "registeredBodies": len(start),
        "strokeMm": [0, 370],
        "supplierButtonTopMm": 484.9 if selected else 484.8,
        "results": results,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for k in ("artifacts", "lamp", "out"):
        p.add_argument(k, type=Path)
    p.add_argument("--stations", nargs="+", default=["118,66"])
    p.add_argument("--selected", action="store_true")
    a = p.parse_args()
    points = [list(map(float, v.split(","))) for v in a.stations]
    r = run(a.artifacts, a.lamp, points, a.selected)
    a.out.write_text(json.dumps(r, indent=2) + "\n", encoding="utf8", newline="\n")
