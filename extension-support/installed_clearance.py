"""Check added keeper material throughout the stroke against the registered robot and harness."""

from pathlib import Path
import argparse
import json
import sys
import numpy as np
from rail_geometry import HERE, ROOT, read, exchange, cad, translated, digest
from rail_capture import stock
from tray import subtract

sys.path.insert(0, str(ROOT / "guide-arm-route"))
from arm_route import carriage, inputs, j
from shop_fit import check_translation, swept_box


def gap_on_path(part, edge, travel, radius):
    intervals = [(0.0, travel)]
    lower = float("inf")
    while intervals:
        a, b = intervals.pop()
        mid = (a + b) / 2
        net = j.distance(translated(part, x=mid), edge) - radius
        bound = net - (b - a) / 2
        if bound > 0 or net <= 0 or b - a < 0.025:
            lower = min(lower, bound)
        else:
            intervals.extend(((a, mid), (mid, b)))
    return lower


def run(registry):
    scene, edges, _ = carriage.scene(registry, ROOT / "fixed-hoist")
    supports = carriage.selected(
        ROOT / "carriage-support",
        "carriage-support.step",
        "fit.json",
        "stepSha256",
        "volumesMm3",
    )
    scene.update(
        {
            k: v
            for k, v in supports.items()
            if not k.startswith("support-machined-mast-")
        }
    )
    scene.update(inputs())
    registered = read(registry)
    fixture = read(HERE / "keepers.json")
    articles = exchange(
        HERE / "rail-fixture.step", fixture["stepSha256"], fixture["volumesMm3"]
    )
    before = stock()
    for k in before:
        if k not in scene:
            raise ValueError(f"Missing selected rail: {k}")
        del scene[k]
    additions = {}
    for k, v in articles.items():
        if k.startswith("slide-") and k.endswith("-outer"):
            additions[k + "/added-wall"] = (subtract(v, before[k]), 0.0)
        elif k not in before:
            additions[k] = (v, 0.0 if k.startswith("outer-dog") else 370.0)
    # Only these three previously boxed articles enter the new occupied spans.
    # Require both endpoint boxes and their actual red-shop motion before
    # replacing the conservative broad-phase proxy by a relative native sweep.
    shop = ROOT / "design/competition-package/shop-duplex/red"
    frames = read(shop / "part-frames.json")
    native = exchange(
        shop / "stowed.step",
        read(shop / "review.json")["stowed"]["export_sha256"],
        frames,
    )
    dz = 438.0 - cad.bounds(native["slide-L-outer"])[2]
    changes = {}
    for key in (
        "slide-sync-bracket--79",
        "slide-sync-bracket-79",
        "slide-middle-stop-dog-leg",
    ):
        original = translated(native[key], z=dz)
        if (
            frames[key] != "middle"
            or not np.allclose(
                cad.bounds(original), registered["stowed"][key], atol=0.002, rtol=0
            )
            or not np.allclose(
                swept_box(original, 185.0),
                registered["bounds"][key],
                atol=0.002,
                rtol=0,
            )
        ):
            raise ValueError("Changed relative registration: " + key)
        scene[key] = original
        changes[key] = 185.0
    extents = {k: swept_box(v, changes.get(k, 0.0)) for k, v in scene.items()}
    close = {}
    for name, (part, travel) in additions.items():
        moving = swept_box(part, travel)
        near = {}
        for key, v in scene.items():
            if j.lower_bound(moving, extents[key]) > 0.001:
                continue
            penetration, open_interval = check_translation(
                part, v, travel - changes.get(key, 0.0)
            )
            near[key] = {"commonMm3": penetration, "unresolved": open_interval}
        cable = {}
        for key, edge, bb, r in edges:
            broad = j.lower_bound(moving, bb) - r
            if broad > 5:
                continue
            lower = gap_on_path(part, edge, travel, r)
            cable[key] = min(cable.get(key, float("inf")), lower)
        close[name] = {"nearbyShapes": near, "nearbyCableGapLowerMm": cable}
    return {
        "fixtureSha256": fixture["stepSha256"],
        "registrySha256": digest(registry),
        "otherShapes": len(scene),
        "harnessEdges": len(edges),
        "newMaterial": close,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("registry", type=Path)
    p.add_argument("out", type=Path)
    a = p.parse_args()
    a.out.write_text(
        json.dumps(run(a.registry), indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
