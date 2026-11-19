"""Compare source and machined shelf depth and silhouette at the selected pupils."""

import argparse
import json
from pathlib import Path
import sys

import numpy as np
from tray import ROOT, HERE, SHELF, cad, digest, source

sys.path.insert(0, str(ROOT / "relieved-match"))
from cameras import mount, camera, optics

sys.path.insert(0, str(ROOT / "selected-optics"))
import scenes


def stereo(parts, stroke):
    mesh = []
    for name, solid in parts.items():
        v, t = mount.original.triangulate(cad.translated(solid, (stroke, 0, 0)))
        # Original external surface finish used for nominal ideal-light comparisons.
        mesh.append([name, "shop", "#737d85", v, t])
    return camera.images_for_pose({"parts": mesh})


def run(installed=False):
    from installed import STATIONS, GAP
    from damper_fit import cylinder, envelope
    from tray import box, unite

    meta = json.loads(
        (HERE / ("installed.json" if installed else "native.json")).read_bytes()
    )
    exchange = HERE / ("stow-receiver.step" if installed else "shelf-insert.step")
    expected = meta["manufacturingSha256" if installed else "kitSha256"]
    if digest(exchange) != expected:
        raise ValueError("Unknown shelf machining")
    parts = cad.read_named(exchange, [{"id": k} for k in meta["volumeMm3"]])
    parts.pop("slide-base-bridge", None)
    old = {SHELF: source()}
    fixed = {}
    if installed:
        for i, y in enumerate((-70.0, 0.0, 70.0)):
            pad = unite(
                cylinder(155, y, 500, 3.6, 0.8),
                box(125, y - 2.5, 500, 155, y + 2.5, 500.8),
            )
            old["pad-" + str(i)] = pad
            parts["pad-" + str(i)] = cad.translated(pad, (0, 0, -0.8))
        for i, (x, y) in enumerate(STATIONS):
            fixed["absorber-" + str(i)] = envelope(x, y, 485 - GAP)
            for j, z in enumerate((430.0, 438.0)):
                fixed[f"KM8-{i}-{j}"] = cylinder(x, y, z, 10 / np.sqrt(3), 3.0)
    reg = json.loads((ROOT / "selected-optics/mount.json").read_bytes())[
        "registrations"
    ]
    scenes.camera = camera
    origins = {
        k: np.array(reg[k]["assumedRectifiedOriginMm"])
        for k in ("roof", "fixed-datum-up")
    }
    pitch = np.deg2rad(26.0)
    axis = np.array([np.cos(pitch), 0.0, -np.sin(pitch)])
    rows = []
    for stroke in (0, 185, 370):
        for head in origins:
            for off in (-0.75, 0.0, 0.75):
                if head == "roof":
                    optics.ORIGIN = origins[head] + [stroke, 0, off]
                    optics.configure()
                else:
                    scenes.fixed_pose(origins[head] + off * axis)
                before = stereo(old, stroke)
                after = stereo(parts, stroke)
                if fixed:
                    added = stereo(fixed, 0)
                    after = [
                        (np.where(zz < z, gg, g), np.minimum(zz, z))
                        for (g, z), (gg, zz) in zip(after, added)
                    ]
                a, b = before, after
                for eye, (before, after) in enumerate(zip(a, b)):
                    g, z = before
                    h, zz = after
                    delta = np.abs(z.astype(float) - zz.astype(float))
                    rows.append(
                        {
                            "strokeMm": stroke,
                            "head": head,
                            "pupilOffsetMm": off,
                            "eye": eye,
                            "changedGreyPixels": int(np.count_nonzero(g != h)),
                            "depthChangedAbove01mm": int(np.count_nonzero(delta > 0.1)),
                            "maxDepthChangeMm": float(np.max(delta)),
                            "sourceOccupiedPixels": int(np.count_nonzero(z < 3490)),
                        }
                    )
    return {
        "sourceNewStep": expected,
        "includesFixedCartridgesAndFlushedPads": installed,
        "pupilRegister": digest(ROOT / "selected-optics/mount.json"),
        "nominalUndeformed": True,
        "resolution": [640, 480],
        "views": rows,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("out", type=Path)
    p.add_argument("--installed", action="store_true")
    a = p.parse_args()
    a.out.write_text(
        json.dumps(run(a.installed), indent=2) + "\n", encoding="utf8", newline="\n"
    )
