"""Export original lift-off, half-round guide sections and mounting arm blanks."""

import argparse
import math
from pathlib import Path
import numpy as np
from OCP.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_MakeFace,
)
from OCP.BRepOffsetAPI import BRepOffsetAPI_MakePipe
from OCP.GC import GC_MakeArcOfCircle
from OCP.gp import gp_Pnt
from paths import routes, unit
from scene import CAD, sp1, write, m
from hardware import RADIUS, solids


def edge(a, b):
    return BRepBuilderAPI_MakeEdge(gp_Pnt(*a), gp_Pnt(*b)).Edge()


def arc(a, b, c):
    return BRepBuilderAPI_MakeEdge(
        GC_MakeArcOfCircle(gp_Pnt(*a), gp_Pnt(*b), gp_Pnt(*c)).Value()
    ).Edge()


def section(p, tangent, normal, r):
    v = unit(normal - tangent * (normal @ tangent))
    w = np.cross(tangent, v)
    inside = r + 0.15
    outside = r + 1.10
    ring = BRepBuilderAPI_MakeWire()
    for e in (
        arc(p + v * outside, p + w * outside, p - v * outside),
        edge(p - v * outside, p - v * inside),
        arc(p - v * inside, p + w * inside, p + v * inside),
        edge(p + v * inside, p + v * outside),
    ):
        ring.Add(e)
    return BRepBuilderAPI_MakeFace(ring.Wire()).Face()


def manufactured():
    included = {
        k: v
        for k, v in solids()[0].items()
        if k.startswith(
            ("mast-", "head-", "transition-support", "host-post", "base-", "roof-guide")
        )
    }
    guides = {}
    for name, path in routes()[0].items():
        n = 0
        for length, ps, r in path:
            count = math.ceil(length / 160)
            idx = np.linspace(0, len(ps) - 1, count + 1).round().astype(int)
            for i, j in zip(idx, idx[1:]):
                if r is None:
                    spine = edge(ps[i], ps[j])
                    tangent = unit(ps[j] - ps[i])
                else:
                    mid = (i + j) // 2
                    spine = arc(ps[i], ps[mid], ps[j])
                    # Circumcircle tangent to the source arc at its start.
                    direction = unit(ps[mid] - ps[i])
                    other = unit(ps[j] - ps[mid])
                    axis = unit(np.cross(direction, other))
                    center_dir = unit(np.cross(axis, direction))
                    chord = np.linalg.norm(ps[mid] - ps[i])
                    center = (ps[mid] + ps[i]) / 2 + math.sqrt(
                        max(0, r * r - chord * chord / 4)
                    ) * center_dir
                    tangent = unit(np.cross(axis, ps[i] - center))
                normal = (
                    np.array([0.0, 1.0, 0.0])
                    if abs(tangent[1]) < 0.85
                    else np.array([1.0, 0.0, 0.0])
                )
                face = section(ps[i], tangent, normal, RADIUS[name])
                tool = BRepOffsetAPI_MakePipe(BRepBuilderAPI_MakeWire(spine).Wire(), face)
                if not tool.IsDone():
                    raise ValueError("Could not form " + name)
                n += 1
                guides[f"{name}-trough-{n:02}"] = tool.Shape()
    return included, guides


def run(out):
    out.mkdir(parents=True, exist_ok=True)
    posts, rails = manufactured()
    bodies = {**posts, **rails}
    step = out / "harness-guides.step"
    sp1.write_step(step, bodies)
    sp1.finalize_step(step)
    reopened = CAD.read_named(step, [{"id": k} for k in bodies])
    volume = {k: round(CAD.volume(s)[0], 5) for k, s in reopened.items()}
    if any(abs(volume[k] - CAD.volume(v)[0]) > 0.01 for k, v in bodies.items()):
        raise ValueError("Guide exchange differed")
    write(
        out / "guides.json",
        {
            "stepSha256": m.digest(step),
            "supportBlanks": list(posts),
            "removableTroughs": list(rails),
            "volumeMm3": volume,
            "separateSourceHardwareIncluded": False,
            "manufacturingGates": [
                "Support-to-trough shoes, keys and ties must stay within the issued retained envelope; any change in exterior requires a rerun.",
                "Proof-load, deburr and dress completed guides; receiving tolerances and clearance under vibration remain unmeasured.",
            ],
        },
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("out", type=Path)
    run(p.parse_args().out)
