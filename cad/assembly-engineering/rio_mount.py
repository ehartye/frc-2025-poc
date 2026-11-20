"""Locate original roboRIO carrier cleats from the four horizontal native eyelets."""

import argparse
import json
from pathlib import Path
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.STEPControl import STEPControl_Reader
from attachments import (
    base,
    cyl,
    cut,
    common,
    fuse,
    tube,
    hexagon,
    volume,
    dist,
    bounds,
)
from installed_audit import HERE, sha, box_gap
from clearance import move

RIO = "6e19ed3e825c8c521f57ec510eda860556ac422f134dc584b8342ca4d94be791"


def ycyl(x, y, z, r, h):
    return BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(x, y, z), gp_Dir(0, 1, 0)), r, h
    ).Shape()


def make(deck):
    components = {}
    ops = {}
    X, Y, Z = 245.0, -119.5, 302.652
    # Original public representation with genuine four hollow passages. Optical
    # case extent is conservative; the independently supplied model is checked separately.
    case = base.block((X - 67.1, Y - 71.59, 293, X + 67.1, Y + 71.59, Z + 25.057))
    for sx in (-1, 1):
        for sy in (-1, 1):
            x = X + 70.22 * sx
            y = Y + 50.8 * sy
            z = Z - 4.70
            k = f"rio-eye-{sx}-{sy}"
            eye = base.block(
                (x - 3.12, y - 4.679, z - 3.1, x + 3.12, y + 4.679, z + 3.1)
            )
            case = fuse(case, cut(eye, ycyl(x, y - 5, z, 2.41, 10)))
            outer = X + 79 * sx
            a, b = sorted((x - 3.4 * sx, outer + 3 * sx))
            c, d = sorted((X + 74.4 * sx, outer + 3 * sx))
            strap = base.block((c, y - 8, 293, d, y + 8, z + 3.4))
            for a0, a1 in ((-7.25, -4.88), (4.88, 7.25)):
                ear = base.block((a, y + a0, 293, b, y + a1, z + 3.4))
                strap = fuse(strap, ear)
            strap = cut(strap, ycyl(x, y - 9, z, 2.15, 18))
            components[k + "-M4-shoulder"] = fuse(
                ycyl(x, y - 10.25, z, 3.5, 3), ycyl(x, y - 7.25, z, 2, 20)
            )
            # Receiving selection: prevailing M4 hex nut and 7.5 mm OD driver.
            components[k + "-M4-locknut"] = cut(
                ycyl(x, y + 7.25, z, 3.55, 4), ycyl(x, y + 7.15, z, 2.0, 4.2)
            )
            for j, dy in enumerate((-4, 4)):
                xx = outer
                yy = y + dy
                strap = cut(strap, cyl(xx, yy, 292, 1.65, 12))
                deck = cut(deck, cyl(xx, yy, 288, 2.25, 7))
                # Screws drive from below into the two M4 tapped cleat holes.
                screw = fuse(cyl(xx, yy, 286.5, 3.5, 4), cyl(xx, yy, 290.5, 2, 10))
                components[k + f"-base-M4-{j}"] = screw
            components[k + "-cleat"] = strap
            ops[k] = dict(pinMm=[x, y, z], anchorXmm=outer, anchorYmm=[y - 4, y + 4])
    return deck, components, case, ops


def reference(path):
    if sha(path) != RIO:
        raise ValueError("Incorrect roboRIO 2.0 article")
    rd = STEPControl_Reader()
    rd.ReadFile(str(path))
    rd.TransferRoots()
    return move(rd.OneShape(), (245, -119.5, 302.652))


def assess(path):
    deck = base.named(HERE / "station-and-jack-interfaces.step")[
        "front-electronics-platform"
    ]
    deck, stock, case, ops = make(deck)
    product = reference(path)
    # Isolate exact native interference per cleat and lower shoulder. Catalog
    # attachment torque and thread access are separate from this test.
    evidence = {}
    native = [(s, bounds(s)) for s in base.subsolids(product)]
    for k, shape in stock.items():
        box = bounds(shape)
        possible = [s for s, b in native if box_gap(box, b) < 5.0]
        evidence[k] = dict(
            nativeIntersectionMm3=sum(common(s, shape) for s in possible),
            nativeGapMm=min((dist(s, shape) for s in possible), default=5.0),
            checkedVendorSolids=len(possible),
            totalVendorSolids=len(native),
        )
    return dict(
        source="https://andymark.com/products/ni-roborio-2-0",
        sourceSha256=RIO,
        originalLimitsMm=[-73.076, -71.590, -9.652, 73.076, 71.590, 25.057],
        mountingRadiusMm=2.41,
        registrationMm=[245, -119.5, 302.652],
        bodyBoundsMm=bounds(product),
        operations=ops,
        correspondence=evidence,
    )


if __name__ == "__main__":
    c = argparse.ArgumentParser()
    c.add_argument("vendor_step", type=Path)
    (HERE / "rio-mount.json").write_bytes(
        (json.dumps(assess(c.parse_args().vendor_step), indent=2) + "\n").encode()
    )
