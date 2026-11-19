"""Build the project-owned open yokes and inspect their real solid volume without supplier geometry."""

from pathlib import Path
import argparse
import json
import math
import sys

from screen import HERE, ROOT, sha

CAD = ROOT / "experiments/six-wheel/design/station-l1"
sys.path.insert(0, str(CAD))
import build_assembly as cad
from mechanical import box, cylinder, cut


def make(side):
    """Machine one mirror member pair in the open position; X forward, Y left."""

    def block(x, y, z, X, Y, Z):
        return box(x, min(side * y, side * Y), z, X - x, Y - y, Z - z)

    def rod(x, y, z, r, length):
        return cylinder(x, side * y, z, (0, side, 0), r, length)

    def hub(x, y, z, r, length):
        return cylinder(x, side * y, z, (1, 0, 0), r, length)

    def drill(x, y, z, r, length):
        return cylinder(x, side * y, z, (0, 0, 1), r, length)

    def join(*forms):
        return ["union", list(forms)]

    prefix = "retainer-left" if side > 0 else "retainer-right"
    parts = []

    def add(name, body, frame, density, role="shelf"):
        parts.append(
            dict(
                id=f"{prefix}-{name}",
                group=role,
                body=body,
                frame=frame,
                density=density,
            )
        )

    # Bored, open-ended tubes; a blind press socket locates the outer end.
    for name, x in (("aft", 39), ("front", 110)):
        add(
            name + "-tube",
            cut(rod(x, 161, 579, 4, 60), rod(x, 160, 579, 3, 62)),
            "yoke",
            2.70e-6,
            "shaft",
        )
    bar = block(34, 219, 575, 123, 225, 583)
    sockets = [rod(x, 218, 579, 4.01, 3.1) for x in (39, 110)]
    bar = cut(bar, *sockets)
    add(
        "rack-and-bar",
        join(bar, block(119, 183, 579.3, 123, 225, 583)),
        "yoke",
        1.27e-6,
    )
    add("probe-post", block(73, 219, 583, 77, 225, 613), "yoke", 1.27e-6)
    add("rim-probe", rod(75, 183, 611, 0.75, 36), "probe", 7.85e-6, "sensor")
    # Differential force is measured through a spring sleeve on this rod.
    add(
        "probe-guide",
        cut(rod(75, 212, 611, 2.2, 7), rod(75, 211, 611, 0.86, 10)),
        "yoke",
        2.70e-6,
    )
    beam = block(34, 174, 573, 114, 183, 585)
    add(
        "guide-beam",
        cut(beam, *[rod(x, 173, 579, 4.2, 11) for x in (39, 110)]),
        "slide",
        1.27e-6,
    )
    add("beam-brace", block(46, 178, 568, 54, 203, 573), "slide", 2.70e-6)
    add("support-post", block(49, 197, 503, 53, 203, 568), "slide", 2.70e-6)
    add(
        "mount-foot",
        cut(
            block(44, 175, 500, 59, 209, 503),
            *[drill(51.5, y, 499, 1.7, 5) for y in (179.5, 203)],
        ),
        "slide",
        2.70e-6,
    )
    add("motor-plate", block(107, 183, 570, 118, 196, 575), "slide", 2.70e-6)
    # Consistent catalog-envelope model. The pinion disk and rack stock are
    # motion reservations; tooth profiles, servo spline and horn are still open.
    # Assembly bounds of the HS-65MG manufacturer model. Keep the sourced
    # housing, included horn and axial screw in one conservative optical body.
    servo = join(
        block(88.55, 177.53, 551.2, 114.65, 209.87, 562.8),
        hub(114.64, 188, 557, 12, 4.36),
        hub(118.9, 188, 557, 1.9, 1.4),
    )
    add("servo-envelope", servo, "slide", None, "drive")
    add(
        "pinion-blank",
        cut(hub(119, 188, 557, 22.3, 4), hub(118, 188, 557, 3.0, 6)),
        "slide",
        1.27e-6,
    )
    return parts


def export(destination):
    parts = make(1) + make(-1)
    bodies = [cad.brep(p["body"]) for p in parts]
    cad.write_named(destination, bodies, parts)
    checked = cad.read_named(destination, parts)
    rows = []
    for p in parts:
        shape = checked[p["id"]]
        volume, center = cad.volume(shape)
        mass = 12.5 if p["density"] is None else volume * p["density"] * 1000
        rows.append(
            dict(
                id=p["id"],
                frame=p["frame"],
                materialVolumeMm3=round(volume, 5),
                estimatedMassG=mass,
                boundsMm=list(cad.bounds(shape)),
            )
        )
    return dict(
        stepSha256=sha(destination),
        parts=rows,
        totalNominalG=sum(r["estimatedMassG"] for r in rows),
        excluded=[
            "sensor electronics and switch mounts",
            "fasteners and receiver reinforcements",
            "power wiring and converter",
            "sector fasteners / servo horns / spline",
            "strain relief and assembly tolerances",
        ],
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("destination", type=Path)
    p.add_argument("receipt", type=Path)
    a = p.parse_args()
    a.receipt.write_text(
        json.dumps(export(a.destination), indent=2) + "\n", encoding="utf-8"
    )
