"""Reopen the telescoping rail contacts and the receiver underside above each fixed button."""

import argparse
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "shelf-structure"))
from tray import FACE, TRAY, cad, common, digest, unite, volume

STROKES = (0, 3, 10, 25, 49, 100, 185, 215, 218, 222, 275, 345, 370)


def read(path):
    return json.loads(path.read_bytes())


def exchange(path, expected, names):
    if digest(path) != expected:
        raise ValueError(
            f"Exchange does not match its manufacturing record: {path.name}"
        )
    return cad.read_named(path, [{"id": name} for name in names])


def translated(shape, x=0, z=0):
    return cad.translated(shape, (float(x), 0, float(z)))


def rail_contact(upper, lower):
    # A 50 micron witness beyond the design clearance identifies the actually
    # coincident floor. Counting bounding-box overlap alone includes air.
    contact = common(translated(upper, z=-1.05), lower)
    if volume(contact) <= 0:
        raise ValueError("No downward floor witness")
    bounds = cad.bounds(contact)
    return {
        "downwardXMm": [bounds[0], bounds[3]],
        "floorCommonMm3": volume(contact),
        "upwardCommonAt1p05Mm3": cad.intersection(translated(upper, z=1.05), lower),
    }


def button_contact(shelf, stroke, x, y, free_top):
    # This cylinder queries the entire projected button face along its vertical
    # line of action. With no material overhead it cannot act on the shelf.
    def cylinder(bottom, height):
        return cad.brep(["cyl", [x - stroke, y, bottom, 0, 0, 1, 3.2, height]])

    sample = common(shelf, cylinder(free_top, 18))
    if volume(sample) < 1e-7:
        return {"overhead": False, "firstGapMm": None, "faceFractionWithin0p15Mm": 0.0}
    witness = cylinder(free_top + 0.149, 0.001)
    return {
        "overhead": True,
        "firstGapMm": cad.bounds(sample)[2] - free_top,
        "faceFractionWithin0p15Mm": volume(common(shelf, witness)) / volume(witness),
    }


def run():
    shop = ROOT / "design/competition-package/shop-duplex/red"
    package = ROOT / "selected-match"
    shelf_dir = ROOT / "shelf-structure"
    review = read(shop / "review.json")
    frames = read(shop / "part-frames.json")
    kit = read(package / "kit.json")
    manufacturing = read(shelf_dir / "installed.json")
    pin = read(shelf_dir / "damper-fit.json")
    old = exchange(shop / "stowed.step", review["stowed"]["export_sha256"], frames)
    final = exchange(shop / "place.step", review["place"]["export_sha256"], frames)
    raised = exchange(
        package / "receiver-clearance-kit.step", kit["kitSha256"], kit["volumeMm3"]
    )
    machined = exchange(
        shelf_dir / "stow-receiver.step",
        manufacturing["manufacturingSha256"],
        manufacturing["volumeMm3"],
    )
    combined = unite(machined[FACE], machined[TRAY])
    # Use the same comparison material densities as the selected plate model.
    from response import RHO_AL, RHO_PE

    masses = {}

    def register_mass(name, shape, density, stage):
        v, center = cad.volume(shape)
        masses[name] = {
            "volumeMm3": v,
            "densityKgM3": density,
            "massKg": v * density * 1e-9,
            "stowedCenterXMm": center[0],
            "stage": stage,
        }

    register_mass(FACE, machined[FACE], RHO_PE, "inner")
    register_mass(TRAY, machined[TRAY], RHO_AL, "inner")
    contacts = {}
    for side in ("L", "R"):
        outer_id, mid_id, inner_id = (
            f"slide-{side}-outer",
            f"slide-{side}-middle",
            f"drilled-slide-{side}-inner",
        )

        def travel(key):
            return round(cad.bounds(final[key])[0] - cad.bounds(old[key])[0], 6)

        ratio = travel(mid_id) / travel(inner_id)
        if (
            travel(outer_id) != 0
            or travel(inner_id) != pin["strokeMm"][1]
            or ratio != 0.5
        ):
            raise ValueError("Unexpected stage motion")
        dz = 438.0 - cad.bounds(old[outer_id])[2]
        outer = translated(old[outer_id], z=dz)
        mid = translated(old[mid_id], z=dz)
        inner = raised[inner_id]
        register_mass(inner_id, inner, RHO_AL, "inner")
        register_mass(mid_id, mid, RHO_AL, "middle")
        contacts[side] = {}
        for s in STROKES:
            moving_mid = translated(mid, x=ratio * s)
            contacts[side][s] = {
                "innerOnMiddle": rail_contact(translated(inner, x=s), moving_mid),
                "middleOnOuter": rail_contact(moving_mid, outer),
            }
    buttons = {
        s: [
            button_contact(combined, s, *seat["xyMm"], pin["supplierButtonTopMm"])
            for seat in manufacturing["supports"]
        ]
        for s in STROKES
    }
    files = [
        shop / "stowed.step",
        shop / "place.step",
        shop / "review.json",
        shop / "part-frames.json",
        package / "receiver-clearance-kit.step",
        package / "kit.json",
        shelf_dir / "stow-receiver.step",
        shelf_dir / "installed.json",
        shelf_dir / "damper-fit.json",
        shelf_dir / "response.py",
    ]
    return {
        "sources": {p.relative_to(ROOT).as_posix(): digest(p) for p in files},
        "strokeMm": list(STROKES),
        "movingStock": masses,
        "contacts": contacts,
        "fixedButtons": buttons,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    path = parser.parse_args().output
    path.write_text(
        json.dumps(run(), indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
