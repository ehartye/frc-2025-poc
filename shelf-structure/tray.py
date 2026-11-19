"""Machine an underside structural insert while preserving the issued receiving face."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "axle-retention"))
from fit import cad, distance, finalize_step, subtract, unite, write_step
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepCheck import BRepCheck_Analyzer

SHELF = "receiver-R6-drilled-for-camera"
FACE = "r6-polymer-facing"
TRAY = "r6-receiver-insert"
BOTTOM = 485.0
TOP = 496.0
SKIN = 1.6
# A continuous 4 mm original polymer face and the peripheral stock remain.
# x0, y0, x1, y1. The four end lugs sit directly on the original two spacers.
OUTLINE = [(-93.0, -168.0, 153.0, 168.0)] + [
    (x - 10.0, y - 12.0, x + 10.0, y + 12.0)
    for x in (-80.0, 140.0)
    for y in (-176.0, 176.0)
]
# Parent mechanical interfaces that must retain full thickness down to Z485.
LANDS = [
    (29, -12, 51, 12),
    (89, -12, 111, 12),
    (-97, 51, -87, 61),
    (85, 51, 95, 61),
    (74, -46, 92, -34),
    (68, -134, 120, -120),
]
RIB_X = [-80, -45, -10, 25, 60, 92, 115, 140]
RIB_Y = [-165, -110, -55, 0, 55, 110, 165]
RIB = 3.0
RADIUS = 3.0
FIT = 0.3


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def box(x0, y0, z0, x1, y1, z1):
    return cad.brep(["box", [x0, y0, z0, x1 - x0, y1 - y0, z1 - z0]])


def rounded(rect, z, top, radius):
    x, y, u, v = rect
    shape = box(x + radius, y, z, u - radius, v, top)
    shape = unite(shape, box(x, y + radius, z, u, v - radius, top))
    for xx in (x + radius, u - radius):
        for yy in (y + radius, v - radius):
            shape = unite(
                shape, cad.brep(["cyl", [xx, yy, z, 0, 0, 1, radius, top - z]])
            )
    return shape


def common(a, b):
    op = BRepAlgoAPI_Common(a, b)
    if not op.IsDone():
        raise ValueError("Intersection failed")
    return op.Shape()


def volume(s):
    return cad.volume(s)[0]


def shape(rects, z, top, margin):
    blocks = []
    for x, y, u, v in rects:
        blocks.append(
            rounded(
                (x - margin, y - margin, u + margin, v + margin),
                z,
                top,
                RADIUS + margin,
            )
        )
    solid = blocks[0]
    for b in blocks[1:]:
        solid = unite(solid, b)
    return solid


def source():
    folder = ROOT / "selected-match"
    receipt = json.loads((folder / "kit.json").read_bytes())
    path = folder / "receiver-clearance-kit.step"
    if digest(path) != receipt["kitSha256"]:
        raise ValueError("Changed shelf revision")
    return cad.read_named(path, [{"id": k} for k in receipt["volumeMm3"]])[SHELF]


def manufacture(original):
    cavity = shape(OUTLINE, BOTTOM - 1, TOP, FIT)
    facing = subtract(original, cavity)
    blank = common(original, shape(OUTLINE, BOTTOM, TOP, 0))
    xs = [-93, *RIB_X, 153]
    ys = [-168, *RIB_Y, 168]
    pockets = []
    for x, u in zip(xs, xs[1:]):
        for y, v in zip(ys, ys[1:]):
            rect = (x + RIB / 2, y + RIB / 2, u - RIB / 2, v - RIB / 2)
            if rect[2] - rect[0] > 2 * RADIUS and rect[3] - rect[1] > 2 * RADIUS:
                pockets.append(rounded(rect, BOTTOM - 1, TOP - SKIN, RADIUS))
    # Protect the original sensor, two rack tabs, moving loom and SP1 feet.
    lands = [box(x, y, BOTTOM - 2, u, v, TOP) for x, y, u, v in LANDS]
    insert = blank
    for pocket in pockets:
        pb = cad.bounds(pocket)
        for land in lands:
            if cad.occupied(pb, cad.bounds(land)):
                pocket = subtract(pocket, land)
        insert = subtract(insert, pocket)
    return {FACE: facing, TRAY: insert}


def run(out):
    from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain

    original = source()
    items = manufacture(original)
    for k, s in items.items():
        cleaner = ShapeUpgrade_UnifySameDomain(s, True, True, False)
        cleaner.Build()
        items[k] = cleaner.Shape()
    out.mkdir(parents=True, exist_ok=True)
    native = out / "shelf-insert.step"
    write_step(native, items)
    finalize_step(native)
    parts = cad.read_named(native, [{"id": k} for k in items])
    for part in parts.values():
        if cad.solid_count(part) != 1 or not BRepCheck_Analyzer(part).IsValid():
            raise ValueError("Invalid machined part")
    face, tray = parts[FACE], parts[TRAY]
    stock = volume(original)
    removed = stock - volume(face)
    inserts = volume(tray)
    union = unite(face, tray)
    # Overlap volumes from thin material witnesses, not bounding rectangles.
    land_checks = {}
    strips = [(-95, -190, 155, -170), (-95, 170, 155, 190), *LANDS]
    for i, (x, y, u, v) in enumerate(strips):
        probe = box(x, y, BOTTOM, u, v, BOTTOM + 0.05)
        before = volume(common(probe, original))
        after = volume(common(probe, union))
        land_checks[str(i)] = {
            "originalAreaMm2": before / 0.05,
            "retainedAreaMm2": after / 0.05,
        }
    face_band = box(-102, -212, TOP, 168, 212, 501)
    old_top = common(original, face_band)
    new_top = common(face, face_band)
    return {
        "sourceSha256": digest(ROOT / "selected-match/receiver-clearance-kit.step"),
        "kitSha256": digest(native),
        "volumeMm3": {k: volume(v) for k, v in parts.items()},
        "originalVolumeMm3": stock,
        "cutPolymerMm3": removed,
        "sourceContainmentEscapeMm3": volume(subtract(union, original)),
        "interferenceMm3": volume(common(face, tray)),
        "facingToInsertGapMm": distance(face, tray),
        "topFourMmLostMm3": volume(subtract(old_top, new_top)),
        "mountingWitness": land_checks,
        "densityRangeKgPerM3": {"polymer": [930, 980], "aluminum": [2650, 2850]},
        "netMachinedKg": [
            (inserts * 2650 - removed * 980) * 1e-9,
            (inserts * 2850 - removed * 930) * 1e-9,
        ],
        "physicalMassKg": None,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", type=Path)
    a = parser.parse_args()
    r = run(a.out)
    (a.out / "native.json").write_text(
        json.dumps(r, indent=2) + "\n", encoding="utf8", newline="\n"
    )
