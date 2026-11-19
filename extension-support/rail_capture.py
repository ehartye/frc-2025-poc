"""An opposed-pin desktop fixture for reacting extension pitch in both telescoping joints."""

import argparse
from pathlib import Path
import json
from rail_geometry import ROOT, exchange, read, digest, translated
from tray import cad, box, subtract, unite, write_step, finalize_step

# Coordinates in the nominal horizontal chassis, mm. The facing openings are
# through the 3 mm middle channel webs. Upper contact is at +0.35 mm per joint.
RADIUS = 2.0
RACE_RADIUS = 2.8
INNER_X, INNER_Z = -137.0, 467.0
OUTER_X, OUTER_Z = 40.0, 454.0
STROKE = 370.0


def cylinder(x, y, z, r, h):
    return cad.brep(["cyl", [x, y, z, 0, 1, 0, r, h]])


def slot(x0, x1, z, y):
    return unite(
        unite(
            box(x0, y, z - RACE_RADIUS, x1, y + 30, z + RACE_RADIUS),
            cylinder(x0, y, z, RACE_RADIUS, 30),
        ),
        cylinder(x1, y, z, RACE_RADIUS, 30),
    )


def stock():
    shop = ROOT / "design/competition-package/shop-duplex/red"
    kit = ROOT / "selected-match"
    refs = read(kit / "kit.json")
    original = exchange(
        shop / "stowed.step",
        read(shop / "review.json")["stowed"]["export_sha256"],
        read(shop / "part-frames.json"),
    )
    inner = exchange(
        kit / "receiver-clearance-kit.step", refs["kitSha256"], refs["volumeMm3"]
    )
    dz = 438.0 - cad.bounds(original["slide-L-outer"])[2]
    return {
        **{
            n: translated(original[n], z=dz)
            for side in ("L", "R")
            for n in (f"slide-{side}-outer", f"slide-{side}-middle")
        },
        **{n: v for n, v in inner.items() if n.endswith("-inner")},
    }


def machine(side, y, rails):
    ni, nm, no = (
        f"drilled-slide-{side}-inner",
        f"slide-{side}-middle",
        f"slide-{side}-outer",
    )
    axle = cylinder(INNER_X, y - 13.65, INNER_Z, RADIUS, 27.3)
    rings = {}
    for sign in (-1, 1):
        # Two split keeper rings bear against the INSIDE web faces (|Y|=7).
        # Either end of the axle can still engage its own slot after the
        # opposing ring takes up the nominal axial float.
        lo = y + sign * 6.625 - 0.225
        relief = subtract(
            cylinder(INNER_X, lo - 0.025, INNER_Z, 2.4, 0.50),
            cylinder(INNER_X, lo - 0.1, INNER_Z, 1.75, 0.7),
        )
        axle = subtract(axle, relief)
        ring = subtract(
            cylinder(INNER_X, lo, INNER_Z, 2.95, 0.45),
            cylinder(INNER_X, lo - 0.1, INNER_Z, 1.78, 0.65),
        )
        # Model the open clip; the section and spring material are prototype inputs.
        ring = subtract(
            ring,
            box(INNER_X - 0.7, lo - 0.1, INNER_Z, INNER_X + 0.7, lo + 0.6, INNER_Z + 4),
        )
        rings[f"axle-clip-{side}-{sign:+d}"] = ring
    small = subtract(rails[ni], cylinder(INNER_X, y - 15, INNER_Z, 2.03, 30))
    # The upper end of the long inner race has one extra mm at each end.
    middle = subtract(
        rails[nm], slot(INNER_X - 1, INNER_X + STROKE / 2 + 1, INNER_Z - 0.45, y - 15)
    )
    middle = subtract(
        middle, slot(OUTER_X - STROKE / 2 - 1, OUTER_X + 1, OUTER_Z + 0.45, y - 15)
    )
    outer = rails[no]
    dogs = {}
    for sign in (-1, 1):
        # The enlarged wall is integral trial stock. Smooth shanks reserve the
        # threaded M6 interface; a 4 mm nose reaches into the middle web only.
        cy = y + sign * 18
        band = sorted([cy, cy + sign * 4])
        outer = unite(
            outer,
            box(OUTER_X - 6, band[0], OUTER_Z - 6, OUTER_X + 6, band[1], OUTER_Z + 6),
        )
        hole = sorted([y + sign * 14.9, y + sign * 22.1])
        outer = subtract(
            outer, cylinder(OUTER_X, hole[0], OUTER_Z, 3.05, hole[1] - hole[0])
        )
        stem = sorted([y + sign * 11.35, y + sign * 15])
        shank = sorted([y + sign * 15, y + sign * 22])
        head = sorted([y + sign * 22, y + sign * 25.5])
        dog = unite(
            cylinder(OUTER_X, stem[0], OUTER_Z, RADIUS, stem[1] - stem[0]),
            cylinder(OUTER_X, shank[0], OUTER_Z, 3, shank[1] - shank[0]),
        )
        dogs[f"outer-dog-{side}-{sign:+d}"] = unite(
            dog, cylinder(OUTER_X, head[0], OUTER_Z, 4.5, head[1] - head[0])
        )
    return {
        ni: small,
        nm: middle,
        no: outer,
        f"inner-crosspin-{side}": axle,
        **rings,
        **dogs,
    }


def run(out):
    rails = stock()
    parts = {}
    for side, y in (("L", 180.0), ("R", -180.0)):
        parts.update(machine(side, y, rails))
    out.mkdir(exist_ok=True, parents=True)
    step = out / "rail-fixture.step"
    write_step(step, parts)
    finalize_step(step)
    reopened = cad.read_named(step, [{"id": k} for k in parts])
    cases = {}
    retention = {}
    for side in ("L", "R"):
        inner = reopened[f"drilled-slide-{side}-inner"]
        middle = reopened[f"slide-{side}-middle"]
        outer = reopened[f"slide-{side}-outer"]
        axle = reopened[f"inner-crosspin-{side}"]
        dogs = [k for k in reopened if k.startswith(f"outer-dog-{side}-")]
        for d in (-0.25, 0.25):
            clip = reopened[f"axle-clip-{side}-{(-1 if d < 0 else 1):+d}"]
            retention[f"{side}/{d}"] = {
                "installedCommonMm3": cad.intersection(clip, inner),
                "axiallyShiftedClipCommonMm3": cad.intersection(
                    cad.translated(clip, (0, d, 0)), inner
                ),
            }
        for stroke in (0.0, 185.0, 370.0):
            key = f"{side}/{stroke:g}"
            mid = translated(middle, x=stroke / 2)
            pin = translated(axle, x=stroke)
            moving_inner = translated(inner, x=stroke)
            # Relative displacements at each individual joint, with the other
            # stage fixed. The 0.01 mm overtravel is a geometric touch witness.
            witnesses = {}
            for dz in (-1.05, 0.0, 0.34, 0.36, 0.45):
                moved_mid = translated(mid, z=dz)
                witnesses[f"{dz:g}"] = {
                    "innerPinToMiddleMm3": cad.intersection(translated(pin, z=dz), mid),
                    "middleToOuterDogsMm3": [
                        cad.intersection(moved_mid, reopened[k]) for k in dogs
                    ],
                    "innerFloorToMiddleMm3": cad.intersection(
                        translated(moving_inner, z=dz), mid
                    ),
                    "middleFloorToOuterMm3": cad.intersection(moved_mid, outer),
                }
            # Longitudinal slots and fixed cross sections give continuous pin
            # clearance over the stroke, including the rounded end approach.
            cases[key] = witnesses
    volumes = {k: cad.volume(v)[0] for k, v in reopened.items()}
    delta = {k: volumes[k] - cad.volume(v)[0] for k, v in rails.items()}
    result = {
        "stepSha256": digest(step),
        "volumesMm3": volumes,
        "machinedDeltaMm3": delta,
        "movingStock": {
            key: {
                "stage": "middle" if key.endswith("-middle") else "inner",
                "massKg": vol * (2700.0 if key in rails else 7850.0) * 1e-9,
                "stowedCenterXMm": cad.volume(reopened[key])[1][0],
            }
            for key, vol in volumes.items()
            if not key.endswith("-outer") and not key.startswith("outer-dog")
        },
        "idealUpperTravelFromNominalMm": 0.35,
        "lowerFloorTravelFromNominalMm": 1.0,
        "innerCrosspinLocalXMm": INNER_X,
        "outerDogFixedXMm": OUTER_X,
        "nominalClipToWebAxialGapMm": 0.15,
        "retainingRingWitnesses": retention,
        "noseRadiusMm": RADIUS,
        "strokeMm": STROKE,
        "witnesses": cases,
        "modelScope": "nominal rail fixture; split-ring capture with smooth thread reservations; material, thread and groove capacity remain design inputs",
    }
    (out / "keepers.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("out", type=Path)
    run(p.parse_args().out)
