"""Body-bearing staggered guide shoes; separate vertical load from positioning."""

import argparse
import itertools
import json
from pathlib import Path
from OCP.gp import gp_Trsf
from OCP.TopLoc import TopLoc_Location
from attachments import base, common, cut, cyl, dist, fuse, write_step
from installed_audit import HERE, ROOT, sha
from clearance import hanging_inventory, move


def stock():
    carrier = base.block((285, -29, 459, 315, 29, 484))
    # Cleared channels open at both ends for assembly. Nine mm across the
    # captured solid section remains after cutting the stop slot near the tip.
    shapes = {}
    for name, sgn, x in (
        ("Positive bearing shoe", 1, 293),
        ("Negative bearing shoe", -1, 307),
    ):
        slot = base.block((x - 5.8, -30, 467.95, x + 5.8, 30, 480.05))
        carrier = cut(carrier, slot)
        a, b = sorted((-29 * sgn, 24 * sgn))
        shoe = base.block((x - 5.75, a, 468, x + 5.75, b, 480))
        a, b = sorted((-24 * sgn, 12 * sgn))
        rad = 1.55
        opening = fuse(
            base.block((x - rad, a, 467.9, x + rad, b, 480.1)),
            cyl(x, a, 467.8, rad, 12.4),
        )
        opening = fuse(opening, cyl(x, b, 467.8, rad, 12.4))
        shapes[name] = cut(shoe, opening)
        # Ø3 dowel bridges the housing above and below the moving rail.
        carrier = cut(carrier, cyl(x, 12 * sgn, 458, 1.50, 27))
        shapes[name + " stop pin"] = cyl(x, 12 * sgn, 459, 1.5, 25)
    shapes["Shoe dovetail carrier"] = carrier
    return shapes


def run(field):
    cage_map, natives = hanging_inventory(field)
    items = stock()
    p = HERE / "retained-shoes.step"
    write_step(items, p)
    out = {}
    for side, key, sgn in (("red", "NAUO260", -1), ("blue", "NAUO263", 1)):
        station = cage_map[key]
        xx, zz = station["axis_mm"]
        t = gp_Trsf()
        t.SetValues(sgn, 0, 0, 300 - sgn * xx, 0, 0, -sgn, sgn * zz, 0, 1, 0, 0)
        cage = natives[station["cage"]].Moved(TopLoc_Location(t))
        rr = {}
        for title, z, spread in (
            ("raised-closed", 328, 0),
            ("raised-open", 328, 36),
            ("seated", 319.4, 36),
            ("early", 317, 36),
        ):
            check = {}
            for k, s in items.items():
                ss = (
                    1
                    if k == "Positive bearing shoe"
                    else -1
                    if k == "Negative bearing shoe"
                    else 0
                )
                pos = move(s, (0, ss * spread, z))
                ov = common(pos, cage)
                gap = dist(pos, cage)
                row = dict(gapMm=gap, intersectionMm3=ov)
                if ss and title == "seated":
                    row["downwardBearingPatchMm2"] = (
                        common(cage, move(pos, (0, 0, -0.1))) / 0.1
                    )
                check[k] = row
            rr[title] = check
        out[side] = rr
    mechanical = {}
    for k, sgn in (("Positive bearing shoe", 1), ("Negative bearing shoe", -1)):
        pin = items[k + " stop pin"]
        shoe = items[k]
        carrier = items["Shoe dovetail carrier"]
        trials = {}
        for val in (0, 18, 36, 36.05, 36.15):
            aw = move(shoe, (0, sgn * val, 0))
            trials[str(val)] = dict(
                guideGapMm=dist(aw, carrier),
                pinGapMm=dist(aw, pin),
                pinOverlapMm3=common(aw, pin),
            )
        at = move(shoe, (0, sgn * 36, 0))
        # Relative upward motion of the shoe closes the upper guide roof, which
        # actually carries the upward field reaction into the central housing.
        for z in (0.04, 0.05, 0.10):
            shifted = move(at, (0, 0, z))
            trials[f"up{z}"] = dict(
                gapMm=dist(shifted, carrier), overlapMm3=common(shifted, carrier)
            )
        mechanical[k] = trials
    record = dict(
        fieldSha256=sha(field),
        sourceShoeStepSha256=sha(ROOT / "cad/shallow-cage/cage-crosshead.step"),
        retainedStepSha256=sha(p),
        railLengthMm=53,
        nominalGuideRoofOverlapMm=22,
        closedPinSlotCenterTravelMm=36,
        pressureAndBending="Use actual remaining section through the slot and the root of the machined guide. Thin loading on the original plate and an independently observed horizontal latch remain open.",
        results=out,
        mechanical=mechanical,
    )
    (HERE / "retained-shoes.json").write_bytes(
        (json.dumps(record, indent=2) + "\n").encode()
    )


if __name__ == "__main__":
    c = argparse.ArgumentParser()
    c.add_argument("field", type=Path)
    run(c.parse_args().field)
