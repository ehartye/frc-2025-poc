"""Machine matched case stops and a height-limited insulating keeper on the issued chassis."""

import argparse
import json
from pathlib import Path

from attachments import (
    base,
    bounds,
    box_gap,
    common,
    cut,
    cyl,
    dist,
    fuse,
    tube,
    volume,
    write_step,
)
from clearance import move
from installed_audit import HERE, reopen, sha
from issue import document

FROZEN = "4b23af084c0f95f7d47a61705973cdff7f0de85cc183cb388b26c5c12b44c9e0"
STATIONS = {
    "battery-cheek-left": [(-240, 84), (-184, 84)],
    "battery-cheek-right": [(-231, -113), (-184, -113)],
    "battery-side-stop-in": [(-259, -82), (-259, 53)],
    "battery-side-stop-out": [(-165, -82), (-165, 53)],
}
PACK = "np18-battery"
RODS = {"front": 86.0, "rear": -116.0}
KEEPER_BOTTOM = 324.80
LAMINATE = 4.7625
CX, CY = -212.0, -14.5


def box(*coordinates):
    return base.block(list(coordinates))


def stock_case(w, length, h, dx=0, dy=0):
    return box(
        CX - w / 2 + dx,
        CY - length / 2 + dy,
        155,
        CX + w / 2 + dx,
        CY + length / 2 + dy,
        155 + h,
    )


def stops(w, length):
    # Air gap per face at the measured matched case dimension. Bounds are checked
    # again independently for +/-0.10 mm machining and +/-0.05 mm measurement.
    half_w, half_l = w / 2 + 0.90, length / 2 + 0.90
    x0, x1 = CX - half_w, CX + half_w
    y0, y1 = CY - half_l, CY + half_l
    changes = {}
    for name, anchors in STATIONS.items():
        if name.endswith("cheek-left"):
            # Keep end gaps outside the side walls so the four angles assemble.
            foot = box(-250, y1, 155, -174, 94, 158)
            wall = box(-250, y1, 158, -174, y1 + 3, 200)
        elif name.endswith("cheek-right"):
            foot = box(-250, -122, 155, -174, y0, 158)
            wall = box(-250, y0 - 3, 158, -174, y0, 200)
        elif name.endswith("stop-in"):
            foot = box(-265, -103, 155, x0, 74, 158)
            wall = box(x0 - 3, -103, 158, x0, 74, 165)
        else:
            foot = box(x1, -103, 155, -159, 74, 158)
            wall = box(x1, -103, 158, x1 + 3, 74, 165)
        angle = fuse(foot, wall)
        if name.endswith("cheek-right"):
            # Local breaker keepout extends 1 mm beyond the registered case.
            angle = cut(angle, box(-325, -163, 154, -236, -114, 204))
        if "cheek" in name:
            y = RODS["front" if name.endswith("left") else "rear"]
            angle = cut(angle, cyl(CX, y, 154, 2.80, 6))
        for x, y in anchors:
            angle = cut(angle, cyl(x, y, 154, 2.25, 6))
            # Real cap-head outside radius 3.5; deliver usable head clearance
            # wherever the bought part approaches the inner angle corner.
            angle = cut(angle, cyl(x, y, 158, 3.85, 4.35))
        changes[name] = angle
    return changes


def made(parts):
    changes = stops(78, 183)
    # Nonconductive cover: clearance at rest; nuts bottom on two machined
    # sleeves, so fastening the cap cannot crush an unknown lower lid surface.
    top = KEEPER_BOTTOM + LAMINATE
    keeper = box(-224, -122, KEEPER_BOTTOM, -200, 92, top)
    keeper = fuse(keeper, box(-223, -101, KEEPER_BOTTOM, -182, 72, top))
    for y0, y1 in ((-40, -19), (5, 26)):
        keeper = fuse(keeper, box(-246, y0, KEEPER_BOTTOM, -180, y1, top))
    for end, y in RODS.items():
        keeper = cut(keeper, cyl(CX, y, KEEPER_BOTTOM - 1, 2.8, LAMINATE + 2))
        rod = "battery-" + end + "-tie-rod"
        bottom = bounds(parts[rod])[2]
        changes[rod] = cyl(CX, y, bottom, 2.5, 197.0)
        changes["battery-" + end + "-compression-sleeve"] = tube(
            CX, y, 158, 8.0, 5.4, KEEPER_BOTTOM - 158
        )
        for item in ("washer", "locknut"):
            name = f"battery-{end}-upper-{item}"
            changes[name] = move(parts[name], (0, 0, top - 327))
    changes["battery-hold-down"] = keeper
    return changes


def collisions(shape, parts, boxes, skip):
    test = bounds(shape)
    result = {}
    for k, other in parts.items():
        if k in skip or box_gap(test, boxes[k]) > 0.001:
            continue
        overlap = common(shape, other)
        if overlap > 0.01:
            result[k] = round(overlap, 4)
    return result


def audit(output):
    source = HERE / "issue" / "station-and-jack-interfaces.step"
    assert sha(source) == FROZEN
    manifest = json.loads((HERE / "issue" / "interfaces.json").read_bytes())
    parts = {k: s for k, *_, s in reopen()} | base.named(source)
    for name in manifest["fixedHeight"]["removed"]:
        parts.pop(name)
    original = parts.copy()
    changed = made(parts)
    output.mkdir(exist_ok=True, parents=True)
    neutral = output / "battery-retention.step"
    write_step(changed, neutral)
    parts.update(base.named(neutral))
    boxes = {k: bounds(s) for k, s in parts.items()}
    members = {k for k in parts if k.startswith("battery-") or k == PACK}
    contacts = {}
    for k in sorted(members):
        found = collisions(parts[k], parts, boxes, {k})
        contacts.update(
            {" / ".join(sorted([k, other])): v for other, v in found.items()}
        )
    receiving = {}
    for w, l, h in (
        (74, 179, 165),
        (75.2, 180, 165),
        (76.2, 181, 166),
        (77.2, 182, 167),
        (78, 183, 169),
    ):
        case = stock_case(w, l, h)
        cases = {}
        for size in ("interchangeable-maximum", "matched-to-measured"):
            jaws = changed if size.startswith("interchangeable") else stops(w, l)
            gap = {k: round(dist(case, jaws[k]), 4) for k in STATIONS}
            # Exact 0.10 mm penetration after consuming the measured free gap.
            axis = {
                "battery-cheek-left": (0, 1, 0),
                "battery-cheek-right": (0, -1, 0),
                "battery-side-stop-in": (-1, 0, 0),
                "battery-side-stop-out": (1, 0, 0),
            }
            pressure = {
                k: common(
                    move(case, tuple(z * (gap[k] + 0.10) for z in axis[k])), jaws[k]
                )
                for k in STATIONS
            }
            cases[size] = dict(
                gapMm=gap,
                pressWitnessMm3=pressure,
                guaranteedCenteredGapMm=min(gap.values()) - 0.10 - 0.05 / 2,
            )
        receiving[str([w, l, h])] = cases
    released = {PACK, "battery-hold-down"} | {
        k
        for k in parts
        if k.startswith("battery-")
        and ("-upper-" in k or k.endswith("compression-sleeve"))
    }
    service = {}
    for dz in (172, 190, 250):
        gauges = {
            "maximumCase": bounds(stock_case(78, 183, 169)),
            "sideGrip": [-173, -57, 242, -148, 28, 309],
            "upperAccessory": [-256, -111, 324, -168, 82, 374],
        }
        run = {}
        for name, (x0, y0, z0, x1, y1, z1) in gauges.items():
            run[name] = {
                key: collisions(box(*extent), parts, boxes, released)
                for key, extent in {
                    "up": [x0, y0, z0, x1, y1, z1 + dz],
                    "outPlusY": [x0, y0, z0 + dz, x1, 426 + y1 - y0, z1 + dz],
                    "outMinusY": [x0, -426 - (y1 - y0), z0 + dz, x1, y1, z1 + dz],
                }.items()
            }
        service[str(dz)] = run
    keeper_tool = {}
    for end, y in RODS.items():
        k = f"battery-{end}-upper-locknut"
        z = top = KEEPER_BOTTOM + LAMINATE + 1
        wrench = tube(CX, y, z, 13, 10, 55)
        keeper_tool[end] = collisions(wrench, parts, boxes, released | {k})
    table = {}
    for k, s in changed.items():
        old = volume(original[k]) if k in original else 0.0
        v = volume(s)
        old_density = 2850 if k in STATIONS or k == "battery-hold-down" else 8050
        new_density = (
            2000
            if k == "battery-hold-down"
            else (2850 if k in STATIONS or k.endswith("compression-sleeve") else 8050)
        )
        table[k] = {
            "oldMm3": old,
            "newMm3": v,
            "upperDeltaKg": (v * new_density - old * old_density) * 1e-9,
        }
    result = dict(
        sourceSha256=FROZEN,
        drawingSha256=sha(neutral),
        installedSolids=len(parts),
        caseBoundsMm=dict(width=[74, 78], length=[179, 183], guardHeight=[165, 169]),
        assumedHorizontalMachiningMm=0.10,
        assumedCaseMeasurementMm=0.05,
        inspectionDeltaKg=sum(q["upperDeltaKg"] for q in table.values()),
        installedChanges=table,
        installedIntersectionsMm3=contacts,
        caseFit=receiving,
        keeperUndersideZmm=KEEPER_BOTTOM,
        guardVerticalTravelBeforeContactMm=[KEEPER_BOTTOM - 324, KEEPER_BOTTOM - 320],
        keeperTopSockets=keeper_tool,
        service=service,
        limits="The top body in the precursor robot is a rectangular battery display. Flat-top, tab and pad footprints in that model do not qualify ABS bearing. The drawing only supplies the guard height and terminal positions. The width/length fit, continuous service gauges and machining travels are independent of the undimensioned lid. Dressed leads, keeper support inside manufacturer guard footprint and actual permitted bearing load require separate checks.",
    )
    document(output / "fit.json", result)
    print(
        json.dumps(
            dict(
                massChangeKg=result["inspectionDeltaKg"],
                installedHits=contacts,
                tools=keeper_tool,
                service250=service["250"],
            )
        )
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    audit(parser.parse_args().output)
