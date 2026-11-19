"""Machine two passive stow landing lands and registered axial absorber mount holes."""

import argparse
import json
import math
from pathlib import Path
import sys

from tray import (
    HERE,
    ROOT,
    FACE,
    TRAY,
    SHELF,
    TOP,
    box,
    rounded,
    common,
    cad,
    digest,
    distance,
    volume,
    unite,
    subtract,
    write_step,
    finalize_step,
)
from damper_fit import cylinder, envelope

sys.path.insert(0, str(ROOT / "selected-match"))
from rail_profile import prism_y
from OCP.BRepCheck import BRepCheck_Analyzer

STATIONS = [(118.0, -66.0), (118.0, 42.0)]
# Deep pockets only in the outboard polymer overhang, outside both Y180 support
# lines. The six camera bosses and the two saddle lands are in the retained band.
SIDE_POCKETS = [
    (a, b, c, d)
    for a, c in [(-95.0, -34.0), (40.0, 156.0)]
    for b, d in [(-207.5, -191.5), (191.5, 207.5)]
]
GAP = 0.1


def parts():
    issued = json.loads((HERE / "native.json").read_bytes())
    if digest(HERE / "shelf-insert.step") != issued["kitSha256"]:
        raise ValueError("Different candidate stock")
    source = cad.read_named(
        HERE / "shelf-insert.step", [{"id": k} for k in (FACE, TRAY)]
    )
    old = json.loads((ROOT / "selected-match/kit.json").read_bytes())
    kit = cad.read_named(
        ROOT / "selected-match/receiver-clearance-kit.step",
        [{"id": k} for k in old["volumeMm3"]],
    )
    bridge = kit["slide-base-bridge"]
    face, insert = source[FACE], source[TRAY]
    for rect in SIDE_POCKETS:
        face = subtract(face, rounded(rect, 484.0, 496.0, 3.0))
    # Original moving pressure-laminate envelope is 0.8 mm thick. Retain
    # a controlled 0.15 mm pocket clearance and recess the three nominal tops.
    for y in (-70.0, 0.0, 70.0):
        tail = box(124.85, y - 2.65, 499.2, 155.15, y + 2.65, 501.0)
        head = cylinder(155.0, y, 499.2, 3.75, 1.8)
        face = subtract(face, unite(tail, head))
    for x, y in STATIONS:
        # Buttress the pad underneath the constant PE cover in the existing open cell.
        insert = unite(insert, rounded((x - 5, y - 5, x + 5, y + 5), 485.0, TOP, 2.0))
        # No leading edge hits the free button. A 46 mm ramp lifts the old shelf
        # underside 0.8 mm behind this stop. The angle is below one degree.
        path = [
            (-101.0, 484.0),
            (x - 3.0, 484.0),
            (x - 3.0, 485.0),
            (x - 49.0, 485.8),
            (-101.0, 485.8),
        ]
        cut = prism_y(path, y - 5.0, 10.0)
        face = subtract(face, cut)
        insert = subtract(insert, cut)
        bridge = subtract(bridge, cylinder(x, y, 432, 4.15, 7))
    return (
        {FACE: face, TRAY: insert, "slide-base-bridge": bridge},
        kit[SHELF],
        source,
        kit["slide-base-bridge"],
    )


def run(out):
    source, old, prior, old_bridge = parts()
    out.mkdir(parents=True, exist_ok=True)
    path = out / "stow-receiver.step"
    write_step(path, source)
    finalize_step(path)
    new = cad.read_named(path, [{"id": k} for k in source])
    for s in new.values():
        if cad.solid_count(s) != 1 or not BRepCheck_Analyzer(s).IsValid():
            raise ValueError("Bad manufacturing solid")
    assembly = unite(new[FACE], new[TRAY])
    removed = volume(old) - volume(new[FACE])
    al = volume(new[TRAY])
    checks = []
    for x, y in STATIONS:
        pad = cylinder(x, y, 485.0, 3.2, 0.03)
        collar = cylinder(x, y, 433.0, 10 / math.sqrt(3), 0.03)
        collar = subtract(collar, cylinder(x, y, 432, 4.15, 2))
        checks.append(
            {
                "xyMm": [x, y],
                "tipLandingFraction": volume(common(pad, new[TRAY])) / volume(pad),
                "lowerNutBoundingRingAtBridgeFraction": volume(
                    common(collar, new["slide-base-bridge"])
                )
                / volume(collar),
                "freeTipToUndeformedLandMm": distance(
                    envelope(x, y, 485.0 - GAP), assembly
                ),
            }
        )
    return {
        "upstreamShelfStep": digest(HERE / "shelf-insert.step"),
        "parentReceiver": digest(ROOT / "selected-match/receiver-clearance-kit.step"),
        "manufacturingSha256": digest(path),
        "volumeMm3": {k: volume(v) for k, v in new.items()},
        "oldShelfVolumeMm3": volume(old),
        "removedPeMm3": removed,
        "bridgeDrillMm3": volume(old_bridge) - volume(new["slide-base-bridge"]),
        "polymerRoofStockAtOrAbove496LostMm3": volume(
            subtract(common(old, box(-101, -211, 496, 166, 211, 501)), new[FACE])
        ),
        "receiverEscapeMm3": volume(subtract(assembly, old)),
        "resultFaceInsertInterferenceMm3": volume(common(new[FACE], new[TRAY])),
        "outboardPeReliefMm3": volume(prior[FACE]) - volume(new[FACE]),
        "supports": checks,
        "pressureLaminates": {
            "source": digest(ROOT / "release-observer/build_mount.py"),
            "headCentersXyMm": [[155, -70], [155, 0], [155, 70]],
            "zShiftMm": -0.8,
            "nominalTopMm": 500,
            "newPhysicalPressureThreshold": None,
        },
        "fullReceiverMachinedDeltaKg": [
            (al * 2650 - removed * 980) * 1e-9,
            (al * 2850 - removed * 930) * 1e-9,
        ],
        "buyerUpperKg": {
            "twoMA30EUMAt15g": 0.03,
            "fourKM8At2.5g": 0.010,
            "twoWitnessAndRetentionAllowancesAt2g": 0.004,
        },
        "actualWeighedDelta": None,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("out", type=Path)
    a = p.parse_args()
    (a.out / "installed.json").write_text(
        json.dumps(run(a.out), indent=2) + "\n", encoding="utf8", newline="\n"
    )
