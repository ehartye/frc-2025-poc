"""Bound the fixed absorber reservations at both reef sectors and signed incoming docks."""

import argparse
import json
import math
from pathlib import Path
import numpy as np
from tray import HERE, ROOT, cad, digest
from installed import STATIONS, GAP
from damper_fit import envelope, cylinder
from clearance import (
    corrected,
    dock,
    heading_sweep,
    mod,
    native_box,
    native_pose,
    read,
    station_parts,
    stationary,
)
from field import check as stock_check


def run(field):
    registered = read("full-match/installed-stock.json")
    if digest(field) != registered["field_step_sha256"]:
        raise ValueError("Different field release")
    geom = {}
    for i, (x, y) in enumerate(STATIONS):
        geom[f"reservoir-{i}"] = envelope(x, y, 485 - GAP)
        for z in (430.0, 438.0):
            geom[f"KM8-{i}-{z:g}"] = cylinder(x, y, z, 10 / math.sqrt(3), 3.0)
    sweeps = {k: heading_sweep(np.array(cad.bounds(v)), 1.0) for k, v in geom.items()}
    pads = np.array([-11.526342, -11.526342, -1.526342, 11.526342, 11.526342, 1.526342])
    solids = {k: native_box(np.array(b) + pads) for k, b in sweeps.items()}
    reef = mod("shelf_fit_reef", ROOT / "moving-observer/clearance.py")
    structure = reef.reef_parts(field)
    layout = corrected()
    sectors = {}
    for name, sector in (("A", -0.18), ("B", 0.22)):
        near = reef.find_min(structure, native_pose(solids, sector * 1000))
        stock = stock_check(
            layout,
            registered["stock"],
            sweeps,
            stationary(layout.routes(sector)[1].end),
        )
        sectors[name] = {
            "expandedStructureMm": near["minimum_mm"],
            "stockQualifiedMm": min(v["lower_bound_mm"] for v in stock.values())
            - 11.526342,
        }
    station = station_parts(field)
    sb = {k: cad.bounds(v) for k, v in station.items()}
    trials = read("station-guide/optics-narrow.json")["trials"]
    incoming = {}
    for name in ("leftSmall", "rightSmall"):
        normal, y, yaw = trials[name]["finalStationMmDeg"]
        incoming[name] = dock.native_dock(geom, station, sb, 10 - normal, yaw, -y)
    return {
        "fieldSha256": digest(field),
        "manufacturingSha256": digest(HERE / "stow-receiver.step"),
        "twoStationDocks": incoming,
        "twoPlacementSectors": sectors,
        "assumption": "At the nominal mounting all new moving material lies inside the original shelf; field checks above are for the added fixed hardware.",
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("field", type=Path)
    p.add_argument("out", type=Path)
    a = p.parse_args()
    a.out.write_text(
        json.dumps(run(a.field), indent=2) + "\n", encoding="utf8", newline="\n"
    )
