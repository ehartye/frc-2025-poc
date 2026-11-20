"""Register the Anderson SB50 family casting to the battery tray and resolved mounts."""

import argparse
import math
from pathlib import Path

from OCP.STEPControl import STEPControl_Reader
from attachments import (
    base,
    bounds,
    common,
    cut,
    cyl,
    dist,
    fuse,
    hexagon,
    tube,
    write_step,
)
from battery_fit import collisions
from clearance import move, evaluate
from cold_issue import precise
from installed_audit import sha
from issue import document
from receiver_mechanics import rotate
from receiver_motion import assembly, OUT

SOURCE = "b5a233b9a0c845159b742b22e9a8e6158a2644bfc5989308ecbf71ad3cccc32e"
X = -284.0
FIXED = "SB50-fixed-housing"
CABLE = "SB50-battery-housing"


def registration(vendor):
    if sha(vendor) != SOURCE:
        raise ValueError("Different Anderson SB50 revision")
    reader = STEPControl_Reader()
    reader.ReadFile(str(vendor))
    reader.TransferRoots()
    (shell,) = list(base.subsolids(reader.OneShape()))
    bb = precise(shell)
    fixed = move(rotate(shell, (0, 0, 1), math.pi), (X, -40.65 + bb[4], 156 - bb[2]))
    bb = precise(fixed)
    datum = (X, 0, (bb[2] + bb[5]) / 2)
    mate = rotate(fixed, (1, 0, 0), math.pi, datum)
    return fixed, mate, bb


def features(parts, fixed):
    screws = {}
    tools = {}
    host = parts["host-drilled-swerve-deck"]
    tray = parts["battery-tray"]
    shell = base.block([X - 18.26, -40.65, 156.0, X + 18.26, -0.125, 172.05])
    for i, (y, d, h) in enumerate(((-34.30, 6.0, 1.889), (-15.25, 6.0, 1.127))):
        shell = cut(shell, cyl(X, y, 155.9, 1.829, 20))
        shell = cut(shell, cyl(X, y, 155.9, 3.05, 155 + h - 155.9))
        shell = cut(shell, cyl(X, y, 168.827, 4.05, 6))
        host = cut(host, cyl(X, y, 149.9, 1.70, 5.2))
        tray = cut(tray, cyl(X, y, 153.9, 1.70, 1.2))
        spacer = tube(X, y, 155, d, 3.4, h)
        screw = fuse(cyl(X, y, 143.827, 1.5, 25), cyl(X, y, 168.827, 2.75, 3))
        screw = cut(screw, hexagon(X, y, 170.4, 2.5, 2.0))
        washer = tube(X, y, 149.5, 7, 3.4, 0.5)
        nut = cut(hexagon(X, y, 145.5, 5.5, 4), cyl(X, y, 145.4, 1.5, 4.2))
        k = "SB50-base-" + str(i)
        screws.update(
            {
                k + "-spacer": spacer,
                k + "-M3x25": screw,
                k + "-washer": washer,
                k + "-locknut": nut,
            }
        )
        tools[k + "-top-key"] = cyl(X, y, 171.827, 1.55, 35)
        tools[k + "-below-socket"] = tube(X, y, 100, 8.5, 6.45, 49.45)
    # Public CAD shows allocated envelopes for these separate bought items.
    # Independent evidence uses the authorized vendor casting above.
    free = base.block([X - 18.26, 0.125, 156.0, X + 18.26, 40.9, 172.05])
    screws.update(
        {
            FIXED: shell,
            CABLE: free,
            "battery-tray": tray,
            "host-drilled-swerve-deck": host,
        }
    )
    return screws, tools


def run(vendor):
    _, installed, _, _ = assembly()
    fixed, mate, b = registration(vendor)
    delta, tools = features(installed, fixed)
    geometry = installed | delta
    true = dict(geometry)
    true[FIXED] = fixed
    true[CABLE] = move(mate, (0, 0.25, 0))
    box = {k: bounds(s) for k, s in true.items()}
    pairs = {}
    seats = {}
    for k, s in delta.items():
        if k in ("battery-tray", "host-drilled-swerve-deck"):
            continue
        pairs[k] = collisions(true[k], true, box, {k})
    gauges = {k: collisions(s, true, box, set()) for k, s in tools.items()}
    for i in (0, 1):
        k = "SB50-base-" + str(i)
        bearing = delta[k + "-spacer"]
        screw = delta[k + "-M3x25"]
        seats[k] = dict(
            spacerToActualShellMm=dist(bearing, fixed),
            spacerNominalMm3=common(bearing, fixed),
            spacerPressMm2=common(move(bearing, (0, 0, 0.01)), fixed) / 0.01,
            capToActualShellMm=dist(screw, fixed),
            capNominalMm3=common(screw, fixed),
            capPressMm2=common(move(screw, (0, 0, -0.01)), fixed) / 0.01,
            sleeveToTrayMm2=common(move(bearing, (0, 0, -0.01)), delta["battery-tray"])
            / 0.01,
            washerToDeckMm2=common(
                move(delta[k + "-washer"], (0, 0, 0.01)),
                delta["host-drilled-swerve-deck"],
            )
            / 0.01,
        )
    peers = [(k, s, box[k]) for k, s in true.items() if k not in (FIXED, CABLE)]
    withdraw = evaluate(true[CABLE], peers, (0, 1, 0), 0, 24.7)
    step = OUT / "battery-disconnect.step"
    write_step(delta, step)
    document(
        OUT / "battery-disconnect.json",
        dict(
            supplierSha256=SOURCE,
            publicDrawingSha256=sha(step),
            supplier="Anderson SB-50-Std-02-08-STEP, published with 992G1. Neutral family shell has one solid, contact springs and keyed red revision require separate receiving checks.",
            sourceBoundsMm=b,
            fitAtCatalogPairLengthMm3=common(fixed, mate),
            fitAtQuarterMmLongerMm3=common(fixed, true[CABLE]),
            ringSeating=seats,
            nominalIntersectionsMm3={k: v for k, v in pairs.items() if v},
            bothTools=gauges,
            fullWithdrawal=withdraw,
            scope="Robot-side housing is retained by independent seats and through screws. Loose half is a disconnect-position envelope on this revision. Cable copper, boots, fitted routing and continuity are separate; density of the rectangular public envelopes does not determine product mass.",
        ),
    )
    print(
        "Supplier interfaces",
        seats,
        {k: v for k, v in pairs.items() if v},
        gauges,
        withdraw["adverse"],
        flush=True,
    )


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("vendor", type=Path)
    run(ap.parse_args().vendor)
