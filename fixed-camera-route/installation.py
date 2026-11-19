"""Check mounting seats, panel ventilation, terminal travel and the cable carrier lane."""

import argparse
import math
import numpy as np
from pathlib import Path
from hardware import solids, service, WALL, RADIUS
from scene import get, write, CAD, sp1
from paths import points, routes
from harness_fit import nearest, edges, pair, sample_box
from routing import FIXED_X, MOVING_X, STROKE, RADIUS as CHAIN_R, MANAGED_LENGTH, power_feed


def run(artifacts, lamp):
    bodies, _ = get(artifacts, lamp)
    parts, anchors = solids()
    paths, board = routes()
    seat = {}
    for k, parent in anchors.items():
        gap = nearest(parts[k], bodies[parent])
        if "shoe" in k:
            # Yoke registers to both outside faces of the square tube.
            directions = [(0.1, 0, 0), (0, 0.1, 0)]
        else:
            directions = [(0, 0, -0.1)]
        volumes = [
            CAD.intersection(CAD.translated(parts[k], v), bodies[parent]) for v in directions
        ]
        seat[k] = {"substrate": parent, "gapMm": gap, "pressedContactMm3": volumes}
    # Preserve the complete ventilation allocation of the revised converter panel,
    # except the designed upfacing output for the fixed head.
    panel = sp1.native_box(np.array([-190, -202, 164, -55, -117, 320]))
    gaps = {}
    for k, p in paths.items():
        if k != "power":
            gaps[k] = min(nearest(e, panel) for e in edges(p, np.zeros(3))) - RADIUS[k] - WALL
    for k, s in parts.items():
        if k.startswith(("usb-A", "host-", "base-guide", "roof-guide")):
            gaps[k] = nearest(s, panel)
    # Lower return is fixed at x=-5. The half-speed moving turn reaches its most
    # forward at full stroke. The left semicircle lies aft of that center at every Z.
    turn = (FIXED_X + MOVING_X + STROKE + math.pi * CHAIN_R - MANAGED_LENGTH) / 2
    # igus links are 30.5 mm pitch. 157 mm envelope was inherited for the outside
    # turn; conservatively extend forward one full pitch. The lower fixed-link
    # receiver has modeled intentional entry, all higher moving material starts at Z>~788.
    bounds = [
        [-389.421, -341, 588.5, FIXED_X, -305, 611.5],
        [-389.421, -341, 588.5, turn + 30.5, -305, 831.5],
        [-389.421, -341, 788.5, 372.5, -305, 831.5],
    ]
    carrier = {}
    new_power = np.vstack([ps for _, ps in power_feed(-329)])
    for k, p in paths.items():
        pts = points(p)
        scan = pts
        if k == "roof-host":
            # The final unbent insertion enters the stationary open terminal only.
            # Exclude a 20 mm mouth region, inside the manufacturer mounting-detail acceptance.
            scan = pts[(pts[:, 2] < 580) | (pts[:, 0] > FIXED_X + 20)]
        gap = min(sample_box(scan, np.asarray(b)).min() for b in bounds) - RADIUS[k] - WALL - 0.5
        carrier[k] = {
            "externalLaneMm": float(gap),
            "otherFixedPowerMm": pair(pts, new_power) - RADIUS[k] - WALL - 4.35 - 1.5,
        }
    for k, s in parts.items():
        carrier[k] = {"externalLaneMm": min(nearest(s, sp1.native_box(b)) for b in bounds)}
    # Actual plugs come out on different mating axes. Other installed jackets
    # remain present. The serviced lead and its open guides are removed together.
    leave = {
        "fixed-C-data": "data",
        "fixed-C-power": "pigtail",
        "host-A-fixed": "data",
        "host-A-roof": "roof-host",
    }
    withdrawal = {}
    for n, sw in service().items():
        ds = {}
        for k, p in paths.items():
            if k == leave[n]:
                continue
            jacket_only = n.startswith("host-A") and k in ("roof-host", "data")
            ds[k] = (
                min(nearest(sw, e) for e in edges(p, np.zeros(3)))
                - RADIUS[k]
                - (0 if jacket_only else WALL)
            )
        withdrawal[n] = ds
    overall = []
    for k, p in paths.items():
        ps = points(p)
        r = RADIUS[k] + WALL
        overall.append(np.r_[ps.min(0) - r, ps.max(0) + r])
    overall.extend(np.asarray(CAD.bounds(v)) for v in parts.values())
    return {
        "bearingSeats": seat,
        "converterPanelReservationMm": gaps,
        "continuousCarrier": carrier,
        "withdrawalToOtherRoutesMm": withdrawal,
        "fullInstallBoundsMm": np.r_[
            np.min(overall, axis=0)[:3], np.max(overall, axis=0)[3:]
        ].tolist(),
        "lowerCarrierMouthServiceZoneMm": 20,
        "sourceMotion": "fixed return / half-speed turning arc / moving upper run",
        "physicalReceiving": None,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for k in ("artifacts", "lamp", "out"):
        p.add_argument(k, type=Path)
    a = p.parse_args()
    write(a.out, run(a.artifacts, a.lamp))
