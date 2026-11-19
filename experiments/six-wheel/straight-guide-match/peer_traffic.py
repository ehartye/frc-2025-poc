"""Check the other ten robot pairs, plus their field and stationary-stock reservations."""

import argparse
from itertools import combinations
import json
import math
from pathlib import Path
import numpy as np
from shared_clock import ROOT, HERE, read
from field_screen import nav
from field import screen


def run():
    plans = read(ROOT / "fixed-height-match/placements.json")["placements"]
    field, _, team, _ = nav.route.route.field.scenario(plans, inboard=True)
    stock = read(ROOT / "full-match/installed-stock.json")["stock"]
    others = {k: v for k, v in team.items() if k != "red2"}
    rows = {}
    t = np.arange(0, 153.0001, 0.01)
    for key, driver in others.items():
        poses = np.array([driver.at(max(0, s - 18))[0] for s in t])
        poses[:, 2] = np.unwrap(poses[:, 2])
        other_programs = {k: v for k, v in others.items() if k != key}
        r = screen(field, t, poses, stock, other_programs, 153)
        rows[key] = {
            "barrierMm": r["minimumBarrierMm"],
            "stockMm": r["minimumStockMm"],
            "peersMm": r["peerMm"],
            "nearestStock": min(r["stockMm"], key=r["stockMm"].get),
            "nearestBarrier": min(r["barriersMm"], key=r["barriersMm"].get),
        }
    return {
        "stepS": 0.01,
        "stationaryUntilWallS": 18,
        "assumedDimensionsMm": [850, 850, 1066.8],
        "cases": rows,
        "otherPairLowerBoundsMm": {
            a + " / " + b: min(rows[a]["peersMm"][b], rows[b]["peersMm"][a])
            for a, b in combinations(others, 2)
        },
    }


if __name__ == "__main__":
    (HERE / "peer-traffic.json").write_text(
        json.dumps(run(), indent=2) + "\n", encoding="utf8", newline="\n"
    )
