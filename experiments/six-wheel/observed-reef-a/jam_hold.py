"""Continue the same pipe through the finite wheel brake with both belt surfaces jammed."""

import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import numpy as np

from retention import (
    HERE,
    ROOT,
    nav,
    path_for,
    feed,
    Shelf,
    GUIDE,
    CompliantPipe,
    momentum,
    fictitious,
)
from field_screen import brake, DRIVE


def simulate(assets, row):
    side = row["wheelCase"]
    source = assets / "observed-reef-a" / (row["id"] + ".npy")
    if nav.sha(source) != row["trajectorySha256"]:
        raise ValueError("Wrong jammed pipe")
    phase = np.load(source, mmap_mode="r")[-1]
    incoming = next(
        r
        for r in nav.read(ROOT / "conditioned-match/feeds.json")
        if r["trajectorySha256"] == row["sourceFeedSha256"]
    )
    feed.cfg = replace(feed.cfg, slices=10)
    pipe = CompliantPipe(
        incoming["sourceRobotMmDeg"],
        0,
        replace(GUIDE, sectors=128, radial_bins=4),
        Shelf(stations=16),
    )
    pipe.p, pipe.a, pipe.v = phase[1:4].copy(), phase[4:7].copy(), phase[7:10].copy()
    pipe.l = momentum(pipe.a, phase[10:13])
    pipe.touch = ["jammed drive, same pipe"]
    t, pose, _ = path_for(assets, side, "nominal")
    cfg = nav.read(ROOT / "station-landmark/routes.json")["cases"][side]
    tick = 21.571672635314284 + row["firstFault"][0] / 1000
    t, pose, stop = brake(t, pose, tick - DRIVE, cfg)
    # Include a stationary settling interval after both wheels reach zero.
    t = np.r_[t, t[-1] + 0.005, t[-1] + 0.25]
    pose = np.vstack((pose, pose[-1], pose[-1]))
    rates = np.column_stack(
        [np.gradient(col, t) for col in (pose[:, 0], pose[:, 1], np.unwrap(pose[:, 2]))]
    )
    acc = np.column_stack([np.gradient(col, t) for col in rates.T])
    c, s = np.cos(pose[:, 2]), np.sin(pose[:, 2])
    motion = np.column_stack(
        (
            c * acc[:, 0] + s * acc[:, 1],
            -s * acc[:, 0] + c * acc[:, 1],
            rates[:, 2],
            acc[:, 2],
        )
    )
    wall = row["finalWallS"]
    end = DRIVE + t[-1]
    dt = row["stepS"]
    min_gap = math.inf
    while wall < end - 1.0e-10 and pipe.hard is None and not pipe.loss:
        h = min(dt, end - wall)
        ax, ay, w, alpha = [np.interp(wall - DRIVE, t, col) for col in motion.T]
        fictitious(pipe, np.array([ax, ay, 0.0]), w, alpha, h)
        pipe.moves(0.0, h)
        min_gap = min(
            min_gap,
            1000
            * (
                0.625
                - pipe.p[2]
                - feed.HALF * abs(pipe.a[2])
                - feed.R * math.sqrt(1 - pipe.a[2] ** 2)
            ),
        )
        wall += h
    return {
        "wheelCase": side,
        "piece": row["piece"],
        "jammedSourceSha256": row["trajectorySha256"],
        "startWallS": row["finalWallS"],
        "wheelsStoppedWallS": stop["stoppedWallS"],
        "lastWallS": wall,
        "endPositionMm": (1000 * pipe.p).tolist(),
        "negativeGuideOrCartridgeStop": pipe.hard,
        "lostPipe": pipe.loss,
        "minGuideGapMm": min_gap,
        "independentPostTripOpticalAuthorization": False,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets", type=Path)
    a = p.parse_args()
    rows = [
        simulate(a.assets, r)
        for r in nav.read(HERE / "quantized-stuck-cargo-nominal-active.json")
    ]
    (HERE / "jam-settle.json").write_text(
        json.dumps(rows, indent=2) + "\n", encoding="utf8", newline="\n"
    )
