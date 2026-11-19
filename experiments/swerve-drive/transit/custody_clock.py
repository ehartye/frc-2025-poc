"""Service actual stereo observations in order and return the first irrevocable brake tick."""

import argparse
import math
from pathlib import Path

import numpy as np

from register import prior, read, sha, write
from transit_images import backgrounds, observation
import receiver_witness as camera


def run(raw, route_folder, pipe_folder, field, out, case, step, latency=0.015):
    _, before, _, original = prior(raw, case, step)
    route = read(route_folder / "route.json")
    contact = read(pipe_folder / "contact.json")
    pipe_path = pipe_folder / contact["file"]
    wheel_path = route_folder / "path.npz"
    if not (
        sha(pipe_path) == contact["sha256"]
        and sha(wheel_path) == contact["sourceRoute"] == route["trajectory"]
        and contact["serviceState"] == route["sourcePipe"] == sha(original)
        and contact["piece"] == before["pipe"]["piece"]
    ):
        raise ValueError("Wheel, tube or identity registration differs")
    if contact["hardStop"] or contact["lost"]:
        raise ValueError("Source pipe reports a mechanical loss or stop")
    pipe = np.load(pipe_path)
    with np.load(wheel_path) as st:
        times, poses = st["relativeS"], st["stationMmMmRad"]
    view = backgrounds(field)
    masks = camera.load_wheel_mask(raw / "wheel-visibility.json")
    start = route["startWallS"]
    last = before["vision"]["frames"][-1]
    next_stamp = last["captureWallS"] + 0.017
    if not last["accepted"] or not 0.0 < next_stamp - start <= 0.017001:
        raise ValueError("Prior optical clock does not cover this transition")
    previous = before["vision"]["frames"][-4:]
    rows = []
    tick = round(start * 200)
    final_tick = math.floor((start + min(pipe[-1, 0], times[-1])) * 200 + 1e-8)
    while tick <= final_tick:
        now = tick / 200
        while next_stamp <= now + 1e-10:
            rel = next_stamp - start
            x = np.array([np.interp(rel, pipe[:, 0], v) for v in pipe[:, 1:7].T])
            q = np.array([np.interp(rel, times, v) for v in poses.T])
            source = observation(
                view, masks, q, 1000 * x[:3], x[3:] / np.linalg.norm(x[3:])
            )
            rows.append(dict(captureWallS=next_stamp, **source))
            next_stamp += 0.017
        received = [
            r for r in [*previous, *rows] if r["captureWallS"] + latency <= now + 1e-10
        ]
        if (
            not received
            or not received[-1]["accepted"]
            or now - received[-1]["captureWallS"] > 0.040000001
        ):
            break
        tick += 1
    held = tick <= final_tick
    write(
        out,
        dict(
            case=case,
            stepUs=step,
            piece=contact["piece"],
            sourceRoute=sha(wheel_path),
            sourcePipe=sha(pipe_path),
            startWallS=start,
            decisionWallS=min(tick, final_tick) / 200,
            decision="brake" if held else "end of contact evidence",
            frames=rows,
            frameLagS=latency,
            wheelMask=sha(raw / "wheel-visibility.json"),
        ),
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("raw", "route", "pipe", "field", "out"):
        p.add_argument(name, type=Path)
    p.add_argument("case", choices=("plus", "minus"))
    p.add_argument("step", type=float)
    a = p.parse_args()
    run(a.raw, a.route, a.pipe, a.field, a.out, a.case, a.step)
