"""Carry a single identified pipe and all plate/plunger states on the measured-in-simulation drive."""

import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

from checkpoint import HERE, ROOT, coupled, digest, plant

sys.path.insert(0, str(ROOT / "relieved-match"))
from carry import fictitious


def restore(file, expected):
    if digest(file) != expected["stateSha256"]:
        raise ValueError("Changed station state")
    with np.load(file, allow_pickle=False) as d:
        pipe = plant(d["casePose"].tolist())
        np.testing.assert_array_equal(pipe.freq, d["freq"])
        modal_frame = json.loads((HERE / "mode-frame.json").read_bytes())
        actual = hashlib.sha256(
            np.asarray(pipe.table, dtype="<f8").tobytes()
        ).hexdigest()
        if modal_frame["tableSha256"] != actual:
            raise ValueError("Modal basis/sign or interpolation frame changed")
        for key in ("p", "a", "v", "l", "z", "zdot", "plunger"):
            setattr(pipe, key, d[key].copy())
        pipe.time = float(d["time"])
        pipe.belt = float(d["belt"])
        pipe.touch = json.loads(str(d["touch"]))
        # The stored first-contact times are diagnostic; resumed maxima begin at zero.
    return pipe


def run_case(job):
    folder, departure, route, dt = job
    side = departure["case"]
    pipe = restore(folder / "checkpoints" / (side + "-state.npz"), departure)
    path = folder / "routes" / (side + "-route.npz")
    if digest(path) != route["routeSha256"]:
        raise ValueError("Changed loaded route")
    saved = np.load(path)
    drive_wall = json.loads((ROOT / "relieved-match/first-feed.json").read_bytes())[
        "scheduledDriveWallS"
    ]
    lip = 21.671672635314284
    initial_wall = lip + pipe.time
    wait = drive_wall - initial_wall
    if not 0 < wait < 0.001:
        raise ValueError("Station and powered drive clocks disagree")
    t = np.r_[0.0, wait + saved["times"]]
    poses = np.vstack([saved["poses"][0], saved["poses"]])
    yaw = np.unwrap(poses[:, 2])
    rate = np.column_stack([np.gradient(c, t) for c in (poses[:, 0], poses[:, 1], yaw)])
    accel = np.column_stack([np.gradient(c, t) for c in rate.T])
    c, s = np.cos(yaw), np.sin(yaw)
    spatial = np.column_stack(
        (
            c * accel[:, 0] + s * accel[:, 1],
            -s * accel[:, 0] + c * accel[:, 1],
            np.zeros(len(t)),
            rate[:, 2],
            accel[:, 2],
        )
    )
    coupled.feed.cfg = replace(coupled.feed.cfg, slices=10)
    trace = []

    def record(now):
        trace.append(
            [
                now,
                *pipe.p,
                *pipe.a,
                *pipe.v,
                *pipe.angular(pipe.l),
                *pipe.z,
                *pipe.zdot,
                *pipe.plunger,
            ]
        )

    record(0.0)
    index = 0
    steps = math.ceil(t[-1] / dt)
    interval = round(0.001 / dt)
    for i in range(steps):
        now = i * dt
        step = min(dt, t[-1] - now)
        while index + 1 < len(t) - 1 and t[index + 1] <= now:
            index += 1
        fraction = (now - t[index]) / (t[index + 1] - t[index])
        q = spatial[index] + fraction * (spatial[index + 1] - spatial[index])
        fictitious(pipe, q[:3], q[3], q[4], step)
        pipe.moves(0.0, step)
        if (i + 1) % interval == 0 or pipe.hard or pipe.loss or i == steps - 1:
            record(now + step)
        if pipe.hard or pipe.loss:
            break
    raw = folder / "carry" / f"{side}-{round(dt * 1e9)}ns.npy"
    raw.parent.mkdir(parents=True, exist_ok=True)
    np.save(raw, np.asarray(trace))
    out = {
        "case": side,
        "stepS": dt,
        "stationStateSha256": departure["stateSha256"],
        "routeSha256": route["routeSha256"],
        "startWallS": initial_wall,
        "driveWallS": drive_wall,
        "endWallS": initial_wall + trace[-1][0],
        "zeroBeltMps": pipe.belt,
        "caseEndS": trace[-1][0],
        "routeEndS": float(t[-1]),
        "forecastBeyondCameraInterlock": True,
        "mechanicalStop": pipe.hard,
        "loss": pipe.loss,
        "peakRollerCartridgeMm": float(pipe.peak),
        "localPipeCompressionMm": 1000 * pipe.peak_shelf_compression,
        "maxFlushTailWitnessInsideWallMm": (1000 * pipe.flush_overlap).tolist(),
        "lastPipeXyzMm": (1000 * pipe.p).tolist(),
        "traceSha256": digest(raw),
    }
    print(side, dt, out["endWallS"], out["mechanicalStop"], out["loss"], flush=True)
    return out


def run(folder, out, steps, workers):
    departures = json.loads((HERE / "departure.json").read_bytes())["cases"]
    routes = {
        r["case"]: r for r in json.loads((HERE / "route.json").read_bytes())["cases"]
    }
    with ProcessPoolExecutor(max_workers=workers) as pool:
        results = list(
            pool.map(
                run_case,
                [
                    (folder, r, routes[r["case"]], dt)
                    for dt in steps
                    for r in departures
                ],
            )
        )
    out.write_text(json.dumps(results, indent=2) + "\n", encoding="utf8", newline="\n")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("folder", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument("--steps", type=float, nargs="+", default=[0.000125, 0.0000625])
    p.add_argument("--workers", type=int, default=2)
    a = p.parse_args()
    run(a.folder, a.out, a.steps, a.workers)
