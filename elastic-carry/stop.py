"""Brake the two wheels from the first sampled custody alarm and continue the same pipe."""

import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import json
import math
from pathlib import Path
import numpy as np
from drive import restore, fictitious
from checkpoint import HERE, ROOT, coupled, digest
from carry import momentum


def velocity(t, poses, when):
    heading = np.unwrap(poses[:, 2])
    path = np.column_stack((poses[:, :2], heading))
    rates = np.column_stack([np.gradient(col, t) for col in path.T])
    here = np.array([np.interp(when, t, col) for col in path.T])
    speed = np.array([np.interp(when, t, col) for col in rates.T])
    c, s = np.cos(here[2]), np.sin(here[2])
    return (
        here,
        np.array([c * speed[0] + s * speed[1], -s * speed[0] + c * speed[1]]),
        speed[2],
    )


def point(start, body, yaw, elapsed, duration):
    # Decelerate proportional wheel speeds to zero while retaining turn curvature.
    d = min(elapsed, duration)
    distance = d - d * d / (2 * duration)
    angle = yaw * distance
    c, s = np.cos(start[2] + angle / 2), np.sin(start[2] + angle / 2)
    x, y = body * distance * np.sinc(angle / (2 * np.pi))
    return [start[0] + c * x - s * y, start[1] + s * x + c * y, start[2] + angle]


def run_case(job):
    folder, side, dt, late, monitor, *capture = job
    dep = next(
        r
        for r in json.loads((HERE / "departure.json").read_bytes())["cases"]
        if r["case"] == side
    )
    original = next(
        r
        for r in json.loads((HERE / "drive.json").read_bytes())
        if r["case"] == side and r["stepS"] == dt
    )
    src = folder / "carry" / f"{side}-{round(dt * 1e9)}ns.npy"
    if digest(src) != original["traceSha256"]:
        raise ValueError("Changed unbraked trajectory")
    if monitor in ("roof", "latest-pupil"):
        names = ("roof",) if monitor == "roof" else ("roof", "roof-front", "roof-rear")
        choices = [
            r
            for n in names
            for r in json.loads((HERE / (n + ".json")).read_bytes())
            if r["case"] == side
        ]
        r = max(choices, key=lambda r: r["firstUnresolvedServiceWallS"])
        if r["lastVerdictAndIntervals"][0] == "inside":
            raise ValueError("Only a camera-observable alarm may trigger the brake")
        alarm_ms = round(
            1000 * (r["firstUnresolvedServiceWallS"] - original["startWallS"])
        )
    else:
        r = next(
            r
            for r in json.loads((HERE / "ports.json").read_bytes())
            if r["case"] == side and r["stepS"] == dt
        )
        alarm_ms = r["firstMissingFromDepartureMs"]
    handoff = alarm_ms + late
    raw = np.load(src, mmap_mode="r")
    d = raw[handoff]
    if abs(d[0] * 1000 - handoff) > 1e-6:
        raise ValueError("Actuation instant lies outside measured-in-simulation prefix")
    pipe = restore(folder / "checkpoints" / (side + "-state.npz"), dep)
    pipe.time += d[0]
    pipe.p, pipe.a, pipe.v = (x.copy() for x in (d[1:4], d[4:7], d[7:10]))
    pipe.l = momentum(pipe.a, d[10:13])
    np.testing.assert_allclose(pipe.angular(pipe.l), d[10:13], atol=1e-12, rtol=1e-12)
    n = len(pipe.freq)
    pipe.z = d[13 : 13 + n].copy()
    pipe.zdot = d[13 + n : 13 + 2 * n].copy()
    pipe.plunger = d[13 + 2 * n :].copy()
    coupled.feed.cfg = replace(coupled.feed.cfg, slices=10)
    nav = json.loads((ROOT / "conditioned-match/slow-navigation.json").read_bytes())[
        side
    ]
    track = nav["trackMm"] / 1000
    decel = nav["peakWheelMmps2"] / 1000
    course = folder / "routes" / (side + "-route.npz")
    if digest(course) != original["routeSha256"]:
        raise ValueError("Different comparison route")
    path = np.load(course)
    t = path["times"]
    poses = path["poses"]
    epoch = original["startWallS"] + d[0]
    route_time = epoch - original["driveWallS"]
    base, vel, yaw = velocity(t, poses, route_time)
    wheels = [vel[0] - track * yaw / 2, vel[0] + track * yaw / 2]
    duration = max(abs(v) for v in wheels) / decel
    end = duration + 1.0
    elapsed = 0.0
    rest_state = None
    while elapsed < end - 1e-12:
        stopping = elapsed < duration - 1e-12
        h = min(dt, (duration if stopping else end) - elapsed)
        scale = max(0.0, 1 - elapsed / duration)
        if stopping:
            vx, vy = scale * vel
            wz = scale * yaw
            acc = [-vel[0] / duration - wz * vy, -vel[1] / duration + wz * vx, 0.0]
            alpha = -yaw / duration
        else:
            wz = alpha = 0.0
            acc = [0.0, 0.0, 0.0]
        fictitious(pipe, np.asarray(acc), wz, alpha, h)
        pipe.moves(0.0, h)
        elapsed += h
        if abs(elapsed - duration) < 1e-10:
            rest_state = (1000 * pipe.p).tolist()
        if pipe.hard or pipe.loss:
            break
    keys = np.unique(np.r_[np.arange(0, duration, 0.005), duration, end])
    brake = np.asarray([point(base, vel, yaw, t, duration) for t in keys])
    file = folder / "stop" / f"{side}-{monitor}-{late}ms-{round(dt * 1e9)}ns.npz"
    file.parent.mkdir(exist_ok=True, parents=True)
    np.savez(file, wall=epoch + keys, poses=brake)
    result = {
        "case": side,
        "monitor": monitor,
        "stepS": dt,
        "assumedActuationLatencyMs": late,
        "sourceCarrySha256": digest(src),
        "alarmWallS": original["startWallS"] + alarm_ms / 1000,
        "actuationWallS": epoch,
        "initialWheelMps": wheels,
        "initialBodyLateralMps": vel[1],
        "wheelLimitMps2": decel,
        "stopDurationS": duration,
        "wheelStopWallS": epoch + duration,
        "chassisTravelM": np.linalg.norm(vel) * duration / 2,
        "wheelStopPipeXyzMm": rest_state,
        "endPipeXyzMm": (1000 * pipe.p).tolist(),
        "loss": pipe.loss,
        "mechanicalStop": pipe.hard,
        "endWallS": epoch + elapsed,
        "requestedHoldS": 1.0,
        "endPorts": list(map(bool, pipe.ports())),
        "peakCartridgeMm": float(pipe.peak),
        "pathSha256": digest(file),
    }
    if capture:
        result.update(lastAxis=pipe.a.tolist(), lastVelocityMps=pipe.v.tolist())
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--latest-pupil", action="store_true")
    parser.add_argument("--final-optics", action="store_true")
    args = parser.parse_args()
    jobs = [
        (args.folder, side, dt, latency, monitor)
        for side in ("leftSmall", "rightSmall")
        for dt in (0.000125, 0.0000625)
        for latency, monitor in (
            ((60, "latest-pupil"),)
            if args.latest_pupil
            else ((20, "roof"), (60, "roof"), (20, "ports"))
        )
    ]
    if args.final_optics:
        jobs = [
            (args.folder, side, 0.0000625, latency, monitor, True)
            for side in ("leftSmall", "rightSmall")
            for latency, monitor in ((20, "roof"), (60, "latest-pupil"))
        ]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = list(pool.map(run_case, jobs))
    args.out.write_text(
        json.dumps(rows, indent=2) + "\n", encoding="utf8", newline="\n"
    )
