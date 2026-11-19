"""Replay a single stationary retention assessment from the two source-bound, measured stops."""

import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

from fingers import CLOSED, SPEED, mechanics, observations, permits

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "transit"))
from register import MOTION, read, sha, write
from departure_contact import engine, restore


def load_stop(artifacts, side):
    stop = read(HERE.parent / "transit/stops.json")["cases"][side]
    cargo = stop["cargo"]
    folder = artifacts / "swerve-transit" / f"{side}-31.25" / "internal-stop-contact"
    file = (folder / cargo["file"]).with_suffix(".npz")
    wheel = artifacts / "retained-coral" / side / "stop"
    run = read(wheel / "route.json")
    if not (
        sha(file) == cargo["terminalStateSha256"]
        and cargo["piece"] == "coral-A"
        and sha(wheel / "path.npz") == cargo["sourceRoute"]
        and sha(wheel / "end-wheels.npz") == run["terminalWheels"]
        and run["stopFeedback"] == stop["stopFeedback"]
        and run["stopFeedback"]["confirmed"]
    ):
        raise ValueError(
            "Could not join the same attained wheel, pipe and measured stop"
        )
    feed = next(
        r
        for r in read(MOTION / "loaded-feed.json")["attempts"]
        if r["case"] == side and r["plantStepUs"] == 31.25
    )
    pipe, dt, _ = restore(artifacts / "swerve-energy/contact", feed)
    with np.load(file, allow_pickle=False) as old:
        for k in ("p", "a", "v", "l", "z", "zdot", "plunger"):
            setattr(pipe, k, old[k].copy())
        pipe.time = float(old["time"])
        pipe.belt = float(old["belt"])
        pipe.guides.q = old["guide_q"].copy()
        pipe.guides.v = old["guide_v"].copy()
        pipe.guides.last = pipe.time
        pipe.touch = json.loads(str(old["touch"]))
        pipe.first_belts = [
            None if np.isnan(x) else float(x) for x in old["first_belts"]
        ]
    if abs(pipe.time + feed["lipWallS"] - run["endWallS"]) > 1e-8 or pipe.belt != 0:
        raise ValueError("Pipe clock/held command did not reach wheel service")
    engine.feed.cfg = replace(engine.feed.cfg, slices=10)
    return pipe, dt, run, file


def save_state(pipe, destination):
    np.savez(
        destination,
        **{
            k: getattr(pipe, k)
            for k in (
                "p",
                "a",
                "v",
                "l",
                "z",
                "zdot",
                "plunger",
                "time",
                "belt",
                "freq",
            )
        },
        guide_q=pipe.guides.q,
        guide_v=pipe.guides.v,
        touch=json.dumps(pipe.touch),
        first_belts=[np.nan if v is None else v for v in pipe.first_belts],
    )


def run(artifacts, side, refine=1):
    pipe, dt, wheel, source = load_stop(artifacts, side)
    dt /= refine
    out = artifacts / "retained-coral" / side / f"capture-{refine}"
    out.mkdir(exist_ok=True, parents=True)
    initial = pipe.p.copy()
    initial_momentum = pipe.l.copy()
    cases = ("nominal", "missing-probe", "stuck-open", "empty")
    windows = {f: [] for f in cases}
    first = {f: None for f in cases}
    trace = []
    gaps = [float("inf")] * 3
    peak = np.zeros(3)
    for k in range(round(0.5 / dt)):
        elapsed = (k + 1) * dt
        stroke = min(CLOSED, SPEED * elapsed)
        rows = mechanics(pipe, stroke, dt)
        pipe.moves(0.0, dt)
        gaps = np.minimum(
            gaps,
            [
                min(p["clearance"] for r in rows for p in r["pins"]),
                min(r["land"] for r in rows),
                min(5.0 - r["compression"] for r in rows),
            ],
        )
        peak = np.maximum(
            peak,
            [
                max(r["probeN"] for r in rows),
                max(p["forceN"] for r in rows for p in r["pins"]),
                1000 * np.linalg.norm(pipe.p - initial),
            ],
        )
        if (k + 1) % round(0.005 / dt) == 0:
            for fault in cases:
                w = windows[fault]
                w.append(observations(rows, stroke, fault))
                if len(w) > 5:
                    w.pop(0)
                if permits(w) and first[fault] is None:
                    first[fault] = elapsed
            trace.append(
                [
                    elapsed,
                    stroke,
                    *pipe.p,
                    *pipe.a,
                    *pipe.v,
                    *pipe.l,
                    *[r["compression"] for r in rows],
                    *[r["probeN"] for r in rows],
                ]
            )
        if pipe.hard or pipe.loss or min(gaps) < 0:
            break
    pipe.guides.finish(pipe.time)
    np.save(out / "trace.npy", np.array(trace))
    save_state(pipe, out / "state.npz")
    valid = (
        elapsed >= 0.5 - 1e-9
        and not pipe.hard
        and not pipe.loss
        and all(gaps >= 0)
        and permits(windows["nominal"])
    )
    result = dict(
        case=side,
        sourcePipe=sha(source),
        sourceWheel=wheel["terminalWheels"],
        piece="coral-A",
        startWallS=wheel["endWallS"],
        endWallS=wheel["endWallS"] + elapsed,
        stepUs=dt * 1e6,
        firstSensorPermissionAfterStopS=first,
        endSensorPermission={f: permits(w) for f, w in windows.items()},
        pinEntryClearanceMm=float(gaps[0]),
        completeProbeLandMm=float(gaps[1]),
        remainingProbeTravelMm=float(gaps[2]),
        peakProbeN=float(peak[0]),
        peakPinN=float(peak[1]),
        peakCenterTravelMm=float(peak[2]),
        finalCenterMm=(pipe.p * 1000).tolist(),
        finalAxis=pipe.a.tolist(),
        simulatedStationaryCapture=bool(valid),
        baseHeldAtMeasuredStop=True,
        initialAngularMomentum=initial_momentum.tolist(),
        hardContact=pipe.hard,
        lost=pipe.loss,
        traceSha256=sha(out / "trace.npy"),
        terminalPipe=sha(out / "state.npz"),
    )
    write(out / "receipt.json", result)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifacts", type=Path)
    parser.add_argument("side", choices=("plus", "minus"))
    parser.add_argument("--refine", type=int, choices=(1, 2), default=1)
    args = parser.parse_args()
    print(json.dumps(run(args.artifacts, args.side, args.refine)))
