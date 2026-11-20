"""Acquire and transport the machined-cam station's own coral on one 5 ms wheel clock."""

from pathlib import Path
from argparse import ArgumentParser
from dataclasses import replace
import json
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
SW = HERE.parent
sys.path.insert(0, str(SW / "score-stroke"))
from cam_model import activate
from loaded_rack import read, digest

CAM = activate()
sys.path.insert(0, str(SW / "retention"))
import held
from retained_report import permission
from fingers import SPEED
from first_service import restore as restore_station, engine

sys.path.insert(0, str(SW / "reef-correction"))
import importlib.util

spec = importlib.util.spec_from_file_location(
    "observed_reef_cargo", SW / "reef-correction/cargo.py"
)
prior_cargo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prior_cargo)
restore = prior_cargo.restore


def run(artifacts, side, stage):
    src = artifacts / "r4-score-stroke"
    name = f"cam-station-{side}-31.25us-0.1mm"
    feed = read(src / (name + ".json"))
    seat = read(src / (name + "-service.json"))
    own = src / name
    initial = own / "first-service" / f"{side}-31.25us-service.npz"
    if (
        digest(initial) != seat["outgoing"]
        or feed["pipeStateSha256"] != seat["incoming"]
        or seat["candidateStepSha256"] != CAM["candidateStepSha256"]
    ):
        raise ValueError("The station source and machined shelf do not match")
    pipe, dt, _ = restore_station(own, feed)
    engine.feed.cfg = replace(engine.feed.cfg, slices=10)
    out = artifacts / "cam-scoring" / side
    if stage == "station":
        reg = read(out / "station-wheels.json")
        wheels = out / (side + "-0.npz")
        first = seat["wallS"]
        if reg["serviceState"] != seat["outgoing"]:
            raise ValueError("Wrong station wheel payload")
    else:
        prior = "station"
        previous = read(out / (prior + "-piece.json"))
        initial = out / (prior + "-piece.npz")
        if previous["state"] != digest(initial) or not previous["allGranted"]:
            raise ValueError("No acquired cam state")
        reg = read(out / stage / "route.json")
        wheels = out / stage / "path.npz"
        first = reg["startWallS"]
        if (
            reg["sourcePipe"] != digest(initial)
            or abs(first - previous["endWallS"]) > 1e-7
        ):
            raise ValueError("Out-of-sequence chassis or pipe")
    # The historical contiguous cam modal basis was built with the host BLAS
    # setting. Limit only the faster repeated tiny solves after exact restart.
    try:
        from threadpoolctl import threadpool_limits

        threadpool_limits(limits=1, user_api="blas")
    except ImportError:
        pass
    if digest(wheels) != reg["trajectory"]:
        raise ValueError("Changed wheel input")
    restore(pipe, initial)
    if abs(pipe.time + feed["lipWallS"] - first) > 1e-8:
        raise ValueError("Station service and motion clocks differ")
    with np.load(wheels, allow_pickle=False) as data:
        acc = data["bodyAcceleration"].copy()
    first_permit = None
    last = False
    fault = None
    min_pin = 1000.0
    peak_force = 0.0
    trace = []
    for tick, sample in enumerate(acc):
        for sub in range(round(0.005 / dt)):
            t = tick * 0.005 + (sub + 1) * dt
            stroke = min(35.0, SPEED * t) if stage == "station" else 35.0
            held.fictitious(pipe, sample[:3], sample[3], sample[4], dt)
            rows = held.mechanics(pipe, stroke, dt)
            pipe.moves(0.0, dt)
            overlap = min(p["overlap"] for r in rows for p in r["pins"])
            peak_force = max(peak_force, max(r["probeN"] for r in rows))
            if (
                any(r["land"] < 0 or r["compression"] >= 8 for r in rows)
                or any(p["forceN"] > 6 for r in rows for p in r["pins"])
                or pipe.hard
                or pipe.loss
            ):
                fault = "rim, bore or physical custody lost"
                break
            if stroke >= 34.8:
                min_pin = min(min_pin, overlap)
        readings = held.observations(rows, stroke, "nominal")
        ports = [bool(v) for v in pipe.ports()]
        grant = permission([v for _, v in readings], ports, [v for v, _ in readings])
        if grant and first_permit is None:
            first_permit = t
        if (stage != "station" or t >= 0.30) and not grant:
            fault = "unverified end/latch/port measurement"
        trace.append(
            [
                t,
                stroke,
                *pipe.p,
                *pipe.a,
                *pipe.v,
                *pipe.l,
                *[v for _, v in readings],
                *ports,
                grant,
            ]
        )
        last = grant
        if tick % 400 == 0:
            held.write(
                out / (stage + "-progress.json"),
                dict(
                    elapsedS=t, observed=grant, peakAxialN=peak_force, engagedMm=min_pin
                ),
            )
        if fault:
            break
    pipe.guides.finish(pipe.time)
    path = out / (stage + "-piece.npz")
    history = out / (stage + "-piece.npy")
    np.save(history, trace)
    held.save_state(pipe, path)
    receipt = dict(
        stage=stage,
        case=side,
        piece=seat["piece"],
        station=seat["outgoing"],
        cam=CAM["candidateStepSha256"],
        before=digest(initial),
        state=digest(path),
        trace=digest(history),
        wheel=digest(wheels),
        stepS=dt,
        startWallS=first,
        endWallS=first + t,
        services=len(trace),
        firstPermitS=first_permit,
        allGranted=bool(last and not fault and min_pin > 12),
        fault=fault,
        minimumBoreMm=min_pin,
        peakAxialN=peak_force,
        center=pipe.p.tolist(),
        axis=pipe.a.tolist(),
        angularMomentum=pipe.l.tolist(),
        matchCredit=0,
    )
    held.write(out / (stage + "-piece.json"), receipt)
    return receipt


if __name__ == "__main__":
    p = ArgumentParser(description=__doc__)
    p.add_argument("artifacts", type=Path)
    p.add_argument("side", choices=["plus", "minus"])
    p.add_argument("stage", choices=["station", "route"])
    a = p.parse_args()
    print(run(a.artifacts, a.side, a.stage))
