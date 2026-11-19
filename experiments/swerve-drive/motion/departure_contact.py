"""Continue the acquired rigid tube and translating shelf at the actual wheel-service clock."""

from dataclasses import replace
import json
import math
import sys
import numpy as np

from first_service import ROOT, digest, engine, restore

sys.path.insert(0, str(ROOT / "relieved-match"))
from carry import fictitious


def run(raw, feed, service, checkpoint, chassis, destination):
    if (
        feed["pipeStateSha256"] != service["incoming"]
        or feed["case"] != service["case"]
        or feed["physicalPieceId"] != service["piece"]
        or digest(checkpoint) != service["outgoing"]
    ):
        raise ValueError("Different station state, pipe or wheel service")
    pipe, step, _ = restore(raw, feed)
    with np.load(checkpoint, allow_pickle=False) as st:
        for key in ("p", "a", "v", "l", "z", "zdot", "plunger"):
            setattr(pipe, key, st[key].copy())
        pipe.time = float(st["time"])
        pipe.belt = float(st["belt"])
        pipe.guides.q = st["guide_q"].copy()
        pipe.guides.v = st["guide_v"].copy()
        pipe.guides.last = pipe.time
        pipe.touch = json.loads(str(st["touch"]))
        pipe.first_belts = [
            None if np.isnan(v) else float(v) for v in st["first_belts"]
        ]
    first = service["wallS"]
    if abs(pipe.time + feed["lipWallS"] - first) > 1e-9:
        raise ValueError("Opening and dyno clocks disagree")
    route, chassis_receipt = chassis
    if (
        digest(route) != chassis_receipt["trajectory"]
        or chassis_receipt["driveClock"] != feed["driveTraceSha256"]
        or chassis_receipt["serviceState"] != service["outgoing"]
    ):
        raise ValueError("Different attained wheel trajectory")
    with np.load(route) as data:
        t = data["relativeS"]
        load = data["bodyAcceleration"]
    center0 = pipe.p.copy()
    trace = []

    def record(now):
        trace.append([now, *pipe.p, *pipe.a, *pipe.v, *pipe.l])

    record(0.0)
    engine.feed.cfg = replace(engine.feed.cfg, slices=10)
    for k in range(math.ceil(t[-1] / step)):
        now = k * step
        dt = min(step, t[-1] - now)
        index = min(len(load) - 1, int((now + 1e-12) / 0.005))
        force = load[index]
        fictitious(pipe, force[:3], force[3], force[4], dt)
        pipe.moves(0.0, dt)
        if (k + 1) % round(0.001 / step) == 0 or pipe.hard or pipe.loss:
            record(now + dt)
        if pipe.hard or pipe.loss:
            break
    pipe.guides.finish(pipe.time)
    array = np.asarray(trace)
    destination.mkdir(parents=True, exist_ok=True)
    filename = f"{feed['case']}-{feed['plantStepUs']:g}-{round(chassis_receipt['leaseS'] * 1000)}-pipe.npy"
    path = destination / filename
    np.save(path, array)
    end_state = path.with_suffix(".npz")
    full = {
        k: getattr(pipe, k)
        for k in ("p", "a", "v", "l", "z", "zdot", "plunger", "time", "belt", "freq")
    }
    np.savez(
        end_state,
        **full,
        guide_q=pipe.guides.q,
        guide_v=pipe.guides.v,
        touch=json.dumps(pipe.touch),
        first_belts=[np.nan if v is None else v for v in pipe.first_belts],
        registeredStationMmMmDeg=feed["trueChassisAtFeedMmMmDeg"],
    )

    return dict(
        case=feed["case"],
        stepUs=feed["plantStepUs"],
        piece=service["piece"],
        serviceState=service["outgoing"],
        chassis=chassis_receipt["trajectory"],
        file=filename,
        sha256=digest(path),
        terminalStateSha256=digest(end_state),
        startWallS=first,
        endRelativeS=array[-1, 0],
        hardStop=pipe.hard,
        lost=pipe.loss,
        peakTravelMm=1000
        * float(np.linalg.norm(array[:, 1:4] - center0, axis=1).max()),
        lastCenterMm=(1000 * pipe.p).tolist(),
        lastAxis=pipe.a.tolist(),
        heldBeltMps=pipe.belt,
    )
