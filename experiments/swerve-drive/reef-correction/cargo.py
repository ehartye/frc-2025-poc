"""Apply the measured reef correction to the same retained six-degree-of-freedom pipe and flexible receiver."""

from pathlib import Path
import argparse
import json
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "retention"))
import held
from retained_report import permission


def restore(pipe, state):
    with np.load(state, allow_pickle=False) as data:
        for key in ("p", "a", "v", "l", "z", "zdot", "plunger"):
            setattr(pipe, key, data[key].copy())
        pipe.guides.q = data["guide_q"].copy()
        pipe.guides.v = data["guide_v"].copy()
        pipe.time = float(data["time"])
        pipe.guides.last = pipe.time
        pipe.belt = float(data["belt"])
        pipe.touch = json.loads(str(data["touch"]))
        pipe.first_belts = [
            None if np.isnan(v) else float(v) for v in data["first_belts"]
        ]


def run(artifacts, side, *, cam_run=None, cam_source=None, gap_mm=.1):
    if cam_run is None:
        raw = artifacts / "reef-correction" / side
        state = artifacts / "captive-observer" / side / "state.npz"
        source = held.read(HERE.parent / "retention/held.json")["cases"][side]["source"]
    else:
        raw = cam_run / side
        arrived = (artifacts / "cam-scoring" if cam_source is None else cam_source) / side
        state = arrived / "route-piece.npz"
        receipt = held.read(arrived / "route-piece.json")
        issued = held.read(HERE.parent / "cam-scoring/results.json")["developmentCases"][side]
        if not receipt["allGranted"] or (cam_source is None and issued["stages"]["route"]["state"] != receipt["state"]):
            raise ValueError("No matched cam cargo at the reef stop")
        source = {"checkpoint": receipt["state"], "piece": receipt["piece"],
                  "endWallS": receipt["endWallS"]}
    chassis = held.read(raw / "chassis.json")
    if not (
        source["checkpoint"] == chassis["sourcePipe"] == held.sha(state)
        and source["endWallS"] == chassis["startWallS"]
        and source["piece"] == chassis["piece"] == "coral-A"
        and chassis["route"] == held.sha(raw / "chassis.npz")
    ):
        raise ValueError("Different loaded source or actuator clock")
    if cam_run is None:
        pipe, dt, _, _, _ = held.restore_interlock(artifacts, side)
    else:
        sys.path.insert(0, str(HERE.parent / "cam-scoring"))
        import captivity
        src = artifacts / "r4-score-stroke"
        name = f"cam-station-{side}-31.25us-{gap_mm:g}mm"
        feed = held.read(src / (name + ".json"))
        if feed["candidateStepSha256"] != receipt["cam"] or receipt["cam"] != captivity.CAM["candidateStepSha256"]:
            raise ValueError("Different plate section at restart")
        pipe, dt, _ = captivity.restore_station(src / name, feed)
        from dataclasses import replace
        captivity.engine.feed.cfg = replace(captivity.engine.feed.cfg, slices=10)
        try:
            from threadpoolctl import threadpool_limits
            threadpool_limits(limits=1, user_api="blas")
        except ImportError:
            pass
    restore(pipe, state)
    if cam_run is not None and abs(pipe.time + feed["lipWallS"] - source["endWallS"]) > 1e-7:
        raise ValueError("Discontinuous cam clock")
    original_time = pipe.time
    with np.load(raw / "chassis.npz") as route:
        acc = route["bodyAcceleration"].copy()
    trace = []
    peak = 0.0
    min_overlap = 1000.0
    failure = None
    for t, sample in enumerate(acc):
        for substep in range(round(0.005 / dt)):
            held.fictitious(pipe, sample[:3], sample[3], sample[4], dt)
            rows = held.mechanics(pipe, held.CLOSED, dt)
            pipe.moves(0, dt)
            peak = max(peak, max(r["probeN"] for r in rows))
            min_overlap = min(
                min_overlap, min(p["overlap"] for r in rows for p in r["pins"])
            )
            if (
                pipe.hard
                or pipe.loss
                or min_overlap < 12
                or any(r["compression"] >= 8 or r["land"] < 0 for r in rows)
                or any(
                    r["probeN"] > 3.5 or any(p["forceN"] > 6 for p in r["pins"])
                    for r in rows
                )
            ):
                failure = "Mechanical enclosure exceeded"
                break
        observations, ports, load = held.service(pipe, rows)
        grant = permission(
            [v for _, v in observations], ports, [v for v, _ in observations]
        )
        trace.append(
            [
                pipe.time - original_time,
                *pipe.p,
                *pipe.a,
                *pipe.v,
                *pipe.l,
                *ports,
                load,
                grant,
            ]
        )
        if not grant:
            failure = "Observed custody veto"
        if failure:
            break
    pipe.guides.finish(pipe.time)
    np.save(raw / "coral.npy", trace)
    held.save_state(pipe, raw / "coral.npz")
    end = chassis["startWallS"] + pipe.time - original_time
    if not failure and abs(end - chassis["endWallS"]) > 1e-7:
        raise ValueError("Physical endpoint clocks disagree")
    result = dict(
        source=source["checkpoint"],
        piece=source["piece"],
        startWallS=source["endWallS"],
        endWallS=end,
        trace=held.sha(raw / "coral.npy"),
        state=held.sha(raw / "coral.npz"),
        wheels=chassis["route"],
        stepS=dt,
        services=len(trace),
        allGranted=failure is None,
        failure=failure,
        peakAxialN=peak,
        minEngagementMm=min_overlap,
        centerM=pipe.p.tolist(),
        axis=pipe.a.tolist(),
        velocityMps=pipe.v.tolist(),
        angularMomentum=pipe.l.tolist(),
        deliveryCredit=0,
    )
    held.write(raw / "coral.json", result)
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("artifacts", type=Path)
    p.add_argument("case", choices=["plus", "minus"])
    a = p.parse_args()
    print(run(a.artifacts, a.case))
