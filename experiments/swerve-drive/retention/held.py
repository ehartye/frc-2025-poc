"""Compare two custody signals on identical four-module paths after the first narrow-range report.

The revised feeler needs 8 mm of free travel, with the same 0.45 N/mm law and tip through
the original 5 mm range. Before this source checkpoint both plants therefore have
identical dynamics. The independent aperture rays follow the deformed receiver face.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from stopped import HERE, load_stop, read, save_state, sha, write
from departure_contact import fictitious
from fingers import CLOSED, geometry, mechanics, observations

MAX_INDICATED_MM = 7.5
MAX_AXIAL_N = 3.50
MAX_RADIAL_N = 6.0


def decisions(readings, ports, current, fault=None):
    """One service after identified acquisition. No pose or simulator ID is read here."""
    switches = [ended for ended, mm in readings]
    travel = [mm for ended, mm in readings]
    fresh = True
    if fault == "missing-pipe":
        ports, travel = [False, False], [0, 0]
    elif fault == "beam-open":
        ports = [ports[0], False]
    elif fault == "switch-open":
        switches[0] = False
    elif fault == "overload":
        current = MAX_AXIAL_N + 1
    elif fault == "calibration-offset":
        travel[1] += 5
    elif fault == "stale":
        fresh = False
    physical_limit = current <= MAX_AXIAL_N and max(travel) <= MAX_INDICATED_MM
    healthy = all(switches) and fresh and physical_limit
    return {
        "long-probe": healthy and min(travel) >= 0.65,
        "ports-and-switches": healthy and all(ports),
    }


def restore_interlock(artifacts, side):
    folder = artifacts / "retained-coral" / side
    regs = read(HERE / "carry.json")[side]
    record = regs["pipe"]
    pipe, dt, initial, _ = load_stop(artifacts, side)
    data = folder / "carry/state.npz"
    if (
        sha(data) != record["terminalPipe"]
        or sha(folder / "carry/pipe.npy") != record["cargoTrace"]
    ):
        raise ValueError("Retained piece checkpoint or service trace differs")
    path = folder / "continued/path.npz"
    if record["wheelTrajectory"] != sha(path) or record["piece"] != "coral-A":
        raise ValueError("Expected exactly the reviewed wheel path and same pipe")
    with np.load(data, allow_pickle=False) as state:
        for key in ("p", "a", "v", "l", "z", "zdot", "plunger"):
            setattr(pipe, key, state[key].copy())
        pipe.guides.q = state["guide_q"].copy()
        pipe.guides.v = state["guide_v"].copy()
        pipe.time = float(state["time"])
        pipe.guides.last = pipe.time
        pipe.belt = float(state["belt"])
        pipe.touch = json.loads(str(state["touch"]))
        pipe.first_belts = [
            None if np.isnan(v) else float(v) for v in state["first_belts"]
        ]
    if (
        pipe.belt
        or abs(
            record["startWallS"]
            + record["firstInvalidAfterDepartureS"]
            - record["endWallS"]
        )
        > 1e-8
    ):
        raise ValueError("Interlock and drive times disagree")
    with np.load(path, allow_pickle=False) as wheels:
        acc = wheels["bodyAcceleration"]
    tick = round(record["firstInvalidAfterDepartureS"] / 0.005)
    history = np.load(folder / "carry/pipe.npy", mmap_mode="r")
    if tick != len(history) or not np.allclose(
        history[-1, 2:5], pipe.p, rtol=0, atol=1e-10
    ):
        raise ValueError("Source service index does not match the physical pipe")
    return pipe, dt, acc, tick, regs


def service(pipe, rows):
    readings = observations(rows, CLOSED, "nominal")
    ports = [bool(v) for v in pipe.ports()]
    # Independent analog displacement load bound, including 0.15 mm calibration
    # and 0.05 mm quantization in the direction that increases the possible load.
    # The mechanical 0.7 Ns/m damper is also charged a 0.12 N budget, which is
    # checked against the actual forces on every mechanics substep below.
    load = 0.45 * (max(mm for _, mm in readings) + 0.20) + 0.12
    return readings, ports, load


def run(artifacts, side):
    pipe, dt, acc, start, source = restore_interlock(artifacts, side)
    out = artifacts / "captive-observer" / side
    out.mkdir(exist_ok=True, parents=True)
    rows = geometry(pipe.p * 1000, pipe.a, CLOSED)
    readings, ports, load = service(pipe, rows)
    initial = dict(
        wallS=source["pipe"]["endWallS"],
        ports=ports,
        readings=readings,
        permissions=decisions(readings, ports, load),
    )
    faults = {
        name: decisions(readings, ports, load, name)
        for name in (
            "missing-pipe",
            "beam-open",
            "switch-open",
            "overload",
            "calibration-offset",
            "stale",
        )
    }
    first = {key: None for key in initial["permissions"]}
    trace = []
    peak_axial = peak_radial = 0.0
    worst_bore = 100.0
    worst_land = 100.0
    safe = True
    reason = None
    for tick in range(start, len(acc)):
        sample = acc[tick]
        for j in range(round(0.005 / dt)):
            elapsed = tick * 0.005 + (j + 1) * dt
            fictitious(pipe, sample[:3], sample[3], sample[4], dt)
            rows = mechanics(pipe, CLOSED, dt)
            pipe.moves(0.0, dt)
            axial = max(r["probeN"] for r in rows)
            radial = max(pin["forceN"] for r in rows for pin in r["pins"])
            peak_axial = max(peak_axial, axial)
            peak_radial = max(peak_radial, radial)
            worst_bore = min(
                worst_bore, min(pin["overlap"] for r in rows for pin in r["pins"])
            )
            worst_land = min(worst_land, min(r["land"] for r in rows))
            if (
                axial > MAX_AXIAL_N
                or radial > MAX_RADIAL_N
                or max(r["compression"] for r in rows) >= 8.0
                or min(r["land"] for r in rows) < 0
                or any(pin["overlap"] < 12 for r in rows for pin in r["pins"])
                or pipe.hard
                or pipe.loss
            ):
                safe = False
                reason = "finite mechanical envelope exceeded"
                break
        readings, ports, load = service(pipe, rows)
        verdict = decisions(readings, ports, load)
        if axial > load + 1e-8:
            safe = False
            reason = "measured-force enclosure exceeded"
        for name, allowed in verdict.items():
            if first[name] is None and not allowed:
                first[name] = elapsed
        trace.append(
            [
                elapsed,
                *pipe.p,
                *pipe.a,
                *pipe.v,
                *pipe.l,
                *[v for _, v in readings],
                *ports,
                load,
                axial,
                radial,
                *verdict.values(),
            ]
        )
        if tick % 400 == 0:
            write(
                out / "progress.json",
                dict(
                    elapsed=elapsed,
                    first=first,
                    ports=ports,
                    peakAxialN=peak_axial,
                    peakRadialN=peak_radial,
                ),
            )
        if not safe or all(t is not None for t in first.values()):
            break
    pipe.guides.finish(pipe.time)
    np.save(out / "trace.npy", np.array(trace))
    save_state(pipe, out / "state.npz")
    report = dict(
        case=side,
        source=source["pipe"]["terminalPipe"],
        sourceRoute=source["pipe"]["wheelTrajectory"],
        piece=source["pipe"]["piece"],
        dtS=dt,
        start=initial,
        faults=faults,
        endWallS=source["pipe"]["startWallS"] + elapsed,
        safe=safe,
        limit=reason,
        stepAfterOriginalInterlockS=elapsed - start * 0.005,
        firstPolicyHoldElapsedS=first,
        qualifiedAtWheelEndpoint={
            k: bool(safe and t is None and tick == len(acc) - 1)
            for k, t in first.items()
        },
        lastReadings=readings,
        lastPorts=ports,
        peakAxialN=peak_axial,
        peakRadialN=peak_radial,
        minimumBoreEngagementMm=worst_bore,
        minimumFullRimLandMm=worst_land,
        finalPoseM=pipe.p.tolist(),
        finalAxis=pipe.a.tolist(),
        finalMomentum=pipe.l.tolist(),
        trace=sha(out / "trace.npy"),
        checkpoint=sha(out / "state.npz"),
        matchDeliveries=0,
    )
    write(out / "receipt.json", report)
    return report


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("artifacts", type=Path)
    p.add_argument("side", choices=("plus", "minus"))
    a = p.parse_args()
    print(run(a.artifacts, a.side))
