"""Propagate the original pipe, shelf modes, two carriages and two plungers over the attained wheel path."""

import argparse
from pathlib import Path
import numpy as np
from stopped import load_stop, read, write, sha, save_state
from fingers import CLOSED, SPEED, mechanics, observations, permits
from departure_contact import fictitious


def run(artifacts, side):
    base = artifacts / "retained-coral" / side
    run = read(base / "continued/route.json")
    path = base / "continued/path.npz"
    pipe, dt, stop, original = load_stop(artifacts, side)
    if not (
        run["trajectory"] == sha(path)
        and run["sourcePipe"] == sha(original)
        and run["startWallS"] == stop["endWallS"]
        and run["continuation"]["holdS"] == 0.5
    ):
        raise ValueError("Inconsistent opening, stop, contact or service clock")
    with np.load(path) as wheels:
        acceleration = wheels["bodyAcceleration"]
    out = base / "carry"
    out.mkdir(exist_ok=True, parents=True)
    windows = []
    trace = []
    initial = pipe.p.copy()
    peak_travel = 0.0
    peak_pin = 0.0
    peak_probe = 0.0
    first = None
    held = None
    invalid = None
    adverse = None
    for tick, load in enumerate(acceleration):
        for k in range(round(0.005 / dt)):
            time = tick * 0.005 + (k + 1) * dt
            stroke = min(CLOSED, SPEED * time)
            fictitious(pipe, load[:3], load[3], load[4], dt)
            hands = mechanics(pipe, stroke, dt)
            pipe.moves(0.0, dt)
            peak_pin = max(
                peak_pin, *[p["forceN"] for hand in hands for p in hand["pins"]]
            )
            peak_probe = max(peak_probe, *[hand["probeN"] for hand in hands])
            peak_travel = max(
                peak_travel, float(np.linalg.norm(pipe.p - initial)) * 1000
            )
            if (
                min(r["land"] for r in hands) < 0
                or max(r["compression"] for r in hands) > 5
            ):
                adverse = "Contact moved off the complete sensing land or reached the probe stop"
            if pipe.hard or pipe.loss or adverse:
                break
        windows.append(observations(hands, stroke, "nominal"))
        if len(windows) > 5:
            windows.pop(0)
        permit = permits(windows)
        if permit and first is None:
            first = time
        if abs(time - 0.5) < 1e-8:
            held = permit
        if time >= 0.5 - 1e-8 and not permit:
            invalid = time
        trace.append(
            [
                time,
                stroke,
                *pipe.p,
                *pipe.a,
                *pipe.v,
                *pipe.l,
                *[h["compression"] for h in hands],
                int(permit),
            ]
        )
        if tick % 200 == 0:
            write(
                out / "progress.json",
                dict(
                    elapsedS=time,
                    peakTravelMm=peak_travel,
                    peakPinN=peak_pin,
                    firstPermitS=first,
                    chassisThroughS=len(acceleration) * 0.005,
                ),
            )
        if pipe.hard or pipe.loss or invalid is not None or adverse:
            break
    pipe.guides.finish(pipe.time)
    np.save(out / "pipe.npy", np.array(trace))
    save_state(pipe, out / "state.npz")
    end = run["startWallS"] + time
    result = dict(
        case=side,
        sourcePipe=sha(original),
        wheelTrajectory=sha(path),
        piece="coral-A",
        startWallS=run["startWallS"],
        endWallS=end,
        firstPermitAfterStopS=first,
        releasedWheelsAfterHalfSecond=held,
        firstInvalidAfterDepartureS=invalid,
        reachedWheelEndpoint=bool(
            abs(end - run["endWallS"]) < 1e-8
            and permit
            and not (adverse or pipe.hard or pipe.loss)
        ),
        peakTravelMm=peak_travel,
        peakPinN=peak_pin,
        peakProbeN=peak_probe,
        finalCenterMm=(pipe.p * 1000).tolist(),
        finalAxis=pipe.a.tolist(),
        finalVelocityMps=pipe.v.tolist(),
        finalAngularMomentum=pipe.l.tolist(),
        adverse=adverse,
        hardContact=pipe.hard,
        lost=pipe.loss,
        cargoTrace=sha(out / "pipe.npy"),
        terminalPipe=sha(out / "state.npz"),
        matchDeliveries=0,
    )
    write(out / "receipt.json", result)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifacts", type=Path)
    parser.add_argument("side", choices=("plus", "minus"))
    args = parser.parse_args()
    print(run(args.artifacts, args.side))
