"""Service actual stereo observations in order and return the first irrevocable brake tick."""

import argparse
import math
from pathlib import Path

import numpy as np

from register import prior, read, sha, write
from transit_images import backgrounds, observation
import receiver_witness as camera


def run(
    raw,
    route_folder,
    pipe_folder,
    field,
    out,
    case,
    step,
    latency=0.015,
    resume_clock=None,
    resume_contact=None,
    missing_eye_after=None,
    internal=False,
):
    _, before, _, original = prior(raw, case, step)
    route = read(route_folder / "route.json")
    contact = read(pipe_folder / "contact.json")
    pipe_path = pipe_folder / contact["file"]
    wheel_path = route_folder / "path.npz"
    if not (
        sha(pipe_path) == contact["sha256"]
        and sha(wheel_path) == contact["sourceRoute"] == route["trajectory"]
        and route["sourcePipe"] == sha(original)
        and contact["piece"] == before["pipe"]["piece"]
    ):
        raise ValueError("Wheel, tube or identity registration differs")
    if contact["hardStop"] or contact["lost"]:
        raise ValueError("Source pipe reports a mechanical loss or stop")
    pipe = np.load(pipe_path)
    with np.load(wheel_path) as st:
        times, poses = st["relativeS"], st["stationMmMmRad"]
    cache=out.parent.parent/"native-scene"
    if internal:
        from inside_camera import setup
        from fixed_contour import read_pair
        fixed,roof,masks=setup(raw,cache)
        def observe(q,center,axis,fault):
            return camera.observe(fixed,roof,center,axis,fault,masks,fixed_reader=read_pair)
    else:
        view=backgrounds(field,cache)
        masks=camera.load_wheel_mask(raw/"wheel-visibility.json")
        def observe(q,center,axis,fault):
            return observation(view,masks,q,center,axis,fault)
    start = contact["startWallS"]
    if resume_clock is None:
        if (
            contact["serviceState"] != route["sourcePipe"]
            or start != route["startWallS"]
        ):
            raise ValueError("First exposure does not start from the issued drive")
        previous = before["vision"]["frames"][-4:]
    else:
        previous_result = read(resume_clock)
        if resume_contact is None:
            raise ValueError("Previous camera segment requires its mechanical source")
        older = read(resume_contact)
        if (
            previous_result["decision"] != "end of contact evidence"
            or abs(previous_result["decisionWallS"] - start) > 1e-8
            or previous_result["sourceRoute"] != older["sourceRoute"]
            or previous_result["sourcePipe"] != older["sha256"]
            or older["sourceRoute"] != contact["previousRoute"]
            or sha(resume_contact) != contact["previousContact"]
            or older["terminalStateSha256"] != contact["serviceState"]
        ):
            raise ValueError(
                "Optical or physical history does not join the continuation"
            )
        previous = previous_result["frames"][-4:]
    last = previous[-1]
    next_stamp = last["captureWallS"] + 0.017
    if not last["accepted"] or not 0.0 < next_stamp - start <= 0.017001:
        raise ValueError("Prior optical clock does not cover this transition")
    rows = []
    tick = round(start * 200)
    final_tick = math.floor((start + min(pipe[-1, 0], times[-1])) * 200 + 1e-8)
    while tick <= final_tick:
        now = tick / 200
        while next_stamp <= now + 1e-10:
            rel = next_stamp - start
            x = np.array([np.interp(rel, pipe[:, 0], v) for v in pipe[:, 1:7].T])
            drive_rel = next_stamp - route["startWallS"]
            q = np.array([np.interp(drive_rel, times, v) for v in poses.T])
            source = observe(
                q,
                1000 * x[:3],
                x[3:] / np.linalg.norm(x[3:]),
                "fixed-left-drop"
                if missing_eye_after is not None and drive_rel >= missing_eye_after
                else None,
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
            sourceContactReceipt=sha(pipe_folder / "contact.json"),
            terminalPipeState=contact["terminalStateSha256"],
            startWallS=start,
            decisionWallS=min(tick, final_tick) / 200,
            decision="brake" if held else "end of contact evidence",
            frames=rows,
            observer="inside-backed" if internal else "station-background",
            frameLagS=latency,
            missingEyeAfterS=missing_eye_after,
            wheelMask=sha(raw / "wheel-visibility.json"),
        ),
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("raw", "route", "pipe", "field", "out"):
        p.add_argument(name, type=Path)
    p.add_argument("case", choices=("plus", "minus"))
    p.add_argument("step", type=float)
    p.add_argument("--resume-clock", type=Path)
    p.add_argument("--resume-contact", type=Path)
    p.add_argument("--missing-eye-after", type=float)
    p.add_argument("--internal",action="store_true")
    a = p.parse_args()
    run(
        a.raw,
        a.route,
        a.pipe,
        a.field,
        a.out,
        a.case,
        a.step,
        resume_clock=a.resume_clock,
        resume_contact=a.resume_contact,
        missing_eye_after=a.missing_eye_after,
        internal=a.internal,
    )
