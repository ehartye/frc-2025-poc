"""Restart the full compliant carriage and pipe on an attained transit trajectory."""

import argparse
from pathlib import Path

import numpy as np

from register import MOTION, prior, read, sha, write
from departure_contact import run as replay_contact


def run(
    raw,
    contact_raw,
    folder,
    out,
    case,
    step,
    until=None,
    resume=None,
    resume_route=None,
):
    row, previous, _, pipe_path = prior(raw, case, step)
    route = read(folder / "route.json")
    if (
        route["case"] != case
        or route["stepUs"] != step
        or route["sourcePipe"] != sha(pipe_path)
    ):
        raise ValueError("Route did not start with this attained tube")
    chassis = folder / "path.npz"
    if sha(chassis) != route["trajectory"]:
        raise ValueError("Unregistered chassis samples")
    feed = next(
        v
        for v in read(MOTION / "loaded-feed.json")["attempts"]
        if (v["case"], v["plantStepUs"]) == (case, step)
    )
    out.mkdir(exist_ok=True, parents=True)
    with np.load(chassis) as sampled:
        t = sampled["relativeS"]
        start_index = 0
        source_slice = None
        if resume is not None:
            before = read(resume / "contact.json")
            start_index = round(before["requestedThroughS"] / 0.005)
            expiry = before["startWallS"] + before["endRelativeS"]
            if abs(expiry - route["startWallS"] - start_index * 0.005) > 1e-8:
                raise ValueError("Continuation requires the same source epoch")
            if before["sourceRoute"] != route["trajectory"]:
                if (
                    resume_route is None
                    or sha(resume_route / "path.npz") != before["sourceRoute"]
                ):
                    raise ValueError(
                        "An alternate brake path needs the original verified wheel route"
                    )
                with np.load(resume_route / "path.npz") as previous_wheels:
                    for key in (
                        "relativeS",
                        "worldPose",
                        "worldVelocity",
                        "estimatedPose",
                        "stationMmMmRad",
                        "bodyAcceleration",
                    ):
                        count = (
                            start_index
                            if key == "bodyAcceleration"
                            else start_index + 1
                        )
                        if not np.array_equal(
                            previous_wheels[key][:count], sampled[key][:count]
                        ):
                            raise ValueError(
                                f"Brake path diverged before handoff in {key}"
                            )
            pipe_path = (resume / before["file"]).with_suffix(".npz")
            if (
                before["hardStop"]
                or before["lost"]
                or sha(pipe_path) != before["terminalStateSha256"]
            ):
                raise ValueError("Previous contact checkpoint is missing or rejected")
            source_slice = sha(resume / "contact.json")
        stop = len(t) - 1 if until is None else min(len(t) - 1, round(until / 0.005))
        if stop <= start_index:
            raise ValueError("Empty contact interval")
        if stop + 1 < len(t) or start_index:
            chassis = out / "prefix.npz"
            np.savez(
                chassis,
                relativeS=np.arange(stop - start_index + 1) * 0.005,
                bodyAcceleration=sampled["bodyAcceleration"][start_index:stop],
            )
        elapsed = (stop - start_index) * 0.005
    arrival = dict(
        case=case,
        incoming=feed["pipeStateSha256"],
        piece=row["pipe"],
        outgoing=sha(pipe_path),
        wallS=route["startWallS"] + start_index * 0.005,
    )
    meta = dict(
        trajectory=sha(chassis),
        driveClock=feed["driveTraceSha256"],
        serviceState=arrival["outgoing"],
        leaseS=elapsed,
    )
    result = replay_contact(contact_raw, feed, arrival, pipe_path, (chassis, meta), out)
    result.update(
        sourceRoute=route["trajectory"],
        inputPose=route["startPose"],
        requestedThroughS=stop * 0.005,
        previousContact=source_slice,
        previousRoute=before["sourceRoute"] if resume is not None else None,
    )
    write(out / "contact.json", result)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("raw", "contact_raw", "route", "out"):
        p.add_argument(name, type=Path)
    p.add_argument("case", choices=("plus", "minus"))
    p.add_argument("step", type=float)
    p.add_argument("--until", type=float)
    p.add_argument("--resume", type=Path)
    p.add_argument("--resume-route", type=Path)
    a = p.parse_args()
    run(
        a.raw,
        a.contact_raw,
        a.route,
        a.out,
        a.case,
        a.step,
        a.until,
        a.resume,
        a.resume_route,
    )
