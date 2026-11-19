"""Restart the full compliant carriage and pipe on an attained transit trajectory."""

import argparse
from pathlib import Path

import numpy as np

from register import MOTION, prior, read, sha, write
from departure_contact import run as replay_contact


def run(raw, contact_raw, folder, out, case, step, until=None):
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
        stop = len(t) - 1 if until is None else min(len(t) - 1, round(until / 0.005))
        if stop < 1:
            raise ValueError("Empty contact interval")
        if stop + 1 < len(t):
            chassis = out / "prefix.npz"
            np.savez(
                chassis,
                relativeS=t[: stop + 1],
                bodyAcceleration=sampled["bodyAcceleration"][:stop],
            )
        elapsed = float(t[stop])
    arrival = dict(
        case=case,
        incoming=feed["pipeStateSha256"],
        piece=row["pipe"],
        outgoing=sha(pipe_path),
        wallS=route["startWallS"],
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
        requestedThroughS=elapsed,
    )
    write(out / "contact.json", result)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("raw", "contact_raw", "route", "out"):
        p.add_argument(name, type=Path)
    p.add_argument("case", choices=("plus", "minus"))
    p.add_argument("step", type=float)
    p.add_argument("--until", type=float)
    a = p.parse_args()
    run(a.raw, a.contact_raw, a.route, a.out, a.case, a.step, a.until)
