"""Collect independently generated drives, contact results and camera decisions."""

import argparse
from pathlib import Path

import numpy as np

from register import MOTION, read, sha, write


def wheel(folder):
    entry = read(folder / "route.json")
    with np.load(folder / "path.npz") as st:
        trail = np.column_stack(
            (entry["startWallS"] + st["relativeS"], st["worldPose"])
        )
    if entry["trajectory"] != sha(folder / "path.npz"):
        raise ValueError("Published wheel path has changed")
    return dict(
        sourceWheel=entry["sourceWheel"],
        sourcePipe=entry["sourcePipe"],
        trajectory=entry["trajectory"],
        startWallS=entry["startWallS"],
        endWallS=entry["endWallS"],
        endPose=entry["endPose"],
        pathLengthM=entry["pathLengthM"],
        peaks=entry["peaks"],
        battery=entry.get("battery"),
        field=entry["field"],
        maxOdometryErrorMm=entry["maxOdometryErrorMm"],
        targets=entry["goals"],
        waypointCrossings=entry["waypoints"],
        timedPath=np.round(
            trail[np.unique([*range(0, len(trail), 40), len(trail) - 1])], 5
        ).tolist(),
    )


def run(raw):
    result = {}
    for case in ("plus", "minus"):
        base = raw / f"{case}-31.25"
        paths = {key: wheel(base / key) for key in ("full", "settled", "south-offset")}
        if (
            len({v["sourceWheel"] for v in paths.values()}) != 1
            or len({v["sourcePipe"] for v in paths.values()}) != 1
        ):
            raise ValueError("Policies were run from different hardware states")
        result[case] = dict(routes=paths)
    return dict(
        schema="reefscape-loaded-swerve-corridor/1",
        priorMatch=sha(MOTION / "loaded-departure.json"),
        scope="Conditional four-module wheel trajectory with coral-A included in chassis mass.",
        cases=result,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("raw", type=Path)
    p.add_argument("out", type=Path)
    a = p.parse_args()
    write(a.out, run(a.raw))
