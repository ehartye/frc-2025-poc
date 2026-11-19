"""Verify the stowed vertical load path using the selected assembly's actual solids."""

import argparse
import json
from pathlib import Path
import sys

from tray import ROOT, SHELF, common, volume, box, cad, digest, distance

sys.path.insert(0, str(ROOT / "sp1-mount"))
import check as sp1


def pair(a, b, shifts):
    return {
        "initialSeparationMm": distance(a, b),
        "downwardCommonMm3": {
            str(d): volume(common(cad.translated(a, (0, 0, -d)), b)) for d in shifts
        },
    }


def run(artifacts, lamp):
    pose = sp1.scene(artifacts, lamp)[0]
    out = {}
    for sign, side in [(-1, "R"), (1, "L")]:
        mast = f"backup-{side}-drilled-mast"
        y = sign * 180
        names = [
            SHELF,
            f"shelf-spacer-{side}",
            f"drilled-slide-{side}-inner",
            f"slide-{side}-middle",
            f"slide-{side}-outer",
            "slide-base-bridge",
            "lift-crossbar-drilled",
            f"height-{side}-pin",
            f"height-{side}-mast-sleeve",
            mast,
            "deck-main",
            f"drive-rail-inner-{side}",
        ]
        path = {}
        for i, (n, m) in enumerate(zip(names, names[1:])):
            a, b = pose[n], pose[m]
            shifts = (
                [0.99, 1.01]
                if i in (2, 3)
                else [0.05, 0.3]
                if i in (6, 7, 8)
                else [0.05]
            )
            path[n + " to " + m] = pair(a, b, shifts)
        # Fore-aft extent of actual bearing at 1.05 mm take-up; open channel can
        # translate up without its paired floor acting as a positive vertical stop.
        upper, mid = [pose[k] for k in names[2:4]]
        common1 = common(cad.translated(upper, (0, 0, -1.05)), mid)
        path["innerFloorWitnessBoundsMm"] = cad.bounds(common1)
        path["innerFloorToMiddleUp1mmCommonMm3"] = volume(
            common(cad.translated(upper, (0, 0, 1.0)), mid)
        )
        path["innerFloorFrontLocalSeparationMm"] = distance(
            common(upper, box(115, y - 4, 440, 118, y + 4, 480)),
            common(mid, box(115, y - 4, 440, 118, y + 4, 480)),
        )
        # Boundary witnesses are geometric areas; the design pressure requires
        # measured loaded contact distribution and bolt preload.
        out[side] = {
            "parts": names,
            "boundsMm": {n: cad.bounds(pose[n]) for n in names},
            "interfaces": path,
        }
    return {
        "sourcePins": {
            k: digest(ROOT / k)
            for k in (
                "selected-match/receiver-clearance-kit.step",
                "sp1-mount/sp1-mount.step",
                "design/fixed-height/low-locks.step",
                "sp1-mount/check.py",
            )
        },
        "loadingPose": {"liftMm": 560, "slideMm": 0},
        "sides": out,
        "physicalLoadRatingN": None,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for key in ("artifacts", "lamp", "out"):
        p.add_argument(key, type=Path)
    a = p.parse_args()
    result = run(a.artifacts, a.lamp)
    a.out.write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
