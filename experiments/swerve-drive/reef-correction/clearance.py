"""Place the pitch and stroke envelope at every optically admitted attained sector-A dock pose."""

import argparse
from itertools import product
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SCORE = HERE.parent / "score-stroke"


def read(path):
    return json.loads(path.read_bytes())


def run(artifacts, side):
    source = read(artifacts / "reef-correction" / side / "chassis.json")
    leg = read(SCORE / "candidate.json")["dock"][side]
    package = ROOT / "cad/full-swerve-r4/appearances.json"
    parts = {row["id"]: row["boundsMm"] for row in read(package)["solids"]}
    kit = read(HERE.parent / "retention/open-yokes.json")
    for row in kit["parts"]:
        box = np.array(row["boundsMm"])
        if row["frame"] in ("probe", "yoke"):
            # Include all possible commanded pin positions and spring travel.
            box[1] -= 35 if "left" in row["id"] else 8
            box[4] += 35 if "right" in row["id"] else 8
        parts[row["id"]] = box
    rail_ids = ["drilled-slide-L-inner", "drilled-slide-R-inner"]
    floor_x = 490.0
    import hashlib

    file = artifacts / "r4-score-stroke" / f"{side}-cam-cycle-31.25us.npz"
    if hashlib.sha256(file.read_bytes()).hexdigest() != leg["trajectorySha256"]:
        raise ValueError("Changed reference shelf deformation")
    l, h = np.array(source["enclosure"])
    ang = np.linspace(l[2], h[2], 61)
    with np.load(file) as state:
        stroke = state["strokeM"]
        phi = state["q"][:, 5]
    gaps = {}
    for name in rail_ids + [row["id"] for row in kit["parts"]]:
        b = parts[name]
        gap = 1e9
        for x, y, z in product((b[0], b[3]), (b[1], b[4]), (b[2], b[5])):
            fx = np.cos(phi) * x - np.sin(phi) * (z - 467) + 1000 * stroke
            # A 1 ms trajectory grid; include an extra 1 mm translation and
            # rotation allowance between reported rack samples.
            for yaw in ang:
                dx = fx * math.cos(yaw) - y * math.sin(yaw)
                between = math.hypot(float(np.max(abs(fx))), y) * (ang[1] - ang[0]) / 2
                gap = min(gap, float(floor_x - h[0] - np.max(dx) - between - 1.0))
        gaps[name] = gap
    # A change in tangential delivery relative to the old independent rim fixture
    # cannot inherit its clearance. This comparison screens installed stock only.
    return dict(
        case=side,
        chassis=source["route"],
        enclosure=source["enclosure"],
        minimumMm=min(gaps.values()),
        referenceRack=leg["trajectorySha256"],
        referenceCad=kit["stepSha256"],
        perPartMm=gaps,
        strokeSamples=len(stroke),
        enclosingYawSamples=len(ang),
        rackInterpolationAllowanceMm=1,
        projection="Complete front separating plane of the surveyed reef wall",
        clearanceConditionalOnReferenceStrokeAndPitch=True,
        deliveryCredit=0,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("artifacts", type=Path)
    p.add_argument("side", choices=["plus", "minus"])
    a = p.parse_args()
    result = run(a.artifacts, a.side)
    (a.artifacts / "reef-correction" / a.side / "clearance.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf8"
    )
    print(result["minimumMm"])
