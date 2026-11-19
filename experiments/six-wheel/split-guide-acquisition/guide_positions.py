"""Map all six native nested channel bounds into the four-land moving optical frame."""

import hashlib
import itertools
import json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent


def bounds_at(point, registry, matrix):
    site = np.asarray(point["landsM"])
    stages = np.asarray(point["guideStagesM"]).reshape(2, 2)
    table = {}
    for side in ("L", "R"):
        bounds = registry["sides"][side]["boundsMm"]
        for depth, key in enumerate(
            (
                f"slide-{side}-outer",
                f"slide-{side}-middle",
                f"drilled-slide-{side}-inner",
            )
        ):
            lo = bounds[key][:3]
            hi = bounds[key][3:]
            basic = []
            for xmm, ymm in itertools.product((lo[0], hi[0]), (lo[1], hi[1])):
                x, y = xmm / 1000, ymm / 1000
                # Native outer channel follows bridge. Middle adds first guide displacement,
                # and the innermost shelf-bearing rail adds both; +Z is robot-up.
                shape = np.array([1.0, x, y, x * y]) @ matrix
                g = np.array([0.5 + y / 0.36, 0.5 - y / 0.36])
                down = float(shape @ site + g @ np.sum(stages[:depth], axis=0))
                basic.append(-1000 * down)
            table[key] = {
                "unloadedBoundsMm": bounds[key],
                "zOffsetMm": [min(basic), max(basic)],
                "conservativeModelZRangeMm": [lo[2] + min(basic), hi[2] + max(basic)],
            }
    return table


def run():
    frames = json.loads((HERE / "candidate.json").read_bytes())
    source = json.loads((ROOT / "shelf-structure/supports.json").read_bytes())
    pts = (
        np.asarray(
            json.loads((ROOT / "compliant-carriage/boundary.json").read_bytes())[
                "positionsMm"
            ]
        )
        / 1000
    )
    matrix = np.linalg.inv(
        np.column_stack((np.ones(4), pts[:, 0], pts[:, 1], pts[:, 0] * pts[:, 1]))
    )
    output = []
    for r in frames["cases"]:
        if r["state"] != "IN_BAND":
            continue
        output.append(
            {
                "side": r["side"],
                "stepS": r["stepS"],
                "exposures": {
                    ms: bounds_at(s, source, matrix) for ms, s in r["exposures"].items()
                },
            }
        )
    return {
        "sourceSha256": {
            str(p.relative_to(ROOT)).replace(chr(92), "/"): hashlib.sha256(
                p.read_bytes()
            ).hexdigest()
            for p in (
                ROOT / "shelf-structure/supports.json",
                ROOT / "compliant-carriage/boundary.json",
                HERE / "candidate.json",
            )
        },
        "boundsType": "Bilinear vertical model evaluated at all four XY corners of each source native whole-body AABB; actual lateral, torsional and assembly tolerance are unmeasured.",
        "cases": output,
    }


if __name__ == "__main__":
    (HERE / "guide-bounds.json").write_text(
        json.dumps(run(), indent=2) + "\n", encoding="utf8", newline="\n"
    )
