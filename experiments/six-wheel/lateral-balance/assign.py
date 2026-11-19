"""Select six interchangeable wheel articles by modeled mass, effective radius and rate."""

import argparse
import itertools
import json
from pathlib import Path
import math

EXAMPLE = [
    {"id": name, "kg": kg, "radiusErrorMm": radius, "rateRatio": rate}
    for name, kg, radius, rate in (
        ("A", 0.235, -0.15, 0.96),
        ("B", 0.242, 0.07, 1.00),
        ("C", 0.257, -0.11, 1.05),
        ("D", 0.260, 0.12, 0.96),
        ("E", 0.284, -0.07, 1.02),
        ("F", 0.289, 0.15, 1.06),
    )
]


def arrange(items):
    if len(items) != 6 or len({r["id"] for r in items}) != 6:
        raise ValueError("Six unique wheel articles required")
    for r in items:
        vals = [r[k] for k in ("kg", "radiusErrorMm", "rateRatio")]
        if (
            not all(math.isfinite(x) for x in vals)
            or not 0.20 <= vals[0] <= 0.30
            or abs(vals[1]) > 0.1524
            or not 0.9 <= vals[2] <= 1.1
        ):
            raise ValueError("Wheel outside the selected mass, height or rate envelope")
    best = None
    for p in itertools.permutations(items):
        frontL, frontR, centerL, centerR, rearL, rearR = p
        if any(
            c["radiusErrorMm"] < max(f["radiusErrorMm"], r["radiusErrorMm"])
            for f, c, r in ((frontL, centerL, rearL), (frontR, centerR, rearR))
        ):
            continue
        mismatch = max(abs(a["kg"] - b["kg"]) for a, b in zip(p[::2], p[1::2]))
        if mismatch > 0.020 + 1e-12:
            continue
        score = (mismatch, tuple(r["id"] for r in p))
        if best is None or score < best[0]:
            best = (score, p)
    if not best:
        return {"assignable": False, "assignment": None}
    return {
        "assignable": True,
        "maxAxleMismatchKg": best[0][0],
        "assignment": dict(
            zip(
                ("front-L", "front-R", "center-L", "center-R", "rear-L", "rear-R"),
                best[0][1],
            )
        ),
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument("--example", action="store_true")
    group.add_argument("--wheels", type=Path)
    a = p.parse_args()
    print(
        json.dumps(
            arrange(EXAMPLE if a.example else json.loads(a.wheels.read_bytes())),
            indent=2,
        )
    )
