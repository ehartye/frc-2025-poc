"""Convert the remaining placement uncertainty to a modeled lateral CG budget."""

import json
from contact import HERE, read


def run():
    moments = read(HERE / "moments.json")
    opening = read(HERE / "opening.json")
    contact = read(HERE / "contact.json")
    mass = moments["fieldMassUpperKg"]
    remainder = moments["unlocatedUpperRemainderKg"]
    lo, hi = moments["knownLateralMomentKgMm"]
    track = contact["trackMm"]
    g = 9.80665
    accel = max(
        r["maxLateralAccelerationAt50mmMS2"] for r in opening["signed"].values()
    )
    rows = {}
    for h in (0, 250, 350, 450):
        # h=0 isolates the static condition instead of claiming a zero-height chassis.
        allowance = contact["static44pctYmaxMm"] - h * accel / g
        limits = [
            (-mass * allowance - lo) / remainder,
            (mass * allowance - hi) / remainder,
        ]
        cases = {}
        for y in (-40, -20, 0, 20, 40):
            cg = [(remainder * y + v) / mass for v in (lo, hi)]
            worst = max(map(abs, cg)) + h * accel / g
            cases[str(y)] = {
                "wholeCgYmm": cg,
                "minimumSideFraction": 0.5 - worst / track,
            }
        rows[str(h)] = {
            "maxChassisCgYMm": allowance,
            "permittedRemainderYmm": limits,
            "placementControls": cases,
        }
    return {
        "maxOpeningLateralAccelerationMS2": accel,
        "assumedHorizontalCgRadiusMm": 50,
        "stationaryAndOpening": rows,
        "selectedFullCgMeasuredMm": None,
    }


if __name__ == "__main__":
    (HERE / "balance.json").write_text(
        json.dumps(run(), indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
