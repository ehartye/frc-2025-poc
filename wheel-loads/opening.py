"""Apply the six-wheel contact sweep to the actual corrected pre-feed opening clocks."""

import argparse, json
from pathlib import Path
import numpy as np
from drive_replay import HERE, ROOT, sha, read, screen, wheel_steps


def check(assets, registered):
    sources = read(registered)
    info = read(HERE / "contact.json")
    kg = info["mass"]["noCutCreditKg"]["emptyFieldKg"][1]
    shares = (
        2 / 3,
        0.70,
        0.75,
        0.775,
        0.8,
        0.825,
        0.85,
        0.875,
        0.9,
        0.925,
        0.95,
        0.975,
        1.0,
    )
    out = {}
    for side, row in sources.items():
        path = assets / "straight-guide-match" / (side + "-revised-opening.npz")
        if sha(path) != row["pathSha256"]:
            raise ValueError("Revised station wheel path differs")
        with np.load(path) as d:
            t, pose = d["t"], d["pose"]
        if (
            len(t) != 4221
            or not np.allclose(np.diff(t), 0.005)
            or not np.isclose(t[-1], 21.10)
        ):
            raise ValueError("Require the achieved 0–21.10 s opening")
        arcs = wheel_steps(pose)
        grids = {}
        for ohms in (0.027, 0.032):
            data = []
            for outer in shares:
                r = screen(
                    t,
                    arcs,
                    np.full(len(arcs), "opening"),
                    {"coral-A": {"feed": 21.1}},
                    kg,
                    "conditioned",
                    (11.8, ohms),
                    assets,
                    outer=outer,
                    side=0.44,
                )
                data.append(
                    {
                        "ends": outer,
                        **{
                            k: r[k]
                            for k in (
                                "peakMotorPhaseA",
                                "peakFrictionRequired",
                                "minTerminalV",
                                "firstReferenceInhibitWallS",
                            )
                        },
                    }
                )
            grids[str(ohms)] = {
                "trials": data,
                "lastPassingTestedShare": max(
                    (
                        r["ends"]
                        for r in data
                        if r["firstReferenceInhibitWallS"] is None
                    ),
                    default=None,
                ),
                "firstFailingTestedShare": min(
                    (
                        r["ends"]
                        for r in data
                        if r["firstReferenceInhibitWallS"] is not None
                    ),
                    default=None,
                ),
            }
        speeds = arcs / 0.005
        omega = (speeds[:, 1] - speeds[:, 0]) / 0.580
        alpha = (
            np.maximum(abs(np.diff(omega, prepend=0)), abs(np.diff(omega, append=0)))
            / 0.005
        )
        lateral = abs(np.mean(speeds, axis=1) * omega) + 0.05 * (alpha + omega**2)
        out[side] = {
            "clockSha256": sha(path),
            "endS": float(t[-1]),
            "observedStationWallS": row["stationWallS"],
            "grid": grids,
            "maxLateralAccelerationAt50mmMS2": float(lateral.max()),
            "peakLateralWallS": float(t[np.argmax(lateral)]),
        }
    return {
        "sourceReceiptSha256": sha(registered),
        "emptyMassKg": kg,
        "cargoKg": 0,
        "minSideFraction": 0.44,
        "signed": out,
        "scope": "pre-feed approach; subsequent pickup and placement are separate",
        "physicalTraction": None,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets", type=Path)
    p.add_argument("register", type=Path)
    a = p.parse_args()
    (HERE / "opening.json").write_text(
        json.dumps(check(a.assets, a.register), indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
