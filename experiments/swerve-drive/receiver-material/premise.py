"""Compare what the four camera pupils, end pads, encoders and restraint can witness."""

import argparse
import json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def read(path):
    return json.loads(path.read_bytes())


def run(root):
    models = read(ROOT / "cad/full-swerve-r4/appearances.json")["solids"]
    pads = sorted(
        (r for r in models if r["id"].startswith("observer-moving-pad-")),
        key=lambda r: r["id"],
    )
    if len(pads) != 3:
        raise ValueError("Missing installed end pad")
    # The 7.2 mm wide disk ends at the body's most forward edge. The thin tail
    # toward X125 carries the circuit. It is outside the pressure-sensitive area.
    disk_min = min(
        r["boundsMm"][3] - (r["boundsMm"][4] - r["boundsMm"][1]) for r in pads
    )
    by_case = []
    for path in sorted(root.glob("rho-*/qualified-rims-core.json")):
        state = read(path)
        gaps = []
        for shot in state["physicalPipeAtCaptures"]:
            ax = shot["axis"][0]
            x = shot["centerMm"][0]
            front = x + 301.625 / 2 * abs(ax) + 57.15 * np.sqrt(1 - ax * ax)
            gaps.append(disk_min - front)
        by_case.append(
            dict(
                case=path.parent.name,
                minimumForeAftPadGapMm=min(gaps) if gaps else None,
                poseSha256=state["terminalStateSha256"],
            )
        )
    chosen = root / "rho-950-mu-0.05-plus-31.25us"
    material = read(chosen / "result.json")
    accepted = read(chosen / "qualified-rims-core.json")["observations"]["nominal"]
    nominal = read(chosen / "loaded/result.json")
    fault = read(chosen / "loaded/no-belt-traction/result.json")
    if not (
        nominal["service"] == fault["service"]
        and nominal["drive"] == fault["drive"]
        and nominal["materialState"]
        == fault["materialState"]
        == material["feed"]["pipeStateSha256"]
    ):
        raise ValueError("Lost common pre-fault state or applied wheel input")
    traj = chosen / "loaded/plus-600.npz"
    with np.load(traj) as control:
        peak = float(np.linalg.norm(control["bodyAcceleration"][:, :2], axis=1).max())
    baseline = np.load(chosen / "loaded" / nominal["contact"]["file"])
    failure = np.load(chosen / "loaded/no-belt-traction" / fault["contact"]["file"])
    end = min(baseline[-1, 0], failure[-1, 0])
    keep = baseline[:, 0] <= end + 1e-10
    common = baseline[keep]
    np.testing.assert_array_equal(baseline[0], failure[0])
    diff = np.stack(
        [
            np.interp(common[:, 0], failure[:, 0], failure[:, idx]) - common[:, idx]
            for idx in (1, 2, 3)
        ],
        axis=1,
    )
    shift = 1000 * np.linalg.norm(diff, axis=1)
    first_shift = {}
    for threshold in (0.25, 1.0, 2.0, 4.0, 8.0):
        hits = np.flatnonzero(shift > threshold)
        first_shift[str(threshold)] = float(common[hits[0], 0]) if len(hits) else None
    # This screen addresses only translation and assumes level ground and full weight
    # on the shelf. It omits belt constraint, rocking, safety factor and angular terms.
    return dict(
        scope="Same-pipe conditional first station departure",
        sourceDiskIds=[r["id"] for r in pads],
        sensitiveForemostRangeMm=[disk_min, pads[0]["boundsMm"][3]],
        padGaps=by_case,
        matchedInputs={
            "wheelServiceSha256": nominal["service"]["outgoing"],
            "driveSha256": nominal["drive"]["trajectory"],
            "preFaultExposureSha256": accepted["images"],
            "surfaceFriction": material["shelfFriction"],
        },
        firstLease={
            "peakHorizontalAccelerationMps2": peak,
            "pipeMassKg": 0.816466266,
            "peakInertialDemandN": 0.816466266 * peak,
            "shelfOnlyIdealStaticCapN": material["shelfFriction"]
            * 0.816466266
            * 9.80665,
            "nominal": nominal["contact"],
            "zeroBeltTraction": fault["contact"],
            "commonHistoryS": float(end),
            "peakCenterDifferenceMm": float(shift.max()),
            "firstCenterDifferenceAboveMmS": first_shift,
        },
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("raw", type=Path)
    p.add_argument("output", type=Path)
    args = p.parse_args()
    args.output.write_text(
        json.dumps(run(args.raw), indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
