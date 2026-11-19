"""Screen both inner rails through the loaded cam cycle and each possible dwell release."""

import argparse
import json
import math
from pathlib import Path
import numpy as np
from native_score import ROOT, SIX, LIP, SurfaceTrace, digest, read, release_control
from dynamics import plate


def loaded_cap_direction(trace_path, trace):
    with np.load(trace_path) as data:
        force = data["buttonN"]
    rows = []
    for side, y in enumerate((-0.066, 0.042)):
        ix = np.flatnonzero(force[:, side] > 0.1)
        x = 0.118 - trace.stroke[ix]
        yy = np.full_like(x, y)
        q = trace.q[ix]
        sx = q[:, 5] + np.einsum(
            "ij,ij->i", plate.basis(x, yy, 13, 20, 1, 0) @ trace.shapes, q[:, 6:]
        )
        sy = np.einsum(
            "ij,ij->i", plate.basis(x, yy, 13, 20, 0, 1) @ trace.shapes, q[:, 6:]
        )
        # Charge the ramp slope any time the cap's longitudinal footprint
        # touches the transition, including overlap with a flat endpoint.
        on_ramp = (x - 0.0032 < 0.115) & (x + 0.0032 > 0.001)
        sx = np.maximum(abs(sx), abs(sx - np.where(on_ramp, 3 / 114, 0)))
        angle = np.arctan(np.hypot(sx, sy))
        loads = {
            str(mu): float(max(force[ix, side] * np.tan(angle + math.atan(mu))))
            for mu in (0.0, 0.05, 0.1)
        }
        rows.append(
            dict(
                side=side,
                minimumReportedNormalN=0.1,
                sampledContactStates=len(ix),
                maximumSurfaceAngleDeg=float(np.degrees(angle).max()),
                peakHorizontalReactionNByAssumedMu=loads,
            )
        )
    return rows


def run(trace_path, receipt_path):
    receipt = read(receipt_path)
    trace = SurfaceTrace(trace_path, receipt["traceSha256"])
    appearances = read(ROOT / "cad/full-swerve-r4/appearances.json")
    rails = [
        r
        for r in appearances["solids"]
        if r["id"] in ("drilled-slide-L-inner", "drilled-slide-R-inner")
    ]
    original = read(Path(__file__).with_name("native.json"))
    nav = read(SIX / "relieved-match/navigation.json")["cases"]
    result = {}
    for piece in ("coral-A", "coral-B"):
        old = original["release"][piece]
        # The source CAD comparison uses only these three registered wall planes.
        front, back = old["wallXFromLipMm"]
        bounds = [LIP + front, 0, 0, LIP + back, old["wallTopMm"], 0]
        dx, _, yaw = next(
            r["finalBodyErrorMmDeg"]
            for r in nav
            if r["pipe"] == piece and r["mode"] == "nominal"
        )
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        phi = trace.q[:, 5]
        rail_min = float("inf")
        for row in rails:
            bx, by, bz, ex, ey, ez = row["boundsMm"]
            from itertools import product

            for x, y, z in product((bx, ex), (by, ey), (bz, ez)):
                local_x = (
                    np.cos(phi) * x - np.sin(phi) * (z - 467) + 1000 * trace.stroke
                )
                rail_min = min(
                    rail_min, float(np.min(490 + front - dx + 42 - c * local_x + s * y))
                )
        dwell = np.flatnonzero((trace.time >= 3) & (trace.time <= 3.3))
        margins = []
        exact = {d: [] for d in (42, 50, 52, 54, 56, 58, 60, 65, 70, 80, 100)}
        for i in dwell:
            flight = release_control(piece, bounds, trace, trace.time[i])
            case = next(r for r in flight["cases"] if r["setbackMm"] == 42)
            margins.append(case["farFaceMarginMm"])
            for entry in flight["cases"]:
                if entry["setbackMm"] in exact:
                    exact[entry["setbackMm"]].append(
                        entry["continuousCapsuleClearanceMm"]
                    )
        matched = [
            dict(
                setbackMm=d,
                railGapLowerMm=rail_min + d - 42,
                flightGapLowerMm=min(values),
                combinedGapLowerMm=min(rail_min + d - 42, min(values)),
            )
            for d, values in exact.items()
        ]
        result[piece] = dict(
            addedSetbackMm=42,
            railForeAftSeparatingPlaneMinMm=rail_min,
            releaseWindowS=[float(trace.time[dwell[0]]), float(trace.time[dwell[-1]])],
            cases=len(dwell),
            pipeFarFaceMarginMm=[min(margins), max(margins)],
            worstReleaseS=float(trace.time[dwell[np.argmin(margins)]]),
            setbackSweep=matched,
        )
    return dict(
        trajectorySha256=digest(trace_path),
        candidateStepSha256=receipt["camStepSha256"],
        issuedFixture=receipt["issuedFixture"],
        samples=len(trace.time),
        capSlidingSensitivity=loaded_cap_direction(trace_path, trace),
        cases=result,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for key in ("trace", "receipt", "output"):
        p.add_argument(key, type=Path)
    args = p.parse_args()
    args.output.write_text(
        json.dumps(run(args.trace, args.receipt), indent=2, allow_nan=False) + "\n",
        encoding="utf8",
    )
