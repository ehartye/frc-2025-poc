"""Check the unpowered geometric envelope, pre-contact opening and powered closing target."""

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def read(path):
    return json.loads(path.read_bytes())


def sha(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def reservations(stroke):
    """One positive-Y end in chassis millimeters. Reflect Y for the other end.

    These are outer stock reservations, including the full radial gear sweep.
    A closing command moves the two yokes in opposite directions.
    """

    def yoke(x0, y0, z0, x1, y1, z1):
        return [x0, y0 - stroke, z0, x1, y1 - stroke, z1]

    return {
        "aft-tube": yoke(35, 161, 575, 43, 221, 583),
        "front-tube": yoke(106, 161, 575, 114, 221, 583),
        "crossbar": yoke(34, 219, 575, 123, 225, 583),
        "probe-post": yoke(72, 219, 582, 78, 225, 613),
        "rim-probe": yoke(74.25, 183, 610.25, 75.75, 222, 611.75),
        "rack": yoke(119, 183, 577.3, 123, 229, 583),
        "gear-sweep": [119, 164.7, 533.7, 123, 211.3, 580.3],
        "servo-body-reserve": [91, 175, 546, 119, 207, 570],
        "slide-foot": [44, 194, 500, 59, 209, 503],
        "support-post": [49, 197, 503, 53, 203, 575],
        "guide-beam": [34, 174, 573, 114, 183, 585],
        "beam-brace": [46, 178, 568, 54, 203, 575],
        "motor-mount": [107, 183, 570, 118, 196, 575],
    }


def package():
    cad = ROOT / "cad/full-swerve-r4/appearances.json"
    roles = ROOT / "cad/swerve-articulation/motion-groups.json"
    appearance = read(cad)
    if sha(cad.with_name("full-robot.step")) != appearance["sourceSha256"]:
        raise ValueError("Robot source differs from the ordered bounds.")
    groups = read(roles)
    rows = appearance["solids"]
    bounds = np.array([r["boundsMm"] for r in rows])
    travel = np.array(
        [
            0
            if r["id"] in groups["slide"]
            else 185
            if r["id"] in groups["middle"]
            else 370
            for r in rows
        ]
    )
    # Translate existing bodies to their swept envelopes in the OUTER slide frame.
    bounds[:, 0] -= travel
    observed = {}
    opened, closed = reservations(0), reservations(35)
    for side in (-1, 1):
        for name, box in opened.items():
            a = np.minimum(box[:3], closed[name][:3])
            b = np.maximum(box[3:], closed[name][3:])
            if side < 0:
                a[1], b[1] = -b[1], -a[1]
            intersection = np.minimum(bounds[:, 3:], b) - np.maximum(bounds[:, :3], a)
            observed[f"{side}/{name}"] = [
                rows[i]["id"]
                for i in np.flatnonzero(np.all(intersection > 0.0001, axis=1))
            ]
    return dict(
        source=appearance["sourceSha256"],
        appearance=sha(cad),
        frames=sha(roles),
        solids=len(rows),
        coveredStrokeMm=[0, 370],
        closedTravelMm=[0, 35],
        positiveVolumeBoxOverlaps=observed,
    )


def receiving(artifacts):
    source = HERE.parent / "motion/loaded-feed.json"
    results = []
    for feed in read(source)["attempts"]:
        side, step = feed["case"], feed["plantStepUs"]
        folder = artifacts / "swerve-energy/contact" / side
        files = [p for p in folder.glob(f"*folded-{step:g}us*/*-recover.npy")]
        (path,) = files
        if sha(path) != feed["pipeTrajectorySha256"]:
            raise ValueError("Receiver source trajectory changed.")
        raw = np.load(path, mmap_mode="r")
        ay = np.clip(abs(raw[:, 5]), 0, 1)
        # Support of an outer finite cylinder in the lateral direction.
        extreme = abs(raw[:, 2]) + 150.8125 * ay + 57.15 * np.sqrt(1 - ay**2)
        index = int(np.argmax(extreme))
        results.append(
            dict(
                case=side,
                stepUs=step,
                trajectory=sha(path),
                samples=len(raw),
                largestLateralExtentMm=float(extreme[index]),
                minimumToOpenTipsMm=float(161 - extreme[index]),
                javaMsAtMinimum=float(raw[index, 0]),
            )
        )
    return results


def servo(peak_probe_n):
    radius = 0.0223
    route_speed = 0.150
    efficiency = 0.80
    guide_drag = 0.25
    result = []
    for volts, free_seconds, stall_kgcm in [(6, 0.11, 2.2), (4.8, 0.14, 1.8)]:
        free_rate = math.radians(60) / free_seconds
        available = (
            stall_kgcm * 0.0980665 * max(0, 1 - route_speed / radius / free_rate)
        )
        required = radius * (peak_probe_n + guide_drag) / efficiency
        result.append(
            dict(
                volts=volts,
                strokeMm=35,
                targetMmPerS=150,
                sweepDegrees=math.degrees(0.035 / radius),
                modeledAvailableNm=available,
                modeledRequiredNm=required,
                modeledMarginNm=available - required,
                passes=available >= required,
            )
        )
    return result


def run(artifacts):
    capture = {
        s: read(artifacts / "retained-coral" / s / "capture-1/receipt.json")
        for s in ("minus", "plus")
    }
    results = dict(
        schema="reefscape-retainer-screen/1",
        robot=package(),
        station=receiving(artifacts),
        stationary=capture,
        servo=servo(max(r["peakProbeN"] for r in capture.values())),
    )
    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifacts", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.write_text(
        json.dumps(run(args.artifacts), indent=2) + "\n", encoding="utf-8"
    )
