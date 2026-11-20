"""Bound robot separation from the near face of the official low wall at both stopped docks."""

from pathlib import Path
import argparse
import hashlib
import json
import math

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
NATIVE = (
    ROOT / "cad/assembly-engineering/receiver-service/issue/engineering-reopen.json"
)
FIELD = HERE.parent / "score-stroke/native.json"


def read(path):
    return json.loads(path.read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def evaluate(bodies, enclosure, commanded, tracking, pitch, heave):
    x_far = enclosure[1][0]
    yaw = max(abs(v[2]) for v in enclosure)
    distances = []
    for body in bodies:
        if body["instance"] is None:
            continue
        occurrence = body["occurrence"].split(":")[0]
        ratio = {
            "Middle slide": 0.5,
            "Outer slide": 1,
            "Left capture yoke": 1,
            "Right capture yoke": 1,
        }.get(occurrence, 0)
        x0, y0, z0, x1, y1, z1 = body["boundsMm"]
        # Enclose pitch about the carriage's 467 mm datum in its own frame
        # before extension. Treat motor/encoder error as possible overtravel.
        phi = pitch if ratio else 0
        zarm = max(abs(z0 - 467), abs(z1 - 467))
        xarm = max(abs(x0), abs(x1))
        low = (
            z0
            - math.sin(phi) * xarm
            - (1 - math.cos(phi)) * zarm
            - (heave if ratio else 0)
        )
        if low > 457.2:
            continue
        front = (
            x1
            + math.sin(phi) * zarm
            + (1 - math.cos(phi)) * xarm
            + ratio * (commanded + tracking)
        )
        lateral = max(abs(y0), abs(y1))
        projected = max(front, front * math.cos(yaw)) + lateral * math.sin(yaw)
        distances.append(
            (490 - x_far - projected, body["occurrence"] + "/" + body["body"], low)
        )
    gap, name, low = min(distances)
    return dict(
        commandMm=commanded,
        actualPlusMm=tracking,
        pitchLimitRad=pitch,
        heaveLimitMm=heave,
        minimumPlaneGapMm=gap,
        limitingBody=name,
        lowerZBoundMm=low,
        possibleBodiesBelowWall=len(distances),
        clears=gap > 0,
    )


def run(artifacts):
    native = read(NATIVE)
    field = read(FIELD)
    if native["nativeBodyCount"] != 1269 or not math.isclose(
        field["release"]["coral-A"]["wallTopMm"], 457.2, abs_tol=1e-7
    ):
        raise ValueError("Unexpected robot or reef")
    cases = {}
    for side in ("plus", "minus"):
        folder = artifacts / "low-placement/advance-25" / side
        reg = read(folder / "chassis.json")
        if not reg["stopped"] or not reg["enclosure"]:
            raise ValueError("Missing observed stationary enclosure")
        cases[side] = dict(
            chassisSha256=sha(folder / "chassis.json"),
            enclosure=reg["enclosure"],
            trials=[
                evaluate(native["bodies"], reg["enclosure"], mm, tol, 0.04, 5)
                for mm, tol in [(345, 0), (345, 2), (337, 2), (337, 8)]
            ],
        )
    return dict(
        nativeSha256=sha(NATIVE),
        issuedArchive=native["archiveSha256"],
        field=field["fieldSha256"],
        wall=field["fieldPart"],
        basis="Reopened parked bounds for 1268 installed bodies. Plane X490 spans the full sector at all heights through 457.2 mm. Positive is a sufficient clearance; a negative bound calls for refinement.",
        limits="Stowed jack, closed capture yokes, parked wheel poses during stroke. Horizontal straight slide under 0.04 rad pitch and 5 mm heave. 2 mm is the project SP1 stroke allowance. Covers one stationary field wall, not ALL reef components or flexible leads.",
        cases=cases,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("artifacts", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    a.output.write_bytes((json.dumps(run(a.artifacts), indent=2) + "\n").encode())
