"""Cold native wall and finite tube at attained poses before and during separation."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "score-stroke"))
from native_score import LIP, wall_from_field, cad
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepExtrema import BRepExtrema_DistShapeShape


def read(p):
    return json.loads(p.read_bytes())


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def sample_wall(wall, center, axis):
    start = center - 150.8125 * axis
    line = gp_Ax2(gp_Pnt(*start), gp_Dir(*axis))
    body = BRepAlgoAPI_Cut(
        BRepPrimAPI_MakeCylinder(line, 57.15, 301.625).Shape(),
        BRepPrimAPI_MakeCylinder(line, 50.8, 301.625).Shape(),
    ).Shape()
    distance = BRepExtrema_DistShapeShape(body, wall)
    if not distance.IsDone():
        raise ValueError("Failed tube/wall distance")
    gap = distance.Value()
    return gap, cad.intersection(body, wall) if gap < 1e-6 else 0.0


def run(field, files):
    official = read(HERE.parent / "score-stroke/native.json")
    if digest(field) != official["fieldSha256"]:
        raise ValueError("Wrong field STEP")
    wall = wall_from_field(field)
    for path in files:
        row = read(path)
        f = path.with_suffix(".npy")
        if digest(f) != row["trace"]:
            raise ValueError("Wrong coupled pipe trace")
        dock = read(path.parent / "chassis.json")["terminalMmMmRad"]
        co, si = math.cos(dock[2]), math.sin(dock[2])
        mat = np.array([[co, -si, 0], [0, 0, 1], [-si, -co, 0]])
        offset = np.array([LIP - 490 + dock[0], 0.0, -dock[1]])
        samples = []
        for record in np.load(f):
            if record[0] < 3.3:
                continue
            center = 1000 * (mat @ record[2:5]) + offset
            axis = mat @ record[5:8]
            # Simple all-axis capsule bound discards the early high tube frames.
            if center[1] - abs(axis[1]) * 150.8125 - 57.15 > 457.2:
                continue
            gap, overlap = sample_wall(wall, center, axis)
            samples.append(
                dict(
                    elapsedS=record[0],
                    wallS=record[0] + row["startWallS"],
                    shelfReactionN=record[16],
                    gapMm=gap,
                    commonMm3=overlap,
                )
            )
        if row["event"]:
            e = row["event"]
            gap, volume = sample_wall(
                wall, 1000 * (mat @ e["chassisP"]) + offset, mat @ e["axis"]
            )
            final = dict(time=e["time"], gapMm=gap, commonMm3=volume)
        else:
            final = None
        result = dict(
            source=digest(path),
            trace=digest(f),
            field=official["fieldSha256"],
            wall=official["fieldPart"],
            pose=dock,
            exit=final,
            steps=samples,
            firstPositiveOverlap=next(
                (r for r in samples if r["commonMm3"] > 0.01), None
            ),
            maxOverlapMm3=max((r["commonMm3"] for r in samples), default=0.0),
            scope="Exact 5 ms sampled positives disprove noncontact. Discrete nonintersection alone does not certify an unobserved interval.",
        )
        path.with_suffix(".wall.json").write_bytes(
            (json.dumps(result, indent=2) + "\n").encode()
        )
        print(row["side"], result["firstPositiveOverlap"], final, flush=True)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("field", type=Path)
    p.add_argument("runs", nargs="+", type=Path)
    a = p.parse_args()
    run(a.field, a.runs)
