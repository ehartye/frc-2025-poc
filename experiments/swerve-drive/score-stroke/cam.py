"""Machine continuous absorber lands in the R4 insert and corresponding trailing relief."""

import argparse
import math
import sys
from pathlib import Path
from native_score import ROOT, SIX, R4, cad, digest, named, save

sys.path.insert(0, str(SIX / "shelf-structure"))
from tray import (
    FACE,
    TRAY,
    box,
    subtract,
    unite,
    write_step,
    finalize_step,
    volume,
    common,
)
from installed import prism_y
from OCP.BRepCheck import BRepCheck_Analyzer
import numpy as np

YS = (-66.0, 42.0)
HIGH_X, LOW_X = 115.0, 1.0
RISE, WIDTH = 3.0, 10.0


def lift(x):
    return RISE * np.clip((HIGH_X - x) / (HIGH_X - LOW_X), 0.0, 1.0)


def manufacture(bodies):
    face, insert = bodies[FACE], bodies[TRAY]
    for y in YS:
        # Retain aluminum from the original billet up to the 496 mm skin. Forward
        # end terminates inside the original stow land. No bonded runner or screw.
        runner = prism_y(
            [(LOW_X, 485 + RISE), (HIGH_X, 485), (118, 485), (118, 496), (LOW_X, 496)],
            y - WIDTH / 2,
            WIDTH,
        )
        insert = unite(insert, runner)
        cutter = prism_y(
            [
                (-105, 480),
                (HIGH_X, 480),
                (HIGH_X, 485),
                (LOW_X, 485 + RISE),
                (-105, 485 + RISE),
            ],
            y - WIDTH / 2,
            WIDTH,
        )
        insert, face = subtract(insert, cutter), subtract(face, cutter)
    return {FACE: face, TRAY: insert}


def run(output):
    robot = ROOT / "cad/full-swerve-r4/full-robot.step"
    if digest(robot) != R4:
        raise ValueError("Unregistered robot")
    bodies = named(robot)
    changed = manufacture(bodies)
    path = output / "cam-lands.step"
    write_step(path, changed)
    finalize_step(path)
    changed = named(path)
    delta = {}
    for key, body in changed.items():
        if not BRepCheck_Analyzer(body).IsValid() or cad.solid_count(body) != 1:
            raise ValueError("Invalid machined stock")
        delta[key] = volume(body) - volume(bodies[key])
    gap_rows = []
    for y in YS:
        # Compare exact swept cap disk to an analytical guaranteed lower floor.
        for stroke in (
            0,
            3,
            6.2,
            10,
            21.375,
            24,
            50,
            75,
            100,
            115,
            120,
            125,
            145,
            175,
            214,
            218,
        ):
            cap = bodies[f"receiver-absorber-cap-{YS.index(y)}"]
            floor = 0.1 + float(lift(118 - stroke + 3.2))
            index = YS.index(y)

            def intersection(dz):
                head = cad.translated(cap, [-stroke, 0, dz])
                return sum(cad.intersection(head, shape) for shape in changed.values())

            gap_rows.append(
                dict(
                    strokeMm=stroke,
                    side=index,
                    minimumGapMm=floor,
                    belowFloorCommonMm3=intersection(floor - 0.001),
                    aboveFloorCommonMm3=intersection(floor + 0.02),
                )
            )
    top = box(-101, -211, 496, 166, 211, 501)
    stow = [box(114.8, y - 3.2, 484.9, 121.2, y + 3.2, 496.0) for y in YS]
    retained = [
        volume(common(bodies[TRAY], w)) - volume(common(changed[TRAY], w)) for w in stow
    ]
    charged = max(0, delta[TRAY]) * 2850e-9 + max(0, delta[FACE]) * 980e-9
    result = dict(
        originalRobotSha256=R4,
        candidateStepSha256=digest(path),
        dimensionsMm=dict(
            axesY=YS,
            landWidth=WIDTH,
            fullDepthX=LOW_X,
            transitionX=HIGH_X,
            rise=RISE,
            faceZ=500.0,
            originalStowFloorZ=485.0,
        ),
        slopeDeg=math.degrees(math.atan(RISE / (HIGH_X - LOW_X))),
        volumeDeltaMm3=delta,
        upperAddedMassKgIgnoringRemovedStock=charged,
        topLayerChangedMm3=volume(
            subtract(common(bodies[FACE], top), common(changed[FACE], top))
        ),
        originalStowLandVolumeLossMm3=retained,
        capChecks=gap_rows,
    )
    if any(r["belowFloorCommonMm3"] > 1.0e-5 for r in gap_rows):
        raise ValueError("Candidate floor function misses native material")
    save(output / "cam.json", result)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    run(parser.parse_args().output)
