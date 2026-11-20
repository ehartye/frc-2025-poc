"""Screen continuous cage-relative translations against all 1,001 stowed robot solids."""

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Trsf, gp_Vec

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "cad/swerve-captive"))
from viewer_source import read_assembly
from exchange import base

sys.path.insert(0, str(ROOT / "experiments/six-wheel/leave-park"))
from cage_motion import hanging_inventory
from native_park import bounds, box_distance, distance


def move(shape, vec):
    tr = gp_Trsf()
    tr.SetTranslation(gp_Vec(*vec))
    return shape.Moved(TopLoc_Location(tr))


def evaluate(cage, parts, direction, lo, hi):
    cb = list(bounds(cage))
    broad = [
        cb[i] + min(direction[i] * lo, direction[i] * hi) - 1 for i in range(3)
    ] + [cb[3 + i] + max(direction[i] * lo, direction[i] * hi) + 1 for i in range(3)]
    candidates = [(k, s) for k, s, b in parts if box_distance(b, broad) == 0]
    out = []
    for name, shape in candidates:

        def interval(a, b):
            mid = (a + b) / 2
            fixed = move(cage, tuple(x * mid for x in direction))
            d = distance(shape, fixed)
            # Distance to a stationary body is 1-Lipschitz in translation.
            margin = d - (b - a) / 2
            if margin > 0.02:
                return dict(status="clear", minimumMm=margin)
            if d < 0.001:
                volume, _ = base.cad.volume(BRepAlgoAPI_Common(shape, fixed).Shape())
                if volume > 0.001:
                    return dict(status="interference", positionMm=mid, volumeMm3=volume)
            if b - a < 0.05:
                return dict(status="unresolved", positionMm=mid, minimumMm=margin)
            l = interval(a, mid)
            if l["status"] != "clear":
                return l
            r = interval(mid, b)
            if r["status"] != "clear":
                return r
            return dict(status="clear", minimumMm=min(l["minimumMm"], r["minimumMm"]))

        out.append(dict(id=name, **interval(lo, hi)))
    return dict(
        considered=len(parts),
        narrow=len(out),
        clearanceMm=min(
            [1.0] + [v["minimumMm"] for v in out if v["status"] == "clear"]
        ),
        adverse=[r for r in out if r["status"] != "clear"],
    )


def run(field, center_x):
    stations, shapes = hanging_inventory(field)
    source = ROOT / "cad/swerve-captive/public-robot.step"
    parts = [
        (k, s, bounds(s))
        for group, k, s in read_assembly(source)
        if group != "Demonstration CORAL"
    ]
    assert len(parts) == 1001
    results = {}
    for side, key, sign in [("red", "NAUO260", -1), ("blue", "NAUO263", 1)]:
        station = stations[key]
        x, z = station["axis_mm"]
        tr = gp_Trsf()
        tr.SetValues(sign, 0, 0, center_x - sign * x, 0, 0, -sign, sign * z, 0, 1, 0, 0)
        cage = shapes[station["cage"]].Moved(TopLoc_Location(tr))
        results[side] = dict(
            frontApproach=evaluate(cage, parts, (1, 0, 0), 0, 650),
            positiveSideApproach=evaluate(cage, parts, (0, 1, 0), 0, 650),
            negativeSideApproach=evaluate(cage, parts, (0, -1, 0), 0, 650),
            suspendedTravel=evaluate(cage, parts, (0, 0, -1), 0, 319.4),
        )
    return dict(
        robotSha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        fieldSha256=hashlib.sha256(field.read_bytes()).hexdigest(),
        cageCenterRobotXYmm=[center_x, 0],
        cases=results,
        scope="Original empty stowed assembly against complete own shallow cage during the final straight approach and prescribed coupled lift. Relative cage and chassis attitudes locked after clamping. The lift assumes guided adhesion-free top and bottom contact. Other field material and the jack require separate review.",
    )


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("field", type=Path)
    ap.add_argument("output", type=Path)
    ap.add_argument("--center-x", type=float, default=-218)
    args = ap.parse_args()
    args.output.write_bytes(
        (json.dumps(run(args.field, args.center_x), indent=2) + "\n").encode()
    )
