"""Screen whole-robot carpet and exact upright cage for prospective top-only hanger poses.

The input is a local derived copy of one registered upright shallow cage centered
at robot X=Y=0, from the same authenticated FE-2025 source as high_support.py.
"""

import argparse
import itertools
import json
import math
from pathlib import Path
from OCP.STEPControl import STEPControl_Reader
from installed_audit import HERE, ROOT, bounds, box_gap, sha
from attachments import base, common
from clearance import read_assembly, move
from high_support import turn
from lift import CG_BOUNDS


def run(field_cache):
    rd = STEPControl_Reader()
    rd.ReadFile(str(field_cache))
    rd.TransferRoots()
    bare = rd.OneShape()
    chassis = {
        k: s
        for g, k, s in read_assembly(ROOT / "cad/swerve-captive/public-robot.step")
        if g != "Demonstration CORAL"
    }
    step = HERE / "station-and-jack-interfaces.step"
    overrides = base.named(step)
    chassis.update({k: s for k, s in overrides.items() if "rail saddle" not in k})
    r = json.loads((HERE / "interfaces.json").read_bytes())
    for k in r["fixedHeight"]["removed"]:
        chassis.pop(k)
    boxes = {k: bounds(s) for k, s in chassis.items()}
    out = {}
    for x, p in itertools.product((390, 415, 440, 465), (240, 280, 319.4)):
        cage = move(bare, (x, 0, 0))
        h = 1340
        about = (x, 0, h)
        rows = []
        for cg in itertools.product(*CG_BOUNDS):
            dx, dy = 1000 * cg[0] - x, 1000 * cg[1]
            rr = math.hypot(dx, dy)
            nx, ny = -dx / rr, -dy / rr
            angle = math.atan2(rr, h - p - 1000 * cg[2])
            ax = (ny, -nx, 0)
            # Lower every bounding box's eight corners. Any bound below a known
            # minimum requires an exact transformed BRep query.
            lower = []
            for k, b in boxes.items():
                xx = b[0] if nx > 0 else b[3]
                yy = b[1] if ny > 0 else b[4]
                v = (
                    h
                    + ((xx - x) * nx + yy * ny) * math.sin(angle)
                    + (b[2] + p - h) * math.cos(angle)
                )
                lower.append((v, k))
            floor = float("inf")
            name = ""
            for v, k in sorted(lower):
                if v > floor:
                    break
                val = bounds(turn(move(chassis[k], (0, 0, p)), about, ax, angle))[2]
                if val < floor:
                    floor = val
                    name = k
            shape = move(turn(cage, about, ax, -angle), (0, 0, -p))
            cb = bounds(shape)
            hits = {}
            for k, s in chassis.items():
                if box_gap(cb, boxes[k]) < 0.002:
                    ov = common(shape, s)
                    if ov > 0.01:
                        hits[k] = round(ov, 3)
            rows.append(
                dict(cgm=cg, floorMm=round(floor, 3), floorBody=name, overlap=hits)
            )
        out[f"{x},{p}"] = rows
    return dict(
        cacheSha256=sha(field_cache),
        targetInterfaceSha256=sha(step),
        cases=out,
        scope="Static attained-free-equilibrium design screen across eight CG corners. New yoke, flight/approach, field tolerances, cage mass/swing and intermediate guided-to-free lift remain separate.",
    )


if __name__ == "__main__":
    c = argparse.ArgumentParser()
    c.add_argument("registered_cage", type=Path)
    (HERE / "top-yoke-targets.json").write_bytes(
        (json.dumps(run(c.parse_args().registered_cage), indent=2) + "\n").encode()
    )
