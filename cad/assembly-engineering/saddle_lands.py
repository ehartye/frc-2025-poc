"""Match all eight saddles to the purchased module's independently recovered tab planes."""

import argparse
import json
import math
import sys
from pathlib import Path
import numpy as np
from OCP.BRep import BRep_Builder
from OCP.BRepTools import BRepTools
from OCP.TopoDS import TopoDS_Shape
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Trsf
from installed_audit import HERE, ROOT, sha
from attachments import common, dist, tube, fuse, cut, cyl, hexagon, bounds, base
from clearance import move

PKG = ROOT / "experiments/swerve-drive/packaging"
sys.path.insert(0, str(PKG))
from module_placement import SITES, basis


def apply(robot, repl):
    survey = json.loads((HERE / "tab-fasteners.json").read_bytes())
    lands = {}
    for k, row in survey["stations"].items():
        short, pitch = k.rsplit("-", 1)
        name = short + "-tab-saddle"
        shape = repl.get(name, robot[name])
        x, y = row["x"], row["y"]
        low = row["plateIntervals"][0][1]
        high = row["plateIntervals"][1][0]
        for face, start, end in (("under", 69.85, 69.9), ("over", 120.6, 120.65)):
            land = tube(x, y, start, 11, 5.3, end - start)
            shape = fuse(shape, land)
            lands[k + "-" + face] = land
        repl[name] = shape
        # Fastener installed under-head length and retained top nut catalog bounds.
        bolt = short + "-tab-screw-" + pitch
        z = row["plateIntervals"][0][0]
        shaft = cyl(x, y, z, 4.826 / 2, 69.85)
        head = cyl(x, y, z - 0.190 * 25.4, 0.312 * 25.4 / 2, 0.190 * 25.4)
        repl[bolt] = cut(fuse(head, shaft), hexagon(x, y, z - 4.87, 3.96875, 2.6))
        nut = short + "-tab-locknut-" + pitch
        zz = row["plateIntervals"][1][1]
        repl[nut] = cut(
            hexagon(x, y, zz, 0.375 * 25.4, 6.3246),
            cyl(x, y, zz - 0.01, 4.826 / 2, 6.4),
        )
    return lands


def test(assets):
    report = json.loads((HERE / "tab-fasteners.json").read_bytes())
    if sha(assets / "REV-21-3005.STEP") != report["sourceModule"]:
        raise ValueError("Wrong purchased module")
    plates = {}
    for part in ("9-REV-21-3005-P01", "10-REV-21-3005-P02"):
        b = TopoDS_Shape()
        assert BRepTools.Read_s(
            b, str(assets / "vendor-native" / (part + ".brep")), BRep_Builder()
        )
        plates[part] = b
    frame = base.named(PKG / "swerve-frame.step")
    repl = {}
    lands = apply(frame, repl)
    out = {}
    for k, s in lands.items():
        site = k[:2]
        under = k.endswith("-under")
        m = np.column_stack((basis(site), [*SITES[site], 38.1]))
        tr = gp_Trsf()
        tr.SetValues(*m.flatten())
        target = BRepBuilderAPI_Transform(
            plates[("9-REV-21-3005-P01" if under else "10-REV-21-3005-P02")], tr, False
        ).Shape()
        dz = -0.01 if under else 0.01
        toward = move(s, (0, 0, dz))
        away = move(s, (0, 0, -dz))
        out[k] = dict(
            gapMm=dist(s, target),
            overlapMm3=common(s, target),
            press001Mm3=common(toward, target),
            unseat001GapMm=dist(away, target),
        )
    return dict(
        supplierStepSha256=report["sourceModule"],
        supplierPlateBodyIds=list(plates),
        tabStations=16,
        solidFaces=32,
        nominalLandODmm=11,
        nominalBoreMm=5.3,
        landHeightMm=0.05,
        stacks=out,
        selection="Machine these raised rings into one saddle blank; face each pair to the received modules. No loose unowned shims. Positive areas are 0.01 mm inward press witnesses.",
    )


if __name__ == "__main__":
    cli = argparse.ArgumentParser()
    cli.add_argument("supplier", type=Path)
    (HERE / "saddle-bearing.json").write_bytes(
        (json.dumps(test(cli.parse_args().supplier), indent=2) + "\n").encode()
    )
