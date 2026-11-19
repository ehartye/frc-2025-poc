"""Register the dynamics' two contact patches against interior/exterior points of the issued STEP."""

import argparse
import importlib.util
import json
from pathlib import Path
import sys
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.TopAbs import TopAbs_IN
from OCP.gp import gp_Pnt
from beam import ROOT, digest, guide


def run(kit):
    file = ROOT / "carriage-support/fit.py"
    sys.path.insert(0, str(file.parent))
    spec = importlib.util.spec_from_file_location("carriage_native_guide", file)
    assembly = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(assembly)
    issued = assembly.selected(
        kit, "guide-mounts.step", "fit.json", "stepSha256", "volumesMm3"
    )
    rows = {}
    for sign, prefix in [(-1, "guide-R-"), (1, "guide-L-")]:
        liner = issued[prefix + "liner"]
        backing = issued[prefix + "backing"]

        def solid(shape, x, y, z):
            return (
                BRepClass3d_SolidClassifier(shape, gp_Pnt(x, y, z), 1e-7).State()
                == TopAbs_IN
            )

        inside = outside = 0
        for z in [625.1, 638, 687, 729, 769.9, 770.1, 787, 795.4, 812.9]:
            # Independent formula from the pipe/contact frame, in millimetres.
            x = 1000 * guide.x_center(z / 1000) + 20
            y = 1000 * guide.feed.HALF - 0.3 + 0.038 * max(z - 770, 0)
            for dx in [-19.0, 0.0, 19.0]:
                if not solid(liner, x + dx, sign * (y + 0.025), z):
                    raise ValueError("Missed fitted liner")
                if solid(liner, x + dx, sign * (y - 0.025), z):
                    raise ValueError("Model starts outside issued face")
                if not solid(liner, x + dx, sign * (y + 2.975), z):
                    raise ValueError("Wrong liner depth")
                if not solid(backing, x + dx, sign * (y + 3.4), z):
                    raise ValueError("Wrong backing")
                inside += 3
                outside += 1
            for dx in [-20.1, 20.1]:
                if solid(liner, x + dx, sign * (y + 1.5), z):
                    raise ValueError("Wrong face width")
                outside += 1
        for z in [624.95, 813.05, 830.0, 849.0]:
            if solid(
                liner,
                1000 * guide.x_center(z / 1000) + 20,
                sign * (1000 * guide.feed.HALF + 1.2 + 0.038 * max(z - 770, 0)),
                z,
            ):
                raise ValueError("Face continued beyond machined tip")
            outside += 1
        rows[prefix] = {"inside": inside, "outside": outside}
    return {
        "stepSha256": digest(kit / "guide-mounts.step"),
        "alternateThroatDatum": digest(kit / "datum.json"),
        "parts": len(issued),
        "sides": rows,
        "faceHalfWidthMm": 20,
        "faceZmm": [625, 813],
        "inwardOutwardProbeMm": 0.025,
        "xCenterMmAtZ625": 165,
        "xSlope": 0.46,
        "yCenterPlaneMm": 1000 * guide.feed.HALF - 0.3,
        "ySlopeAbove770": 0.038,
        "linerThicknessMm": 3,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("out", type=Path)
    p.add_argument("--kit", type=Path, default=ROOT / "guide-mounts")
    a = p.parse_args()
    a.out.write_text(
        json.dumps(run(a.kit), indent=2) + "\n", encoding="utf8", newline="\n"
    )
