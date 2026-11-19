"""Extract source-bound conservative movement boxes for the 603-body placement."""
import argparse
import json
import re
from pathlib import Path
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from connector_fit import ROOT, HERE, cad, digest


def read(file):
    return json.loads(file.read_bytes())


def extract(step):
    source = read(ROOT / "camera-datum-shop/native-place.json")
    if digest(step) != source["step_sha256"]:
        raise ValueError("Wrong 603-body STEP")
    ids = set(re.findall(r"\bPRODUCT\('([^']+)'", step.read_text(encoding="ascii"))) - {"SOLID", "COMPOUND"}
    if len(ids) != 603:
        raise ValueError("Wrong placement inventory")
    shapes = cad.read_named(step, [{"id": name} for name in ids])
    frames = read(ROOT / "design/competition-package/shop-duplex/red/part-frames.json")
    rotations = read(ROOT / "design/competition-package/shop-duplex/red/rotation-bounds.json")
    boxes = {}
    for name, shape in shapes.items():
        frame = frames.get(name, "base")
        if name.startswith("roof-") or name == "receiver-R6-drilled-for-camera":
            frame = "slide"
        fraction = {"slide": 1., "middle": .5}.get(frame, 0.)
        bb = Bnd_Box()
        BRepBndLib.Add_s(shape, bb, False)
        b = [*bb.CornerMin().Coord(), *bb.CornerMax().Coord()]
        if name in rotations:
            r = rotations[name]["bounds_mm"]
            b = [r[0]+370*fraction,r[1],r[2]+560,r[3]+370*fraction,r[4],r[5]+560]
        if name.startswith(("grip-L-","grip-R-")):
            b[1] -= 1.5
            b[4] += 1.5
        # Express obstacles relative to the slide, covering all intermediate extensions.
        b[0] -= 370
        b[3] -= 370*fraction
        boxes[name] = b
    return {"source_sha256": source["step_sha256"], "native_body_count": len(shapes), "relative_sweep_bounds_mm": boxes}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("step", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.write_text(json.dumps(extract(args.step)), encoding="utf8")
