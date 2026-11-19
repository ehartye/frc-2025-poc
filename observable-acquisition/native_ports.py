"""Read the actual optical bores and return-channel envelopes from the issued red receive STEP."""
import hashlib
import json
import sys
from pathlib import Path
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT/"design/station-l1"))
from build_assembly import bounds, read_named

POSE = ROOT/"design/competition-package/shop-duplex/red"


def inspect():
    step = POSE / "receive.step"
    digest = hashlib.sha256(step.read_bytes()).hexdigest()
    expected = json.loads((ROOT/"axial-spin/study.json").read_bytes())["robot_pose_sha256"]["receive"]
    if digest != expected: raise ValueError("different 545 receive assembly")
    names = json.loads((POSE/"part-frames.json").read_bytes())
    parts = read_named(step, [{"id": k} for k in names])
    result = {}
    for name, x in (("seat", 280), ("entry", 340)):
        channel = {}
        for body in ("receiver-R6-ported", "optical-mount-" + name):
            faces = TopExp_Explorer(parts[body], TopAbs_FACE)
            matches = []
            while faces.More():
                face = TopoDS.Face(faces.Current())
                geometry = BRepAdaptor_Surface(face)
                if geometry.GetType() == GeomAbs_Cylinder:
                    cylinder = geometry.Cylinder()
                    pos, axis = cylinder.Location(), cylinder.Axis().Direction()
                    if (abs(pos.X()-x) < 1e-6 and abs(pos.Y()) < 1e-6 and
                            abs(abs(axis.Z())-1) < 1e-7 and abs(cylinder.Radius()-6) < 1e-6):
                        matches.append({"axis_xy_mm": [pos.X(), pos.Y()], "radius_mm": cylinder.Radius(),
                                        "face_bounds_mm": bounds(face)})
                faces.Next()
            if len(matches) != 1: raise ValueError(f"{body} port {x} unresolved: {matches}")
            channel[body] = matches[0]
        channel["qtrx_body"] = "qtrx-" + name
        channel["optical_envelope_mm"] = bounds(parts[channel["qtrx_body"]])
        result[name] = channel
    return {"receive_sha256": digest, "installed_bodies": len(parts), "channels": result}


if __name__ == "__main__":
    (HERE/"ports.json").write_text(json.dumps(inspect(), indent=2)+"\n", encoding="utf-8", newline="\n")
