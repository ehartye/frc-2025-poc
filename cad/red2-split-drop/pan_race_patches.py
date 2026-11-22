"""Measure the finite area of each installed thrust washer seat on the issued cold assembly."""
import json
import sys
from pathlib import Path

from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GeomAbs import GeomAbs_Plane
from OCP.GProp import GProp_GProps
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

from output_mount import ROOT, digest, read
from pan_thrust_layout import translate


def transverse_faces(shape):
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = TopoDS.Face(explorer.Current())
        explorer.Next()
        surface = BRepAdaptor_Surface(face)
        if surface.GetType() == GeomAbs_Plane:
            plane = surface.Plane()
            if abs(plane.Axis().Direction().Y()) > 1 - 1e-8:
                yield plane.Location().Y(), face


def patches(left, right):
    rows = []
    for y, a in transverse_faces(left):
        for y2, b in transverse_faces(right):
            if abs(y-y2) > 1e-5:
                continue
            op = BRepAlgoAPI_Common(a, b)
            if not op.IsDone() or op.Shape().IsNull():
                raise ValueError("Unable to intersect the registered coplanar faces")
            props = GProp_GProps()
            BRepGProp.SurfaceProperties_s(op.Shape(), props)
            if props.Mass() > 1e-5:
                rows.append(dict(yMm=y, areaMm2=props.Mass(), centroidMm=props.CentreOfMass().Coord()))
    return rows


def measure(plugin):
    sys.path.insert(0, str(plugin/"scripts"))
    from mechanical_motion import common, distance
    from showcase_contacts import load_solids

    folder = ROOT/"artifacts/red2-integrated/pan-thrust/fixed-races"
    report = read(folder/"pan-r3-fixed-races-01.json")
    register = read(ROOT/"cad/fusion-authority.json")
    if report["authority"]["archiveSha256"] != digest(ROOT/register["source"]["archive"]["file"]):
        raise ValueError("The current robot authority changed")
    src, step = folder/"checked-297.f3d", folder/"checked-297.step"
    if report["archiveSha256"] != digest(src) or report["stepSha256"] != digest(step):
        raise ValueError("Need the derivative of the cold guarded successor")
    parts, extras = load_solids(step)
    if extras or len(parts) != 297:
        raise ValueError("Unexpected issued census")
    plate = parts["Piloted bilateral bearing support"]
    rows = {}
    for side, inward in [("inboard", -1.), ("outboard", 1.)]:
        name = f"6655K17 {side} stationary housing race"
        part = parts[name]
        seats = {}
        for signed in [0., -.05, .25]:
            moved = translate(part, [0., inward*signed, 0.])
            seats[str(signed)] = dict(towardSeatMm=signed, gapMm=distance(moved,plate),
                                     commonMm3=common(moved,plate), patches=patches(moved,plate))
        contact, floating, driven = seats["0.0"], seats["-0.05"], seats["0.25"]
        area = sum(p["areaMm2"] for p in contact["patches"])
        if area < 1. or contact["gapMm"] > .001 or contact["commonMm3"] > .001:
            raise ValueError(f"Missing seated face {side}: {contact}")
        if floating["patches"] or abs(floating["gapMm"]-.05) > .001 or driven["commonMm3"] < 1.:
            raise ValueError(f"Matched adverse controls did not react on the {side} side")
        rows[side] = dict(name=name, areaMm2=area, states=seats)
    result = dict(authority=report["authority"],archiveSha256=digest(src),stepSha256=digest(step),
                  nativeReceiptSha256=digest(folder/"pan-r3-fixed-races-01.json"),seats=rows,
                  scope="Static finite contact area at the unloaded, nominal fixed race seats. Applied contact pressure, deformation and bearing life remain separate.")
    (folder/"seated-face-patches.json").write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps(rows),flush=True)


if __name__ == "__main__":
    measure(Path(sys.argv[1]))
