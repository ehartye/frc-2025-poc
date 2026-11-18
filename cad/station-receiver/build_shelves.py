"""Build and reopen complete receiver shelf solids from the project-owned XZ section."""
import argparse
import json
import math
from pathlib import Path

from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire, BRepBuilderAPI_MakeFace
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.Bnd import Bnd_Box
from OCP.GC import GC_MakeArcOfCircle
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Reader, STEPControl_Writer
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.gp import gp_Pnt, gp_Vec


def shelf(design, radius):
    length, width, thickness = (design[key] for key in ("length_mm", "width_mm", "thickness_mm"))
    pt = lambda x, z: gp_Pnt(x, -width/2, z)
    pts = [pt(0,-thickness), pt(length,-thickness), pt(length,0), pt(radius,0)]
    wire = BRepBuilderAPI_MakeWire()
    for start,end in zip(pts,pts[1:]):
        wire.Add(BRepBuilderAPI_MakeEdge(start,end).Edge())
    if radius:
        midpoint = pt(radius*(1-1/math.sqrt(2)), -radius*(1-1/math.sqrt(2)))
        arc = GC_MakeArcOfCircle(pts[-1], midpoint, pt(0,-radius)).Value()
        wire.Add(BRepBuilderAPI_MakeEdge(arc).Edge())
    wire.Add(BRepBuilderAPI_MakeEdge(pt(0,-radius),pts[0]).Edge())
    return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(wire.Wire()).Face(), gp_Vec(0,width,0)).Shape()


def reopen(path, design, radius):
    reader = STEPControl_Reader()
    if reader.ReadFile(str(path)) != IFSelect_RetDone or reader.TransferRoots() != 1:
        raise ValueError("STEP root could not be reopened")
    shape = reader.OneShape()
    solids = TopExp_Explorer(shape, TopAbs_SOLID)
    count = 0
    while solids.More():
        count += 1
        solids.Next()
    if count != 1 or not BRepCheck_Analyzer(shape).IsValid():
        raise ValueError("Expected one valid reopened solid")
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    l, w, h = (design[k] for k in ("length_mm","width_mm","thickness_mm"))
    expected = w * (l*h - radius**2*(1-math.pi/4))
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    bounds = (*box.CornerMin().Coord(), *box.CornerMax().Coord())
    if not math.isclose(props.Mass(),expected,abs_tol=1e-4,rel_tol=0) or any(
        abs(a-b)>1e-5 for a,b in zip(bounds,(0,-w/2,-h,l,w/2,0))):
        raise ValueError("Reopened volume or extent differs from the dimensioned profile")
    return {"solid_count":count, "bounds_mm":list(bounds), "volume_mm3":props.Mass()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    design = json.loads(Path(__file__).with_name("design.json").read_text())
    args.output.mkdir(exist_ok=True,parents=True)
    for radius in design["radius_choices_mm"]:
        path = args.output / f"shelf-r{radius}.step"
        writer = STEPControl_Writer()
        if writer.Transfer(shelf(design,radius), STEPControl_AsIs) != IFSelect_RetDone:
            raise ValueError("Could not transfer receiver shelf to STEP")
        if writer.Write(str(path)) != IFSelect_RetDone:
            raise ValueError("Could not write receiver STEP")
        lines = path.read_text(encoding="ascii").splitlines()
        path.write_text("\n".join(line.rstrip() for line in lines) + "\n", encoding="ascii", newline="\n")
        print(path.name, json.dumps(reopen(path,design,radius)))


if __name__ == "__main__":
    main()
