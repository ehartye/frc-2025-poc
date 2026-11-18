"""Measure line/face overlap on reopened, robot-posed shelf geometry in station coordinates."""
import hashlib
import json
import math
import sys
from pathlib import Path

from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.GeomAbs import GeomAbs_Plane
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from scripts import receiver_dock


def top_face(path):
    reader = STEPControl_Reader()
    if reader.ReadFile(str(path)) != IFSelect_RetDone or reader.TransferRoots() != 1:
        raise ValueError("Could not reopen a single STEP root")
    explorer = TopExp_Explorer(reader.OneShape(),TopAbs_FACE)
    matches = []
    while explorer.More():
        face = TopoDS.Face(explorer.Current())
        surf = BRepAdaptor_Surface(face)
        if surf.GetType() == GeomAbs_Plane:
            plane = surf.Plane()
            direction = plane.Axis().Direction()
            if abs(plane.Location().Z()) < 1e-6 and abs(direction.Z()) > 1-1e-9:
                matches.append(face)
        explorer.Next()
    if len(matches) != 1:
        raise ValueError("Expected exactly one elevated flat face")
    return matches[0]


def shift(shape, dx, dy, dz):
    transform = gp_Trsf()
    transform.SetTranslation(gp_Vec(dx,dy,dz))
    return BRepBuilderAPI_Transform(shape,transform,False).Shape()


def native_length(face, trial):
    # Source shelf leading edge at robot X=-385 mm, centered at 220 mm extension.
    placed = shift(face,-385-(trial["extension_mm"]-220),0,0)
    angle = math.radians(trial["yaw_deg"])
    turn = gp_Trsf()
    turn.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,0,1)),angle)
    placed = BRepBuilderAPI_Transform(placed,turn,False).Shape()
    placed = shift(placed,trial["after_retreat_chassis_x_mm"],trial["lateral_mm"],880)
    x = trial["bottom_x_mm"]
    a, b = (gp_Pnt(x,y,880) for y in (-150.8125,150.8125))
    common = BRepAlgoAPI_Common(BRepBuilderAPI_MakeEdge(a,b).Edge(),placed)
    if not common.IsDone():
        raise ValueError("Native finite-edge / top-face intersection failed")
    length = GProp_GProps()
    BRepGProp.LinearProperties_s(common.Shape(),length)
    return length.Mass()


def main():
    directory = Path(__file__).resolve().parent
    design = json.loads((directory/"design.json").read_text())
    inputs = [directory/"design.json", Path(__file__), Path(receiver_dock.__file__),
              Path(receiver_dock.receiver.__file__), Path(receiver_dock.receiver.station.__file__),
              ROOT/"scripts/station_edge_contact.py"]
    rows = []
    for radius in (0,6):
        path = directory/f"shelf-r{radius}.step"
        inputs.append(path)
        face = top_face(path)
        for yaw,lateral,speed,extension in ((0,0,0.5,220),(2,0,0.5,220),(2,0,0.5,240),
                                           (5,0,0.5,240),(-5,45,0.5,240),(10,0,0.5,240)):
            trial = receiver_dock.dock(design,speed,yaw,lateral,extension,radius)
            value = native_length(face,trial)
            if abs(value-trial["flat_contact_length_mm"]) > 1e-5:
                raise ValueError(f"Native overlap {value} differs from analytical interval {trial}")
            rows.append({"radius_mm":radius,"yaw_deg":yaw,"lateral_mm":lateral,"speed_mps":speed,
                         "extension_mm":extension,"native_length_mm":round(value,6),
                         "analytical_length_mm":round(trial["flat_contact_length_mm"],6)})
    pins = {p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    print(json.dumps({"schema":"receiver-docking-line-face/1","input_sha256":pins,"cases":rows},indent=2))


if __name__ == "__main__":
    main()
