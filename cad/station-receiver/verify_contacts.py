"""Check six analytical stations against the reopened STEP shelf and a centered finite cylinder."""
import hashlib
import json
import sys
from pathlib import Path

from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepGProp import BRepGProp
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts import station_receiver_impact as model

DESIGN_DIR = model.DESIGN.parent
PIPE_LENGTH_MM = 301.625


def evaluate(shelf, x_mm, z_mm):
    # Finite outer surface is sufficient for the first outside contact. The
    # shell bore is deliberately omitted; deep penetration is outside the test.
    axis = gp_Ax2(gp_Pnt(x_mm, -PIPE_LENGTH_MM/2, z_mm), gp_Dir(0,1,0))
    envelope = BRepPrimAPI_MakeCylinder(axis, model.station.RADIUS_M*1000, PIPE_LENGTH_MM).Shape()
    distance = BRepExtrema_DistShapeShape(shelf, envelope)
    common = BRepAlgoAPI_Common(shelf,envelope)
    if not distance.IsDone() or not common.IsDone():
        raise ValueError("Native distance or common-volume calculation failed")
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(common.Shape(),props)
    return {"gap_mm": round(distance.Value(), 8), "overlap_mm3": round(props.Mass(), 6)}


def sample(radius, speed, offset, extension, shelf, design):
    result = model.first_impact(speed,offset,extension,radius,design)
    state = model.station.departure(speed)
    front = model.station.EXTENDED_FRONT + (offset + design["baseline_extension_mm"] - extension) / 1000
    values = []
    for dt in (-0.0005, 0, 0.0005):
        x,z = model.station.flight_center(state, result["time_s"] + dt)
        values.append(evaluate(shelf, (x-front)*1000, (z-model.station.SHELF_TOP)*1000))
    before, at, beyond = values
    if (before["gap_mm"] < 0.01 or before["overlap_mm3"] > 1e-5 or at["gap_mm"] > 1e-5
            or abs(at["overlap_mm3"]) > 1e-4 or beyond["overlap_mm3"] < 0.01):
        raise ValueError(f"Unexpected approach/tangency/penetration for radius {radius} at {offset} mm: {values}")
    return {"radius_mm": radius, "extension_mm": extension, "speed_mps": speed, "offset_mm": offset,
            "surface": result["surface"], "before": before, "at_first_contact": at,
            "unimpeded_negative_control": beyond}


def main():
    design = json.loads(model.DESIGN.read_text())
    inputs = [model.DESIGN, Path(__file__), DESIGN_DIR/"build_shelves.py", Path(model.__file__),
              Path(model.station.__file__), ROOT/"scripts/station_edge_contact.py"]
    cases = []
    for radius in design["radius_choices_mm"]:
        path = DESIGN_DIR / f"shelf-r{radius}.step"
        reader = STEPControl_Reader()
        if reader.ReadFile(str(path)) != IFSelect_RetDone or reader.TransferRoots() != 1:
            raise ValueError(f"Could not open single STEP root: {path.name}")
        inputs.append(path)
        for speed,offset,extension in ((0.5,0,220),(0.5,40,220),(0.5,40,240)):
            cases.append(sample(radius,speed,offset,extension,reader.OneShape(),design))
    pins = {p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    print(json.dumps({"schema":"receiver-step-first-contact/1", "input_sha256":pins,
                      "offset_from_impact_ms":[-0.5,0,0.5], "cases":cases},indent=2,allow_nan=False))


if __name__ == "__main__":
    main()
