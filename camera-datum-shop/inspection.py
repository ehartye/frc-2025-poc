"""Read the released shelf STEP and match its machined faces to the shared operation sheet."""
import csv
import json

from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cone, GeomAbs_Cylinder
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

from camera_parts import HERE, SHELF, cad, read, sha


def inspect():
    step = HERE / "camera-shelf.step"
    operations = HERE / "shelf-operations.csv"
    inventory = read(HERE / "native.json")["parts"]
    shelf = cad.read_named(step, [{"id": name} for name in inventory])[SHELF]
    faces = TopExp_Explorer(shelf, TopAbs_FACE)
    machined = []
    nose_radii = []
    while faces.More():
        face = TopoDS.Face(faces.Current())
        faces.Next()
        surface = BRepAdaptor_Surface(face)
        kind = surface.GetType()
        if kind not in (GeomAbs_Cone, GeomAbs_Cylinder):
            continue
        cone = kind == GeomAbs_Cone
        geom = surface.Cone() if cone else surface.Cylinder()
        axis = geom.Axis()
        x, y, _ = axis.Location().Coord()
        _, dy, dz = axis.Direction().Coord()
        if abs(dy) > .999 and not cone:
            nose_radii.append(round(geom.Radius(), 3))
        if abs(dz) < .999:
            continue
        bounds = cad.bounds(face)
        diameter = bounds[3] - bounds[0] if cone else geom.Radius() * 2
        machined.append((round(x + 100, 3), round(y + 210, 3), round(diameter, 3),
                         "cone" if cone else "cylinder", round(bounds[2], 3), round(bounds[5], 3)))
    matched = []
    with operations.open(encoding="utf8", newline="") as stream:
        for row in csv.DictReader(stream):
            name = row["operation"]
            shape = "cone" if name == "Keeper countersink" else "cylinder"
            key = (float(row["U_mm"]), float(row["V_mm"]), float(row["diameter_mm"]), shape)
            candidates = [face for face in machined if face[:4] == key]
            if len(candidates) != 1:
                raise ValueError(f"{name} at {key}: found {candidates}")
            _, _, _, _, z0, z1 = candidates[0]
            expected = {"Shelf to receiver": (485., 497.), "Shelf counterbore": (497., 500.),
                        "Saddle keeper screw": (485., 498.7), "Keeper countersink": (498.7, 500.)}
            if (z0, z1) != expected.get(name, (485., 500.)):
                raise ValueError(f"{name}: observed depth faces {z0}, {z1}")
            matched.append({"name": name, "U_mm": key[0], "V_mm": key[1], "diameter_mm": key[2],
                            "profile": shape, "bottom_top_Z_mm": [z0, z1]})
    if len(matched) != len(machined) or nose_radii != [6.0]:
        raise ValueError(f"Undocumented features: {len(machined)} vertical faces and nose radii {nose_radii}")
    return {"step_sha256": sha(step), "operations_sha256": sha(operations),
            "forward_radius_mm": nose_radii[0], "finished_native_faces": matched}


if __name__ == "__main__":
    (HERE / "shelf-proof.json").write_text(json.dumps(inspect(), indent=2) + "\n", encoding="utf8", newline="\n")
