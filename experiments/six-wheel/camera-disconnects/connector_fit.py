"""Register supplier-native connector geometry on the fixed-low camera robot. Inputs stay external."""
import argparse
from functools import lru_cache
import hashlib
import json
import sys
from pathlib import Path

from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.gp import gp_Trsf

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "camera-datum-shop"))
from camera_parts import cad

EXPECTED = {
    "camera.stp": "26a56d59b6bc4e8baadf3256704ce74a5fe9a2e9d0b4f51f8ba2b76c2629461a",
    "board.step": "bd6ce228d351a1f70077a27ab72ec9da126986f38abbf2528fc5e03f9d1b6605",
    "adapter.stp": "3f5b00ed25398797077f7483fe57a9901dc24e8e02b108ee6db82b9620cd6432",
    "plug.step": "01dc336f4e1cb5d0035991379fad2ac19652863448a3179f7404272ce6a44453",
}


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def load(path):
    if digest(path) != EXPECTED[path.name]:
        raise ValueError(f"Unexpected supplier STEP {path.name}")
    reader = STEPControl_Reader()
    if reader.ReadFile(str(path)) != IFSelect_RetDone or reader.TransferRoots() < 1:
        raise ValueError(path)
    return reader.OneShape()


def solids(shape):
    it = TopExp_Explorer(shape, TopAbs_SOLID)
    while it.More():
        yield it.Current()
        it.Next()


def transform(shape, matrix, position):
    t = gp_Trsf()
    t.SetValues(*(v for row, shift in zip(matrix, position, strict=True) for v in [*row, shift]))
    return BRepBuilderAPI_Transform(shape, t, False).Shape()


CAM = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]
ADAPTER = [[0, 0, -1], [-1, 0, 0], [0, 1, 0]]
PLUG = [[-1, 0, 0], [0, -1, 0], [0, 0, 1]]


def distance(a, b):
    solver = BRepExtrema_DistShapeShape(a, b)
    if not solver.IsDone():
        raise ValueError("Native distance did not converge")
    return solver.Value()


@lru_cache(maxsize=1000)
def bounds(body):
    return cad.bounds(body)


def conflicts(left, right):
    result = {}
    for key, body in left.items():
        for other, obstacle in right.items():
            if not cad.occupied(bounds(body), bounds(obstacle)):
                continue
            common = cad.intersection(body, obstacle)
            if common > .002:
                result[f"{key}/{other}"] = round(common, 5)
    return result


def classifiers(shapes):
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.TopAbs import TopAbs_IN
    from OCP.gp import gp_Pnt
    classifiers = [BRepClass3d_SolidClassifier(o) for o in shapes]
    def inside(q):
        for tool in classifiers:
            tool.Perform(gp_Pnt(*q), 1e-7)
            if tool.State() == TopAbs_IN:
                return True
        return False
    return inside


def check(folder):
    from camera_parts import box, cut
    cam = list(solids(load(folder / "camera.stp")))
    board = list(solids(load(folder / "board.step")))
    adapter = list(solids(load(folder / "adapter.stp")))
    cable = list(solids(load(folder / "plug.step")))
    local = ROOT / "camera-datum-shop"
    labels = json.loads((local / "native.json").read_bytes())["parts"]
    mount = cad.read_named(local / "camera-shelf.step", [{"id": k} for k in labels])
    # M4 axes (3.5/78.5,15.5) and rear contact surface Z8.5 register to the shop bridge.
    camera = {f"cam-{i}": transform(o, CAM, (32.5, 41, 851)) for i, o in enumerate(cam)}
    face_x = bounds(transform(board[240], CAM, (32.5, 41, 851)))[0]
    nose_z = min(bounds(p)[2] for p in adapter)
    cam_inside = classifiers([cam[0], cam[7]])
    factory_inside = classifiers([adapter[3], adapter[4]])
    # The PCB and electronics stay untouched. Screen removal of material from two housing pieces.
    plane = -2.65
    tool = box(-14, -7, -5, 28, 14, 5+plane)
    relieved = {i: cut(adapter[i], tool) for i in (3, 4)}
    case_edge = min(bounds(shape)[0] for shape in camera.values())
    uptake = {}
    for depth in (5., 5.5, 6., 6.2, 6.4):
        origin = face_x + depth + nose_z
        hits = []
        for x in (29.55, 29.75, 29.95, 30.15, 30.35, 30.55, 30.75, 30.95):
            for y in (-27, -25, -23, -21, -11, -9, -7, -5):
                for z in (845., 846.5, 848., 849.5, 851.):
                    if cam_inside((41-y, x-32.5, z-851)) and factory_inside((-15.55-y, z-847.78, origin-x)):
                        hits.append([x,y,z])
        # The separate half-spaces prove clearance over all case material at all lesser insertion depths.
        gap = case_edge - max(origin-bounds(shape)[2] for shape in relieved.values())
        uptake[depth] = {"overlap_sample_count": len(hits), "first_hard_material_point_mm": hits[:1],
                         "relieved_case_plane_gap_mm": gap}
    datum = face_x + 6.2 + nose_z
    targets = {}
    for key, adapter_x in (("data-J1", 8.0005), ("power-J3", -7.9995)):
        # Hirose CX80B1-24P mouth 6.5 above PCB. Plug withdrawal is toward -X.
        face = datum-6.5
        y = -15.55-adapter_x
        shoulder = face + 6.2 - 6.65
        boot = transform(cable[0], PLUG, (shoulder, y, 847.78))
        b = bounds(boot)
        targets[key] = {"mouth_mm": [face, y, 847.78], "native_comparison_boot": "Tensility 10-03675",
                        "selected_boot": "Newnex U3HLA01C12-020" if key=="data-J1" else "Tensility 10-03675", "boot_bounds_mm": b,
                        "cable_start_mm": [b[0],y,847.78], "outward_direction": [-1,0,0]}
    from support import design
    bracket = design()[0]
    mounted = {f"relieved-case-{i}":transform(sh,ADAPTER,(datum,-15.55,847.78)) for i,sh in relieved.items()}
    nominal_studs = {f"unused-data-stud-{side}":cad.brep(["cyl",[datum-6.5,-23.5505,847.78+7.5*side,1,0,0,1.25,5.2]]) for side in (-1,1)}
    physical_case = {"support_overlaps_mm3":conflicts(mounted, bracket),"stud_overlaps_mm3":conflicts(mounted,nominal_studs)}
    selected = {"mated_external_hard_case":physical_case, "removed_casing_mm3": {k: cad.volume(adapter[k])[0]-cad.volume(v)[0] for k,v in relieved.items()},
                "new_face_local_Z_mm": plane, "other_parts_except_output_connector_min_Z_mm":
                min(bounds(adapter[i])[2] for i in [1,2,5,6])}
    # Supplier connector model is direct evidence for its shell, not for optical pupils or contact force.
    return {"sha256": EXPECTED, "camera_board_mouth_x_mm": face_x, "source_plug_tip_z_mm": nose_z,
            "camera_case_min_x_mm": case_edge, "camera_port_center_mm": [face_x,-15.55,847.78],
            "window_axes_at_registered_bolts_mm": [[48.54,-37.5],[48.54,37.5]],
            "adapter_depths": uptake, "adapter_case_relief": selected, "disconnects": targets,
            "retained_603_kit": digest(local / "camera-shelf.step"),
            "pupil_location_physically_measured": False, "electrical_mating_measured": False}

if __name__ == "__main__":
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument("folder", type=Path)
    result = check(args.parse_args().folder)
    (HERE / "interfaces.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf8", newline="\n")
