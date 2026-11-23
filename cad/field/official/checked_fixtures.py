"""Write the official field parts a robot layout is checked against, placed at the robot's two docks.

    python -I -B cad/field/official/checked_fixtures.py OUT_FOLDER OUT.json [--standoff MM] [--dock-gap MM]

The layout check takes one named solid per field fixture. This picks the official parts the mission touches or passes
(checked-fixtures.json), takes each solid as FIRST drew it through the dock frame of official-field.json into the robot
datum, and writes one STEP per solid into OUT_FOLDER (the check reads a fixture's file once for every case, so small
files keep it fast). Nothing is redrawn or simplified. Official parts that overlap each other in the official model are
united into one solid (the `unite` lists of the selection), because a layout check cannot hold two interpenetrating
fixtures; the union changes no outer surface.

OUT.json records the selection, the two dock placements, each part's bounds, the pairs of official parts that touch each
other (the official model's own joints) and the section the planar free-piece trials use: the chute's top surface and
upper guard, the trough lip, floor and riser, each measured on these solids.

Docks. The delivery dock puts the trough lip face --standoff mm ahead of the front bumper face (50 by default, the
dock frame of official-field.json). The receive dock puts the station's lower barrier, the part the bumper meets,
--dock-gap mm behind the rear bumper face (10 by default); the dock frame of official-field.json is taken from the
upper-panel datum 9.525 mm farther back, so the parts are moved by the difference.
"""
import json
import sys
from itertools import combinations
from pathlib import Path

from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Fuse
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.gp import gp_Dir, gp_Lin, gp_Pnt, gp_Trsf
from OCP.IFSelect import IFSelect_RetDone
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.Interface import Interface_Static
from OCP.OCP.collections import Sequence_TDF_Label
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from tessellate import bounds, label_name, read  # noqa: E402

ST = XCAFDoc_ShapeTool
# The robot the docks are measured from: bumper outer faces fore and aft, millimetres in the robot datum.
BUMPER_FRONT_X, BUMPER_REAR_X = 445.5, -445.5
TOUCH_MM = 2.0          # official parts closer than the layout's check reserve are joints of the official model


def arg(flag, default):
    return float(sys.argv[sys.argv.index(flag) + 1]) if flag in sys.argv else default


def placed(shape, matrix, shift):
    """A copy of an official solid in the robot datum: the dock frame, then a translation along X."""
    t = gp_Trsf()
    rows = [list(r) for r in matrix]
    rows[0][3] += shift
    t.SetValues(*[v for row in rows for v in row])
    return BRepBuilderAPI_Transform(shape, t, True).Shape()


def leaves(step):
    doc, root = read(step)
    out = []

    def walk(label, loc, path):
        kids = Sequence_TDF_Label()
        ST.GetComponents_s(label, kids)
        if kids.Length():
            for i in range(1, kids.Length() + 1):
                child, ref = kids.Value(i), TDF_Label()
                if ST.GetReferredShape_s(child, ref):
                    walk(ref, loc.Multiplied(ST.GetLocation_s(child)), path + [label_name(ref)])
            return
        out.append((path, ST.GetShape_s(label).Moved(loc)))
    walk(root, TopLoc_Location(), [label_name(root)])
    return out


def solids_of(shape):
    found, explorer = [], TopExp_Explorer(shape, TopAbs_SOLID)
    while explorer.More():
        found.append(explorer.Current())
        explorer.Next()
    return found


def volume(shape):
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    return props.Mass()


def top_z(shape, x, y, down=True):
    """Height of the first surface a vertical ray meets at (x, y): from above, or from below."""
    hit = IntCurvesFace_ShapeIntersector()
    hit.Load(shape, 1e-6)
    hit.Perform(gp_Lin(gp_Pnt(x, y, 0.0), gp_Dir(0, 0, 1)), -1e5, 1e5)
    zs = [hit.Pnt(i).Z() for i in range(1, hit.NbPnt() + 1)]
    return (max(zs) if down else min(zs)) if zs else None


def line(shape, xs, ys, down=True):
    """Least-squares Z = z0 + slope * X of a planar face sampled by vertical rays, and the largest residual."""
    pts = [(x, z) for x in xs for y in ys for z in [top_z(shape, x, y, down)] if z is not None]
    n = len(pts)
    mx, mz = sum(p[0] for p in pts) / n, sum(p[1] for p in pts) / n
    slope = sum((p[0] - mx) * (p[1] - mz) for p in pts) / sum((p[0] - mx) ** 2 for p in pts)
    z0 = mz - slope * mx
    return z0, slope, max(abs(p[1] - (z0 + slope * p[0])) for p in pts), n


def main():
    out_dir, out_json = Path(sys.argv[1]), Path(sys.argv[2])
    standoff, dock_gap = arg("--standoff", 50.0), arg("--dock-gap", 10.0)
    record = json.loads((HERE / "official-field.json").read_text(encoding="utf-8"))
    chosen = json.loads((HERE / "checked-fixtures.json").read_text(encoding="utf-8"))
    solids, rows = {}, []
    for fx in chosen["fixtures"]:
        source = next(f for f in record["fixtures"] if f["id"] == fx["official"])
        frame = next(d for d in source["dockFrames"] if d["id"] == fx["dockFrame"])
        parts = leaves(HERE / source["step"])
        in_frame = [(path, placed(shape, frame["matrix"], 0.0)) for path, shape in parts if any(path[-1] == p["part"] for p in fx["parts"])]
        matched = {}
        for spec in fx["parts"]:
            hits = [(path, s) for path, s in in_frame if path[-1] == spec["part"]
                    and all(abs(a - b) <= 2.0 for a, b in zip(bounds(s), spec["boundsDockMm"]))]
            assert len(hits) == 1, (spec["name"], len(hits))
            matched[spec["name"]] = hits[0]
        # The dock frame as recorded, then the shift that puts the named face at the requested distance from the bumper.
        face = fx["dockFace"]
        at = bounds(matched[face["name"]][1])[face["bound"]]
        want = BUMPER_FRONT_X + standoff if fx["dockFrame"] == "delivery-dock" else BUMPER_REAR_X - dock_gap
        shift = round(want - at, 3)
        for spec in fx["parts"]:
            path, shape = matched[spec["name"]]
            shape = placed(shape, [[1, 0, 0, shift], [0, 1, 0, 0], [0, 0, 1, 0]], 0.0)
            found = solids_of(shape)
            assert len(found) == 1, (spec["name"], len(found))
            shape = found[0]          # the solid itself, so the STEP names it and not a wrapper around it
            assert spec["name"] not in solids, spec["name"]
            solids[spec["name"]] = shape
            rows.append(dict(name=spec["name"], fixture=fx["id"], official=" / ".join(path[1:]), role=spec["role"],
                             boundsMm=[round(v, 2) for v in bounds(shape)]))
        fx["shiftXMm"], fx["dockFaceXMm"] = shift, want
    # The section the planar trials use, measured on these solids at the pipe's span (Y within 150 mm of the dock centre).
    ys = (-120.0, 0.0, 120.0)
    by_role = {r["role"]: r for r in rows}
    chute, guard = solids[by_role["chute"]["name"]], solids[by_role["guard"]["name"]]
    cb, gb = by_role["chute"]["boundsMm"], by_role["guard"]["boundsMm"]
    # The chute plate has 32 mm slots every 203 mm, one on the dock centre: the pipe bridges it, so the surface is read beside it.
    cys = (-120.0, -60.0, 60.0, 120.0)
    c0, cs, cres, cn = line(chute, [cb[3] - d for d in (30.0, 60.0, 150.0, 240.0, 320.0)], cys)
    # The flat ramp ends where the plate turns at its top: the farthest ray still on the fitted plane.
    top_x = min(x for x in [cb[3] - 0.5 * i for i in range(60, int(2 * (cb[3] - cb[0])))]
                if (z := top_z(chute, x, 60.0)) is not None and abs(z - (c0 + cs * x)) < 0.2)
    g0, gs, gres, gn = line(guard, [gb[0] + f * (gb[3] - gb[0]) for f in (0.25, 0.4, 0.55, 0.7)], ys, down=False)
    floor, lip, riser = solids[by_role["floor"]["name"]], by_role["lip"]["boundsMm"], by_role["riser"]["boundsMm"]
    fb = by_role["floor"]["boundsMm"]
    f0, fs, fres, fn = line(floor, [fb[0] + f * (fb[3] - fb[0]) for f in (0.1, 0.3, 0.5, 0.7, 0.9)], ys)
    import math
    section = dict(
        basis="Vertical rays on the placed official solids at Y -120, 0 and 120 mm (chute: -120, -60, 60 and 120, beside its centre slot); each line is a least-squares fit and residualMm its largest miss",
        station=dict(barrierFaceXMm=by_role["barrier"]["boundsMm"][3], dockGapMm=dock_gap,
                     chuteExitEdgeMm=[cb[3], round(c0 + cs * cb[3], 3)], chuteSlopeDeg=round(math.degrees(math.atan(-cs)), 3),
                     chuteTopEndMm=[round(top_x, 2), round(c0 + cs * top_x, 3)], chuteRampMm=round(math.hypot(cb[3] - top_x, cs * (cb[3] - top_x)), 2),
                     chuteSlotOnCentreMm="32 mm slot at Y -15 to 17, 7.75 mm deep; slots repeat every 203 mm", chuteResidualMm=round(cres, 3), chuteSamples=cn,
                     guardUndersideMm=[[round(x, 3), round(g0 + gs * x, 3)] for x in (gb[0], gb[3])], guardResidualMm=round(gres, 3), guardSamples=gn,
                     openingMm=[by_role["lower-panel"]["boundsMm"][5], by_role["upper-panel"]["boundsMm"][2]]),
        trough=dict(lipFaceXMm=lip[0], standOffMm=standoff, lipTopMm=lip[5], lipDepthMm=round(lip[3] - lip[0], 3),
                    floorStartXMm=fb[0], floorEndXMm=fb[3], floorZ0AtLipFaceMm=round(f0 + fs * lip[0], 4), floorSlope=round(fs, 6),
                    floorResidualMm=round(fres, 3), floorSamples=fn,
                    riserFaceXMm=riser[0], riserTopMm=riser[5], riserThicknessMm=round(riser[3] - riser[0], 3),
                    riserYMm=[riser[1], riser[4]], floorYMm=[fb[1], fb[4]]))
    # Official parts that interpenetrate in the official model become one solid; every outer surface stays as drawn.
    for fx in chosen["fixtures"]:
        for group in fx.get("unite", []):
            members = [r for r in rows if r["role"] in group["roles"]]
            assert len(members) == len(group["roles"]), group["name"]
            united = solids[members[0]["name"]]
            for m in members[1:]:
                op = BRepAlgoAPI_Fuse(united, solids[m["name"]])
                assert op.IsDone(), group["name"]
                united = op.Shape()
            found = solids_of(united)
            assert len(found) == 1, (group["name"], len(found))
            total = sum(volume(solids[m["name"]]) for m in members)
            assert 0.0 <= total - volume(found[0]) < 0.001 * total, (group["name"], total, volume(found[0]))
            for m in members:
                del solids[m["name"]]
            rows[:] = [r for r in rows if r not in members]
            solids[group["name"]] = found[0]
            rows.append(dict(name=group["name"], fixture=fx["id"], official="; ".join(m["official"] for m in members), role=group["role"],
                             unitedFrom=[m["name"] for m in members], overlapRemovedMm3=round(total - volume(found[0]), 1),
                             boundsMm=[round(v, 2) for v in bounds(found[0])]))
    # Official parts that touch each other: the official model's own joints. None may still overlap.
    joints = []
    for (a, sa), (b, sb) in combinations(sorted(solids.items()), 2):
        ba, bb = bounds(sa), bounds(sb)
        if any(ba[i] - bb[i + 3] > TOUCH_MM or bb[i] - ba[i + 3] > TOUCH_MM for i in range(3)):
            continue
        d = BRepExtrema_DistShapeShape(sa, sb)
        if d.IsDone() and d.Value() < TOUCH_MM:
            overlap = 0.0
            if d.Value() <= 1e-7:
                op = BRepAlgoAPI_Common(sa, sb)
                overlap = volume(op.Shape()) if op.IsDone() else float("nan")
            joints.append(dict(pair=[a, b], gapMm=round(d.Value(), 3), overlapMm3=round(overlap, 3)))
    assert all(j["overlapMm3"] == 0 for j in joints), [j for j in joints if j["overlapMm3"] != 0]
    Interface_Static.SetCVal_s("write.step.unit", "MM")
    out_dir.mkdir(parents=True, exist_ok=True)
    for row in rows:
        name = row["name"]
        row["file"] = f"{row['fixture']}-{row['role']}.step"
        doc = TDocStd_Document(TCollection_ExtendedString("BinXCAF"))
        table = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
        label, definition = table.AddShape(solids[name], False), TDF_Label()
        TDataStd_Name.Set_s(label, TCollection_ExtendedString(name))
        if ST.GetReferredShape_s(label, definition):      # a placed solid: name its definition too, which readers report
            TDataStd_Name.Set_s(definition, TCollection_ExtendedString(name))
        writer = STEPCAFControl_Writer()
        target = out_dir / row["file"]
        assert not target.exists(), row["file"]
        assert writer.Transfer(doc, STEPControl_AsIs) and writer.Write(str(target)) == IFSelect_RetDone
    out_json.write_text(json.dumps(dict(
        schema="frc-2025-poc/checked-fixtures@1",
        source=dict(record="cad/field/official/official-field.json", selection="cad/field/official/checked-fixtures.json",
                    edition=record["source"]["edition"], url=record["source"]["url"]),
        units="mm", frame="robot datum: +X toward the reef, +Y port, +Z up, origin at the chassis centre on the floor",
        docks=[dict(fixture=fx["id"], official=fx["official"], dockFrame=fx["dockFrame"], face=fx["dockFace"]["name"],
                    faceXMm=fx["dockFaceXMm"], shiftFromRecordedFrameXMm=fx["shiftXMm"]) for fx in chosen["fixtures"]],
        parts=rows, officialJoints=joints, section=section), indent=1) + "\n", encoding="utf-8")
    print(len(solids), "fixture solids;", len(joints), "joints; chute", section["station"]["chuteExitEdgeMm"], section["station"]["chuteSlopeDeg"],
          "floor", section["trough"]["floorZ0AtLipFaceMm"], section["trough"]["floorSlope"])


if __name__ == "__main__":
    main()
