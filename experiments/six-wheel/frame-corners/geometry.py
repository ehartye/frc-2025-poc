"""Fit four machined corner splices to the registered fixed-height shop chassis."""
import argparse
import csv
import json
import math
from pathlib import Path
import sys

from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Pnt, gp_Vec

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT/"moving-harness"))
from continuous_screen import SHOP_STEP, read_shop
from converter_panel import cad, digest, distance, nearest
from service import write_step, finalize_step

LEG = 64.0
WALL = 2.5
BOTTOM = 106.0
TOP = 145.0
OFFSETS = (32.0,52.0)
LEVELS = (116.0,132.0)
BORE = 4.925
WHEEL_THROW = 4.0


def cut(part, tool):
    op = BRepAlgoAPI_Cut(part,tool)
    if not op.IsDone():
        raise ValueError("Native cut failed")
    return op.Shape()


def join(a,b):
    op = BRepAlgoAPI_Fuse(a,b)
    if not op.IsDone():
        raise ValueError("Native union failed")
    return op.Shape()


def prism(sx,sy,uv,low,high):
    loop = BRepBuilderAPI_MakePolygon()
    for u,v in uv:
        loop.Add(gp_Pnt(sx*(330-u),sy*(325-v),low))
    loop.Close()
    return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(loop.Wire()).Face(),gp_Vec(0,0,high-low)).Shape()


def splice(sx,sy):
    outline = [(0,1),(1,0),(LEG,0),(LEG,WALL),(WALL,WALL),(WALL,LEG),(0,LEG)]
    kit = prism(sx,sy,outline,BOTTOM,TOP)
    # Keep a 3 mm top web. A 4 mm vertical end mill leaves an inside R2 in the pocket.
    cap = prism(sx,sy,[(0,1),(1,0),(48,0),(0,48)],TOP-3,TOP)
    r = 2
    square = prism(sx,sy,[(WALL,WALL),(WALL+r,WALL),(WALL+r,WALL+r),(WALL,WALL+r)],BOTTOM,TOP-3)
    smooth = cad.brep(["cyl",[sx*(330-WALL-r),sy*(325-WALL-r),BOTTOM,0,0,1,r,TOP-BOTTOM-3]])
    return join(join(kit,cap),cut(square,smooth))


def axis_cylinder(sx,sy,face,offset,z,start,end,radius):
    # Distance is positive toward the empty chassis interior, zero at the rail wall.
    u,v = (start,offset) if face=="end" else (offset,start)
    vector = (-sx,0,0) if face=="end" else (0,-sy,0)
    direction = 1 if end>start else -1
    return cad.brep(["cyl",[sx*(330-u),sy*(325-v),z,*(direction*c for c in vector),radius,abs(end-start)]])


def fitted_corner(parts,sx,sy):
    tag = ("front" if sx>0 else "rear")+"-"+("L" if sy>0 else "R")
    rails = {"end":"cross-"+("front" if sx>0 else "rear"),
             "side":"drive-rail-outer-"+("L" if sy>0 else "R")}
    bracket = splice(sx,sy)
    rivets = {}
    operations = []
    tool_axes = []
    for face,rail in rails.items():
        for offset in OFFSETS:
            for z in LEVELS:
                def cyl(start,end,radius):
                    return axis_cylinder(sx,sy,face,offset,z,start,end,radius)
                bore = cyl(WALL+1,-3,BORE/2)
                before = cad.volume(parts[rail])[0]
                parts[rail] = cut(parts[rail],bore)
                drilled = before-cad.volume(parts[rail])[0]
                if not math.isclose(drilled,math.pi*(BORE/2)**2*2,abs_tol=.001):
                    raise ValueError(f"Rail hole did not cut one 2 mm wall in {rail}")
                bracket = cut(bracket,bore)
                head = cyl(WALL,WALL+.090*25.4,.385*25.4/2)
                shank = cyl(WALL,-2,.192*25.4/2)
                # Installed blind bulb is a reserved envelope, verified on the purchased lot.
                bulb = cyl(-2,-8.5,5.2)
                key = f"rivet-{tag}-{face}-{int(offset)}-{int(z)}"
                rivets[key] = join(join(head,shank),bulb)
                tool_axes.append((key,face,offset,z))
                operations.append({"corner":tag,"rail":rail,"face":face,"fromInsideCornerMm":offset,
                                   "heightAboveFloorMm":z,"boreMm":BORE})
    return tag,rails,bracket,rivets,tool_axes,operations


def check(artifacts,out):
    original,_ = read_shop(artifacts)
    parts = dict(original)
    rail_names = {"cross-front","cross-rear","drive-rail-outer-L","drive-rail-outer-R"}
    corners = {}
    package = {}
    operations = []
    for sx,sy in ((1,1),(1,-1),(-1,1),(-1,-1)):
        tag,rails,bracket,rivets,axes,steps = fitted_corner(parts,sx,sy)
        operations.extend(steps)
        key = "corner-splice-"+tag
        package.update({key:bracket,**rivets})
        wheel_key = "wheel-"+tag
        tire = original[wheel_key]
        outer_tire = cad.translated(tire,[sx*WHEEL_THROW,0,0])
        # Swept tire bound: any intermediate center is within 0.25 mm of a sampled center.
        installed = {key:bracket,**rivets}
        min_gap = min((distance(shape,cad.translated(tire,[sx*dx/2,0,0])),name,dx/2)
                      for dx in range(-8,9) for name,shape in installed.items())
        low = prism(sx,sy,[(0,2.5),(2.5,2.5),(2.5,44),(0,44)],62,145)
        obstructions = {k:v for k,v in parts.items() if k not in rails.values()}
        boxes = {k:cad.bounds(v) for k,v in obstructions.items()}
        nearest_other = min((n["mm"],member,n["bodies"]) for member,shape in installed.items()
                            for n in [nearest(shape,obstructions,boxes)])
        stationary_tools = {k:parts[k] for k in rail_names}
        stationary_tools[key] = bracket
        stationary_tools.update(rivets)
        tb = {k:cad.bounds(v) for k,v in stationary_tools.items()}
        tool_gap = math.inf
        blind_gap = math.inf
        for rivet,face,offset,z in axes:
            # H701B-456 head envelope and additional shop gauge for the power unit.
            tip = WALL+.090*25.4
            nose = axis_cylinder(sx,sy,face,offset,z,tip,tip+44.45,9.525)
            body = axis_cylinder(sx,sy,face,offset,z,tip+44.45,tip+210,27)
            tool_gap = min(tool_gap,*(nearest(shape,stationary_tools,tb,{rivet})["mm"] for shape in (nose,body)))
            reserve = axis_cylinder(sx,sy,face,offset,z,-2.01,-2-.415*25.4,5.2)
            blind_gap = min(blind_gap,nearest(reserve,stationary_tools,tb,{rivet})["mm"])
        corners[tag] = {"wheelGapLowerMm":min_gap[0]-.25,"wheelLimiter":min_gap[1],
                        "wheelLimiterAdjustmentMm":min_gap[2],"nearestInstalled":nearest_other,
                        "lowBlankOverlapAtOuterTakeupMm3":cad.intersection(low,outer_tire),
                        "bareFrameToolGapMm":tool_gap,"blindReserveGapMm":blind_gap,
                        "machinedSpliceMm3":cad.volume(bracket)[0]}
    package.update({k:parts[k] for k in rail_names})
    out.mkdir(exist_ok=True,parents=True)
    exchange = out/"frame-joints.step"
    write_step(exchange,package)
    finalize_step(exchange)
    reopened = cad.read_named(exchange,[{"id":k} for k in package])
    max_volume_error = max(abs(cad.volume(obj)[0]-cad.volume(reopened[k])[0]) for k,obj in package.items())
    if max_volume_error>.001:
        raise ValueError("Joint material changed on reimport")
    with (out/"drill-table.csv").open("w",encoding="utf-8",newline="") as handle:
        writer = csv.DictWriter(handle,fieldnames=operations[0],lineterminator="\n")
        writer.writeheader()
        writer.writerows(operations)
    result = {"shopStepSha256":SHOP_STEP,"exchangeSha256":digest(exchange),"reopenedBodies":len(reopened),
              "forming":"machined single-piece cap and walls","rivetCount":len(operations),"corners":corners,
              "maxReopenedVolumeErrorMm3":max_volume_error,
              "removedRailMm3":sum(cad.volume(original[k])[0]-cad.volume(parts[k])[0] for k in rail_names)}
    (out/"fit.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8",newline="\n")
    return result


if __name__=="__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifacts",type=Path)
    parser.add_argument("output",type=Path)
    args = parser.parse_args()
    print(json.dumps(check(args.artifacts,args.output)["corners"]))
