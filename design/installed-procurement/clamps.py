"""Fit a removable clamp at every previously unsecured inner drivetrain carrier."""
from itertools import combinations
from pathlib import Path
import json
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PACKAGE = ROOT / "design/competition-package/shop-duplex"
sys.path.insert(0, str(PACKAGE))
from assembly import cad, sha
from build_successor import write_json
from check_routing import clashes
from duplex import finalize_step
from mechanical import box, cut, cylinder
from drive_parts import CENTERS, Z

STATIONS = {"rear": -CENTERS, "center": 0, "front": CENTERS}
REPLACED = {f"gearbox-faceplate-{lr}" for lr in "LR"} | {
    f"carrier-{station}-inner-{lr}" for station in ("front","rear") for lr in "LR"}


def parts(side, station, adjustment=0, light=True):
    """Robot-coordinate assembly. Inboard faces are the small |Y| faces."""
    lr = "L" if side == 1 else "R"
    x = STATIONS[station] + adjustment
    prefix = f"inner-{lr}-{station}-"
    result = {}

    def billet(u, t, z, length, depth, height):
        return box(u, t if side > 0 else -t-depth, z, length, depth, height)

    def cyl(u, z, t, length, radius):
        return cylinder(u, side*t, z, [0, side, 0], radius, length)

    def hardware(tag, u, z, t, length):
        # Suppress the female thread and male socket. Model the limiting head OD/height.
        result[prefix+tag+"-washer"] = cad.brep(cut(cyl(u,z,t,1,5),cyl(u,z,t-1,3,2.65)))
        result[prefix+tag+"-screw"] = cad.brep(["union",[
            cyl(u,z,t+1,3.5,4.25), cyl(u,z,t+1-length,length,2.5)]])

    centers = [(x+du,Z+dv,f"{du}_{dv}") for du in (-19,19) for dv in (-17,17)]
    if station=="center":
        for u,z,tag in centers:
            hardware(tag,u,z,241,17.5)
    if station == "center":
        if adjustment: raise ValueError("The center bearing is fixed")
        plate = billet(-48,224,Z-33,96,5,66)
        drills = [cyl(0,Z,223,7,18)]
        drills.extend(cyl(0,Z+dv,223,7,2.65) for dv in (-25.4,25.4))
        drills.extend(cyl(u,z,223,7,2.5) for u,z,_ in centers)
        for du in (-40,40):
            for dv in (-12,12):
                drills.append(cyl(du,Z+dv,223,7,2.5))
                hardware(f"adapter-{du}_{dv}",du,Z+dv,235,11.5)
        if light:
            # An octagonal blank retains material behind each clamp. Two
            # windows sit between the socket circle and the outer clamp columns.
            from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
            from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace
            from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
            from OCP.gp import gp_Pnt, gp_Vec
            outline=[(-28,-33),(28,-33),(48,-13),(48,13),(28,33),(-28,33),(-48,13),(-48,-13)]
            wire=BRepBuilderAPI_MakePolygon()
            for u,v in outline:wire.Add(gp_Pnt(u,side*224,Z+v))
            wire.Close()
            body=BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(wire.Wire()).Face(),gp_Vec(0,side*5,0)).Shape()
            drills.extend(cyl(u,Z,223,7,7) for u in (-32,32))
            for drill in drills:
                op=BRepAlgoAPI_Cut(body,cad.brep(drill))
                if not op.IsDone():raise ValueError("Faceplate machining failed")
                body=op.Shape()
            result[prefix+"threaded-faceplate"]=body
        else:result[prefix+"threaded-faceplate"] = cad.brep(cut(plate,*drills))
    else:
        plate=billet(x-28,235,Z-26,56,6,52)
        holes=[cyl(x,Z,234,8,14.31)]
        for u,z,tag in centers:
            if light:
                holes.append(cyl(u,z,234,8,2.65))
                holes.append(billet(u-5.1,237.25,z-5.1,10.2,5,10.2))
                for corner_u in (-5.1,5.1):
                    for corner_v in (-5.1,5.1):holes.append(cyl(u+corner_u,z+corner_v,237.25,5,1))
                result[prefix+tag+"-square-nut"]=cad.brep(cut(
                    billet(u-5,237.25,z-5,10,6,10),cyl(u,z,236.25,8,2.5)))
            else:holes.append(cyl(u,z,234,8,2.5))
            # Standard M5 x 12 enters the tapped carrier from inside the chassis.
            thickness=2 if light else 1
            head_face=229-thickness
            length=16 if light else 12
            result[prefix+tag+"-washer"] = cad.brep(cut(cyl(u,z,head_face,thickness,5),cyl(u,z,head_face-1,4,2.65)))
            result[prefix+tag+"-screw"] = cad.brep(["union",[
                cyl(u,z,head_face-5,5,4.25),cyl(u,z,head_face,length,2.5)]])
        result[prefix+("pocket-carrier" if light else "threaded-carrier")] = cad.brep(cut(plate,*holes))
    return result


def source_robot(pose):
    directory = PACKAGE / "red"
    row = json.loads((directory/"review.json").read_bytes())[pose]
    step = directory/(pose+".step")
    if sha(step) != row["export_sha256"]: raise ValueError("Changed robot source STEP")
    names = json.loads((directory/"part-frames.json").read_bytes())
    return cad.read_named(step,[{"id":name} for name in names]),row["export_sha256"]


def study(light=True):
    receipt = {"replaces":sorted(REPLACED),"poses":{},"takeup":{},"toolAccess":{}}
    new = {}
    for side in (-1,1):
        for station in STATIONS:
            new.update(parts(side,station,light=light))
    for a,b in combinations(new,2):
        if clashes({a:new[a]},{b:new[b]}): raise ValueError(("candidate pair",a,b))
    source,digest = source_robot("stowed")
    for pose in ("stowed","receive","transport","place"):
        old,source_sha = (source,digest) if pose=="stowed" else source_robot(pose)
        neighbors = {k:s for k,s in old.items() if k not in REPLACED}
        hits = clashes(new,neighbors)
        if hits: raise ValueError((pose,hits))
        receipt["poses"][pose] = {"redStepSha256":source_sha,"candidateBodies":len(new),
                                  "retainedBodies":len(neighbors),"intersectionsMm3":hits}
    for side in (-1,1):
        lr = "L" if side==1 else "R"
        for station in STATIONS:
            for shift in ((0,) if station=="center" else (-4,0,4)):
                group = parts(side,station,shift,light)
                carrier_name = f"carrier-{station}-inner-{lr}"
                neighbors = {k:s for k,s in source.items() if k not in REPLACED}
                if shift:
                    # Each slotted end bearing follows its axle, wheel, hub and spacers.
                    for k in list(neighbors):
                        if k.startswith((f"bearing-{station}-",f"axle-{station}-",f"wheel-{station}-",
                                         f"hub-{station}-",f"spacer-{station}-",f"jack-{station}-")) and k.endswith(lr):
                            neighbors[k]=cad.translated(neighbors[k],(shift,0,0))
                collisions = clashes(group, neighbors)
                if collisions: raise ValueError((lr,station,shift,collisions))
                receipt["takeup"][f"{lr}-{station}-{shift}"] = {"clampInterferenceMm3":collisions}
            # A 4 mm AF end socket key fits in this conservative 5 mm circle.
            # Approach axially through the wheel bay before fitting the wheel/chain pack.
            tools = {}
            for name,shape in parts(side,station,light=light).items():
                if not name.endswith("-screw"): continue
                x,y,z,xx,yy,zz = cad.bounds(shape)
                outside = max(abs(y),abs(yy)) if station=="center" else min(abs(y),abs(yy))
                direction = side if station=="center" else -side
                tools[name] = cad.brep(cylinder((x+xx)/2,side*outside,(z+zz)/2,[0,direction,0],2.5,25))
            occupied = clashes(tools,source)
            removed = {pair.split(" / ")[1] for pair in occupied}
            receipt["toolAccess"][f"{lr}-{station}"] = {"diameterMm":5,"reachMm":25,
                "requiresRemoval":sorted(removed),"assembledConflictsMm3":occupied}
    # Compare candidate stock and hardware to six removed aluminum originals.
    receipt["deltaKg"] = sum(cad.volume(s)[0]*(2.70e-6 if k.endswith("pocket-carrier") else 7.85e-6) for k,s in new.items()) - sum(
        cad.volume(source[name])[0]*2.70e-6 for name in REPLACED)
    labels = [{"id":k,"group":"shaft"} for k in new]
    step = HERE/("inner-clamps.step" if light else "steel-alternative.step")
    cad.write_named(step,list(new.values()),labels)
    finalize_step(step)
    restored = cad.read_named(step,labels)
    errors = {}
    for name,shape in new.items():
        common = cad.intersection(shape,restored[name])
        errors[name] = max(abs(cad.volume(v)[0]-common) for v in (shape,restored[name]))
    if max(errors.values())>.01: raise ValueError("Candidate changed on STEP reopen")
    receipt["candidateStepSha256"] = sha(step)
    receipt["reopenedNamedBodies"] = len(restored)
    receipt["maxReopenDifferenceMm3"] = max(errors.values())
    return receipt


if __name__=="__main__":
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steel",action="store_true",help="Rebuild the heavier directly threaded alternative")
    args=parser.parse_args()
    write_json(HERE/("steel-review.json" if args.steel else "clamp-review.json"),study(not args.steel))
