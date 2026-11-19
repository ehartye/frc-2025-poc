"""Reopen the proposed ten axle retainers with the selected inner carrier clamps."""
import argparse
import json
import math
from pathlib import Path
import sys

from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeWire
from OCP.BRepPrimAPI import BRepPrimAPI_MakeRevol
from OCP.GC import GC_MakeArcOfCircle
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt
from measure import HERE, SHOP_STEP, along_y, cad, distance, lower_bound, read_shop
from converter_panel import digest
from service import finalize_step, write_step

sys.path.insert(0,str(HERE.parent/"design/installed-procurement"))
import clamps

NOSE_IN = 7.50
NOSE_OUT = 8.35
NOSE_HEIGHT = .30
SHIM_BORE = 6.95
PILOT = 2.02
STATIONS = {"front":247.65,"center":0.,"rear":-247.65}
GROOVE_START = 314.19
GROOVE_WIDTH = 1.03
GROOVE_RADIUS = 5.715
CLIP_THICK = .889
CLIP_ENVELOPE = 7.62
THRUST_LENGTH = 2.18


def boolean(op,a,b):
    operation = op(a,b)
    if not operation.IsDone():
        raise ValueError("Failed retainer Boolean")
    return operation.Shape()


def subtract(a,b):
    return boolean(BRepAlgoAPI_Cut,a,b)


def unite(a,b):
    return boolean(BRepAlgoAPI_Fuse,a,b)


def ring(x,y,direction,length,inner,outer):
    return subtract(along_y(x,y,direction,outer,length),along_y(x,y-direction,direction,inner,length+2))


def machine_groove(shaft,sy):
    # Turn the entire rounded hex to a cylindrical bottom and two R0.10 fillets.
    start,stop = GROOVE_START,GROOVE_START+GROOVE_WIDTH
    r,fillet = GROOVE_RADIUS,.10
    def p(radius,y):
        return gp_Pnt(radius,sy*y,76.2)
    wire=BRepBuilderAPI_MakeWire()
    def line(a,b):
        wire.Add(BRepBuilderAPI_MakeEdge(p(*a),p(*b)).Edge())
    def arc(a,b,c):
        wire.Add(BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(p(*a),p(*b),p(*c)).Value()).Edge())
    q=fillet*(1-1/math.sqrt(2))
    line((8,start),(r+fillet,start))
    arc((r+fillet,start),(r+q,start+q),(r,start+fillet))
    line((r,start+fillet),(r,stop-fillet))
    arc((r,stop-fillet),(r+q,stop-q),(r+fillet,stop))
    line((r+fillet,stop),(8,stop))
    line((8,stop),(8,start))
    tool=BRepPrimAPI_MakeRevol(BRepBuilderAPI_MakeFace(wire.Wire()).Face(),
                               gp_Ax1(p(0,0),gp_Dir(0,1,0))).Shape()
    return subtract(shaft,tool)


def shim(x,y,direction,length):
    body = ring(x,y,-direction,length-NOSE_HEIGHT,SHIM_BORE,NOSE_OUT)
    face = ring(x,y-direction*(length-NOSE_HEIGHT),-direction,NOSE_HEIGHT,NOSE_IN,NOSE_OUT)
    return unite(body,face)


def swept_cylinder(x,y,sign,radius,length,travel):
    # Exact continuous union for a cylinder translated parallel to robot X.
    center = along_y(x,y,sign,radius,length)
    if not travel:
        return center
    prism = cad.brep(["box",[x-travel,min(y,y+sign*length),76.2-radius,2*travel,length,2*radius]])
    return unite(unite(cad.translated(center,[-travel,0,0]),prism),cad.translated(center,[travel,0,0]))


def witnesses(shape,parts,bounds,exclude):
    bb = cad.bounds(shape)
    pairs = sorted((lower_bound(bb,box),name) for name,box in bounds.items() if name not in exclude)
    hits = {}
    gap = math.inf
    limiting = None
    for lower,name in pairs:
        if lower>gap+1e-5:
            break
        actual = distance(shape,parts[name])
        if actual < gap:
            gap,limiting = actual,name
        if lower<=.001:
            volume = cad.intersection(shape,parts[name])
            if volume>.001:
                hits[name] = round(volume,4)
    return {"clearanceMm":round(gap,4),"closest":limiting,"overlapMm3":hits}


def add_exchange(scene,path,names,expected):
    if digest(path)!=expected:
        raise ValueError("Wrong issued native exchange: "+path.name)
    scene.update(cad.read_named(path,[{"id":name} for name in names]))


def chassis_scene(artifacts):
    shop,_ = read_shop(artifacts)
    scene = {k:v for k,v in shop.items() if k not in clamps.REPLACED}
    clamp_names = {k for sy in (1,-1) for pos in STATIONS for k in clamps.parts(sy,pos)}
    clamp_review = json.loads((clamps.HERE/"clamp-review.json").read_bytes())
    add_exchange(scene,clamps.HERE/"inner-clamps.step",clamp_names,clamp_review["candidateStepSha256"])
    frame = HERE.parent/"frame-corners"
    frame_receipt = json.loads((frame/"weld-fit.json").read_bytes())
    welds = [f"corner-{tag}-{region}" for tag in frame_receipt["corners"]
             for region in ("fillet","groove","bottom-cap","top-cap")]
    rails = ["cross-front","cross-rear","drive-rail-outer-L","drive-rail-outer-R"]
    add_exchange(scene,frame/"frame-weldment.step",rails+welds,frame_receipt["stepSha256"])
    # Screw shanks and head profiles sweeping around each wheel's bolt circle.
    # The selected shop study limits the actual head to 11 mm OD by 4 mm high.
    hub_face = 302.7+.188*25.4
    for pos,x in STATIONS.items():
        for side,sy in (("L",1),("R",-1)):
            scene[f"wheel-hardware-head-{pos}-{side}"] = ring(x,sy*hub_face,sy,4,23.8125-5.5,23.8125+5.5)
            scene[f"wheel-hardware-thread-{pos}-{side}"] = ring(x,sy*hub_face,-sy,31.8,23.8125-2.45,23.8125+2.45)
    return shop,scene


def check(artifacts,out):
    shop,scene=chassis_scene(artifacts)
    bounds = {k:cad.bounds(v) for k,v in scene.items()}
    removable = {k for k in scene if k.startswith("bumper-") and "sleeve" not in k}
    fitted = {}
    locations = {}
    for pos,x in STATIONS.items():
        travel = 0 if pos=="center" else 4
        for side,sy in (("L",1),("R",-1)):
            axle = f"axle-{pos}-{side}"
            shaft = shop[axle]
            if not travel:
                shaft = machine_groove(shaft,sy)
                name = "spacer-center-3-"+side
                fitted[name] = subtract(shop[name],along_y(0,sy*314.05,sy,10,5))
                clip_y=GROOVE_START+(GROOVE_WIDTH-CLIP_THICK)/2
                blank=ring(0,sy*clip_y,sy,CLIP_THICK,GROOVE_RADIUS-.001,CLIP_ENVELOPE)
                fitted["clip-sweep-"+side]=subtract(blank,shaft)
                fitted["clip-thrust-"+side]=shim(0,sy*315.18,-sy,THRUST_LENGTH)
            for face,ds in (("outer",1),("inner",-1)):
                if not travel and face=="inner":
                    continue
                s = sy*ds
                y = sy*(327 if face=="outer" else 233.4)
                length = 1.55 if face=="outer" else 1.15
                tag = f"{pos}-{side}-{face}"
                bearing = f"bearing-{pos}-{face}-{side}"
                pilot = along_y(x,y+s,-s,PILOT,18)
                shaft = subtract(shaft,pilot)
                head = along_y(x,y,s,8.5,3.8)
                screw = unite(head,along_y(x,y,-s,2.0,12.7))
                fitted["retaining-screw-"+tag] = screw
                fitted["select-shim-"+tag] = shim(x,y,s,length)
                # At least 16 mm of straight withdrawal is needed to clear 12.7 mm of thread.
                seated = swept_cylinder(x,y,s,8.5,3.8,travel)
                removed = swept_cylinder(x,y,s,8.5,19.8,travel)
                tip = y+s*3.8
                driver = unite(swept_cylinder(x,tip,s,3,81,travel),
                               swept_cylinder(x,tip+s*65,s,11,86,travel))
                ignored = {axle,bearing}
                access = {}
                for key,gauge in (("seat",seated),("withdraw",removed),("driver",driver)):
                    access[key] = witnesses(gauge,scene,bounds,ignored)
                    if access[key]["overlapMm3"]:
                        ends = ("front","rear") if pos=="center" else (pos,)
                        released = {k for k in removable if any(f"-{end}" in k for end in ends)}
                        access[key+"AfterRemoval"] = witnesses(gauge,scene,bounds,ignored|released)
                        access[key+"RemovedEnds"] = ends
                if any(v["overlapMm3"] for k,v in access.items() if k.endswith("AfterRemoval")):
                    raise ValueError("Selected service sequence is obstructed at "+tag)
                locations[tag] = {"raceGapMm":distance(fitted["select-shim-"+tag],shop[bearing]),
                                  "pilotDepthMm":17,"access":access}
            fitted[axle] = shaft
    out.mkdir(parents=True,exist_ok=True)
    path=out/"axle-kit.step"
    write_step(path,fitted)
    finalize_step(path)
    reopen = cad.read_named(path,[{"id":k} for k in fitted])
    if len(reopen)!=32:
        raise ValueError("Changed axle assembly inventory")
    result = {"shopSha256":SHOP_STEP,"lightClampStepSha256":digest(clamps.HERE/"inner-clamps.step"),
              "frameStepSha256":digest(HERE.parent/"frame-corners/frame-weldment.step"),
              "reopenedBodies":len(reopen),"stepSha256":digest(path),"sceneCount":len(scene),
              "maxReopenVolumeErrorMm3":max(abs(cad.volume(v)[0]-cad.volume(reopen[k])[0]) for k,v in fitted.items()),
              "volumeMm3":{k:cad.volume(v)[0] for k,v in fitted.items()},
              "originalVolumeMm3":{k:cad.volume(shop[k])[0] for k in fitted if k in shop},
              "locations":locations}
    (out/"fit.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8",newline="\n")
    return result


if __name__=="__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifacts",type=Path)
    parser.add_argument("out",type=Path)
    args = parser.parse_args()
    check(args.artifacts,args.out)
