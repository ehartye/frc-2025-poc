"""Close and check all twelve flanged bearing pockets on the clamped chassis."""
import argparse
import json
import math
from pathlib import Path

from OCP.BRepPrimAPI import BRepPrimAPI_MakeCone
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt
from fit import HERE,STATIONS,SHOP_STEP,add_exchange,along_y,cad,clamps,digest,distance,finalize_step
from fit import read_shop, ring, subtract, unite, witnesses, write_step
from converter_panel import nearest

THICK = 3.10
POCKET = 1.65
INNER = 13.40
RELIEF = 16.00
BOLT_RADIUS = 18.90
ANGLES = (14,104,194,284)


def cylinder(x,y,z,direction,radius,length):
    return cad.brep(["cyl",[x,y,z,0,direction,0,radius,length]])


def cone(x,y,z,direction,r0,r1,length):
    axis=gp_Ax2(gp_Pnt(x,y,z),gp_Dir(0,direction,0))
    return BRepPrimAPI_MakeCone(axis,r0,r1,length).Shape()


def plate(x,y,s):
    body = along_y(x,y,s,17.5,THICK)
    holes=[]
    for angle in ANGLES:
        theta=math.radians(angle)
        px,pz=x+BOLT_RADIUS*math.cos(theta),76.2+BOLT_RADIUS*math.sin(theta)
        body=unite(body,cylinder(px,y,pz,s,3.95,THICK))
        holes.append((px,pz))
    body=subtract(body,along_y(x,y-s,s,INNER,THICK+2))
    body=subtract(body,along_y(x,y-s,s,RELIEF,POCKET+1))
    screws=[]
    for px,pz in holes:
        bore=cylinder(px,y-s,pz,s,1.65,THICK+2)
        # 90 degree countersink, 6.8 mouth. Flat head stays flush at received size.
        sink=cone(px,y+s*THICK,pz,-s,3.4,1.65,1.75)
        body=subtract(subtract(body,bore),sink)
        head=cone(px,y+s*THICK,pz,-s,3.36,1.49,1.87)
        # Preserve conservative full head; male thread is suppressed to pilot radius.
        shank=cylinder(px,y+s*(THICK-1.87),pz,-s,1.24,8-1.87)
        screws.append(unite(head,shank))
    return body,holes,screws


def check(artifacts,out):
    shop,_=read_shop(artifacts)
    hardware={k for sy in (-1,1) for pos in STATIONS for k in clamps.parts(sy,pos)}
    parts={k:v for k,v in shop.items() if k not in clamps.REPLACED}
    data=json.loads((clamps.HERE/"clamp-review.json").read_bytes())
    add_exchange(parts,clamps.HERE/"inner-clamps.step",hardware,data["candidateStepSha256"])
    # Full wheel fastener rotations are additional possible occupants.
    for pos,x in STATIONS.items():
        for side,sy in (("L",1),("R",-1)):
            y=sy*(302.7+.188*25.4)
            parts[f"rotating-hub-screws-{pos}-{side}"]=ring(x,y,sy,4,23.8125-5.5,23.8125+5.5)
            parts[f"rotating-hub-threads-{pos}-{side}"]=ring(x,y,-sy,31.8,23.8125-2.45,23.8125+2.45)
    bounds={k:cad.bounds(v) for k,v in parts.items()}
    kit={}
    result={}
    removed_volumes={}
    for pos,x in STATIONS.items():
        for side,sy in (("L",1),("R",-1)):
            for face,ds in (("outer",-1),("inner",1)):
                tag=f"{pos}-{face}-{side}"
                carrier="carrier-"+tag
                if face=="inner" and pos!="center":
                    carrier=f"inner-{side}-{pos}-pocket-carrier"
                bearing="bearing-"+tag
                y=sy*(319 if face=="outer" else 241)
                s=sy*ds
                cap,holes,screws=plate(x,y,s)
                revised=parts[carrier]
                start_volume=cad.volume(revised)[0]
                for px,pz in holes:
                    before=cad.volume(revised)[0]
                    revised=subtract(revised,cylinder(px,y+s,pz,-s,1.25,8))
                    if not math.isclose(before-cad.volume(revised)[0],6*math.pi*1.25**2,abs_tol=.001):
                        raise ValueError("Keeper tap breaks into existing void: "+tag)
                kit[carrier]=revised
                removed_volumes[carrier]=start_volume-cad.volume(revised)[0]
                kit["bearing-keeper-"+tag]=cap
                for n,screw in enumerate(screws,1):
                    kit[f"keeper-M3-{tag}-{n}"]=screw
                # The separate screws may move relative to the plate only on removal.
                # Keep their individual material checks, including the drilled carrier.
                fixed={k:v for k,v in parts.items() if k not in (carrier,bearing)}
                fb={k:bounds[k] for k in fixed}
                witness=witnesses(cap,fixed,fb,set())
                clashes={}
                for n,obj in enumerate(screws):
                    for k,v in {**witnesses(obj,fixed,fb,set())["overlapMm3"],
                                carrier:cad.intersection(obj,revised),"keeper":cad.intersection(obj,cap)}.items():
                        if v>.001:
                            clashes[f"{n+1}/{k}"]=v
                # The shoulder side of the housing blocks motion in one direction.
                # Without the keeper, travel toward the wheel has no geometric stop in the carrier.
                loose=cad.translated(parts[bearing],[0,s*.5,0])
                hit=cad.translated(parts[bearing],[0,s*.11,0])
                row={"nearestOther":witness,"screwInterferencesMm3":clashes,
                     "nominalCapToBearingMm":distance(cap,parts[bearing]),
                     "unretainedCarrierCommonAtHalfMm3":cad.intersection(loose,revised),
                     "oppositeCarrierCommonAt005Mm3":cad.intersection(cad.translated(parts[bearing],[0,-s*.05,0]),revised),
                     "keeperCommonAt011Mm3":cad.intersection(hit,cap)}
                if pos!="center":
                    # Distances between parts on the same adjustable carriage do not
                    # change. Challenge all other registered hardware as stationary.
                    follower = {name for name in parts if
                                (name.startswith((f"inner-{side}-{pos}-",f"{pos}-{side}-")) or
                                 (name.endswith("-"+side) and name.startswith(tuple(
                                  prefix+pos+"-" for prefix in ("axle-","bearing-","wheel-","hub-","spacer-",
                                  "carrier-","rotating-hub-screws-","rotating-hub-threads-")))))}
                    moving_clearance = min(
                        nearest(cad.translated(obj,[offset,0,0]),parts,bounds,follower)["mm"]
                        for offset in range(-4,5) for obj in (cap,*screws))
                    row["fixedHardwareFullTakeupLowerMm"]=moving_clearance-.501
                    if moving_clearance<=.501:
                        raise ValueError(f"Adjustment clearance requires exact refinement at {tag}")
                result[tag]=row
                if (witness["overlapMm3"] or clashes or row["keeperCommonAt011Mm3"]<=.01
                    or row["oppositeCarrierCommonAt005Mm3"]<=.01 or row["unretainedCarrierCommonAtHalfMm3"]>.001):
                    raise ValueError("Keeper contact or clearance failed at "+tag+" "+json.dumps(row))
    out.mkdir(parents=True,exist_ok=True)
    path=out/"bearing-keepers.step"
    write_step(path,kit)
    finalize_step(path)
    reopened=cad.read_named(path,[{"id":k} for k in kit])
    result={"shopSha256":SHOP_STEP,"stepSha256":digest(path),"count":len(reopened),"locations":result,
            "volumeMm3":{k:cad.volume(v)[0] for k,v in kit.items()},
            "removedCarrierMm3":removed_volumes,
            "maxReopenVolumeErrorMm3":max(abs(cad.volume(v)[0]-cad.volume(reopened[k])[0]) for k,v in kit.items())}
    (out/"keepers.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8",newline="\n")
    return result


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for arg in ("artifacts","output"):
        parser.add_argument(arg,type=Path)
    a=parser.parse_args()
    check(a.artifacts,a.output)
