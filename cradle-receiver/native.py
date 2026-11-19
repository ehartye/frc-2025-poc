"""Rebuild a two-rail tipping cradle on the retained shelf and check the surrounding original robot."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
from model import ROOT, HERE, PIVOT, BACK_PROFILE
sys.path.insert(0,str(ROOT/"station-supply"))
from entry_window import registered_robot, ASSEMBLY, cad, receiver_drives
import assembly
from mechanical import cylinder
from duplex import finalize_step
from check_routing import write_step
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Ax1, gp_Pnt, gp_Dir, gp_Trsf

PIVOT_MM=(142.,18.)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def local_shapes():
    stuff={}
    profile=[(1000*x,1000*z) for x,z in BACK_PROFILE]
    # Four millimetre vertical thickness on the slopes. Rear wall is four millimetres thick in X.
    outline=profile+[(155,44),(-90,0),(-94,0),(-94,110)]
    for s in (-1,1):
        label="L" if s>0 else "R"
        y=85*s-15
        stuff["rail-"+label]=receiver_drives.prism([(x,y,z) for x,z in outline],(0,30,0))
        # Pivot axis below the working front edge. Bored side ears attach the rail to its hub.
        ear=receiver_drives.block(133,y,10,18,30,35)
        bore=cad.brep(cylinder(142,y-1,18,(0,1,0),5.1,32))
        stuff["rail-"+label]=cad.BRepAlgoAPI_Cut(cad.BRepAlgoAPI_Fuse(stuff["rail-"+label],ear).Shape(),bore).Shape()
        # Use a hub concentric with the surface endpoint. Its inboard half shares the body;
        # to retain the actual tube profile the hub must stay outside the two 30 mm working strips.
        hub=receiver_drives.annulus(142,100 if s==1 else -112,18,"y",8,5.1,12)
        stuff["hub-"+label]=hub
        sy=116 if s==1 else -128
        post=receiver_drives.block(133,sy,0,18,12,28)
        hole=cad.brep(cylinder(142,sy-1,18,(0,1,0),7,14))
        stuff["bearing-post-"+label]=cad.BRepAlgoAPI_Cut(post,hole).Shape()
        stuff["bearing-"+label]=receiver_drives.annulus(142,sy,18,"y",7,5,12)
        gy=180 if s==1 else -183
        guard=receiver_drives.block(-92,gy,2,248,3,53)
        if s==1:
            port=cad.brep(cylinder(142,gy-1,18,(0,1,0),6.,5))
            guard=cad.BRepAlgoAPI_Cut(guard,port).Shape()
        stuff["guard-"+label]=guard
        foot=cad.brep(["box",[-95,160 if s==1 else -188,0,250,28,2]])
        for x in (-80,140):
            cutter=cad.brep(cylinder(x,180*s,-1,(0,0,1),2.7,4))
            foot=cad.BRepAlgoAPI_Cut(foot,cutter).Shape()
        if s==1:
            toe=receiver_drives.block(121,188,0,42,20.5,2)
            foot=cad.BRepAlgoAPI_Fuse(foot,toe).Shape()
            flange=receiver_drives.block(121,208.5,0,42,3,42)
            flange=cad.BRepAlgoAPI_Cut(flange,cad.brep(cylinder(142,207.5,18,(0,1,0),6,6))).Shape()
            foot=cad.BRepAlgoAPI_Fuse(foot,flange).Shape()
        stuff["guard-foot-"+label]=foot
        # Positive capture seats take vertical impact near the rail back; direct the force to the retained shelf.
        stuff["capture-stop-"+label]=receiver_drives.prism([(x,y,z) for x,z in ((-70,0),(-58,0),(-58,32*44/245),(-70,20*44/245))],(0,30,0))
    # Transverse pivot shaft is an independent collision surface on the front discharge path.
    stuff["shaft"]=cad.brep(cylinder(142,-136,18,(0,1,0),5,352.5))
    # A single stock color-coded article reserve for the motor and still-to-be-detailed reduction.
    reserve=receiver_drives.block(122,211.5,-2,40,130,44)
    stuff["drive-reserve"]=cad.BRepAlgoAPI_Cut(reserve,cad.brep(cylinder(142,210.5,18,(0,1,0),5.3,8))).Shape()
    return stuff


def placed(solids,height,stroke,angle):
    xf=gp_Trsf()
    xf.SetRotation(gp_Ax1(gp_Pnt(142,0,18),gp_Dir(0,1,0)),math.radians(angle))
    trans={}
    for k,shape in solids.items():
        if k.startswith(("rail-","hub-")):
            shape=BRepBuilderAPI_Transform(shape,xf,False).Shape()
        trans[k]=cad.translated(shape,(stroke,0,height-60))
    return trans


def overlaps(a,b):
    ba,bb=cad.bounds(a),cad.bounds(b)
    if all(min(ba[i+3],bb[i+3])-max(ba[i],bb[i])>.0001 for i in range(3)):
        return cad.intersection(a,b)
    return 0.


def check(step):
    local=local_shapes()
    receive=placed(local,940,270,0)
    write_step(step,receive)
    finalize_step(step)
    reopened=cad.read_named(step,[{"id":k} for k in receive])
    if set(reopened)!=set(receive) or any(cad.solid_count(s)!=1 for s in reopened.values()):
        raise ValueError("Cradle export or part count changed")
    poses=[]
    baseline=registered_robot()
    retired={k for k in baseline if k.startswith("grip-")}
    for state,h,x,a in (("receive",940,270,0),("transport",670,0,0),("stow",940,0,0),
                          ("placed-flat",560,370,0),("place-tilted",560,370,18),("place-high-angle",560,370,21)):
        added=reopened if state=="receive" else placed(local,h,x,a)
        base=baseline if state=="receive" else assembly.pose_shapes(h,x,"red")
        hits={}
        fixed={k:(s,cad.bounds(s)) for k,s in base.items() if k not in retired}
        for k,shape in added.items():
            ba=cad.bounds(shape)
            for other,(original,bb) in fixed.items():
                if all(min(ba[i+3],bb[i+3])-max(ba[i],bb[i])>.0001 for i in range(3)):
                    v=cad.intersection(shape,original)
                    if v>.02:hits[k+"/"+other]=v
        internal={}
        for index,(k,shape) in enumerate(added.items()):
            for other,b in list(added.items())[index+1:]:
                volume=overlaps(shape,b)
                if volume>.02:internal[k+"/"+other]=volume
        poses.append(dict(state=state,originalBodyHitsMm3=hits,internalOverlapMm3=internal))
    with (ROOT/"design/receiver-drives/bom.csv").open() as f:
        items=[x for x in csv.DictReader(f) if x["id"] in retired]
    if {r["id"] for r in items}!=retired:raise ValueError("Retired geometry has an unpriced BOM item")
    return {"sourceSha256":{p.relative_to(ROOT).as_posix():sha(p) for p in (
                    ASSEMBLY/"red/source-register.json",ROOT/"design/receiver-drives/bom.csv",ROOT/"station-relief/geometry.json")},
            "nativeSha256":sha(step),"bodyCount":len(reopened),"retiredIds":sorted(retired),
            "retiredKg":[sum(float(r[col]) for r in items) for col in ("lowMassKg","highMassKg")],
            "nativeVolumeMm3":{k:cad.volume(s)[0] for k,s in reopened.items()},"poses":poses}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    p=parser.parse_args()
    p.output.write_text(json.dumps(check(p.step),indent=2)+"\n",encoding="utf-8",newline="\n")
