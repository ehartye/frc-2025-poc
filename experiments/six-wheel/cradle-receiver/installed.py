"""Finite-pipe spot checks against the rebuilt receiving robot and registered corner station."""
import argparse
from collections import defaultdict
from dataclasses import replace
import json
import math
from pathlib import Path

from study import BASE
from native import HERE, ROOT, ASSEMBLY, cad, local_shapes, placed, assembly, sha
from model import run
from dock_tolerance import bounded_dock, native_dock
from native_station import parts, FIRST_SHA
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt


def tube(x,y,z,angle,cfg):
    direction=(-math.sin(angle),math.cos(angle),0.)
    axis=gp_Ax2(gp_Pnt(x-500*cfg.length*direction[0],y-500*cfg.length*direction[1],z),gp_Dir(*direction))
    outer=500*cfg.diameter
    return BRepAlgoAPI_Cut(BRepPrimAPI_MakeCylinder(axis,outer,1000*cfg.length).Shape(),
                           BRepPrimAPI_MakeCylinder(axis,outer-1000*cfg.wall,1000*cfg.length).Shape()).Shape()


def audit(cfg,fixed,local):
    d=run(cfg,trace=True)
    path=d.pop("trace")
    times=[0,.025,.055,.085,.12,.16,.20,.24,.30,.40,.60,1.1,1.5,2.4,3.5,3.65,3.85,4.0]
    if d["exit"]:times.extend(d["exit"][0]+i*.01 for i in range(-4,12))
    if d["firstFloor"]:times.extend(d["firstFloor"][0]+i*.05 for i in range(7))
    hits=defaultdict(list);intended=set()
    fixed_boxes={state:{k:cad.bounds(s) for k,s in bodies.items()} for state,bodies in fixed.items()}
    for t in sorted(set(times)):
        row=min(path,key=lambda r:abs(r[0]-t))
        if abs(row[0]-t)>.005:continue
        stamp,x,z,vx,vz,omega,a,retract=row
        state="receive" if t<1.5 else "transport" if t<3.5 else "place"
        height,stroke={"receive":(940,270),"transport":(670,0),"place":(560,370)}[state]
        moving=placed(local,height,stroke-1000*retract,math.degrees(a))
        # Parent robot shelf-linked slides use the corresponding actual pose at release.
        if retract>0:
            # Conservative sample checks other robot solids at their original fully extended pose,
            # plus the actual translated shelf. Qualifying all retracting axes needs a new combined replay.
            shelf=cad.translated(fixed[state]["receiver-R6-ported"],(-1000*retract,0,0))
            extras={"receiver-R6-ported":shelf}
        else:extras={}
        shapes=fixed[state]|moving|extras
        piece=tube(1000*x+stroke,1000*(cfg.lateral+cfg.rail_offset_y),height-60+1000*z,math.radians(cfg.yaw),cfg)
        b=cad.bounds(piece)
        for key,obj in shapes.items():
            bb=fixed_boxes[state][key] if key in fixed_boxes[state] and key not in extras else cad.bounds(obj)
            if not all(min(b[i+3],bb[i+3])-max(b[i],bb[i])>.001 for i in range(3)):continue
            v=cad.intersection(piece,obj)
            if v>.025:
                if key.startswith("rail-") and (d["exit"] is None or t<d["exit"][0]):
                    intended.add(key)
                else:hits[key].append((stamp,v))
    return {"samplesRequested":len(set(times)),"planarOutcome":d["outcome"],"intendedContact":sorted(intended),
            "otherContacts":{k:{"maxMm3":max(v for _,v in a),"firstS":a[0][0]} for k,a in hits.items()}}


def build(field_path):
    baseline=json.loads((HERE/"native.json").read_bytes())
    retired=set(baseline["retiredIds"])
    local=local_shapes()
    fixed={key:{k:s for k,s in assembly.pose_shapes(h,x,"red").items() if k not in retired}
           for key,h,x in (("receive",940,270),("transport",670,0),("place",560,370))}
    field=parts(field_path)
    boxes={k:cad.bounds(v) for k,v in field.items()}
    adds=placed(local,940,270,0)
    bounded=bounded_dock({k:cad.bounds(s) for k,s in adds.items()},boxes,40)
    touch=[native_dock(adds,field,boxes,x,yaw,side) for x,yaw,side in ((40,0,0),(30,3,10),(30,-3,-10))]
    cases={}
    for name,cfg in (("nominal",BASE),("left-long",replace(BASE,length=.307975,yaw=4,normal=.01,lateral=.01)),
                     ("right-short",replace(BASE,length=.295275,yaw=-4,normal=-.01,lateral=-.01))):
        cases[name]=audit(cfg,fixed,local)
    return {"fieldSha256":FIRST_SHA,"nativeSha256":sha(HERE/"cradle.step"),"registeredStationBodies":len(field),
            "fieldSweep":bounded,"fieldSample":touch,"receiverSamples":cases}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--field",type=Path,required=True);p.add_argument("--output",type=Path,required=True)
    a=p.parse_args();a.output.write_text(json.dumps(build(a.field),indent=2)+"\n",encoding="utf-8",newline="\n")
