"""Native station, reef and finite-pipe clearance for the new raised shelf members."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
from mount import HERE,ROOT,cad,parts,limits
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/"station-supply"))
from native_station import parts as station_parts,FIRST_SHA
from dock_tolerance import moved
from mission.check_branch_exclusion import direct_children,name
from mission.field_body_clearance import cylinder_sweep
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.gp import gp_Trsf

def reef_parts(step):
    doc,table=cad.xcaf_doc()
    reader=cad.STEPCAFControl_Reader()
    if reader.ReadFile(str(step))!=cad.IFSelect_RetDone or not reader.Transfer(doc):raise ValueError("field load")
    roots=cad.Sequence_TDF_Label()
    table.GetFreeShapes(roots)
    parent,ref=next((a,b) for a,b in direct_children(table,roots.Value(1)) if name(a)=="NAUO36" and name(b)=="GE-25300_Red")
    return {name(a)+"/"+name(b):table.GetShape_s(b).Moved(table.GetLocation_s(parent).Multiplied(table.GetLocation_s(a)))
            for a,b in direct_children(table,ref)}

def find_min(obstacles,kit):
    boxes={k:cad.bounds(s) for k,s in obstacles.items()}
    minimum=math.inf
    witness=None
    for n,body in kit.items():
        b=cad.bounds(body)
        for k in sorted(obstacles,key=lambda k:limits(b,boxes[k])):
            if limits(b,boxes[k])>minimum:break
            dist=BRepExtrema_DistShapeShape(body,obstacles[k])
            if not dist.IsDone():raise ValueError("native distance failure")
            if dist.Value()<minimum:
                minimum=dist.Value()
                witness=[n,k]
    return {"minimum_mm":minimum,"pair":witness}

def run(step,scratch):
    if hashlib.sha256(step.read_bytes()).hexdigest()!=FIRST_SHA:raise ValueError("registered field hash required")
    kit,_,_=parts()
    result={"official_field_sha256":FIRST_SHA,"station":find_min(station_parts(step),moved(kit,444.525,0,0)),
            "reef":{},"incoming_pipe":{}}
    reef=reef_parts(step)
    for sector in (-180,220):
        xf=gp_Trsf()
        # Robot X points into red reef, +Y file-sector, +Z up. Native frame uses -Z for field +Y.
        xf.SetValues(1,0,0,3453.050980729168-490,0,0,1,0,0,-1,0,-sector)
        swept={}
        for n,shape in kit.items():
            b=cad.bounds(shape)
            block=cad.brep(["box",[*b[:3],b[3]-b[0]+370,b[4]-b[1],b[5]-b[2]]])
            swept[n]=BRepBuilderAPI_Transform(block,xf,False).Shape()
        result["reef"][str(sector)]=find_min(reef,swept)
    sys.path.insert(0,str(ROOT/"full-match"))
    from installed_stock import field_stock,corrected,stationary,check
    active=corrected()
    stock,native_stock=field_stock(step)
    travel={k:list(cad.bounds(shape)) for k,shape in kit.items()}
    deployed={k:[*b[:3],b[3]+370,*b[4:]] for k,b in travel.items()}
    result["issued_start_stock"]={}
    for sector in (-.18,.22):
        _,loaded,_=active.routes(sector)
        result["issued_start_stock"][str(sector)]={
            "carried_route":check(active,stock,travel,loaded.segments),
            "target_extension":check(active,stock,deployed,stationary(loaded.end))}
    old_kit,_,_=parts(raised=True)
    old_plate=old_kit["top-plate"]
    b=cad.bounds(old_plate)
    old_sweep={"top-plate":[*b[:3],b[3]+370,*b[4:]]}
    old_gap=check(active,stock,old_sweep,stationary(active.routes(-.18)[1].end))["reef-ball"]["lower_bound_mm"]
    ball=native_stock["NAUO201"]
    old_xf=gp_Trsf()
    old_xf.SetValues(1,0,0,3453.050980729168-490+370,0,0,1,0,0,-1,0,180)
    installed=BRepBuilderAPI_Transform(old_plate,old_xf,False).Shape()
    result["rejected_raised_plate"]={"added_optical_origin_mm":[115,0,860],"sector_mm":-180,
        "outer_ball_envelope_mm":old_gap,"native_intersection_mm3":cad.intersection(installed,ball),
        "actual_native_gap_mm":BRepExtrema_DistShapeShape(installed,ball).Value()}
    for key in ("coral-A","coral-B"):
        trace=np.load(scratch/f"{key}-trajectory.npy")
        # This incoming path is transverse with no roll-axis yaw. Broadphase at every 4th sample
        # encloses all intermediate centers by min/max positions, including the last sub-window.
        trace=trace[trace[:,0]<=1700]
        minimum=math.inf
        witness=None
        for k,shape in kit.items():
            b=cad.bounds(shape)
            for start in range(0,len(trace),32):
                segment=trace[start:start+33,1:4]
                gap=cylinder_sweep(segment.min(axis=0),segment.max(axis=0),b,1.0)
                if gap<minimum:
                    minimum=gap
                    witness=[k,float(trace[start,0])]
        result["incoming_pipe"][key]={"lower_bound_mm":minimum,"body_and_interval_start_ms":witness}
    (HERE/"clearance.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--field-step",type=Path,required=True)
    p.add_argument("--scratch",type=Path,required=True)
    a=p.parse_args()
    run(a.field_step,a.scratch)
