"""Propagate the project tracking, slide and registration envelopes against occupied field geometry."""
import argparse
import json
import math
import sys
from pathlib import Path
from design import HERE,ROOT,mount,parts
from replay_candidate import model,sha,mod
sys.path.insert(0,str(ROOT/"full-match"))
from installed_stock import field_stock,corrected,coordinate,check,stationary,BALL_RADIUS,box_gaps
from OCP.gp import gp_Trsf
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepExtrema import BRepExtrema_DistShapeShape

ANGLE=1.0
TRAVEL=10.0
STROKE=15.0
MOUNT=1.0
cad=mount.cad
load=lambda p:json.loads(p.read_bytes())

def heading_sweep(box,angle):
    """Enclose every heading in the closed interval about the chassis origin."""
    xs=[]
    ys=[]
    a=math.radians(angle)
    for x in (box[0],box[3]):
        for y in (box[1],box[4]):
            radius=math.hypot(x,y)
            origin=math.atan2(y,x)
            limits=[origin-a,origin+a]
            limits.extend(k*math.pi/2 for k in range(-4,5) if origin-a<=k*math.pi/2<=origin+a)
            xs.extend(radius*math.cos(t) for t in limits)
            ys.extend(radius*math.sin(t) for t in limits)
    return [min(xs),min(ys),box[2],max(xs),max(ys),box[5]]

def robust(box,point,flex,shift=0,angle=ANGLE,translation=TRAVEL):
    box=heading_sweep([box[0]-STROKE+shift,box[1],box[2],box[3]+370+STROKE+shift,box[4],box[5]],angle)
    dx,dy,dz=box_gaps(point,box)
    h=max(0,math.hypot(dx,dy)-translation)
    return math.hypot(h,dz)-BALL_RADIUS-MOUNT-flex

def native_pose(kit,sector):
    xf=gp_Trsf()
    xf.SetValues(1,0,0,3453.050980729168-490,0,0,1,0,0,-1,0,-sector)
    return {k:BRepBuilderAPI_Transform(v,xf,False).Shape() for k,v in kit.items()}

def run(field,lamp,scratch):
    # Use the original red surveyed frames for both robot variants.
    source=mod("roof_clearance",ROOT/"moving-observer/clearance.py")
    f=corrected()
    stock,native=field_stock(field)
    reef=source.reef_parts(field)
    flex=load(ROOT/"moving-observer/checks.json")["two_g_ideal_post_tip_mm"]
    kit=parts()[0]
    baseline=mount.parts()[0]
    rows={}
    for sec in (-.18,.22):
        pose=f.routes(sec)[1].end
        points={k:coordinate(f,v,pose) for k,v in stock.items() if v["group"]=="reef-ball"}
        result={}
        for kind,group in (("issued",baseline),("candidate",kit)):
            boxes={k:list(cad.bounds(v)) for k,v in group.items()}
            def nearest(yaw,translation):
                return min((robust(b,p,flex,angle=yaw,translation=translation),k,key) for key,p in points.items() for k,b in boxes.items())
            gap,k,occ=nearest(ANGLE,TRAVEL)
            result[kind]={"robust_ball_mm":gap,"body":k,"occurrence":occ,"translation_20_mm":nearest(ANGLE,20)[0],
                "translation_35_mm":nearest(ANGLE,35)[0],"nominal_ball_mm":check(f,stock,{k:[*b[:3],b[3]+370,*b[4:]] for k,b in boxes.items()},stationary(pose))["reef-ball"]["lower_bound_mm"]}
            member=group["top-tube"]
            displaced=cad.translated(member,[370+15+10,0,0])
            native_member=native_pose({"tube":displaced},sec*1000)["tube"]
            result[kind]["native_25_mm_forward_control"]={"intersection_mm3":cad.intersection(native_member,native["NAUO201"]),
                "gap_mm":BRepExtrema_DistShapeShape(native_member,native["NAUO201"]).Value()}
        sweep={k:heading_sweep([b[0]-STROKE,b[1],b[2],b[3]+370+STROKE,b[4],b[5]],ANGLE) for k,v in kit.items() for b in [cad.bounds(v)]}
        shapes={k:cad.brep(["box",[*b[:3],*(b[i+3]-b[i] for i in range(3))]]) for k,b in sweep.items()}
        field_min=source.find_min(reef,native_pose(shapes,sec*1000))
        result["reef_structure"]={**field_min,"after_translation_mount_flex_mm":field_min["minimum_mm"]-TRAVEL-MOUNT-flex}
        travel={k:list(cad.bounds(v)) for k,v in kit.items()}
        result["route_added_bodies"]=check(f,stock,travel,f.routes(sec)[1].segments)
        rows[str(sec)]=result
    full=model(lamp)
    frames=load(ROOT/"design/competition-package/shop-duplex/red/part-frames.json")
    posed={}
    rotating=load(ROOT/"design/competition-package/shop-duplex/red/rotation-bounds.json")
    for k,v in full.items():
        frame=frames.get(k,"slide" if k.startswith("roof-") or k=="receiver-R6-drilled-for-camera" else "base")
        factor=1 if frame=="slide" else .5 if frame=="middle" else 0
        bounds=cad.bounds(v)
        if k in rotating:
            b=rotating[k]["bounds_mm"]
            bounds=[b[0]+370*factor,b[1],b[2]+560,b[3]+370*factor,b[4],b[5]+560]
        # At fixed height the vertical return line is constant. The 370 mm pose has the
        # longest horizontal draw wire; its free end can extend 15 mm farther.
        if k=="slide-position-line":
            bounds=list(bounds)
            bounds[3]+=STROKE
        grip=1.5 if k.startswith(("grip-L-","grip-R-")) else 0.
        posed[k]=heading_sweep([bounds[0]-(370+STROKE)*factor,bounds[1]-grip,bounds[2],
                               bounds[3]+STROKE*factor,bounds[4]+grip,bounds[5]],ANGLE)
    for sector,row in rows.items():
        pose=f.routes(float(sector))[1].end
        row["whole_target_stock"]=check(f,stock,posed,stationary(pose))
        for rec in row["whole_target_stock"].values():rec["after_translation_and_camera_allowance_mm"]=rec["lower_bound_mm"]-TRAVEL-MOUNT-flex
    neighbor=load(ROOT/"fixed-height-match/native.json")["firstTubeToSecondDockMm"]["inflatedTubeBoxMm"]
    gap,part=min((mount.limits(b,neighbor),k) for k,b in posed.items())
    rows["0.22"]["settled_first_pipe"]={"inflated_prior_bounds_mm":neighbor,"body":part,
            "lower_mm":gap-TRAVEL-MOUNT-flex}
    from native_station import parts as station_parts
    from dock_tolerance import moved
    station=source.find_min(station_parts(field),moved(kit,444.525,0,0))
    maximum=max(math.hypot(x,y) for v in kit.values() for b in [cad.bounds(v)] for x in (b[0],b[3]) for y in (b[1],b[4]))
    station["after_translation_heading_stroke_mount_flex_mm"]=station["minimum_mm"]-TRAVEL-2*(maximum+STROKE)*math.sin(math.radians(ANGLE)/2)-STROKE-MOUNT-flex
    import numpy as np
    from mission.field_body_clearance import cylinder_sweep
    incoming={}
    for piece in ("coral-A","coral-B"):
        tr=np.load(scratch/f"{piece}-trajectory.npy");tr=tr[tr[:,0]<=1700]
        incoming[piece]=min(cylinder_sweep(block.min(axis=0),block.max(axis=0),cad.bounds(v),1) for v in kit.values() for n in range(0,len(tr),32) for block in [tr[n:n+33,1:4]])
    out={"field_sha256":sha(field),"combined_register_sha256":sha(ROOT/"design/fixed-no-air/whole-robot.json"),"collision_bodies":len(full),
        "translation_disk_mm":TRAVEL,"chassis_yaw_deg":ANGLE,"slide_endpoint_error_mm":STROKE,"bridge_registration_mm":MOUNT,
        "ideal_flex_2g_mm":flex,"bounding_ball_radius_mm":BALL_RADIUS,"station_added_hardware":station,"incoming_pipe_lower_mm":incoming,"sectors":rows}
    (HERE/"field.json").write_text(json.dumps(out,indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for n in ("field","lamp","scratch"):p.add_argument("--"+n,required=True,type=Path)
    a=p.parse_args();run(a.field,a.lamp,a.scratch)
