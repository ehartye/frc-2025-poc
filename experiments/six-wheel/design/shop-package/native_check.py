"""Reconcile drawing schedules and two proposed fastener joints with the archived robot."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

from profiles import profiles

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT/"design/station-l1"))
sys.path.insert(0,str(ROOT/"design/receiver-lift"))
import build_assembly as cad
from mechanical import box,cylinder,cut
from check_routing import clashes
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge,BRepBuilderAPI_MakeFace,BRepBuilderAPI_MakeWire,BRepBuilderAPI_Transform
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.GC import GC_MakeArcOfCircle
from OCP.gp import gp_Pnt,gp_Vec,gp_Trsf

PACKAGE=ROOT/"design/competition-package"
STOWED_SHA="6ab8b98d5f5a196809a87f299279b8f8c01a06384d6c39dcda1f51423f865990"


def native_inputs():
    src=PACKAGE/"poses/stowed.step"
    if hashlib.sha256(src.read_bytes()).hexdigest()!=STOWED_SHA:
        raise ValueError("This shop revision requires its 401-body source STEP. Use the recorded commit for the historic build.")
    ids=json.loads((PACKAGE/"part-frames.json").read_text())
    return cad.read_named(src,[{"id":k} for k in ids])


def local(shape,view):
    tr=gp_Trsf()
    if view=="XZ":
        tr.SetValues(1,0,0,0,0,0,1,0,0,-1,0,0)
    elif view=="YZ":
        tr.SetValues(0,1,0,0,0,0,1,0,1,0,0,0)
    body=BRepBuilderAPI_Transform(shape,tr,True).Shape()
    return cad.translated(body,[-v for v in cad.bounds(body)[:3]])


def subtract(shape,tool):
    op=BRepAlgoAPI_Cut(shape,tool)
    if not op.IsDone():
        raise ValueError("Native cut failed")
    return op.Shape()


def blank(spec):
    length,width,height=spec.size
    if spec.radius:
        # U/T section of the rounded leading edge, then extrude uniformly across V.
        r=spec.radius
        pts=[gp_Pnt(x,0,z) for x,z in [(0,0),(length,0),(length,height-r)]]
        rounded=gp_Pnt(length-r,0,height)
        mid=gp_Pnt(length-r+r/math.sqrt(2),0,height-r+r/math.sqrt(2))
        edges=[BRepBuilderAPI_MakeEdge(a,b).Edge() for a,b in zip(pts,pts[1:])]
        edges.extend([BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(pts[-1],mid,rounded).Value()).Edge(),
                      BRepBuilderAPI_MakeEdge(rounded,gp_Pnt(0,0,height)).Edge(),
                      BRepBuilderAPI_MakeEdge(gp_Pnt(0,0,height),pts[0]).Edge()])
        w=BRepBuilderAPI_MakeWire()
        for e in edges:w.Add(e)
        return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(w.Wire()).Face(),gp_Vec(0,width,0)).Shape()
    shape=box(0,0,0,length,width,height)
    if spec.wall:
        t=spec.wall
        shape=cut(shape,box(-1,t,t,length+2,width-2*t,height-2*t))
    return cad.brep(shape)


def machine(spec):
    shape=blank(spec)
    for item in spec.holes:
        start=spec.size[2]-item.depth if item.depth else -1
        height=spec.size[2]+1-start
        radius=item.diameter/2
        def bore(x):
            return cylinder(x,item.v,start,[0,0,1],radius,height)
        tool=bore(item.u)
        if item.travel:
            left=item.u-item.travel/2
            tool=["union",[bore(left),bore(left+item.travel),box(left,item.v-radius,start,item.travel,item.diameter,height)]]
        shape=subtract(shape,cad.brep(tool))
    return shape


def discrepancy(a,b):
    common=cad.intersection(a,b)
    return max(0,cad.volume(a)[0]-common)+max(0,cad.volume(b)[0]-common)


def write_shapes(path,shapes):
    labels=[{"id":k,"group":"shaft"} for k in shapes]
    cad.write_named(path,list(shapes.values()),labels)
    restored=cad.read_named(path,labels)
    if any(discrepancy(v,restored[k])>.01 for k,v in shapes.items()):
        raise ValueError("STEP round-trip changed a joint or part body")


def outer_joint(x,side):
    def block(xx,zz,tt,u,v,t):
        return box(xx,tt if side>0 else -tt-t,zz,u,t,v)
    def shaft(xx,zz,t0,t1,rad):
        return cylinder(xx,side*t0,zz,[0,side,0],rad,t1-t0)
    # Rest the web on the tube floor. Lower center clearance retains access to the axle end.
    parts={}
    saddle=cut(block(x-25,52,327,50,48.4,8),block(x-13,55,326,26,47,10))
    for col in [-19,19]:
        for dz in [-17,17]:
            xx,zz=x+col,76.2+dz
            label=f"{col}/{dz}"
            saddle=cut(saddle,shaft(xx,zz,326,336,2.5))
            parts["washer "+label]=cad.brep(cut(shaft(xx,zz,318,319,5),shaft(xx,zz,317,320,2.65)))
            parts["screw "+label]=cad.brep(["union",[shaft(xx,zz,313,318,4.25),shaft(xx,zz,318,334,2.5)]])
    parts["nut saddle"]=cad.brep(saddle)
    return parts


def clamp_access(x,side,column,row):
    # A 4 mm AF key fits a 4.62 mm enclosing cylinder. Swing a short nib outwards
    # from vertical through 60 degrees; this box includes the entire 60 mm handle arc.
    xx,zz=x+column,76.2+row
    lower,upper=sorted((xx,xx+(1 if column>0 else -1)*60*math.sin(math.pi/3)))
    half=2.31
    t=310.60
    y0=min(side*(t-half),side*(t+half))
    compact={"key-swing":cad.brep(box(lower-half,y0,zz-half,upper-lower+2*half,2*half,60+2*half)),
             "short-nib":cad.brep(cylinder(xx,side*t,zz,[0,side,0],half,5.3))}
    deep=cad.brep(cylinder(xx,side*284,zz,[0,side,0],5,29))
    return compact,deep


def receiver_joint(source,side):
    cy=side*180
    lr="L" if side==1 else "R"
    railkey="slide-"+lr+"-inner"
    rail=source[railkey]
    zlift=940
    new={}
    originals=[railkey]
    for x in [-80,140]:
        key=f"anchor-block-{lr}-{x}"
        originals.append(key)
        # One long screw enters each block by 6 mm. Nominal mating thread is suppressed.
        body=cad.brep(cut(box(x-8,cy-7,zlift-88,16,14,7),cylinder(x,cy,zlift-89,[0,0,1],2.5,9)))
        for dx in [-5,5]:
            z=zlift-84.5
            bore=cad.brep(cylinder(x+dx,cy-11,z,[0,1,0],1.55,22))
            rail=subtract(rail,bore)
            pin=cad.brep(cylinder(x+dx,cy-10,z,[0,1,0],1.5,20))
            body=subtract(body,pin)
            new[f"pin-{lr}-{x}-{dx}"]=pin
        new["tapped-"+key]=body
    new["drilled-"+railkey]=rail
    return new, originals


def evaluate(out,export):
    src=native_inputs()
    receipt={"sourceCommit":"3fc3f27b716546dc92a34c9c6dda8c99fd5b8ff4","stowedSha256":STOWED_SHA,
             "drawingMismatchMm3":{},"outerClamps":{},"receiverCrossblocks":{}}
    for spec in profiles():
        candidate=machine(spec)
        errs={key:round(discrepancy(candidate,local(src[key],spec.view)),6) for key in spec.ids}
        receipt["drawingMismatchMm3"][spec.key]=errs
        if max(errs.values())>.01:raise ValueError((spec.key,errs))
        if export:write_shapes(out/(spec.key+".step"),{spec.key:candidate})
    groups={}
    mass_tracks={"base":[0.,0.,0.,0.],"slide":[0.,0.,0.,0.]}
    def add_mass(frame,shape,density):
        volume,center=cad.volume(shape)
        mass=volume*density
        for i,v in enumerate([1,*center]):
            mass_tracks[frame][i]+=mass*v
    for side,lr in [(1,"L"),(-1,"R")]:
        for nominal,station in [(-247.65,"rear"),(0,"center"),(247.65,"front")]:
            for dx in ([0] if station=="center" else [-4,0,4]):
                joint=outer_joint(nominal+dx,side)
                others=src.copy()
                # Wheel/hub, spacers, both bearings and their carriers move together at take-up.
                prefixes=tuple(f"{kind}-{station}-" for kind in ["carrier","wheel","hub","bearing","axle","spacer"])
                for key,q in src.items():
                    if dx and key.startswith(prefixes) and key.endswith("-"+lr):
                        others[key]=cad.translated(q,(dx,0,0))
                overlaps=clashes(joint,others)
                for i,(name,q) in enumerate(joint.items()):
                    overlaps.update(clashes({name:q},dict(list(joint.items())[:i])))
                corridor=cad.brep(cylinder(nominal+dx,side*327,76.2,[0,side,0],8.5,24))
                blocked=clashes({"end-retention tool":corridor},joint)
                receipt["outerClamps"][f"{station}-{lr}-{dx}"]={"overlapMm3":overlaps,"axialToolMm3":blocked}
                if overlaps or blocked:raise ValueError((station,lr,dx,overlaps,blocked))
                access={}
                for col in [-19,19]:
                    for row in [-17,17]:
                        tag=f"{col}/{row}"
                        tools,deep=clamp_access(nominal+dx,side,col,row)
                        other=others|{k:v for k,v in joint.items() if k!="screw "+tag}
                        tool_hit=clashes(tools,other)
                        if tool_hit:raise ValueError(("clamp-tool",station,lr,dx,tag,tool_hit))
                        blocked_wheel=clashes({"ordinary long tool":deep},{"wheel":others[f"wheel-{station}-{lr}"]})
                        access[tag]=round(sum(blocked_wheel.values()),2)
                receipt["outerClamps"][f"{station}-{lr}-{dx}"]["ordinaryToolWheelOverlapMm3"]=access
                if dx==0:
                    for q in joint.values():add_mass("base",q,7.85e-6)
                    groups.update({f"{station}-{lr}-"+k.replace("/","_").replace(" ","-"):q for k,q in joint.items()})
        new,old=receiver_joint(src,side)
        reference={k:v for k,v in src.items() if k not in old}
        hit=clashes(new,reference)
        for i,(k,q) in enumerate(new.items()):hit.update(clashes({k:q},dict(list(new.items())[:i])))
        pins=[v for k,v in new.items() if k.startswith("pin-")]
        old_solid=[src[k] for k in old]
        outside=[round(max(0,cad.volume(pin)[0]-sum(cad.intersection(pin,o) for o in old_solid)),6) for pin in pins]
        receipt["receiverCrossblocks"][lr]={"collisionMm3":hit,"pinVolumeOutsidePreviousMaterialMm3":outside}
        if hit or max(outside)>.01:raise ValueError((lr,hit,outside))
        groups.update(new)
        for k,v in new.items():add_mass("slide",v,2.70e-6 if k.startswith("drilled-") else 7.85e-6)
        for k in old:add_mass("slide",src[k],-2.70e-6)
    candidate_mass=sum(v[0] for v in mass_tracks.values())
    receipt["incrementalMassByFrame"]={k:{"deltaKg":round(v[0],6),"stowedCgMm":[round(a/v[0],4) for a in v[1:]]}
                                      for k,v in mass_tracks.items()}
    receipt["addedNominalKg"]=round(candidate_mass,4)
    if export:write_shapes(out/"proposed-joints-in-stowed-frame.step",groups)
    out.mkdir(parents=True,exist_ok=True)
    (out/"native-review.json").write_text(json.dumps(receipt,indent=2)+"\n",newline="\n")
    print(f"Matched {sum(len(v) for v in receipt['drawingMismatchMm3'].values())} instances; incremental mass {candidate_mass:.4f} kg.")


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output",type=Path)
    parser.add_argument("--export",action="store_true")
    args=parser.parse_args()
    args.output.mkdir(exist_ok=True,parents=True)
    evaluate(args.output,args.export)
