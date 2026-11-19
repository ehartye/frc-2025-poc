"""Reopen the issued stowed robot and a positive fixed-height, retracted slide lock."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(HERE.parent/"lift-arrest"))
from end_stop import cad, contact, digest, shared
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.gp import gp_Ax1,gp_Dir,gp_Pnt,gp_Trsf
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform

HEIGHT=940
# Unit vectors and positions are all local to the chassis, +X forward.
def x_cyl(x,y,z,r,length):
    return ["cyl",[x,y,z,1,0,0,r,length]]


def y_cyl(x,y,z,r,length):
    return ["cyl",[x,y,z,0,1,0,r,length]]


def ring(outer,inner):
    return cad.brep(["cut",[outer,inner]])


def replace_cut(container,name,tool):
    result=BRepAlgoAPI_Cut(container[name],cad.brep(tool))
    if not result.IsDone():
        raise ValueError("Failed machining "+name)
    container[name]=result.Shape()


def hardware(source,bad=False,height=None):
    height=HEIGHT if height is None else height
    modified={}
    added={}
    for side,tag,shoe_name in ((1,"L","lift-shoe-210"),(-1,"R","lift-shoe--235")):
        mast=f"backup-{tag}-drilled-mast"
        cross="lift-crossbar-drilled"
        for name in (mast,cross,shoe_name):
            modified.setdefault(name,source[name])
        y=side*(214 if bad else 222.5)
        z=height-137.5
        key="height-"+tag+"-"
        replace_cut(modified,mast,x_cyl(-71,y,z,6.05,52))
        replace_cut(modified,cross,x_cyl(-99,y,z,4.1,24))
        replace_cut(modified,shoe_name,x_cyl(-78,y,z,6.05,9))
        replace_cut(modified,shoe_name,x_cyl(-21,y,z,9.5,8))
        added[key+"mast-sleeve"]=ring(x_cyl(-70,y,z,6,50),x_cyl(-71,y,z,4.1,52))
        added[key+"shoe-stand-off"]=ring(x_cyl(-77,y,z,6,7),x_cyl(-78,y,z,4.1,9))
        # Smooth 8 mm shoulder at both load interfaces; end threads represented as a full-diameter reserve.
        added[key+"pin"]=cad.brep(["union",[x_cyl(-107,y,z,6.5,8),x_cyl(-99,y,z,4,91)]])
        for label,x in (("rear",-99),("front",-20)):
            added[key+label+"-washer"]=ring(x_cyl(x,y,z,8,2),x_cyl(x-1,y,z,4.2,4))
        added[key+"locking-nut"]=ring(x_cyl(-18,y,z,7.5,8),x_cyl(-19,y,z,4.02,10))
        # Four independent vertical bolts join the carriage crossbar and the base of the slide.
        bridge="slide-base-bridge"
        modified.setdefault(bridge,source[bridge])
        for j,cy0 in enumerate((100 if bad else 128,145),1):
            cy0*=side
            key=f"crossbar-{tag}-{j}-"
            def axial(z0,r,length):
                return ["cyl",[-87,cy0,height+z0,0,0,1,r,length]]
            for k in (cross,bridge):
                replace_cut(modified,k,axial(-150,3.3,31))
            added[key+"screw"]=cad.brep(["union",[axial(-155.6,5,6),axial(-149.6,3,40)]])
            for pos,z0 in (("bottom",-149.6),("top",-122)):
                added[key+pos+"-washer"]=ring(axial(z0,6,1.6),axial(z0-1,3.2,4))
            added[key+"locking-nut"]=ring(axial(-120.4,5.8,6.5),axial(-121.4,3.02,9))
        # Fillets join each fixed outer rail to its base. The solid is a bead envelope;
        # fusion, HAZ strength, process qualification and distortion require a fixture.
        from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon,BRepBuilderAPI_MakeFace
        from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
        from OCP.gp import gp_Pnt,gp_Vec
        for sign in (-1,1):
            y0=180*side+sign*18
            wz=height-122
            wire=BRepBuilderAPI_MakePolygon()
            for yy,zz in ((y0,wz),(y0+sign*3,wz),(y0,wz+3)):
                wire.Add(gp_Pnt(-50,yy,zz))
            wire.Close()
            added[f"rail-{tag}-fillet-{sign}"]=BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(wire.Wire()).Face(),gp_Vec(100,0,0)).Shape()
        if height!=HEIGHT:
            continue
        # Retain all three nested stages; the grip force comes only from the positive through-shank.
        x,z=(-80 if bad else 45),HEIGHT-99
        cy=180*side
        channels=[f"slide-{tag}-outer",f"slide-{tag}-middle",f"drilled-slide-{tag}-inner"]
        for name in channels:
            modified[name]=source[name]
            replace_cut(modified,name,y_cyl(x,cy-20,z,3.1,40))
        key="stroke-"+tag+"-"
        # Symmetric wire-retained clevis pin. The cotter is a packaging reserve with its own cross-hole.
        end=cy+18
        pin=cad.brep(["union",[y_cyl(x,cy-22,z,5,4),y_cyl(x,cy-18,z,3,42)]])
        keeper=["cyl",[x-5,end+4,z,1,0,0,.9,10]]
        hole=BRepAlgoAPI_Cut(pin,cad.brep(keeper))
        added[key+"pin"]=hole.Shape()
        added[key+"washer"]=ring(y_cyl(x,end,z,6,1.6),y_cyl(x,end-1,z,3.2,4))
        added[key+"formed-keeper"]=cad.brep(["union",[
            ["cyl",[x-5,end+4,z,1,0,0,.8,9.6]],
            ["cyl",[x-5.4,end+4,z,1,0,0,1.7,1.2]],
            ["cyl",[x+3.9,end+4,z-.2,0,0,1,.7,3.3]]]])
    return modified,added


def run(color,bad=False,step=None,low=None):
    folder=ROOT/"design/competition-package/shop-duplex"/color
    names=json.loads((folder/"part-frames.json").read_bytes())
    pose="place" if low is not None else "stowed"
    path=folder/(pose+".step")
    assert digest(path)==json.loads((folder/"review.json").read_bytes())[pose]["export_sha256"]
    original=cad.read_named(path,[{"id":k} for k in names])
    if low=="receive":
        sys.path.insert(0,str(ROOT/"design/competition-package/shop-duplex"))
        from assembly import pose_shapes
        original=pose_shapes(560,0,color)
    modified,added=hardware(original,bad,560 if low is not None else HEIGHT)
    assembly={**original,**modified}
    bounds={k:cad.bounds(v) for k,v in assembly.items()}
    hits={}
    for name,shape in added.items():
        if h:=contact(shape,assembly,bounds):
            hits[name]=h
        assembly[name]=shape
        bounds[name]=cad.bounds(shape)
    delta={k:cad.volume(v)[0]-cad.volume(original[k])[0] for k,v in modified.items()}
    info={"pose":low or "stowed","nativeInputPose":pose,"heightMm":560 if low is not None else HEIGHT,"revision":digest(path),"nativeIds":len(original),"changeCount":len(modified),
          "newCount":len(added),"installedIntersectionsMm3":hits,
          "editedVolumesMm3":delta,"addedVolumesMm3":{k:cad.volume(v)[0] for k,v in added.items()},
          "overallBoxMm":[(min if axis<3 else max)(b[axis] for b in bounds.values()) for axis in range(6)],
          "lockBoxMm":[(min if axis<3 else max)(bounds[k][axis] for k in added) for axis in range(6)]}
    # Positive-volume counterfactuals establish mechanical load transfer with actuators absent.
    witnesses={}
    for tag in ("L","R"):
        pin=added["height-"+tag+"-pin"]
        down=cad.translated(modified["lift-crossbar-drilled"],[0,0,-1])
        witnesses[tag+"-down1mm-crossbar-pin"]=shared(pin,down)
        witnesses[tag+"-down1mm-pin-mast-sleeve"]=shared(cad.translated(pin,[0,0,-1]),added["height-"+tag+"-mast-sleeve"])
        witnesses[tag+"-down1mm-sleeve-mast"]=shared(cad.translated(added["height-"+tag+"-mast-sleeve"],[0,0,-1]),modified["backup-"+tag+"-drilled-mast"])
        if low is None:
            witnesses[tag+"-slide1mm-pin"]=shared(cad.translated(modified["drilled-slide-"+tag+"-inner"],[1,0,0]),added["stroke-"+tag+"-pin"])
    info["blockedStepsMm3"]=witnesses
    if low=="receive":
        # Native start/end swept boxes cover continuously translating slide and midstage.
        # Rotating pinions keep the manufacturer's project 11 mm radial bound.
        far={}
        for k,v in original.items():
            fr=names[k]
            if fr not in ("middle","slide") and k not in ("slide-position-line","slide-power-pinion"):
                continue
            box=list(cad.bounds(v))
            travel=185 if fr=="middle" else 370
            if k=="slide-power-pinion":
                travel=0
            if k in ("slide-power-pinion","slide-fixed-rack-pinion","slide-inner-rack-pinion"):
                box[0]-=2;box[3]+=2;box[2]-=2;box[5]+=2
            # The draw wire extends in +X. Include the full-length possible wire envelope.
            box[3]+=travel
            far[k]=box
        unresolved={}
        for name in added:
            a=bounds[name]
            for k,b in far.items():
                spans=[min(a[i+3],b[i+3])-max(a[i],b[i]) for i in range(3)]
                if all(d>1e-5 for d in spans):
                    unresolved.setdefault(name,[]).append(k)
        info["continuousSlide"]={"movingSourceBodies":len(far),"strokeMm":[0,370],"overlappingSweptBoxes":unresolved}

    if step:
        sys.path.insert(0,str(HERE.parent/"receiver-lift"))
        from check_routing import write_step
        from duplex import finalize_step
        parts={**modified,**added}
        write_step(step,parts)
        finalize_step(step)
        reopened=cad.read_named(step,[{"id":k} for k in parts])
        info["exportSha256"]=digest(step)
        info["reopenedSolids"]=sum(cad.solid_count(v) for v in reopened.values())
        info["maxReopenVolumeErrorMm3"]=max(abs(cad.volume(v)[0]-cad.volume(reopened[k])[0]) for k,v in parts.items())
    return info


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("color",choices=("red","blue"))
    p.add_argument("output",type=Path)
    p.add_argument("--step",type=Path)
    p.add_argument("--low",choices=("receive","place"))
    p.add_argument("--counterexample",action="store_true")
    args=p.parse_args()
    args.output.write_text(json.dumps(run(args.color,args.counterexample,args.step,args.low),indent=2)+"\n",encoding="utf-8",newline="\n")
