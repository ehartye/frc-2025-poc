"""Evaluate shop-proposed lower-stop mounts against all source-native alliance poses (mm)."""
import argparse
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0,str(HERE.parent/"lift-arrest"))
from end_stop import cad, contact, digest, shapes_at, shared
from sizing import buffer
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Pnt, gp_Vec

FAMILY = ROOT/"design/competition-package/shop-duplex"


def merge_cut(shape, tool):
    op=BRepAlgoAPI_Cut(shape,tool)
    if not op.IsDone():
        raise ValueError("Cut failed")
    return op.Shape()


def disk(side,x,y,z,radius,length):
    return ["cyl",[x,side*y,z,0,side,0,radius,length]]


def tube(outer,inner):
    return cad.brep(["cut",[outer,inner]])


def hex_nut_z(side,z):
    wire=BRepBuilderAPI_MakePolygon()
    for i in range(6):
        a=i*math.pi/3
        wire.Add(gp_Pnt(-60+23/math.sqrt(3)*math.cos(a),side*298+23/math.sqrt(3)*math.sin(a),z))
    wire.Close()
    outer=BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(wire.Wire()).Face(),gp_Vec(0,0,8)).Shape()
    return merge_cut(outer,cad.brep(["cyl",[-60,side*298,z-1,0,0,1,10.02,10]]))


def shapes(height, revised=True):
    old=shapes_at(height)
    out={}
    drills={}
    tools={}
    for side,tag in ((1,"L"),(-1,"R")):
        for suffix in ("bracket-reserve","body-reserve","rod"):
            out[tag+"-"+suffix]=old[tag+"-"+suffix]
        if revised:
            # Open a shallow counter-relief outside the flange. The full 8 mm flange stays intact.
            notch=cad.brep(disk(side,-46,243,344,6.5,8))
            out[tag+"-bracket-reserve"]=merge_cut(out[tag+"-bracket-reserve"],notch)
        for z,which in ((363,"upper"),(339,"lower")):
            out[tag+"-"+which+"-KM20"]=hex_nut_z(side,z)
            # Open-jaw wrench presented from robot +X, slot across Y. Sweep of straight insertion.
            def box(x,y,length,width):
                return ["box",[x,side*298+y,z+1,length,width,6]]
            open_jaw=cad.brep(["union",[box(-66,-19,130,7.4),box(-66,11.6,130,7.4),box(-46,-19,110,38)]])
            for angle in (-15,0,15):
                transform=cad.gp_Trsf()
                axis=cad.gp_Ax1(cad.gp_Pnt(-60,side*298,z),cad.gp_Dir(0,0,1))
                transform.SetRotation(axis,math.radians(angle))
                tools[f"{tag}-{which}-wrench-{angle}"]=cad.BRepBuilderAPI_Transform(open_jaw,transform,True).Shape()
        for z,loc in ((344,"lower"),(373,"upper")):
            x=-46
            key=tag+"-"+loc
            drills[key]=cad.brep(disk(side,x,209,z,5.05,27))
            out[key+"-sleeve"]=tube(disk(side,x,210,z,5,25),disk(side,x,209,z,3.3,27))
            for y,name in ((208.4,"in"),(243,"out")):
                out[key+"-"+name+"-washer"]=tube(disk(side,x,y,z,6,1.6),disk(side,x,y-1,z,3.2,4))
            # A 45 mm screw starts at the washer/head contact plane and projects through its locknut.
            head=["union",[disk(side,x,199.6,z,3,45),disk(side,x,244.6,z,5,6)]]
            key_start=246.6 if side==1 else -252
            out[key+"-screw"]=cad.brep(["cut",[head,["hex",[x,key_start,z,5.0,5.4]]]])
            y=side*201.9 if side==1 else -208.4
            out[key+"-locknut"]=tube(["hex",[x,y,z,10,6.5]],disk(side,x,200.9,z,3.01,9))
            # First 40 mm of the socket admits the screw tail; a 10.2 AF nose seats over the nut.
            nose_start=201.8 if side==1 else -208.5
            tools[key+"-socket"]=cad.brep(["cut",[disk(side,x,161.9,z,8,46.5),disk(side,x,160.9,z,3.3,48),
                                                  ["hex",[x,nose_start,z,10.2,6.7]]]])
            reach=30 if revised else 50
            tip_start=246.7 if side==1 else -250.7
            tools[key+"-key"]=cad.brep(["union",[disk(side,x,250.6,z,3,reach),
                                                 ["hex",[x,tip_start,z,4.9,4]]]])
    return out,drills,tools


def study(color,pose,original=False):
    folder=FAMILY/color
    step=folder/(pose+".step")
    review=json.loads((folder/"review.json").read_bytes())[pose]
    if digest(step)!=review["export_sha256"]:
        raise ValueError("Native bytes differ")
    frames=json.loads((folder/"part-frames.json").read_bytes())
    source=cad.read_named(step,[{"id":name} for name in frames])
    height={"place":560,"transport":670,"receive":940,"stowed":940}[pose]
    hardware,drills,tools=shapes(height,revised=not original)
    removed={}
    uncut={}
    changed={}
    for tag in "LR":
        name=f"backup-{tag}-drilled-mast"
        original=source[name]
        changed[name]=original
        for label in ("lower","upper"):
            key=tag+"-"+label
            uncut[key]=round(shared(hardware[key+"-sleeve"],original),4)
            changed[name]=merge_cut(changed[name],drills[key])
        removed[name]=round(cad.volume(original)[0]-cad.volume(changed[name])[0],5)
        source[name]=changed[name]
    bounds={name:cad.bounds(val) for name,val in source.items()}
    collisions={name:value for name,shape in hardware.items() if (value:=contact(shape,source,bounds))}
    access={name:value for name,shape in tools.items() if (value:=contact(shape,source,bounds))}
    own_bounds={name:cad.bounds(val) for name,val in hardware.items()}
    own={a:{b:v for b,v in hits.items() if a<b} for a,shape in hardware.items()
         if (hits:=contact(shape,{b:v for b,v in hardware.items() if a<b},own_bounds))}
    for key,tool in tools.items():
        # Tool rotates with its mating nut. Their own intended jaw engagement is separate.
        mates={k:v for k,v in hardware.items() if not ("-wrench-" in key and k==key.split("-wrench-")[0]+"-KM20")}
        hits=contact(tool,mates,own_bounds)
        if hits:
            access[key]={**access.get(key,{}),**hits}
    return {"color":color,"pose":pose,"nativeSha256":digest(step),"nativeIds":len(source),
            "materialRemovedMm3":removed,"sleeveOverlapInUndrilledMastMm3":uncut,
            "installedIntersectionsMm3":collisions,"kitInternalIntersectionsMm3":own,
            "toolApproachIntersectionsMm3":access},hardware,changed


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("color",choices=("red","blue"))
    p.add_argument("pose",choices=("place","stowed","receive","transport"))
    p.add_argument("receipt",type=Path)
    p.add_argument("--step",type=Path)
    p.add_argument("--original",action="store_true")
    args=p.parse_args()
    result,hardware,masts=study(args.color,args.pose,args.original)
    if args.step:
        sys.path.insert(0,str(HERE.parent/"receiver-lift"))
        from check_routing import write_step
        from duplex import finalize_step
        assembly={**hardware,**masts}
        write_step(args.step,assembly)
        finalize_step(args.step)
        reopened=cad.read_named(args.step,[{"id":k} for k in assembly])
        if any(cad.solid_count(s)!=1 for s in reopened.values()):
            raise ValueError("Non-solid export")
        result["mountedExport"]={"count":len(reopened),"sha256":digest(args.step),
            "volumeMm3":{name:round(cad.volume(shape)[0],5) for name,shape in reopened.items()},
            "maximumReopenErrorMm3":max(abs(cad.volume(reopened[k])[0]-cad.volume(v)[0]) for k,v in assembly.items())}
    args.receipt.write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8",newline="\n")


if __name__=="__main__":
    main()
