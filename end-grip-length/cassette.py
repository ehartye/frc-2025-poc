"""Rebuild the actuated low receiver, extend its jaw travel and sample real rigid intersections."""
from functools import cache
from pathlib import Path
import json
import math
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"station-supply"))
from entry_window import cad, receiver_drives as mech
from duplex import finalize_step
import assembly
sys.path.insert(0,str(ROOT/"design/fixed-height"))
from native import hardware
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt

def rod(x,y,z,axis,r,length):
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x,y,z),gp_Dir(*axis)),r,length).Shape()

def box(x,y,z,dx,dy,dz,side):
    return mech.block(x,y if side==1 else -y-dy,z,dx,dy,dz)

def touch(a,b,bound_a=None,bound_b=None):
    aa=bound_a if bound_a is not None else cad.bounds(a)
    bb=bound_b if bound_b is not None else cad.bounds(b)
    if any(min(aa[i+3],bb[i+3])-max(aa[i],bb[i])<=.001 for i in range(3)):return 0.
    return cad.intersection(a,b)

@cache
def originals():
    source=assembly.pose_shapes(560,0,"red")
    fixed,adds=hardware(source,height=560)
    source=source|fixed|adds
    cheeks=[{"id":f"grip-{side}-{deck}-{end}"} for side in "LR" for deck in ("upper","lower")
            for end in ("cheek","replaceable-offset-insert")]
    relief=cad.read_named(ROOT/"station-relief/cheeks.step",cheeks)
    source.update({k:cad.translated(v,(-270,0,-380)) for k,v in relief.items()})
    return source

def built(q_left,q_right,variant=True,index=None):
    src=originals();fixed={};moving={}
    for k,v in src.items():
        if not k.startswith("grip-"):fixed[k]=v;continue
        _,side,part=k.split("-",2)
        s=1 if side=="L" else -1
        q=q_left if s==1 else q_right
        if not variant:
            if part=="inboard-hardstop":v=cad.translated(v,(0,s*1.15,0))
            if part.endswith("-spring"):
                x=-73 if "rear" in part else 82
                y=s*(mech.GRIP_Y+10+q)
                shell=rod(x,y,543.5,(0,s,0),3.2,13.25-q)
                hole=rod(x,y-s,543.5,(0,s,0),1.7,15.25-q)
                v=BRepAlgoAPI_Cut(shell,hole).Shape()
        if variant:
            if part=="inboard-hardstop" and index is not None:
                v=box(76,mech.GRIP_Y-3.5,522,4,2+index+5.65,16,s)
            elif part=="outboard-hardstop" and index is not None:
                pad=4.6-index
                v=box(76,mech.GRIP_Y+14.5-pad,522,4,2+pad,16,s)
            if part.endswith("-spring") or part.startswith("dashpot-") or part=="fixed-backbone":continue
            if part=="inboard-hardstop" and index is None:
                v=cad.translated(v,(0,s*(-4.5),0))
            if part=="outboard-hardstop" and index is None:
                v=cad.translated(v,(0,s*4.5,0))
            if part.endswith("slide-guide"):
                x=-73 if "rear" in part else 82
                v=rod(x,s*(mech.GRIP_Y-3),560-16.5,(0,s,0),1.5,46)
            if part.endswith(("riser","support-foot")):
                # Clear the longer pipe; preserve the original shelf holes at Ãƒâ€šÃ‚Â±180.
                tool=box(-120,120,489,290,39.5,39,s)
                v=BRepAlgoAPI_Cut(v,tool).Shape()
        if mech.translates_with_grip(k):moving[k]=cad.translated(v,(0,s*q,0))
        else:fixed[k]=v
    if variant:
        for lr,s,q in (("L",1,q_left),("R",-1,q_right)):
            key="cassette-"+lr+"-"
            # Rear-mounted ribbon reel pushes no part into the approach corridor. Moving reel
            # drives the rear cheek with an alloy bracket; fixed tip anchor is on the shelf foot.
            cy=174+q
            z=534.
            # Two independently rotating drums on a common fixed axle. The ribbon surfaces
            # are thin rectangular loaded-run envelopes; the reserve encloses a full coil.
            for part,x,width,dr,rr,th in (("primary",-118.,12.7,6.64,8.6,.15),
                                          ("trim",-108.,6.35,6.77,7.75,.10)):
                drum=rod(x-width/2,s*cy,z,(1,0,0),dr,width)
                bore=rod(x-width/2-1,s*cy,z,(1,0,0),2.55,width+2)
                moving[key+part+"-drum"]=BRepAlgoAPI_Cut(drum,bore).Shape()
                rim=rod(x-width/2,s*cy,z,(1,0,0),rr,width)
                moving[key+part+"-coil-reserve"]=BRepAlgoAPI_Cut(rim,rod(x-width/2-1,s*cy,z,(1,0,0),dr,width+2)).Shape()
                moving[key+part+"-ribbon"]=box(x-width/2,135,z+rr,width,cy-135,th,s)
            moving[key+"axle"]=rod(-126,s*cy,z,(1,0,0),2.5,25)
            def union(a,b):return cad.BRepAlgoAPI_Fuse(a,b).Shape()
            # Rear arm formed onto the retained lower cheek, ending in a double-shear axle fork.
            ear=box(-104.8,cy-4,z-4,3.2,8,16,s)
            far=box(-127,cy-4,z-4,2.6,8,16,s)
            bridge=box(-127,cy-4,z+9,25.5,8,3,s)
            fork=union(union(ear,far),bridge)
            fork=BRepAlgoAPI_Cut(fork,rod(-128,s*cy,z,(1,0,0),2.55,28)).Shape()
            tongue=box(-102,164+q,543,11,14,3,s)
            cheek=f"grip-{lr}-lower-cheek"
            moving[cheek]=union(moving[cheek],union(fork,tongue))
            # Fixed arm stays below the pickup at x=-90. Clamp ends at y=135.
            bottom=box(-126,159.5,504,36,5,4,s)
            toe=box(-126,130,504,22,30,4,s)
            post=box(-126,130,508,22,5,37,s)
            riser=f"grip-{lr}-rear-riser"
            fixed[riser]=union(fixed[riser],union(union(bottom,toe),post))
    return fixed,moving

def check(q,variant):
    fixed,moving=built(q,q,variant)
    fb={k:cad.bounds(v) for k,v in fixed.items()}
    hits={}
    for k,obj in moving.items():
        own=cad.bounds(obj)
        for other,body in fixed.items():
            v=touch(obj,body,own,fb[other])
            if v>.05:hits[k+" / "+other]=round(v,4)
    return hits

def tube(x,y,z,yaw,length):
    a=math.radians(yaw)
    d=(-math.sin(a),math.cos(a),0)
    p=(x-length/2*d[0],y-length/2*d[1],z)
    return BRepAlgoAPI_Cut(rod(*p,d,57.15,length),rod(*p,d,50.8,length)).Shape()

if __name__=="__main__":
    result={kind:{str(q):check(q,new) for q in (-6.,-2.,0.,2.,6.)}
            for kind,new in (("old-travel",False),("prototype",True))}
    (HERE/"mechanics.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8",newline="\n")
