"""Original moving belt gates, common drive shaft and swing hood on the registered low robot."""
from functools import cache
import hashlib
import json
import math
from pathlib import Path
import sys
from motion import Case,ROOT,HERE,ARM
sys.path.insert(0,str(ROOT/"station-supply"))
from entry_window import cad,receiver_drives as rd
import assembly
from duplex import finalize_step
sys.path.insert(0,str(ROOT/"design/fixed-height"))
from native import hardware
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut,BRepAlgoAPI_Fuse
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax1,gp_Ax2,gp_Dir,gp_Pnt,gp_Trsf

C=Case()
Z=1000*C.bottom
PR=45/math.pi
R=PR+1.2
REAR=1000*C.belt_rear
FRONT=1000*C.belt_front
Z0=Z-R
Z1=Z0+223.5*math.sin(math.radians(6))
AX=(1,0,0);AY=(0,1,0);AZ=(0,0,1)
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def rod(x,y,z,axis,r,length):return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x,y,z),gp_Dir(*axis)),r,length).Shape()
def subtract(a,b):return BRepAlgoAPI_Cut(a,b).Shape()
def fuse(a,b):return BRepAlgoAPI_Fuse(a,b).Shape()
def box(x,y,z,dx,dy,dz):return rd.block(x,y,z,dx,dy,dz)
def ann(x,y,z,axis,ro,ri,length):
    v={"y":AY,"x":AX,"z":AZ}[axis]
    return subtract(rod(x,y,z,v,ro,length),rod(x-v[0],y-v[1],z-v[2],v,ri,length+2))
def rotate(shape,xyz,axis,deg):
    tr=gp_Trsf();tr.SetRotation(gp_Ax1(gp_Pnt(*xyz),gp_Dir(*axis)),math.radians(deg))
    return BRepBuilderAPI_Transform(shape,tr,False).Shape()
def beam(p,q,rad):
    delta=[b-a for a,b in zip(p,q)];l=math.sqrt(sum(v*v for v in delta))
    return rod(*p,tuple(v/l for v in delta),rad,l)
def profile(x1,x2,y,z,r,w):
    points=[(x2+r*math.cos(-math.pi/2+k*math.pi/32),y,z+r*math.sin(-math.pi/2+k*math.pi/32)) for k in range(33)]
    points +=[(x1+r*math.cos(math.pi/2+k*math.pi/32),y,z+r*math.sin(math.pi/2+k*math.pi/32)) for k in range(33)]
    return rd.prism(points,(0,w,0))
def cross(a,b):
    return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def hex_shaft(x,y,z,flat,length):
    rc=flat/math.sqrt(3.)
    pts=[(x+rc*math.cos(k*math.pi/3),y,z+rc*math.sin(k*math.pi/3)) for k in range(6)]
    return rd.prism(pts,(0,length,0))
def interior(a,b):
    return all(min(a[i+3],b[i+3])-max(a[i],b[i])>.01 for i in range(3))
def intersection(a,b):
    if not interior(cad.bounds(a),cad.bounds(b)):return 0.
    return cad.intersection(a,b)
def retired(k):
    return k.startswith(("grip-","observer-moving-pad-","optical-mount-","qtrx-")) or k=="receiver-R6-ported"

@cache
def base(stroke):
    model=assembly.pose_shapes(560,stroke,"red")
    edits,added=hardware(model,height=560)
    return {k:v for k,v in (model|edits|added).items() if not retired(k)}

@cache
def local():
    p={}
    def put(k,frame,material,s):p[k]=(frame,material,s)
    def common(k,s,material="Al"):put(k,"base",material,s)
    # Acquire the four existing steel anchor blocks at x=-80/+140 and y=ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â±180.
    for side,sign in (("L",1),("R",-1)):
        y=sign*180
        sill=box(-96,y-9,485,253,18,4)
        for x in (-80,140):sill=subtract(sill,rod(x,y,483,AZ,2.7,8))
        common("sill-"+side,sill)
        for x in (-80,140):
            fast=rod(x,y,474.5,AZ,2.45,14.5)
            head=rod(x,y,489,AZ,4,2.5)
            common(f"anchor-{side}-{x}",fuse(fast,head),"steel")
    for x in (-91,92):
        # Each shallow bridge picks up the original inner rack anchors at y=56.
        common("rack-bridge-"+str(x),box(x-4,-171,485,8,342,4))
    for gx in (-10,130):
        for sign in (-1,1):
            y=sign*130
            guide=rod(gx,min(sign*53,sign*225),499,AY,5,172)
            common(f"guide-{gx}-{sign}",guide,"steel")
            common(f"guide-rib-{gx}-{sign}",box(gx-7,min(53*sign,225*sign)-5,489,14,182,2))
            for yy in (sign*53,sign*225):
                mount=subtract(box(gx-7,yy-5,491,14,10,16),rod(gx,yy-6,499,AY,5.05,12))
                common(f"guide-support-{gx}-{yy}",mount)
    common("rear-drive-hex",hex_shaft(REAR,-205,Z0,12.7,410),"steel")
    common("rear-bulkhead",box(-58,-160,Z+2,4,320,120),"poly")
    for sign in (-1,1):
        post=box(-102,sign*144-3,488.5,6,6,Z+94-488.5)
        foot=box(-102,sign*144-3,488,14,6,4)
        upper=box(-101,sign*144-3,Z+88,45,6,6)
        common(f"bulkhead-stay-{sign}",fuse(fuse(post,foot),upper))
    # Two side cassettes ride four independent 10 mm linear bearings.
    for lr,s in (("L",1),("R",-1)):
        y=85*s
        def moving(k,shape,material="Al"):put(lr+"-"+k,lr,material,shape)
        def tilt(shape):return rotate(shape,(REAR,0,Z0),AY,-6)
        belt=subtract(profile(REAR,REAR+223.5,y-7.5,Z0,PR+1.2,15),
                      profile(REAR,REAR+223.5,y-8.5,Z0,PR-1.14,17))
        moving("537-3M-15",tilt(belt),"rubber")
        for name,x,z in (("drive",REAR,Z0),("nose",FRONT,Z1)):
            yr=y-7.5
            pulley=rod(x,yr,z,AY,PR-1.24,15)
            bore=hex_shaft(x,yr-1,z,12.85,17) if name=="drive" else rod(x,yr-1,z,AY,4.05,17)
            moving(name+"-pulley",subtract(pulley,bore))
            if name=="nose":moving("nose-axle",rod(x,y-13,z,AY,4,26),"steel")
            for n in (-1,1):
                yy=y+n*10
                ring=ann(x,yy-2,z,"y",8.25,7.45 if name=="drive" else 4.05,4)
                moving(f"{name}-bearing-{n}",ring,"bearing")
        for n in (-1,1):
            yy=y+n*10
            plate=profile(REAR,REAR+223.5,yy-1.8,Z0,10,3.6)
            for x in (REAR,REAR+223.5):plate=subtract(plate,rod(x,yy-3,Z0,AY,8.3,6))
            moving("cheek-"+str(n),tilt(plate))
        strip=box(REAR+14,y-6,Z0+PR-4.15,223.5-28,12,3)
        moving("tooth-backing",tilt(strip),"poly")
        for gx in (-10,130):
            housing=subtract(box(gx-8,y-12,491,16,24,16),rod(gx,y-13,499,AY,5.1,26))
            moving("linear-bushing-"+str(gx),housing,"bearing")
            bottom=Z0+(gx-REAR)*math.tan(math.radians(6))-9
            moving("bushing-stay-"+str(gx),box(gx-5,y+8.2,507,10,3.6,bottom-507))
    # Captively engaged opposite timing-belt spans separate the cassettes.
    common("gate-belt-loop",cad.translated(rotate(rd.loop(-210,210,-38,497,9.5493,9,"z"),(0,0,0),AZ,90),(0,4,0)),"rubber")
    # A pivot-to-head transmission with constant center distance follows the swinging hood.
    common("hood-pivot-shaft",rod(-ARM*1000,-194,682,AY,5,388),"steel")
    hood_y=0.
    put("hood-hex-shaft","hood","Al",hex_shaft(0,-189,682,12.7,354))
    for lr,s in (("L",1),("R",-1)):
        y=s*78
        put("hood-wheel-"+lr,"hood","rubber",ann(0,y-6.35,682,"y",38.1,18,12.7))
        hub=subtract(rod(0,y-6.35,682,AY,18,12.7),hex_shaft(0,y-7,682,12.85,15))
        put("hood-hub-"+lr,"hood","Al",hub)
        yy=s*160
        arm=profile(-ARM*1000,0,yy-2,682,11,4)
        for x,r0 in ((-ARM*1000,5.1),(0,7.45)):arm=subtract(arm,rod(x,yy-3,682,AY,r0,6))
        put("hood-arm-"+lr,"hood","Al",arm)
        common("hood-upright-"+lr,beam((-89,s*160,489),(-ARM*1000,s*168,672),5))
        common("hinge-bearing-"+lr,ann(-ARM*1000,s*168-2,682,"y",11,5.05,4))
        # Receiver-presence channels look down outside the wheel lanes.
        put("optical-range-"+lr,"hood","sensor",box(-3,s*110-4,664,6,8,4))
    # Trim contact faces of welded structural members to the corresponding parent.
    for sign in (-1,1):
        key=f"bulkhead-stay-{sign}"
        f,mat,obj=p[key]
        p[key]=(f,mat,subtract(subtract(obj,p["rack-bridge--91"][2]),p["rear-bulkhead"][2]))
        lr="L" if sign==1 else "R"
        key="hood-upright-"+lr
        f,mat,obj=p[key];p[key]=(f,mat,subtract(subtract(obj,p["rack-bridge--91"][2]),p["hinge-bearing-"+lr][2]))
        for gx in (-10,130):
            k=lr+"-bushing-stay-"+str(gx);f,mat,obj=p[k]
            p[k]=(f,mat,subtract(obj,p[lr+"-cheek-1"][2]))
    put("optical-crossbar","hood","Al",box(-61,-161,666,5,322,4))
    for lr,s in (("L",1),("R",-1)):
        put("optical-probe-arm-"+lr,"hood","Al",box(-56,s*110-2,664,53,4,3))
        put("optical-riser-"+lr,"hood","Al",box(-61,s*160-1.8,670,5,3.6,2.1))
    for lr,sgn in (("L",1),("R",-1)):
        k="optical-riser-"+lr;f,mat,obj=p[k];p[k]=(f,mat,subtract(obj,p["hood-arm-"+lr][2]))
        k="guide-rib-130-"+str(sgn);f,mat,obj=p[k]
        p[k]=(f,mat,subtract(obj,rod(140,180*sgn,488,AZ,4.35,5)))
    import transmission
    transmission.register(p,put,sys.modules[__name__])
    return p

def shapes(stroke,opening=0.,height=0.,right_open=None):
    result={}
    for k,(frame,_,shape) in local().items():
        dy=0
        if frame in ("L","R"):dy=opening if frame=="L" else -(opening if right_open is None else right_open)
        if frame=="hood":
            shape=rotate(shape,(-ARM*1000,0,682),AY,-math.degrees(math.asin(height/(1000*ARM))))
        result[k]=cad.translated(shape,(stroke,dy,0))
    return result

def check():
    added=shapes(0,height=40)
    path=HERE/"split-gate.step"
    tags=[{"id":k,"group":local()[k][0]} for k in added]
    cad.write_named(path,list(added.values()),tags,{"base":"#708492","L":"#648e91","R":"#749d91","hood":"#c79854"})
    finalize_step(path);opened=cad.read_named(path,tags)
    if any(cad.solid_count(s)!=1 for s in opened.values()):raise ValueError("Split part")
    cases={}
    for label,stroke,op,h in (("receive",0,0,40),("clamp",0,0,0),("place",370,0,0),("half",370,56,0),("open",370,112,0)):
        robot=base(stroke)
        native=opened if label=="receive" else shapes(stroke,op,h)
        boxes={k:cad.bounds(v) for k,v in (robot|native).items()}
        hits={};self_hits={}
        for i,(k,obj) in enumerate(native.items()):
            for key,other in robot.items():
                if interior(boxes[k],boxes[key]):
                    v=cad.intersection(obj,other)
                    if v>.02:hits[k+" / "+key]=v
            for key in list(native)[:i]:
                if interior(boxes[k],boxes[key]):
                    v=cad.intersection(obj,native[key])
                    if v>.05:self_hits[k+" / "+key]=v
        cases[label]={"onRobot":hits,"withinGate":self_hits}
    return {"nativeSha256":sha(path),"bodyCount":len(opened),"poses":cases,
            "volumeMm3":{k:cad.volume(s)[0] for k,s in opened.items()}}
if __name__=="__main__":
    (HERE/"package.json").write_text(json.dumps(check(),indent=2)+"\n",encoding="utf-8",newline="\n")
