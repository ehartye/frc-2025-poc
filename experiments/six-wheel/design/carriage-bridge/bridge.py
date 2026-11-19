"""Reopen the four full robots and qualify a windowed replacement to a single lift member."""
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
SOURCE=ROOT/"design/competition-package/shop-duplex"
sys.path.insert(0,str(SOURCE))
from assembly import cad
from build_successor import write_json
from duplex import finalize_step
from mechanical import box,cylinder
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepExtrema import BRepExtrema_DistShapeShape

ID="slide-base-bridge"
NEW_ID="slide-base-bridge-windowed"
HEIGHTS={"stowed":940,"receive":940,"transport":670,"place":560}
X_BANDS=[(-135,-109),(-64,0),(18,100),(118,163)]
Y_BANDS=[(-150,-90),(-80,-15),(0,65),(93,150)]
RADIUS=8
# Leave the powered slide bracket, bearing and extend-stop land unbroken.
WINDOWS=[(a,b,c,d) for a,c in X_BANDS for b,d in Y_BANDS if (a,b)!=(18,-150)]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rounded_window(x,y,xx,yy,z):
    r=RADIUS
    solids=[box(x+r,y,z,xx-x-2*r,yy-y,7),box(x,y+r,z,xx-x,yy-y-2*r,7)]
    for cx in (x+r,xx-r):
        for cy in (y+r,yy-r):solids.append(cylinder(cx,cy,z,[0,0,1],r,7))
    return cad.brep(["union",solids])


def cut(shape,tool):
    op=BRepAlgoAPI_Cut(shape,tool)
    if not op.IsDone():raise ValueError("Failed plate cut")
    return op.Shape()


def make(source,height):
    shape=source
    for window in WINDOWS:shape=cut(shape,rounded_window(*window,height-128))
    return shape


def inspect_contacts(full,before,after,height):
    """A 0.05 mm outward skin reveals the actual mate's intersecting material.
    Keep its entire witness from both faces and from both side edges.
    """
    b0=cad.bounds(before)
    results={}
    for name,s in full.items():
        if name==ID:continue
        b=cad.bounds(s)
        if any(b[i]>b0[i+3]+.001 or b[i+3]<b0[i]-.001 for i in range(3)):continue
        if BRepExtrema_DistShapeShape(before,s).Value()>.0001:continue
        if BRepExtrema_DistShapeShape(after,s).Value()>.0001:raise ValueError("Removed contact "+name)
        positions=[]
        for direction in [(0,0,.05),(0,0,-.05),(0,.05,0),(0,-.05,0)]:
            left=cad.intersection(cad.translated(before,direction),s)
            right=cad.intersection(cad.translated(after,direction),s)
            if abs(left-right)>.001:raise ValueError(("Lost interface",name,left,right))
            if left>.001:positions.append(round(left,5))
        if not positions:raise ValueError("Contact has no registered surface witness "+name)
        results[name]=positions
    return results


def write_drawing():
    def uv(v,axis):return v+(147 if axis=='x' else 205)
    with (HERE/"windows.csv").open("w",encoding="utf-8",newline="") as f:
        writer=csv.writer(f,lineterminator="\n")
        writer.writerow(["window","minU","minV","maxU","maxV","internalRadius","throughMm"])
        for i,(x,y,xx,yy) in enumerate(WINDOWS,1):
            writer.writerow([i,uv(x,'x'),uv(y,'y'),uv(xx,'x'),uv(yy,'y'),RADIUS,5])
    entities=[]
    def entity(*v):entities.extend(str(a) for a in v)
    def line(layer,x,y,xx,yy):entity(0,"LINE",8,layer,10,x,20,y,30,0,11,xx,21,yy,31,0)
    entity(0,"SECTION",2,"HEADER",9,"$INSUNITS",70,4,0,"ENDSEC",0,"SECTION",2,"ENTITIES")
    for a,b,c,d in [(0,0,322,0),(322,0,322,410),(322,410,0,410),(0,410,0,0)]:line("STOCK",a,b,c,d)
    for x,y,xx,yy in WINDOWS:
        x,xx=x+147,xx+147
        y,yy=y+205,yy+205
        r=RADIUS
        for a,b,c,d in [(x+r,y,xx-r,y),(xx,y+r,xx,yy-r),(xx-r,yy,x+r,yy),(x,yy-r,x,y+r)]:
            line("THROUGH",a,b,c,d)
        for cx,cy,start in [(xx-r,y+r,270),(xx-r,yy-r,0),(x+r,yy-r,90),(x+r,y+r,180)]:
            entity(0,"ARC",8,"THROUGH",10,cx,20,cy,30,0,40,r,50,start,51,start+90)
    entity(0,"ENDSEC",0,"EOF")
    (HERE/"bridge.dxf").write_text("\n".join(entities)+"\n",encoding="ascii",newline="\n")


def check():
    write_drawing()
    directory=SOURCE/"red"
    frames=json.loads((directory/"part-frames.json").read_bytes())
    reviews=json.loads((directory/"review.json").read_bytes())
    if frames[ID]!="lift":raise ValueError("Unverified ownership")
    result={"replaces":ID,"newId":NEW_ID,"frame":"lift","poses":{}}
    for pose,h in HEIGHTS.items():
        step=directory/(pose+".step")
        if sha(step)!=reviews[pose]["export_sha256"]:raise ValueError("Changed 545 source")
        full=cad.read_named(step,[{"id":k} for k in frames])
        old=full[ID]
        new=make(old,h)
        v0=cad.volume(old)[0]
        v1,cg=cad.volume(new)
        shared=cad.intersection(new,old)
        if abs(shared-v1)>.01:raise ValueError("Not contained in predecessor")
        area=sum((xx-x)*(yy-y)-(4-math.pi)*RADIUS**2 for x,y,xx,yy in WINDOWS)
        if abs(v0-v1-area*5)>.01:raise ValueError("Drawing differs from CAD")
        result["poses"][pose]={"sourceSha256":sha(step),"referenceVolumeMm3":v0,"volumeMm3":v1,
            "newMaterialOutsideSourceMm3":max(0,v1-shared),"cgMm":cg,
            "contactWitnessesMm3":inspect_contacts(full,old,new,h)}
        if pose=="place":
            # The alternative full-width void demonstrates why outer envelope alone is insufficient.
            lost=cut(old,cad.brep(box(-70,60,h-128,160,35,7)))
            mate=full["slide-fixed-rack-bed"]
            def witness(s):return cad.intersection(cad.translated(s,(0,0,.05)),mate)
            result["rejectedRackWitnessMm3"]={"stock":witness(old),"uncorrectedVoid":witness(lost)}
            labels=[{"id":NEW_ID,"group":"plate"}]
            path=HERE/"bridge-place.step"
            cad.write_named(path,[new],labels)
            finalize_step(path)
            reopen=cad.read_named(path,labels)[NEW_ID]
            common=cad.intersection(reopen,new)
            if max(abs(cad.volume(s)[0]-common) for s in (reopen,new))>.01:raise ValueError("STEP round trip")
            result["exportSha256"]=sha(path)
    return result


if __name__=="__main__":
    write_json(HERE/"native.json",check())
