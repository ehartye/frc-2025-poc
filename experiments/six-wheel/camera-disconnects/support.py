"""Project-original strain relief, removable rail clamp, adapter rest and splice board in shelf coordinates."""
import json
import math
import re
from pathlib import Path
from unittest.mock import patch
import numpy as np
from connector_fit import HERE, ROOT, bounds, cad, digest, transform
from camera_parts import box, cylinder, cut, unite
from routes import read, values

def rod(a,b,r):
    a=np.array(a,float);d=np.array(b)-a;l=np.linalg.norm(d)
    return cylinder(*a,r,l,d/l)

def design():
    p=read(HERE/"route-controls.json")
    bodies={};mat={};mounts={}
    def put(name,shape,material="aluminum"):
        bodies[name]=shape;mat[name]=material
    # Permanent clamp sits clear of the welded camera plate at Y -53..-65.
    x0,x1,y0,y1=-13.,18.,-65.,-53.
    screw_xy=[(-9.5,-59.),(14.5,-59.)]
    lower=cut(box(x0,y0,858.5,x1-x0,y1-y0,15.5),box(-5.15,y0-1,862.4,15.30,y1-y0+2,17))
    cap=cut(box(x0,y0,874.15,x1-x0,y1-y0,7.35),box(-5.15,y0-1,873.8,15.3,y1-y0+2,3.8))
    for x,y in screw_xy:
        lower=cut(lower,cylinder(x,y,858.4,1.5,15.7))
        cap=cut(cap,cylinder(x,y,874,1.65,8))
        put(f"rail-M3-{x:g}",unite(cylinder(x,y,865.5,1.5,16),cylinder(x,y,881.5,2.7,3)),"steel")
    put("rail-foot",lower);put("rail-cap",cap)
    anchor=np.array([-10.,-59.,859.])
    # This cross-member joins both wire clamps below the camera and bridge fastener tools.
    spine=np.array([-49.,-45.,835.])
    put("rail-to-spine",rod(anchor,spine,2.5))
    for k,d,t in (("data",6.30,.90),("power",8.60,.93)):
        center=values(p[k],t)[0];direction=values(p[k],t,1)[0];direction/=np.linalg.norm(direction)
        lateral=np.cross([0,0,1.],direction);lateral/=np.linalg.norm(lateral)
        up=np.cross(direction,lateral)
        rotation=np.column_stack((direction,lateral,up))
        width=d+12;h=d/2+4.5;lh=5.;hole=d/2+.30
        def place(sh): return transform(sh,rotation,center)
        base=cut(box(-lh,-width/2,-h,2*lh,width,h-.20),cylinder(-lh-.1,0,0,hole,2*lh+.2,(1,0,0)))
        lid=cut(box(-lh,-width/2,.20,2*lh,width,h-.20),cylinder(-lh-.1,0,0,hole,2*lh+.2,(1,0,0)))
        for side in (-1,1):
            y=side*(d/2+3.25)
            base=cut(base,cylinder(0,y,-h-.1,1.5,h+.2))
            lid=cut(lid,cylinder(0,y,.1,1.7,h+.2))
            fastener=unite(cylinder(0,y,-h+.5,1.5,2*h-.5),cylinder(0,y,h,2.7,3))
            put(f"{k}-M3-{side}",place(fastener),"steel")
        put(k+"-shoe",place(base));put(k+"-cap",place(lid))
        foot=center+up*(-h+.05)
        put(k+"-support",rod(spine,foot,2.3))
        mounts[k]={"t":t,"center_mm":center.tolist(),"axis":direction.tolist(),"clear_bore_mm":2*hole,
                   "nominal_pad_thickness_mm":.45,"release_cap_direction":up.tolist()}
    # Open saddle carries the light adapter case without imposing a side load on its male USB shell.
    # A thin removable keyed liner is fitted to the purchased housing after measuring the actual fit.
    rest=box(20.5,-29.0,839.4,8.0,27.,1.8)
    rest=unite(rest,box(20.5,-30.4,840.9,8,1.4,6.1),box(20.5,-2.,840.9,8,1.4,6.1))
    rest=cut(rest,box(20.4,-25.55,838.8,5.6,4.,3.))  # Clearance for the lower locking stud of the data plug.
    # Central tongue bears on the end face between the two C inputs during withdrawal.
    rest=unite(rest,box(17.3,-17.55,839.4,6.7,4,1.8),box(17.3,-17.55,840.1,2.2,4,12))
    for i,y in enumerate((-31.2,.1)):
        rest=unite(rest,box(21.5,y-1.85,839.4,6,3.7,15.0))
        rest=cut(rest,cylinder(24.5,y,848.8,1.0,5.8))
    strap=cut(box(21.5,-33.05,854.4,6,35.,1.5),box(21.4,-25.55,854.2,4.6,4.,1.9))
    for i,y in enumerate((-31.2,.1)):
        strap=cut(strap,cylinder(24.5,y,854.3,1.1,1.8))
        put(f"adapter-bench-M2-{i}",unite(cylinder(24.5,y,849.9,1,6),cylinder(24.5,y,855.9,1.65,1.8)),"steel")
    put("adapter-bench-strap",strap)
    put("adapter-rest",rest)
    put("rest-support",rod(spine,[24.,-30.1,840.5],2.4))
    # Purchased cord is cut back from USB-A; both heavy conductors terminate at holes on the left.
    # Individual sleeves isolate the unused high-speed cores and shield end.
    pcb=box(-53,-15.55,844.0,18,16,1.6)
    for x,r in ((-49,1.15),(-40,.55)):
        for y in (-10.55,-4.55):
            pcb=cut(pcb,cylinder(x,y,843.8,r,2))
    clips=box(-47.5,-15.55,841.6,4.,16,2.4)
    for i,y in enumerate((-13.0,-2.1)):
        pcb=cut(pcb,cylinder(-45.5,y,843.9,1.1,2))
        clips=cut(clips,cylinder(-45.5,y,841.5,1.,2.6))
        pin=unite(cylinder(-45.5,y,841.6,1.,4.),cylinder(-45.5,y,845.6,1.65,1.8))
        put(f"board-M2-{i}",pin,"steel")
    put("transition-board",pcb,"FR4")
    put("board-support",rod(spine,[-46.,-14.6,842.1],2.1))
    put("insulating-board-clip",clips,"insulator")
    return bodies,mat,mounts

def run():
    shapes,mat,clamps=design()
    colors={"aluminum":"#a9b3bd","FR4":"#42775a","steel":"#686c76","insulator":"#566779"}
    labels=[{"id":k,"group":mat[k]} for k in shapes]
    writer_type=cad.STEPCAFControl_Writer
    def plain():
        w=writer_type();w.SetColorMode(False);return w
    path=HERE/"service-support.step"
    with patch.object(cad,"STEPCAFControl_Writer",plain):
        cad.write_named(path,list(shapes.values()),labels,colors)
    path.write_text(re.sub(r"(FILE_NAME\('Open CASCADE Shape Model',')[^']+",r"\g<1>2026-11-19T00:00:00",path.read_text(encoding="ascii"),count=1),encoding="ascii",newline="\n")
    reopened=cad.read_named(path,labels)
    density={"aluminum":2.70e-6,"steel":7.85e-6,"FR4":1.85e-6,"insulator":1.25e-6}
    records={k:{"material":mat[k],"kg":cad.volume(v)[0]*density[mat[k]],"box_mm":list(bounds(v))} for k,v in reopened.items()}
    output={"step_sha256":digest(path),"native_count":len(records),"nominal_solid_kg":sum(r["kg"] for r in records.values()),
            "clamps":clamps,"parts":records}
    (HERE/"support.json").write_text(json.dumps(output,indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":run()
