"""Original bolt-on moving camera bridge. Frames, members and shop cut dimensions use robot millimetres."""
import argparse
import json
import math
from pathlib import Path
import sys
from optics import ROOT,HERE
sys.path.insert(0,str(ROOT/"low-observation"))
from geometry import pose_layers,original,hardware
cad=original.native

def box(*v):return ["box",v]
def zc(x,y,z,r,h):return ["cyl",[x,y,z,0,0,1,r,h]]
def minus(a,*b):return ["cut",[a,*b]]
def ztube(x,y,z,h):return minus(box(x,y,z,15,15,h),box(x+1,y+1,z-1,13,13,h+2))

def parts(raised=False):
    cx,cz=(115.,860.) if raised else (98.,842.)
    plate_length,plate_width,pocket_length,pocket_width=(87,98,29,58) if raised else (65,86,22,54)
    items={}
    bores=[]
    materials={}
    def put(name,form,kind):
        items[name]=cad.brep(form)
        materials[name]=kind
    for tag,sgn in (("left",1),("right",-1)):
        y=sgn*200
        holes=[zc(x,y,472,2.25,37) for x in (29,76)]
        bores.extend(holes)
        put(tag+"-foot",minus(box(20,y-9,500,65,18,3),*holes),"6061")
        put(tag+"-post",ztube(45,y-7.5,503,cz+20.5-503),"6061")
        for x in (29,76):
            head=zc(x,y,503,3.5,3.8)
            shaft=zc(x,y,477.5,2,25.5)
            put(f"{tag}-M4x25-{x}",["union",[head,shaft]],"steel")
            put(f"{tag}-washer-{x}",minus(zc(x,y,484,4.5,1),zc(x,y,483,2.15,3)),"steel")
            put(f"{tag}-locknut-{x}",minus(zc(x,y,479,4,5),zc(x,y,478,2,7)),"steel")
    put("top-tube",minus(box(45,-207.5,cz+20.5,15,415,15),box(46,-209,cz+21.5,13,418,13)),"6061")
    camera_bores=[zc(cx,y,cz+15,2.25,9) for y in (-37.5,37.5)]
    put("top-plate",minus(box(45,-plate_width/2,cz+17.5,plate_length,plate_width,3),
                          box(64,-pocket_width/2,cz+17,pocket_length,pocket_width,5),*camera_bores),"6061")
    # Two M4 rear mounts per the supplier's 75 mm pattern. Internal camera thread engagement is conditional.
    put("camera-envelope",minus(box(cx-14,-45.5,cz,28,91,17.5),*[zc(cx,y,cz+13.5,2,6) for y in (-37.5,37.5)]),"camera")
    for y in (-37.5,37.5):
        put(f"camera-M4-{y}",["union",[zc(cx,y,cz+14,2,6.5),zc(cx,y,cz+20.5,3.5,4)]],"steel")
    return items,materials,bores

def limits(box1,box2):
    return math.hypot(*(max(0,box1[i]-box2[i+3],box2[i]-box1[i+3]) for i in range(3)))

def evaluate(scratch):
    shapes,mat,bores=parts()
    layers,_,_,frames=pose_layers(560,0,"red")
    body=original.flatten(layers)
    modified,added=hardware(body,height=560)
    body.update(modified)
    body.update(added)
    frames.update({k:"base" for k in added})
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    shelf=body["receiver-R6-ported"]
    drilled=0.
    for bore in bores:
        cutter=cad.brep(bore)
        delta=cad.intersection(shelf,cutter)
        if abs(delta-15*math.pi*2.25**2)>.01:raise ValueError(f"hole misses full shelf thickness: {delta}")
        drilled+=delta
        shelf=BRepAlgoAPI_Cut(shelf,cutter).Shape()
    body["receiver-R6-ported"]=shelf
    swept={}
    f={"slide":1,"middle":.5,"base":0,"lift":0,"variable-rope":0,"variable-draw-wire":0}
    for k,v in body.items():
        box0=list(cad.bounds(v))
        if k in original.rotation_bounds():
            t=original.rotation_bounds()[k]["bounds_mm"]
            box0=[t[0],t[1],t[2]+560,t[3],t[4],t[5]+560]
        dx=(1-f[frames[k]])*370
        box0[0]-=dx
        swept[k]=box0
    # A relative swept AABB proves disjointness on the full linear 0–370 mm path.
    # Where it reaches a same-frame stock body, test exact positive OCCT volume at constant pose.
    conflicts=[]
    potential=[]
    ray_unknown=[]
    # Optical corridor covers calibrated inner x/axis columns, both near end edges, and rigid backdrops.
    # It intentionally includes body-backed pixels below the shelf-facing tube.
    for k,b in swept.items():
        if frames[k]!="slide" and limits(b,[-65,-205,499,185,205,860])<.001:ray_unknown.append(k)
    for k,v in shapes.items():
        b=cad.bounds(v)
        for n,w in body.items():
            if limits(b,swept[n])>1e-5:continue
            potential.append([k,n,frames[n]])
            if frames[n]!="slide":conflicts.append([k,n,"unresolved moving sweep"])
            elif (vol:=cad.intersection(v,w))>.001:conflicts.append([k,n,vol])
    records={}
    for k,v in shapes.items():
        vol=cad.volume(v)[0]
        mass=.061 if mat[k]=="camera" else vol*(2.70e-6 if mat[k]=="6061" else 7.85e-6)
        records[k]={"material":mat[k],"bounds_mm":cad.bounds(v),"kg":mass}
    data={"parts":records,"starting_bodies":len(body),"bore_count":len(bores),"removed_shelf_mm3":drilled,
          "combined_count":len(body)+len(shapes),"mass_before_cable_welds_and_power_kg":sum(r["kg"] for r in records.values()),
          "positive_or_unresolved_interference":conflicts,"contact_candidates":potential,"fixed_or_middle_optical_candidates":ray_unknown}
    path=HERE/"moving-bridge.step"
    all_parts={**shapes,"receiver-R6-drilled-for-camera":shelf}
    labels=[{"id":k,"group":"observed"} for k in all_parts]
    # XCAF emits a hash-ordered color table. This mechanical export needs persistent names, not styling.
    from unittest.mock import patch
    writer_type=cad.STEPCAFControl_Writer
    def uncolored_writer():
        writer=writer_type()
        writer.SetColorMode(False)
        return writer
    with patch.object(cad,"STEPCAFControl_Writer",uncolored_writer):
        cad.write_named(path,list(all_parts.values()),labels,{"observed":"#505b6c"})
    # Neutral STEP header timestamp makes a regenerated engineering artifact comparable byte for byte.
    import re
    text=path.read_text(encoding="ascii")
    text=re.sub(r"(FILE_NAME\('Open CASCADE Shape Model',')[^']+",r"\g<1>2026-11-18T00:00:00",text,count=1)
    path.write_text(text,encoding="ascii",newline="\n")
    restored=cad.read_named(path,labels)
    if set(restored)!=set(all_parts):raise ValueError("STEP did not reopen")
    data["step_parts"]=len(restored)
    (HERE/"mount.json").write_text(json.dumps(data,indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")

if __name__=="__main__":
    evaluate(None)
