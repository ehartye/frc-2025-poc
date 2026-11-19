"""Editable shelf, bolted datum frame, captive screws and keeper pins, in robot millimetres."""
from pathlib import Path
import argparse,hashlib,json,math,re,sys
from itertools import combinations
from unittest.mock import patch
HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"bridge-tolerance"))
from design import mount
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut,BRepAlgoAPI_Fuse
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCone
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt
cad=mount.cad
SHELF="receiver-R6-drilled-for-camera"
DENSITY={"steel":7.85e-6,"6061":2.70e-6}
X_BOLTS=(-21.,26.)
X_RETAIN=(-12.,17.)
PIN=2.5


def cylinder(x,y,z,r,h,axis=(0,0,1)):
    return cad.brep(["cyl",[x,y,z,*axis,r,h]])
def box(x,y,z,dx,dy,dz):
    return cad.brep(["box",[x,y,z,dx,dy,dz]])
def cone(x,y,z,r0,r1,h):
    return BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(x,y,z),gp_Dir(0,0,1)),r0,r1,h).Shape()
def alter(body,tools,operation):
    for tool in tools:
        cmd=operation(body,tool)
        if not cmd.IsDone():raise ValueError("CAD boolean failed")
        body=cmd.Shape()
    return body
def unite(body,*tools):return alter(body,tools,BRepAlgoAPI_Fuse)
def cut(body,*tools):return alter(body,tools,BRepAlgoAPI_Cut)
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def read(path):return json.loads(path.read_bytes())


def bore_hex(x,y,z,across,h):
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon,BRepBuilderAPI_MakeFace
    from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
    from OCP.gp import gp_Vec
    w=BRepBuilderAPI_MakePolygon()
    for i in range(6):
        angle=math.pi*i/3
        w.Add(gp_Pnt(x+across/math.sqrt(3)*math.cos(angle),y+across/math.sqrt(3)*math.sin(angle),z))
    w.Close()
    return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(w.Wire()).Face(),gp_Vec(0,0,h)).Shape()


def captive_boss(x,y,base,top,pin_z):
    # Machine integral ears by drilling the central opening and two transverse pin bores.
    ears=cut(box(x-5.4,y-5.4,base,10.8,10.8,top-base),cylinder(x,y,base-.1,4.15,top-base+.2))
    pins={}
    for key,dx in (("rear",-2.75),("front",2.75)):
        p=cylinder(x+dx,y-5.4,pin_z,.75,10.8,(0,1,0))
        ears=cut(ears,p)
        pins[key]=p  # circular interference envelope of a roll pin, installed flush
    return ears,pins


def screw(x,y,face,length,head=3.8):
    raw=unite(cylinder(x,y,face-length,2,length),cylinder(x,y,face,3.5,head))
    return cut(raw,bore_hex(x,y,face+head-2,3,2.1))


def build():
    old_meta=read(ROOT/"bridge-tolerance/mount.json")
    old=cad.read_named(ROOT/"bridge-tolerance/moving-bridge.step",[{"id":k} for k in (*old_meta["parts"],SHELF)])
    body={}
    materials={}
    def put(key,v,m):
        body[key]=v
        materials[key]=m
    shelf=old[SHELF]
    for side,sign in (("left",1),("right",-1)):
        y=200*sign
        # The bar's integral bosses close the main screw's preload entirely through metal.
        bar=box(-27,y-6,481,59,12,4)
        for x in X_BOLTS:
            shelf=cut(shelf,cylinder(x,y,484.9,4.10,15.2))
            bar=unite(bar,cylinder(x,y,485,4,15))
            # A thread-envelope is hollow at major diameter so assembled native BReps do not overlap.
            bar=cut(bar,cylinder(x,y,480.9,2,19.2))
        shelf=cut(shelf,cylinder(PIN,y,484.9,4.10,15.2))
        bar=unite(bar,cylinder(PIN,y,485,4,15),cylinder(PIN,y,500,3,1.8),cone(PIN,y,501.8,3,2.8,.2))
        for x in X_RETAIN:
            shelf=cut(shelf,cylinder(x,y,484.9,1.7,15.2),cone(x,y,498.7,1.7,3,1.3))
            bar=cut(bar,cylinder(x,y,480.9,1.5,4.2))
            fastener=unite(cylinder(x,y,480,1.5,18.5),cone(x,y,498.5,1.5,3,1.5))
            put(f"{side}-M3x20-rail-{x:g}",cut(fastener,bore_hex(x,y,498.9,2,1.2)),"steel")
        put(side+"-threaded-saddle",bar,"steel")
        # The forward inboard corner of the issued 18-wide foot crosses the grip dashpot
        # on lift-off. Leave the post untouched and machine this edge back to |Y|=194.
        pocket=box(11,190 if sign>0 else -194,499,25,4,5)
        full_edge=box(-31,190 if sign>0 else -192,499,67,2,5)
        foot=cut(old[side+"-foot"],full_edge,pocket)
        locating=cylinder(PIN,y,499.9,3.04,3.2)
        if sign<0:
            locating=unite(cylinder(PIN,y-1,499.9,3.04,3.2),cylinder(PIN,y+1,499.9,3.04,3.2),
                            box(PIN-3.04,y-1,499.9,6.08,2,3.2))
        foot=cut(foot,locating)
        for x in X_BOLTS:
            ears,pins=captive_boss(x,y,503,516,514)
            foot=unite(foot,ears)
            put(f"{side}-M4x8-{x:g}",screw(x,y,503,8),"steel")
            for suffix,shape in pins.items():put(f"{side}-foot-pin-{x:g}-{suffix}",shape,"steel")
        put(side+"-datum-foot",foot,"6061")
        tube=cut(box(-5,y-7.5,503,15,15,359.5),box(-3,y-5.5,502,11,11,361.5))
        put(side+"-post",tube,"6061")
    cap=cut(box(-5,-207.5,862.5,15,415,15),box(-3,-208,864.5,11,417,11))
    put("top-tube",cap,"6061")
    # Leave two integral 4 x 3 upper ribs on the cantilever lands beside the camera window.
    top=unite(old["top-plate"],*(box(10,y,862.5,36,4,3) for y in (-34,30)))
    for y in (-37.5,37.5):
        ears,pins=captive_boss(48,y,862.5,873,871.5)
        top=unite(top,ears)
        put(f"camera-M4-{y:g}",screw(48,y,862.5,6.5,4),"steel")
        for suffix,shape in pins.items():put(f"camera-pin-{y:g}-{suffix}",shape,"steel")
    put("top-plate-and-keepers",top,"6061")
    put("camera-envelope",old["camera-envelope"],"camera")
    put(SHELF,shelf,"PE")
    return old,body,materials


def run():
    old,body,materials=build()
    labels=[{"id":k,"group":"design"} for k in body]
    writer_type=cad.STEPCAFControl_Writer
    def plain():
        w=writer_type();w.SetColorMode(False);return w
    out=HERE/"camera-shelf.step"
    with patch.object(cad,"STEPCAFControl_Writer",plain):cad.write_named(out,list(body.values()),labels,{"design":"#669ba7"})
    text=out.read_text(encoding="ascii")
    out.write_text(re.sub(r"(FILE_NAME\('Open CASCADE Shape Model',')[^']+",r"\g<1>2026-11-19T00:00:00",text,count=1),
                   encoding="ascii",newline="\n")
    reopened=cad.read_named(out,labels)
    conflicts=[]
    for k,n in combinations(reopened,2):
        if not cad.occupied(cad.bounds(reopened[k]),cad.bounds(reopened[n])):continue
        volume=cad.intersection(reopened[k],reopened[n])
        if volume>.002:conflicts.append([k,n,volume])
    if conflicts:raise ValueError(conflicts)
    records={}
    for k,s in reopened.items():
        v,c=cad.volume(s)
        if abs(v-cad.volume(body[k])[0])>.005:raise ValueError(k)
        kg=.061 if materials[k]=="camera" else DENSITY.get(materials[k],0)*v
        records[k]={"material":materials[k],"volume_mm3":v,"centroid_mm":c,"bounds_mm":cad.bounds(s),"kg":kg if materials[k]!="PE" else None}
    prior=cad.volume(old[SHELF])[0]
    record={"source_step":sha(ROOT/"bridge-tolerance/moving-bridge.step"),"step_sha256":sha(out),"reopened_named_solids":len(reopened),
            "positive_interference":conflicts,"shelf_removed_mm3":prior-records[SHELF]["volume_mm3"],"parts":records}
    (HERE/"native.json").write_text(json.dumps(record,indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":run()
