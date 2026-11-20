"""Size and physically reopen the four through ties and electronics tray on the issued robot."""

import argparse
import csv
import json
import math
from pathlib import Path

from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCone, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakePrism
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Vec

from installed_audit import HERE, ROOT, box_gap, bounds, reopen, sha, volume
from clearance import base

cad = base.cad


def axis(x, y, z):
    return gp_Ax2(gp_Pnt(x, y, z), gp_Dir(0, 0, 1))


def cyl(x, y, z, r, h):
    return BRepPrimAPI_MakeCylinder(axis(x, y, z), r, h).Shape()


def cone(x, y, z, r1, r2, h):
    return BRepPrimAPI_MakeCone(axis(x, y, z), r1, r2, h).Shape()


def tube(x, y, z, od, inner, length):
    return cut(cyl(x,y,z,od/2,length),cyl(x,y,z-1,inner/2,length+2))


def hexagon(x,y,z,across,length):
    poly=BRepBuilderAPI_MakePolygon()
    for i in range(6):
        phi=math.pi*i/3
        r=across/math.sqrt(3)
        poly.Add(gp_Pnt(x+r*math.cos(phi),y+r*math.sin(phi),z))
    poly.Close()
    return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(poly.Wire()).Face(),gp_Vec(0,0,length)).Shape()


def cut(a,b):
    c=BRepAlgoAPI_Cut(a,b)
    if not c.IsDone(): raise ValueError("Unfinished cut")
    return c.Shape()


def fuse(a,b):
    f=BRepAlgoAPI_Fuse(a,b)
    if not f.IsDone(): raise ValueError("Unfinished fuse")
    return f.Shape()


def common(a,b):
    op=BRepAlgoAPI_Common(a,b)
    if not op.IsDone(): raise ValueError("Unfinished intersection")
    return volume(op.Shape())


def dist(a,b):
    d=BRepExtrema_DistShapeShape(a,b)
    if not d.IsDone(): raise ValueError("Unfinished distance")
    return d.Value()


def write_step(parts,path):
    doc,table=cad.xcaf_doc()
    for k,s in sorted(parts.items()):
        label=table.AddShape(s,False)
        cad.TDataStd_Name.Set_s(label,cad.TCollection_ExtendedString(k))
    writer=cad.STEPCAFControl_Writer()
    if not writer.Transfer(doc,cad.STEPControl_AsIs) or writer.Write(str(path))!=cad.IFSelect_RetDone:
        raise ValueError("Could not export changes")
    # STEP timestamp varies across identical rebuilds. Preserve native entity bytes.
    import re
    data=path.read_text(encoding="ascii")
    data=re.sub(r"(FILE_NAME\('[^']*',')[^']+(')",r"\g<1>1970-01-01T00:00:00\2",data,count=1)
    path.write_bytes(("\n".join(line.rstrip() for line in data.splitlines())+"\n").encode("ascii"))
    restored=base.named(path)
    assert set(restored)==set(parts)
    for k,shape in parts.items():
        if abs(volume(shape)-volume(restored[k]))>.005: raise ValueError(k)


def make(items):
    robot={k:s for k,_,_,_,_,s in items}
    added={}
    replaced={}
    drills=[]
    tools={}
    for side,sgn,direction in [("L",1,"Left"),("R",-1,"Right")]:
        rail="swerve-bored-drive-rail-outer-"+side
        saddle=direction+" rail saddle"
        rail_shape=robot[rail]
        plate=robot[saddle]
        for x in (270,317):
            label=f"{side}-{x}"
            y=sgn*341
            drill=cyl(x,y,49,5.05,102)
            pilots=cyl(x,y,149,3.30,12)
            # 90-degree countersink, diameter 13.45 mm at the saddle top.
            countersink=cone(x,y,155.575,3.30,6.725,3.425)
            drills.append({"location":label,"x":x,"y":y,
                           "railVolumeRemovedMm3":common(rail_shape,drill),
                           "saddleVolumeRemovedMm3":common(plate,fuse(pilots,countersink))})
            rail_shape=cut(rail_shape,drill)
            plate=cut(cut(plate,pilots),countersink)
            added[label+"-sleeve"]=tube(x,y,50.025,9.98,6.6,99.95)
            # Thread minor/major detail is omitted. Fastener head/body are one article.
            shank=cyl(x,y,38.85,3,116.425)
            head=cone(x,y,155.275,3,6.575,3.575)
            added[label+"-screw"]=cut(fuse(shank,head),hexagon(x,y,156.35,4,4))
            added[label+"-washer"]=tube(x,y,48.4,12,6.4,1.6)
            added[label+"-nut"]=cut(hexagon(x,y,42.4,10,6),cyl(x,y,41,3,9))
            # Complete straight installation paths for a 4 mm hex key (4.8 mm shank gauge) and thin-wall socket.
            tools[label+"-driver"]=cyl(x,y,159,2.4,135)
            tools[label+"-socket"]=cut(cyl(x,y,1,7.5,47.4),cyl(x,y,1,6.0,47.4))
        replaced[rail]=rail_shape
        replaced[saddle]=plate
    plate_src=ROOT/"experiments/six-wheel/design/signal-light/deck-mount.step"
    platform=base.named(plate_src)["front-electronics-platform"]
    posts={}
    for i in range(4):
        name="platform-post-"+str(i)
        b=bounds(robot[name])
        x,y=(b[0]+b[3])/2,(b[1]+b[4])/2
        posts[name]=dict(x=x,y=y,initialFaceGapMm=dist(platform,robot[name]))
        # Isolate this correction from the device attachment patterns (not released yet).
        top_sink=cone(x,y,291.34,1.7,3.36,1.66)
        platform=cut(cut(platform,cyl(x,y,289,1.7,5)),top_sink)
        post=robot[name]
        replaced[name]=cut(cut(post,cyl(x,y,279,1.25,12)),cyl(x,y,153,1.25,12))
        # Smooth major diameters represent the mating M3 x 0.5 external threads.
        added["M3-platform-"+str(i)]=fuse(cyl(x,y,283.0,1.5,8.2),cone(x,y,291.2,1.5,3.3,1.8))
        # Local underside of a pocket can differ by post; find material on the axis.
        deck_id="host-drilled-swerve-deck"
        deck=replaced.get(deck_id,robot[deck_id])
        probe=cyl(x,y,135,.2,21)
        local=BRepAlgoAPI_Common(deck,probe).Shape()
        z_bottom=round(bounds(local)[2],5)
        posts[name]["lowerDeckUndersideZmm"]=z_bottom
        replaced[deck_id]=cut(deck,cyl(x,y,z_bottom-1,1.7,155-z_bottom))
        replaced[deck_id]=cut(replaced[deck_id],cone(x,y,z_bottom,3.36,1.7,1.66))
        added["M3-post-base-"+str(i)]=fuse(cone(x,y,z_bottom,3.30,1.5,1.80),cyl(x,y,z_bottom+1.8,1.5,10-1.8))
        tools["M3-post-base-"+str(i)+"-driver-before-deck"]=cyl(x,y,z_bottom-43,1.6,43)
    # Four tiny ear and two long rail clashes predate this change. Cut their actual
    # mated footprint from the underside; preserve the top datum at Z154 mm.
    reliefs={}
    for obstacle in ["swerve-cranked-mast-rail-L","swerve-cranked-mast-rail-R"]+[
        "relocated-"+end+"-"+side+"-ear" for end in ("front","rear") for side in ("L","R")
    ]:
        deck=replaced["host-drilled-swerve-deck"]
        hit=BRepAlgoAPI_Common(robot["host-drilled-swerve-deck"],robot[obstacle]).Shape()
        reliefs[obstacle]={"originalOverlapMm3":volume(hit),"boundsMm":list(bounds(hit))}
        # Offset only the obstacle clearance envelope across the tiny local overlap.
        # Individual rectangular pocket avoids an as-formed curved zero-clearance fit.
        reliefs[obstacle]["pocketsMm"]=[]
        for fragment in base.subsolids(hit):
            b=bounds(fragment)
            pocket=[b[0]-.25,b[1]-.25,b[2]-.25,b[3]+.25,b[4]+.25,b[5]+.25]
            reliefs[obstacle]["pocketsMm"].append(pocket)
            deck=cut(deck,base.block(pocket))
        replaced["host-drilled-swerve-deck"]=deck
    added["front-electronics-platform"]=platform
    return replaced,added,tools,dict(plateSource=plate_src.relative_to(ROOT).as_posix(),plateSourceSha256=sha(plate_src),posts=posts,drills=drills,deckReliefs=reliefs)


def evaluate(lamp_path):
    items=reopen()
    original={i[0]:i[-1] for i in items}
    repl,added,tools,result=make(items)
    from weight_variant import apply
    removed,options=apply(original,repl,added,tools,None,None)
    result["fixedHeight"]=options
    assembled={k:s for k,s in (original|repl|added).items() if k not in removed}
    boxes={k:bounds(s) for k,s in assembled.items()}
    touches=[]
    conflicts=[]
    considered=set()
    for k,shape in (repl|added|tools).items():
        b=bounds(shape)
        for name,other in assembled.items():
            pair=tuple(sorted((k,name)))
            if k==name or pair in considered or box_gap(b,boxes[name])>.025: continue
            considered.add(pair)
            v=common(shape,other)
            if v>.02: conflicts.append(dict(a=k,b=name,mm3=round(v,4)))
            elif dist(shape,other)<.025:touches.append(pair)
    # Against the sealed article the keyed platform has a close fit. Its public
    # box spans the hidden threaded neck and has no mating-hole detail.
    import sys
    sys.path.insert(0,str(ROOT/"experiments/six-wheel/design/signal-light"))
    from inspect_light import product,mounted
    lamp=mounted(product(lamp_path),145,-250,293)
    result["supplierLamp"]={"sha256":sha(lamp_path),
        "platformOverlapMm3":common(lamp,added["front-electronics-platform"]),
        "spacerOverlapMm3":common(lamp,assembled["light-spacer"])}
    parts=repl|added
    path=HERE/"station-and-jack-interfaces.step"
    write_step(parts,path)
    result.update(sourceArchiveSha256="88cc5f0927094bd2bbc60da862a137e5eada4484cc84510a514d760f27117c49",outputSha256=sha(path),replacement=list(repl),additional=list(added),
                  portionSolids=len(parts),robotSolids=len(assembled),conflicts=conflicts,contacts=sorted(touches),castTools=list(tools),
                  intent="Install M6 through-ties into the two drive rails; restore the RSL electronics platform and bolt it into all four posts. Threaded interfaces use nominal smooth profile for the major diameter. The rest of the robot has its issued representation.")
    (HERE/"interfaces.json").write_bytes((json.dumps(result,indent=2)+"\n").encode())


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("supplier_lamp",type=Path)
    evaluate(parser.parse_args().supplier_lamp)
