"""Make the through-bolted inner rail ears and verify them against the selected shop robot."""
import argparse
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "frame-corners"))
from weld import cad, digest, distance, extrude, finalize_step, lower_bound, write_step
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse


def read(path):
    return json.loads(path.read_bytes())


def write(path, item):
    path.write_text(json.dumps(item,indent=2)+"\n",encoding="utf-8",newline="\n")


def boolean(operation, first, second):
    query = operation(first,second)
    if not query.IsDone():
        raise RuntimeError("OpenCascade Boolean failed")
    return query.Shape()


def cut(first, second):
    return boolean(BRepAlgoAPI_Cut,first,second)


def fuse(first, second):
    return boolean(BRepAlgoAPI_Fuse,first,second)


def box(x0,x1,y0,y1,z0,z1):
    return cad.brep(["box",[min(x0,x1),min(y0,y1),z0,abs(x1-x0),abs(y1-y0),z1-z0]])


def rod(x,sy,y0,y1,z,radius):
    return cad.brep(["cyl",[x,sy*y0,z,0,sy,0,radius,y1-y0]])


def hexnut(x,sy,z):
    # Across flats 13 mm, height 8 mm, conservative sharp corners.
    radius=6.5/math.cos(math.pi/6)
    points=[(x+radius*math.cos(i*math.pi/3),sy*244.6,z+radius*math.sin(i*math.pi/3))
            for i in range(6)]
    return cut(extrude(points,(0,sy*8,0)),rod(x,sy,244,254,z,4))


def hardware(x,sy,z):
    return {"screw":fuse(rod(x,sy,219.4,227.4,z,6.635),rod(x,sy,227.4,257.4,z,4)),
            "head-washer":cut(rod(x,sy,227.4,229,z,8),rod(x,sy,227,230,z,4.2)),
            "nut-washer":cut(rod(x,sy,243,244.6,z,8),rod(x,sy,242,245,z,4.2)),
            "nut":hexnut(x,sy,z)}


def slot(x,sy,z):
    return fuse(fuse(rod(x-.60,sy,228,236,z,3.015),rod(x+.60,sy,228,236,z,3.015)),
                box(x-.6,x+.6,sy*228,sy*236,z-3.015,z+3.015))


def end_features(rail,sx,sy):
    x=sx*312
    # Trim only the inner rail. Keep the issued shaft and adapter datums.
    rail=cut(rail,box(sx*326,sx*331,sy*228,sy*236,49,151))
    ear=box(sx*300,sx*330,sy*235,sy*243,78,142)
    buttress=extrude([(sx*322,sy*243,78),(sx*330,sy*243,78),(sx*330,sy*255,78),(sx*327,sy*255,78)],(0,0,64))
    ear=fuse(ear,buttress)
    for z in (90,130):
        rail=cut(rail,rod(x,sy,228,236,z,4.5))
        ear=cut(ear,rod(x,sy,234,244,z,4.5))
    rail=cut(rail,rod(x,sy,228,236,110,3.015) if sx==1 else slot(x,sy,110))
    ear=cut(ear,rod(x,sy,234,244,110,3))
    metal={"ear":ear,"pilot":rod(x,sy,231,243,110,3)}
    # Both 2.5 mm fillets run the whole 64 mm vertical seam.
    for face,y,direction in (("in",235,-1),("out",255,1)):
        toe_x,toe_y=327.5,y
        metal["weld-"+face]=extrude([(sx*330,sy*y,78),(sx*toe_x,sy*toe_y,78),
                                      (sx*330,sy*(y+direction*2.5),78)],(0,0,64))
    for z in (90,130):
        metal.update({name+"-"+str(z):shape for name,shape in hardware(x,sy,z).items()})
    return rail,metal


def clearance(shape, others):
    bounds=cad.bounds(shape)
    best=(float("inf"),"")
    hits={}
    for lb,key in sorted((lower_bound(bounds,cad.bounds(s)),k) for k,s in others.items()):
        if lb>best[0]+1e-6:
            break
        gap=distance(shape,others[key])
        best=min(best,(gap,key))
        if gap<1e-6:
            volume=cad.intersection(shape,others[key])
            if volume>.001:
                hits[key]=round(volume,5)
    return {"nearestMm":round(best[0],5),"nearestBody":best[1],"overlapMm3":hits}


def prove(stage):
    if stage["overlapMm3"]:
        raise ValueError(stage)
    return stage


def load_source(artifacts,axle):
    sys.path.insert(0,str(axle))
    from fit import SHOP_STEP, add_exchange, chassis_scene
    _,parts=chassis_scene(artifacts)
    pins={"603-body shop STEP":SHOP_STEP}
    for file,receipt in (("axle-kit.step","fit.json"),("bearing-keepers.step","keepers.json")):
        r=read(axle/receipt)
        add_exchange(parts,axle/file,r["volumeMm3"],r["stepSha256"])
        pins["axle-retention/"+file]=r["stepSha256"]
    for path in ("frame-corners/frame-weldment.step","design/installed-procurement/inner-clamps.step",
                 "design/drivetrain/route-receipt.json","design/selected-shop/articles.json"):
        pins[path]=digest(ROOT/path)
    return parts,pins


def run(artifacts,axle,out):
    parts,pins=load_source(artifacts,axle)
    originals={name:parts[name] for name in ("drive-rail-inner-L","drive-rail-inner-R")}
    kit=dict(originals)
    results={}
    sides=[("L",1),("R",-1)]
    ends=[("front",1),("rear",-1)]
    for side,sy in sides:
        rail="drive-rail-inner-"+side
        for end,sx in ends:
            tag=end+"-"+side+"-"
            kit[rail],metal=end_features(kit[rail],sx,sy)
            kit.update({tag+k:v for k,v in metal.items()})
    out.mkdir(parents=True,exist_ok=True)
    step=out/"inner-rail-joints.step"
    write_step(step,kit)
    finalize_step(step)
    actual=cad.read_named(step,[{"id":k} for k in kit])
    volume={k:cad.volume(v)[0] for k,v in actual.items()}
    if max(abs(volume[k]-cad.volume(v)[0]) for k,v in kit.items())>.001:
        raise ValueError("Issued STEP solid volume changed on reopening")
    installed={**parts,**actual}
    for side,sy in sides:
        rail="drive-rail-inner-"+side
        for end,sx in ends:
            tag=end+"-"+side+"-"
            cross="cross-"+end
            ear=actual[tag+"ear"]
            inward=actual[tag+"weld-in"]
            original=originals[rail]
            case={"originalEndToTubeMm":round(distance(original,parts[cross]),5),
                  "originalAgainstNewInboardFilletMm3":round(cad.intersection(original,inward),4),
                  "cutRailToInnerFilletMm":round(distance(actual[rail],inward),5)}
            if case["originalAgainstNewInboardFilletMm3"]<195 or case["cutRailToInnerFilletMm"]<1.49:
                raise ValueError(case)
            # All intentional tangent mates are retained; exact material overlap would fail.
            ids=[k for k in actual if k.startswith(tag)]
            for k in ids:
                prove(clearance(actual[k],{n:v for n,v in installed.items() if n!=k}))
            tools={}
            x=sx*312
            for z in (90,130):
                ignore={tag+name+"-"+str(z) for name in ("screw","nut","head-washer","nut-washer")}
                obstacles={k:v for k,v in installed.items() if k not in ignore}
                inward_x = (x-sx*20,x+sx*8)
                # Hex key points into its screw; nut socket seats on the retained nut.
                tools[str(z)]={
                    "inboardDrive":prove(clearance(rod(x,sy,147,219.4,z,4),obstacles)),
                    "inboardHandle":prove(clearance(box(*inward_x,sy*140,sy*147,z-84,z+8),obstacles)),
                    "outboardHandle":prove(clearance(box(*inward_x,sy*259,sy*269,z-84,z+8),obstacles)),
                    "outboardSocket":prove(clearance(rod(x,sy,244.6,269,z,9.5),obstacles)),
                    "retreatScrewHead":prove(clearance(rod(x,sy,188,227.4,z,6.635),obstacles)),
                    "retreatScrewBody":prove(clearance(rod(x,sy,227.4,257.4,z,4),obstacles)),
                    "retreatNutAndWasher":prove(clearance(rod(x,sy,243,269,z,8),obstacles))}
            # A long straight socket is a useful control for the lower access hole.
            control=clearance(rod(x,sy,244.6,315,90,9.5),{k:v for k,v in installed.items()
                              if k not in {tag+"nut-90",tag+"nut-washer-90",tag+"screw-90"}})
            if not control["overlapMm3"]:
                raise ValueError("Long-socket interference control failed")
            case.update(tools=tools,longSocketControl=control)
            results[tag[:-1]]=case
    removed_volume=sum(cad.volume(s)[0]-volume[k] for k,s in originals.items())
    qty={"ears":4,"dowels":4,"fillets":8,"M8 screws":8,"M8 locknuts":8,"M8 washers":16}
    write(out/"fit.json",{"sourceSha256":pins,"stepSha256":digest(step),"sourceCount":len(parts),
          "reopenedBodies":len(actual),"removedRailMm3":removed_volume,"volumeMm3":volume,
          "ends":results,"quantities":qty})


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("artifacts",type=Path)
    p.add_argument("--axle",type=Path,default=ROOT/"axle-retention")
    p.add_argument("--out",type=Path,default=HERE)
    a=p.parse_args()
    run(a.artifacts,a.axle,a.out)
