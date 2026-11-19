"""Reopen the deck joint and screen every installed or serviced station in robot millimetres."""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
from kit import AXES, CHANGED, HERE, ROOT, REMOVAL, bodies, cylinder, integral, j, name, overlay
sys.path.insert(0,str(ROOT/"fixed-camera-route"))
import scene
import hardware
from paths import routes, points


def bounds_union(a,b):
    return [min(a[i],b[i]) if i<3 else max(a[i],b[i]) for i in range(6)]


def broad_box(bounds):
    x,y,z,X,Y,Z=bounds
    return j.box(x,X,y,Y,z,Z)


def joined(a,b):
    # Use a compound when both solids only share a face or are disjoint.
    from OCP.BRep import BRep_Builder
    from OCP.TopoDS import TopoDS_Compound
    builder=BRep_Builder()
    assembly=TopoDS_Compound()
    builder.MakeCompound(assembly)
    for item in (a,b):
        builder.Add(assembly,item)
    return assembly


def source(artifacts,lamp,cover):
    start,end=scene.get(artifacts,lamp)
    counts={"originalOpticalAndReceiver":len(start)}
    rotation=j.read(ROOT/"design/competition-package/shop-duplex/red/rotation-bounds.json")
    frames=j.read(ROOT/"design/competition-package/shop-duplex/red/part-frames.json")
    for k in rotation.keys() & start.keys():
        ab=np.array(rotation[k]["bounds_mm"])+[0,0,560,0,0,560]
        start[k]=broad_box(ab)
        end[k]=broad_box(ab+([185,0,0,185,0,0] if frames[k]=="middle" else 0))
    mobile=0
    for k in start:
        a,b=j.cad.bounds(start[k]),j.cad.bounds(end[k])
        if max(abs(x-y) for x,y in zip(a,b))>.001:
            start[k]=broad_box(bounds_union(a,b))
            mobile+=1
    counts["movingSweepBoxes"]=mobile
    pins={}
    def add(path,keys,expected):
        if j.digest(path)!=expected:
            raise ValueError("Unexpected source "+path.name)
        pins[path.name]=expected
        start.update(j.cad.read_named(path,[{"id":k} for k in keys]))
    labels=("groove","fillet","bottom-cap","top-cap")
    corners=[f"corner-{e}-{s}-{n}" for e in ("front","rear") for s in ("L","R") for n in labels]
    add(ROOT/"frame-corners/frame-weldment.step",["cross-front","cross-rear","drive-rail-outer-L","drive-rail-outer-R",*corners],j.read(ROOT/"frame-corners/weld-fit.json")["stepSha256"])
    repack=j.read(ROOT/"chain-covers/study.json")["repack"]
    add(ROOT/"chain-covers/front-loop-repack.step",repack["volumeChangesMm3"],repack["stepSha256"])
    record=j.read(cover/"fit.json")
    add(cover/"supported-covers.step",record["volumeMm3"],record["stepSha256"])
    fixed=j.read(ROOT/"fixed-camera-route/guides.json")
    add(ROOT/"fixed-camera-route/harness-guides.step",fixed["volumeMm3"],fixed["stepSha256"])
    host,_=hardware.solids()
    start.update({"fixed-optical/"+k:v for k,v in {**host,**hardware.service()}.items()})
    # Each solid box encloses a continuous circular/straight segment plus jacket, including
    # the <1 mm gap between sampled centerline points, with generous additional 1 mm inflation.
    for key,path in routes()[0].items():
        for i,(_,line,_) in enumerate(path):
            radius=hardware.RADIUS[key]+1.
            start[f"jacket/{key}/{i}"]=broad_box(np.r_[line.min(axis=0)-radius,line.max(axis=0)+radius])
    guide=j.read(ROOT/"station-guide/fit-narrow.json")
    for k,b in guide["boundsMm"].items():
        start["optional/"+k]=broad_box(bounds_union(b,[v+(370 if i in (0,3) else 0) for i,v in enumerate(b)]))
    cap=j.read(ROOT/"fixed-lift-omission/fit.json")
    add(ROOT/"fixed-lift-omission/input-cap.step",["winch-input-cap"],cap["capSha256"])
    # Keep motor in the obstacle registry as an additional bound on the retained-motor choice.
    counts["sourceAndConservativeReservations"]=len(start)
    return start,counts,pins


def gauges(x,y,steel):
    s=math.copysign(1,x)
    center=s*340
    stem=cylinder(x,y,46-REMOVAL,2,117.5+REMOVAL)
    head=cylinder(x,y,42-REMOVAL,3.5,4+REMOVAL)
    trunk=cylinder(x,y,50-110,4,97.95+110)
    foot=j.box(center-9.75,center+9.75,y-11,y+11,46-110,50)
    # Upper 7 AF deep socket approaches from above. A straight short seating stroke
    # follows horizontal entry over the bumper. 12 mm OD also encloses the raised nut.
    r=6 if steel else 6.5
    socket=cylinder(x,y,154.8,r,64.)
    upper=j.box(x-r*s,x+111*s,y-r,y+r,197,218.8)
    # Outside-reaching compact sliding handle, 10 mm thick and 12 mm wide.
    # Angle +/-12 degrees is bounded for the first 45 mm of handle.
    handle=j.box(x,x+45*s,y-16,y+16,192,218.8)
    if not steel:
        # Tool stack: 40 mm from socket mouth to handle underside, then 8 mm drive.
        # Translate from outside at Z197, then lower 42 mm at its working axis.
        socket=cylinder(x,y,154.8,r,90.2)
        upper=j.box(x-r*s,x+111*s,y-r,y+r,197,245)
        handle=j.box(x,x+45*s,y-16,y+16,194.8,245)
        handle=joined(handle,j.box(x,x+156*s,y-16,y+16,237,245))
    top={"topSocket":socket,"socketEntry":upper,"handle":handle}
    if steel:
        return {**top,"screwWithdrawal":joined(stem,head),"sleeveWithdrawal":joined(trunk,foot),
                "bottomKey":cylinder(x,y,-46,2.25,88)}
    # A forked plate holds the pair of flange flats during nut service. Bound the open
    # spanner by its full outer rectangle, and approach from below on a supported frame.
    wedge=joined(j.box(center-8,center+8,y-15,y-11.15,-64,49.8),
                 j.box(center-8,center+8,y+11.15,y+15,-64,49.8))
    exterior=j.box(center-8,center+8,y-15,y+65,-64,45.5)
    lip=joined(j.box(s*328,s*331,y-11,y+11,46-REMOVAL,50),
               j.box(s*328,s*329.8,y-11,y+11,49-REMOVAL,53))
    anchor=joined(lip,joined(cylinder(x,y,147.95-REMOVAL,2.5,16.05+REMOVAL),
                  joined(cylinder(x,y,50-REMOVAL,4,97.95+REMOVAL),
                         j.box(center-9.75,center+9.75,y-11,y+11,46-REMOVAL,50))))
    return {**top,"anchorWithdrawal":anchor,"bottomHoldingFork":joined(wedge,exterior)}


def screen(part,obstacles,boxes):
    bound=j.cad.bounds(part)
    overlapping=[k for k,b in boxes.items() if j.lower_bound(bound,b)<1e-5]
    hits={}
    for k in overlapping:
        v=j.cad.intersection(part,obstacles[k])
        if v>.001: hits[k]=round(v,4)
    # Exact nearest for potentially limiting neighbours; all others have a larger AABB bound.
    nearest=(float("inf"),None)
    for lower,k in sorted((j.lower_bound(bound,b),k) for k,b in boxes.items()):
        if lower>nearest[0]+.0001: break
        nearest=min(nearest,(j.distance(part,obstacles[k]),k))
    return {"nearest":[round(nearest[0],4),nearest[1]],"overlapMm3":hits}


def run(artifacts,lamp,cover,steel):
    src,counts,pins=source(artifacts,lamp,cover)
    generated=overlay(src,steel)
    step=HERE/("steel-control.step" if steel else "deck-through-ties.step")
    j.write_step(step,generated)
    j.finalize_step(step)
    parts=j.cad.read_named(step,[{"id":k} for k in generated])
    volumes={k:round(j.cad.volume(v)[0],5) for k,v in parts.items()}
    if max(abs(volumes[k]-j.cad.volume(v)[0]) for k,v in generated.items())>.001:
        raise ValueError("Changed STEP geometry")
    deck=parts["tie-machined-deck-main"]
    allparts={**{k:v for k,v in src.items() if k not in CHANGED},**parts}
    boxes={k:j.cad.bounds(v) for k,v in allparts.items()}
    def check(shape,omit):
        return screen(shape,allparts,{k:b for k,b in boxes.items() if k not in omit})
    cases={}
    for x,y in AXES:
        tag="deck-tie-"+name(x,y)+"-"
        rail=parts["tie-machined-cross-"+("front" if x>0 else "rear")]
        own={k for k in parts if k.startswith(tag)}
        inspect={k:check(parts[k],{k}) for k in sorted(own)}
        service={}
        for k,v in gauges(x,y,steel).items():
            # The six stations may be undone individually, leaving other five installed.
            omit=own.copy()
            if k=="screwWithdrawal": omit.remove(tag+"sleeve")
            if k=="bottomHoldingFork": omit.remove(tag+"anchor")
            service[k]=check(v,omit)
        witness={}
        def press(label,a,b,delta):
            witness[label]=round(j.cad.intersection(j.cad.translated(a,delta),b),4)
        press("deckToWasher",deck,parts[tag+"washer"],[0,0,.2])
        press("washerToNut",parts[tag+"washer"],parts[tag+"locknut"],[0,0,.2])
        if steel: press("headToFoot",parts[tag+"screw"],parts[tag+"sleeve"],[0,0,.2])
        press("footToCrossrail",parts[tag+("sleeve" if steel else "anchor")],rail,[0,0,.2])
        if not steel:
            from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
            from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf
            for angle in (-2.,2.):
                twist=gp_Trsf()
                twist.SetRotation(gp_Ax1(gp_Pnt(x,y,50),gp_Dir(0,0,1)),math.radians(angle))
                turned=BRepBuilderAPI_Transform(parts[tag+"anchor"],twist,True).Shape()
                witness["keyAt"+str(angle)+"Deg"]=round(j.cad.intersection(turned,rail),4)
        press("stemToDeckInShear",parts[tag+("screw" if steel else "anchor")],deck,[.5,0,0])
        if min(witness.values())<=0: raise ValueError(witness)
        cases[name(x,y)]={"parts":inspect,"service":service,"pressedCommonMm3":witness}
    # Move only the fastener to the previous proposed station at Y=175;
    # its solid sleeve would cross the existing bumper attachment.
    trial=bodies(339.,175.)["sleeve"]
    neg=check(trial,{k for k in allparts if not k.startswith("bumper")})
    rejected_nut=bodies(-339.,-72.)["locknut"]
    rejected_race=j.cad.intersection(rejected_nut,allparts["observer-rear-cable-raceway"])
    if rejected_race<50: raise ValueError("Missing cable raceway conflict")
    output={"sources":pins,"counts":counts,"rejectedRearMinus72NutRacewayMm3":rejected_race,
            "stepSha256":j.digest(step),"volumeMm3":volumes,
            "cutVolumeMm3":{k:round(j.cad.volume(src[k])[0]-volumes["tie-machined-"+k],5) for k in CHANGED},
            "axesMm":AXES,"stations":cases,"misplacedSleeve":neg,"unmeasuredInstalledFit":True}
    j.write(HERE/("steel-control.json" if steel else "fit.json"),output)
    failures=[(station,stage,key,row) for station,station_data in cases.items()
              for stage in ("parts","service") for key,row in station_data[stage].items() if row["overlapMm3"]]
    if failures or not neg["overlapMm3"]: raise ValueError([failures,neg])


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for key in ("artifacts","lamp","cover"):
        p.add_argument(key,type=Path)
    p.add_argument("--steel-control",action="store_true")
    a=p.parse_args()
    run(a.artifacts,a.lamp,a.cover,a.steel_control)
