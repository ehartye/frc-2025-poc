"""Check the new occupied volumes across the registered receiver and both attained docks."""
import argparse
import json
from pathlib import Path

import numpy as np
from sp1_hardware import (ROOT, REMOVED, GUIDE, PIN, Y, Z, box, cad, manufactured, native_box,
                   supplied, xrod)
from clearance import (add_exchange, corrected, digest, dock, heading_sweep, mod,
                       native_pose, poses, read, stationary, wire_boxes)
from field import check as stock_check
from fit import distance, finalize_step, write_step
from rail_profile import revised
from shop_fit import check_translation, linear_travel, swept_box

HERE = Path(__file__).resolve().parent
CABLE = "sp1-live-cable"
MOVING = {"sp1-shelf-clevis","sp1-eye-washer","sp1-eye-shoulder-pin","sp1-eye-locknut","sp1-eye-reservation"}


def scene(artifacts, lamp):
    original = poses(artifacts,lamp)
    receipt = read("drive-side-attachment/fit.json")
    for stroke, group in zip((0,370),original):
        add_exchange(group,ROOT/"drive-side-attachment/inner-rail-joints.step",
                     receipt["volumeMm3"],receipt["stepSha256"])
        group.update(revised(group,stroke))
        for key in REMOVED:
            del group[key]
        group.update(manufactured(stroke))
        group.update(supplied(stroke))
    return original


def scan(start, end):
    start = dict(start)
    rotation = read("design/competition-package/shop-duplex/red/rotation-bounds.json")
    frames = read("design/competition-package/shop-duplex/red/part-frames.json")
    movement = {}
    for key,part in list(start.items()):
        if key in rotation:
            start[key]=native_box(np.array(rotation[key]["bounds_mm"])+[0,0,560,0,0,560])
            movement[key]=185 if frames[key]=="middle" else 0
        else:
            movement[key]=linear_travel(cad.bounds(part),cad.bounds(end[key]))
    fixed,moving = wire_boxes()
    for boxes,travel in ((fixed,0),(moving,370)):
        for key,b in boxes.items():
            start["envelope/"+key]=native_box(b)
            movement["envelope/"+key]=travel
    boxes={k:cad.bounds(v) for k,v in start.items()}
    items={**manufactured(0),**supplied(0)}
    checks=0
    issues={}
    for name,part in items.items():
        for key,other in start.items():
            if name==key or (key in items and key<name):
                continue
            # A varying-length coaxial wire, expressed in the counterpart frame,
            # fills exactly the interval between either endpoint guide and eye neck.
            if CABLE in (key,name):
                static=key if name==CABLE else name
                delta=movement[static]
                if delta is None:
                    raise ValueError("Unbounded relative cable motion: "+static)
                begin=min(GUIDE,GUIDE-delta)
                finish=max(PIN-23.9,PIN+370-23.9-delta)
                shape=xrod(begin,Y,Z,.019*25.4/2,finish-begin)
                obstacle=start[static]
                travel=0
            else:
                shape=part
                obstacle=other
                delta=movement[key]
                if delta is None:
                    b,e=np.array(boxes[key]),np.array(cad.bounds(end[key]))
                    obstacle=native_box(np.r_[np.minimum(b[:3],e[:3]),np.maximum(b[3:],e[3:])])
                    delta=0
                travel=movement[name]-delta
            if dock.separation(swept_box(shape,travel),cad.bounds(obstacle))>1e-6:
                continue
            checks+=1
            hit,unresolved=check_translation(shape,obstacle,travel)
            if hit>.001 or unresolved:
                issues[name+"/"+key]={"commonMm3":hit,"unresolved":unresolved}
    return {"comparisonBodiesAndReservations":len(start),"narrowPairs":checks,"findings":issues}


def clearance(field):
    expected=read("selected-match/corrected-clearance.json")
    if digest(field)!=expected["fieldSha256"]:
        raise ValueError("Field registration changed")
    # Split the high shelf flange from the narrower low leg, exactly as for the
    # earlier field check. Preserve all existing motion, heading and field pads.
    geom={**manufactured(0),**supplied(0)}
    sweeps={}
    for name,shape in geom.items():
        if name=="sp1-shelf-clevis":
            bounds=[(69,Y-5.25,410,81,Y+5.25,481),(75,-133,481,119,-121,485)]
        else:
            bounds=[cad.bounds(shape)]
        for i,b in enumerate(bounds):
            if name in MOVING or name==CABLE:
                shift=np.array([-15,0,0,385,0,0])
            else:
                shift=0
            sweeps[f"{name}/{i}"]=heading_sweep(np.array(b)+shift,1.)
    expanded=np.array([-11.526342,-11.526342,-1.526342,11.526342,11.526342,1.526342])
    shapes={k:native_box(np.array(bounds)+expanded) for k,bounds in sweeps.items()}
    reef_ops=mod("sp1_fixed_reef",ROOT/"moving-observer/clearance.py")
    reef=reef_ops.reef_parts(field)
    stock=read("full-match/installed-stock.json")["stock"]
    layout=corrected()
    result={}
    for name,sector in (("A",-.18),("B",.22)):
        structure=reef_ops.find_min(reef,native_pose(shapes,sector*1000))
        occupants=stock_check(layout,stock,sweeps,stationary(layout.routes(sector)[1].end))
        result[name]={"structure":structure,"stockLowerMm":min(v["lower_bound_mm"] for v in occupants.values())-11.526342}
    return result


def run(artifacts, lamp, field, out):
    if digest(lamp)!=read("selected-match/kit.json")["lampSha256"]:
        raise ValueError("Unexpected signal light")
    start,end=scene(artifacts,lamp)
    from service_fit import run as inspect_service
    installation=inspect_service(start,end)
    print("Installation",installation,flush=True)
    checked=scan(start,end)
    print("Scan",checked,flush=True)
    parts=manufactured(0)
    out.mkdir(parents=True,exist_ok=True)
    step=out/"sp1-mount.step"
    write_step(step,parts)
    finalize_step(step)
    reopened=cad.read_named(step,[{"id":k} for k in parts])
    volumes={k:cad.volume(s)[0] for k,s in reopened.items()}
    if max(abs(volumes[k]-cad.volume(parts[k])[0]) for k in parts)>.001:
        raise ValueError("STEP volume mismatch")
    old=read("selected-match/kit.json")
    controls={"unboredOldTabEyeMm3":cad.intersection(reopened["sp1-eye-shoulder-pin"],box(75,-129,412,81,-125,485)),
              "unboredOldTabCableEndDistanceMm":distance(supplied(0)["sp1-live-cable"],box(75,-129,412,81,-125,485))}
    result={"source":{f:digest(ROOT/f) for f in ("selected-match/kit.json","selected-match/corrected-clearance.json","sp1-mount/sp1_hardware.py","sp1-mount/service_fit.py","sp1-mount/check.py")},
            "priorKitSha256":old["kitSha256"],"fieldSha256":digest(field),"stepSha256":digest(step),
            "volumeMm3":volumes,"supplierExcluded":sorted(supplied(0)),
            "nativeScan":checked,"installation":installation,"oldTargetControl":controls,"registeredDocks":clearance(field)}
    (out/"fit.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n",encoding="utf8",newline="\n")
    print("Docks",result["registeredDocks"],flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ("artifacts","lamp","field","output"):
        parser.add_argument(key,type=Path)
    args=parser.parse_args()
    run(args.artifacts,args.lamp,args.field,args.output)
