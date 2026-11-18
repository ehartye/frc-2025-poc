"""Source-bound mass and load transfer for the selected assembled revision."""
import json
from pathlib import Path
import sys

from assembly import HERE,ROOT,LIFT,SHOP,cad,pose_layers,sha,original
from build_successor import write_json
sys.path.insert(0,str(ROOT/"design/stability"))
import stability


def norm_box(center):
    return tuple(center)+tuple(center)


def additions(pose,dimensions,shapes):
    lift=json.loads((LIFT/"delivery-retention.json").read_text())
    meta=json.loads((LIFT/"duplex-inventory.json").read_text())["poses"][pose]
    shop=json.loads((SHOP/"native-review.json").read_text())["incrementalMassByFrame"]
    rows=[]
    for frame,info in shop.items():
        cg=list(info["stowedCgMm"])
        if frame=="slide":
            cg[0]+=dimensions["stroke"]
            cg[2]+=dimensions["height"]-940
        keys=[k for k in shapes if k.startswith(("tapped-anchor-","drilled-slide-","pin-"))] if frame=="slide" else [
            k for k in shapes if k[:6] in ("front-","rear-L","rear-R","center") and ("-washer-" in k or "-screw-" in k or k.endswith("-nut-saddle"))]
        ext=stability.union([cad.bounds(shapes[k]) for k in keys])
        rows.append(stability.Load("Shop delta "+frame,(info["deltaKg"],)*2,norm_box(cg),ext))
    for frame,budget in [("base",lift["candidateFixedAddedKg"]),("lift",lift["candidateMovingAddedKg"])]:
        all_boxes=[]
        for key,mass in budget.items():
            name="long-common-shaft" if key=="longer-steel-hex-shaft" else key
            box=cad.bounds(shapes[name])
            all_boxes.append(box)
            center=norm_box(meta[name]["centroidMm"])
            if meta[name]["frame"]=="variable-rope":
                # A fraction may be wound on the drum at this height. Permit both its
                # complete path and the drum position under the maximum rope mass.
                side=name.split("-")[1]
                box=stability.union([box,cad.bounds(shapes["backup-"+side+"-drum-core"])])
                center=box
            rows.append(stability.Load(key,(mass,mass),center,box))
        allowance=lift["additionalFixedHardwareAllowanceKg" if frame=="base" else "additionalMovingTerminationAllowanceKg"]
        box=stability.union(all_boxes)
        rows.append(stability.Load("Unplaced duplex "+frame,(0,allowance),box,box))
    return rows


def report():
    result={"sourceSha256":{},"poses":{}}
    for p in [ROOT/"design/stability/review.json",ROOT/"design/stability/stability.py",SHOP/"native-review.json",
              LIFT/"delivery-retention.json",LIFT/"duplex-inventory.json",Path(__file__)]:
        result["sourceSha256"][p.relative_to(ROOT).as_posix()]=sha(p)
    for pose,desc in cad.MODEL["poses"].items():
        old,_=stability.loads_for(pose)
        shapes=original.flatten(pose_layers(desc["height"],desc["stroke"])[0])
        increments=additions(pose,desc,shapes)
        total=old+increments
        empty=stability.analyze(total)
        seated=stability.analyze(total+[stability.payload(pose)])
        # The same weighed robot is used for both limits. Remove its one battery
        # and the corresponding bumper allowance at the same mass endpoint.
        lo,hi=empty["mass_kg"]
        r103=[lo-6.17-5.9,hi-6.17-7.5]
        r408=[lo-6.17,hi-6.17]
        result["poses"][pose]={"added_kg":[sum(x.mass[i] for x in increments) for i in [0,1]],
                               "empty":empty,"seated":seated,"R103_kg":r103,"R408_kg":r408,
                               "R103_margin_at_high_kg":115*.45359237-r103[1],
                               "R408_margin_at_high_kg":135*.45359237-r408[1]}
    prior_lift=json.loads((LIFT/"delivery-retention.json").read_text())
    slide_delta=json.loads((SHOP/"native-review.json").read_text())["incrementalMassByFrame"]["slide"]["deltaKg"]
    result["liftLoadedUpperKg"]=prior_lift["candidateLoadedUpperKg"]+slide_delta
    result["weighedKg"]=None
    return result


if __name__=="__main__":
    write_json(HERE/"mass.json",report())
