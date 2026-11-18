"""Rebuild the native kit, inspect both alliances, and check the actual CAD stack."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
from bumper import (cad, HERE, ROOT, AXIS_Z, FRAME, FOAM, HALF_SEAM, LOW, HIGH, BACK,
                    REPLACED, anchors, brick, bumper_parts, export_step, machined_rails, specifications)

for directory in ["design/receiver-lift","design/electrical-inspection","design/receiver-drives","release-observer"]:
    sys.path.insert(0,str(ROOT/directory))
import package as electrical
import receiver_drives as grip
from drive_parts import parts as drivetrain
from check_routing import clashes, routing
from build_mount import parts as observer_parts


def foreign_shapes(name):
    pose = cad.MODEL["poses"][name]
    source = ROOT/"design/drivetrain/review"/(name+".step")
    baseline = cad.read_named(source, drivetrain())
    baseline.update(machined_rails(baseline))
    superseded = electrical.REPLACED | grip.REPLACES | REPLACED | {"lift-drive"}
    rest = {k:v for k,v in baseline.items() if k not in superseded}
    rest.update(electrical.electrical_parts(pose))
    rest.update(grip.added_shapes(pose))
    rest.update({"lift-"+k:v for k,v in routing(pose["height"]).items()})
    rest.update({"observer-"+k:(cad.translated(v,(pose["stroke"]-370,0,pose["height"]-560))
                 if k.startswith("moving-pad") else v) for k,v in observer_parts().items()})
    return rest, hashlib.sha256(source.read_bytes()).hexdigest()


def removal_sweep(end):
    # These boxes bound *every* position of the U strip as it slides 440 mm along X.
    # Round heads/washers are removed before pulling. Crush sleeves remain fixed.
    x0, x1 = end*(HALF_SEAM-.5), end*865
    return {"end":cad.brep(brick(end*FRAME,x1,-FRAME,FRAME,LOW-.5,HIGH+.5)),
            "left":cad.brep(brick(x0,x1,FRAME,425,LOW-.5,HIGH+.5)),
            "right":cad.brep(brick(x0,x1,-425,-FRAME,LOW-.5,HIGH+.5))}


def access(mount):
    # A right-angle bit on a stand. Upper chassis deck prevents reaching end screws from above.
    inside = FRAME-mount.width
    swept = mount.cylinder(9.5, inside-68, inside)
    r = inside-24
    if mount.face == "end":
        x,y = mount.sign*r,mount.tangent
    else:
        x,y = mount.tangent,mount.sign*r
    return {"withdraw-and-drive":cad.brep(swept),
            "bottom-entry":cad.brep(brick(x-10,x+10,y-10,y+10,0,AXIS_Z+8))}


def mass_rows(shapes,alliance):
    labels = specifications(alliance)
    # The 0.5 mm cloth shell represents a nominal textile areal mass of 350 g/m².
    density = {"alloy":2700, "steel":7850, "foam":37, alliance:700}
    rows=[]
    for key,shape in shapes.items():
        group=labels[key][0]
        volume,center=cad.volume(shape)
        rows.append({"solid":key,"assignment":"chassis" if "sleeve" in key else "removable",
                     "material":group,"densityKgM3":density[group],"volumeMm3":round(volume,4),
                     "massKg":round(volume*density[group]*1e-9,7),
                     **{axis:round(value,5) for axis,value in zip(("cgXmm","cgYmm","cgZmm"),center)}})
    return rows


def write_mass(path,rows):
    with path.open("w",newline="",encoding="utf-8") as out:
        writer=csv.DictWriter(out,fieldnames=list(rows[0]),lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def kit_review(shapes,alliance):
    mass = {r["solid"]:r["massKg"] for r in mass_rows(shapes,alliance)}
    solids = list(shapes.items())
    hits = {}
    for idx,(key,shape) in enumerate(solids):
        hits.update(clashes({key:shape},dict(solids[:idx])))
    # Completely filled corner samples extend 57.15 mm outboard on BOTH axes.
    corner_deficits = {}
    for f,label in [(1,"front"),(-1,"rear")]:
        for side in [-1,1]:
            prism = cad.brep(brick(f*BACK,f*(BACK+57.15),side*BACK,side*(BACK+57.15),LOW,HIGH))
            corner_deficits[label+str(side)] = round(abs(cad.volume(prism)[0]-cad.intersection(prism,shapes["bumper-foam-"+label])),5)
    return {"solidCount":len(shapes),"internalOverlapMm3":hits,"cornerVoidMm3":corner_deficits,
            "installedKitKg":round(sum(mass.values()),3),
            "frameSleevesKg":round(sum(v for k,v in mass.items() if "sleeve" in k),3)}


def run(out):
    out.mkdir(exist_ok=True,parents=True)
    result={"alliances":{},"poses":{}, "radialCornerMaximumMm":round(math.hypot(75,67),4)}
    for alliance in ["red","blue"]:
        parts = bumper_parts(alliance)
        reopened = export_step(out/(alliance+".step"),alliance,parts)
        result["alliances"][alliance]=kit_review(reopened,alliance)
        if alliance=="blue":
            write_mass(HERE/"mass.csv",mass_rows(reopened,alliance))
    for name in cad.MODEL["poses"]:
        rest,sha=foreign_shapes(name)
        pose={"sourceSha256":sha,"foreignBodies":len(rest),"overlapMm3":clashes(reopened,rest)}
        if name == "stowed":
            sleeves={k:v for k,v in reopened.items() if "sleeve" in k}
            pose["moduleWithdrawalMm3"]={}
            for end,label,other in [(1,"front","rear"),(-1,"rear","front")]:
                opposite={k:v for k,v in reopened.items() if other in k}
                pose["moduleWithdrawalMm3"][label]=clashes(removal_sweep(end),rest|sleeves|opposite)
            pose["fastenerAccessMm3"]={}
            for a in anchors():
                ignored={"bumper-screw-"+a.tag,"bumper-washer-"+a.tag}
                others = rest|{k:v for k,v in reopened.items() if k not in ignored}
                pose["fastenerAccessMm3"][a.tag]=clashes(access(a),others)
        result["poses"][name]=pose
    print(json.dumps(result,indent=2))
    (HERE/"review.json").write_text(json.dumps(result,indent=2)+"\n")
    failures = [p["overlapMm3"] for p in result["poses"].values()]
    for group in ["moduleWithdrawalMm3","fastenerAccessMm3"]:
        failures.extend(result["poses"]["stowed"][group].values())
    for a in result["alliances"].values():
        failures.extend([a["internalOverlapMm3"], any(v>.001 for v in a["cornerVoidMm3"].values())])
    if any(failures):
        raise ValueError("Native review found interference or corner void")


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out",type=Path,default=ROOT/"artifacts/bumper-kit")
    run(parser.parse_args().out)
