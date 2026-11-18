"""Original packaging envelopes in the robot frame. Export/review without vendor CAD."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
for d in ["design/station-l1","design/drivetrain","design/receiver-lift","release-observer"]:
    sys.path.insert(0,str(ROOT/d))
import build_assembly as cad
from mechanical import box,cylinder
from drive_parts import parts as drivetrain
from check_routing import routing,clashes,write_step
from build_mount import parts as observer_parts

REPLACED={"battery","battery-anchor-L","battery-anchor-R","battery-restraint","protected-distribution","controller-radio",
          "optical-seat","optical-entry"}
Z=[0,0,1]


def layout(pose):
    # Footprints of terminals, mating plugs and sprung motion are distinct from enclosure extents.
    z=pose["height"]
    x=pose["stroke"]
    v={
        "np18-battery":box(-250,-105,155,76,181,167),
        "battery-cheek-left":box(-251,76,155,78,3,167),
        "battery-cheek-right":box(-251,-108,155,78,3,167),
        "battery-hold-down":box(-224,-108,322,24,187,5),
        "battery-side-stop-in":box(-253,-105,155,3,181,10),
        "battery-side-stop-out":box(-174,-105,155,3,181,10),
        "distribution-bridge":box(-328,97,158,232,116,3),
        "rev-pdh":box(-325,100,161,225.425,111.125,39.70),
        "main-breaker":box(-322,-158,155,85,43,46),
        "front-electronics-platform":box(1,-211,290,336,422,3),
        "viair-90c":box(35,-66,154,150.88,53.60,115.07),
        "tank-avt-pp-35":cylinder(274,-155,195,[0,1,0],35.4,310.2),
        "tank-saddle-front":box(255,-124,154,38,8,5.6),
        "tank-saddle-rear":box(255,118,154,38,8,5.6),
        "rio2-envelope":box(165,-198,293,160,157,54),
        "vh109-reserve":box(181,32,293,145,112,53),
        "rev-ph":box(3,145,293,111.125,47.625,19.05),
        "regulated-pawl-valve":box(6,44,293,117,67,22),
        "gauge-regulator-dump-reserve":box(7,97,154,108,83,112),
        "rsl-reserve":cylinder(-47,221,1020,Z,11,39),
        "shelf-harness-moving-anchor":box(x+75,-45,z-90,16,10,15),
        "lift-harness-moving-anchor":box(-47,189,z-143,15,9,15),
        "harness-fixed-anchor":box(-12,190,154,13,10,15),
    }
    for key,xx,yy in [("lift",5,-204),("slide",82,-204),("left-grip",5,-154),("right-grip",82,-154)]:
        v["spark-"+key]=box(xx,yy,293,69.3,34.5,25)
    for tag,u in [("seat",40),("entry",100)]:
        v["qtrx-"+tag]=box(x+u-2.5,-5,z-86.4,5,20,4.4)
    for i,(xx,yy) in enumerate([(2,-210),(328,-210),(2,201),(328,201)]):
        v["platform-post-"+str(i)]=box(xx,yy,154,8,8,136)
    return v


def electrical_parts(pose):
    if isinstance(pose,str):
        pose=cad.MODEL["poses"][pose]
    return {k:cad.brep(v) for k,v in layout(pose).items()}


def service_volumes():
    return [
        ("battery-after-restraint-removal",[-250,-105,322,76,181,300],["battery-hold-down"]),
        ("fuse-and-breaker-extraction",[-325,100,200.71,225.425,111.125,55],[]),
        ("main-disconnect-access",[-322,-158,201.01,85,43,65],[])]


def evidence(step_dir=None):
    receipt={"method":"OCP native intersections above 0.001 mm3 on named original envelopes",
             "replaced":sorted(REPLACED),"poses":{}}
    optics=observer_parts()
    for name,p in cad.MODEL["poses"].items():
        path=ROOT/"design/drivetrain/review"/(name+".step")
        previous=cad.read_named(path,drivetrain())
        other={k:v for k,v in previous.items() if k not in REPLACED and k!="lift-drive"}
        other.update({"lift/"+k:v for k,v in routing(p["height"]).items()})
        other.update({"observer/"+k:(cad.translated(v,(p["stroke"]-370,0,p["height"]-560))
                       if k.startswith("moving-pad") else v) for k,v in optics.items()})
        new=electrical_parts(p)
        defects=clashes(new,other)
        for i,(k,v) in enumerate(new.items()):
            defects.update(clashes({k:v},dict(list(new.items())[:i])))
        service={}
        for key,flat,detached in service_volumes():
            service[key]=clashes({"service":cad.brep(box(*flat))},{k:v for k,v in {**other,**new}.items() if k not in detached})
        receipt["poses"][name]={"sourceSha256":hashlib.sha256(path.read_bytes()).hexdigest(),
                                "newBodies":sorted(new),"collisionMm3":defects,"serviceMm3":service}
        if step_dir:
            step_dir.mkdir(exist_ok=True,parents=True)
            write_step(step_dir/(name+".step"),new)
    return receipt


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step-dir",type=Path)
    args=parser.parse_args()
    r=evidence(args.step_dir)
    (HERE/"packaging.json").write_text(json.dumps(r,indent=2)+"\n")
    for name,p in r["poses"].items():
        print(name,p["collisionMm3"],p["serviceMm3"])
