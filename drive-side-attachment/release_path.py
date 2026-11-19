"""Check the stripped rail withdrawal with fixed ears and pilots left on both crossrails."""
import argparse
from pathlib import Path

from joint import HERE, ROOT, box, cad, clearance, digest, load_source, prove, read, write


def swept_hits(shapes,obstacles):
    result={}
    for sh in shapes:
        for k,v in clearance(sh,obstacles)["overlapMm3"].items():
            result[k]=max(result.get(k,0),v)
    return result


def run(artifacts,axle,out):
    source,pins=load_source(artifacts,axle)
    fit=read(out/"fit.json")
    step=out/"inner-rail-joints.step"
    if fit["sourceSha256"]!=pins or fit["stepSha256"]!=digest(step):
        raise ValueError("Reopen the original issued rail and axle kit")
    installed={**source,**cad.read_named(step,[{"id":k} for k in fit["volumeMm3"]])}
    cases={}
    for side,sy in (("L",1),("R",-1)):
        rail="drive-rail-inner-"+side
        stock=installed[rail]
        # The rail has a constant drilled profile across its thickness. Three
        # overlapping placements spaced 4 mm cover its continuous 8 mm retreat.
        retract=[cad.translated(stock,[0,-sy*d,0]) for d in (0,4,8)]
        # Once the pilots are clear the gross, unperforated profile bounds every
        # intermediate location throughout the 180 mm drop, including both ends.
        lowering=box(-326,326,sy*221,sy*227,-130,150)
        removed={k for k in fit["volumeMm3"] if k.startswith(("front-"+side+"-","rear-"+side+"-"))
                 and any(a in k for a in ("-screw-","-nut-","-washer-"))}
        tabs=[f"takeup-tab-{e}-inner-{side}" for e in ("front","rear")]
        jacks=[f"jack-{e}-inner-{side}" for e in ("front","rear")]
        legs=[lowering]
        for tab in tabs:
            a,b,c,d,e,f=cad.bounds(installed[tab])
            retract.append(box(a,d,min(b,b-8*sy),max(e,e-8*sy),c,f))
            legs.append(box(a,d,b-8*sy,e-8*sy,c-180,f))
        obstacles={k:v for k,v in installed.items() if k!=rail and k not in removed|set(tabs)|set(jacks)}
        controls=swept_hits([*retract,*legs],obstacles)
        # These detachable families are attached to the selected side only. A
        # new obstruction elsewhere cannot disappear into the disassembly list.
        planned={f"axle-{pos}-{side}" for pos in ("front","center","rear")}
        planned.update({f"gearbox-{side}",f"gearbox-face-screw-low-{side}",f"gearbox-face-screw-high-{side}"})
        planned.update(k for k in obstacles if k.startswith("inner-"+side+"-"))
        planned.update("bearing-"+pos+"-inner-"+side for pos in ("front","center","rear"))
        if not set(controls)<=planned:
            raise ValueError("Unplanned stationary obstruction: "+str(set(controls)-planned))
        staged={k:v for k,v in obstacles.items() if k not in planned}
        for shape in retract:
            prove(clearance(shape,staged))
        drop=[prove(clearance(shape,staged)) for shape in legs]
        # Dropping before releasing the dowels puts their steel in the hole wall.
        wrong=swept_hits([cad.translated(stock,[0,0,-3])],{k:v for k,v in staged.items()
                          if k.endswith("-"+side+"-pilot")})
        if len(wrong)!=2 or not controls:
            raise ValueError("Missing negative controls")
        cases[side]={"jointHardwareRemoved":sorted(removed),"attachedTabsTraveling":tabs,"removedJacks":jacks,"stagedDrivetrainWitnesses":controls,
                     "withdrawInboardMm":8,"downMm":180,"loweredEnvelope":drop,
                     "dropBeforeUnpinningMm3":wrong}
    write(out/"rail-service.json",{"candidateSha256":digest(step),"sourceSha256":pins,"ends":cases})


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("artifacts",type=Path)
    p.add_argument("--axle",type=Path,default=ROOT/"axle-retention")
    p.add_argument("--out",type=Path,default=HERE)
    a=p.parse_args()
    run(a.artifacts,a.axle,a.out)
