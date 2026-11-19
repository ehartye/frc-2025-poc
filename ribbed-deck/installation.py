"""Reopen the mounted panel against the source robot and the six new deck ties."""
import argparse
import importlib.util
from pathlib import Path
import sys
from deck import HERE, ROOT, PART, shop

def run(artifacts,lamp,tie_project):
    sys.path.insert(0,str(tie_project))
    spec=importlib.util.spec_from_file_location("deck_tie_fit",tie_project/"fit.py")
    ties=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ties)
    assembly,counts,pins=ties.source(artifacts,lamp,ROOT/"cover-retention")
    report=shop.read(tie_project/"fit.json")
    step=tie_project/"deck-through-ties.step"
    data=shop.read(HERE/"geometry.json")
    candidate=HERE/"ribbed-deck.step"
    if shop.digest(candidate)!=data["stepSha256"] or shop.digest(step)!=report["stepSha256"]:
        raise ValueError("Exchange changed")
    installed=shop.cad.read_named(step,[{"id":k} for k in report["volumeMm3"]])
    plate=shop.cad.read_named(candidate,[{"id":PART}])[PART]
    del assembly["deck-main"]
    # The tie-machined stock replaces both end tubes and the previous whole deck.
    del assembly["cross-front"],assembly["cross-rear"]
    for k,v in installed.items():
        if k!="tie-machined-deck-main":
            assembly[k]=v
    reference=installed["tie-machined-deck-main"]
    outside=shop.cut(plate,reference)
    # Bound the complete new volume at once, including nominal machining reserve.
    envelope=shop.box(-302.5,302.5,-227,227,130.,150.0001)
    if shop.cad.volume(shop.cut(outside,envelope))[0]>.002:
        raise ValueError("New material exceeds the audited underside envelope")
    env_bounds=shop.cad.bounds(envelope)
    native={}
    for k,v in assembly.items():
        lb=shop.lower_bound(env_bounds,shop.cad.bounds(v))
        if lb<.001:
            gap=shop.distance(envelope,v)
            vol=shop.cad.intersection(envelope,v) if gap<.001 else 0.
            native[k]={"gapMm":round(gap,5),"overlapMm3":round(vol,5)}
    hits={k:r for k,r in native.items() if r["overlapMm3"]>.01}
    if hits:
        print("Broad control overlaps",hits)
    study=shop.clearance(outside,assembly)
    if study["overlapMm3"]:
        raise ValueError(study)
    service={}
    for x,y in ties.AXES:
        for name,tool in ties.gauges(x,y,False).items():
            key=f"{x:g}/{y:g}/{name}"
            service[key]=round(shop.distance(outside,tool),4)
    if min(service.values())<=0:
        raise ValueError("Added stock obstructs a deck tie service tool")
    # The shelf and attachments still have all coplanar land area from the tied plate.
    # Each expected footprint comes from the source registry, expanded 4 mm in plan.
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
    removed=shop.cut(reference,plate)
    checks={}
    for name,(x,xx,y,yy) in data["upperBearingLands"].items():
        pad=shop.box(x,xx,y,yy,150.001,153.999)
        lost=BRepAlgoAPI_Common(removed,pad).Shape()
        checks[name]=round(shop.cad.volume(lost)[0],5)
    if any(v>.002 for v in checks.values()):
        raise ValueError("A contact foot loses supporting material")
    result={"candidateSha256":data["stepSha256"],"jointExchangeSha256":report["stepSha256"],
        "jointAssemblySourceSha256":shop.digest(tie_project/"fit.py"),"sourceShapes":pins,
        "counts":counts,"installedOrReservedBodies":len(assembly),"installedRibCheck":study,
        "upperContactLostMm3":checks,"reservedBoxMm":env_bounds,
        "undersideBoxAdversePairs":hits,"boxCloseBodies":native,"serviceMm":service,"physicalFit":None}
    shop.write(HERE/"fit.json",result)
    print(len(assembly),study,checks)

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for n in ("artifacts","lamp","tie_project"):
        p.add_argument(n,type=lambda x:Path(x).resolve())
    a=p.parse_args()
    run(a.artifacts,a.lamp,a.tie_project)
