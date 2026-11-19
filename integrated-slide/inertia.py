"""Identify the actual selected physical articles, then partition moving mass from chassis mass."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"frame-corners"))
from account import digest, read, write, bind_sources


def direct(plugin, issued, out):
    data=read(issued)
    out.mkdir(parents=True,exist_ok=True)
    other=[]
    for s in data["sources"]:
        original=(issued.parent/s["file"]).resolve()
        if digest(original)!=s["sha256"]:
            raise ValueError("Stale accounting record "+s["id"])
        if s["id"]=="selected-shop-mass":
            target=out/"upstream-shop-mass.json"
            if target.resolve()!=original:
                target.write_bytes(original.read_bytes())
            s["file"]=target.name
        else:
            other.append(s)
    bind_sources(other,issued,ROOT,out)
    change=next(c for c in data["changes"] if c["id"]=="sp1-install")
    rail=next(c for c in data["changes"] if c["id"]=="clearance-fit")
    mass=read(ROOT/"sp1-mount/mass-input.json")
    if data["selected"][-2:]!=["selected-optics","sp1-install"] or "clearance-fit" in data["selected"]:
        raise ValueError("Expected the registered optical and SP1 alternatives")
    added=set(change["add"])-set(rail["add"])
    removed={"clearance/"+k for k in mass["removedMachined"]}
    if added != set(mass["materialVolumeMm3"]) or not removed <= set(rail["add"]):
        raise ValueError("The physical SP1 parts no longer match the audited baseline")
    change.update(add=sorted(added),remove=sorted(removed))
    change["delta"].update(pointer="/budgetDeltaKg",rangeKg=mass["budgetDeltaKg"])
    data["selected"].insert(data["selected"].index("selected-optics"),"clearance-fit")
    target=out/"direct-sp1.json"
    write(target,data)
    evidence=json.loads(subprocess.check_output(["node",str(plugin/"scripts/assembly-accounting.mjs"),str(target)]))
    flattened=read(ROOT/"sp1-mount/optics-mass.json")
    counts=evidence["coverage"]
    if any(counts[k] for k in ("missing","duplicates","unattached","orphanIncluded")):
        raise ValueError("Uncovered or duplicate installed articles")
    active=set(evidence["make"])
    active.update(k for row in evidence["buy"] for k in row["instances"])
    required=added|set(flattened["retainedOpticalInstances"])|{"slide-position-SP1-25"}
    if not required<=active or active&removed:
        raise ValueError("Inconsistent effective hardware")
    represented=evidence["mass"]["representedRangeKg"]
    if max(abs(a-b) for a,b in zip(represented,flattened["representedKg"]))>1e-8:
        raise ValueError("Direct and independently flattened mass differ")
    return {"installedVersion":read(plugin/"package.json")["version"],"checkerSha256":digest(plugin/"scripts/assembly-accounting.mjs"),
            "priorFlatMassSha256":digest(ROOT/"sp1-mount/optics-mass.json"),"activeAttachments":sorted(required),
            "selected":data["selected"],"uniqueInstances":evidence["representedInstances"],
            "representedKg":represented,"representedMarginKg":evidence["mass"]["representedMarginKg"],
            "physicalMarginKg":evidence["mass"]["configurationMarginKg"]}


def moving():
    sizes=read(ROOT/"sp1-mount/mass-input.json")
    volumes=sizes["materialVolumeMm3"]
    loose=sizes["looseHardwareUpperKg"]
    route=read(ROOT/"camera-disconnects/mass-conditions.json")["groups_kg"]
    for name in ("fixed_complete_panel_with_fixed_wiring",):
        del route[name]
    with (ROOT/"design/receiver-drives/bom.csv").open(encoding="utf8") as stream:
        introduced=list(csv.DictReader(stream))
    old=next(r for r in introduced if r["id"]=="slide-position-target")
    sensor=next(r for r in introduced if r["id"]=="slide-position-SP1-25")
    assert old["frame"]=="slide" and sensor["frame"]=="lift"
    retained=read(ROOT/"selected-match/clearance-mass.json")["fastenerUpperKg"]
    terms={"roofShop":read(ROOT/"camera-datum-shop/mechanics.json")["extra_upper_moving_mass_kg"],
           "cableAndCarrierConservative":sum(x[1] for x in route.values()),
           "removedOriginalTargetFromUpper":-float(old["highMassKg"]),
           "finalClevisAluminum":volumes["sp1-shelf-clevis"]*2.85e-6,
           "pinSteel":volumes["sp1-eye-shoulder-pin"]*8.1e-6,
           "nutAndWasher":loose["sp1-eye-locknut"]+loose["sp1-eye-washer"],
           "retainedShelfFlagFasteners":sum(v for k,v in retained.items() if k.startswith("flag-")),
           "fourReceiverShims":read(ROOT/"selected-match/receiver-mass.json")["budgetDeltaKg"][1],
           "shelfMarks":read(ROOT/"selected-optics/mass-input.json")["finishUpperKg"]}
    instrument=float(sensor["highMassKg"])
    bounds=read(ROOT/"sp1-mount/fit.json")["boundsMm"]["370"]
    eye=bounds["sp1-eye-reservation"]
    volume=math.prod(eye[i+3]-eye[i] for i in range(3))
    diameter=2*.019*25.4/2
    cable=bounds["sp1-live-cable"]
    volume+=math.pi*(diameter/2)**2*(cable[3]-cable[0])
    source=read(ROOT/"design/receiver-drives/sizing.json")
    base=source["loaded_outer_mass_kg"]
    fixed_terms=sum(terms.values())
    return {"selectedMovingDeltaTermsKg":terms,"originalLoadedOuterUpperKg":base,
            "inheritedMovingCameraAllocationKg":.440,"middleMassKg":source["moving_middle_mass_kg"],
            "unallocatedSuppliedArticleUpperKg":instrument,"conditionalWornEndAndCableAt8_1gCm3Kg":volume*8.1e-6,
            "extraToIssuedPlantKg":fixed_terms+instrument,"conditionalExtraKg":fixed_terms+volume*8.1e-6,
            "outerWithAllowanceKg":base+.440+fixed_terms+instrument,"physicalWeighing":None}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("plugin","issued","output"):
        p.add_argument(name,type=Path)
    a=p.parse_args()
    data=direct(a.plugin.resolve(),a.issued.resolve(),a.output.resolve())
    data["moving"]=moving()
    data["massSources"]={name:digest(ROOT/name) for name in ("design/receiver-drives/bom.csv",
        "design/receiver-drives/sizing.json","camera-disconnects/mass-conditions.json","camera-datum-shop/mechanics.json",
        "selected-match/clearance-mass.json","selected-match/receiver-mass.json","selected-optics/mass-input.json",
        "sp1-mount/mass-input.json","sp1-mount/fit.json")}
    write(HERE/"mass.json",data)
