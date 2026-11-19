"""Exchange the one fully charged deck after selecting the six end-rail ties."""
import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"cover-retention"))
from account_mounts import read, sha, write

def booked_and_replacement():
    geometry=read(HERE/"geometry.json")
    stock=ROOT/"design/drivetrain/review/bom.csv"
    with stock.open(encoding="utf8") as stream:
        rows=[r for r in csv.DictReader(stream) if r["id"]=="deck-main"]
    if len(rows)!=1 or rows[0]["group"]!="plate":
        raise ValueError("Deck row is ambiguous")
    booked=[float(rows[0][k]) for k in ("minMassKg","maxMassKg")]
    inspection=ROOT/"design/electrical-inspection/inspection.json"
    if "deck-main" in read(inspection)["excludedPriorIds"]:
        raise ValueError("Historical deck is excluded")
    volume=geometry["solidVolumeMm3"]
    # Finished-piece receiving conditions. Stock overage includes fillets,
    # cutter radii and setup tolerances at all lands, holes, ribs and webs.
    volume_band=[volume-20_000,volume+40_000]
    density=[2.67e-6,2.73e-6]
    new=[v*rho for v,rho in zip(volume_band,density)]
    return {"bookedDeckKg":booked,"netFinishedVolumeBandMm3":volume_band,"lotDensityKgPerMm3":density,
        "finishedDeckKg":new,"deltaKg":[new[0]-booked[1],new[1]-booked[0]],
        "sources":{"stock":sha(stock),"inspection":sha(inspection),"geometry":sha(HERE/"geometry.json")},
        "physicalMassKg":None}

def apply(plugin,source,out,material):
    data=read(source)
    if "deck-through-ties" not in data["selected"] or "integral-deck" in data["selected"]:
        raise ValueError("Requires a previously tied full-thickness deck")
    changes=[r for r in data["changes"] if r["id"] in data["selected"]]
    if [r["id"] for r in changes if "deck-main" in r["remove"]]!=["deck-through-ties"]:
        raise ValueError("Unexpected deck history")
    if "tie-machined-deck-main" not in next(r for r in changes if r["id"]=="deck-through-ties")["add"]:
        raise ValueError("Missing tie-drilled predecessor")
    out.mkdir(parents=True,exist_ok=True)
    pins={}
    for item in data["sources"]:
        path=(source.parent/item["file"]).resolve()
        if sha(path)!=item["sha256"]:
            raise ValueError("Source changed: "+item["id"])
        pins[item["id"]]=path
        item["file"]=os.path.relpath(path,out).replace(os.sep,"/")
    if read(pins["deck-tie-material"])["removedStockMassCreditKg"]!=0:
        raise ValueError("Tie drilling already received chip credit")
    if data["mass"]["baseline"]["pointer"]!="/wheel_replaced_baseline_kg":
        raise ValueError("Unsupported mass baseline")
    selected=read(pins["selected-shop-mass"])
    shop_mass=ROOT/"design/competition-package/shop-duplex/mass.json"
    for dependency in (shop_mass,ROOT/"design/drivetrain/review/bom.csv"):
        if sha(dependency)!=selected["source_sha256"][dependency.relative_to(ROOT).as_posix()]:
            raise ValueError("Selected ledger no longer pins the deck's source bill")
    before_wheels=read(shop_mass)["poses"]["stowed"]["R103_kg"]
    if before_wheels!=selected["old_baseline_kg"]:
        raise ValueError("Baseline history changed")
    evidence=out/"replacement.json"
    write(evidence,material)
    data["sources"].append({"id":"integral-deck-bill","file":evidence.name,"sha256":sha(evidence)})
    data["instances"].append({"id":"integral-rib-deck","disposition":"make","presence":"candidate"})
    data["changes"].append({"id":"integral-deck","configuration":data["configuration"],"revision":data["revision"],
        "remove":["tie-machined-deck-main"],"add":["integral-rib-deck"],
        "delta":{"source":"integral-deck-bill","basis":"estimated","pointer":"/deltaKg","rangeKg":material["deltaKg"]}})
    data["selected"].append("integral-deck")
    path=out/"selected.json"
    write(path,data)
    command=["node",str(plugin/"scripts/assembly-accounting.mjs")]
    pre=json.loads(subprocess.check_output(command+[str(source)]))
    final=json.loads(subprocess.check_output(command+[str(path)]))
    if set(final["make"])!=(set(pre["make"])-{"tie-machined-deck-main"})|{"integral-rib-deck"}:
        raise ValueError("Unrelated fabrication changes")
    if final["buy"]!=pre["buy"] or final["representedInstances"]!=pre["representedInstances"]:
        raise ValueError("Article or installed count changed")
    if any(final["coverage"][k] for k in ("missing","duplicates","unattached","orphanIncluded")):
        raise ValueError("Incomplete inventory")
    return {"sourceChoices":data["selected"],"instances":final["representedInstances"],
        "representedKg":final["mass"]["representedRangeKg"],"representedMarginKg":final["mass"]["representedMarginKg"],
        "priorUpperMarginKg":pre["mass"]["representedMarginKg"][0],"physicalMarginKg":final["mass"]["configurationMarginKg"]}

def run(plugin,maps,out):
    material=booked_and_replacement()
    cases={label:apply(plugin,maps/label/"deck-ties.json",out/label,material) for label in
           ("harness","restrictedNoMotorCreditKg","conditionalMatchedArticleDeltaKg")}
    write(HERE/"budget.json",{"plugin":read(plugin/"package.json")["version"],"material":material,"cases":cases})
    print({k:round(v["representedMarginKg"][0],5) for k,v in cases.items()})

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for k in ("plugin","maps","out"):
        p.add_argument(k,type=lambda s:Path(s).resolve())
    a=p.parse_args()
    run(a.plugin,a.maps,a.out)
