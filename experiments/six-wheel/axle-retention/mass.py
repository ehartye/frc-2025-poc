"""Add the issued axle and bearing changes to the selected robot shop map."""
import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"frame-corners"))
from account import bind_sources, digest, read, write

SHOP_MASS_SHA="c1b3b4c432548d4e7e50bd92a9816f2515beb598f5d03e39d87739ff4fca5369"


def apply(plugin,baseline,out):
    data=read(baseline)
    expected={"low-lock","deck-lamp","moving-camera","no-match-air","four-welded-corners","light","shop-supplies"}
    if set(data["selected"])!=expected or "axle-retention" in {c["id"] for c in data["changes"]}:
        raise ValueError("Begin with the selected light-clamp and wheel shop map")
    out.mkdir(parents=True,exist_ok=True)
    portable=[]
    for source in data["sources"]:
        path=(baseline.parent/source["file"]).resolve()
        if digest(path)!=source["sha256"]:
            raise ValueError("Stale selected-shop input: "+source["id"])
        if source["id"]=="selected-shop-mass":
            if source["sha256"]!=SHOP_MASS_SHA:
                raise ValueError("Selected wheel and hardware allowances changed")
            for relative,sha in read(path)["source_sha256"].items():
                if digest(ROOT/relative)!=sha:
                    raise ValueError("Selected-shop checkout input changed: "+relative)
            copy=out/"upstream-shop-mass.json"
            copy.write_bytes(path.read_bytes())
            source["file"]=copy.name
        else:
            portable.append(source)
    bind_sources(portable,baseline,ROOT,out)
    fit=read(HERE/"fit.json")
    keep=read(HERE/"keepers.json")
    if digest(HERE/"axle-kit.step")!=fit["stepSha256"] or digest(HERE/"bearing-keepers.step")!=keep["stepSha256"]:
        raise ValueError("Machined hardware differs from its receipts")
    existing={i["id"] for i in data["instances"]}
    changed=list(fit["originalVolumeMm3"])+list(keep["removedCarrierMm3"])
    for name in changed:
        key="light/"+name if name.startswith("inner-") else name
        if key not in existing:
            raise ValueError("Reworked part was not previously accounted: "+key)
    volumes={**fit["volumeMm3"],**keep["volumeMm3"]}
    with (ROOT/"design/drivetrain/review/bom.csv").open(encoding="utf-8",newline="") as source:
        booked=[row for row in csv.DictReader(source) if row["id"] in fit["originalVolumeMm3"]]
    if len(booked)!=8 or any(r["minMassKg"]!=r["maxMassKg"] for r in booked):
        raise ValueError("Changed baseline shaft mass identities or ranges")
    old=sum(float(row["maxMassKg"]) for row in booked)
    def volume(prefix):
        return sum(v for k,v in volumes.items() if k.startswith(prefix))
    # Densities in kg/mm³. Intervals are receiving allocations; weigh the actual
    # parts before accepting the finish, shims and vendor supply.
    def density(total,min_g_cm3,max_g_cm3):
        return [total*min_g_cm3*1e-6,total*max_g_cm3*1e-6]
    groups={
        "old_six_shafts_and_two_spacers":[-old,-old],
        "new_six_7075_shafts":density(volume("axle-"),2.75,2.86),
        "shortened_spacers_and_twelve_keepers":density(volume(("spacer-center-","bearing-keeper-")),2.65,2.78),
        "twelve_machined_steel_thrust_washers":density(volume(("select-shim-","clip-thrust-")),7.70,8.10),
        "ten_shaft_end_screws":[.045,.055],
        "two_radial_rings":[.0005,.0008],
        "forty_eight_M3_countersunk_screws":[48*.00045,48*.00090],
        "twelve_drilled_carriers":[-sum(keep["removedCarrierMm3"].values())*2.85e-6,0]}
    span=[sum(v[i] for v in groups.values()) for i in (0,1)]
    source=HERE/"mass-input.json"
    write(source,{"groupsKg":groups,"selectedDeltaKg":span,"receivingOnly":True,
                  "fitSha256":digest(HERE/"fit.json"),"keeperSha256":digest(HERE/"keepers.json")})
    data["sources"].append({"id":"axle-mass","file":os.path.relpath(source,out).replace(os.sep,"/"),
                            "sha256":digest(source)})
    catalog={"retaining-screw-":("REV V2 shaft end screw","REV-21-2805-PK25",25),
             "keeper-M3-":("M3 x 8 keeper screw","243013X8",None),
             "clip-sweep-":("Unplated spring steel half-inch crescent retaining ring","C-50",None)}
    data["articles"].extend({"id":label,"sku":sku,"unitsPerPack":qty,"onHand":None}
                            for label,sku,qty in catalog.values())
    additions=[]
    for name in volumes:
        if name in changed:
            continue
        if name in existing:
            raise ValueError("Duplicate hardware: "+name)
        additions.append(name)
        item={"id":name,"presence":"candidate","disposition":"make"}
        for prefix,(label,_,_) in catalog.items():
            if name.startswith(prefix):
                item.update(disposition="buy",article=label)
                break
        data["instances"].append(item)
    if len(additions)!=84:
        raise ValueError("Expected 24 newly made and 60 purchased parts")
    data["changes"].append({"id":"axle-retention","revision":data["revision"],"configuration":data["configuration"],
                            "add":additions,"remove":[],"delta":{"source":"axle-mass","pointer":"/selectedDeltaKg",
                            "basis":"estimated","rangeKg":span}})
    data["selected"].append("axle-retention")
    target=out/"selected-axles.json"
    write(target,data)
    res=json.loads(subprocess.check_output(["node",str(plugin/"scripts/assembly-accounting.mjs"),str(target)]))
    coverage={k:res["coverage"][k] for k in ("missing","duplicates","orphanIncluded")}
    if any(coverage.values()):
        raise ValueError("Selected axle accounting coverage failed")
    write(HERE/"accounting.json",{"installedVersion":read(plugin/"package.json")["version"],"coverage":coverage,
          "addedInstalled":len(additions),"retainedReworked":len(changed),"representedMassKg":res["mass"]["representedRangeKg"],
          "representedMarginKg":res["mass"]["representedMarginKg"],"physicalMarginKg":res["mass"]["configurationMarginKg"]})


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ("plugin","baseline","output"):
        parser.add_argument(key,type=Path)
    a=parser.parse_args()
    apply(a.plugin.resolve(),a.baseline.resolve(),a.output.resolve())
