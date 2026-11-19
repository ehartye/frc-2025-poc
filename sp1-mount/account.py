"""Account for seven new parts, retaining the same supplied sensor, bracket and cable."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"frame-corners"))
from account import digest, read, write, bind_sources


def run(plugin, baseline, output):
    output.mkdir(parents=True,exist_ok=True)
    data=read(baseline)
    if data["selected"][-1]!="clearance-fit" or "sp1-install" in data["selected"]:
        raise ValueError("Requires the issued shop-receiver selection exactly once")
    pinned=next(s for s in data["sources"] if s["id"]=="clearance-fit-mass")
    if pinned["sha256"]!=digest(ROOT/"selected-match/clearance-mass.json"):
        raise ValueError("Stale receiver mass input")
    regular=[]
    for s in data["sources"]:
        if s["id"]=="selected-shop-mass":
            src=(baseline.parent/s["file"]).resolve()
            if digest(src)!=s["sha256"]:
                raise ValueError("Historical material stock changed")
            dest=output/"upstream-shop-mass.json"
            dest.write_bytes(src.read_bytes())
            s["file"]=dest.name
        else:
            regular.append(s)
    bind_sources(regular,baseline,ROOT,output)
    fit=read(HERE/"fit.json")
    if digest(HERE/"sp1-mount.step")!=fit["stepSha256"]:
        raise ValueError("Different SP1 machining receipt")
    sizes=fit["volumeMm3"]
    old=read(ROOT/"selected-match/kit.json")["volumeMm3"]
    removed=["slide-position-saddle","slide-position-target"]
    retained="slide-position-SP1-25"
    aluminum=["sp1-step-adapter","sp1-shelf-clevis"]
    steel=["sp1-eye-shoulder-pin"]
    hardware={"sp1-eye-washer":("M3 washer 3.2 x 7 x 0.5",.0002),"sp1-eye-locknut":("M3 low locknut",.0009),
              "sp1-foot-M3-0":("M3 x 6 socket cap",.0012),"sp1-foot-M3-1":("M3 x 6 socket cap",.0012)}
    if set(sizes)!=set(aluminum+steel)|set(hardware):
        raise ValueError("Exchange parts without material assignment")
    # Low: credit the removed machined parts. High: do not reduce an unweighed
    # predecessor budget. Supplier bracket, cable, eyelet and the original screws
    # and nuts for both mounting lands already belong to the retained articles.
    delta=[sum(sizes[k] for k in aluminum)*2.65e-6 + sizes[steel[0]]*7.7e-6
           - sum(old[k] for k in removed)*2.85e-6,
           sum(sizes[k] for k in aluminum)*2.85e-6 + sizes[steel[0]]*8.1e-6
           + sum(ceiling for _,ceiling in hardware.values())]
    prior=next(c for c in data["changes"] if c["id"]=="clearance-fit")
    prior_range=read(ROOT/"selected-match/clearance-mass.json")["budgetDeltaKg"]
    combined=[a+b for a,b in zip(prior_range,delta)]
    mass=HERE/"mass-input.json"
    write(mass,{"exchangeSha256":fit["stepSha256"],"retainedSuppliedArticle":retained,
                "removedMachined":removed,"materialVolumeMm3":sizes,"looseHardwareUpperKg":{
                    k:cap for k,(_,cap) in hardware.items()},"budgetDeltaKg":delta,"priorRangeKg":prior_range,"combinedRangeKg":combined,
                "priorMassSourceSha256":pinned["sha256"]})
    source="sp1-mount-mass"
    data["sources"].append({"id":source,"file":os.path.relpath(mass,output).replace(os.sep,"/"),"sha256":digest(mass)})
    if not any(a["id"]==retained for a in data["instances"]):
        raise ValueError("SP1 source article absent")
    for article in sorted({a for a,_ in hardware.values()}-{a["id"] for a in data["articles"]}):
        data["articles"].append({"id":article,"sku":None,"unitsPerPack":None,"onHand":None})
    for name in sizes:
        item={"id":name,"presence":"candidate","disposition":"make"}
        if name in hardware:
            item.update(disposition="buy",article=hardware[name][0])
        data["instances"].append(item)
    superseded={"clearance/"+k for k in removed}
    if not superseded <= set(prior["add"]):
        raise ValueError("Superseded made items missing from the upstream overlay")
    data["changes"].append({"id":"sp1-install","revision":data["revision"],"configuration":data["configuration"],
        "remove":prior["remove"],"add":sorted((set(prior["add"])-superseded)|set(sizes)),
        "delta":{"source":source,"basis":"estimated","pointer":"/combinedRangeKg","rangeKg":combined}})
    data["selected"].remove("clearance-fit")
    data["selected"].append("sp1-install")
    target=output/"sp1-install.json"
    write(target,data)
    check=json.loads(subprocess.check_output(["node",str(plugin/"scripts/assembly-accounting.mjs"),str(target)]))
    report={"installedVersion":read(plugin/"package.json")["version"],"coverage":check["coverage"],
            "budgetDeltaKg":delta,"representedKg":check["mass"]["representedRangeKg"],
            "representedMarginKg":check["mass"]["representedMarginKg"],"physicalMarginKg":check["mass"]["configurationMarginKg"]}
    if any(report["coverage"][k] for k in ("missing","duplicates","orphanIncluded","unattached")):
        raise ValueError("Selected SP1 kit no longer reconciles")
    write(HERE/"mass.json",report)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ("plugin","baseline","output"):
        parser.add_argument(key,type=Path)
    args=parser.parse_args()
    run(args.plugin.resolve(),args.baseline.resolve(),args.output.resolve())
