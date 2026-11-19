"""Evaluate omission of the one fixed lift motor on any source-bound selected build."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent


def read(path):
    return json.loads(path.read_bytes())


def write(path,value):
    path.write_text(json.dumps(value,indent=2)+"\n",encoding="utf8",newline="\n")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def create_mass():
    with (ROOT/"design/selected-shop/supply.csv").open(encoding="utf8") as stream:
        supply=list(csv.reader(stream))
    neo=next(r for r in supply if r[1]=="REV-21-1650")
    ids=set(neo[-1].split(" | "))
    if ids!={"motor-body","motor-L","motor-R"} or int(neo[2])!=3:
        raise ValueError("Expected exactly three issued NEO articles")
    group=read(ROOT/"design/electrical-inspection/inspection.json")["projectMassAllowancesKg"]
    bounds=group["lift drive, ratchet, drums, lines, hardware and release"]
    volume=read(HERE/"fit.json")["stockVolumeMm3"]
    if bounds!=[1.8,3.9] or volume*2.85e-6>.030:
        raise ValueError("Engineering allowances no longer apply")
    # All new items are booked even when a pair of measured fit screws could be reused.
    added=[volume*2.65e-6,.030+.012+.012]
    return {"historicalLiftGroupKg":bounds,"supplierNominalKg":.425,"groupSpecificMinimumMotorKg":None,
        "newHardwareKg":added,"newUpperKg":{"cap":.030,"twoSHCS":.012,"electricalIsolationAndIdentification":.012},
        "restrictedNoMotorCreditKg":added,"conditionalMatchedArticleDeltaKg":[added[0]-.450,added[1]-.400],
        "conditionalRemovedMassRangeKg":[.400,.450],"minimumForPositiveRecoveryKg":added[1],
        "sources":{f:digest(ROOT/f) for f in ("design/selected-shop/supply.csv","design/electrical-inspection/inspection.json",
            "design/drivetrain/review/bom.csv","fixed-lift-omission/fit.json")},"measuredRemovedKg":None}


def run(plugin,source,out):
    out.mkdir(parents=True,exist_ok=True)
    data=read(source)
    active=set(data["selected"])
    if not {"low-lock","sp1-install","no-match-air"}<=active or "fixed-lift-motor-omitted" in active:
        raise ValueError("Requires the fixed low, no-air SP1 assembly")
    material=HERE/"mass.json"
    mass=create_mass()
    write(material,mass)
    for s in data["sources"]:
        path=(source.parent/s["file"]).resolve()
        if digest(path)!=s["sha256"]:
            raise ValueError("Changed accounting input "+s["id"])
        s["file"]=os.path.relpath(path,out).replace(os.sep,"/")
    data["sources"].append({"id":"motor-omission","file":os.path.relpath(material,out).replace(os.sep,"/"),"sha256":digest(material)})
    articles={"motor-cap-screw-1":"10-32x1-socket-screw","motor-cap-screw-2":"10-32x1-socket-screw",
              "lift-branch-insulation":"lift-branch-insulation-kit"}
    data["articles"].extend({"id":a,"sku":None,"unitsPerPack":None,"onHand":None} for a in sorted(set(articles.values())))
    data["instances"].append({"id":"winch-input-cap","presence":"candidate","disposition":"make"})
    for name,article in articles.items():
        data["instances"].append({"id":name,"presence":"candidate","disposition":"buy","article":article})
    evidence={}
    option={"id":"fixed-lift-motor-omitted","revision":data["revision"],"configuration":data["configuration"],
            "remove":["motor-body"],"add":["winch-input-cap","motor-cap-screw-1","motor-cap-screw-2","lift-branch-insulation"]}
    data["changes"].append(option)
    data["selected"].append(option["id"])
    for label in ("restrictedNoMotorCreditKg","conditionalMatchedArticleDeltaKg"):
        option["delta"]={"source":"motor-omission","basis":"estimated","pointer":"/"+label,"rangeKg":mass[label]}
        target=out/(label+".json")
        write(target,data)
        report=json.loads(subprocess.check_output(["node",str(plugin/"scripts/assembly-accounting.mjs"),str(target)]))
        buy={k for row in report["buy"] for k in row["instances"]}
        if "motor-body" in buy or not {"motor-R","motor-L","spark-lift","motor-cap-screw-1","motor-cap-screw-2"}<=buy:
            raise ValueError("Motor procurement did not resolve as intended")
        if any(report["coverage"][s] for s in ("missing","unattached","duplicates","orphanIncluded")):
            raise ValueError("Coverage changed")
        evidence[label]={"installedInstances":report["representedInstances"],"estimatedR103Kg":report["mass"]["representedRangeKg"],
                         "representedMarginKg":report["mass"]["representedMarginKg"],"physicalMarginKg":report["mass"]["configurationMarginKg"]}
    write(out/"summary.json",{"pluginVersion":read(plugin/"package.json")["version"],"choices":data["selected"],"cases":evidence})
    return evidence


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for arg in ("plugin","source","out"):
        p.add_argument(arg,type=lambda s:Path(s).resolve())
    args=p.parse_args()
    run(args.plugin,args.source,args.out)
