"""Reconcile the match-drilled parts against the selected installed robot on all four bills."""
import argparse
import importlib.util
import os
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
spec=importlib.util.spec_from_file_location("carriage_ledger",ROOT/"carriage-support/account.py")
ledger=importlib.util.module_from_spec(spec)
spec.loader.exec_module(ledger)


def apply(plugin, source, dest, fit):
    dest.mkdir(parents=True,exist_ok=True)
    before=ledger.check(plugin,source)
    inventory=ledger.inventory(before)
    old=fit["changedInstalled"]
    if len(old)!=6 or not set(old)<=inventory:
        raise ValueError("The issued guide kit is missing")
    model=ledger.read(source)
    if "supported-guides" not in model["selected"]:
        raise ValueError("The mounted guide revision must already be selected")
    for item in model["sources"]:
        path=(source.parent/item["file"]).resolve()
        if ledger.digest(path)!=item["sha256"]:
            raise ValueError("A source for the selected robot changed")
        item["file"]=os.path.relpath(path,dest).replace(os.sep,"/")
    replacements=["match-drilled/"+k for k in old]
    for new_id in replacements:
        model["instances"].append({"id":new_id,"presence":"candidate","disposition":"make"})
    evidence=dest/"drilling.json"
    ledger.write(evidence,{"stepSha256":fit["stepSha256"],"predecessorSha256":fit["guideSha256"],
        "old":old,"new":replacements,"shopPinsRemovedBeforeMatch":fit["pitOnly"],
        "estimatedDeltaKg":[0.,0.],"weighedKg":None})
    model["sources"].append({"id":"guide-setting-bores","file":evidence.name,"sha256":ledger.digest(evidence)})
    model["changes"].append({"id":"match-drilled-guide","configuration":model["configuration"],
        "revision":model["revision"],"remove":old,"add":replacements,
        "delta":{"source":"guide-setting-bores","basis":"estimated","pointer":"/estimatedDeltaKg","rangeKg":[0.,0.]}})
    model["selected"].append("match-drilled-guide")
    selected=dest/"selected.json"
    ledger.write(selected,model)
    after=ledger.check(plugin,selected)
    actual=ledger.inventory(after)
    if (actual!=(inventory-set(old))|set(replacements) or set(fit["pitOnly"])&actual
            or any(after["coverage"][k] for k in ("duplicates","missing","unattached","orphanIncluded"))
            or after["mass"]["representedMarginKg"]!=before["mass"]["representedMarginKg"]):
        raise ValueError("Inventory or mass changed outside the six replacements")
    return {"inputSha256":ledger.digest(source),"selectedCount":len(actual),"marginKg":after["mass"]["representedMarginKg"],
            "physicalMarginKg":after["mass"]["configurationMarginKg"]}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ("plugin","guide_maps","output"):
        parser.add_argument(key,type=lambda s:Path(s).resolve())
    args=parser.parse_args()
    fit=ledger.read(HERE/"fit.json")
    if ledger.digest(HERE/"setting.step")!=fit["stepSha256"]:
        raise ValueError("The reopened geometry changed")
    results={}
    for shelf in ("densityMatchedRemovalKg","noCutCreditKg"):
        results[shelf]={}
        for policy in ("completeGroupDeltaKg","unknownGroupDeltaKg"):
            results[shelf][policy]=apply(args.plugin,args.guide_maps/shelf/policy/"selected.json",
                                          args.output/shelf/policy,fit)
    ledger.write(HERE/"accounting.json",{"plugin":ledger.read(args.plugin/"package.json")["version"],
        "installedReplacements":6,"shopPins":4,"chipCreditKg":0.,"cases":results})
    print("Reconciled four configurations")
