"""Compare one arm substitution on all four existing assembly bills."""
import argparse
import os
import sys
from pathlib import Path
from field_view import HERE, ROOT, read, digest

sys.path.insert(0,str(ROOT/"guide-setting"))
from installed import ledger

OLD="match-drilled/guide-L-687-mount-arm"
NEW="returned/guide-L-687-mount-arm"


def allowance(revision):
    delta=revision["armVolumeChangeMm3"]
    after=revision["volumesMm3"]["guide-L-687-mount-arm"]
    before=after-delta
    lo,hi=2.60e-6,2.85e-6
    # Planning reserve for four additional joints and finishing, in kg.
    extra=.005
    return {"matchedStock":[delta*lo,delta*hi+extra],
            "independentStock":[after*lo-before*hi,after*hi-before*lo+extra]}


def replay(plugin,source,out,basis,delta,revision):
    out.mkdir(parents=True,exist_ok=True)
    original=ledger.check(plugin,source)
    inventory=ledger.inventory(original)
    model=read(source)
    if OLD not in inventory or NEW in inventory or model["selected"][-1]!="match-drilled-guide":
        raise ValueError("Expected the six-part match-drilled configuration")
    for item in model["sources"]:
        path=(source.parent/item["file"]).resolve()
        if digest(path)!=item["sha256"]:raise ValueError("Assembly source changed")
        item["file"]=os.path.relpath(path,out).replace(os.sep,"/")
    bill=out/"arm-bill.json"
    ledger.write(bill,{"geometrySha256":revision["stepSha256"],"old":OLD,"new":NEW,
        "basis":basis,"rangeKg":delta,"additionalJointAndFinishAllowanceKg":[0,.005],"physicalMassKg":None})
    model["sources"].append({"id":"returned-arm-bill","file":bill.name,"sha256":digest(bill)})
    model["instances"].append({"id":NEW,"presence":"candidate","disposition":"make"})
    model["changes"].append({"id":"return-lower-guide-arm","configuration":model["configuration"],
        "revision":model["revision"],"remove":[OLD],"add":[NEW],"delta":{"source":"returned-arm-bill",
        "basis":"estimated","pointer":"/rangeKg","rangeKg":delta}})
    model["selected"].append("return-lower-guide-arm")
    output=out/"selected.json"
    ledger.write(output,model)
    result=ledger.check(plugin,output)
    actual=ledger.inventory(result)
    if actual!=(inventory-{OLD})|{NEW} or any(result["coverage"][k] for k in ("duplicates","missing","unattached","orphanIncluded")):
        raise ValueError("Unexpected change in the installed roster")
    before=original["mass"]["representedMarginKg"]
    after=result["mass"]["representedMarginKg"]
    if max(abs(after[i]-(before[i]-delta[1-i])) for i in (0,1))>1e-7:
        raise ValueError("Mass margin failed to reconcile")
    return {"inputSha256":digest(source),"installed":len(actual),"priorMarginKg":before,"marginKg":after,
            "physicalMarginKg":result["mass"]["configurationMarginKg"]}


def run(plugin,maps,folder):
    source=read(folder/"revision.json")
    if digest(folder/"route.step")!=source["stepSha256"]:raise ValueError("Unregistered replacement")
    budgets=allowance(source)
    cases={}
    for basis,delta in budgets.items():
        for shelf in ("densityMatchedRemovalKg","noCutCreditKg"):
            for credit in ("completeGroupDeltaKg","unknownGroupDeltaKg"):
                key="/".join((basis,shelf,credit))
                cases[key]=replay(plugin,maps/shelf/credit/"selected.json",folder/"assembly"/key,basis,delta,source)
    ledger.write(folder/"accounting.json",{"plugin":read(plugin/"package.json")["version"],
        "stepSha256":source["stepSha256"],"deltaKg":budgets,"cases":cases})


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("installed_plugin","match_drilled_maps","folder"):
        p.add_argument(name,type=lambda arg:Path(arg).resolve())
    args=p.parse_args()
    run(args.installed_plugin,args.match_drilled_maps,args.folder)
