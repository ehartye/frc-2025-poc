"""Overlay each physical deck tie and all three machined predecessors on one selected robot."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

HERE=Path(__file__).resolve().parent
STOCK={"deck-main","cross-front","cross-rear"}
ARTICLES={"washer":("M5 ISO7089 5.3x10x1","B0112.05"),
          "locknut":("M5 DIN982 class 8 8 AF height 6.3","03715")}


def read(path): return json.loads(path.read_bytes())


def write(path,data):
    path.write_text(json.dumps(data,indent=2)+"\n",encoding="utf8",newline="\n")


def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def run(plugin,incoming,out):
    data=read(incoming)
    out.mkdir(parents=True,exist_ok=True)
    if "deck-through-ties" in data["selected"] or "four-welded-corners" not in data["selected"]:
        raise ValueError("Requires a welded chassis with its three original stock IDs still selected")
    fit=read(HERE/"fit.json")
    step=HERE/"deck-through-ties.step"
    if sha(step)!=fit["stepSha256"]: raise ValueError("Different machined kit")
    for source in data["sources"]:
        old=(incoming.parent/source["file"]).resolve()
        if sha(old)!=source["sha256"]: raise ValueError("Changed input "+source["id"])
        source["file"]=os.path.relpath(old,out).replace(os.sep,"/")
    by_type={}
    for k,v in fit["volumeMm3"].items():
        item={"id":k,"presence":"candidate","disposition":"make"}
        if k.startswith("deck-tie-"):
            kind=k.split("-")[-1]
            # Tolerance/density bounds on representative native solids. All bought items
            # retain allowance for receiving variation. No hole-chip credit or spare weight.
            density=(2.75,2.90) if kind=="anchor" else (5.0,8.15)
            upper_extra={"washer":.00015,"locknut":.0006,"anchor":.0008}[kind]
            row=by_type.setdefault(kind,[0.,0.])
            row[0]+=v*density[0]/1e6
            row[1]+=v*density[1]/1e6+upper_extra
            if kind in ARTICLES:
                art,sku=ARTICLES[kind]
                item.update(disposition="buy",article=art)
                if art not in [a["id"] for a in data["articles"]]:
                    data["articles"].append({"id":art,"sku":sku,"unitsPerPack":None,"onHand":None})
        data["instances"].append(item)
    delta=[sum(r[i] for r in by_type.values()) for i in (0,1)]
    receipt={"stepSha256":sha(step),"estimatedKg":delta,"classKg":by_type,"removedStockMassCreditKg":0,
             "weighedFinishedKitKg":None}
    material=out/"deck-tie-material.json"
    write(material,receipt)
    data["sources"].append({"id":"deck-tie-material","file":material.name,"sha256":sha(material)})
    data["changes"].append({"id":"deck-through-ties","revision":data["revision"],"configuration":data["configuration"],
        "remove":sorted(STOCK),"add":sorted(fit["volumeMm3"]),"delta":{"source":"deck-tie-material",
            "basis":"estimated","pointer":"/estimatedKg","rangeKg":delta}})
    data["selected"].append("deck-through-ties")
    target=out/"deck-ties.json"
    write(target,data)
    report=json.loads(subprocess.check_output(["node",str(plugin/"scripts/assembly-accounting.mjs"),str(target)]))
    selected=set(report["make"])|{k for r in report["buy"] for k in r["instances"]}
    if not set(fit["volumeMm3"])<=selected or STOCK & selected: raise ValueError("Wrong manufactured stock selection")
    if any(report["coverage"][k] for k in ("missing","unattached","duplicates","orphanIncluded")): raise ValueError(report["coverage"])
    result={"version":read(plugin/"package.json")["version"],"choices":data["selected"],
            "installedInstances":report["representedInstances"],"estimatedRangeKg":report["mass"]["representedRangeKg"],
            "representedMarginKg":report["mass"]["representedMarginKg"],"physicalMarginKg":report["mass"]["configurationMarginKg"],
            "hardware":receipt}
    write(out/"summary.json",result)
    return result


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for k in ("plugin","source","out"): p.add_argument(k,type=lambda v:Path(v).resolve())
    a=p.parse_args()
    run(a.plugin,a.source,a.out)
