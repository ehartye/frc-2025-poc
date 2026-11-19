"""Charge the 50 installed carriage joint articles on a source-bound fixed-low selection."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
def read(p):return json.loads(p.read_bytes())
def write(p,data):p.write_text(json.dumps(data,indent=2)+"\n",encoding="utf8",newline="\n")
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def inventory(report):
    return set(report["make"])|set(report["undecided"])|{i for a in report["buy"] for i in a["instances"]}
def check(plugin,p):
    return json.loads(subprocess.check_output(["node",str(plugin/"scripts/assembly-accounting.mjs"),str(p)]))

def article(k):
    if k.endswith("sleeve"):return "made nominal 25 mm aluminum mast sleeve"
    if k.endswith("ribbed-bracket"):return "machined 7075-T7351 bracket"
    if "-mast-" in k and k.endswith("-screw"):return "M6 x 45 8.8 hex head trimmed to 43 mm"
    if k.endswith("rear-screw"):return "M6 x 25 8.8 hex head"
    if k.endswith("flush-screw"):return "M5 x 25 ISO 10642 countersunk screw trimmed to 23 mm"
    diameter="M5" if "-front-" in k else "M6"
    return diameter+(" steel plain washer" if k.endswith("washer") else " steel prevailing torque locknut")

def run(plugin,path,folder):
    path=path.resolve()
    folder.mkdir(parents=True,exist_ok=True)
    before=check(plugin,path)
    orig=read(path)
    req={"underside-shelf-insert","mast-foot-splices","fixed-secondary-haul","low-lock"}
    if not req<=set(orig["selected"]):raise ValueError("Require selected damper shelf, mast splice and stationary shaft history")
    replace={"crossbar":"low-lock/lift-crossbar-drilled","bridge":"shelf-structure/through-bore-bridge",
             "mast-L":"mast-base-L-mast","mast-R":"mast-base-R-mast"}
    if not set(replace.values())<=inventory(before):raise ValueError("Missing mounted predecessor")
    for s in orig["sources"]:
        f=(path.parent/s["file"]).resolve()
        if digest(f)!=s["sha256"]:raise ValueError("Changed source "+s["id"])
        s["file"]=os.path.relpath(f,folder).replace(os.sep,"/")
    fit=read(HERE/"fit.json")
    if digest(HERE/"carriage-support.step")!=fit["stepSha256"]:raise ValueError("Changed manufactured kit")
    groups={};added=[]
    for k,v in fit["volumesMm3"].items():
        item={"id":k,"presence":"candidate","disposition":"make"}
        if not k.startswith("support-machined-"):
            a=article(k)
            if k.endswith("ribbed-bracket"):lo,hi,extra=2.75,2.92,.003
            elif k.endswith("sleeve"):lo,hi,extra=2.60,2.85,.0005
            else:
                lo,hi,extra=5.3,8.15,.0005
                item.update(disposition="buy",article=a)
                if not any(x["id"]==a for x in orig["articles"]):orig["articles"].append({"id":a,"sku":None,"unitsPerPack":None,"onHand":None})
            g=groups.setdefault(a,{"installed":0,"kg":[0.,0.]})
            g["installed"]+=1;g["kg"][0]+=v*lo/1e6;g["kg"][1]+=v*hi/1e6+extra
        added.append(k);orig["instances"].append(item)
    # Reserve thread-locking agent, markings and surface finish on the assembly.
    groups["joint compound and witness markings"]={"installed":0,"kg":[0.,.004]}
    delta=[sum(g["kg"][i] for g in groups.values()) for i in (0,1)]
    bill={"deltaKg":delta,"groups":groups,"oldStockAndChipCreditKg":0,"weighedKg":None,"stepSha256":fit["stepSha256"]}
    f=folder/"support-bill.json";write(f,bill)
    orig["sources"].append({"id":"carriage-support-bill","file":f.name,"sha256":digest(f)})
    orig["changes"].append({"id":"fixed-carriage-supports","configuration":orig["configuration"],"revision":orig["revision"],
        "remove":list(replace.values()),"add":added,"delta":{"source":"carriage-support-bill","pointer":"/deltaKg","rangeKg":delta,"basis":"estimated"}})
    orig["selected"].append("fixed-carriage-supports")
    target=folder/"selected.json";write(target,orig);final=check(plugin,target)
    if inventory(final)!=(inventory(before)-set(replace.values()))|set(added) or any(final["coverage"][k] for k in ("duplicates","orphanIncluded","missing","unattached")):raise ValueError("Installed inventory differs")
    margin=final["mass"]["representedMarginKg"];start=before["mass"]["representedMarginKg"]
    if abs(start[0]-delta[1]-margin[0])>1e-7:raise ValueError("Weight failed to reconcile")
    return {"sourceMapSha256":digest(path),"configured":len(inventory(final)),"bill":bill,"priorMarginKg":start,
        "representedMarginKg":margin,"physicalMarginKg":final["mass"]["configurationMarginKg"]}

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("plugin",type=Path);p.add_argument("shelf_maps",type=Path);p.add_argument("out",type=Path)
    a=p.parse_args();result={}
    for case in ("noCutCreditKg","densityMatchedRemovalKg"):
        result[case]=run(a.plugin,a.shelf_maps/(case+".json"),a.out/case)
    write(HERE/"accounting.json",result)
