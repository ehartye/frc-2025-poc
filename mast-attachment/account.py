"""Charge the complete shelf-mast hardware to one declared chassis and deck combination."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

HERE=Path(__file__).resolve().parent


def read(path): return json.loads(path.read_bytes())


def write(path,data):path.write_text(json.dumps(data,indent=2)+"\n",encoding="utf8",newline="\n")


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def run(plugin,path,out):
    data=read(path)
    out.mkdir(parents=True,exist_ok=True)
    selected=set(data["selected"])
    if not {"fixed-lead-install","deck-through-ties","supported-drive-covers"}<=selected or "mast-foot-splices" in selected:
        raise ValueError("Require issued deck ties, fixed optical lead and supported rails")
    fitted=read(HERE/"fit.json")
    if sha(HERE/"mast-joints.step")!=fitted["stepSha256"]:raise ValueError("Changed mast hardware")
    ribbed=any(i["id"]=="integral-rib-deck" and i["id"] in c["add"] for c in data["changes"] if c["id"] in selected for i in data["instances"] if i["id"]=="integral-rib-deck")
    prior_deck="integral-rib-deck" if ribbed else "tie-machined-deck-main"
    removed=[prior_deck]+["supported-covers/drive-rail-inner-"+s for s in ("L","R")]+["low-lock/backup-"+s+"-drilled-mast" for s in ("L","R")]
    for ref in data["sources"]:
        original=(path.parent/ref["file"]).resolve()
        if sha(original)!=ref["sha256"]:raise ValueError("Changed map source "+ref["id"])
        ref["file"]=os.path.relpath(original,out).replace(os.sep,"/")
    result={}
    add=[]
    articles={"long":("M6x40 8.8 bolt trimmed to 37.5","01000060040"),"short":("M6x20 8.8 screw trimmed to 18.5","01210060020"),
              "washer":("M6 ISO7089 flat washer","B0112.06"),"nut":("M6 DIN985 locknut class 8 or higher",None)}
    for k,v in fitted["volumeMm3"].items():
        if k=="mast-slotted-deck" and ribbed:k="mast-slotted-ribbed-deck"
        item={"id":k,"presence":"candidate","disposition":"make"}
        if k.endswith(("splice","sleeve","washer","nut","screw")):
            kind=k.split("-")[-1]
            if kind=="screw":kind="long" if int(k.split("-")[-2])>=4 else "short"
            density=(2.75,2.92) if kind=="splice" else ((2.60,2.85) if kind=="sleeve" else (5.3,8.15))
            r=result.setdefault(kind,[0,0])
            r[0]+=v*density[0]/1e6
            r[1]+=v*density[1]/1e6+(.003 if kind=="splice" else .0005)
            if kind in articles:
                article,sku=articles[kind]
                item.update(disposition="buy",article=article)
                if article not in [a["id"] for a in data["articles"]]:
                    data["articles"].append({"id":article,"sku":sku,"unitsPerPack":None,"onHand":None})
        data["instances"].append(item)
        add.append(k)
    charge={"groupsKg":result,"deltaKg":[sum(r[i] for r in result.values()) for i in (0,1)],
            "drillingAndSlotCreditKg":0,"measuredKitKg":None,"stepSha256":fitted["stepSha256"]}
    source=out/"mast-material.json"
    write(source,charge)
    data["sources"].append({"id":"mast-joint-material","file":source.name,"sha256":sha(source)})
    data["changes"].append({"id":"mast-foot-splices","revision":data["revision"],"configuration":data["configuration"],
            "remove":removed,"add":add,"delta":{"source":"mast-joint-material","pointer":"/deltaKg","basis":"estimated","rangeKg":charge["deltaKg"]}})
    data["selected"].append("mast-foot-splices")
    target=out/"selected.json"
    write(target,data)
    report=json.loads(subprocess.check_output(["node",str(plugin/"scripts/assembly-accounting.mjs"),str(target)]))
    inventory=set(report["make"])|{n for a in report["buy"] for n in a["instances"]}
    if set(removed)&inventory or not set(add)<=inventory:raise ValueError("The selected quantities did not close")
    if any(report["coverage"][k] for k in ("missing","unattached","orphanIncluded","duplicates")):raise ValueError(report["coverage"])
    return {"replaces":removed,"addedOccurrences":len(add),"configured":report["representedInstances"],"charge":charge,
            "representedMarginKg":report["mass"]["representedMarginKg"],"physicalMarginKg":report["mass"]["configurationMarginKg"],
            "baselinePins":{r["id"]:r["sha256"] for r in data["sources"]}}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for k in ("plugin","ties","ribbed","out"):p.add_argument(k,type=lambda d:Path(d).resolve())
    a=p.parse_args()
    cases={}
    for kind,base,file in (("flat",a.ties,"deck-ties.json"),("ribbed",a.ribbed,"selected.json")):
        for option in ("harness","restrictedNoMotorCreditKg","conditionalMatchedArticleDeltaKg"):
            cases[kind+"/"+option]=run(a.plugin,base/option/file,a.out/kind/option)
    write(HERE/"comparison.json",cases)
