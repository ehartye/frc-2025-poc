"""Replace the skin-only option with a complete four-shell attachment bill."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

HERE=Path(__file__).resolve().parent


def read(p):
    return json.loads(p.read_bytes())


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def write(p,data):
    p.write_text(json.dumps(data,indent=2)+"\n",encoding="utf8",newline="\n")


def run(plugin,incoming,folder,report_file):
    folder.mkdir(parents=True,exist_ok=True)
    data=read(incoming)
    old="repacked-light"
    if data["selected"].count(old)!=1:
        raise ValueError("Select the thin repacked wheel bays exactly once")
    original=next(row for row in data["changes"] if row["id"]==old)
    sleeves=[k for k in original["add"] if "/spacer-" in k]
    skins=set(original["add"])-set(sleeves)
    if len(sleeves)!=8 or len(skins)!=8:
        raise ValueError("Wrong spacer or cover inventory")
    for row in data["sources"]:
        actual=(incoming.parent/row["file"]).resolve()
        if sha(actual)!=row["sha256"]:
            raise ValueError("Stale source: "+row["id"])
        row["file"]=os.path.relpath(actual,folder).replace(os.sep,"/")
    fit=read(HERE/"fit.json")
    if sha(HERE/"supported-covers.step")!=fit["stepSha256"]:
        raise ValueError("Reopen the issued CAD")
    delta=[0.,0.]
    groups={}
    added=[]
    rails=[]
    for key,volume in fit["volumeMm3"].items():
        # The original rail stock remains fully charged. No chip mass credit.
        art=None
        if key.startswith("drive-rail-inner"):
            rails.append(key)
        elif key.startswith("tray-"):
            group="formed sheet"; densities=(1.14,1.26)
        elif key.startswith(("post-","retainer-")):
            group="turned posts and shoulders"; densities=(2.55,2.85)
        else:
            length=12 if key.startswith("base-screw-") else 8
            art=f"M3x{length} ISO 4762 socket screw"
            group="purchased screws"
            # Receiving band relative to envelope volume. Heads omit the socket
            # and threads omit root relief; weigh the complete selected lot.
            densities=(5.2,8.1)
        if key not in rails:
            band=groups.setdefault(group,[0.,0.])
            for i,rho in enumerate(densities):
                amount=volume*rho/1e6
                delta[i]+=amount
                band[i]+=amount
        label="supported-covers/"+key
        added.append(label)
        item={"id":label,"presence":"candidate","disposition":"make"}
        if art:
            item.update(disposition="buy",article=art)
            if not any(a["id"]==art for a in data["articles"]):
                data["articles"].append({"id":art,"sku":None,"unitsPerPack":None,"onHand":None})
        data["instances"].append(item)
    source="supported-cover-stock"
    evidence=folder/"cover-material.json"
    write(evidence,{"stepSha256":fit["stepSha256"],"estimatedKg":delta,"groupsKg":groups,"railCreditKg":0})
    data["sources"].append({"id":source,"file":evidence.name,"sha256":sha(evidence)})
    option="supported-drive-covers"
    data["changes"].append({"id":option,"revision":data["revision"],"configuration":data["configuration"],
                         "remove":sorted(original["remove"]+rails),"add":sorted(added+sleeves),
                         "delta":{"source":source,"pointer":"/estimatedKg","basis":"estimated","rangeKg":delta}})
    data["selected"][data["selected"].index(old)]=option
    data["omissions"].remove("Chain cover attachment, closure and joining material have no issued bill.")
    target=folder/"supported-covers.json"
    write(target,data)
    result=json.loads(subprocess.check_output(["node",str(plugin/"scripts/assembly-accounting.mjs"),str(target)]))
    configured=set(result["make"]) | {k for a in result["buy"] for k in a["instances"]}
    if not set(added+sleeves)<=configured or configured & (skins|set(rails)|set(original["remove"])):
        raise ValueError("Configured cover and rail stock differs from its selected exchange")
    report={"plugin":read(plugin/"package.json")["version"],"selected":data["selected"],
            "groupsKg":groups,"completeGuardKg":delta,"railCreditKg":0,
            "configuredInstances":result["representedInstances"],"representedKg":result["mass"]["representedRangeKg"],
            "representedMarginKg":result["mass"]["representedMarginKg"],"physicalMarginKg":result["mass"]["configurationMarginKg"]}
    write(report_file,report)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for argument in ("plugin","incoming","folder","report"):
        p.add_argument(argument,type=Path)
    a=p.parse_args()
    run(a.plugin.resolve(),a.incoming.resolve(),a.folder.resolve(),a.report.resolve())
