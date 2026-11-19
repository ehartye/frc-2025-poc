"""Charge each guide article once on both shelf selections and both whole-winch controls."""
import argparse
import importlib.util
import os
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
spec=importlib.util.spec_from_file_location("carriage_account",ROOT/"carriage-support/account.py")
ledger=importlib.util.module_from_spec(spec)
spec.loader.exec_module(ledger)


def material(name):
    if name.endswith("-liner"):
        return "formed liner",.30,.70,0
    if "face-screw-" in name:
        return "M3 x 16 countersunk steel screw",7.0,8.15,.0002
    for ending,title in (("screw","M4 x 65 steel screw"),("washer","M4 washer"),("locknut","M4 locknut")):
        if name.endswith("-"+ending):
            return title,6.4,8.15,.0003
    if name.endswith("-sleeve"):
        return "50 mm aluminum crush sleeve",2.60,2.85,.0001
    if name.endswith("-mount-arm"):
        return "aluminum arm and welded gusseted root",2.60,2.85,.002
    if name.endswith("-tapped-bar"):
        return "aluminum tapped insert",2.60,2.85,.0002
    if name.endswith("-wedge"):
        return "aluminum support wedge",2.60,2.85,.0002
    if name.endswith("-backing"):
        return "folded aluminum spine",2.60,2.85,.0004
    raise ValueError(name)


def apply(plugin, path, dest):
    dest.mkdir(parents=True,exist_ok=True)
    model=ledger.read(path)
    old=ledger.check(plugin,path)
    predecessor=["support-machined-mast-"+s for s in ("L","R")]
    if "winchless" not in model["selected"] or not set(predecessor)<=ledger.inventory(old):
        raise ValueError("Expected the winchless robot with direct carriage brackets")
    for src in model["sources"]:
        f=(path.parent/src["file"]).resolve()
        if ledger.digest(f)!=src["sha256"]:
            raise ValueError("Changed accounting source "+src["id"])
        src["file"]=os.path.relpath(f,dest).replace(os.sep,"/")
    fit=ledger.read(HERE/"fit.json")
    if ledger.digest(HERE/"guide-mounts.step")!=fit["stepSha256"]:
        raise ValueError("Changed guide geometry")
    groups={}
    for k,v in fit["volumesMm3"].items():
        item={"id":k,"presence":"candidate","disposition":"make"}
        if not k.endswith("machined-mast"):
            title,lo,hi,extra=material(k)
            if "steel screw" in title or title.startswith("M4"):
                item.update(disposition="buy",article=title)
                if not any(a["id"]==title for a in model["articles"]):
                    model["articles"].append({"id":title,"sku":None,"unitsPerPack":None,"onHand":None})
            row=groups.setdefault(title,{"installed":0,"rangeKg":[0.,0.]})
            row["installed"]+=1
            row["rangeKg"][0]+=v*lo/1e6
            row["rangeKg"][1]+=v*hi/1e6+extra
        model["instances"].append(item)
    groups["liner bond, finish and compound allowance"]={"installed":0,"rangeKg":[0.,.015]}
    delta=[sum(v["rangeKg"][i] for v in groups.values()) for i in (0,1)]
    bill={"stepSha256":fit["stepSha256"],"groups":groups,"rangeKg":delta,
          "mastChipCreditKg":0,"installedMassMeasuredKg":None}
    receipt=dest/"guide-bill.json"
    ledger.write(receipt,bill)
    model["sources"].append({"id":"guide-bill","file":receipt.name,"sha256":ledger.digest(receipt)})
    model["changes"].append({"id":"supported-guides","configuration":model["configuration"],
        "revision":model["revision"],"remove":predecessor,"add":list(fit["volumesMm3"]),
        "delta":{"source":"guide-bill","pointer":"/rangeKg","rangeKg":delta,"basis":"estimated"}})
    model["selected"].append("supported-guides")
    selected=dest/"selected.json"
    ledger.write(selected,model)
    result=ledger.check(plugin,selected)
    expected=(ledger.inventory(old)-set(predecessor))|set(fit["volumesMm3"])
    if ledger.inventory(result)!=expected or any(result["coverage"][k] for k in ("missing","duplicates","unattached","orphanIncluded")):
        raise ValueError("Different guide roster")
    before=old["mass"]["representedMarginKg"][0]
    margin=result["mass"]["representedMarginKg"][0]
    if abs(before-delta[1]-margin)>1e-7:
        raise ValueError("Mass does not reconcile")
    return {"sourceSha256":ledger.digest(path),"priorUpperMarginKg":before,"upperMarginKg":margin,
            "count":len(expected),"physicalMarginKg":result["mass"]["configurationMarginKg"]},bill


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for k in ("plugin","winchless_maps","output"):
        parser.add_argument(k,type=lambda s:Path(s).resolve())
    a=parser.parse_args()
    result={}
    for shelf in ("densityMatchedRemovalKg","noCutCreditKg"):
        result[shelf]={}
        for policy in ("completeGroupDeltaKg","unknownGroupDeltaKg"):
            case,bill=apply(a.plugin,a.winchless_maps/shelf/(policy+".json"),a.output/shelf/policy)
            result[shelf][policy]=case
    ledger.write(HERE/"accounting.json",{"plugin":ledger.read(a.plugin/"package.json")["version"],
                                         "bill":bill,"cases":result})
    print(result)
