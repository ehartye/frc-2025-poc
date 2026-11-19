"""Apply the four inner-rail joints once to the source-verified selected axle map."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
from joint import ROOT, HERE, digest, read, write

sys.path.insert(0,str(ROOT/"frame-corners"))
from account import bind_sources


def apply(plugin, baseline, out):
    data=read(baseline)
    if set(data["selected"])!={"light","moving-camera","deck-lamp","four-welded-corners","low-lock",
                              "no-match-air","shop-supplies","axle-retention"}:
        raise ValueError("Issue the axle-retention map before fitting inner rails")
    out.mkdir(parents=True,exist_ok=True)
    portable=[]
    for src in data["sources"]:
        old=(baseline.parent/src["file"]).resolve()
        if digest(old)!=src["sha256"]:
            raise ValueError("Wrong incoming map source: "+src["id"])
        if src["id"]=="selected-shop-mass":
            for relative,sha in read(old)["source_sha256"].items():
                if digest(ROOT/relative)!=sha:
                    raise ValueError("Wrong source in current checkout: "+relative)
            target=out/"upstream-shop-mass.json"
            target.write_bytes(old.read_bytes())
            src["file"]=target.name
        else:
            portable.append(src)
    bind_sources(portable,baseline,ROOT,out)
    fit=read(HERE/"fit.json")
    if digest(HERE/"inner-rail-joints.step")!=fit["stepSha256"]:
        raise ValueError("Reopen shop STEP before accounting")
    volumes=fit["volumeMm3"]
    names=set(volumes)
    groups={}
    for name in names:
        if name.startswith("drive-rail-inner-"):
            continue
        typ=name.split("-",2)[2]
        if typ.startswith("weld-"):
            kind="weld"
        elif "washer-" in typ:
            kind="washer"
        else:
            kind=typ.split("-")[0]
        groups.setdefault(kind,[]).append(name)
    counts={k:len(v) for k,v in groups.items()}
    if counts!={"ear":4,"weld":8,"pilot":4,"screw":8,"nut":8,"washer":16}:
        raise ValueError("Changed four-joint stack: "+str(counts))
    ear=sum(volumes[k] for k in groups["ear"])
    # Conditional stock and receiving ranges; upper conservatively credits no rail cut.
    ranges={"rail-machining":[-fit["removedRailMm3"]*2.85e-6,0],
            "four-ears":[ear*2.65e-6,ear*2.85e-6],"eight-fillet-regions":[.003,.012],
            "four-steel-pilots":[.009,.013],"eight-screws":[8*.015,8*.020],
            "eight-locknuts":[8*.004,8*.0065],"sixteen-washers":[16*.0015,16*.0021]}
    delta=[round(sum(row[i] for row in ranges.values()),7) for i in (0,1)]
    mass=HERE/"mass-input.json"
    write(mass,{"nativeFitSha256":digest(HERE/"fit.json"),"groupsKg":ranges,
                "deltaKg":delta,"basis":"Conditional density and loose-article receiving allowances; calibration and final robot weight pending."})
    data["sources"].append({"id":"inner-rail-mass","file":os.path.relpath(mass,out).replace(os.sep,"/"),"sha256":digest(mass)})
    supply={"screw":("M8 x 30 socket screw","SSC-M8-30-HK-12.9"),"washer":("M8 hardened washer","200125684"),
            "nut":("M8 all-metal nut class 10","03698"),"pilot":("6 x 12 ground and matched dowel","shop receiving specification")}
    existing={i["id"] for i in data["instances"]}
    if not {"drive-rail-inner-L","drive-rail-inner-R"}<=existing:
        raise ValueError("Charge the rail rework against the two issued rails")
    additions=[]
    for kind,ids in sorted(groups.items()):
        if kind=="weld":
            # Both beads are embodied in each welded ear; no extra bought metal part.
            continue
        if kind in supply:
            label,sku=supply[kind]
            data["articles"].append({"id":label,"sku":sku,"unitsPerPack":None,"onHand":None})
        for name in sorted(ids):
            part="inner-rail-joint/"+name
            if part in existing:
                raise ValueError("Joint already counted")
            item={"id":part,"disposition":"make","presence":"candidate"}
            if kind in supply:
                item.update(disposition="buy",article=supply[kind][0])
            additions.append(part)
            data["instances"].append(item)
    data["selected"].append("inner-rail-joints")
    data["changes"].append({"id":"inner-rail-joints","revision":data["revision"],"configuration":data["configuration"],
                            "add":additions,"remove":[],"delta":{"source":"inner-rail-mass","pointer":"/deltaKg",
                            "basis":"estimated","rangeKg":delta}})
    selected=out/"selected-rail-joints.json"
    write(selected,data)
    import json
    res=json.loads(subprocess.check_output(["node",str(plugin/"scripts/assembly-accounting.mjs"),str(selected)]))
    coverage={k:res["coverage"][k] for k in ("missing","duplicates","orphanIncluded")}
    if any(coverage.values()):
        raise ValueError(coverage)
    write(HERE/"accounting.json",{"installedVersion":read(plugin/"package.json")["version"],"coverage":coverage,
            "newPhysicalArticles":len(additions),"representedRangeKg":res["mass"]["representedRangeKg"],
            "representedMarginKg":res["mass"]["representedMarginKg"],"physicalMarginKg":res["mass"]["configurationMarginKg"]})


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for arg in ("plugin","baseline","out"):
        p.add_argument(arg,type=Path)
    a=p.parse_args()
    apply(a.plugin.resolve(),a.baseline.resolve(),a.out.resolve())
