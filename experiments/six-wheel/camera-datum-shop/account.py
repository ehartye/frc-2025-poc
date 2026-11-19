"""Select the shop roof in the installed assembly checker, counting the replaced camera kit once."""
import argparse,json,os,subprocess
from pathlib import Path
from camera_parts import HERE,ROOT,SHELF,read,sha

def run(plugin,baseline_map,out):
    out.mkdir(parents=True,exist_ok=True)
    mapping=read(baseline_map)
    old=next(c for c in mapping["changes"] if c["id"]=="moving-camera")
    if old["id"] not in mapping["selected"] or "no-match-air" not in mapping["selected"]:raise ValueError("Expected selected fixed-low camera, no-air map")
    old_names=set(old["add"])
    mapping["instances"]=[i for i in mapping["instances"] if i["id"] not in old_names]
    mapping["sources"]=[s for s in mapping["sources"] if s["id"]!="camera-kit"]
    for s in mapping["sources"]:
        original=(baseline_map.parent/s["file"]).resolve()
        if sha(original)!=s["sha256"]:raise ValueError("Source changed: "+s["id"])
        s["file"]=os.path.relpath(original,out).replace(os.sep,"/")
    parts=read(HERE/"native.json")["parts"]
    prior=read(ROOT/"moving-observer/mass.json")
    metal=sum(v["kg"] for k,v in parts.items() if k not in (SHELF,"camera-envelope"))
    new_install=[.061+scale*metal+sum(v[i] for v in prior["estimated_installation"].values()) for i,scale in enumerate((.95,1.05))]
    evidence={"complete_installation_kg":new_install,"native_sha256":sha(HERE/"native.json"),"held_installation_allowances":prior["estimated_installation"]}
    source=HERE/"shop-mass.json"
    source.write_text(json.dumps(evidence,indent=2)+"\n",encoding="utf8",newline="\n")
    mapping["sources"].append({"id":"shop-roof","file":os.path.relpath(source,out).replace(os.sep,"/"),"sha256":sha(source)})
    old["delta"]={"source":"shop-roof","pointer":"/complete_installation_kg","rangeKg":new_install,"basis":"estimated"}
    old["add"]=[]
    for k,v in parts.items():
        if k==SHELF:continue # Original installed shelf is retained once, with additional material removed.
        ident="roof-"+k; old["add"].append(ident)
        make=k.endswith(("saddle","datum-foot","post","tube","keepers")) or k.startswith("camera-M4")
        item={"id":ident,"presence":"candidate","disposition":"make" if make else "undecided"}
        if k=="camera-envelope":item.update(disposition="buy",article="second fixed-focus stereo")
        mapping["instances"].append(item)
    output=out/"shop-roof.json"
    output.write_text(json.dumps(mapping,indent=2)+"\n",encoding="utf8",newline="\n")
    result=json.loads(subprocess.check_output(["node",str(plugin/"scripts/assembly-accounting.mjs"),str(output)]))
    report={"plugin":read(plugin/"package.json")["version"],"coverage":{k:v for k,v in result["coverage"].items() if k in ("missing","duplicates","orphanIncluded")},
        "mass":result["mass"],"added_roof_instances":len(old["add"]),"replaced_roof_instances":len(old_names)}
    (HERE/"accounting.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for n in ("plugin","baseline-map","out"):p.add_argument("--"+n,type=Path,required=True)
    a=p.parse_args();run(a.plugin.resolve(),a.baseline_map.resolve(),a.out.resolve())
