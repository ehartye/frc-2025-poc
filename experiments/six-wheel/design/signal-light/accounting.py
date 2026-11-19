"""Add the selected light and keyed mount to the fixed-height assembly-accounting cases."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess

HERE=Path(__file__).resolve().parent
LOCKS=HERE.parent/"fixed-height"
spec=importlib.util.spec_from_file_location("lock_accounting",LOCKS/"review.py")
locks=importlib.util.module_from_spec(spec)
spec.loader.exec_module(locks)


def build_map(plugin, output, mode, color):
    output.mkdir(parents=True,exist_ok=True)
    records=locks.read(LOCKS/"native.json")
    study=locks.read(HERE/"review.json")
    case=color if mode=="high" else "low-"+color+"-place"
    basis=records["red" if mode=="high" else "low-red-place"]
    if any(abs(x-y)>1e-9 for x,y in zip(locks.mass(records[case])["netDeltaKg"],locks.mass(basis)["netDeltaKg"])):
        raise ValueError("Alliance hardware mass differs")
    locks.review_map(plugin,mode,color,basis,output)
    mapping=locks.read(output/(mode+"-"+color+".json"))
    for key,artifact in [("light-study",HERE/"review.json"),("keyed-deck",HERE/"deck-mount.step")]:
        mapping["sources"].append({"id":key,"file":Path(os.path.relpath(artifact,output)).as_posix(),
                                   "sha256":locks.sha(artifact)})
    mapping["articles"].append({"id":"AndyMark signal light","sku":"am-3583","onHand":None,"unitsPerPack":1})
    parts=[{"id":"signal-light","presence":"candidate","disposition":"buy","article":"AndyMark signal light"},
           {"id":"keyed-deck","presence":"candidate","disposition":"make"},
           {"id":"lamp-spacer","presence":"candidate","disposition":"make"}]
    mapping["instances"].extend(parts)
    mapping["changes"].append({"id":"deck-lamp","exclusiveGroup":"signal-light","configuration":mapping["configuration"],
        "revision":mapping["revision"],"remove":["rsl-reserve","front-electronics-platform"],"add":[p["id"] for p in parts],
        "delta":locks.evidence("light-study","/incremental_budget_kg",study["incremental_budget_kg"])})
    mapping["selected"].append("deck-lamp")
    mapping["mass"]["omissions"].append("Weigh the selected lamp, spacer, terminals and leads in the finished robot.")
    return mapping


def run(plugin, output):
    report={}
    for mode in ("high","low"):
        mapping=build_map(plugin,output,mode,"red")
        final=output/(mode+"-deck-lamp.json")
        locks.write_map(final,mapping)
        result=json.loads(subprocess.check_output(["node",str(plugin/"scripts/assembly-accounting.mjs"),str(final)]))
        report[mode]={k:result["mass"][k] for k in ("representedRangeKg","representedMarginKg","configurationMarginKg")}
        report[mode]["missing"]=result["coverage"]["missing"]
    return report


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plugin",type=Path)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    report={"plugin":locks.read(args.plugin/"package.json")["version"],"cases":run(args.plugin,args.output.resolve())}
    (HERE/"accounting.json").write_bytes((json.dumps(report,indent=2)+"\n").encode())
