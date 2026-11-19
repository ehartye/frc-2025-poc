"""Apply frame welds once to the selected electrical and cable configuration."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
CAMERA_MASS = "7d1a49a624c920c7985a07596ec56bd7f15a6687028f6ffabf0afb1e138001bf"


def read(path):
    return json.loads(path.read_bytes())


def write(path, value):
    path.write_text(json.dumps(value,indent=2)+"\n",encoding="utf-8",newline="\n")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def apply(plugin, baseline, output):
    output.mkdir(parents=True,exist_ok=True)
    data = read(baseline)
    sources = {s["id"]:s for s in data["sources"]}
    active = [c for c in data["changes"] if c["id"] in data["selected"]]
    if (sources["connector-installation"]["sha256"] != CAMERA_MASS
        or not any(c["delta"]["source"]=="connector-installation" for c in active)
        or "four-welded-corners" in data["selected"]):
        raise ValueError("Select the issued moving harness and apply the frame changes exactly once")
    for source in data["sources"]:
        path = (baseline.parent/source["file"]).resolve()
        if digest(path) != source["sha256"]:
            raise ValueError("Baseline source has changed: "+source["id"])
        source["file"] = os.path.relpath(path,output).replace(os.sep,"/")
    fit = read(HERE/"weld-fit.json")
    evidence = {"cornerStudySha256":digest(HERE/"weld-fit.json"),"nominalNetKg":fit["nominalNetKg"],
                "estimatedNetRangeKg":[0,.020],"scaleVerified":False,
                "basis":"Fabrication allocation for four completed corners including dressing. Weigh the bare rails before and the joined frame after."}
    source = HERE/"weld-mass.json"
    write(source,evidence)
    data["sources"].append({"id":"corner-mass","file":os.path.relpath(source,output).replace(os.sep,"/"),
                            "sha256":digest(source)})
    names = ["welded-joint-"+corner for corner in fit["corners"]]
    data["instances"].extend({"id":k,"presence":"candidate","disposition":"make"} for k in names)
    data["changes"].append({"id":"four-welded-corners","revision":data["revision"],"configuration":data["configuration"],
                            "remove":[],"add":names,"delta":{"source":"corner-mass","pointer":"/estimatedNetRangeKg",
                            "basis":"estimated","rangeKg":evidence["estimatedNetRangeKg"]}})
    data["selected"].append("four-welded-corners")
    selected = output/"joined-frame.json"
    write(selected,data)
    result = json.loads(subprocess.check_output(["node",str(plugin/"scripts/assembly-accounting.mjs"),str(selected)]))
    coverage = {k:result["coverage"][k] for k in ("missing","duplicates","orphanIncluded")}
    if any(coverage.values()):
        raise ValueError("Incomplete or duplicate mapped components")
    write(HERE/"accounting.json",{"installedVersion":read(plugin/"package.json")["version"],"coverage":coverage,
          "representedRangeKg":result["mass"]["representedRangeKg"],
          "representedMarginKg":result["mass"]["representedMarginKg"],
          "physicalMarginKg":result["mass"]["configurationMarginKg"],"selectedChanges":data["selected"]})


if __name__=="__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("plugin","baseline","output"):
        parser.add_argument(name,type=Path)
    args=parser.parse_args()
    apply(args.plugin.resolve(),args.baseline.resolve(),args.output.resolve())
