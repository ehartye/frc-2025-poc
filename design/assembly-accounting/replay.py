"""Run the read-only Agent FRC CLI for both alliance models and each compatible alternative."""
import argparse
import json
import subprocess
from pathlib import Path
from tempfile import TemporaryDirectory

from build import BEARING, HERE, POSES, ROOT, build, digest, write_map


def run(checker, path):
    return subprocess.run(["node", str(checker), str(path)], capture_output=True, encoding="utf-8", check=False)


def summarize(report):
    cov = report["coverage"]
    if any(cov[key] for key in ("missing","duplicates","unattached","orphanIncluded")):
        raise ValueError("Occurrence coverage is incomplete")
    buy = {article["article"]:article for article in report["buy"]}
    transducer = "TE/Celesco SP1-25 25-in analog string pot with bracket"
    wire = cov["bindings"]["slide-position-SP1-25"]
    ropes = {side:sorted(row["occurrence"] for row in cov["bindings"]["secondary-line-"+side] if row["export"]=="stowed") for side in "LR"}
    return {"configuration": report["configuration"], "selected": report["selected"],
            "covered": cov["covered"], "roles": cov["roles"], "representedInstances":report["representedInstances"],
            "make":len(report["make"]), "buyUnits":sum(a["requiredUnits"] for a in buy.values()),
            "undecided":len(report["undecided"]), "inventoryComplete":report["inventoryComplete"],
            "orderQuantitiesKnown":report["orderQuantitiesKnown"], "secondaryLines":ropes,
            "includedDrawWirePerPose":sum(r["role"]=="included" for r in wire)//len(POSES),
            "transducerUnits":buy[transducer]["requiredUnits"], "primaryLines":buy["am-4559 primary line"]["requiredUnits"],
            "knownLooseWheelScrews":buy["Wheel-to-hub fasteners"]["requiredUnits"],
            "bearings":{k:buy[BEARING][k] for k in ("sku","requiredUnits","unitsPerPack","onHand","packsToCoverKnownShortfall")},
            "representedRangeKg":report["mass"]["representedRangeKg"],"representedMarginKg":report["mass"]["representedMarginKg"],
            "configurationMarginKg":report["mass"]["configurationMarginKg"]}


def replay(plugin, output):
    checker = plugin/"scripts/assembly-accounting.mjs"
    result = {"checkerSha256":digest(checker),"sourceReaderSha256":digest(plugin/"scripts/release-height.mjs"),
              "configurations":{},"controls":{}}
    scratch = ROOT/"artifacts/assembly-accounting"
    scratch.mkdir(parents=True,exist_ok=True)
    with TemporaryDirectory(dir=scratch) as work:
        path = Path(work)/"request.json"
        for color in ("red","blue"):
            data = build(color,path)
            cases = {}
            for carry in (None,"light","steel"):
                for stop in (False,True):
                    selection = ([carry] if carry else [])+(["stop"] if stop else [])
                    data["selected"] = selection
                    write_map(path,data)
                    completed = run(checker,path)
                    completed.check_returncode()
                    report=json.loads(completed.stdout)
                    cases["+".join(selection) or "baseline"]=summarize(report)
            result["configurations"][color]=cases
        controls = [
            ("incompatible-carrier-materials", lambda data: data.update(selected=["light","steel"]), "Incompatible alternatives"),
            ("changed-pinned-mass", lambda data: data["mass"]["baseline"]["rangeKg"].__setitem__(1, 50), "differs from pinned"),
            ("changed-native-step", lambda data: next(p for p in data["sources"] if p["id"]=="place-step").update(sha256="0"*64), "digest mismatch"),
        ]
        for label, mutate, message in controls:
            data = build("red",path)
            mutate(data)
            write_map(path,data)
            completed=run(checker,path)
            if completed.returncode == 0 or message.lower() not in completed.stderr.lower():
                raise ValueError(f"Expected {label} to be rejected: {completed.stderr}")
            result["controls"][label]={"rejected":True,"reason":message}
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8",newline="\n")


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plugin-root",type=Path,required=True)
    parser.add_argument("--output",type=Path,default=ROOT/"artifacts/assembly-accounting/review.json")
    args=parser.parse_args()
    replay(args.plugin_root.resolve(),args.output.resolve())

