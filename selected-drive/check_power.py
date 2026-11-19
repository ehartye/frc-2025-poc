"""Use installed Agent FRC to compare a fixed worst-demand operating point on two packs."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

HERE=Path(__file__).resolve().parent

def write(path,value):
    path.write_text(json.dumps(value,indent=2)+"\n",encoding="utf8",newline="\n")
    return hashlib.sha256(path.read_bytes()).hexdigest()

def run(plugin):
    folder=HERE/"power";folder.mkdir(exist_ok=True)
    source=HERE/"results.json"
    sha=hashlib.sha256(source.read_bytes()).hexdigest()
    cases=json.loads(source.read_bytes())["cases"]["nominal"]["cases"]
    common={"actionId":"selected-drive-spin","busId":"main-pack","clockId":"match-wall"}
    trials=[]
    start=cases["screened"]["limitingWallS"]
    window={"fromS":start,"toS":round(start+.005,3)}
    for name in ("screened","weak"):
        row=cases[name]
        if row["limitingWallS"]!=start or row["limitingBranch"]!="DriveRight":
            raise ValueError("Profiles no longer share the registered worst operating point")
        condition="11.8 V pack with "+("27" if name=="screened" else "32")+" milliohm common source"
        trace={"schema":"frc-power-trace/v1",**common,"trialId":name,"condition":condition,
               "units":{"time":"s","voltage":"V","current":"A"},
               "provenance":{"kind":"modeled","source":"Held five-millisecond aggregate model demand from results.json "+sha},
               "rows":[{**window,"voltages":[{"location":"drive-right","kind":"lower-bound","valueV":row["minTerminalV"]}],
                        "currents":[{"id":"pack","kind":"source","valueA":row["commonCurrentAtMinimumA"]}]}]}
        file=name+".json"
        trials.append({"id":name,"condition":condition,"trace":{"file":file,"sha256":write(folder/file,trace)}})
    comparison={"schema":"frc-power-comparison/v1",**common,"window":window,"maxSampleIntervalS":.005001,
                "requiredLoads":["pack"],"locations":[{"id":"drive-right","minV":9.5}],"trials":trials}
    write(folder/"comparison.json",comparison)
    out=subprocess.check_output(["node",str(plugin/"scripts/power-review.mjs"),str(folder/"comparison.json")])
    write(folder/"review.json",json.loads(out))

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("plugin",type=Path)
    run(parser.parse_args().plugin)
