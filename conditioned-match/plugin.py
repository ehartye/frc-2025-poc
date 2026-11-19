"""Use the installed power reviewer on the actually achieved opening drive's limiting samples."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

HERE=Path(__file__).resolve().parent

def write(path,value):
    path.write_text(json.dumps(value,indent=2)+"\n",encoding="utf8",newline="\n")
    return hashlib.sha256(path.read_bytes()).hexdigest()

def run(assets,plugin):
    ref=HERE/"opening.json"
    issued=json.loads(ref.read_bytes())
    out={}
    for case,row in issued.items():
        data=row["power"]
        start=data["screened"]["limitingWallS"]
        common={"actionId":"corrected-opening-"+case,"busId":"loaded-pack","clockId":"field-wall"}
        window={"fromS":start,"toS":round(start+.005,3)}
        root=assets/"conditioned-match/power-review"/case
        root.mkdir(exist_ok=True,parents=True)
        trials=[]
        for name in ("screened","weak"):
            r=data[name]
            if r["limitingWallS"]!= start or r["limitingBranch"]!="DriveRight":raise ValueError("The two cases do not share the operating point")
            condition="11.8 V with "+("27" if name=="screened" else "32")+" milliohm common resistance"
            trace={"schema":"frc-power-trace/v1",**common,"trialId":name,"condition":condition,
                   "units":{"time":"s","voltage":"V","current":"A"},
                   "provenance":{"kind":"modeled","source":"Recorded 5 ms drive window from opening.json, SHA256 "+hashlib.sha256(ref.read_bytes()).hexdigest()},
                   "rows":[{**window,"voltages":[{"location":"right-drive","kind":"lower-bound","valueV":r["minTerminalV"]}],
                            "currents":[{"id":"pack","kind":"source","valueA":r["commonCurrentAtMinimumA"]}]}]}
            path=root/(name+".json");h=write(path,trace)
            trials.append({"id":name,"condition":condition,"trace":{"file":path.name,"sha256":h}})
        comparison={"schema":"frc-power-comparison/v1",**common,"window":window,"maxSampleIntervalS":.005001,
                    "requiredLoads":["pack"],"locations":[{"id":"right-drive","minV":9.5}],"trials":trials}
        path=root/"comparison.json";h=write(path,comparison)
        v=json.loads(subprocess.check_output(["node",str(plugin/"scripts/power-review.mjs"),str(path)]))
        out[case]={"requestSha256":h,"review":v}
    write(HERE/"power-review.json",{"version":json.loads((plugin/"package.json").read_bytes())["version"],"cases":out})

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("assets",type=Path);p.add_argument("plugin",type=Path);a=p.parse_args();run(a.assets,a.plugin)
