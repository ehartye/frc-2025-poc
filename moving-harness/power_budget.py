"""Reconcile the moving harness route with the installed electrical review."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0,str(ROOT / "fixed-low-power"))
import dual_camera


def write(path, obj):
    path.write_text(json.dumps(obj,indent=2)+"\n", encoding="utf-8", newline="\n")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def evaluate(plugin, folder):
    geometry = json.loads((HERE/"geometry.json").read_bytes())
    source = ROOT/"fixed-low-power/dual_camera.py"
    spec = dual_camera.SUPPLIES["MovingCamera"]
    _, _, awg, old_length, interfaces, amps = spec
    copper_per_m = 2*1.25*dual_camera.power.electric.AWG[awg]
    planned_length = 1.90
    original_min = dual_camera.inlet("MovingCamera")["conditionalMinV"]
    nominal = original_min - amps * copper_per_m * (planned_length-old_length)
    common = {"actionId":"loaded-camera-on-moving-shelf","busId":"camera-converter-output",
              "clockId":"steady-load-budget"}
    window = {"fromS":0,"toS":0.02}
    comparison = {"schema":"frc-power-comparison/v1", **common, "window":window,
                  "maxSampleIntervalS":.02,"requiredLoads":["camera"],
                  "locations":[{"id":"camera-input","minV":5.0}],"trials":[]}
    folder.mkdir(parents=True,exist_ok=True)
    for key, extra in (("screened",0),("worn",.05)):
        condition = f"1.9 m moving-camera lead with {extra:g} ohm added interface resistance"
        trace = {"schema":"frc-power-trace/v1", **common,"trialId":key,"condition":condition,
                 "units":{"time":"s","voltage":"V","current":"A"},
                 "provenance":{"kind":"modeled","source":
                    "fixed-low-power/dual_camera.py SHA-256 "+dual_camera.sha(source)+
                    ". Conditional DC budget on the regulator output, steady 1.20 A equivalent load. "
                    "The selected cable must meet the modeled loop resistance under temperature and flex."},
                 "rows":[{**window,"voltages":[{"location":"camera-input","kind":"lower-bound",
                                                "valueV":nominal-amps*extra}],
                          "currents":[{"id":"camera","kind":"source","valueA":amps}]}]}
        filename = key+".json"
        comparison["trials"].append({"id":key,"condition":condition,
                                     "trace":{"file":filename,"sha256":write(folder/filename,trace)}})
    write(folder/"comparison.json",comparison)
    cli = plugin/"scripts/power-review.mjs"
    review = json.loads(subprocess.check_output(["node",str(cli),str(folder/"comparison.json")]))
    if [row["status"] for row in review["trials"]] != ["complete","incomplete"]:
        raise ValueError("Voltage bound classification changed")
    write(folder/"review.json",review)
    return {"routePowerLengthBudgetMm":planned_length*1000,
            "remainingPowerTailAndTerminationsMm":planned_length*1000-geometry["cables"]["power"]["toHandoffMm"],
            "remainingDataTailAndTerminationsMm":2100-geometry["cables"]["data"]["toHandoffMm"],
            "conditionalMinV":nominal,"wornLowerBoundV":nominal-.05*amps,
            "assumedHotWireLoopOhm":copper_per_m*planned_length,"assumedInterfaceOhm":interfaces,
            "maxHotWireAndInterfacesOhm":(5.13-.010-5.0)/amps,
            "remainingUnmodeledResistanceOhm":(nominal-5.0)/amps,
            "reviewCliSha256":dual_camera.sha(cli),
            "pluginVersion":json.loads((plugin/"plugin.json").read_bytes())["version"]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plugin",type=Path)
    parser.add_argument("folder",type=Path)
    args = parser.parse_args()
    write(args.folder/"budget.json",evaluate(args.plugin,args.folder))
