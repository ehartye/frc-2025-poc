"""Review the specified taper on the installed project's lower-bound checker and test both limit corners."""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT/"moving-harness"))
from power_budget import write

def build(plugin, folder):
    version = json.loads((plugin/"package.json").read_bytes())["version"]
    routepath = HERE/"route-proof.json"
    route = json.loads(routepath.read_bytes())
    old = json.loads((ROOT/"moving-harness/power/comparison.json").read_bytes())
    comparison = {**old,"trials":[]}
    window = comparison["window"]
    amps=1.2
    metal=(5.12-route["estimated_power_terminal_V"])/amps-.055
    rows={}
    folder.mkdir(parents=True,exist_ok=True)
    # Bench acceptance includes connector, fuse, PCB, Y silicon, and both sides of the return.
    # Use the same toleranced no-load test for every selection, when positive drop is smallest.
    for name,low,high,contact in (
        ("old-band-with-taper",5.13,5.20,.055),
        ("low-resistance-only",5.13,5.20,.038),
        ("selected-screen",5.16,5.20,.050),
        ("worn-selected",5.16,5.20,.100),
        ("overtrim-counterexample",5.20,5.25,.050),
    ):
        lower=low-.010-amps*(metal+contact)
        upper=high+.010
        condition=f"Screw-terminal band {low:.2f} to {high:.2f} V with total interface resistance bounded by {contact*1000:.0f} mOhm"
        src={"schema":"frc-power-trace/v1","actionId":comparison["actionId"],"busId":comparison["busId"],"clockId":comparison["clockId"],
             "trialId":name,"condition":condition,"units":{"time":"s","voltage":"V","current":"A"},
             "provenance":{"kind":"modeled","source":"Connector-resolved modeled load. Route SHA-256 "+hashlib.sha256(routepath.read_bytes()).hexdigest()},
             "rows":[{**window,"voltages":[{"location":"camera-input","kind":"lower-bound","valueV":lower}],
                      "currents":[{"id":"camera","kind":"source","valueA":amps}]}]}
        filename=name+".json"
        comparison["trials"].append({"id":name,"condition":condition,"trace":{"file":filename,"sha256":write(folder/filename,src)}})
        rows[name]={"loaded_min_V":lower,"no_load_max_V":upper,"within_both_limits":lower>=5 and upper<=5.25}
    cli=plugin/"scripts/power-review.mjs"
    selected=comparison["trials"][2]
    for alternative in comparison["trials"]:
        if alternative["id"] == selected["id"]:
            continue
        pair={**comparison,"trials":[alternative,selected]}
        source=folder/("compare-"+alternative["id"]+".json")
        write(source,pair)
        review=json.loads(subprocess.check_output(["node",str(cli),str(source)]))
        write(folder/("review-"+alternative["id"]+".json"),review)
    write(folder/"corners.json",{"plugin_version":version,"project_lower_V":5.0,"project_upper_V":5.25,"selected":"selected-screen",
                                "conditions":rows,"physically_measured":False,"functional_dropout_observed":False})

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plugin",type=Path)
    build(parser.parse_args().plugin,HERE/"power")
