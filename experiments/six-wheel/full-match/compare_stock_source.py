"""Compare historical floor contact reports with the independently installed object extents."""
import argparse
import json
import os
from pathlib import Path
import subprocess

from match import corrected
from mission.review_combined_departure import digest
from mission.check_branch_exclusion import FIRST_SHA

HERE = Path(__file__).resolve().parent


def compare(folder, field_step, plugin):
    def read(ref):
        path = folder/ref["file"]
        if digest(path) != ref["sha256"]:
            raise ValueError("Stale source: "+path.name)
        return json.loads(path.read_text())

    def ref(path):
        return {"file": os.path.relpath(path,folder).replace(os.sep,"/"), "sha256":digest(path)}

    def write(name,data):
        path = folder/("installed-"+name+".json")
        path.write_text(json.dumps(data,separators=(",",":"))+"\n",encoding="utf8",newline="\n")
        return ref(path)

    stock = json.loads((HERE/"installed-stock.json").read_text())
    if digest(field_step) != stock["field_step_sha256"] or digest(field_step) != FIRST_SHA:
        raise ValueError("Changed installed field")
    f = corrected()
    obstacles = {}
    occurrences = {}
    for occurrence, obj in stock["stock"].items():
        if obj["group"] == "reef-ball":
            continue
        x,y,z,xx,yy,zz = obj["native_bounds_mm"]
        a,b = f.field_from_native(x/25.4,zz/25.4)
        c,d = f.field_from_native(xx/25.4,z/25.4)
        cx,_,cz = obj["native_center_mm"]
        label = ("red" if cx>0 else "blue")+"-"+("upper" if cz < -1 else "lower" if cz > 1 else "middle")
        label += "-algae" if obj["group"] == "floor-ball" else "-coral"
        if label in obstacles:
            raise ValueError("Ambiguous native occurrence: "+label)
        obstacles[label] = [a,b,y/1000,c,d,yy/1000]
        occurrences[label] = occurrence
    old = json.loads((folder/"request.json").read_text())
    scenario, sweep, events, inventory = [read(old[k]) for k in ("scenario","route","detector","inventory")]
    scenario["sources"]["field"] = ref(field_step)
    inventory["fieldSha256"] = FIRST_SHA
    inventory["export"] = write("floor-export",{"fieldSha256":FIRST_SHA,"slab_obstacles":obstacles,
                                                "occurrences":occurrences,"evidence":ref(HERE/"installed-stock.json")})
    s = write("scenario",scenario)
    for doc in (events,sweep):
        doc["scenarioSha256"] = s["sha256"]
    refinements = []
    for i,previous in enumerate(sweep["refinements"]):
        item = read(previous)
        item["scenarioSha256"] = s["sha256"]
        refinements.append(write("refinement-"+str(i),item))
    sweep["refinements"] = refinements
    request = write("request",{"scenario":s,"inventory":write("inventory",inventory),
                               "route":write("sweep",sweep),"detector":write("events",events)})
    result = subprocess.run(["node",str(plugin/"scripts/field-route-evidence.mjs"),str(folder/request["file"])],
                            capture_output=True,text=True,check=True)
    full = json.loads(result.stdout)
    write("result",full)
    row = full["events"][0]
    return {"plugin":json.loads((plugin/".codex-plugin/plugin.json").read_text())["version"],
            "field_step_sha256":FIRST_SHA,"declared":row["declaredRouteStatus"],"route":row["routeStatus"],
            "whole_field":row["wholeFieldStatus"],"reported_contacts":row["contacts"],
            "inventory_issues":sorted({v["reason"] for v in row["inventoryGaps"]}),
            "native_hits_at_same_sample":stock["earlier_route_native_check"]["hits"]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("case","field","plugin","output"):
        parser.add_argument("--"+key,type=Path,required=True)
    args = parser.parse_args()
    args.output.write_text(json.dumps(compare(args.case,args.field,args.plugin),indent=2)+"\n",encoding="utf8",newline="\n")

