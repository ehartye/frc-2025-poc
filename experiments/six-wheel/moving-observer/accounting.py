"""Combine the second stereo installation with the already selected low locks and deck light."""
import argparse
from importlib.util import spec_from_file_location,module_from_spec
import json
import os
from pathlib import Path
import subprocess
HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
spec=spec_from_file_location("issued_light",ROOT/"design/signal-light/accounting.py")
light=module_from_spec(spec)
spec.loader.exec_module(light)
lib=light.locks


def run(plugin,output):
    light.run(plugin,output)
    mapping=lib.read(output/"low-deck-lamp.json")
    material=lib.read(HERE/"mount.json")
    nominal=material["mass_before_cable_welds_and_power_kg"]
    # Preliminary kit values are allocation intervals. Use catalogue mass for the camera (61 g)
    # and +/-5% on solid-volume metal. All remaining installation articles require a weighing.
    cuts=nominal-.061
    extra={"weld_filler_kg":[.005,.015],"moving_USB_connector_clips_and_service_loop_kg":[.055,.100],
           "fixed_power_conversion_protection_wires_kg":[.030,.065]}
    low=.061+.95*cuts+sum(v[0] for v in extra.values())
    high=.061+1.05*cuts+sum(v[1] for v in extra.values())
    data={"nominal_kit_kg":nominal,"heavier_candidate_kg":.4299340915020383,
          "catalog_camera_kg":.061,"estimated_installation":extra,"incremental_budget_kg":[low,high],
          "stage_carried_kg":[low-extra["fixed_power_conversion_protection_wires_kg"][0],
                              high-extra["fixed_power_conversion_protection_wires_kg"][1]]}
    path=HERE/"mass.json"
    path.write_text(json.dumps(data,indent=2)+"\n",encoding="utf-8",newline="\n")
    mapping["sources"].append({"id":"camera-kit","file":os.path.relpath(path,output).replace(os.sep,"/"),"sha256":lib.sha(path)})
    mapping["articles"].append({"id":"second fixed-focus stereo","sku":"A00532","onHand":None,"unitsPerPack":1})
    instances=[]
    for k,row in material["parts"].items():
        kind="buy" if k=="camera-envelope" else "make" if row["material"]=="6061" else "undecided"
        # Individual fastening articles lack a qualified part number and supplied-pack mapping.
        item={"id":"roof-"+k,"presence":"candidate","disposition":kind}
        if kind=="buy":item["article"]="second fixed-focus stereo"
        instances.append(item)
    mapping["instances"].extend(instances)
    mapping["changes"].append({"id":"moving-camera","exclusiveGroup":"separate-vision","configuration":mapping["configuration"],
        "revision":mapping["revision"],"remove":[],"add":[v["id"] for v in instances],
        "delta":lib.evidence("camera-kit","/incremental_budget_kg",data["incremental_budget_kg"])})
    mapping["selected"].append("moving-camera")
    mapping["mass"]["omissions"].append("Weigh both leads, harness travel loop, power conversion, welds and complete installed camera.")
    final=output/"moving-camera.json"
    lib.write_map(final,mapping)
    result=json.loads(subprocess.check_output(["node",str(plugin/"scripts/assembly-accounting.mjs"),str(final)]))
    summary={"plugin":json.loads((plugin/"package.json").read_bytes())["version"],
             "represented_kg":result["mass"]["representedRangeKg"],"margin_kg":result["mass"]["representedMarginKg"],
             "configuration_margin_kg":result["mass"]["configurationMarginKg"]}
    (HERE/"accounting.json").write_text(json.dumps(summary,indent=2)+"\n",encoding="utf-8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--plugin",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    args=p.parse_args()
    run(args.plugin,args.output.resolve())
