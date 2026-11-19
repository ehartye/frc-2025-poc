"""Replace overlapping installation allowances once and evaluate three power/cable mass cases."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
from routes import HERE,read

ROOT=HERE.parent

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def write(p,data):
    p.write_text(json.dumps(data,indent=2)+"\n",encoding="utf8",newline="\n")
    return sha(p)

def run(plugin, baseline, out):
    out.mkdir(parents=True,exist_ok=True)
    mapping=read(baseline)
    for source in mapping["sources"]:
        p=(baseline.parent/source["file"]).resolve()
        if source["id"]=="shop-roof":
            p=ROOT/"camera-datum-shop/shop-mass.json"
        if sha(p)!=source["sha256"]:
            raise ValueError("Original mass source changed: "+source["id"])
        source["file"]=os.path.relpath(p,out).replace(os.sep,"/")
    old=next(c for c in mapping["changes"] if c["id"]=="moving-camera")
    if old["id"] not in mapping["selected"]:raise ValueError("No selected camera")
    roof=read(ROOT/"camera-datum-shop/shop-mass.json")
    allowance=roof["held_installation_allowances"]
    old_total=roof["complete_installation_kg"]
    rolled=[sum(allowance[k][i] for k in ("moving_USB_connector_clips_and_service_loop_kg","fixed_power_conversion_protection_wires_kg")) for i in (0,1)]
    native=read(HERE/"support.json")
    # These pending installed scale ceilings do not turn an unresolved purchase or as-built weight into evidence.
    groups={"fixed_complete_panel_with_fixed_wiring":[0,.750],"23_carrier_links":[.2806,.2806],"complete_Newnex_including_plugs":[0,.170],
            "cut_LAPP_bulk_power":[0,.220],"DM6010":[.005,.005],"short_power_pigtail_less_than_complete_donor":[0,.03225],
            "original_support_and_board":[native["nominal_solid_kg"]*.95,native["nominal_solid_kg"]*1.05],
            "local_liners_solder_retainers_and_finish":[0,.025],"carrier_guides_supports_and_park_hardware":[0,.120]}
    total=[old_total[i]-rolled[i]+sum(g[i] for g in groups.values()) for i in (0,1)]
    result={"selected_complete_installation_kg":total,"removed_prior_ranges_kg":rolled,"groups_kg":groups,"scale_verified":False,
            "installed_coverage":"Full selected supply and cable map. The two bulk cables and chain are counted once."}
    mass_file=HERE/"mass-conditions.json";fsha=write(mass_file,result)
    mapping["sources"].append({"id":"connector-installation","file":os.path.relpath(mass_file,out).replace(os.sep,"/"),"sha256":fsha})
    old["delta"]={"source":"connector-installation","pointer":"/selected_complete_installation_kg","basis":"estimated","rangeKg":total}
    def instance(name,disposition,article=None,sku=None):
        item={"id":name,"presence":"candidate","disposition":disposition}
        if article:
            mapping["articles"].append({"id":article,"sku":sku,"unitsPerPack":None,"onHand":None})
            item["article"]=article
        mapping["instances"].append(item);old["add"].append(name)
    for name in native["parts"]:
        instance("camera-service-"+name,"make" if not ("M3-" in name or "M2-" in name) else "undecided")
    for article,sku in (("selected moving high-flex USB","U3HLA01C12-020"),("moving power bulk cut","1028952"),
                        ("dual USB-C power and data adapter","DM6010"),("short moving power donor","10-03675"),
                        ("selected chain as 23-link assembly",".".join(("10","025","100","0")))):
        instance("service-"+sku,"buy",article,sku)
    selected=out/"selected.json";write(selected,mapping)
    cli=plugin/"scripts/assembly-accounting.mjs"
    report=json.loads(subprocess.check_output(["node",str(cli),str(selected)]))
    write(HERE/"accounting.json",{"installed_version":read(plugin/"package.json")["version"],"source_sha256":fsha,
          "native_support_count":native["native_count"],"coverage":{k:report["coverage"][k] for k in ("missing","duplicates","orphanIncluded")},
          "represented_range_kg":report["mass"]["representedRangeKg"],"represented_margin_kg":report["mass"]["representedMarginKg"],
          "physical_margin_kg":report["mass"]["configurationMarginKg"]})

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for k in ("plugin","baseline","out"):p.add_argument(k,type=Path)
    a=p.parse_args();run(a.plugin,a.baseline,a.out)
