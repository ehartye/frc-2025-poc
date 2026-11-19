"""Reconcile the relieved receiver with the actual axle, camera and cable purchasing map."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "frame-corners"))
from account import bind_sources, digest, read, write

SELECTED = {"low-lock","deck-lamp","moving-camera","no-match-air","four-welded-corners",
            "light","shop-supplies","axle-retention","inner-rail-joints"}


def bind_inputs(mapping, baseline, output):
    if set(mapping["selected"]) != SELECTED:
        raise ValueError("Expected the selected axle, wheel, weld and camera map")
    portable = []
    for source in mapping["sources"]:
        original = (baseline.parent/source["file"]).resolve()
        if digest(original) != source["sha256"]:
            raise ValueError("Upstream accounting source changed: "+source["id"])
        if source["id"] == "selected-shop-mass":
            for path, expected in read(original)["source_sha256"].items():
                if digest(ROOT/path) != expected:
                    raise ValueError("Selected shop inputs changed: "+path)
            copy = output/"upstream-shop-mass.json"
            if copy.resolve() != original:
                copy.write_bytes(original.read_bytes())
            source["file"] = copy.name
        else:
            portable.append(source)
    bind_sources(portable,baseline,ROOT,output)


def apply_clearance_kit(data, output):
    kit = read(HERE/"kit.json")
    if digest(HERE/"receiver-clearance-kit.step") != kit["kitSha256"]:
        raise ValueError("Clearance kit differs from its reopened receipt")
    old = kit["originalVolumeMm3"]
    size = kit["volumeMm3"]
    made = ["drilled-slide-L-inner","drilled-slide-R-inner","slide-position-saddle","slide-position-target"]
    # The selected low-lock bridge and shop shelf remain installed and are drilled.
    active_source = {s["id"] for s in data["instances"]}
    if not set(made+["low-lock/slide-base-bridge","receiver-R6-ported"]) <= active_source:
        raise ValueError("Different fixed-low bridge or receiver in shop map")
    metal = {k:size[k]-v for k,v in old.items() if k!="receiver-R6-drilled-for-camera"}
    shelf_cut = size["receiver-R6-drilled-for-camera"]-old["receiver-R6-drilled-for-camera"]
    removed = sum(min(0,v) for v in metal.values())
    added = sum(max(0,v) for v in metal.values())
    # Use cut credits only at the lower end; at the high end retain the full
    # previously booked stock. Gross cap for each received M3 screw and locknut.
    fasteners = {"flag-M3":("M3 x 25 flat head",.00230),"sensor-saddle-M3":("M3 x 12 flat head",.00150),
                 "flag-locknut":("M3 low locknut",.00090),"sensor-saddle-locknut":("M3 low locknut",.00090)}
    articles = sorted({name for name,_ in fasteners.values()})
    data["articles"].extend({"id":n,"sku":None,"unitsPerPack":None,"onHand":None} for n in articles)
    names = []
    caps = {}
    for key in sorted(set(size)-set(old)):
        article,cap = fasteners[key.rsplit("-",1)[0]]
        caps[key] = cap
        name = "clearance/"+key
        data["instances"].append({"id":name,"presence":"candidate","disposition":"buy","article":article})
        names.append(name)
    for key in made:
        name = "clearance/"+key
        data["instances"].append({"id":name,"presence":"candidate","disposition":"make"})
        names.append(name)
    delta = [removed*2.85e-6+added*2.65e-6+min(shelf_cut,0)*1.0e-6,
             added*2.85e-6+sum(caps.values())]
    source = HERE/"clearance-mass.json"
    write(source,{"kitSha256":kit["kitSha256"],"aluminumNetMm3":metal,"shelfCutMm3":shelf_cut,
                  "fastenerUpperKg":caps,"budgetDeltaKg":delta,
                  "inPlaceMachining":["low-lock/slide-base-bridge","receiver-R6-ported"]})
    data["sources"].append({"id":"clearance-fit-mass","file":os.path.relpath(source,output).replace(os.sep,"/"),
                            "sha256":digest(source)})
    data["changes"].append({"id":"clearance-fit","revision":data["revision"],"configuration":data["configuration"],
                            "remove":made,"add":names,"delta":{"source":"clearance-fit-mass","basis":"estimated",
                            "pointer":"/budgetDeltaKg","rangeKg":delta}})
    data["selected"].append("clearance-fit")


def run(plugin, baseline, output):
    data = read(baseline)
    output.mkdir(exist_ok=True, parents=True)
    bind_inputs(data,baseline,output)
    stock = read(ROOT/"station-relief/geometry.json")
    shims = read(ROOT/"station-relief/shims.json")
    if (digest(ROOT/"station-relief/cheeks.step") != stock["reopenedStepSha256"]
            or digest(ROOT/"station-relief/shims.step") != shims["nativeSha256"]):
        raise ValueError("Receiver solids differ from the registered machining receipts")
    identities = {row["id"] for row in data["instances"]}
    retained = set(shims["parts"]) | set(shims["stopChecks"])
    replaced = sorted(stock["stock"])
    successors = {"relieved/"+key:key for key in replaced}
    adds = [key+"-seat-shim" for key in shims["parts"]]
    if (len(retained) != 6 or len(replaced) != 8 or not retained.union(replaced) <= identities
            or (set(adds) | set(successors)) & identities):
        raise ValueError("Receiver articles do not match this installed purchasing revision")
    removed = sum(x["removed_mm3"] for x in stock["stock"].values())
    added = shims["fourShimVolumeMm3"]
    # kg/mm3 for the registered aluminum stock and steel shims. Credit no cut metal
    # at the upper bound. Changes in spring deflection do not purchase another spring.
    delta = [added*7.70e-6 - removed*2.85e-6, added*8.10e-6]
    source = HERE/"receiver-mass.json"
    write(source, {"machinedVolumeMm3":removed,"addedShimVolumeMm3":added,
                   "nominalDeltaKg":added*7.85e-6 - removed*2.70e-6,"budgetDeltaKg":delta,
                   "registeredCheekStep":stock["reopenedStepSha256"],"registeredShimStep":shims["nativeSha256"],
                   "retainedIdentities":sorted(retained),"replacementLocation":successors,"newShims":sorted(adds)})
    data["sources"].append({"id":"shop-relieved-receiver","sha256":digest(source),
                            "file":os.path.relpath(source,output).replace(os.sep,"/")})
    # New machined plates and inserts supersede stock at the same eight locations.
    data["instances"].extend({"id":name,"presence":"candidate","disposition":"make"} for name in successors)
    data["instances"].extend({"id":name,"presence":"candidate","disposition":"make"} for name in adds)
    change = {"id":"shop-relieved-receiver","revision":data["revision"],"configuration":data["configuration"],
              "add":adds+list(successors),"remove":replaced,"delta":{"source":"shop-relieved-receiver","basis":"estimated",
                                               "pointer":"/budgetDeltaKg","rangeKg":delta}}
    data["changes"].append(change)
    data["selected"].append(change["id"])
    apply_clearance_kit(data,output)
    target = output/"shop-receiver.json"
    write(target,data)
    evidence = json.loads(subprocess.check_output(["node",str(plugin/"scripts/assembly-accounting.mjs"),str(target)]))
    coverage = {k:evidence["coverage"][k] for k in ("missing","duplicates","orphanIncluded")}
    if any(coverage.values()):
        raise ValueError("Combined shop map has unmatched articles")
    write(HERE/"mass.json",{"installedVersion":read(plugin/"package.json")["version"],"addedShims":len(adds),
                            "replacedAtExistingLocations":len(replaced),"retainedSpringsAndStops":len(retained),"upstreamSources":len(data["sources"])-2,
                            "coverage":coverage,"representedKg":evidence["mass"]["representedRangeKg"],
                            "representedMarginKg":evidence["mass"]["representedMarginKg"],
                            "physicalMarginKg":evidence["mass"]["configurationMarginKg"]})


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for arg in ("plugin","baseline","output"):
        parser.add_argument(arg,type=Path)
    args = parser.parse_args()
    run(args.plugin.resolve(),args.baseline.resolve(),args.output.resolve())
