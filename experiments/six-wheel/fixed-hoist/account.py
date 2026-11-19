"""Apply only the explicitly booked secondary haul removals to the capped, locked robot."""
import argparse
import json
import os
import subprocess
from pathlib import Path

from fit import ENDS, HERE, ROOT, SHAFT, j, retired


def installed(plugin, path):
    result = subprocess.check_output(["node", str(plugin/"scripts/assembly-accounting.mjs"), str(path)])
    return json.loads(result)


def identities(result):
    return set(result["make"]) | set(result["undecided"]) | {
        key for article in result["buy"] for key in article["instances"]
    }


def run(plugin, source, out, receipt):
    source = source.resolve()
    out.mkdir(parents=True, exist_ok=True)
    model = j.read(source)
    before = installed(plugin, source)
    changes = {row["id"]:row for row in model["changes"] if row["id"] in model["selected"]}
    required = {"low-lock","no-match-air","fixed-lift-motor-omitted","supported-drive-covers",
                "integral-deck","fixed-lead-install","deck-through-ties"}
    if not required <= changes.keys() or changes["fixed-lift-motor-omitted"]["delta"]["pointer"]!="/restrictedNoMotorCreditKg":
        raise ValueError("Select the fixed low locks and zero-credit motor omission on the dressed ribbed deck")
    original_bill = ROOT/"design/competition-package/shop-duplex/mass.json"
    inputs = {}
    for item in model["sources"]:
        path = (source.parent/item["file"]).resolve()
        if j.digest(path)!=item["sha256"]:
            raise ValueError("Selected source changed: "+item["id"])
        inputs[item["id"]] = path
        item["file"] = os.path.relpath(path,out).replace(os.sep,"/")
    if j.read(inputs["selected-shop-mass"])["source_sha256"][original_bill.relative_to(ROOT).as_posix()]!=j.digest(original_bill):
        raise ValueError("Incorrect original mass ledger")
    booking = ROOT/"design/receiver-lift/delivery-retention.json"
    pins = j.read(original_bill)["sourceSha256"]
    if any(j.digest(ROOT/k)!=h for k,h in pins.items()):
        raise ValueError("Original static mass allocation changed")
    bill = j.read(booking)
    masses = {**bill["candidateFixedAddedKg"],**bill["candidateMovingAddedKg"]}
    rows = {r["occurrence"]:r["instance"] for r in model["assignments"] if r["export"]=="place" and r.get("instance")}
    physical = {rows[name] for name in retired()}
    if {k for k,v in rows.items() if v in physical}!=retired():
        raise ValueError("A retired article still has an installed occurrence")
    incoming = identities(before)
    removed = physical | {"long-common-shaft"}
    if not removed<=incoming:
        raise ValueError("Some removals are absent in the selected configuration")
    fit = j.read(HERE/"fit.json")
    if fit["stepSha256"]!=j.digest(HERE/"stationary-shaft.step"):
        raise ValueError("Wrong exchange")
    trim = fit["trimmedEndsVolumeMm3"]*7.85e-6
    if abs(trim-.8*masses["longer-steel-hex-shaft"])>1e-8:
        raise ValueError("Shaft trim exceeds separately booked extension share")
    gross = sum(masses[k] for k in retired())+trim
    # Per-piece receiving windows in grams. Weigh each dressed pair before taking credit.
    limits = {"sleeve":(6,10),"stop":(7,11),"screw":(3,6),"locknut":(.9,2)}
    ranges = [sum(2*v[n] for v in limits.values())/1000 for n in (0,1)]
    articles = {"screw":"M4 x 30 cross screw","locknut":"M4 prevailing-torque locknut"}
    for name in articles.values():
        if name not in {r["id"] for r in model["articles"]}:
            model["articles"].append({"id":name,"sku":None,"unitsPerPack":None,"onHand":None})
    new = {SHAFT}
    model["instances"].append({"id":SHAFT,"presence":"candidate","disposition":"make"})
    for kind, limit in limits.items():
        for side,_ in ENDS:
            name = f"haul-{kind}-{side}"
            new.add(name)
            nominal = fit["volumeMm3"][name]*(7.85 if kind in articles else 2.70)/1000
            if not limit[0]<=nominal<=limit[1]:
                raise ValueError("Receiving range excludes nominal "+name)
            row = {"id":name,"presence":"candidate","disposition":"make"}
            if kind in articles:
                row.update(disposition="buy",article=articles[kind])
            model["instances"].append(row)
    delta = [v-gross for v in ranges]
    evidence = {"removedOccurrencesKg":{k:masses[k] for k in sorted(retired())},"retiredArticles":sorted(physical),
                "oldBookedExtensionKg":masses["longer-steel-hex-shaft"],"trimCreditKg":trim,
                "grossRecoveryKg":gross,"addedPartsReceivingGramsEach":limits,
                "addedKg":ranges,"deltaKg":delta,"retainedLooseAllowancesKg":[bill["additionalFixedHardwareAllowanceKg"],
                bill["additionalMovingTerminationAllowanceKg"]],"physicalWeightKg":None}
    material = out/"fixed-hoist-ledger.json"
    j.write(material,evidence)
    model["sources"].append({"id":"fixed-hoist-bill","file":material.name,"sha256":j.digest(material)})
    model["changes"].append({"id":"fixed-secondary-haul","configuration":model["configuration"],
        "revision":model["revision"],"remove":sorted(removed),"add":sorted(new),
        "delta":{"source":"fixed-hoist-bill","basis":"estimated","pointer":"/deltaKg","rangeKg":delta}})
    model["selected"].append("fixed-secondary-haul")
    selected = out/"selected.json"
    j.write(selected,model)
    after = installed(plugin,selected)
    if identities(after)!=(incoming-removed)|new or any(after["coverage"][k] for k in ("missing","duplicates","unattached","orphanIncluded")):
        raise ValueError("Changed unrelated installed identities or coverage")
    prior,final = before["mass"],after["mass"]
    if abs(final["representedMarginKg"][0]-prior["representedMarginKg"][0]+delta[1])>1e-8:
        raise ValueError("Upper mass mismatch")
    j.write(receipt,{"pluginVersion":j.read(plugin/"package.json")["version"],
        "sourceMapSha256":j.digest(source),"sourceBillSha256":j.digest(booking),"fitSha256":j.digest(HERE/"fit.json"),
        "evidence":evidence,"installedBefore":len(incoming),"installedAfter":len(identities(after)),
        "priorUpperMarginKg":prior["representedMarginKg"][0],"representedRangeKg":final["representedRangeKg"],
        "representedMarginKg":final["representedMarginKg"],"physicalMarginKg":final["configurationMarginKg"]})


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ("plugin","source","out"):
        parser.add_argument(key,type=lambda value:Path(value).resolve())
    parser.add_argument("--receipt",type=Path,default=HERE/"budget.json")
    args=parser.parse_args()
    run(args.plugin,args.source,args.out,args.receipt)
