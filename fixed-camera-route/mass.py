"""Apply the removable harness guide and attachment budget to the issued SP1 plus optics map."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "frame-corners"))
from account import read, write, digest, bind_sources


def run(plugin, baseline, out):
    out.mkdir(parents=True, exist_ok=True)
    data = read(baseline)
    if data["selected"][-2:] != ["selected-optics", "sp1-install"]:
        raise ValueError("Requires the selected #93 plus #94 map")
    standard = []
    for s in data["sources"]:
        if s["id"] == "selected-shop-mass":
            original = (baseline.parent / s["file"]).resolve()
            if digest(original) != s["sha256"]:
                raise ValueError("Stale upstream stock")
            target = out / "upstream-shop-mass.json"
            if target != original:
                target.write_bytes(original.read_bytes())
            s["file"] = target.name
        else:
            standard.append(s)
    bind_sources(standard, baseline, ROOT, out)
    issue = read(HERE / "guides.json")
    if digest(HERE / "harness-guides.step") != issue["stepSha256"]:
        raise ValueError("Unregistered native kit")
    vol = issue["volumeMm3"]
    aluminum = sum(vol[k] for k in issue["supportBlanks"])
    nylon = sum(vol[k] for k in issue["removableTroughs"])
    # New mass only. Existing fixed adapter and its two leads, powered panel with
    # fixed wiring, complete roof USB lead, carrier and both project cameras stay booked.
    upper = {
        "finished-transition-box-board-and-terminations": 0.018,
        "mounting-closures-ties-lugs-and-finish": 0.035,
    }
    delta = [
        aluminum * 2.65e-6 + nylon * 0.9e-6,
        aluminum * 2.85e-6 + nylon * 1.20e-6 + sum(upper.values()),
    ]
    source = HERE / "mass-input.json"
    write(
        source,
        {
            "guidesStepSha256": issue["stepSha256"],
            "volumeMm3": {"aluminumBlanks": aluminum, "polymerGuide": nylon},
            "densityUpperKgPerMm3": {"aluminum": 2.85e-6, "polymer": 1.20e-6},
            "additionalReceivingCapsKg": upper,
            "preservedReceivingKg": {
                "panelAndFixedWiring": 0.750,
                "fixedAdapterAndBothLeads": 0.105,
                "roofFactoryDataLead": 0.170,
            },
            "rangeKg": delta,
            "measuredInstallation": None,
        },
    )
    data["sources"].append(
        {
            "id": "harness-issue",
            "file": os.path.relpath(source, out).replace(os.sep, "/"),
            "sha256": digest(source),
        }
    )
    names = ["fixed-leads/" + k for k in [*vol, *upper]]
    data["instances"].extend(
        {"id": k, "presence": "candidate", "disposition": "make"} for k in names
    )
    change = "fixed-lead-install"
    data["changes"].append(
        {
            "id": change,
            "revision": data["revision"],
            "configuration": data["configuration"],
            "add": names,
            "remove": [],
            "delta": {
                "source": "harness-issue",
                "pointer": "/rangeKg",
                "basis": "estimated",
                "rangeKg": delta,
            },
        }
    )
    data["selected"].append(change)
    mapfile = out / "fixed-camera-route.json"
    write(mapfile, data)
    report = json.loads(
        subprocess.check_output(
            ["node", str(plugin / "scripts/assembly-accounting.mjs"), str(mapfile)]
        )
    )
    coverage = {
        k: report["coverage"][k] for k in ("missing", "duplicates", "orphanIncluded", "unattached")
    }
    if any(coverage.values()):
        raise ValueError(coverage)
    prior = read(ROOT / "sp1-mount/optics-mass.json")["representedKg"]
    actual = report["mass"]["representedRangeKg"]
    if max(abs(actual[i] - prior[i] - delta[i]) for i in range(2)) > 1e-7:
        raise ValueError("Nonadditive accounting")
    write(
        HERE / "mass.json",
        {
            "priorRepresentedKg": prior,
            "netAddedKg": delta,
            "representedKg": actual,
            "representedMarginKg": report["mass"]["representedMarginKg"],
            "physicalMarginKg": report["mass"]["configurationMarginKg"],
            "addedFabricationAndReceivingGroups": len(names),
            "coverage": coverage,
        },
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("plugin", "baseline", "out"):
        p.add_argument(name, type=Path)
    a = p.parse_args()
    run(a.plugin.resolve(), a.baseline.resolve(), a.out.resolve())
