"""Compare the booked inspection mass with four populated MAXSwerve modules."""

import argparse
import csv
import json
from pathlib import Path
import subprocess
import numpy as np
from screen import sha, write
from electrical import ROOT, HERE


def read(path):
    return json.loads(path.read_bytes())


OLD_PREFIXES = (
    "carrier-",
    "bearing-",
    "takeup-tab-",
    "jack-",
    "wheel-",
    "hub-",
    "axle-",
    "spacer-front-",
    "spacer-center-",
    "spacer-rear-",
    "chain-",
    "sprocket-",
    "gearbox-",
    "motor-",
)


def baseline_removals():
    """Original R103 baseline row ranges at matched endpoints; rails, deck, battery,
    controller pair and all unattributed wire/fastener reserves stay in place."""
    source = ROOT / "design/drivetrain/review/bom.csv"
    parent = read(ROOT / "winchless-bill/original.json")
    if sha(source) != parent["sourceSha256"]["design/drivetrain/review/bom.csv"]:
        raise ValueError("Unregistered baseline rows")
    with source.open(newline="") as f:
        rows = list(csv.DictReader(f))
    wheels = read(ROOT / "design/selected-shop/articles.json")["wheels"]
    old = {}
    for row in rows:
        i = row["id"]
        if row["kinematicFrame"] != "base" or not i.startswith(OLD_PREFIXES):
            continue
        old[i] = (
            wheels["accept_bare_kg"]
            if i.startswith("wheel-")
            else [float(row[k]) for k in ("minMassKg", "maxMassKg")]
        )
    return old


def calc(plugin, source, required_ids=None):
    installed = json.loads(
        subprocess.check_output(
            ["node", str(plugin / "scripts/assembly-accounting.mjs"), str(source)]
        )
    )
    items = set(
        installed["make"]
        + installed["undecided"]
        + [i for r in installed["buy"] for i in r["instances"]]
    )
    model = read(source)
    if len(items) != read(ROOT / "extension-support/mass.json")["installed"]:
        raise ValueError("Captured-rail parent required")
    old = baseline_removals()
    changes = {c["id"]: c for c in model["changes"] if c["id"] in model["selected"]}
    if required_ids is not None:
        # Incoming geometry must explicitly retire these still-installed old identifiers.
        missing = (set(old) & items) - required_ids
        if missing:
            raise ValueError("Booked articles still physically present: " + str(sorted(missing)))
    if not set(["drive-controller-L", "drive-controller-R", "motor-L", "motor-R"]) <= items:
        raise ValueError("Controller/motor reuse unresolved")
    b = np.array(installed["mass"]["representedRangeKg"])
    cap = installed["mass"]["limitKg"]
    base_retired = np.sum(list(old.values()), axis=0)
    extra = {}
    # Fully surrendered delta groups; keep mixed cover/rail and electronics groups.
    # The six earlier machined carrier and faceplate originals belong to these old rows.
    for key in ("light", "axle-retention"):
        c = changes[key]
        if not set(c["remove"]) <= set(old):
            raise ValueError("Group includes retained original stock")
        if required_ids is not None and not (set(c["add"]) & items) <= required_ids:
            continue
        extra[key] = c["delta"]["rangeKg"]
    retired = base_retired + np.sum(list(extra.values()), axis=0)
    # Four populated module receiving bands (±5% against catalog); six new
    # controllers with retained drive pair separately already in baseline.
    bought = {
        "fourCompletePopulatedModulesKg": [4 * 1.72 * 0.95, 4 * 1.72 * 1.05],
        "sixAdditionalSPARKMAXKg": [6 * 0.108, 6 * 0.120],
    }
    additions = np.sum(list(bought.values()), axis=0)
    bare = b - retired + additions
    return dict(
        plugin=read(plugin / "plugin.json")["version"],
        configuration=model["configuration"],
        sourceMapSha256=sha(source),
        inputRegisteredMassKg=b.tolist(),
        sourcePhysicalIdentities=len(items),
        inspectionLimitKg=cap,
        oldBaselineRows=old,
        baselineRetiredKg=base_retired.tolist(),
        surrenderedGroupsKg=extra,
        matchedRetiredKg=retired.tolist(),
        newReceivingBands=bought,
        projectedBareKg=bare.tolist(),
        remainingForAllMountsWireAndFinishKg=float(cap - bare[1]),
        noOldMassCreditUpperKg=float(b[1] + additions[1]),
        installedPhysicalMassKg=None,
        accountingStatus="Matched-endpoint design comparison; full revised physical manifest and installed scale mass remain open.",
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("plugin", type=Path)
    p.add_argument("map", type=Path)
    p.add_argument("out", type=Path)
    a = p.parse_args()
    write(a.out, calc(a.plugin, a.map))
