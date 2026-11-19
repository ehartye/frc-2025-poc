"""Count the two locating shoes and two screws on the issued shop receiver map."""

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "frame-corners"))
from account import bind_sources, digest, read, write


def run(plugin, baseline, out):
    model = read(baseline)
    if model["selected"][-2:] != ["shop-relieved-receiver", "clearance-fit"]:
        raise ValueError("Expected the corrected shop receiver map")
    out.mkdir(parents=True, exist_ok=True)
    other = []
    for row in model["sources"]:
        path = (baseline.parent / row["file"]).resolve()
        if digest(path) != row["sha256"]:
            raise ValueError("Stale source " + row["id"])
        if row["id"] == "selected-shop-mass":
            for name, h in read(path)["source_sha256"].items():
                if digest(ROOT / name) != h:
                    raise ValueError("Stale shop component " + name)
            copy = out / "upstream-shop-mass.json"
            if path != copy.resolve():
                shutil.copyfile(path, copy)
            row["file"] = copy.name
        else:
            other.append(row)
    bind_sources(other, baseline, ROOT, out)
    old = "observer-slotted-cradle-weldment"
    ids = {r["id"] for r in model["instances"]}
    if old not in ids or "receiver-R6-ported" not in ids:
        raise ValueError("Fixed cradle or shelf missing")
    mount = read(HERE / "mount.json")
    if digest(HERE / "fixed-datum.step") != mount["stepSha256"]:
        raise ValueError("Changed machined camera mount")
    shoes = mount["addedShoesMm3"]
    delta = [
        2.65e-6 * shoes + mount["holderDeltaMm3"] * 8.1e-6,
        2.85e-6 * shoes + 2 * 0.0032 + 0.00025,
    ]
    article = "M4 x 18 socket head, fixed camera"
    source = HERE / "mass-input.json"
    write(
        source,
        {
            "stepSha256": mount["stepSha256"],
            "shoeVolumeMm3": shoes,
            "removedCradleVolumeMm3": -mount["holderDeltaMm3"],
            "receivingMaxKgEachScrew": 0.0032,
            "finishUpperKg": 0.00025,
            "retainedMachining": [old, "receiver-R6-ported"],
            "budgetDeltaKg": delta,
            "physicalWeighing": None,
        },
    )
    model["sources"].append(
        {
            "id": "selected-optics-mass",
            "file": os.path.relpath(source, out).replace(os.sep, "/"),
            "sha256": digest(source),
        }
    )
    model["articles"].append(
        {"id": article, "sku": None, "unitsPerPack": None, "onHand": None}
    )
    names = []
    for key in mount["reopenedParts"]:
        if key.endswith("drilled"):
            continue
        if key in ids:
            raise ValueError("Optical hardware already included")
        inst = {"id": key, "presence": "candidate", "disposition": "make"}
        if "-M4-" in key:
            inst.update(disposition="buy", article=article)
        model["instances"].append(inst)
        names.append(key)
    model["changes"].append(
        {
            "id": "selected-optics",
            "revision": model["revision"],
            "configuration": model["configuration"],
            "add": names,
            "remove": [],
            "delta": {
                "source": "selected-optics-mass",
                "basis": "estimated",
                "pointer": "/budgetDeltaKg",
                "rangeKg": delta,
            },
        }
    )
    model["selected"].append("selected-optics")
    path = out / "registered-optics.json"
    write(path, model)
    accounting = json.loads(
        subprocess.check_output(
            ["node", str(plugin / "scripts/assembly-accounting.mjs"), str(path)]
        )
    )
    coverage = {
        k: accounting["coverage"][k]
        for k in ("missing", "duplicates", "orphanIncluded")
    }
    if any(coverage.values()):
        raise ValueError(coverage)
    mass = accounting["mass"]
    write(
        HERE / "mass.json",
        {
            "installedVersion": read(plugin / "package.json")["version"],
            "addedInstalled": names,
            "coverage": coverage,
            "representedKg": mass["representedRangeKg"],
            "representedMarginKg": mass["representedMarginKg"],
            "physicalMarginKg": mass["configurationMarginKg"],
        },
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for k in ("plugin", "baseline", "out"):
        p.add_argument(k, type=Path)
    a = p.parse_args()
    run(a.plugin.resolve(), a.baseline.resolve(), a.out.resolve())
