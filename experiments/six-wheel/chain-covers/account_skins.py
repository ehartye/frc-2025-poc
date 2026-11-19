"""Run the installed assembly accountant on both sheet gauges in the registered camera robot."""
import argparse
import copy
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "frame-corners"))
from account import bind_sources, digest, read, write


def run(plugin, baseline, out):
    out.mkdir(parents=True, exist_ok=True)
    original = read(baseline)
    if not {"selected-optics", "sp1-install"}.issubset(original["selected"]):
        raise ValueError("The fixed-camera and shelf-sensor revisions must both be installed")
    portable = []
    for source in original["sources"]:
        if source["id"] != "selected-shop-mass":
            portable.append(source)
            continue
        incoming = (baseline.parent / source["file"]).resolve()
        if digest(incoming) != source["sha256"]:
            raise ValueError("Incorrect upstream mass stock")
        for relative, expected in read(incoming)["source_sha256"].items():
            if digest(ROOT / relative) != expected:
                raise ValueError("Upstream source changed: " + relative)
        dest = out / "upstream-shop-mass.json"
        dest.write_bytes(incoming.read_bytes())
        source["file"] = dest.name
    bind_sources(portable, baseline, ROOT, out)
    study = read(HERE / "study.json")
    result = {}
    for name in ("repacked-light", "repacked-thick"):
        trial = study["cases"][name]
        if digest(HERE / f"{name}.step") != trial["stepSha256"]:
            raise ValueError("Cover solids have changed")
        if digest(HERE / "front-loop-repack.step") != study["repack"]["stepSha256"]:
            raise ValueError("Spacer exchange has changed")
        data = copy.deepcopy(original)
        # Same chain and sprocket articles are repositioned. Replace the eight
        # sleeves that change length explicitly. Equal net volume assumes matched stock.
        removed = sorted(k for k in study["repack"]["volumeChangesMm3"] if k.startswith("spacer-"))
        added = [name + "/" + k for k in removed + sorted(trial["volumeMm3"])]
        data["instances"].extend({"id": k, "presence": "candidate", "disposition": "make"} for k in added)
        # This ±5% window is a proposed receiving condition on total skin mass.
        # No manufacturer tolerance or physical measurement is implied.
        weight = trial["sheetGrams"] / 1000
        interval = [0.95*weight, 1.05*weight]
        evidence = out / (name + "-material.json")
        write(evidence, {"rangeKg": interval, "skinStepSha256": trial["stepSha256"],
                         "spacerStepSha256": study["repack"]["stepSha256"]})
        data["sources"].append({"id": name, "file": evidence.name, "sha256": digest(evidence)})
        data["changes"].append({"id": name, "revision": data["revision"], "configuration": data["configuration"],
                                "remove": removed, "add": added, "delta": {"source": name, "basis": "estimated",
                                "pointer": "/rangeKg", "rangeKg": interval}})
        data["selected"].append(name)
        data["omissions"].append("Chain cover attachment, closure and joining material have no issued bill.")
        target = out / (name + ".json")
        write(target, data)
        report = json.loads(subprocess.check_output(["node", str(plugin / "scripts/assembly-accounting.mjs"), str(target)]))
        actual = set(report["make"])
        if not set(added) <= actual or set(removed) & actual:
            raise ValueError("Revised skin and sleeve identities do not reconcile")
        result[name] = {"installedVersion": read(plugin / "package.json")["version"],
                        "replacedSleeves": len(removed), "skins": len(trial["volumeMm3"]),
                        "conditionalDeltaKg": interval, "representedKg": report["mass"]["representedRangeKg"],
                        "representedMarginKg": report["mass"]["representedMarginKg"],
                        "physicalMarginKg": report["mass"]["configurationMarginKg"]}
    write(HERE / "accounting.json", result)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plugin", type=Path)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    run(args.plugin.resolve(), args.baseline.resolve(), args.output.resolve())
