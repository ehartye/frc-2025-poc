"""Compare represented match mass and the union of the two declared role inventories."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE.parent / "signal-light"))
from accounting import build_map, locks


def write(path, data):
    path.write_bytes((json.dumps(data, indent=2) + "\n").encode())


def adjusted_mass(native, mode):
    baseline = locks.read(locks.HERE / "mass.json")[mode]["netDeltaKg"]
    removed = sum(native["critical_holes"][f"{mode}-{side}"]["removed_volume_mm3"] for side in "LR")
    if mode == "low":
        removed += sum(part["removed_mm3"] for part in native["rails"].values())
    nominal_kg = removed * 2.70e-6
    return [baseline[0] - nominal_kg * 1.05, baseline[1] - nominal_kg * .95]


def review_mode(plugin, folder, mode, color, native, budget):
    mapping = build_map(plugin, folder, mode, color)
    lock = next(change for change in mapping["changes"] if change["id"] == mode + "-lock")
    prefix = mode + "-lock/"
    def identity(name):
        return "shared-kit/" + name.removeprefix(prefix)
    for part in mapping["instances"]:
        if part["id"].startswith(prefix):
            part["id"] = identity(part["id"])
    lock["add"] = [identity(name) for name in lock["add"]]
    if mode == "low":
        for rail in native["rails"]:
            lock["remove"].append(rail)
            lock["add"].append(identity(rail))
            mapping["instances"].append({"id": identity(rail), "presence": "candidate", "disposition": "make"})
    for key, path in [("shared-material", HERE / "mass.json"), ("common-masts", HERE / "masts.step"),
                      ("shared-geometry", HERE / "native.json")]:
        mapping["sources"].append({"id": key, "file": Path(os.path.relpath(path, folder)).as_posix(),
                                   "sha256": locks.sha(path)})
    lock["delta"] = locks.evidence("shared-material", "/" + mode, budget[mode])
    path = folder / (mode + "-" + color + "-shared.json")
    locks.write_map(path, mapping)
    output = json.loads(subprocess.check_output(["node", str(plugin / "scripts/assembly-accounting.mjs"), str(path)]))
    parts = set(output["make"] + output["undecided"])
    for item in output["buy"]:
        parts.update(item["instances"])
    if len(parts) != output["representedInstances"] or output["coverage"]["missing"]:
        raise ValueError("Incomplete installed identity crosswalk")
    return parts, output["mass"]


def compare(plugin, folder):
    native = locks.read(HERE / "native.json")
    if native["mast_step_sha256"] != locks.sha(HERE / "masts.step"):
        raise ValueError("Common mast native file changed")
    for source, expected in native["sources"].items():
        if expected != locks.sha(ROOT / source):
            raise ValueError("Stale source: " + source)
    budget = {mode: adjusted_mass(native, mode) for mode in ("low", "high")}
    write(HERE / "mass.json", budget)
    pounds = locks.read(HERE.parent / "assembly-accounting/inspection-limit.json")["kilogramsPerPound"]
    base = locks.read(HERE.parent / "competition-package/shop-duplex/mass.json")["poses"]["stowed"]
    bumpers = [a-b for a, b in zip(base["R408_kg"], base["R103_kg"])]
    receipt = {"plugin": locks.read(plugin / "package.json")["version"], "bumperAllowanceKg": bumpers, "cases": {}}
    pins = {"shared-kit/" + key for key in native["high_only_pin_set_mm3"]}
    for color in ("red", "blue"):
        parts, cases = {}, {}
        for mode in ("high", "low"):
            parts[mode], mass = review_mode(plugin, folder, mode, color, native, budget)
            with_bumper = [a+b for a, b in zip(mass["representedRangeKg"], bumpers)]
            cases[mode] = {"physicalIds": len(parts[mode]), "R103Kg": mass["representedRangeKg"],
                           "R103UpperHeadroomKg": mass["representedMarginKg"][0], "R408Kg": with_bumper,
                           "R408UpperHeadroomKg": 135 * pounds - with_bumper[1],
                           "finishedMarginKg": mass["configurationMarginKg"]}
        if parts["low"] - parts["high"] or parts["high"] - parts["low"] != pins:
            raise ValueError("The declared match inventory difference has changed")
        cases["I103"] = {"uniqueRepresentedIds": len(parts["low"] | parts["high"]),
                         "presentedLooseInLow": sorted(pins),
                         "benchInventoryUpperKg": cases["high"]["R103Kg"][1],
                         "upperHeadroomKg": 150 * pounds - cases["high"]["R103Kg"][1],
                         "finishedMarginKg": None}
        # A dedicated pair at each height can preserve the original beam sections.
        # Start with the high single-height robot and add only the low pair of masts.
        original_high = locks.read(HERE.parent / "signal-light/accounting.json")["cases"]["high"]["representedRangeKg"][1]
        alternate = sum(native["critical_holes"][f"low-{side}"]["single_height_mast_volume_mm3"] for side in "LR")
        alternate_upper = alternate * 2.70e-6 * 1.05
        cases["dedicatedMastPairs"] = {"uniqueRepresentedIds": len(parts["high"]) + 2,
            "additionalLowPairUpperKg": alternate_upper, "benchInventoryUpperKg": original_high + alternate_upper,
            "upperHeadroomKg": 150 * pounds - original_high - alternate_upper, "finishedMarginKg": None}
        receipt["cases"][color] = cases
    return receipt


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plugin", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    write(HERE / "inspection.json", compare(args.plugin.resolve(), args.output.resolve()))
