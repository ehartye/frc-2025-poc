"""Reconcile the complete retired primary lift group on the fixed low shop build."""
import argparse
from copy import deepcopy
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "fixed-hoist"))
from account import identities, installed
from fit import j, routing

GROUP = "lift drive, ratchet, drums, lines, hardware and release"
CAP = {"winch-input-cap", "motor-cap-screw-1", "motor-cap-screw-2"}
REQUIRED = {"low-lock", "no-match-air", "mast-foot-splices", "fixed-secondary-haul"}


def exchange(model, selected):
    changes = {r["id"]: r for r in model["changes"] if r["id"] in model["selected"]}
    if not REQUIRED <= changes.keys() or changes.get("fixed-lift-motor-omitted", {}).get(
            "delta", {}).get("pointer") != "/restrictedNoMotorCreditKg":
        raise ValueError("Require the permanent low locks, mast splices and zero-credit motor omission")
    previous = changes["fixed-secondary-haul"]
    if "long-common-shaft" not in previous["remove"] or "fixed-haul-shaft" not in previous["add"]:
        raise ValueError("Unrecognized shaft replacement")
    assigned = {r["occurrence"]: r["instance"] for r in model["assignments"]
                if r["export"] == "place" and r.get("instance")}
    original = set(routing(560))
    duplex = j.read(ROOT / "design/receiver-lift/duplex-inventory.json")
    predecessor = "common-shaft-round-envelope"
    if predecessor not in duplex["replaces"] or "long-common-shaft" not in duplex["poses"]["place"]:
        raise ValueError("Unrecognized original shaft conversion")
    mapped = {n: assigned[n] for n in original - {predecessor}}
    mapped[predecessor] = "fixed-haul-shaft"
    absent = {name for name, identity in mapped.items() if identity not in selected}
    if absent != {"motor-body"}:
        raise ValueError(f"Unexpected prior removals from the grouped allowance: {absent}")
    existing = set(mapped.values()) & selected
    if {k for k, v in assigned.items() if v in existing} - original:
        raise ValueError("An original winch article still serves a separate installed occurrence")
    remove = existing | set(previous["add"]) | CAP
    if not remove <= selected:
        raise ValueError("Some winch accessories are no longer installed")
    keep = {"lift-branch-insulation", "spark-lift"}
    for side in ("L", "R"):
        keep.update({f"backup-{side}-carriage-plate", f"backup-{side}-service-pin-rail",
                     f"backup-{side}-service-pin-bushing-560", f"low-lock/height-{side}-pin"})
    if not keep <= selected - remove:
        raise ValueError(f"Lost fixed-role support or electrical equipment: {keep-(selected-remove)}")
    return mapped, remove, keep


def run(plugin, source, out):
    model = j.read(source)
    current = installed(plugin, source)
    present = identities(current)
    mapped, remove, keep = exchange(model, present)
    out.mkdir(parents=True, exist_ok=True)
    pins = {}
    for item in model["sources"]:
        path = (source.parent / item["file"]).resolve()
        if j.digest(path) != item["sha256"]:
            raise ValueError(f"Changed predecessor {item['id']}")
        pins[item["id"]] = path
        item["file"] = os.path.relpath(path, out).replace(os.sep, "/")
    inspection = ROOT / "design/electrical-inspection/inspection.json"
    groups = j.read(inspection)["projectMassAllowancesKg"]
    charge = groups[GROUP]
    shop_bill = ROOT / "design/competition-package/shop-duplex/mass.json"
    if any(j.digest(ROOT / f) != h for f, h in j.read(shop_bill)["sourceSha256"].items()):
        raise ValueError("The original mass allocation has changed")
    if (charge != [1.8, 3.9] or j.read(pins["motor-omission"])["sources"][
            "design/electrical-inspection/inspection.json"] != j.digest(inspection)):
        raise ValueError("Changed group allowance or motor basis")
    evidence = {"group": GROUP, "bookedKg": charge, "originalOccurrences": dict(sorted(mapped.items())),
                "removedIdentities": sorted(remove), "requiredRetained": sorted(keep),
                "completeGroupDeltaKg": [-charge[1], -charge[0]], "unknownGroupDeltaKg": [0., 0.],
                "capAndSecondaryKitExtraCreditKg": 0., "surrenderedMeasuredKg": None,
                "sourceSha256": {str(f.relative_to(ROOT).as_posix()): j.digest(f) for f in (
                    inspection, shop_bill, ROOT / "design/stability/stability.py",
                    ROOT / "design/competition-package/build.py", ROOT / "design/receiver-lift/check_routing.py")}}
    file = out / "primary-group.json"
    j.write(file, evidence)
    model["sources"].append({"id": "whole-primary-winch", "file": file.name, "sha256": j.digest(file)})
    cases = {}
    for key in ("completeGroupDeltaKg", "unknownGroupDeltaKg"):
        proposed = deepcopy(model)
        proposed["changes"].append({"id": "winchless", "configuration": model["configuration"],
            "revision": model["revision"], "remove": sorted(remove), "add": [],
            "delta": {"source": "whole-primary-winch", "basis": "estimated", "pointer": "/" + key,
                      "rangeKg": evidence[key]}})
        proposed["selected"].append("winchless")
        target = out / (key + ".json")
        j.write(target, proposed)
        checked = installed(plugin, target)
        if identities(checked) != present - remove or any(checked["coverage"][k]
                for k in ("missing", "duplicates", "unattached", "orphanIncluded")):
            raise ValueError("Unrelated identity changed or coverage lost")
        cases[key] = {"upperMarginKg": checked["mass"]["representedMarginKg"][0],
                      "installedCount": checked["representedInstances"],
                      "physicalMarginKg": checked["mass"]["configurationMarginKg"]}
    return {"sourceSha256": j.digest(source), "evidence": evidence,
            "priorUpperMarginKg": current["mass"]["representedMarginKg"][0], "cases": cases}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("plugin", "shelf_maps", "output"):
        parser.add_argument(name, type=lambda s: Path(s).resolve())
    parser.add_argument("--carriage-support", action="store_true",
                        help="Use the two selected maps with the direct carriage supports installed")
    args = parser.parse_args()
    def incoming(shelf):
        return (args.shelf_maps / shelf / "selected.json" if args.carriage_support
                else args.shelf_maps / (shelf + ".json"))
    results = {s: run(args.plugin, incoming(s), args.output / s)
               for s in ("densityMatchedRemovalKg", "noCutCreditKg")}
    j.write(HERE / ("supported-budget.json" if args.carriage_support else "budget.json"), {"plugin": j.read(args.plugin / "package.json")["version"], "shelfCases": results})
