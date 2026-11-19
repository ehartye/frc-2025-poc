"""Exchange the registered skid-drive articles for the new physical swerve inventory."""

import argparse
import os
from pathlib import Path
import subprocess
import numpy as np
from electrical import ROOT, HERE, IDS, PACKAGE
from mass import read, calc
from harness import kg_per_m
from screen import sha, write


def inventory(plugin, src, out, receipt):
    out.mkdir(parents=True, exist_ok=True)
    package = PACKAGE
    retirement = read(package / "retirement.json")
    frame = read(package / "frame.json")
    fit = read(HERE / "controller-fit.json")
    if (
        sha(package / "swerve-frame.step") != frame["stepSha256"]
        or sha(HERE / "controller-trays.step") != fit["stepSha256"]
    ):
        raise ValueError("Unregistered structure")
    source = read(src)

    def account(file):
        import json

        return json.loads(
            subprocess.check_output(
                ["node", str(plugin / "scripts/assembly-accounting.mjs"), str(file)]
            )
        )

    def roster(d):
        return set(d["make"] + d["undecided"] + [i for r in d["buy"] for i in r["instances"]])

    before = roster(account(src))
    removed = set(retirement["retired"])
    reuse = set(retirement["sameDriveMotorsAndControllers"])
    if len(before) != 1199 or not removed <= before or removed & reuse or len(reuse) != 4:
        raise ValueError("Invalid matched surrender")
    bare = calc(plugin, src, removed | reuse)
    condition = read(ROOT / "cover-retention/accounting.json")
    groups = np.sum(list(condition["groupsKg"].values()), axis=0)
    cr = next(c for c in source["changes"] if c["id"] == "supported-drive-covers")
    if not (set(cr["add"]) & before) <= removed or not np.allclose(groups, cr["delta"]["rangeKg"]):
        raise ValueError("Mixed cover group cannot retire")
    screws = read(ROOT / "design/selected-shop/articles.json")["hub_screws"]["accept_each_kg"]
    if len([k for k in removed if k.startswith("unmodeled-wheel-to-hub-screw-")]) != 36:
        raise ValueError("Incomplete hub groups")
    extra_retired = groups + 36 * np.array(screws)
    stock = frame["stockChange"]
    if any(
        (k.endswith("RemovedMm3") and v < 0) or (k.endswith("ChangedMm3") and v > 0)
        for k, v in stock.items()
    ):
        raise ValueError("Changed structural substitutions")
    # As-built metal removed from matched retained stock (higher/lower density
    # endpoints use reversed order for a negative interval). No hardware chip credit.
    volume = sum(v if k.endswith("RemovedMm3") else -v for k, v in stock.items())
    credit = np.array([volume * 2.85e-6, volume * 2.60e-6])
    hardware = {k: v["volumeMm3"] for k, v in frame["parts"].items() if k.startswith(IDS)}
    alu = sum(v for k, v in hardware.items() if "saddle" in k or "welded-thread-boss" in k)
    steel = sum(hardware.values()) - alu
    metal = np.array([alu * 2.60e-6 + steel * 7.2e-6, alu * 2.85e-6 + steel * 8.10e-6 + 0.040])
    source_budget = read(package / "mount-budget.json")
    if source_budget["sourceFrameSha256"] != frame["stepSha256"]:
        raise ValueError("Mount budget belongs to a different manufactured frame")
    issued = source_budget["mass"]
    if not np.isclose(issued["newKitOnlyKg"], alu * 2.7e-6 + steel * 7.85e-6, rtol=0, atol=1e-8):
        raise ValueError("Registered positive metal no longer matches the fixture")
    metal[1] = issued["zeroChipCreditUpperKg"]
    credit[1] = metal[1] - issued["matchedStockUpperKg"]
    trays = sum(v["volumeMm3"] for v in fit["corners"].values())
    plates = np.array([trays * 2.60e-6, trays * 2.85e-6 + 0.010])
    # Eight blind inserts, screws and collars, sixteen controller straps and
    # fastener witness paint are individually identified; accept the assembled lot.
    fast = np.array([0.025, 0.090])
    wire = read(HERE / "harness.json")["totalNoWireRemovalCreditUpperKg"]
    paths = read(HERE / "paths.json")
    length = 0.0
    for r in paths["groupedSupplyTargets"].values():
        if "lengthMm" not in r or r["lowerGapMm"] <= 0:
            raise ValueError("Unqualified supply span")
        # Provide 0.42m TOTAL for incoming PDH termination, service loop and
        # twin-controller split beyond each qualified common centerline.
        length += r["lengthMm"] / 1000 + 0.42
    supply_new = length * (2 * kg_per_m(12) + 2 * kg_per_m(14))
    # Each local motor/sensor handoff must fit the reserved purchased pigtails
    # and ≤0.15m additional 3-conductor extension per motor.
    local = 4 * 3 * 0.15 * (kg_per_m(12) + kg_per_m(16))
    target_wire = supply_new + local + 2 * 3.2 * kg_per_m(22) + 0.46
    bare_range = np.array(bare["projectedBareKg"])
    old = np.array(bare["matchedRetiredKg"]) + extra_retired
    upper_dressed = (
        bare_range - extra_retired - credit + metal + plates + fast + np.array([0.0, wire])
    )
    shorter = upper_dressed.copy()
    shorter[1] += target_wire - wire
    added = (
        np.sum(list(bare["newReceivingBands"].values()), axis=0)
        + metal
        + plates
        + fast
        + np.array([0.0, wire])
    )
    cap = bare["inspectionLimitKg"]
    totals = dict(
        startKg=bare["inputRegisteredMassKg"],
        bareKg=bare_range.tolist(),
        retiredBaselineAndTwoGroupsKg=bare["matchedRetiredKg"],
        retiredHubScrewsAndCoversKg=extra_retired.tolist(),
        matchedCutVolumeMm3=volume,
        matchedCutMassKg=sorted(credit.tolist()),
        mountHardwareAndWeldKg=metal.tolist(),
        fourControllerTraysKg=plates.tolist(),
        trayFastenersAndControllerStrapsKg=fast.tolist(),
        longWireNoOldCreditUpperKg=wire,
        shorterEndpointCappedWireUpperKg=target_wire,
        shortMaxSupplyLoopLengthM=max(
            r["lengthMm"] / 1000 + 0.42 for r in paths["groupedSupplyTargets"].values()
        ),
        conservativeDressedRangeKg=upper_dressed.tolist(),
        shorterDressedUpperKg=float(shorter[1]),
        longR103MarginKg=float(cap - upper_dressed[1]),
        shortR103MarginKg=float(cap - shorter[1]),
        shortFieldEmptyUpperKg=float(shorter[1] + 6.17 + 7.50),
        longFieldEmptyUpperKg=float(upper_dressed[1] + 6.17 + 7.50),
        longFieldOnePipeUpperKg=float(upper_dressed[1] + 6.17 + 7.50 + 0.816466266),
        shortMaximumRemainderKgForR103=float(5 + cap - shorter[1]),
        currentRemainderRangeKg=[2.0, 5.0],
        longR408UpperKg=float(upper_dressed[1] + 7.50),
        r408LimitKg=135 * 0.45359237,
        noOldStockCreditUpperKg=float(bare["inputRegisteredMassKg"][1] + added[1]),
        physicalRobotKg=None,
    )
    # Independently varying interval for the portable assembly application. The
    # equal-endpoint accounting above separately preserves the original budget rows.
    change = np.array([added[0] - old[1] - credit[0], added[1] - old[0] - credit[1]])
    evidence = out / "exchange.json"
    write(evidence, dict(deltaKg=change.tolist(), matched=totals))
    for s in source["sources"]:
        oldfile = (src.parent / s["file"]).resolve()
        if sha(oldfile) != s["sha256"]:
            raise ValueError("Changed input source " + s["id"])
        s["file"] = os.path.relpath(oldfile, out).replace(os.sep, "/")
    source["sources"].append(dict(id="swerve-rebill", file=evidence.name, sha256=sha(evidence)))
    remove = removed | {
        "mast-base-L-rail",
        "mast-base-R-rail",
        "mast-slotted-ribbed-deck",
        "drive-rail-outer-L",
        "drive-rail-outer-R",
        "tie-machined-cross-front",
        "tie-machined-cross-rear",
    }
    make = ["swerve/" + k for k in frame["parts"]] + ["swerve/tray-" + i for i in IDS]
    buy = {}
    for id in IDS:
        for key, sku in (
            ("module-core", "REV-21-3005"),
            ("steer-550", "REV-21-1651"),
            ("steer-controller", "REV-11-2158"),
        ):
            buy["swerve/" + id + "-" + key] = sku
        if id.startswith("R"):
            buy["swerve/" + id + "-drive-NEO"] = "REV-21-1650"
            buy["swerve/" + id + "-drive-controller"] = "REV-11-2158"
        for i in (1, 2):
            for key in (
                "blind-insert",
                "tray-screw",
                "controller-restraint-a",
                "controller-restraint-b",
            ):
                make.append(f"swerve/{id}-{key}-{i}")
        make.append("swerve/" + id + "-wiring-set")
    # Fasteners and harnesses without source-pinned commercial articles are
    # counted as unresolved procurement rather than assigned a fictional SKU.
    arts = {row["sku"]: row["id"] for row in source["articles"] if row["sku"]}
    for sku in sorted(set(buy.values()) - arts.keys()):
        arts[sku] = "swerve/" + sku
        source["articles"].append(dict(id=arts[sku], sku=sku, unitsPerPack=1, onHand=None))
    for i in make:
        unresolved = any(
            k in i
            for k in (
                "blind-insert",
                "tray-screw",
                "controller-restraint",
                "wiring-set",
                "tab-screw",
                "rail-screw",
                "tab-locknut",
            )
        )
        source["instances"].append(
            dict(id=i, presence="candidate", disposition="undecided" if unresolved else "make")
        )
    for i, sku in buy.items():
        source["instances"].append(
            dict(id=i, presence="candidate", disposition="buy", article=arts[sku])
        )
    added_ids = set(make) | set(buy)
    if len(added_ids) != len(make) + len(buy) or not remove <= before or before & added_ids:
        raise ValueError("Unmatched physical exchange")
    source["changes"].append(
        dict(
            id="four-module-swerve",
            configuration=source["configuration"],
            revision=source["revision"],
            remove=sorted(remove),
            add=sorted(added_ids),
            delta=dict(
                source="swerve-rebill",
                pointer="/deltaKg",
                basis="estimated",
                rangeKg=change.tolist(),
            ),
        )
    )
    source["selected"].append("four-module-swerve")
    dest = out / "selected.json"
    write(dest, source)
    after = account(dest)
    if roster(after) != (before - remove) | added_ids:
        raise ValueError("Wrong selected physical roster")
    write(
        receipt,
        dict(
            plugin=bare["plugin"],
            sourceMapSha256=sha(src),
            sourcePhysicalIdentities=len(before),
            pricedModuleCoresExcludeSeparatelyIdentifiedMotors=True,
            reusedPhysicalItems=sorted(reuse),
            removedPhysicalCount=len(remove),
            newPhysicalCount=len(added_ids),
            installedPhysicalCount=len(roster(after)),
            sourceFiles={
                p.name: sha(p)
                for p in (
                    package / "retirement.json",
                    package / "frame.json",
                    package / "mount-budget.json",
                    HERE / "controller-fit.json",
                    HERE / "paths.json",
                    HERE / "harness.json",
                )
            },
            independentIntervalFromAssemblyTool=after["mass"]["representedRangeKg"],
            matched=totals,
            physicalQualification=None,
        ),
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for k in ("plugin", "source", "output", "receipt"):
        p.add_argument(k, type=Path)
    a = p.parse_args()
    inventory(a.plugin, a.source, a.output, a.receipt)
