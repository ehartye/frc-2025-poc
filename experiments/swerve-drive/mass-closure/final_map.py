"""Reprice the final physical map from original rows with correlated removal endpoints."""

import json
import subprocess

from surrendered_stock import read, same, sha


def materialize(project, plugin, rear_file, after, construction, receipt):
    source = read(rear_file)
    extra = read(project / "experiments/swerve-drive/power/mass.json")
    prior = read(project / "experiments/swerve-drive/power/installed.json")["matched"]
    original = {
        s["id"]: read(rear_file.parent / s["file"])
        for s in source["sources"]
        if s["id"] in ("primary-hoist-rebill", "shelf-insert-stock")
    }
    issued = original["primary-hoist-rebill"]["baselineKg"]["issuedDrawingKg"]
    # First remove all retired original wheel rows from BOTH endpoints of the
    # same historical baseline, then retire its separate outer-shop clamp charge.
    baseline = [
        x - y - construction["originalShopOuterClampOnlyKg"]
        for x, y in zip(issued, extra["baselineRetiredKg"])
    ]
    changes = {c["id"]: c for c in source["changes"] if c["id"] in source["selected"]}
    accepted = read(
        project / "experiments/six-wheel/design/selected-shop/articles.json"
    )["hub_screws"]["accept_each_kg"]
    screws = [36 * v for v in accepted]
    if (
        sum(
            k.startswith("unmodeled-wheel-to-hub-screw-")
            for k in changes["four-module-swerve"]["remove"]
        )
        != 36
    ):
        raise ValueError("Partial hub screw retirement")
    contribution = [
        sum(r[i] for r in extra["newReceivingBands"].values())
        + prior["mountHardwareAndWeldKg"][i]
        + prior["fourControllerTraysKg"][i]
        + prior["trayFastenersAndControllerStrapsKg"][i]
        - prior["matchedCutMassKg"][1 - i]
        for i in range(2)
    ]
    contribution[1] += receipt["cases"]["drawingSelected"]["wireUpperKg"]
    revisions = {
        "light": [0, 0],
        "axle-retention": [0, 0],
        "supported-drive-covers": [0, 0],
        "shop-supplies": [
            x - y for x, y in zip(changes["shop-supplies"]["delta"]["rangeKg"], screws)
        ],
        "four-module-swerve": contribution,
    }
    # Original interval subtractions must agree with the documented group
    # retirements for this specific complete installation.
    for i in range(2):
        same(
            sum(changes[k]["delta"]["rangeKg"][i] for k in ("light", "axle-retention"))
            + extra["baselineRetiredKg"][i],
            prior["retiredBaselineAndTwoGroupsKg"][i],
        )
        same(
            changes["supported-drive-covers"]["delta"]["rangeKg"][i] + screws[i],
            prior["retiredHubScrewsAndCoversKg"][i],
        )
    out = rear_file.parent
    row = {
        "basis": "complete selected four-module fixed-height build only",
        "activeSelection": source["selected"],
        "baselineAfterNamedRetirementsKg": baseline,
        "revisedChangesKg": revisions,
    }

    def write(path, data):
        path.write_text(
            json.dumps(data, indent=2) + "\n", encoding="utf-8", newline="\n"
        )

    ledger = out / "matched-budget-rows.json"
    write(ledger, row)
    source["sources"].append(
        dict(id="fixed-height-swerve-row-rebill", file=ledger.name, sha256=sha(ledger))
    )

    def cost(pointer, value):
        return dict(
            source="fixed-height-swerve-row-rebill",
            pointer=pointer,
            rangeKg=value,
            basis="estimated",
        )

    source["mass"]["baseline"] = cost("/baselineAfterNamedRetirementsKg", baseline)
    for key, value in revisions.items():
        changes[key]["delta"] = cost("/revisedChangesKg/" + key, value)
    changes["underside-shelf-insert"]["delta"] = dict(
        source="shelf-insert-stock",
        pointer="/densityMatchedRemovalKg",
        basis="estimated",
        rangeKg=original["shelf-insert-stock"]["densityMatchedRemovalKg"],
    )
    complete = out / "selected.json"
    write(complete, source)
    result = json.loads(
        subprocess.check_output(
            ["node", str(plugin / "scripts/assembly-accounting.mjs"), str(complete)]
        )
    )
    alive = set(
        result["make"]
        + result["undecided"]
        + [x for r in result["buy"] for x in r["instances"]]
    )
    coverage = {
        k: result["coverage"][k]
        for k in ("missing", "duplicates", "orphanIncluded", "unattached")
    }
    if (
        alive != after
        or any(coverage.values())
        or result["mass"]["configurationMarginKg"] is not None
    ):
        raise ValueError(
            "Different final inventory or loss of estimated-only qualification"
        )
    same(
        result["mass"]["representedRangeKg"][1],
        receipt["joinedCases"]["drawingSelected"]["r103UpperKg"],
    )
    return dict(
        representedRangeKg=result["mass"]["representedRangeKg"],
        coverage=coverage,
        installedCount=len(alive),
        ruleLimitKg=result["mass"]["limitKg"],
        estimatedMarginKg=result["mass"]["representedMarginKg"],
        physicalMarginKg=result["mass"]["configurationMarginKg"],
        frozenSelectedChanges=source["selected"],
    )
