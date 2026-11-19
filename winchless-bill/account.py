"""Apply a direct source-row replacement to a verified fixed-guide accounting map."""

import argparse, copy, json, math, os, subprocess
from pathlib import Path
from stock import HERE, ROOT, sha


def read(p):
    return json.loads(p.read_bytes())


def write(p, v):
    p.write_text(
        json.dumps(v, indent=2, allow_nan=False) + "\n", encoding="utf8", newline="\n"
    )


def installed(plugin, p):
    return json.loads(
        subprocess.check_output(
            ["node", str(plugin / "scripts/assembly-accounting.mjs"), str(p)]
        )
    )


def roster(r):
    return (
        set(r["make"])
        | set(r["undecided"])
        | {i for row in r["buy"] for i in row["instances"]}
    )


def close(a, b):
    return len(a) == len(b) and all(
        math.isclose(x, y, rel_tol=0.0, abs_tol=1e-8) for x, y in zip(a, b)
    )


def pins(model, src, dest):
    files = {}
    for s in model["sources"]:
        path = (src.parent / s["file"]).resolve()
        if sha(path) != s["sha256"]:
            raise ValueError("Source changed: " + s["id"])
        files[s["id"]] = path
        s["file"] = os.path.relpath(path, dest).replace(os.sep, "/")
    return files


def validate_choice(model, source, stock, baseline, present):
    active = {d["id"]: d for d in model["changes"] if d["id"] in model["selected"]}
    needed = {
        "winchless",
        "fixed-lift-motor-omitted",
        "fixed-secondary-haul",
        "supported-guides",
        "fixed-carriage-supports",
        "match-drilled-guide",
        "low-lock",
        "no-match-air",
    }
    if not needed <= active.keys():
        raise ValueError("Missing fixed-height guide selection")
    if active["winchless"]["delta"]["pointer"] != "/unknownGroupDeltaKg" or active[
        "winchless"
    ]["delta"]["rangeKg"] != [0, 0]:
        raise ValueError(
            "Require the zero-credit winchless input for same-row accounting"
        )
    if (
        active["fixed-lift-motor-omitted"]["delta"]["pointer"]
        != "/restrictedNoMotorCreditKg"
    ):
        raise ValueError("A separate motor credit would duplicate this exchange")
    b = model["mass"]["baseline"]
    if (
        b["source"] != "selected-shop-mass"
        or b["pointer"] != "/wheel_replaced_baseline_kg"
        or not close(b["rangeKg"], baseline["summedBaselineKg"])
    ):
        raise ValueError("Different source baseline")
    if not close(source["bookedKg"], baseline["originalGroupKg"]):
        raise ValueError("Old group changed")
    if set(source["originalOccurrences"]) != set(stock["originalRoutingIds"]):
        raise ValueError("Routing catalog mismatch")
    if set(source["removedIdentities"]) != set(active["winchless"]["remove"]):
        raise ValueError("Original omissions no longer agree")
    removed = set(source["removedIdentities"]) | set(
        source["originalOccurrences"].values()
    )
    if overlap := removed & present:
        raise ValueError(f"Old mechanism present: {overlap}")
    keep = (
        set(source["requiredRetained"])
        | set(stock["machinedParentIds"])
        | {"observer-camera-proxy"}
    )
    fixed_camera = next(
        (r for r in model["instances"] if r["id"] == "observer-camera-proxy"), {}
    )
    if fixed_camera.get("article") != "OAK-D Lite fixed focus":
        raise ValueError("Production camera/gyro basis changed")
    if missing := keep - present:
        raise ValueError(f"Retained joint or shared equipment missing: {missing}")
    return keep


def apply(plugin, src, dest):
    dest.mkdir(parents=True, exist_ok=True)
    model = read(src)
    prior = installed(plugin, src)
    present = roster(prior)
    stock = read(HERE / "stock.json")
    basis = read(HERE / "original.json")
    for r in (stock["sources"], basis["sourceSha256"]):
        for n, h in r.items():
            if sha(ROOT / n) != h:
                raise ValueError("Changed project source: " + n)
    files = pins(model, src, dest)
    group = read(files["whole-primary-winch"])
    original_groups = read(ROOT / "winchless/supported-budget.json")["shelfCases"]
    if not any(group == case["evidence"] for case in original_groups.values()):
        raise ValueError(
            "Unrecognized original winch component assignments or bookkeeping"
        )
    shop = read(files["selected-shop-mass"])
    if not close(shop["wheel_replaced_baseline_kg"], basis["summedBaselineKg"]):
        raise ValueError("Selected wheel basis mismatch")
    keep = validate_choice(model, group, stock, basis, present)
    controls = {}
    for label, mistaken in (
        ("old drive still installed", present | {"sport-ratchet-and-mount-reserve"}),
        ("lost retained controller", present - {"spark-lift"}),
        ("missing fixed camera", present - {"observer-camera-proxy"}),
    ):
        try:
            validate_choice(model, group, stock, basis, mistaken)
        except ValueError as error:
            controls[label] = str(error)
        else:
            raise ValueError("Failed roster control: " + label)
    for change, pointer in (
        ("winchless", "/completeGroupDeltaKg"),
        ("fixed-lift-motor-omitted", "/conditionalMatchedArticleDeltaKg"),
    ):
        bad = copy.deepcopy(model)
        item = next(r for r in bad["changes"] if r["id"] == change)
        item["delta"]["pointer"] = pointer
        try:
            validate_choice(bad, group, stock, basis, present)
        except ValueError as error:
            controls["prior credit: " + change] = str(error)
        else:
            raise ValueError("Failed duplicate-credit control: " + change)
    new = basis["baselineWithoutPrimaryLiftKg"]
    # Input total already contains each original endpoint. Replace that identical
    # entry with the independently resummed non-winch baseline just once.
    scenarios = {
        "issuedDrawingKg": 0.0,
        "allSevenStocksKg": stock["stockUpperKg"],
        "allStocksAndFourteenSetsKg": stock["stockAndFastenerUpperKg"],
        "opaqueReserveAlsoPresentKg": stock["stockAndFastenerUpperKg"]
        + stock["opaqueSportSolidBoxSensitivityKg"],
    }
    evidence = {
        "inputMap": sha(src),
        "stockSource": sha(HERE / "stock.json"),
        "originalSource": sha(HERE / "original.json"),
        "baselineKg": {k: [new[0], new[1] + v] for k, v in scenarios.items()},
        "addedRetainedHardwareIsMeasured": False,
    }
    bill = dest / "residual-bill.json"
    write(bill, evidence)
    model["sources"].append(
        {"id": "primary-hoist-rebill", "file": bill.name, "sha256": sha(bill)}
    )
    result = {}
    for key, rng in evidence["baselineKg"].items():
        d = copy.deepcopy(model)
        d["mass"]["baseline"] = {
            "source": "primary-hoist-rebill",
            "basis": "estimated",
            "pointer": "/baselineKg/" + key,
            "rangeKg": rng,
        }
        d["mass"]["omissions"].append(
            "Residual hardware limits and installed mass are modeled. Unexpected separate hardware needs identity and collision review."
        )
        path = dest / (key + ".json")
        write(path, d)
        checked = installed(plugin, path)
        if roster(checked) != present or any(
            checked["coverage"][k]
            for k in ("missing", "duplicates", "unattached", "orphanIncluded")
        ):
            raise ValueError(
                "Identity or coverage changed during mass-only replacement"
            )
        mass = checked["mass"]
        old = prior["mass"]["representedRangeKg"]
        if not close(
            mass["representedRangeKg"], [old[0] - 1.8, old[1] - 3.9 + scenarios[key]]
        ):
            raise ValueError("Installed result disagrees with source sum")
        if mass["configurationMarginKg"] is not None:
            raise ValueError("Physical scale evidence is missing")
        result[key] = {
            "estimatedR103Kg": mass["representedRangeKg"],
            "upperMarginKg": mass["representedMarginKg"][0],
        }
    return {
        "incomingMapSha256": sha(src),
        "instances": len(present),
        "kept": sorted(keep),
        "originalOccurrences": group["originalOccurrences"],
        "originalGroupArticles": len(set(group["originalOccurrences"].values())),
        "zeroCreditMarginKg": prior["mass"]["representedMarginKg"][0],
        "rejectedCompositionControls": controls,
        "outcomes": result,
        "upperResidualBudgetKg": prior["mass"]["representedMarginKg"][0] + 3.9,
        "fixedGyro": {
            "instance": "observer-camera-proxy",
            "article": "OAK-D Lite fixed focus",
            "productionOption": "integrated BMI270",
            "additionalMassKg": 0.0,
            "cameraSource": sha(ROOT / "station-landmark/README.md"),
            "straightApproachRateModel": sha(ROOT / "reef-inertial-observer/rate.py"),
            "unitConfirmed": False,
            "kickstarterProvidesGyro": False,
        },
        "unboundedResidualMarginKg": None,
        "physicalScaleMassKg": None,
    }


def run(plugin, inputs, outputs):
    return {
        s: apply(
            plugin, inputs / s / "unknownGroupDeltaKg" / "selected.json", outputs / s
        )
        for s in ("densityMatchedRemovalKg", "noCutCreditKg")
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for k in ("plugin", "guideMaps", "outputs"):
        p.add_argument(k, type=lambda x: Path(x).resolve())
    p.add_argument("--receipt", type=Path, default=HERE / "straight-guide.json")
    a = p.parse_args()
    report = run(a.plugin, a.guideMaps, a.outputs)
    write(a.receipt, report)
    print(
        {
            k: round(r["outcomes"]["allStocksAndFourteenSetsKg"]["upperMarginKg"], 6)
            for k, r in report.items()
        }
    )
