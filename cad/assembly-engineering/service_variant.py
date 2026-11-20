"""Freeze a receiver-service change package and charge each installed article once."""

import json
from attachments import base, volume, write_step
from installed_audit import HERE, sha
from issue import document, rows, write_csv
from receiver_motion import OUT, assembly
from receiver_frames import corrections, witness_ids
from camera_budget import allocate

PARENT = HERE / "issue"
CONFIG = "fixed-low-captive-shallow-service-2"
FILENAME = "Team-1780-receiver-service.f3d"
LIMIT = 52.16312255
ROOF = OUT / "roof"
PRODUCTION = OUT / "issue"
MATS = {
    "al": ("Aluminum 6061", 2700, 2850),
    "steel": ("Steel, Alloy", 7730, 8050),
    "nylon": ("Nylon 6/6", 1130, 1250),
    "fr4": ("FR4", 1850, 2000),
    "abs": ("ABS Plastic", 1060, 1120),
}
RETIRED_ROOF = {"roof-guide-tower", "roof-guide-rib-2"}
SENSOR_SOURCE = (
    "https://components.omron.com/us-en/system/files/2023-01/datasheet_pdf/B036-E1.pdf"
)


def roof_changes():
    files = [
        ("roof-guide-foot-and-mounts.step", "fit.json", "deltaSha256"),
        ("roof-usb-guide-and-yoke.step", "roof-usb-guide-fit.json", "stepSha256"),
        ("short-boom-keyed-saddle.step", "short-boom-saddle-fit.json", "neutralSha256"),
    ]
    revised = {}
    paths = {}
    for step, receipt, field in files:
        pin = json.loads((ROOF / receipt).read_bytes())[field]
        if sha(ROOF / step) != pin:
            raise ValueError(step)
        for k, s in base.named(ROOF / step).items():
            key = "roof-guide-foot-" + k if k.startswith("M3") else k
            revised[key] = s
            paths[key] = "cad/assembly-engineering/receiver-service/roof/" + step
    return revised, paths


def classify(k, r):
    if k == "battery-hold-down":
        return MATS["fr4"]
    if k.endswith("compression-sleeve") or k.endswith("switch-bracket"):
        return MATS["al"]
    if k.startswith("SB50-") and k.endswith("-housing"):
        return ("Polycarbonate, Clear", 1200, 1260)
    if k.endswith("-switch"):
        return MATS["abs"]
    if k.endswith("-switch-lever"):
        return MATS["steel"]
    if k == "guide-cable-keeper":
        return MATS["nylon"]
    if r:
        nominal = {
            "Rubber, Black": 930,
            "Bronze": 8470,
            "Nylon 6/6": 1130,
            "Polyethylene, High Density": 952,
            "PVC, Unplasticized": 1290,
        }
        return (
            r["physicalDisplayMaterial"],
            nominal.get(
                r["physicalDisplayMaterial"], float(r["nativeNominalDensityKgM3"])
            ),
            float(r["designStockMaxDensityKgM3"]),
        )
    if k in (
        "guide-stay-head-plate",
        "roof-boom-mast-socket",
        "roof-mast-rear-saddle",
        "roof-guide-6061-boom",
    ) or k.endswith(("-pin-plug", "-spacer")):
        return MATS["al"]
    return MATS["steel"]


def run():
    previous, installed, frames, changes = assembly()
    fixes, source = roof_changes()
    stops = base.named(OUT / "stop-bearings.step")
    power = base.named(OUT / "battery-disconnect.step")
    delta = (
        base.named(HERE / "battery-fit/battery-retention.step")
        | changes
        | stops
        | power
        | fixes
    )
    paths_battery = {
        k: "cad/assembly-engineering/battery-fit/battery-retention.step"
        for k in base.named(HERE / "battery-fit/battery-retention.step")
    }
    paths_grip = {
        k: "cad/assembly-engineering/receiver-service/grip-carriers.step"
        for k in base.named(OUT / "grip-carriers.step")
    }
    paths_switch = {
        k: "cad/assembly-engineering/receiver-service/switch-mounts.step"
        for k in base.named(OUT / "switch-mounts.step")
    }
    source = (
        paths_battery
        | paths_grip
        | paths_switch
        | {
            k: "cad/assembly-engineering/receiver-service/stop-bearings.step"
            for k in stops
        }
        | source
    )
    source.update(
        {
            k: "cad/assembly-engineering/receiver-service/battery-disconnect.step"
            for k in power
        }
    )
    selected = installed | stops | power | fixes
    for k in RETIRED_ROOF:
        selected.pop(k)
    retired = set(previous) - set(selected)
    plan = json.loads((PARENT / "interfaces.json").read_bytes())
    original_rows = {r["id"]: r for r in rows(PARENT / "material-register.csv")}
    products = {r["instance"]: r for r in rows(PARENT / "article-register.csv")}
    ledger = dict(original_rows)
    for k in retired:
        del ledger[k]
    assert abs(float(products["slide-retracted-switch"]["upperKg"]) - 0.020) < 1e-8
    # The 20 g BOM rows are explicitly for the switch and operating lever.
    # Omron rates the sealed switch at approximately 0.5 g. Specify a 2 g
    # receiving maximum per unit. Terminals and dressed leads remain separate.
    for end in ("extended", "retracted"):
        k = "slide-" + end + "-switch"
        products[k].update(
            description="Omron D2F-01L-D3 SPDT hinge lever, 0.5 g catalog typical; 2 g receiving maximum",
            upperKg=0.002,
            source=SENSOR_SOURCE,
        )
    products["base-harness"]["description"] = (
        "Base harness installed acceptance cap including both SB50 housings, four contacts, two ring lugs and two battery leads. Allocate <=0.225 kg to this disconnect-and-pigtail set, leaving <=1.425 kg for remaining base wiring/terminations. All charged in inspection."
    )
    receiving = allocate(products)
    catalog_margin = {}
    for k, s in delta.items():
        old = ledger.get(k, {})
        name, density, upper = classify(k, old)
        owner = old.get("instance", k)
        if k.endswith("-switch-lever"):
            owner = k[:-6]
        if k in ("SB50-fixed-housing", "SB50-battery-housing"):
            owner = "base-harness"
        sealed = owner in products
        v = volume(s)
        ledger[k] = dict(
            id=k,
            instance=owner,
            kind="sealed article or routed assembly" if sealed else "made-stock",
            scope="inspection",
            physicalDisplayMaterial=name,
            nativeNominalDensityKgM3=density,
            designStockMaxDensityKgM3=upper,
            volumeMm3=round(v, 5),
            displayStockKg=round(v * density * 1e-9, 8),
            inspectionStockUpperKg=0 if sealed else round(v * upper * 1e-9, 8),
            item=old.get("item", k),
            source=source[k],
        )
    # Purchased shoulder screws have a supplier-size receiving mass above
    # their CAD geometry; include only the excess beyond each geometry charge.
    for k in ("roof-head-shoulder", "roof-root-shoulder"):
        bound = 0.0016
        prior = ledger[k]["inspectionStockUpperKg"]
        catalog_margin[k] = max(0, bound - prior)
        products[k] = dict(
            instance=k,
            description="Accu SSH-M2-3-16-A2 3 mm shoulder screw; modeled installed receiving upper",
            upperKg=max(bound, prior),
            source="cad/assembly-engineering/receiver-service/roof/short-boom-saddle-fit.json",
        )
        ledger[k].update(
            kind="sealed article or routed assembly", inspectionStockUpperKg=0
        )
    for k, r in ledger.items():
        mat = r["physicalDisplayMaterial"]
        if mat in (
            "Rubber, Black",
            "Bronze",
            "Nylon 6/6",
            "Polyethylene, High Density",
            "PVC, Unplasticized",
        ):
            _, rho, _ = classify(k, r)
            r["nativeNominalDensityKgM3"] = rho
            r["displayStockKg"] = round(float(r["volumeMm3"]) * rho * 1e-9, 8)
    assert set(ledger) == set(selected) | {"Practice CORAL tube"}
    PRODUCTION.mkdir(exist_ok=True)
    write_csv(PRODUCTION / "material-register.csv", [ledger[k] for k in sorted(ledger)])
    write_csv(
        PRODUCTION / "article-register.csv", [products[k] for k in sorted(products)]
    )
    neutral = PRODUCTION / "station-and-jack-interfaces.step"
    write_step(delta, neutral)
    added = set(delta) - set(previous)
    # The native material helper inserts these NEW bodies directly into the
    # matching slider component. Existing part prototypes keep their ancestry.
    assignments = {
        k: ("Outer slide" if frames[k] == 1 else "Middle slide")
        for k in added
        if frames.get(k, 0) > 0
    }
    assignments.update(corrections())
    upper = sum(
        float(r["inspectionStockUpperKg"])
        for r in ledger.values()
        if r["scope"] == "inspection"
    ) + sum(
        float(r["upperKg"])
        for r in products.values()
        if r["instance"] != "np18-battery"
    )
    pins = {
        str((HERE.parents[1] / path).relative_to(HERE.parents[1])).replace(
            chr(92), "/"
        ): sha(HERE.parents[1] / path)
        for path in sorted(set(source.values()))
    }
    record = dict(
        configuration=CONFIG,
        filename=FILENAME,
        parentIssue="issue",
        parentRegisterSha256=sha(PARENT / "interfaces.json"),
        sourceArchiveSha256=plan["sourceArchiveSha256"],
        parentArchiveSha256=sha(PARENT / "Team-1780-engineering-shallow.f3d"),
        outputSha256=sha(neutral),
        robotSolids=len(selected),
        inputs=pins,
        replacement=sorted(set(delta) & set(previous)),
        additional=sorted(added),
        frameAssignments=assignments,
        rigidWitnessIds=witness_ids(),
        appearanceRoles={
            k: (
                "bumper-red"
                if k in ("SB50-fixed-housing", "SB50-battery-housing")
                else "guide"
                if k.startswith("roof-") and name in ("Aluminum 6061", "Nylon 6/6")
                else "plate"
                if name in ("Aluminum 6061", "FR4")
                else "fastener"
            )
            for k in sorted(added)
            for name in [ledger[k]["physicalDisplayMaterial"]]
        },
        fixedHeight=dict(removed=sorted(retired | set(plan["fixedHeight"]["removed"]))),
    )
    document(PRODUCTION / "interfaces.json", record)
    document(OUT / "camera-receiving.json", receiving)
    document(
        PRODUCTION / "mass.json",
        dict(
            issue=sha(neutral),
            inspectionUpperKg=upper,
            R103MarginKg=LIMIT - upper,
            includedUnresolvedReserveKg=receiving["unassignedRemainderKg"],
            originalAggregateReserveKg=receiving["historicalCombinedAllowanceKg"],
            additionalFullReserveUpperKg=upper+receiving["reassignedToNamedArticlesKg"],
            installedSolids=len(selected),
            unionIds=sorted(retired),
            contractUpperReplacesGeometric=catalog_margin,
            material=sha(PRODUCTION / "material-register.csv"),
            articles=sha(PRODUCTION / "article-register.csv"),
            scope="Simulation upper with catalog substitutions, fixed hardware included. Battery, detachable bumpers and demonstration CORAL are excluded only from R103. A selected roof or jack restriction remains consequential even when mass passes.",
        ),
    )
    print("Installed / upper / margin", len(selected), upper, LIMIT - upper, flush=True)


if __name__ == "__main__":
    run()
