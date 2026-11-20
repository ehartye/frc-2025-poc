"""Freeze the shop candidate's combined drawings and per-article inspection estimate."""

import csv
import json
from pathlib import Path
from attachments import base, common, cut, cyl, dist, fuse, tube, volume, write_step
from installed_audit import HERE, ROOT, bounds, box_gap, reopen, sha
from fitted_interfaces import machine_platform, mount_rio, lock_rings
import saddle_lands
from clearance import move

OUT = HERE / "issue"


def document(path, data):
    path.write_bytes((json.dumps(data, indent=2) + "\n").encode())


def rows(path):
    with path.open(encoding="utf8", newline="") as f:
        return list(csv.DictReader(f))


def write_csv(path, data):
    with path.open("w", encoding="utf8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(data[0]), lineterminator="\n")
        w.writeheader()
        w.writerows(data)


def run():
    OUT.mkdir(exist_ok=True)
    before = {k: s for k, *_, s in reopen()}
    prior = json.loads((HERE / "interfaces.json").read_bytes())
    old = base.named(HERE / "station-and-jack-interfaces.step")
    assert sha(HERE / "station-and-jack-interfaces.step") == prior["outputSha256"]
    drive = json.loads((HERE / "drive-budget.json").read_bytes())
    hoist = base.named(HERE / "jack-transmission.step") | base.named(
        HERE / "retained-shoes.step"
    )
    assert drive["drives"] == sha(HERE / "jack-transmission.step") and drive[
        "shoes"
    ] == sha(HERE / "retained-shoes.step")
    removed = set(prior["fixedHeight"]["removed"]) | {
        "Right drive and pawl envelope",
        "Left drive and pawl envelope",
    }
    combined = old | hoist
    saddle_lands.apply(before, combined)
    plate, windows = machine_platform(combined["front-electronics-platform"])
    plate, fasteners, case, holes = mount_rio(plate)
    combined["front-electronics-platform"] = plate
    combined["rio2-envelope"] = case
    combined.update(fasteners)
    import shoe_fittings

    shoe_fittings.apply(combined, before)
    import pdh_mount

    pdh, bridgeMount = pdh_mount.apply(combined, before)
    combined.update(pdh)
    rings = lock_rings()
    combined.update(rings)
    selected = {k: s for k, s in (before | combined).items() if k not in removed}
    # Source supplier mounting bore and shoulder validation is in rio-threaded-fit.
    # Isolate M3/M4 mated threads in the other studies. These axes are unthreaded
    # case witnesses, not assumed differentiable thread flanks.
    bboxes = {k: bounds(s) for k, s in selected.items()}
    pair = {}
    for k, s in (fasteners | rings | pdh).items():
        relevant = []
        for name, other in selected.items():
            if name == k or box_gap(bounds(s), bboxes[name]) > 0.01:
                continue
            val = common(s, other)
            if val > 0.01:
                relevant.append([name, round(val, 5)])
        pair[k] = relevant
    parts = OUT / "station-and-jack-interfaces.step"
    write_step(combined, parts)
    updated = {
        k: v
        for k, v in prior.items()
        if k in ("sourceArchiveSha256", "plateSource", "plateSourceSha256")
    }
    updated.update(
        outputSha256=sha(parts),
        replacement=sorted(set(combined) & set(before)),
        additional=sorted(set(combined) - set(before)),
        fixedHeight=dict(removed=sorted(removed)),
        robotSolids=len(selected),
        portionSolids=len(combined),
        inputKits={
            f: sha(HERE / f)
            for f in (
                "station-and-jack-interfaces.step",
                "jack-transmission.step",
                "retained-shoes.step",
                "saddle-bearing.json",
                "tab-fasteners.json",
            )
        },
        machiningWindows=windows,
        roboRioMountingCentersMm=holes,
        pdhRegistrationMm=[-212.2875, 155.5625, 161],
        pdhSupplierSha256=pdh_mount.SKU_SHA,
        pdhBaseFasteners=bridgeMount,
        additionalIntersectionsMm3=pair,
        stationaryAreas={
            k: common(
                move(s, (0, 0, 0.01 if "lower" in k else -0.01)),
                combined["support-machined-bridge"],
            )
            / 0.01
            for k, s in rings.items()
        },
    )
    document(OUT / "interfaces.json", updated)
    # Start with the unmodified complete issuance bill. Surrender actual physical
    # owners by ID, then insert each installed new body and charge once per article.
    ledger = {r["id"]: r for r in rows(HERE / "material-register.csv")}
    products = {r["instance"]: r for r in rows(HERE / "article-register.csv")}
    for k in (
        "Left drive and pawl envelope",
        "Right drive and pawl envelope",
        "jack-controls",
    ):
        products.pop(k)
        ledger.pop(k, None)
    products.update(
        {
            k: dict(
                instance=k,
                description=k,
                upperKg=v,
                source="cad/assembly-engineering/drive-budget.json",
            )
            for k, v in drive["newSeparateAllowancesKg"].items()
        }
    )
    for name, shape in combined.items():
        init = ledger.get(name, {})
        v = volume(shape)
        material = init.get("physicalDisplayMaterial", "Aluminum 6061")
        density = float(init.get("nativeNominalDensityKgM3", 2700))
        upper = float(init.get("designStockMaxDensityKgM3", 2850))
        description = init.get("item", name)
        instance = init.get("instance", name)
        basis = init.get("source", "cad/assembly-engineering/issue.py")
        if name in hoist:
            inst = drive["addedMaterialAndArticlesKg"][name]
            basis = "cad/assembly-engineering/drive-budget.json"
            material, density, upper = (
                ("Bronze", 8800, 9000)
                if "bronze" in name.lower() or "travelling nut" in name
                else ("Steel, Alloy", 7730, 8050)
                if "8050 kg" in inst["basis"] or "screw core" in name
                else ("Aluminum 6061", 2700, 2850)
            )
            if "native net volume" not in inst["basis"]:
                instance = name.replace(
                    "NEO550 connector reserve", "NEO550 REV-21-1651"
                )
                if "connector reserve" not in name:
                    products[instance] = dict(
                        instance=instance,
                        description=name,
                        upperKg=inst["upperKg"],
                        source=basis,
                    )
        if (
            name in pdh
            or "M2.5 stop locknut" in name
            or name == "Stem antirotation spring pin"
        ):
            material, density, upper = "Steel, Alloy", 7730, 8050
        if name in fasteners:
            if not name.endswith("-backing-spacer"):
                material, density, upper = "Steel, Alloy", 7730, 8050
        if name in rings:
            instance = "plunger-" + name.split("-")[2]
            material, density, upper = "Steel, Alloy", 7730, 8050
            description = (
                "ACE KM8 M8x1 AF10 x 3, one physical ring; charged within its absorber"
            )
            basis = "experiments/six-wheel/shelf-structure/damper_fit.py"
        article = instance in products
        ledger[name] = dict(
            id=name,
            instance=instance,
            kind="sealed article or routed assembly"
            if article
            else init.get("kind", "made-stock"),
            scope=init.get("scope", "inspection"),
            physicalDisplayMaterial=material,
            nativeNominalDensityKgM3=density,
            designStockMaxDensityKgM3=upper,
            volumeMm3=round(v, 4),
            displayStockKg=round(v * density * 1e-9, 8),
            inspectionStockUpperKg=0 if article else round(v * upper * 1e-9, 8),
            item=description,
            source=basis,
        )
    assert set(ledger) == set(selected) | {"Practice CORAL tube"}
    write_csv(OUT / "material-register.csv", [ledger[k] for k in sorted(ledger)])
    write_csv(OUT / "article-register.csv", [products[k] for k in sorted(products)])
    mass = sum(
        float(r["inspectionStockUpperKg"])
        for r in ledger.values()
        if r["scope"] == "inspection"
    ) + sum(
        float(r["upperKg"])
        for r in products.values()
        if r["instance"] != "np18-battery"
    )
    document(
        OUT / "mass.json",
        dict(
            issue=sha(parts),
            material=sha(OUT / "material-register.csv"),
            articles=sha(OUT / "article-register.csv"),
            inspectionUpperKg=mass,
            R103MarginKg=52.16312255 - mass,
            lastIntegratedDriveUpperKg=drive["estimatedUpperKg"],
            removedWindowVolumeMm3=sum(r["removedMm3"] for r in windows),
            installedCount=len(selected),
            projectAllowances=drive["newSeparateAllowancesKg"],
            scope="Includes 5 kg unresolved reserve. Sourced module and sealed electronics masses replace display solids. Excludes removable bumpers, the battery and demonstration piece from R103. Assembly strength, lot mass and field qualification remain conditional.",
        ),
    )
    print(sha(parts), mass, flush=True)


if __name__ == "__main__":
    run()
