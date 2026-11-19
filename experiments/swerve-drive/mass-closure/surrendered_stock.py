"""Reopen the original outer clamps and both receiver solids before crediting stock."""

import csv
import hashlib
import json
import math
import re
import sys


def read(file):
    return json.loads(file.read_bytes())


def sha(file):
    return hashlib.sha256(file.read_bytes()).hexdigest()


def same(value, reference, tol=1e-7):
    if not math.isclose(value, reference, abs_tol=tol, rel_tol=0):
        raise ValueError(f"Unreconciled source values {value} and {reference}")


def evidence(project, source):
    hist = project / "experiments/six-wheel"
    swerve = project / "experiments/swerve-drive"
    package = swerve / "packaging"
    power = swerve / "power"
    sys.path.insert(0, str(hist / "design/station-l1"))
    import build_assembly as cad

    files = {}

    def record(path):
        files[path.relative_to(project).as_posix()] = sha(path)
        return path

    def shapes(path):
        names = re.findall(rb"PRODUCT\('([^']+)'", path.read_bytes())
        return cad.read_named(record(path), [{"id": name.decode()} for name in names])

    base = read(record(hist / "winchless-bill/original.json"))
    shop = read(record(hist / "design/competition-package/shop-duplex/mass.json"))
    shop_review = record(hist / "design/shop-package/native-review.json")
    if (
        sha(shop_review)
        != shop["sourceSha256"]["design/shop-package/native-review.json"]
    ):
        raise ValueError("Historical base charge changed")
    if base["separateDuplexIncrementKg"] != shop["poses"]["place"]["added_kg"]:
        raise ValueError("The old outer clamps are not tied to the baseline")
    groups = read(shop_review)["incrementalMassByFrame"]
    surrendered = {
        f"{station}-{side}-{name}"
        for station in ("rear", "center", "front")
        for side in ("L", "R")
        for name in ["nut-saddle"]
        + [
            f"{family}-{x}_{z}"
            for family in ("screw", "washer")
            for x in (-19, 19)
            for z in (-17, 17)
        ]
    }
    clamps = shapes(hist / "design/shop-package/proposed-joints.step")
    if {
        key for key in clamps if key.startswith(("rear-", "center-", "front-"))
    } != surrendered:
        raise ValueError("Expected six distinct nine-piece original clamp sets")
    retirement = read(record(package / "retirement.json"))
    changes = {c["id"]: c for c in source["changes"] if c["id"] in source["selected"]}
    bare = read(record(power / "mass.json"))
    earlier_credits = set(bare["oldBaselineRows"])
    for key in ("light", "axle-retention", "supported-drive-covers"):
        earlier_credits.update(changes[key]["add"] + changes[key]["remove"])
    earlier_credits.update(
        k
        for k in retirement["retired"]
        if k.startswith("unmodeled-wheel-to-hub-screw-")
    )
    if surrendered & earlier_credits or not surrendered <= set(
        retirement["retired"]
    ) & set(changes["four-module-swerve"]["remove"]):
        raise ValueError(
            "Partial surrender or duplicate credit for retired outer clamps"
        )
    amounts = {k: cad.volume(clamps[k])[0] for k in sorted(surrendered)}
    total = sum(amounts.values()) * 7.85e-6
    same(total, groups["base"]["deltaKg"], 0.5e-6)

    stock = read(record(hist / "winchless-bill/stock.json"))
    blanks = stock["retainedStock"]
    if len(blanks) != 7 or len({row["parent"] for row in blanks.values()}) != 4:
        raise ValueError("Wrong fixed-height fabrication")
    if any(row["materialSharedWithParentMm3"] != 0 for row in blanks.values()):
        raise ValueError("A formerly optional blank is now shared structural material")
    if set(blanks) & set(changes["four-module-swerve"]["add"]):
        raise ValueError("Legacy winch stock reintroduced by swerve")
    same(
        stock["stockAndFastenerUpperKg"],
        sum(row["finishedStockUpperKg"] for row in blanks.values())
        + 14 * stock["throughSetUpperKg"],
    )

    shelf = read(record(hist / "shelf-structure/installed.json"))
    purchased = sum(shelf["buyerUpperKg"].values())
    old_step = hist / "selected-match/receiver-clearance-kit.step"
    new_step = hist / "shelf-structure/stow-receiver.step"
    if (
        sha(old_step) != shelf["parentReceiver"]
        or sha(new_step) != shelf["manufacturingSha256"]
    ):
        raise ValueError("Different shelf geometry")
    old = shapes(old_step)
    new = shapes(new_step)
    original = cad.volume(old["receiver-R6-drilled-for-camera"])[0]
    facing = cad.volume(new["r6-polymer-facing"])[0]
    insert = cad.volume(new["r6-receiver-insert"])[0]
    cut = original - facing
    same(cut, shelf["removedPeMm3"], 1e-3)
    same(insert, shelf["volumeMm3"]["r6-receiver-insert"], 1e-3)
    density = read(record(hist / "shelf-structure/native.json"))["densityRangeKgPerM3"]
    cut_credit = cut * density["polymer"][0] * 1e-9
    insert_upper = insert * density["aluminum"][1] * 1e-9
    delta = changes["underside-shelf-insert"]["delta"]
    if (
        delta["pointer"] != "/noCutCreditKg"
        or "receiver-R6-ported" not in changes["underside-shelf-insert"]["remove"]
    ):
        raise ValueError("Shelf credit already taken or wrong source occurrence")
    same(delta["rangeKg"][1], purchased + insert_upper)
    same(shelf["fullReceiverMachinedDeltaKg"][1], insert_upper - cut_credit)
    bom = record(hist / "design/drivetrain/review/bom.csv")
    with bom.open(newline="") as stream:
        part = next(
            row for row in csv.DictReader(stream) if row["id"] == "receiver-R6-ported"
        )
    if (
        part["materialOrBasis"] != "PE"
        or float(part["maxMassKg"]) < facing * density["polymer"][1] * 1e-9 + cut_credit
    ):
        raise ValueError(
            "Parent PE charge cannot support both the facing and the removed stock"
        )
    return dict(
        sourceSha256=files,
        formerOuterClampVolumesMm3=amounts,
        originalShopOuterClampOnlyKg=groups["base"]["deltaKg"],
        retainedSlideJointDeltaKg=groups["slide"]["deltaKg"],
        possibleWinchResidue=blanks,
        legacyWinchBlankUpperKg=stock["stockUpperKg"],
        legacyThroughSetUpperKg=stock["throughSetUpperKg"],
        allPossibleWinchResidueKg=stock["stockAndFastenerUpperKg"],
        unknownRetainedKg=base["keptExamplesKg"]["unmodeled-allocation"],
        originalPEChargeKg=float(part["maxMassKg"]),
        drilledParentVolumeMm3=original,
        retainedFaceVolumeMm3=facing,
        receiverInsertVolumeMm3=insert,
        removedPEVolumeMm3=cut,
        materialDensityKgM3=density,
        densityMatchedR6CreditKg=cut_credit,
        absorberAndRetentionUpperKg=purchased,
        insertUpperKg=insert_upper,
    )
