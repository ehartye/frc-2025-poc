"""Emit the installed Agent FRC mass review from the shop bill and independent CAD identities."""

import argparse
import hashlib
import json
import re
from collections import defaultdict
from pathlib import Path

from installed_audit import HERE, sha
from issue import document, rows

LIBRARY = {
    "ABS Plastic": (
        "022",
        1000,
        1120,
        "ABS thermoplastic; confirm purchased enclosure grade",
    ),
    "Aluminum 6061": (
        "231",
        2650,
        2850,
        "6061-T6 machined stock; verify temper after welding",
    ),
    "Aluminum 7075": ("067-1-2", 2750, 2920, "7075-T6 machined bracket stock"),
    "Bronze": (
        "004",
        8300,
        9000,
        "Bearing bronze; use specified Roton supplied nut or engineering-grade bushing",
    ),
    "Bumper marking, thickness-equivalent": (
        "272",
        850,
        1300,
        "Applied white cover lettering, equivalent solid thickness",
    ),
    "Expanded polyethylene, 37 kg per cubic metre": (
        "272",
        31,
        45,
        "Closed-cell PE foam, receiving density 31 to 45 kg/m3",
    ),
    "FR4": ("401", 1750, 2000, "Glass-reinforced epoxy PCB laminate"),
    "Nylon 6/6": ("023", 1050, 1250, "PA66 cable guide stock"),
    "PE coated cloth, areal equivalent": (
        "272",
        600,
        800,
        "Coated bumper fabric, equivalent solid thickness",
    ),
    "Polyethylene, High Density": ("273", 915, 990, "Receiver wear facing, HDPE stock"),
    "PVC, Unplasticized": ("282", 1250, 1460, "Rigid PVC practice piece"),
    "Rubber, Black": ("053", 900, 1300, "Rubber belt or tread envelope"),
    "Steel, Alloy": (
        "230",
        7500,
        8050,
        "Steel stock; procure alloy and heat treatment from the individual drawing or fastener designation",
    ),
}
CONFIG = "fixed-low-captive-shallow-service-1"


def pointer(key):
    return key.replace("~", "~0").replace("/", "~1")


def native_site(k, initial, changed):
    """Issue paths derive from the original occurrence register and new assembly groups."""
    for side in ("Left", "Right"):
        if k.startswith(side + " "):
            part = k[len(side) + 1 :]
            if part in {
                "screw core",
                "ratchet wheel",
                "upper thrust disk",
                "shoulder thrust disk",
                "M10 thrust locknut",
                "M10 tab washer",
                "shaft drive socket",
                "under thrust shaft washer",
                "over thrust shaft washer",
            }:
                return side + " screw rotation:1", k
            if part == "pawl":
                return side + " pawl rotation:1", k
            if part == "travelling nut":
                return "Cage crosshead:1", k
            if k not in initial:
                return "Shallow cage jack frame:1", k
    if (
        "bearing shoe stop pin" in k
        or "bearing shoe M2.5 stop locknut" in k
        or k == "Stem antirotation spring pin"
    ):
        return "Cage crosshead:1", k
    if k in initial:
        r = initial[k]
        return r["occurrence"], k if k in changed else r["body"]
    return f"Fixed chassis:1+{k}:1", k


def create(folder, archive, observe):
    # Require the native source to be next to the other issued files. A checkout
    # copy and an installed publisher run can reproduce these relative pins.
    if folder != archive.parent or folder != observe.parent:
        raise ValueError(
            "Keep the exact archive and observation beside the generated map."
        )
    source = rows(folder / "material-register.csv")
    buying = rows(folder / "article-register.csv")
    initial = {
        r["body"]
        if r["body"] != "Body1"
        else re.split(r":\d+\+", r["occurrence"])[-1].rsplit(":", 1)[0]: r
        for r in rows(HERE / "as-issued-native.csv")
    }
    changed = json.loads((folder / "interfaces.json").read_bytes())
    changed = set(changed["additional"] + changed["replacement"])
    owners = defaultdict(list)
    plan_rows = []
    evidence = dict(
        occurrences=[],
        mass={},
        supplier={},
        stock={},
        exclusions={},
        limitKg=52.16312255,
    )
    excluded = set()
    for r in source:
        k = r["id"]
        who = r["instance"]
        stock = r["physicalDisplayMaterial"]
        occurrence, body = native_site(k, initial, changed)
        row = dict(
            occurrence=occurrence,
            body=body,
            identity=hashlib.sha256(
                ("team-1780-engineering-v1/" + k).encode()
            ).hexdigest(),
        )
        evidence["occurrences"].append(k)
        if not who:
            row.update(
                role="fixture",
                reason="Demonstration CORAL, independent non-installed article",
            )
        else:
            owners[who].append(r)
            if r["scope"] != "inspection":
                excluded.add(who)
            row["instance"] = who
            if r["kind"] in ("made-stock", "surface treatment"):
                row.update(role="stock", stock=stock)
                spec, lo, hi, grade = LIBRARY[stock]
                evidence["stock"][stock] = dict(
                    id=stock,
                    grade=grade,
                    materialIds=["PrismMaterial-" + spec],
                    densityKgM3=[lo, hi],
                )
            else:
                row["role"] = "article-proxy"
        plan_rows.append(row)
    contracts = {r["instance"]: r for r in buying}
    evidence["exclusions"] = {
        "criterion": "2025 V0 R103 excludes removable battery and bumpers; fixed mounts remain counted.",
        "instances": sorted(excluded),
    }
    instances = []
    policies = []
    articles = {}
    assignments = []
    for k in sorted(set(owners) | set(contracts)):
        rr = owners.get(k, [])
        ref = "/mass/" + pointer(k)
        policy = dict(id=k, inspection="excluded" if k in excluded else "counted")
        if k in excluded:
            policy["reason"] = (
                "Removable battery under R103"
                if k == "np18-battery"
                else "Removable bumper assembly under R103"
            )
            policy["inspectionEvidence"] = dict(source="bill", pointer="/exclusions")
        item = dict(id=k, presence="modeled" if rr else "unmodeled")
        if k not in contracts:
            lo = sum(
                float(r["volumeMm3"]) * LIBRARY[r["physicalDisplayMaterial"]][1] * 1e-9
                for r in rr
            )
            hi = sum(float(r["inspectionStockUpperKg"]) for r in rr)
            interval = [round(lo * 0.999, 9), round(hi + 1e-7, 9)]
            policy["basis"] = "stock"
            item["disposition"] = "make"
        else:
            contract = contracts[k]
            upper = float(contract["upperKg"])
            # Only specific catalog unit masses qualify here. A receiving cap,
            # logistics weight, grouped housing or volume allowance stays visible.
            sku, lower = ("", 0.0)
            if k.startswith("spark-") or k.endswith(
                ("-drive-controller", "-steer-controller")
            ):
                sku, lower = "REV-11-2158", 0.1133
            elif k == "rev-pdh":
                sku, lower = "REV-11-1850", 0.517
            elif k in ("Left NEO550 REV-21-1651", "Right NEO550 REV-21-1651"):
                sku, lower = "REV-21-1651", 0.142
            elif "5.23 stage REV-41-1603" in k:
                sku, lower = "REV-41-1603", 0.036
            elif "NEO plate REV-41-1607" in k:
                sku, lower = "REV-41-1607", 0.0335
            interval = [lower, upper]
            if sku:
                articles[sku] = dict(id=sku, sku=sku)
                item.update(disposition="buy", article=sku)
                evidence["supplier"][k] = dict(
                    sku=sku,
                    unit="kg per installed unit",
                    rangeKg=interval,
                    reference=contract["source"],
                )
                policy.update(
                    basis="supplier",
                    supplier=dict(source="bill", pointer="/supplier/" + pointer(k)),
                )
            else:
                policy["basis"] = "allowance"
                item["disposition"] = "undecided"
        evidence["mass"][k] = interval
        item["mass"] = dict(
            source="bill", basis="estimated", pointer=ref, rangeKg=interval
        )
        instances.append(item)
        policies.append(policy)
    for r in source:
        k, owner = r["id"], r["instance"]
        assignments.append(
            dict(export="stock", occurrence=k, role="installed", instance=owner)
            if owner
            else dict(export="stock", occurrence=k, role="external")
        )
    digest = sha(archive)
    plan = dict(
        schema="agent-frc/engineering-material-plan@1",
        archiveSha256=digest,
        configuration=CONFIG,
        bodies=plan_rows,
        stocks=[
            dict(id=k, source="bill", pointer="/stock/" + pointer(k))
            for k in sorted(evidence["stock"])
        ],
        instances=policies,
    )
    document(folder / "weighing-evidence.json", evidence)
    document(folder / "material-plan.json", plan)
    files = {
        "assembly": archive.name,
        "native": observe.name,
        "bill": "weighing-evidence.json",
        "plan": "material-plan.json",
    }
    mapping = dict(
        schema="agent-frc/assembly-accounting@1",
        revision=digest,
        configuration=CONFIG,
        scope="complete",
        omissions=[],
        sources=[dict(id=k, file=v, sha256=sha(folder / v)) for k, v in files.items()],
        exports=[dict(id="stock", source="bill", pointer="/occurrences")],
        articles=list(articles.values()),
        assignments=assignments,
        instances=instances,
        changes=[],
        selected=[],
        mass=dict(
            configuration=CONFIG,
            revision=digest,
            unit="kg",
            limitKg=52.16312255,
            limitEvidence=dict(source="bill", pointer="/limitKg"),
            baseline=None,
            omissions=[],
        ),
        nativeMass=dict(archive="assembly", observations="native", plan="plan"),
    )
    document(folder / "mapping.json", mapping)


if __name__ == "__main__":
    cli = argparse.ArgumentParser()
    cli.add_argument("folder", type=Path)
    a = cli.parse_args().folder.resolve()
    create(a, a / "Team-1780-engineering-shallow.f3d", a / "engineering-reopen.json")
