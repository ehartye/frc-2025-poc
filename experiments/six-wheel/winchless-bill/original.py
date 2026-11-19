"""Sum the original source rows at matched endpoints with and without the winch row."""

import csv
import hashlib
import json
import sys
from pathlib import Path
from stock import ROOT, HERE, sha, j, read

sys.path.insert(0, str(ROOT / "design/stability"))
import stability


def make():
    base = read("design/stability/review.json")
    duplex = read("design/competition-package/shop-duplex/mass.json")
    for rec, lookup in ((base, "inputs_sha256"), (duplex, "sourceSha256")):
        for n, pin in rec[lookup].items():
            if sha(ROOT / n) != pin:
                raise ValueError("Predecessor changed: " + n)
    rows, exchange = stability.loads_for("place")
    if exchange != base["poses"]["place"]["step_sha256"]:
        raise ValueError("Different baseline")
    ledger = [{"name": r.name, "kg": r.mass} for r in rows]
    hoist = [r for r in rows if r.name == "lift"]
    if len(hoist) != 1 or tuple(hoist[0].mass) != (1.8, 3.9):
        raise ValueError("Original lift row is ambiguous")
    wheels = list(csv.DictReader((ROOT / "design/drivetrain/review/bom.csv").open()))
    wheels = [r for r in wheels if r["id"].startswith("wheel-")]
    if len(wheels) != 6:
        raise ValueError("Different wheels")
    book = [sum(float(r[k]) for r in wheels) for k in ("minMassKg", "maxMassKg")]
    new = [
        6 * v
        for v in read("design/selected-shop/articles.json")["wheels"]["accept_bare_kg"]
    ]
    inc = duplex["poses"]["place"]["added_kg"]
    empty = [sum(r.mass[i] for r in rows) for i in range(2)]
    if any(
        abs(v - u) > 1e-7
        for v, u in zip(empty, base["poses"]["place"]["empty"]["mass_kg"])
    ):
        raise ValueError("Original assembled weights do not close")

    def replaced(include):
        return [
            sum(r.mass[i] for r in rows if include or r.name != "lift")
            + inc[i]
            - 6.17
            - (5.9, 7.5)[i]
            - book[i]
            + new[i]
            for i in range(2)
        ]

    nonlift = [r for r in rows if r.name != "lift"]
    groups = {
        r.name: r.mass
        for r in nonlift
        if r.name
        in (
            "power cables, branches, moving service loops, leads and terminations",
            "four mechanism SPARK MAX units",
            "compressor, reservoir, valve, regulator, gauges and fittings",
            "long-range sensors and anchors",
            "unmodeled-allocation",
            "mast--235",
            "mast-210",
            "lift-crossbar",
            "deck-main",
        )
    }
    return dict(
        sourceRows=len(rows),
        originalRowsDigest=hashlib.sha256(
            json.dumps(ledger, sort_keys=True).encode()
        ).hexdigest(),
        originalGroupKg=hoist[0].mass,
        summedBaselineKg=replaced(True),
        baselineWithoutPrimaryLiftKg=replaced(False),
        selectedWheelKg=new,
        bookedWheelKg=book,
        batteryKg=6.17,
        bumperKg=[5.9, 7.5],
        separateDuplexIncrementKg=inc,
        keptExamplesKg=groups,
        sourceSha256={
            r: sha(ROOT / r)
            for r in (
                "design/stability/review.json",
                "design/stability/stability.py",
                "design/receiver-lift/check_routing.py",
                "design/drivetrain/review/bom.csv",
                "design/selected-shop/articles.json",
                "design/selected-shop/reconcile.py",
                "design/competition-package/shop-duplex/mass.json",
                "design/competition-package/shop-duplex/mass.py",
                "design/electrical-inspection/inspection.json",
            )
        },
    )


if __name__ == "__main__":
    j.write(HERE / "original.json", make())
