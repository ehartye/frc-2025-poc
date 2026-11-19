"""Apply the rail replacements to the selected straight-guide inspection inventory."""

import argparse
import math
import sys
from pathlib import Path
from balance import HERE, ROOT, digest, read

sys.path.insert(0, str(ROOT / "winchless-bill"))
from account import installed, pins, roster, write


def check(plugin, source, out):
    out.mkdir(exist_ok=True, parents=True)
    fixture = read(HERE / "keepers.json")
    if digest(HERE / "rail-fixture.step") != fixture["stepSha256"]:
        raise ValueError("Changed keeper exchange")
    model = read(source)
    before = installed(plugin, source)
    present = roster(before)
    pins(model, source, out)
    volumes = fixture["volumesMm3"]
    changes = fixture["machinedDeltaMm3"]
    old = {k: "clearance/" + k if k.startswith("drilled-slide") else k for k in changes}
    replacement = ["captured/" + k for k in volumes]
    if not (
        set(old.values()) <= present
        and set(replacement).isdisjoint(present)
        and math.isclose(
            before["mass"]["representedMarginKg"][0],
            read(ROOT / "winchless-bill/straight-guide.json")["noCutCreditKg"][
                "outcomes"
            ]["allStocksAndFourteenSetsKg"]["upperMarginKg"],
            abs_tol=1e-8,
        )
    ):
        raise ValueError("Expected the selected no-shelf-credit straight-guide bill")
    # Treat the old and replacement stock as independently varying materials,
    # including all slots and added integral bosses. Additional purchased
    # articles have no removal credit; upper reserve covers joint compound.
    before_vol = sum(volumes[k] - delta for k, delta in changes.items())
    after_vol = sum(volumes[k] for k in changes)
    steel_vol = sum(v for k, v in volumes.items() if k not in changes)
    kg = [
        after_vol * 2.60e-6 - before_vol * 2.85e-6 + steel_vol * 7.70e-6,
        after_vol * 2.85e-6 - before_vol * 2.60e-6 + steel_vol * 8.10e-6 + 0.005,
    ]
    evidence = out / "rail-stock.json"
    write(
        evidence,
        {
            "stepSha256": fixture["stepSha256"],
            "rangeKg": kg,
            "addedFinishReserveKg": [0, 0.005],
            "changeAlMm3": after_vol - before_vol,
            "newSteelMm3": steel_vol,
        },
    )
    model["sources"].append(
        {"id": "captured-rail-stock", "file": evidence.name, "sha256": digest(evidence)}
    )
    model["instances"].extend(
        {"id": key, "presence": "candidate", "disposition": "make"}
        for key in replacement
    )
    model["changes"].append(
        {
            "id": "captured-rail-fixture",
            "configuration": model["configuration"],
            "revision": model["revision"],
            "remove": list(old.values()),
            "add": replacement,
            "delta": {
                "source": "captured-rail-stock",
                "basis": "estimated",
                "pointer": "/rangeKg",
                "rangeKg": kg,
            },
        }
    )
    model["selected"].append("captured-rail-fixture")
    chosen = out / "selected.json"
    write(chosen, model)
    report = installed(plugin, chosen)
    if roster(report) != (present - set(old.values())) | set(replacement):
        raise ValueError("Wrong installed instance partition")
    margin = report["mass"]["representedMarginKg"]
    if not math.isclose(
        margin[0], before["mass"]["representedMarginKg"][0] - kg[1], abs_tol=1e-8
    ):
        raise ValueError("Unreconciled stock")
    return {
        "version": read(plugin / "package.json")["version"],
        "sourceSha256": digest(source),
        "stepSha256": fixture["stepSha256"],
        "removed": list(old.values()),
        "added": replacement,
        "installed": len(roster(report)),
        "deltaKg": kg,
        "priorUpperMarginKg": before["mass"]["representedMarginKg"][0],
        "upperMarginKg": margin[0],
        "installedPhysicalMassKg": None,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for field in ("plugin", "source", "out", "receipt"):
        p.add_argument(field, type=lambda v: Path(v).resolve())
    a = p.parse_args()
    write(a.receipt, check(a.plugin, a.source, a.out))
