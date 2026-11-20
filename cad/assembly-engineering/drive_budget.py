"""Reconcile the bounded screw-drive construction, hardware and actual removed owners."""

import csv
import json
from attachments import base, volume
from installed_audit import HERE, sha


def calculate():
    primary = {
        r["id"]: r
        for r in csv.DictReader((HERE / "material-register.csv").open(encoding="utf8"))
    }
    old_articles = {
        r["instance"]: r
        for r in csv.DictReader((HERE / "article-register.csv").open(encoding="utf8"))
    }
    parts = {}
    for path in ("jack-transmission.step", "retained-shoes.step"):
        parts.update(base.named(HERE / path))
    removed = set(parts) & set(primary)
    removed.update(["Left drive and pawl envelope", "Right drive and pawl envelope"])
    surrender = sum(float(primary[k]["inspectionStockUpperKg"]) for k in removed)
    surrender += sum(
        float(old_articles[k]["upperKg"]) for k in removed if k in old_articles
    )
    reserve = float(old_articles["jack-controls"]["upperKg"])
    limited = {}
    for k, s in parts.items():
        estimate = volume(s)
        if "NEO550 connector reserve" in k:
            limited[k] = (0.0, "integral motor terminal envelope")
        elif "NEO550 REV" in k:
            limited[k] = (0.15, "142 g supplier motor + receiving")
        elif "5.23 stage" in k:
            limited[k] = (0.038, "36 g supplier cartridge + receiving")
        elif "NEO plate " in k:
            limited[k] = (0.036, "33.5 g supplier motor plate + receiving")
        elif "gear output" in k:
            limited[k] = (
                0.095,
                "unweighed complete REV-41-1604 output, project allowance",
            )
        elif "screw core" in k:
            # Even a full unturned blank from Z310 to Z746 is only this catalog
            # linear weight: both machined ends can only remove stock.
            limited[k] = (
                0.436 * 0.820 * 0.45359237 / 0.3048 * 1.04,
                "Roton 59285 catalog mass per length, zero machining credit, +4%",
            )
        else:
            metal = (
                9000
                if "bronze" in k.lower() or "travelling nut" in k.lower()
                else 8050
                if any(
                    n in k.lower()
                    for n in (
                        "m12",
                        "m10",
                        "m3-",
                        "thrust disk",
                        "thrust shaft",
                        "thrust housing",
                        "axk",
                        "pawl",
                        "ratchet",
                        "shaft drive",
                        "pin",
                        "bearing shoe",
                        "dovetail carrier",
                    )
                )
                else 2850
            )
            limited[k] = (
                estimate * metal * 1e-9,
                f"native net volume / {metal} kg per m³ receiving ceiling",
            )
    # Previously reserved combined two drives and 750 g wiring/control are all
    # surrendered exactly once, and replaced by these enumerated budget items.
    missing = {
        "two REV-41-1608 pinions, through-bolts and six cartridge screws per gearbox": 0.06,
        "four hardened washer sets / bearing packaging accuracy above ring volume": 0.02,
        "two spring returns, cable and lateral release actuators/feedback": 0.220,
        "screw lower retainers and upper tab/nut and weld material": 0.06,
        "second SPARK MAX (reuse isolated lift SPARK for the other jack)": 0.12,
        "dual controller power, grounds, three-phase, feedback, branch breaker and terminals": 0.44,
        "shoe spread, positive spread latch, lower preload mechanism and position sensing": 0.40,
    }
    old_total = json.loads((HERE / "mass-comparison.json").read_bytes())["groupsKg"][
        "inspection"
    ]
    proposed = (
        old_total
        - surrender
        - reserve
        + sum(x[0] for x in limited.values())
        + sum(missing.values())
    )
    return dict(
        issue=sha(HERE / "station-and-jack-interfaces.step"),
        drives=sha(HERE / "jack-transmission.step"),
        shoes=sha(HERE / "retained-shoes.step"),
        sourceUpperKg=old_total,
        surrendered=sorted(removed),
        surrenderedHardwareKg=surrender,
        surrenderedControlsAllowanceKg=reserve,
        addedMaterialAndArticlesKg={
            k: {"upperKg": v, "basis": basis}
            for k, (v, basis) in sorted(limited.items())
        },
        newSeparateAllowancesKg=missing,
        estimatedUpperKg=proposed,
        remainingMarginKg=52.16312255 - proposed,
        gates="Geometry represented by the earlier budget and the detailed alternative are distinct. Caps, supplier lot mass, unrepresented springs/cables/welds, terminal service and guide proof remain open. Keep the original full 5 kg unitemized allowance.",
    )


if __name__ == "__main__":
    (HERE / "drive-budget.json").write_bytes(
        (json.dumps(calculate(), indent=2) + "\n").encode()
    )
