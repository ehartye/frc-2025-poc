"""Book the source shelf replacement exactly once on the selected harness inventory."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent


def read(p):
    return json.loads(p.read_bytes())


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def write(p, value):
    p.write_text(json.dumps(value, indent=2) + "\n", encoding="utf8", newline="\n")


def run(plugin, incoming, out, installed=False):
    out.mkdir(parents=True, exist_ok=True)
    base = read(incoming)
    if not {"sp1-install", "selected-optics", "fixed-lead-install"} <= set(
        base["selected"]
    ):
        raise ValueError(
            "Expected issued dual camera, fixed harness and position mount"
        )
    shelf = "receiver-R6-ported"
    alive = set(x["id"] for x in base["instances"] if x["presence"] == "modeled")
    for chosen in base["selected"]:
        change = next(c for c in base["changes"] if c["id"] == chosen)
        alive.difference_update(change["remove"])
        alive.update(change["add"])
    if shelf not in alive:
        raise ValueError("Source shelf no longer independently installed")
    for s in base["sources"]:
        path = (incoming.parent / s["file"]).resolve()
        if sha(path) != s["sha256"]:
            raise ValueError("Changed " + s["id"])
        s["file"] = os.path.relpath(path, out).replace(os.sep, "/")
    native = read(HERE / "native.json")
    if sha(HERE / "shelf-insert.step") != native["kitSha256"]:
        raise ValueError("Different insert geometry")
    delta = native["netMachinedKg"]
    al = native["volumeMm3"]["r6-receiver-insert"]
    source = out / "insert-material.json"
    evidence = {
        "nativeSha256": sha(HERE / "native.json"),
        "densityMatchedRemovalKg": delta,
        "noCutCreditKg": [delta[0], al * 2850e-9],
    }
    write(source, evidence)
    base["sources"].append(
        {"id": "shelf-insert-stock", "file": source.name, "sha256": sha(source)}
    )
    new = ["shelf-structure/r6-polymer-facing", "shelf-structure/r6-receiver-insert"]
    retired = [shelf]
    if installed:
        kit = read(HERE / "installed.json")
        if sha(HERE / "stow-receiver.step") != kit["manufacturingSha256"]:
            raise ValueError("Unknown stow machining")
        bridge = "low-lock/slide-base-bridge"
        if bridge not in alive:
            raise ValueError("Unknown bridge occurrence")
        retired.append(bridge)
        new.append("shelf-structure/through-bore-bridge")
        caps = kit["buyerUpperKg"]
        low = kit["fullReceiverMachinedDeltaKg"][0] + 0.022
        high = kit["fullReceiverMachinedDeltaKg"][1] + sum(caps.values())
        delta = [low, high]
        evidence = {
            "nativeSha256": sha(HERE / "installed.json"),
            "densityMatchedRemovalKg": delta,
            "noCutCreditKg": [
                low,
                kit["volumeMm3"]["r6-receiver-insert"] * 2850e-9 + sum(caps.values()),
            ],
            "separatelyChargedUpperKg": caps,
            "bridgeChipCreditKg": 0,
        }
        write(source, evidence)
        base["sources"][-1]["sha256"] = sha(source)
    base["instances"].extend(
        {"id": k, "presence": "candidate", "disposition": "make"} for k in new
    )
    if installed:
        for ident, article, count in (
            ("absorber", "ACE MA30EUM", 2),
            ("locknut", "ACE KM8", 4),
            ("witness-and-retention", "Installed witness and retention material", 2),
        ):
            if not any(a["id"] == article for a in base["articles"]):
                base["articles"].append(
                    {"id": article, "sku": None, "unitsPerPack": None, "onHand": None}
                )
            for i in range(count):
                key = f"shelf-structure/{ident}-{i}"
                new.append(key)
                base["instances"].append(
                    {
                        "id": key,
                        "presence": "candidate",
                        "disposition": "buy",
                        "article": article,
                    }
                )
    summ = {}
    for key in ("densityMatchedRemovalKg", "noCutCreditKg"):
        import copy

        data = copy.deepcopy(base)
        option = "underside-shelf-insert"
        data["selected"].append(option)
        data["changes"].append(
            {
                "id": option,
                "revision": data["revision"],
                "configuration": data["configuration"],
                "remove": retired,
                "add": new,
                "delta": {
                    "source": "shelf-insert-stock",
                    "pointer": "/" + key,
                    "rangeKg": evidence[key],
                    "basis": "estimated",
                },
            }
        )
        path = out / (key + ".json")
        write(path, data)
        r = json.loads(
            subprocess.check_output(
                ["node", str(plugin / "scripts/assembly-accounting.mjs"), str(path)]
            )
        )
        coverage = {
            k: r["coverage"][k]
            for k in ("missing", "duplicates", "orphanIncluded", "unattached")
        }
        if any(coverage.values()):
            raise ValueError(coverage)
        summ[key] = {
            "deltaKg": evidence[key],
            "representedKg": r["mass"]["representedRangeKg"],
            "representedMarginKg": r["mass"]["representedMarginKg"],
            "coverage": coverage,
            "physicalMarginKg": r["mass"]["configurationMarginKg"],
        }
    cover = read(ROOT / "cover-retention/configurations.json")
    guide = read(ROOT / "station-guide/mass.json")["hypotheticalLinerDensityKgM3"][
        "500"
    ]["guideFaceKg"]
    comb = {}
    for name, row in cover.items():
        before = row["representedMarginKg"][0]
        comb[name] = {
            "upstreamMarginKg": before,
            "withInsertAndGuideBeforeAttachmentsKg": before - delta[1] - guide,
            "noCutCreditWithGuideKg": before - evidence["noCutCreditKg"][1] - guide,
        }
    deck = read(ROOT / "deck-joints/comparison.json")
    remaining = {
        k: {
            "originalMarginKg": v["representedMarginKg"][0],
            "withFaceAndGuideBeforeAttachmentsKg": v["representedMarginKg"][0]
            - delta[1]
            - guide,
        }
        for k, v in deck.items()
        if "representedMarginKg" in v
    }
    integral = read(ROOT / "ribbed-deck/budget.json")["cases"]
    current = {
        k: {
            "beforeKg": v["representedMarginKg"][0],
            "withSelectedAndGuideKg": v["representedMarginKg"][0] - delta[1] - guide,
        }
        for k, v in integral.items()
    }
    return {
        "plugin": read(plugin / "package.json")["version"],
        "sourceSelection": base["selected"],
        "installedChecker": summ,
        "coverAlternatives": comb,
        "tiedDeckAlternatives": remaining,
        "integralDeckAlternatives": current,
        "integralDeckSource": sha(ROOT / "ribbed-deck/budget.json"),
        "deckRegister": sha(ROOT / "deck-joints/comparison.json"),
        "guideUpperExampleKg": guide,
        "coverRegister": sha(ROOT / "cover-retention/configurations.json"),
        "guideRegister": sha(ROOT / "station-guide/mass.json"),
        "physicalInspectionMarginKg": None,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for k in ("plugin", "map", "out", "receipt"):
        p.add_argument(k, type=Path)
    p.add_argument("--installed", action="store_true")
    a = p.parse_args()
    write(
        a.receipt,
        run(a.plugin.resolve(), a.map.resolve(), a.out.resolve(), a.installed),
    )
