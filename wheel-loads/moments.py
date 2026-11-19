"""Bound the stowed center of gravity using source-located parcels on the selected robot."""

import argparse, csv, hashlib, json, sys
from pathlib import Path
import numpy as np
from contact import HERE, ROOT, read

sys.path.insert(0, str(ROOT / "design/stability"))
import stability


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def bound(model, scene):
    record = read(HERE / "contact.json")
    mass = record["mass"]["noCutCreditKg"]["emptyFieldKg"][1]
    if sha(scene) != "291d68c43e21f20c33885064e6bbe8c0e47b684d13a8a06e8af61f9fd9db5886":
        raise ValueError("Different registered scene")
    if sha(model) != "21bac2f0548e950994fd0dee7d9aaa5b1bf5fa53e3a41d275be7b388278b9c09":
        raise ValueError(
            "Regenerate the selected no-credit straight-guide map from the winchless bill"
        )
    data = read(model)
    if (
        data["mass"]["baseline"]["pointer"] != "/baselineKg/allStocksAndFourteenSetsKg"
        or "return-lower-guide-arm" in data["selected"]
    ):
        raise ValueError("Require the issued straight-arm upper remnant case")
    if "noCutCreditKg" not in str(
        next(r for r in data["changes"] if r["id"] == "underside-shelf-insert")["delta"]
    ):
        raise ValueError("Require no shelf-stock credit")
    frames = read(ROOT / "design/competition-package/part-frames.json")
    boxes = read(scene)["stowed"]
    old, pin = stability.loads_for("place")
    active = [r for r in data["changes"] if r["id"] in data["selected"]]
    retired = {n for r in active for n in r["remove"]}
    prior = {
        r["occurrence"]: r["instance"]
        for r in data["assignments"]
        if r["export"] == "stowed" and r.get("instance")
    }
    chosen = read(ROOT / "design/selected-shop/articles.json")["wheels"][
        "accept_bare_kg"
    ]
    # Gross support box for all unlocated stowed material, including later electronics/loom.
    bounds = np.array([-425.0, -425.0, 0.0, 425.0, 425.0, 1100.0])
    lookup = {"NP18-12 battery": "np18-battery"}
    # Some later operations rework the old stock under the same identity. Exclude
    # those parcels even where the new AABB and the buying record still match.
    with (ROOT / "design/bumper-kit/mass.csv").open() as f:
        removable = {
            r["solid"] for r in csv.DictReader(f) if r["assignment"] == "removable"
        }
    purchased = {
        "np18-battery",
        "gearbox-L",
        "gearbox-R",
        "motor-L",
        "motor-R",
        "drive-controller-L",
        "drive-controller-R",
        *("wheel-" + key for key in record["wheelBottomMm"]),
    }
    parcels = {}
    for row in old:
        name = lookup.get(row.name, row.name)
        if name not in purchased | removable:
            continue
        if (
            name is None
            or frames.get(name) != "base"
            or name not in boxes
            or name not in prior
            or prior[name] in retired
        ):
            continue
        if not np.allclose(row.extent, boxes[name], rtol=0.0, atol=1e-4):
            continue
        # Every source row is a disjoint mass parcel even if two CAD solids came in one purchased article.
        kg = chosen if name.startswith("wheel-") else row.mass
        if np.any(np.array(row.bounds[:3]) < bounds[:3] - 1e-5) or np.any(
            np.array(row.bounds[3:]) > bounds[3:] + 1e-5
        ):
            raise ValueError("Parcel outside the proposed hull")
        parcels[row.name] = {
            "body": name,
            "article": prior[name],
            "kg": kg,
            "positionMm": list(row.bounds),
        }
    if (
        "NP18-12 battery" not in parcels
        or len([p for p in parcels if p.startswith("wheel-")]) != 6
    ):
        raise ValueError("Incomplete battery/wheel locator")
    # Reopen the actual final panel. To tolerate a redistribution of machining
    # stock, permit both missing and added material up to the design reserve each.
    material = read(ROOT / "ribbed-deck/budget.json")["material"]
    deck_name = "mast-slotted-ribbed-deck"
    deck_path = ROOT / "mast-attachment/slotted-ribbed-deck.step"
    if sha(deck_path) != read(ROOT / "mast-attachment/fit.json")["ribbedSlottedSha256"]:
        raise ValueError("Changed slotted panel")
    if deck_name in retired or not any(deck_name in r["add"] for r in active):
        raise ValueError("Changed selected panel")
    native = stability.assembly.native
    deck = native.read_named(deck_path, [{"id": deck_name}])[deck_name]
    v, center = native.volume(deck)
    box = native.bounds(deck)
    tolerance = read(ROOT / "ribbed-deck/machining.json")["finishStockVolumeReserveMm3"]
    minimum = v - tolerance
    loc = []
    for i in range(3):
        # First moment relative to the nominal centroid. The maximum absolute error
        # from an added and removed parcel is bounded by tolerance * full span.
        dx = tolerance * (box[i + 3] - box[i]) / minimum
        loc.append((center[i] - dx, center[i] + dx))
    parcels[deck_name] = {
        "body": deck_name,
        "article": deck_name,
        "kg": [
            minimum * material["lotDensityKgPerMm3"][0],
            (v + tolerance) * material["lotDensityKgPerMm3"][1],
        ],
        "positionMm": [r[0] for r in loc] + [r[1] for r in loc],
    }
    total = sum(v["kg"][0] for v in parcels.values())
    centroid = []
    for i in range(3):
        axis = []
        for j in (i, i + 3):
            wall = bounds[j]
            axis.append(
                wall
                + sum(
                    v["kg"][0] * (v["positionMm"][j] - wall) for v in parcels.values()
                )
                / mass
            )
        centroid.append(axis)
    # Guaranteed lateral bracket for centers of mass of *everything not positively
    # located above*. Allocate just the known lower mass at each certified parcel.
    # Additional mass on a known item is conservatively part of the unlocated remainder.
    remainder = mass - total
    lo, hi = [
        sum(v["kg"][0] * v["positionMm"][i] for v in parcels.values()) for i in (1, 4)
    ]
    allowed_y = record["static44pctYmaxMm"]
    target = [(-mass * allowed_y - lo) / remainder, (mass * allowed_y - hi) / remainder]
    return {
        "inputMapSha256": sha(model),
        "sourceStep": pin,
        "sourceSceneSha256": sha(scene),
        "assumedStowedAllMaterialBoxMm": bounds.tolist(),
        "fieldMassUpperKg": mass,
        "locatedParcels": len(parcels),
        "locatedLowerKg": total,
        "unlocatedUpperRemainderKg": remainder,
        "grossCgOuterBoxMm": centroid,
        "knownLateralMomentKgMm": [lo, hi],
        "permittedOtherCentroidYmmForStatic44pctAtUpperMass": target,
        "fixedParcelRows": parcels,
        "deck": {
            "stepSha256": sha(deck_path),
            "nominalVolumeMm3": v,
            "nominalUniformCentroidMm": center,
            "maxAddedAndMissingMm3Each": tolerance,
            "uniformLotDensity": True,
        },
        "actualSelectedCgMm": None,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("map", type=Path)
    p.add_argument("scene", type=Path)
    a = p.parse_args()
    (HERE / "moments.json").write_text(
        json.dumps(bound(a.map, a.scene), indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
