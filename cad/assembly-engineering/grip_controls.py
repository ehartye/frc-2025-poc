"""Check actual fastener seats, assembly tools, masses and signed jaw travel."""

import json
from attachments import base, bounds, box_gap, common, dist, volume
from clearance import move
from installed_audit import HERE, reopen, sha
from issue import document, rows


def moving(name, side):
    if not name.startswith("grip-" + side + "-"):
        return False
    fixed = (
        "-tower",
        "-support-foot",
        "-riser",
        "-long-shelf-screw",
        "-fixed-backbone",
        "-dashpot-housing",
        "-hardstop",
        "-slide-guide",
        "-guide-washer",
        "-guide-locknut",
        "-spring",
        "-spring-seat-shim",
    )
    return not name.endswith(fixed)


def check():
    out = HERE / "receiver-service"
    receipt = json.loads((out / "grip-fit.json").read_bytes())
    assert receipt["drawingSha256"] == sha(out / "grip-carriers.step")
    previous = {k: s for k, *_, s in reopen()} | base.named(
        HERE / "issue/station-and-jack-interfaces.step"
    )
    for k in json.loads((HERE / "issue/interfaces.json").read_bytes())["fixedHeight"][
        "removed"
    ]:
        previous.pop(k)
    previous.update(base.named(HERE / "battery-fit/battery-retention.step"))
    updated = base.named(out / "grip-carriers.step")
    selected = previous | updated
    for k in receipt["removed"]:
        selected.pop(k)
    stocks = {r["id"]: r for r in rows(HERE / "issue/material-register.csv")}
    budget = {}
    for k in receipt["removed"] + list(updated):
        old = stocks.get(k, {})
        oldcharge = float(old.get("inspectionStockUpperKg", 0))
        if k not in updated:
            new = 0
        elif old.get("kind") == "sealed article or routed assembly":
            new = 0
        else:
            density = float(old.get("designStockMaxDensityKgM3", 8050))
            new = volume(updated[k]) * density * 1e-9
        budget[k] = round(new - oldcharge, 8)
    boxes = {k: bounds(s) for k, s in selected.items()}
    bearing = {}
    travel = {}
    broad = {}
    for side, sign in (("L", 1), ("R", -1)):
        prefix = "grip-" + side + "-"
        plate = selected[prefix + "cartridge-mount"]
        cheek = selected[prefix + "upper-cheek"]
        for k, shape in updated.items():
            if not k.startswith(prefix):
                continue
            if "-cheek-tie-" in k or "-output-tie-" in k:
                dz = -0.01 if "-cheek-tie-" in k else 0.01
                area = common(move(shape, (0, 0, dz)), plate) / 0.01
                bearing[k] = dict(
                    headAreaMm2=area, nominalHeadCarrierMm3=common(shape, plate)
                )
        bearing[prefix + "plate-to-cheek"] = dict(
            nominalMm3=common(plate, cheek),
            gapMm=dist(plate, cheek),
            seatAreaMm2=common(move(plate, (0, 0, -0.01)), cheek) / 0.01,
        )
        for suffix in ("front", "rear"):
            name = prefix + suffix
            pin = selected[name + "-slide-guide"]
            tower = selected[name + "-tower"]
            bearing[name + "-pin"] = dict(
                shoulderAreaMm2=common(move(pin, (0, sign * 0.01, 0)), tower) / 0.01,
                nominalMm3=common(pin, tower),
                washerAreaMm2=common(
                    move(selected[name + "-guide-washer"], (0, -sign * 0.01, 0)), tower
                )
                / 0.01,
                threadBeyondNutMm=2,
            )
        # Scope to the newly fitted grip stack. All rigid pieces on this same
        # jaw share one lateral transform; fixed spring guides do not.
        for k, s in updated.items():
            if not k.startswith(prefix) or not moving(k, side):
                continue
            stationary = {j: v for j, v in selected.items() if not moving(j, side)}
            # The single-piece upper cheek already touches the original moving
            # idler insert and touches a compliant spring at the issued lower
            # stack. Only the four new boss volumes above the cheek are tested.
            if k.endswith("-upper-cheek"):
                from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut

                s = BRepAlgoAPI_Cut(s, previous[k]).Shape()
            extents = bounds(s)
            lower = sorted((box_gap(extents, boxes[j]), j) for j in stationary)
            broad[k] = dict(
                fixedBodies=len(lower),
                closestBox=lower[0][1],
                signedClearanceLowerMm=round(lower[0][0] - 1.5, 5),
            )
            active = [j for d, j in lower if d < 1.53]
            for j in active:
                if j.endswith(("-spring", "-spring-seat-shim")):
                    continue
                support = stationary[j]
                distances = []
                for offs in (
                    -1.5,
                    -1.25,
                    -1,
                    -0.75,
                    -0.5,
                    -0.25,
                    0,
                    0.25,
                    0.5,
                    0.75,
                    1,
                    1.25,
                    1.5,
                ):
                    distances.append(dist(move(s, (0, sign * offs, 0)), support))
                # 0.25 mm separation between samples, so translation alone can
                # reduce the adjacent sampled distance by at most 0.125 mm.
                travel[k + " / " + j] = round(min(distances) - 0.125, 5)
    limits = [k for k, v in travel.items() if v <= 0]
    result = dict(
        gripDrawing=receipt["drawingSha256"],
        massDeltaByItemKg=budget,
        totalUpperDeltaKg=sum(budget.values()),
        afterBatteryUpperKg=52.044595421058375
        + json.loads((HERE / "battery-fit/fit.json").read_bytes())["inspectionDeltaKg"]
        + sum(budget.values()),
        seats=bearing,
        broadPhase=broad,
        continuousLateralTravelMm=[-1.5, 1.5],
        clearanceLowerBoundsMm=travel,
        unresolvedTravelPairs=limits,
    )
    document(out / "grip-controls.json", result)
    print(
        "Upper, minimum seat and unresolved pairs:",
        result["afterBatteryUpperKg"],
        min(r["headAreaMm2"] for r in bearing.values() if "headAreaMm2" in r),
        limits,
        flush=True,
    )


if __name__ == "__main__":
    check()
