"""Check each rear facet against the original purchased mounting plates and wheel service."""

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Trsf, gp_Vec

from proposal import ROOT, HERE
from shape import facet, prism, box_gap, distance, volume
from cold_issue import precise

sys.path.insert(0, str(ROOT / "experiments/swerve-drive/packaging"))
from envelopes import corner_data, load_raw
from frame import register
from native_module import PIN


def nearest(article, material):
    bb = precise(article)
    lower = sorted((box_gap(bb, precise(s)), k) for k, s in material.items())
    best = math.inf
    winner = None
    for gap, k in lower:
        if gap >= best:
            break
        value = distance(article, material[k])
        if value < best:
            best, winner = value, k
    return {"gapMm": best, "supplierPart": winner}


def run(cache):
    raw = load_raw(cache)
    envelopes, _, _ = corner_data(cache)
    registered_json = (
        ROOT / "cad/assembly-engineering/receiver-service/issue/engineering-reopen.json"
    )
    report = json.loads(registered_json.read_bytes())
    checks = {}
    for side, corner in ((1, "RL"), (-1, "RR")):
        complete = {k: register(s, corner) for k, s in envelopes.items()}
        plates = {
            k: register(s, corner)
            for k, s, _ in raw
            if k in {"9-REV-21-3005-P01", "10-REV-21-3005-P02"}
        }
        assert len(plates) == 2
        for c in (19.05, 25.4, 50.8, 76.2):
            candidate = facet(c, side)
            tr = gp_Trsf()
            tr.SetTranslation(
                gp_Vec(0.25 / math.sqrt(2), -side * 0.25 / math.sqrt(2), 0)
            )
            shifted = BRepBuilderAPI_Transform(candidate, tr, True).Shape()
            record = {}
            for name, p in plates.items():
                record[name] = {
                    "gapMm": distance(candidate, p),
                    "nominalCommonMm3": volume(
                        BRepAlgoAPI_Common(candidate, p).Shape()
                    ),
                    "inward025CommonMm3": volume(
                        BRepAlgoAPI_Common(shifted, p).Shape()
                    ),
                }
            checks[f"{corner}-{c}"] = {"actualFixedPlates": record}
            if c == 19.05:
                # During a 195 mm vertical cartridge pull, the coupon sweeps
                # upward relative to that cartridge. Its two caps span the full polygon.
                intercept = 700 - c - 20 * math.sqrt(2)
                swept = prism(
                    [
                        (350, 350 - c),
                        (350 - c, 350),
                        (intercept - 325, 325),
                        (330, intercept - 330),
                    ],
                    side,
                    50,
                    345,
                )
                hardware = {
                    v["occurrence"]: box_gap(precise(swept), v["boundsMm"])
                    for v in report["bodies"]
                    if ("+" + corner + "-") in v["occurrence"]
                    and not any(
                        k in v["occurrence"]
                        for k in ("rail-screw", "welded-thread-boss")
                    )
                }
                checks[f"{corner}-{c}"].update(
                    allAzimuth=nearest(candidate, complete),
                    fullCartridgeWithdrawal=nearest(swept, complete),
                    registeredSupplierGroups=len(complete),
                    detachableModuleHardwareBodies=len(hardware),
                    detachableHardwareMinimumBoxGapMm=min(hardware.values()),
                )
    build = json.loads((HERE / "construction.json").read_bytes())
    intercept = build["outsideDiagonalInterceptMm"]
    battery = next(
        r["boundsMm"] for r in report["bodies"] if r["instance"] == "np18-battery"
    )
    route = ROOT / "experiments/six-wheel/moving-harness/routing.py"
    sys.path.insert(0, str(route.parent))
    import routing

    t0 = (
        routing.FIXED_X
        + routing.MOVING_X
        + math.pi * routing.RADIUS
        - routing.MANAGED_LENGTH
    ) / 2
    reverse_extent = t0 - routing.RADIUS - 57
    output = dict(
        sourceArchive=report["archiveSha256"],
        sourceNativeObservations=hashlib.sha256(
            registered_json.read_bytes()
        ).hexdigest(),
        manufacturerModule=PIN,
        controls=checks,
        swerveCentersMm={"RL": [-282, 277], "RR": [-282, -277]},
        originalCarrier=dict(
            chainLinks=routing.CHAIN_LINKS,
            lowestConservativeZmm=588.5,
            highestBumperZmm=181.5,
            verticalClearanceMm=407,
            outerHalfWidthMm=18,
            carrierCenterYmm=-323,
            conservativeRearXmm=reverse_extent,
            diagonalProtectionMarginMm=(intercept - (-reverse_extent + 341))
            / math.sqrt(2),
        ),
        battery=dict(
            receivedCaseBoundsMm=battery,
            studsAndBoots="Separate open receiving and installation test",
            roseBeforeSideExitMm=250,
            lowestSideExitZmm=battery[2] + 250,
            verticalGapToBumperWhenExitingMm=battery[2] + 250 - 181.5,
            minimumHorizontalReserveToBatteryDuringInitialRiseMm=battery[0]
            - max(precise(facet(19.05, side))[3] for side in (-1, 1)),
        ),
        frameMembers="Existing formed rails and eight rear weld articles require new end cuts and attachment drawings. All other installed geometry retains the source registration.",
    )
    (HERE / "fit.json").write_bytes((json.dumps(output, indent=2) + "\n").encode())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("supplier_cache", type=Path)
    run(parser.parse_args().supplier_cache)
