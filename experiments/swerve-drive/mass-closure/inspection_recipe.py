"""Evaluate the fixed-height four-module drawing and its four power-harness cut lists."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess

from surrendered_stock import evidence, read, same, sha
from rear_exchange import register
from final_map import materialize

# Installed conductor length, including both leads' individual service/termination
# slack. Each of the drive and steer supply circuits has a separate positive/return.
CUT_METRES = {"FL": 0.80, "FR": 0.95, "RL": 0.55, "RR": 0.90}


def wire_budget(harness, routes, cuts):
    runs = harness["leadAllowances"]
    if set(cuts) != set(routes) or any(
        not math.isfinite(length) or length <= 0 for length in cuts.values()
    ):
        raise ValueError("Missing or invalid corner length")
    slack = {
        corner: length - routes[corner]["lengthMm"] / 1000
        for corner, length in cuts.items()
    }
    if any(r["lowerGapMm"] <= 0 for r in routes.values()) or min(slack.values()) < 0.42:
        raise ValueError(
            "Power leads cannot cover the registered path and service allowance"
        )
    mass = harness["totalNoWireRemovalCreditUpperKg"]
    for id, gauge in (("driveSupply", 10), ("steerSupply", 14)):
        row = runs[id]
        if row["awg"] != gauge or row["conductors"] != 8:
            raise ValueError("Wrong conductor gauge/count")
        if max(cuts.values()) > row["metresEach"]:
            raise ValueError("Longer than the electrical model's corresponding branch")
        kg_per_m = row["maximumKg"] / (row["conductors"] * row["metresEach"])
        mass += (
            2 * sum(cuts.values()) - row["conductors"] * row["metresEach"]
        ) * kg_per_m
    return mass, slack


def evaluate(project, plugin, source_file, maps):
    hist = project / "experiments/six-wheel"
    swerve = project / "experiments/swerve-drive"
    power = swerve / "power"
    source = read(source_file)
    selected = read(power / "installed.json")
    result = json.loads(
        subprocess.check_output(
            ["node", str(plugin / "scripts/assembly-accounting.mjs"), str(source_file)]
        )
    )
    if any(
        result["coverage"][k]
        for k in ("missing", "duplicates", "orphanIncluded", "unattached")
    ):
        raise ValueError("Unreconciled physical inventory")
    ids = set(
        result["make"]
        + result["undecided"]
        + [key for row in result["buy"] for key in row["instances"]]
    )
    if (
        len(ids) != selected["installedPhysicalCount"]
        or not {
            "support-machined-crossbar",
            "match-drilled/guide-L-machined-mast",
            "match-drilled/guide-R-machined-mast",
            "shelf-structure/r6-polymer-facing",
            "shelf-structure/r6-receiver-insert",
            "shelf-structure/absorber-0",
            "shelf-structure/absorber-1",
        }
        <= ids
    ):
        raise ValueError("Different retained structure")
    sources = {s["id"]: s for s in source["sources"]}

    def linked(key):
        ref = sources[key]
        path = source_file.parent / ref["file"]
        if sha(path) != ref["sha256"]:
            raise ValueError("Changed registered mass source " + key)
        return read(path)

    if (
        source["selected"][-1] != "four-module-swerve"
        or linked("swerve-rebill")["matched"] != selected["matched"]
    ):
        raise ValueError("Expected the source-reconciled four-module configuration")
    native = evidence(project, source)
    if ids & (
        set(native["possibleWinchResidue"]) | set(native["formerOuterClampVolumesMm3"])
    ):
        raise ValueError("Retired articles remain installed")
    for file, digest in selected["sourceFiles"].items():
        path = power / file if (power / file).is_file() else swerve / "packaging" / file
        if sha(path) != digest:
            raise ValueError("Different four-module build " + file)
    baseline = source["mass"]["baseline"]
    rebill = linked("primary-hoist-rebill")
    if (
        baseline["pointer"] != "/baselineKg/allStocksAndFourteenSetsKg"
        or baseline["rangeKg"] != rebill["baselineKg"]["allStocksAndFourteenSetsKg"]
    ):
        raise ValueError("Hoist residual already credited or different baseline")
    if rebill["stockSource"] != sha(hist / "winchless-bill/stock.json"):
        raise ValueError("Changed winch stock")
    winch = baseline["rangeKg"][1] - rebill["baselineKg"]["issuedDrawingKg"][1]
    same(winch, native["allPossibleWinchResidueKg"])
    pe = native["densityMatchedR6CreditKg"]
    sstock = linked("shelf-insert-stock")
    same(pe, sstock["noCutCreditKg"][1] - sstock["densityMatchedRemovalKg"][1])
    matched = selected["matched"]
    cap = result["mass"]["limitKg"]
    r408 = matched["r408LimitKg"]
    if matched["currentRemainderRangeKg"] != native["unknownRetainedKg"] or native[
        "unknownRetainedKg"
    ] != [2, 5]:
        raise ValueError("Unidentified material allowance changed")
    bare_no_wire = (
        matched["conservativeDressedRangeKg"][1] - matched["longWireNoOldCreditUpperKg"]
    )
    clamps = native["originalShopOuterClampOnlyKg"]
    harness = read(power / "harness.json")
    routes = read(power / "paths.json")["groupedSupplyTargets"]
    # Witness centers on the straight run, independent of the wire ordering
    # at connectors. The two small conductors belong to the routed CAN pair.
    witness = [
        (10, -2.8, 1.6),
        (10, 2.8, 1.6),
        (14, -2.2, -3.3),
        (14, 2.2, -3.3),
        (22, -6.0, -1.1),
        (22, 6.0, -1.1),
    ]
    radii = [
        harness["maxInsulatedDiameterMm"][str(gauge)] / 2 for gauge, _, _ in witness
    ]
    gaps = [
        math.dist(a[1:], b[1:]) - radii[i] - radii[j]
        for i, a in enumerate(witness)
        for j, b in enumerate(witness[:i])
    ]
    bundle = max(math.hypot(x, y) + radius for (_, x, y), radius in zip(witness, radii))
    sleeve_space = min(row["radiusMm"] for row in routes.values()) - bundle
    if min(gaps) < 0 or sleeve_space < 0:
        raise ValueError("Conductors no longer fit the common route envelope")
    long = harness["totalNoWireRemovalCreditUpperKg"]
    short = matched["shorterEndpointCappedWireUpperKg"]
    issued, slack = wire_budget(harness, routes, CUT_METRES)
    boards = {}

    def add(label, wire, hoist, cut, outer):
        upper = bare_no_wire + wire - hoist - cut - outer
        boards[label] = dict(
            r103UpperKg=upper,
            r103MarginKg=cap - upper,
            r408UpperKg=upper + 7.5,
            r408MarginKg=r408 - upper - 7.5,
            wireUpperKg=wire,
            omittedHoistResidueKg=hoist,
            matchedShelfCreditKg=cut,
            retiredOuterClampCreditKg=outer,
        )

    add("priorLong", long, 0, 0, 0)
    add("priorShort", short, 0, 0, 0)
    add("drawingSelected", issued, winch, pe, clamps)
    add(
        "uniformOneMetre",
        wire_budget(harness, routes, dict.fromkeys(routes, 1.0))[0],
        winch,
        pe,
        clamps,
    )
    add(
        "uniformOnePointTwoMetres",
        wire_budget(harness, routes, dict.fromkeys(routes, 1.2))[0],
        winch,
        pe,
        clamps,
    )
    add("longFullSurrender", long, winch, pe, clamps)
    add("priorShortFullSurrender", short, winch, pe, clamps)
    add("drawingsWithNoShelfCredit", issued, winch, 0, clamps)
    add("drawingsWithAllWinchResidue", issued, 0, pe, clamps)
    add(
        "drawingsWithAllSevenBlanks",
        issued,
        winch - native["legacyWinchBlankUpperKg"],
        pe,
        clamps,
    )
    add("drawingsWithoutOldClampCorrection", issued, winch, pe, 0)
    same(boards["priorShort"]["r103UpperKg"], matched["shorterDressedUpperKg"])
    same(boards["priorLong"]["r103UpperKg"], matched["conservativeDressedRangeKg"][1])
    chosen = boards["drawingSelected"]
    maximum = cap - bare_no_wire + winch + pe + clamps
    # Uniform lengths for 8 conductors per gauge; derive slope with full local branches fixed.
    kg_per_metres = (
        wire_budget(harness, routes, dict.fromkeys(routes, 1.1))[0]
        - wire_budget(harness, routes, dict.fromkeys(routes, 1.0))[0]
    ) / 0.1
    receipt = dict(
        assemblyCheckerVersion=read(plugin / "plugin.json")["version"],
        registeredRosterSha256=hashlib.sha256(
            "\n".join(sorted(ids)).encode()
        ).hexdigest(),
        originalSourcePins={
            key: sources[key]["sha256"]
            for key in ("primary-hoist-rebill", "shelf-insert-stock", "swerve-rebill")
        },
        sourceEvidence={
            **native.pop("sourceSha256"),
            **{
                (power / file).relative_to(project).as_posix(): sha(power / file)
                for file in ("installed.json", "harness.json", "paths.json")
            },
        },
        originalMaterial=native,
        parentPhysicalRosterCount=len(ids),
        registeredCheckerIndependentParentRangeKg=result["mass"]["representedRangeKg"],
        activePhysicalCoverage={
            k: result["coverage"][k]
            for k in ("missing", "duplicates", "orphanIncluded", "unattached")
        },
        construction=dict(
            supplyExtensionPerConductorMetres=CUT_METRES,
            allowanceBeyondCheckedCenterlineMetres=slack,
            bundleWitnessAwgXYmm=witness,
            minimumInterJacketGapMm=min(gaps),
            straightRunMaximumRadialSheathThicknessMm=sleeve_space,
            referenceR103LimitKg=cap,
            referenceR408LimitKg=r408,
            batteryKg=6.17,
            bumperUpperKg=7.5,
            issuedWireUpperKg=issued,
            maximumWireUpperKg=maximum,
            originalUnidentifiedKg=native["unknownRetainedKg"],
            maximumUniformSupplyMetres=1
            + boards["uniformOneMetre"]["r103MarginKg"] / kg_per_metres,
        ),
        cases=boards,
        fieldEmptyUpperKg=chosen["r103UpperKg"] + 6.17 + 7.50,
        fieldLoadedUpperKg=chosen["r103UpperKg"] + 6.17 + 7.50 + 0.816466266,
        measuredInspectionKg=None,
    )
    if source_file.parent.resolve() == maps.resolve():
        raise ValueError(
            "Keep output maps in a separate directory from the original inventory"
        )
    rear, after, rear_file = register(project, source_file, ids, maps)
    receipt["rearElectronics"] = rear
    joined = {}
    for name, row in boards.items():
        r103 = row["r103UpperKg"] + rear["upperAllowanceKg"]
        joined[name] = dict(
            r103UpperKg=r103,
            r103MarginKg=cap - r103,
            r408UpperKg=r103 + 7.5,
            r408MarginKg=r408 - r103 - 7.5,
            wireUpperKg=row["wireUpperKg"],
        )
    receipt["joinedCases"] = joined
    receipt["joinedUniformSupplyLimitMetres"] = (
        1 + joined["uniformOneMetre"]["r103MarginKg"] / kg_per_metres
    )
    receipt["joinedFieldEmptyUpperKg"] = (
        receipt["fieldEmptyUpperKg"] + rear["upperAllowanceKg"]
    )
    receipt["joinedFieldLoadedUpperKg"] = (
        receipt["fieldLoadedUpperKg"] + rear["upperAllowanceKg"]
    )
    receipt["sourceEvidence"][
        "experiments/swerve-drive/packaging/rear-electronics.json"
    ] = rear["sourceReceiptSha256"]
    receipt["sourceEvidence"][
        "experiments/swerve-drive/packaging/rear-electronics.step"
    ] = rear["stepSha256"]
    receipt["matchedInstalledChecker"] = materialize(
        project, plugin, rear_file, after, native, receipt
    )
    return receipt


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(required=True, dest="command")
    replay = commands.add_parser(
        "replay", help="Reopen registered CAD, assembly and mass sources"
    )
    for name in ("project", "plugin", "input_map", "output_maps", "receipt"):
        replay.add_argument(name, type=Path)
    cuts = commands.add_parser(
        "cuts", help="Check supply-length variants against the selected bill"
    )
    cuts.add_argument(
        "metres",
        type=float,
        nargs=4,
        metavar="M",
        help="Length per conductor in front-left, front-right, rear-left, rear-right order",
    )
    cuts.add_argument(
        "--extra-kg",
        type=float,
        default=0,
        help="Additional identified installed material",
    )
    args = parser.parse_args()
    if args.command == "cuts":
        here = Path(__file__).resolve().parent
        power = here.parent / "power"
        issued = read(here / "recipe.json")
        for name in ("paths.json", "harness.json"):
            if (
                sha(power / name)
                != issued["sourceEvidence"]["experiments/swerve-drive/power/" + name]
            ):
                parser.error("Registered power source changed: " + name)
        if not math.isfinite(args.extra_kg) or args.extra_kg < 0:
            parser.error("Extra mass must be finite and nonnegative")
        try:
            mass, slack = wire_budget(
                read(power / "harness.json"),
                read(power / "paths.json")["groupedSupplyTargets"],
                dict(zip(CUT_METRES, args.metres)),
            )
        except ValueError as error:
            parser.error(str(error))
        base = issued["joinedCases"]["drawingSelected"]
        delta = mass - base["wireUpperKg"] + args.extra_kg
        print(
            json.dumps(
                dict(
                    wireUpperKg=mass,
                    extraKg=args.extra_kg,
                    routeAllowanceMetres=slack,
                    r103UpperKg=base["r103UpperKg"] + delta,
                    r103MarginKg=base["r103MarginKg"] - delta,
                    r408MarginKg=base["r408MarginKg"] - delta,
                    measuredInspectionKg=None,
                ),
                indent=2,
            )
        )
        raise SystemExit(0)
    result = evaluate(
        args.project.resolve(),
        args.plugin.resolve(),
        args.input_map.resolve(),
        args.output_maps.resolve(),
    )
    args.receipt.write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8", newline="\n"
    )
