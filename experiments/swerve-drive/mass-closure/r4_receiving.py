"""Bind the fixed-height inspection budget to R4 and screen alternate incoming masses."""
import argparse
import itertools
import json
import math
from pathlib import Path
import subprocess
import sys

from inspection_recipe import wire_budget
from surrendered_stock import read, sha, same

ROOT = Path(__file__).resolve().parents[3]
SWERVE = ROOT / "experiments/swerve-drive"
R4 = ROOT / "cad/full-swerve-r4"
CORNERS = ("FL", "FR", "RL", "RR")
WORKING_RESERVE_KG = 0.5  # Project planning target beyond the formal inspection limit.


def margin_case(limit, original, original_modules, original_wire, modules, wire, extra=0):
    if len(modules) != 4 or not all(math.isfinite(x) and x > 0 for x in modules):
        raise ValueError("Supply four positive populated-module masses")
    if not all(math.isfinite(x) and x >= 0 for x in (wire, extra)):
        raise ValueError("Invalid installed harness or additional mass")
    upper = original - original_modules - original_wire + sum(modules) + wire + extra
    return {"populatedModulesKg": modules, "fullHarnessUpperKg": wire,
            "unbudgetedExtraKg": extra, "r103UpperKg": upper, "r103MarginKg": limit - upper}


def tail_span(bounds, point):
    """Euclidean distances to nearest and furthest points of the placement box."""
    nearest = [min(max(x, low), high) for x, low, high in zip(point, bounds[:3], bounds[3:])]
    corners = itertools.product(*zip(bounds[:3], bounds[3:]))
    return math.dist(point, nearest), max(math.dist(point, vertex) for vertex in corners)


def inspect(map_file, plugin, archive):
    sys.path.insert(0, str(ROOT / "cad/full-swerve-r3"))
    import build as assembly
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    native = read(R4 / "native-reopen.json")
    drawing = R4 / "full-robot.step"
    appearance_file = R4 / "appearances.json"
    if sha(drawing) != native["source"]["publicStepSha256"] or sha(appearance_file) != native["source"]["publicRegisterSha256"]:
        raise ValueError("Selected public geometry differs from the native reopen")
    if archive and sha(archive) != native["archive"]["sha256"]:
        raise ValueError("Different student Fusion archive")
    registered = {row["id"]: row for row in read(appearance_file)["solids"]}
    shapes = assembly.named(drawing)
    if len(shapes) != 975 or shapes.keys() != registered.keys():
        raise ValueError("Different R4 body inventory")
    geometry = native["originalGeometry"]
    if geometry["missing"] or geometry["discrepancies"] or geometry["duplicates"] or geometry["visible"] != 948:
        raise ValueError("Native placement record incomplete")
    catalog = read(SWERVE / "packaging/module.json")
    for corner in CORNERS:
        center = native["supplierModules"]["issuedWheelCentersMm"][corner]
        datum = catalog["modules"][corner]
        expected = [datum["xM"] * 1000, datum["yM"] * 1000, catalog["wheelDiameterM"] * 500]
        if math.dist(center, expected) > 0.01 or native["rootBodyCounts"][corner + " - REV MAXSwerve"] != 274:
            raise ValueError("Installed corner differs from the registered wheel datum")
    if native["supplierModules"]["changedOrMissing"]:
        raise ValueError("Incomplete native supplier occurrences")

    recipe = read(SWERVE / "mass-closure/recipe.json")
    for relative, digest in recipe["sourceEvidence"].items():
        if sha(ROOT / relative) != digest:
            raise ValueError(f"Inspection input changed: {relative}")
    matched = recipe["matchedInstalledChecker"]
    if matched["installedCount"] != 944 or any(matched["coverage"].values()):
        raise ValueError("Different inspection roster")
    accounting = None
    if map_file:
        checked = json.loads(subprocess.check_output(["node", str(plugin / "scripts/assembly-accounting.mjs"), str(map_file)]))
        ids = checked["make"] + checked["undecided"] + [i for row in checked["buy"] for i in row["instances"]]
        module_cores = {f"swerve/{corner}-module-core" for corner in CORNERS}
        controllers = {f"swerve/{corner}-steer-controller" for corner in CORNERS}
        controllers.update({"drive-controller-L", "drive-controller-R", "swerve/RL-drive-controller", "swerve/RR-drive-controller"})
        if not (module_cores | controllers).issubset(ids):
            raise ValueError("Missing installed module core or external motor controller")
        defects = [checked["coverage"][key] for key in matched["coverage"]]
        if len(set(ids)) != 944 or any(defects) or read(map_file)["selected"] != matched["frozenSelectedChanges"]:
            raise ValueError("A different robot is selected")
        for value, expected in zip(checked["mass"]["representedRangeKg"], matched["representedRangeKg"]):
            same(value, expected)
        accounting = {"mapSha256": sha(map_file), "physicalIds": len(set(ids))}

    stock = recipe["originalMaterial"]
    retired = set(stock["possibleWinchResidue"]) | set(stock["formerOuterClampVolumesMm3"])
    if len(retired) != 61 or retired & shapes.keys():
        raise ValueError("A retired physical identifier is still represented")
    changed = recipe["rearElectronics"]["changedNativeBodies"]
    for old_id, current_id in changed.items():
        if old_id.split("/")[-1] in shapes or current_id not in shapes:
            raise ValueError("Rear repair predecessor/successor mismatch")
    for name in ("support-machined-crossbar", "guide-L-machined-mast", "guide-R-machined-mast"):
        if name not in shapes:
            raise ValueError("An installed hoist support is missing")
    sheet = {}
    for name, key in (("r6-polymer-facing","retainedFaceVolumeMm3"),("r6-receiver-insert","receiverInsertVolumeMm3")):
        props = GProp_GProps()
        BRepGProp.VolumeProperties_s(shapes[name], props)
        sheet[name] = props.Mass()
        same(sheet[name], stock[key], 0.01)
    printed = sum(row["source"] == "cad/full-swerve-r4/markings.py" for row in registered.values())
    if printed != 16:
        raise ValueError("Expected the numbered red covers")
    limits = recipe["construction"]
    harness = read(SWERVE / "power/harness.json")
    routes = read(SWERVE / "power/paths.json")["groupedSupplyTargets"]
    issued = limits["supplyExtensionPerConductorMetres"]
    issued_wire, slack = wire_budget(harness, routes, issued)
    same(issued_wire, limits["issuedWireUpperKg"])
    uniform, _ = wire_budget(harness, routes, dict.fromkeys(CORNERS, 1.0))

    def bounds(name):
        actual = assembly.cad.bounds(shapes[name])
        for value, reference in zip(actual, registered[name]["boundsMm"]):
            same(value, reference, 0.01)
        return actual

    tails = []
    for corner in CORNERS:
        vertices = routes[corner]["polylineVerticesMm"]
        hub = tail_span(bounds("rev-pdh"), vertices[0])
        for job in ("drive","steer"):
            controller = tail_span(bounds(corner + "-" + job + "-controller"), vertices[-1])
            direct = [a + b for a, b in zip(hub, controller)]
            tails.append({"corner":corner,"circuit":job,"cutMm":issued[corner]*1000,
                          "commonPathMm":routes[corner]["lengthMm"],"directTailExtentMm":direct,
                          "remainingAtFurthestBoxPointsMm":slack[corner]*1000 - direct[1]})

    cap = matched["ruleLimitKg"]
    original = matched["representedRangeKg"][1]
    population = read(SWERVE / "power/mass.json")["newReceivingBands"]["fourCompletePopulatedModulesKg"]
    catalog_kg = 1.720
    same(population[1], 4 * catalog_kg * 1.05)
    upper = population[1] / 4
    def case(masses, wire):
        return margin_case(cap, original, population[1], issued_wire, masses, wire)
    cases = {"issued":case([upper]*4, issued_wire),
             "catalogNominals":case([catalog_kg]*4, issued_wire),
             "oneMetreAtReceivingUpper":case([upper]*4, uniform),
             "oneMetreAtCatalogNominals":case([catalog_kg]*4, uniform)}
    same(cases["oneMetreAtReceivingUpper"]["r103UpperKg"], recipe["joinedCases"]["uniformOneMetre"]["r103UpperKg"])
    budgets = {}
    for name, wire in (("issuedCuts",issued_wire),("oneMetreCuts",uniform)):
        mass_available = population[1] + cap - original - (wire - issued_wire)
        budgets[name] = {"forRuleKg":mass_available,"forWorkingReserveKg":mass_available-WORKING_RESERVE_KG,
                         "equalCornerForRuleKg":mass_available/4,
                         "equalCornerForWorkingReserveKg":(mass_available-WORKING_RESERVE_KG)/4}
    return {"configuration":native["configuration"],"sourceStepSha256":sha(drawing),
            "nativeArchiveReferenceSha256":native["archive"]["sha256"],"archiveBytesVerified":bool(archive),
            "budgetReceiptSha256":sha(SWERVE/"mass-closure/recipe.json"),"accounting":accounting,
            "nativeSupplierBodiesPerCorner":274,"publicBodies":len(shapes),"appearanceOnlyDigits":printed,
            "retiredNamedBodiesAbsent":len(retired),"verifiedRearReplacementPairs":changed,
            "installedShelfMm3":sheet,"r408BumperUpperKg":limits["bumperUpperKg"],
            "originalUnidentifiedKg":limits["originalUnidentifiedKg"],"workingReserveKg":WORKING_RESERVE_KG,
            "case":cases,"populatedReceivingBudget":budgets,"tailSensitivity":tails,
            "inspectionMeasurementKg":None}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, help="Existing generated 944-identity selected.json")
    parser.add_argument("--plugin", type=Path, help="Agent FRC root for the optional assembly-map check")
    parser.add_argument("--archive", type=Path, help="Authorized local R4 F3D to independently hash")
    args = parser.parse_args()
    if bool(args.map) != bool(args.plugin):
        parser.error("Use --map and --plugin together")
    print(json.dumps(inspect(args.map, args.plugin, args.archive), indent=2, allow_nan=False))
