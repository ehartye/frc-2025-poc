"""Reconcile the selected 552 lower with the three kickoff inspection categories and carried field mass."""
import argparse
import hashlib
import json
from pathlib import Path
import sys


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
LB = .45359237


def load(path):
    return json.loads(path.read_bytes())


def run(package):
    sys.path.insert(0, str(package / "scripts"))
    from fusion_authority import register_path, verify
    from component_shape import volume
    from showcase_contacts import load_solids

    register = register_path(ROOT)
    lineage = register.parent / "issue-005/lineage.json"
    authority = verify(ROOT, register, lineage)
    solids, sheets = load_solids(lineage.with_name("assembly.step"))
    if len(solids) != 552 or sheets:
        raise ValueError("Require the complete installed 552 issue")
    primary = load(HERE / "crossframe-budget.json")["studies"]["pocketedAluminum"]["items"]
    def allocated(starts):
        return sum(r["kg"] for r in primary if r["name"].startswith(starts))
    original_rails = sum(r["kg"] for r in primary if "socket rail" in r["name"])
    actual_rails = {name:volume(s) * 2700e-9 for name,s in solids.items() if "socket rail [" in name}
    if len(actual_rails) != 4:
        raise ValueError("Missing selected rail in weight calculation")
    four = load(HERE / "terminal-mass-clock.json")
    total_rails = sum(actual_rails.values())
    credit = original_rails - total_rails
    ring = allocated("miter-ring")
    catalog_modules = allocated("SDS NEO")
    electronics = allocated("non-module robot electronics")
    upper = allocated("protected pan")
    allowance = allocated("miscellaneous build")
    bumper = allocated("removable bumper")
    battery = allocated("battery and")
    pipe = allocated("one CORAL")
    courses = {}
    for name, row in four["clock"]["cases"].items():
        field = row["fieldKg"] - credit
        one_config = field - pipe - battery - bumper
        with_bumper = one_config + bumper
        courses[name]=dict(fieldKg=field, carriedPipeKg=pipe, removableOneColorBumperKg=bumper,
                           excludedBatteryAndLimitedBatterySideLeadsKg=battery, fixedRobotKg=one_config,
                           R103MarginKg=115*LB-one_config, withBumpersKg=with_bumper,
                           R408MarginKg=135*LB-with_bumper, I103AdditionalMechanismMarginKg=150*LB-one_config)
    base = courses["low-line-budget"]["fieldKg"]
    structural = base - (ring + catalog_modules + total_rails + four["fourCartridgeKg"] +
                         electronics + upper + allowance + bumper + battery + pipe)
    gear = load(ROOT / "artifacts/red2-integrated/pan-mechanics.json")
    return dict(authority=authority, selectedStepSolids=len(solids), railStockDensityKgM3=2700,
                fourActualDrilledRailsKg=actual_rails, originalRailsAllocationKg=original_rails,
                railDifferenceKg=credit, currentCohortsKg=dict(catalogFourSDSModules=catalog_modules,
                measuredFourCartridgeVolumes=four["fourCartridgeKg"], measuredFourDrilledRails=total_rails,
                plannedSixMiteredGrossTubes=ring, remainingPlannedCrossframeAndInterfaceAllowances=structural,
                plannedElectricalAssembly=electronics, plannedUpperAndDrive=upper,
                unspecifiedInstalledHardwareAndFinish=allowance, removableBumpersOneColor=bumper,
                batteryAndBatterySideConnector=battery, admittedGamePiece=pipe),
                upperReferenceOnly=dict(historicMovingShopStockKg=gear["movingShopStockKg"],
                historicFixedShopStockKg=gear["fixedShopStockKg"], assignedCurrentUpperKg=upper),
                electricalReferenceOnly=dict(eightSparkMaxControllersKg=8*.1133,oneRevPDHKg=.517,
                remainderWithinSevenKg=electronics-8*.1133-.517,
                scope="Budget remainder includes roboRIO, radio, breaker, main cables, branch extensions, sensing, harness and mounting. Generic component mass or a different controller variant cannot close this remainder."),
                ruleBasis=dict(edition="2025 kickoff V0",pages=[76,86,112],rules=["R103","R408","I103"],
                source=load(ROOT/"kickoff/rules-v0.json")["manual"],
                batteryException="Battery-side connector, lugs, fasteners, insulation and at most 12 inches per leg. Robot-side leads and permanently attached bumper mounts remain in R103."),
                cases=courses,
                scope="Catalog module weight is conditional on ordered components, including tire, wiring and shipped plate. Metal masses use measured CAD volumes and commissioned densities. Planning cases include every outstanding allocation; they are neither scale measurements nor complete issued-robot masses. The 2 kg high case spans 1.25 kg additional joints, 0.5 kg upper and 0.25 kg loom. I103 inventory must include every interchangeable mechanism presented for inspection, as applicable.")


if __name__ == "__main__":
    p=argparse.ArgumentParser()
    p.add_argument("package",type=Path)
    result=run(p.parse_args().package)
    (HERE/"inspection-mass-552.json").write_text(json.dumps(result,indent=2)+"\n")
    print(result["currentCohortsKg"], result["cases"])
