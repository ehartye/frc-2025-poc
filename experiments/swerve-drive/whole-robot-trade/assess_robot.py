"""Compare the registered complete robot layouts and size the purchased front hoist."""

import hashlib
import csv
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
INPUTS = {
    "frontField": "cad/shallow-cage/front-clearance.json",
    "rearField": "cad/shallow-cage/rear-clearance.json",
    "frontInternal": "cad/shallow-cage/installed.json",
    "rearInternal": "cad/shallow-cage/rear-stroke.json",
    "frontClock": "cad/shallow-cage/lift.json",
    "contact": "cad/shallow-cage/contact.json",
    "purchasedDrive": "cad/assembly-engineering/jack-transmission.json",
    "selectedMass": "cad/assembly-engineering/receiver-service/issue/mass.json",
    "fieldMassParts": "experiments/swerve-drive/mass-closure/recipe.json",
    "interference": "cad/assembly-engineering/receiver-service/carrier-motor.json",
    "signedPiece": "experiments/swerve-drive/cam-scoring/results.json",
    "reservedPark": "experiments/swerve-drive/motion/fallback-match.json",
}


def write(path, data):
    path.write_bytes((json.dumps(data, indent=2, ensure_ascii=False) + "\n").encode())


def rig(mass, share, dynamic, eff, bus, drive, cage):
    # Independent per-screw challenge with a positive thrust ratio and an ideal
    # upper bound for torque after power removal. A free/released pawl has zero
    # credited locking torque irrespective of the motor's energized torque.
    load = mass * 9.80665 * dynamic * share
    lead = 0.003175
    ratio = 68 / 13
    gear_eff = 0.75
    screw = load * lead / (math.tau * eff)
    motor_a = 1.4 + screw / (ratio * gear_eff * (0.97 / 100))
    rpm = 0.04 / lead * 60 * ratio
    req = (12 - 0.12 * 1.4) * rpm / 11000 + 0.12 * motor_a
    contact = cage["contact"]
    return {
        "operatingMassKg": mass,
        "heavierScrewShare": share,
        "loadFactor": dynamic,
        "screwEfficiency": eff,
        "terminalV": bus,
        "heavierScrewForceN": load,
        "nutRatedLoadToDemand": drive["nutOperatingN"] / load,
        "screwTorqueNm": screw,
        "motorAAtFortyMmPerS": motor_a,
        "requiredMotorV": req,
        "voltageHeadroomV": bus - req,
        "idealReverseDemandNm": load * lead / math.tau,
        "unlatchedHoldingMarginNm": -load * lead / math.tau,
        "requiredUpperContactForceN": contact["totalUpperShoeReactionN"]
        * mass
        / cage["massKg"],
        "requiredLowerContactForceN": contact["lowerFootCompressionN"]
        * mass
        / cage["massKg"],
        "certifiedFieldSupportN": None,
        "qualifiedPawlTorqueNm": None,
    }


def report():
    rows = {k: json.loads((ROOT / v).read_bytes()) for k, v in INPUTS.items()}
    source = {
        k: {"path": v, "sha256": hashlib.sha256((ROOT / v).read_bytes()).hexdigest()}
        for k, v in INPUTS.items()
    }
    field = {}
    for layout in ("front", "rear"):
        actual = rows[layout + "Field"]
        assembly = rows[layout + "Internal"]
        field[layout] = {}
        for side in ("red", "blue"):
            nominal = "frontApproach" if layout == "front" else "positiveSideApproach"
            for case, approach in [
                ("design-entry", nominal),
                ("opposite-side-entry", "negativeSideApproach"),
            ]:
                view = actual["cases"][side]
                adverse = view[approach]["adverse"]
                lift = view["suspendedTravel"]["adverse"]
                field[layout][side + "-" + case] = {
                    "runId": actual["robotSha256"]
                    + "-"
                    + layout
                    + "-"
                    + side
                    + "-"
                    + approach,
                    "concept": layout + "-jack-swerve",
                    "case": side + "-" + case,
                    "approach": approach,
                    "cagePoseObstacles": len(adverse),
                    "suspendedTravelObstacles": len(lift),
                    "internalSweepInterferences": len(
                        assembly["materialInterferences"]
                    ),
                    "largestKnownFieldPenetrationMm3": max(
                        (v.get("volumeMm3", 0.0) for v in adverse + lift), default=0.0
                    ),
                    "addedClimbInspectionUpperKg": assembly[
                        "accountedUpperIncrementKg"
                    ],
                }
    m = rows["selectedMass"]
    operating = rows["fieldMassParts"]["construction"]
    issue = ROOT / "cad/assembly-engineering/receiver-service/issue"
    for k, name in (
        ("stocks", "material-register.csv"),
        ("articles", "article-register.csv"),
    ):
        path = issue / name
        source[k] = {
            "path": path.relative_to(ROOT).as_posix(),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        }
    with (issue / "material-register.csv").open(encoding="utf8") as f:
        bumper_rows = [r for r in csv.DictReader(f) if r["scope"] == "removable bumper"]
    with (issue / "article-register.csv").open(encoding="utf8") as f:
        battery = float(
            next(
                r["upperKg"]
                for r in csv.DictReader(f)
                if r["instance"] == "np18-battery"
            )
        )
    assert len(bumper_rows) == 52
    bumpers = sum(float(r["inspectionStockUpperKg"]) for r in bumper_rows)
    empty = m["inspectionUpperKg"] + battery + bumpers
    controls = {}
    for name, mass, share, dynamic, eff, v in [
        ("operating-split", empty, 0.65, 1.5, 0.32, 9.5),
        (
            "full-extra-reserve",
            m["additionalFullReserveUpperKg"] + battery + bumpers,
            0.65,
            1.5,
            0.32,
            9.5,
        ),
        (
            "historical-bumper-cap",
            m["inspectionUpperKg"] + battery + operating["bumperUpperKg"],
            0.65,
            1.5,
            0.32,
            9.5,
        ),
        ("74kg-source-control", 74.0, 0.65, 1.5, 0.32, 9.5),
        ("single-loaded-screw", empty, 1.0, 1.5, 0.32, 9.5),
        ("binding-low-voltage", empty, 1.0, 1.5, 0.20, 8.5),
    ]:
        controls[name] = rig(
            mass, share, dynamic, eff, v, rows["purchasedDrive"], rows["frontClock"]
        )
    assert (
        abs(
            controls["74kg-source-control"]["requiredMotorV"]
            - rows["purchasedDrive"]["driveTerminalV"]
        )
        < 1e-7
    )
    return {
        "sources": source,
        "layoutCases": field,
        "currentFrontHoist": controls,
        "batteryKg": battery,
        "detachableBumperUpperKg": bumpers,
        "conditions": "Front and rear studies use each complete original assembly and the same official field. Current hoist uses the actual purchased 3.175 mm lead plus 68/13 reduction. The 55 mm field-contact moment arm and eight-CG-corner gravity equilibrium are inherited conditional front-clamp assumptions. A single loaded screw is a fault load requirement; with the other screw disengaged the guide must also resist racking. The calculations do not establish field or pawl capacity, working CG, impact dynamics, or a new match score.",
    }


if __name__ == "__main__":
    write(HERE / "robot-cases.json", report())
