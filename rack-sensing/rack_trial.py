"""Replay the issued optical packets and shop rack plant with calibrated ADC endpoints."""
import argparse
import csv
import json
from pathlib import Path
import shutil
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "selected-drive"))
import rack

MODES = ("nominal", "cal90", "cal94", "cal98", "reverse94", "legacy-slip94", "slip94",
         "adverse90", "assist90", "modern-adverse90", "modern-assist98", "unscaled90",
         "unscaled98", "open94", "curvature94", "heavyDrag",
         "ramp90", "ramp94", "ramp98", "ramp-reverse94", "ramp-adverse90",
         "ramp-adverse94", "ramp-adverse98", "ramp-assist90", "ramp-curvature94",
         "ramp-slip94", "ramp-unscaled90", "ramp-unscaled98", "ramp-open94", "ramp-drag94",
         "ramp-phase-adverse90", "ramp-phase-adverse98", "ramp-drag-adverse94",
         "ramp-phase-slip94", "ramp-reverse-adverse94")


def configure(folder):
    rack.configure(folder)
    for name in ("CalibratedRack", "RackSignalTrial", "RackCalibrationCheck"):
        shutil.copy(HERE / (name + ".java"), folder)
    rewrite = rack.recovery.rewrite
    rewrite(folder, "Retreat", [
        ("private final double ramp;", "double ramp;"),
        ("private double integratedError", "double motionThreshold = 4*(.635/4096);\n    private double integratedError"),
        ("sense.potM() - initialPot >= 4*(.635/4096)", "sense.potM() - initialPot >= motionThreshold")])
    rewrite(folder, "IntegratedTest", [
        ("assertTrue(HAL.initialize(500,0));", "RackCalibrationCheck.verify();\n        assertTrue(HAL.initialize(500,0));"),
        ('List.of("nominal","bridge26","heavyDrag","base","extra50g")',
         "List.of(" + ",".join(json.dumps(m) for m in MODES) + ")"),
        ("double volts,previousMotor;", "double volts,previousMotor; RackSignalTrial signal;"),
        ("motor.extraMass=scenario", """if (!scenario.equals("nominal")) {
                signal = new RackSignalTrial(scenario);
                slide.motionThreshold = signal.threshold();
                if (scenario.startsWith("ramp")) slide.ramp = .035;
            }
            motor.extraMass=scenario"""),
        ('scenario.equals("heavyDrag")?15:0','(scenario.equals("heavyDrag") || scenario.contains("-drag"))?15:0'),
        ("double speed=(enc-previousMotor)/.005;", """if (signal != null) {
                        long elapsed = w - commandWall;
                        pot = signal.sensor(motor.x, elapsed);
                        if (signal.decoupled()) enc = Math.rint(Math.min(.0012, elapsed*.000063)/step)*step;
                    }
                    double speed=(enc-previousMotor)/.005;"""),
        ('motor.step(volts,mode.equals("rackJam") && piece.equals("coral-B"),true);',
         'motor.step(volts,signal != null && signal.decoupled(),true);')])


def run(images, output, jdk, gradle, receipts=None, observation_register=None):
    output.mkdir(parents=True, exist_ok=True)
    # The parent collects all path CSVs in its output. Do not admit earlier trials.
    if any(output.glob("path-*.csv")):
        raise ValueError("Use a fresh output directory so the raw register contains only this run")
    if (receipts is None) != (observation_register is None):
        raise ValueError("Supply both the optical receipts directory and its registered controller inputs")
    expected_register = observation_register or ROOT / "relieved-match/controller-register.json"
    expected_hash = rack.control.sha(expected_register)
    rack.control.run(output, jdk, gradle, output=output, source_cache=images, configure=configure,
                     source_receipts=receipts)
    reg_hash = rack.control.sha(output / "controller-register.json")
    if reg_hash != expected_hash:
        raise ValueError("Optical frames or pressure inputs differ from the issued controller register")
    with (output / "events.csv").open() as stream:
        events = list(csv.DictReader(stream))
    cases = {}
    for mode in MODES:
        cases[mode] = {}
        for piece in ("coral-A", "coral-B"):
            rows = [r for r in events if r["scenario"] == mode and r["piece"] == piece]
            case = {r["event"]: int(r["wall_ms"]) for r in rows if r["event"] in ("withdraw", "motion", "stow", "hold")}
            path = output / f"path-{mode}-{piece}.csv"
            if path.exists():
                with path.open() as stream:
                    samples = list(csv.DictReader(stream))
                case["traceSha256"] = rack.control.sha(path)
                case["maxRackTravelMm"] = round(1000 * max(float(row["withdrawn_m"]) for row in samples), 5)
                case["peakPhaseA"] = round(max(abs(float(row["phase_A"])) for row in samples), 5)
                case["peakVolts"] = round(max(abs(float(row["motor_V"])) for row in samples), 5)
            cases[mode][piece] = case
    source = rack.control.read(ROOT / "selected-drive/rack.json")
    for piece, case in cases["nominal"].items():
        if case["traceSha256"] != source["cases"]["nominal"][piece]["rawSHA256"]:
            raise ValueError("Original shop trial did not reproduce")
    receipt = {"sourceShop": rack.control.sha(ROOT / "selected-drive/rack.json"), "observationRegisterSha256": reg_hash,
               "hal": rack.control.read(output / "hal.json"), "cases": cases}
    (output / "results.json").write_text(json.dumps(receipt, indent=2) + "\n", encoding="utf8", newline="\n")
    return cases


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("images", "output", "jdk", "gradle"):
        parser.add_argument(name, type=Path)
    parser.add_argument("--receipts", type=Path)
    parser.add_argument("--observation-register", type=Path)
    args = parser.parse_args()
    cases = run(args.images.resolve(), args.output.resolve(), args.jdk.resolve(), args.gradle.resolve(),
                args.receipts, args.observation_register)
    for mode, pieces in cases.items():
        print(mode, [(p, r.get("motion"), r.get("stow"), r.get("hold"), r.get("maxRackTravelMm")) for p, r in pieces.items()])
