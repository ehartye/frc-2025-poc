"""Reopen signed station routes, compare the historic motor and check the installed exercises."""

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

from plant import SETTINGS, available_torque, historical_torque, motor_constants

HERE = Path(__file__).resolve().parent
CASES = ("plus", "minus", "heavy", "upper-minus", "blind", "late", "jam", "misaligned")


def sha(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def run(raw, plugin):
    examples = plugin / "examples/swerve"
    exercises = {}
    for filename in ("shaft_power.py", "loaded_motion.py"):
        source = examples / filename
        results = subprocess.check_output([sys.executable, str(source)])
        exercises[filename] = {"sha256": sha(source), "outcome": json.loads(results)}
    spec = importlib.util.spec_from_file_location(
        "plugin_shaft_power", examples / "shaft_power.py"
    )
    checker = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checker)
    points = {}
    coefficients = {}
    for axis in ("drive", "steer"):
        motor = SETTINGS[axis]
        resistance, constant, drag = motor_constants(motor)
        coefficients[axis] = dict(
            ohm=resistance,
            reciprocalSI=constant,
            dragNmPerRadS=drag,
            impliedStallNm=constant * motor["stallCurrentA"],
            dataSheetStallNm=motor["stallTorqueNm"],
        )
        for name, function in (
            ("historical", historical_torque),
            ("reciprocal", available_torque),
        ):
            rows = []
            for goal, rate, cap in (
                (0.2, 100.0, 9.5),
                (-0.2, 100.0, 9.5),
                (0.2, -100.0, 9.5),
                (-0.2, -100.0, 9.5),
                (0.0, 100.0, 0.0),
            ):
                torque, amp, volts = function(goal, rate, motor, cap)
                rows.append(
                    {
                        "demandNm": goal,
                        "radS": rate,
                        "capV": cap,
                        "actualNm": torque,
                        "phaseA": amp,
                        "volts": volts,
                        **checker.motor_port(volts, amp, rate, torque, resistance),
                    }
                )
            points[axis + "-" + name] = rows
    base = json.loads((HERE / "opening.json").read_bytes())["cases"]
    routes = {}
    for case in CASES:
        record = json.loads((raw / "reciprocal" / (case + ".json")).read_bytes())
        if sha(raw / "reciprocal" / (case + ".csv")) != record["csvSha256"]:
            raise ValueError("Different wheel trace for " + case)
        routes[case] = {
            k: v for k, v in record.items() if k not in ("geometry", "samples")
        }
        routes[case]["geometricMinMm"] = {
            k: record["geometry"][k]
            for k in ("minimumBarrierMm", "minimumPeerMm", "minimumStockMm")
        }
        routes[case]["historicalCsvSha256"] = base[case]["csvSha256"]
    preserved = sha(raw / "historical/plus.csv")
    if preserved != base["plus"]["csvSha256"]:
        raise ValueError("Historical wheel trace changed")
    return dict(
        schema="reefscape-swerve-energy/1",
        priorOpeningSha256=sha(HERE / "opening.json"),
        sourceSha256={
            p: sha(HERE / p)
            for p in (
                "model.json",
                "plant.py",
                "opening.py",
                "actual_supply.py",
                "../power/electrical.py",
                "../power/loads.py",
                "../packaging/module.json",
            )
        },
        pluginVersion=json.loads((plugin / ".codex-plugin/plugin.json").read_bytes())[
            "version"
        ],
        exercises=exercises,
        motorFit=coefficients,
        ports=points,
        historicalRebuiltCsvSha256=preserved,
        cases=routes,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for arg in ("raw", "plugin", "output"):
        parser.add_argument(arg, type=Path)
    a = parser.parse_args()
    a.output.write_text(
        json.dumps(run(a.raw, a.plugin), indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
