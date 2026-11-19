"""Render the native stowed receiver into the registered stationary stereo pair."""

import argparse
import json
import math
from pathlib import Path
import sys

from tray import ROOT, HERE, SHELF, FACE, TRAY, cad, digest
from installed import GAP, STATIONS
from damper_fit import cylinder, envelope

sys.path.insert(0, str(ROOT / "integrated-slide"))
from render import scenes, select


def installed_poses(model, artifacts, lamp):
    stages = select(model, artifacts, lamp)
    receipt = json.loads((HERE / "installed.json").read_bytes())
    step = HERE / "stow-receiver.step"
    if digest(step) != receipt["manufacturingSha256"]:
        raise ValueError("Different receiver exchange")
    bodies = cad.read_named(step, [{"id": key} for key in receipt["volumeMm3"]])
    for stroke, stage in zip((0.0, 370.0), stages):
        # Keep the old face name as a rendering alias to retain its nominal finish.
        stage[SHELF] = cad.translated(bodies[FACE], (stroke, 0, 0))
        stage[TRAY] = cad.translated(bodies[TRAY], (stroke, 0, 0))
        stage["slide-base-bridge"] = bodies["slide-base-bridge"]
        for index in range(3):
            key = f"observer-moving-pad-{index}"
            stage[key] = cad.translated(stage[key], (0, 0, -0.8))
        for side, (x, y) in enumerate(STATIONS):
            stage[f"stow-damper-{side}"] = envelope(x, y, 485 - GAP)
            for z in (430.0, 438.0):
                stage[f"stow-locknut-{side}-{z:g}"] = cylinder(
                    x, y, z, 10 / math.sqrt(3), 3
                )
    return stages


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("artifacts", "lamp", "field", "out", "receipt"):
        parser.add_argument(key, type=Path)
    args = parser.parse_args()
    args.assembly = ROOT / "selected-match/clearance.py"
    args.fixed_depth = 0.0
    args.stowed_robot_only = True
    scenes.render(args, installed_poses)
    register = json.loads((args.out / "station-background.json").read_bytes())
    for name in (
        "shelf-structure/stow-receiver.step",
        "shelf-structure/background.py",
        "integrated-slide/render.py",
        "sp1-mount/sp1-mount.step",
    ):
        register["sourceSha256"][name] = digest(ROOT / name)
    if register["hardware"]["selectedPartsIncludingWire"] != 998:
        raise ValueError("Expected the 991 source bodies plus seven net additions")
    args.receipt.write_text(
        json.dumps(register, indent=2) + "\n", encoding="utf8", newline="\n"
    )
