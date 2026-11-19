"""Register stationary, moving and field backgrounds with the actual machined shelf."""

import argparse
import json
from pathlib import Path
import sys
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "shelf-structure"))
from background import installed_poses, scenes
from tray import digest


def run(artifacts, lamp, field, out):
    args = SimpleNamespace(
        assembly=ROOT / "selected-match/clearance.py",
        artifacts=artifacts,
        lamp=lamp,
        field=field,
        out=out,
        fixed_depth=0.0,
        stowed_robot_only=False,
    )
    scenes.render(args, installed_poses)
    import stroke

    stroke.run(args, installed_poses)
    endpoints = json.loads((out / "scenes.json").read_bytes())
    if endpoints["hardware"]["selectedPartsIncludingWire"] != 998:
        raise ValueError("Changed receiver assembly")
    return {
        "shelfStepSha256": digest(ROOT / "shelf-structure/stow-receiver.step"),
        "sceneRegisterSha256": digest(out / "scenes.json"),
        "sweepSha256": digest(out / "sweep.json"),
        "nativeBodies": 998,
        "rendered": endpoints["backgroundSha256"],
        "nominalFixedPupilOnly": True,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for key in ("artifacts", "lamp", "field", "out", "receipt"):
        p.add_argument(key, type=Path)
    a = p.parse_args()
    a.receipt.write_text(
        json.dumps(run(a.artifacts, a.lamp, a.field, a.out), indent=2) + "\n",
        encoding="utf8",
        newline="\n",
    )
