"""Reopen the purchased-article envelopes and compare every sensor block with the issued scene."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

from OCP.BRepExtrema import BRepExtrema_DistShapeShape

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "design/receiver-drives"))
from receiver_drives import cad, parts


def gap(first, second):
    q = BRepExtrema_DistShapeShape(first, second)
    q.Perform()
    if not q.IsDone():
        raise ValueError("Native distance solver failed")
    return round(q.Value(), 8)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check(scene):
    step = ROOT / "design/receiver-drives/grip-slide-place.step"
    model = parts()
    placed = cad.read_named(step, [{"id": name} for name in model])
    registered = json.loads(scene.read_bytes())
    result = {}
    for side in ("L", "R"):
        names = {
            key: f"grip-{side}-{key}"
            for key in (
                "idler-20T",
                "idler-shaft",
                "idler-magnet",
                "idler-angle-board",
                "sensor-standoff",
            )
        }
        bodies = {key: placed[name] for key, name in names.items()}
        inventory = {}
        for key, name in names.items():
            box = list(cad.bounds(bodies[key]))
            # The fixed-height place and stowed horizontal frames differ only by 370 mm.
            box[0] -= 370
            box[3] -= 370
            registered_box = registered["stowed"][name]
            if max(abs(a - b) for a, b in zip(box, registered_box)) > 1.0e-5:
                raise ValueError("Selected scene disagrees with source STEP: " + name)
            inventory[name] = dict(
                category=model[name][1], stowedMm=[round(v, 6) for v in box]
            )
        result[side] = dict(
            inventory=inventory,
            shaftToPulleyMm=gap(bodies["idler-shaft"], bodies["idler-20T"]),
            shaftToMagnetMm=gap(bodies["idler-shaft"], bodies["idler-magnet"]),
            magnetToBoardEnvelopeMm=gap(
                bodies["idler-magnet"], bodies["idler-angle-board"]
            ),
            standoffToBoardMm=gap(
                bodies["sensor-standoff"], bodies["idler-angle-board"]
            ),
        )
    sources = [
        step,
        ROOT / "design/receiver-drives/receiver_drives.py",
        ROOT / "design/electrical-inspection/circuits.json",
    ]
    return dict(
        sources={p.relative_to(ROOT).as_posix(): sha(p) for p in sources},
        sceneRegistrationSha256=sha(scene),
        sceneBodyCount=registered["registeredBodies"],
        sourceBodyCount=len(placed),
        grips=result,
        installedDieGapMm=None,
        verifiedShaftToPulleyAntirotation=False,
        installedAngleCalibration=None,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", type=Path)
    options = parser.parse_args()
    (HERE / "fit.json").write_text(
        json.dumps(check(options.scene), indent=2) + "\n", encoding="utf8", newline="\n"
    )
