"""Register measured screw clocks at unchanged underhead datums for the editable port receiver."""
import json
import math
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent


def read(filename):
    return json.loads((HERE / filename).read_bytes())


def spin_about(head, axis, degrees):
    """World-space rigid transform around the directed screw centerline through its seated head."""
    direction = np.asarray(axis, dtype=float)
    if abs(np.linalg.norm(direction) - 1) > 1e-10:
        raise ValueError("Screw axis must be unit length")
    x, y, z = direction
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    angle = math.radians(degrees)
    rotation = np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * (cross @ cross)
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = head - rotation @ head
    return matrix


def main():
    purchased = read("port-purchase-placements.json")
    trial = read("port-504-seated-phase-inputs.json")
    fit = read("port-504-seated-comparison.json")
    if fit["stepSha256"] != trial["source"] or len(fit["cases"]) != 6:
        raise ValueError("Require physical fit results for all six installed screws")
    seen = set()
    for variant in purchased.values():
        sku = next(name for name in ("91251A542", "97763A232") if name in variant["nativeSource"])
        for target in variant["placements"]:
            head = np.array(target["headSeatXYZMm"])
            label = f"{sku}/{head[0]:g}"
            case = trial["cases"][label]
            phase = case["farFlankAngleDeg"]
            result = next(row for row in fit["cases"][label]["measured"] if row["phaseDeg"] == phase)
            if not result["valid"] or result["solids"] or result["rawCommonMm3"] > 1e-6 or result["underheadAreaMm2"] <= 0:
                raise ValueError(f"Thread or head seat held at {label}")
            if label in seen or np.linalg.norm(head - case["headXYZmm"]) > 1e-6:
                raise ValueError(f"Ambiguous or changed physical seat for {label}")
            seen.add(label)
            transform = spin_about(head, case["intoScrewAxis"], phase) @ np.array(target["worldDeltaMm"])
            datum = transform @ np.array([*variant["datumMm"], 1])
            if np.linalg.norm(datum[:3] - head) > 1e-5:
                raise ValueError(f"Lost supplier underhead datum for {label}")
            target.update(worldDeltaMm=transform.tolist(), commissionedPhaseDeg=phase)
    if seen != set(trial["cases"]):
        raise ValueError("Unaccounted fit")
    (HERE / "port-clocked-placements.json").write_text(json.dumps(purchased, indent=2) + "\n")


if __name__ == "__main__":
    main()
