"""Find current parts in the proposed rear frame wedges from the cold Fusion survey."""

import hashlib
import json
import math
from pathlib import Path


HERE = Path(__file__).resolve().parent


def overlaps_tube_box(bounds, side, setback):
    x0, y0, z0, x1, y1, z1 = bounds
    if side < 0:
        y0, y1 = -y1, -y0
    outer = 700 - setback
    inner = outer - 20 * math.sqrt(2)
    # Conservative separating planes for the two mitered tube ends and outer/inner flats.
    return (
        z1 >= 50
        and z0 <= 150
        and -x0 >= min(330, inner - 325) - 1e-5
        and -x1 <= 350 + 1e-5
        and y0 <= 350 + 1e-5
        and y1 >= min(325, inner - 330) - 1e-5
        and -x0 + y1 >= inner - 1e-5
        and -x1 + y0 <= outer + 1e-5
    )


def run():
    source = (
        HERE.parent
        / "assembly-engineering/receiver-service/issue/engineering-reopen.json"
    )
    rows = json.loads(source.read_bytes())["bodies"]
    design = json.loads((HERE / "construction.json").read_bytes())
    result = {
        "sourceNativeObservations": hashlib.sha256(source.read_bytes()).hexdigest(),
        "surveyedBodiesIncludingDemonstration": len(rows),
        "candidateSetbackMm": design["frameSetbackMm"],
        "possibleParkedIntersections": {
            corner: [
                row["occurrence"]
                for row in rows
                if overlaps_tube_box(row["boundsMm"], side, design["frameSetbackMm"])
            ]
            for corner, side in (("RL", 1), ("RR", -1))
        },
    }
    (HERE / "neighbors.json").write_bytes(
        (json.dumps(result, indent=2) + "\n").encode()
    )


if __name__ == "__main__":
    run()
