"""Observe the predeclared 0.20 mm gap case through the same station pickup gate."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "cam-scoring"))

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("artifacts", type=Path)
    ap.add_argument(
        "--stage",
        choices=(
            "optics",
            "station-wheels",
            "station-cargo",
            "route-wheels",
            "route-cargo",
        ),
        default="optics",
    )
    args = ap.parse_args()
    root = args.artifacts
    cohort = HERE / "cohort.json"
    case = next(
        c for c in json.loads(cohort.read_bytes())["cases"] if c["id"] == "holdout"
    )
    side = case["wheelCalibration"]
    gap = case["plungerGapMm"]
    feed = root / "r4-score-stroke" / f"cam-station-{side}-31.25us-{gap:g}mm.json"
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    if sha(feed) != case["firstFeedReceipt"]:
        raise ValueError("Wrong held-out input")
    if args.stage != "optics":
        station = args.stage.split("-")[0]
        kw = dict(
            gap_mm=gap,
            service_root=root / "low-placement/holdout-service",
            output_root=root / "low-placement/holdout",
        )
        if args.stage.endswith("wheels"):
            import wheels

            r = wheels.run(
                root,
                side,
                station,
                optics_record=root / "low-placement/heldout-station.json",
                **kw,
            )
        else:
            import captivity

            r = captivity.run(root, side, station, **kw)
        print({k: r.get(k) for k in ("allGranted", "fault", "startWallS", "endWallS")})
        sys.exit(0)
    from station_optics import run

    observations = run(root, sides=(side,), gap_mm=gap)[side]
    result = dict(
        cohort=sha(cohort),
        feed=sha(feed),
        piece=case["piece"],
        case=case["id"],
        **{k: v for k, v in observations.items() if k not in ("piece", "case")},
    )
    (root / "low-placement" / "heldout-station.json").write_bytes(
        (json.dumps(result, indent=2) + "\n").encode()
    )
    print(result["counts"])
