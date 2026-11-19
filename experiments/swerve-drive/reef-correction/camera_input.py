"""Build the source-pinned camera foreground and audit the observed corner calibration."""

import argparse
import json
from pathlib import Path
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "retention"))
import reef_fix as fix

V = fix.V
MAX_RESIDUAL = 0.18


def tagged_history(case, ctx):
    rows = []
    review = []
    for row in case["views"]:
        reprojection = V.project(ctx["points"], row["trueMmMmRad"], V.EYES[row["eye"]])[
            0
        ]
        error = float(np.max(abs(reprojection - row["corners"])))
        accepted = row["residualPx"] <= MAX_RESIDUAL
        if accepted and error > fix.PIXEL:
            raise ValueError("Measured corner escapes the selected calibration")
        if accepted:
            rows.append(row)
        review.append(
            dict(
                wallS=row["wallS"],
                residualPx=row["residualPx"],
                actualMaxCornerErrorPx=error,
                accepted=accepted,
            )
        )
    return {**case, "views": rows}, review


def run(artifacts):
    out = artifacts / "reef-correction"
    out.mkdir(exist_ok=True)
    ctx = fix.mono.dock.create_scene(artifacts)
    registered = {}
    for side, case in fix.read(artifacts / "captive-observer/mono.json").items():
        selected, review = tagged_history(case, ctx)
        _, heads, _, _ = V.station_frame(case["final"], ctx)
        result = fix.fix(selected, ctx, heads, "nominal")
        if not result["accepted"]:
            raise ValueError("No corrected initial optical bound")
        registered[side] = dict(
            optical=result,
            cornerReview=review,
            source=case["wheel"],
            pose=case["final"],
        )
    scene = out / "camera.npz"
    np.savez_compressed(
        scene,
        **{k: v for k, v in ctx.items() if k != "faces"},
        face0=ctx["faces"][0],
        face1=ctx["faces"][1],
    )
    data = dict(
        bytes=fix.sha(scene),
        sourceSurvey=fix.sha(artifacts / "captive-observer/mono.json"),
        study=fix.sha(fix.HERE / "reef-fix.json"),
        cases=registered,
    )
    (out / "camera.json").write_text(json.dumps(data, indent=2) + "\n", encoding="utf8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifacts", type=Path)
    args = parser.parse_args()
    run(args.artifacts)
