"""Observe the two new same-cam arrivals from the actual preceding registered wheel histories."""

from pathlib import Path
import argparse
import json
import sys
import numpy as np
from station_optics import SW, foreground, sha

sys.path.insert(0, str(SW / "reef-correction"))
import camera_input


TAG_ERROR = 0.65


def run(artifacts):
    out = artifacts / "cam-scoring"
    fixed, _, cam = foreground()
    robot = out / "cam-fixed.npz"
    np.savez_compressed(
        robot,
        **{
            f"fixed{i}{k}": im
            for i, eye in enumerate(fixed)
            for k, im in zip(("g", "z"), eye)
        },
    )
    ctx = camera_input.fix.mono.dock.create_scene(artifacts, robot)
    routes = {s: out / s / "route" for s in ("plus", "minus")}
    survey = camera_input.fix.mono.run(
        artifacts, scene=ctx, routes=routes, pixel_error=TAG_ERROR
    )
    data = out / "reef-survey.json"
    data.write_text(json.dumps(survey, indent=2) + "\n", encoding="utf8")
    camera_input.run(artifacts, scene=ctx, survey=data, output=out, tag_error=TAG_ERROR)
    return dict(
        foreground=sha(robot),
        survey=sha(data),
        cam=cam["candidateStepSha256"],
        fieldScene=sha(out / "camera.npz"),
        calibration=sha(out / "camera.json"),
        wheelMasks=sha(artifacts / "loaded-swerve/wheel-visibility.json"),
        yokes=sha(SW / "retention/open-yokes.step"),
        initialViews={s: len(c["views"]) for s, c in survey.items()},
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("artifacts", type=Path)
    args = p.parse_args()
    result = run(args.artifacts)
    (args.artifacts / "cam-scoring/reef-optics.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf8"
    )
    print(result)
