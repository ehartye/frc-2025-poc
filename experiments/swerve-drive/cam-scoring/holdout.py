"""Challenge the frozen tag allowance on off-clock poses, lighting and a look-alike tag."""

import argparse
import json
import math
from pathlib import Path
import numpy as np
from reef_camera import camera_input, TAG_ERROR, sha

V = camera_input.V


def run(artifacts):
    folder = artifacts / "cam-scoring"
    with np.load(folder / "camera.npz") as archive:
        ctx = {key: archive[key] for key in archive.files}
    ctx["faces"] = [ctx.pop("face0"), ctx.pop("face1")]
    source_survey = json.loads((folder / "reef-survey.json").read_bytes())
    cases = []
    # The 25 ms offset was excluded from the 50 ms calibration series.
    for side in ("plus", "minus"):
        route = json.loads((folder / side / "route/route.json").read_bytes())
        with np.load(folder / side / "route/path.npz") as z:
            poses = z["worldPose"]
            clock = z["relativeS"]
        origin = np.asarray(route["goals"][-1])
        start = route["startWallS"]
        first = source_survey[side]["views"][0]["wallS"]
        last = source_survey[side]["views"][-1]["wallS"]
        ids = [i for i in range(5, len(clock), 10) if first <= start + clock[i] <= last]
        for i in ids[::2]:
            for treatment, dx, dy, dyaw, gain, bias, tag in [
                ("subpixel", -2.0, 1.25, 0.05, 1.0, 0.0, 10),
                ("dim", 1.5, -2.75, -0.12, 0.72, 12.0, 10),
                ("bright", -1.1, 3.5, 0.17, 1.18, -3.0, 10),
                ("wrong-tag", 0.0, 0.0, 0.0, 1.0, 0.0, 11),
            ]:
                pose = np.r_[
                    1000 * (poses[i, :2] - origin) + [-54 + dx, -180 + dy],
                    poses[i, 2] + math.radians(dyaw),
                ]
                detector = V.detect
                captured = []

                def lighting(pixels):
                    image = np.clip(np.rint(pixels * gain + bias), 0, 255).astype(
                        np.uint8
                    )
                    corners = detector(image)
                    captured.append(corners)
                    return corners

                V.detect = lighting
                try:
                    V.station_frame(pose, ctx, tag)
                finally:
                    V.detect = detector
                for eye, corners in enumerate(captured):
                    row = dict(
                        case=side,
                        treatment=treatment,
                        wallS=start + clock[i],
                        eye=eye,
                        decoded=corners is not None,
                    )
                    if corners is not None:
                        _, res = camera_input.fix.mono.solve(
                            corners, ctx["points"], V.EYES[eye]
                        )
                        true = V.project(ctx["points"], pose, V.EYES[eye])[0]
                        error = float(np.max(abs(corners - true)))
                        row.update(
                            residualPx=res,
                            accepted=res <= camera_input.MAX_RESIDUAL,
                            errorPx=error,
                            withinFrozenBound=error <= TAG_ERROR,
                        )
                    cases.append(row)
    positive = [r for r in cases if r.get("accepted")]
    failures = [
        r
        for r in positive
        if r["treatment"] == "wrong-tag" or not r["withinFrozenBound"]
    ]
    return dict(
        tagErrorPx=TAG_ERROR,
        observableResidualPx=camera_input.MAX_RESIDUAL,
        scene=sha(folder / "camera.npz"),
        observations=cases,
        accepted=len(positive),
        falseGrants=failures,
        maximumAcceptedErrorPx=max((r["errorPx"] for r in positive), default=None),
        limitedCalibration=bool(positive) and not failures,
        deliveryCredit=0,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("artifacts", type=Path)
    args = p.parse_args()
    r = run(args.artifacts)
    (args.artifacts / "cam-scoring/holdout.json").write_text(
        json.dumps(r, indent=2) + "\n"
    )
    print(
        {k: v for k, v in r.items() if k not in ("observations", "falseGrants")},
        len(r["falseGrants"]),
    )
