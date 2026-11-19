"""Find two station poses that admit the same upper-hole measurement under bounded error."""

import argparse
import json
import math
import sys
from pathlib import Path
import numpy as np
from plant import HERE, ROOT, digest, read

sys.path.insert(0, str(ROOT / "station-landmark"))
import panel


def compare(assets):
    image = assets / "mounted-guide-vision/set-nominal-fixed.npz"
    expected = read(ROOT / "mounted-guide-vision/setting-station.json")["set-nominal"][
        "register"
    ]
    if digest(image) != expected["fixedBackgroundSha256"]:
        raise ValueError("Changed station raster")
    with np.load(image) as f:
        depths = [f[k + "_z"] for k in ("left", "right")]

    def segmented(pose):
        ideal = panel.pixels(pose)[:, :2]
        measured = np.empty_like(ideal)
        for i, (frame, eye) in enumerate(zip(panel.render(pose, depths), ideal)):
            centers = np.asarray(panel.components(frame)).reshape(-1, 2)
            for k, center in enumerate(eye):
                candidates = centers[np.linalg.norm(centers - center, axis=1) < 9]
                if len(candidates) != 1:
                    raise ValueError("Not a unique upper aperture")
                measured[i, k] = candidates[0]
        return measured

    source = HERE / "revised-opening.json"
    out = {"openingSha256": digest(source), "rasterSha256": digest(image), "cases": []}
    for side, r in read(source).items():
        if (
            digest(assets / "straight-guide-match" / (side + "-revised-opening.npz"))
            != r["pathSha256"]
        ):
            raise ValueError("Different wheel trajectory")
        q = np.asarray(r["reachedMmDeg"])
        q[2] = math.radians(q[2])
        ideal = panel.pixels(q)[:, :2]
        measured = segmented(q)
        for lateral in (-3, 3):

            def alternative(degrees):
                return q + [0, lateral, math.radians(degrees)]

            def ideal_error(degrees):
                return (
                    np.max(abs(ideal - panel.pixels(alternative(degrees))[:, :2])) / 2
                )

            yaw = min(np.linspace(-0.5, 0.5, 1001), key=ideal_error)
            shared_error = np.max(abs(measured - segmented(alternative(yaw)))) / 2
            out["cases"].append(
                dict(
                    side=side,
                    pathSha256=r["pathSha256"],
                    centerMmDeg=r["reachedMmDeg"],
                    offsetMm=lateral,
                    yawDifferenceDeg=yaw,
                    projectedHalfDifferencePx=ideal_error(yaw),
                    rasterHalfDifferencePx=shared_error,
                )
            )
    return out


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("assets", type=Path)
    a = p.parse_args()
    (HERE / "station-uncertainty.json").write_text(
        json.dumps(compare(a.assets), indent=2) + "\n", encoding="utf8", newline="\n"
    )
