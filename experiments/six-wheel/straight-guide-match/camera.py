"""Render and measure the two roof images at each actual pipe exposure."""

import hashlib
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "mounted-guide-vision"))
from edge import peak
from controls import measure
from station import views, sensor


def serve(assets, mode, attained=False):
    _, targets, _ = views(
        assets,
        "nominal",
        ROOT / "guide-setting" if attained else None,
        write_labels=False,
    )
    actual, ref = targets[mode.removesuffix("-all-edges")]
    saved = {}
    for line in sys.stdin:
        point = np.array(json.loads(line))
        images = sensor.silhouette(actual, point[:3], point[3:])
        key = hashlib.sha256(b"".join(im.tobytes() for im in images)).hexdigest()
        if key not in saved:
            fit = measure(
                images, ref, peak_bound=peak if mode.endswith("-all-edges") else None
            )
            angle = fit.get("independentYawIntervalRad")
            admitted = (
                fit["status"] == "inside"
                and angle is not None
                and max(abs(v) for v in angle) <= np.radians(2)
            )
            status = (
                "inside"
                if admitted
                else ("unqualified-yaw" if fit["status"] == "inside" else fit["status"])
            )
            limits = [
                n
                for dim in ("x_mm", "y_mm", "bottom_mm")
                for n in fit.get(dim, [None, None])
            ]
            saved[key] = [[status, *limits], angle, key]
        print(json.dumps(saved[key]), flush=True)


if __name__ == "__main__":
    serve(Path(sys.argv[1]), sys.argv[2], "--attained" in sys.argv[3:])
