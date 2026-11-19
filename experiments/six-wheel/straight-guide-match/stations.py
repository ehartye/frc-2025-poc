"""Challenge the tighter concavity envelope on the same two physical feeds and fault images."""

import argparse
import json
import math
from pathlib import Path
import numpy as np
from camera import ROOT, views, sensor, measure, peak
from controls import run as challenges
from station import station_packets, CANONICAL_PACKETS, read
import importlib.util

spec = importlib.util.spec_from_file_location(
    "mounted_handoff", ROOT / "mounted-guide-vision/handoff.py"
)
handoff_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(handoff_module)
handoff = handoff_module.handoff

HERE = Path(__file__).resolve().parent


def run(assets):
    _, opts, _ = views(assets, "nominal", write_labels=False)
    issued = read(ROOT / "mounted-guide-vision/station.json")["nominal"]["cases"]
    observations = []
    permits = []
    for packet in station_packets(CANONICAL_PACKETS):
        if not packet["id"].endswith("serial-carriage-recover"):
            continue
        a = next(r for r in issued if r["source"] == packet["id"])
        b = dict(a, boundedGaps=[])
        for f in packet["frames"]:
            if f["javaMs"] not in (5372, 5389, 5406, 5423, 5440, 5457):
                continue
            p = np.asarray(f["centerMm"])
            axis = np.asarray(f["axis"])
            for mode, (bg, ref) in opts.items():
                image = sensor.silhouette(bg, p, axis)
                result = measure(image, ref, peak_bound=peak)
                box = [result.get(k) for k in ("x_mm", "y_mm", "bottom_mm")]
                angle = result.get("independentYawIntervalRad")
                truth = [p[0], p[1], p[2] - 557.15, math.atan2(axis[0], axis[1])]
                enclosed = all(
                    v is not None and v[0] <= x <= v[1]
                    for v, x in zip([*box, angle], truth)
                )
                if result["status"] == "inside" and angle is not None and not enclosed:
                    raise ValueError("Accepted interval excludes independent pipe")
                b["boundedGaps"].append(
                    [
                        f["javaMs"],
                        mode + "-all-edges",
                        result["status"],
                        enclosed,
                        box,
                        angle,
                    ]
                )
        observations.append({"source": a["source"], "frames": b["boundedGaps"]})
        for mode in opts:
            permits.append(handoff(b, mode + "-all-edges"))
    faults = challenges(assets, peak_bound=peak)
    return {"observations": observations, "gate": permits, "controls": faults}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("assets", type=Path)
    args = parser.parse_args()
    (HERE / "stations.json").write_text(
        json.dumps(run(args.assets), indent=2) + "\n", encoding="utf8", newline="\n"
    )
