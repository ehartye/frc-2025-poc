"""Read the observed station state on the reached chassis, actual field and match-drilled guides."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
from plant import ROOT, HERE, read, digest
from camera import views, sensor, measure, peak
from station import field_mesh, O, read_pair, silhouette
from station_scene import scene


def per_case(assets, side, micros, field_path):
    r = read(HERE / f"feed-{side}-{micros:g}.json")
    starting = read(HERE / "revised-opening.json")[side]
    expected = [
        -starting["reachedMmDeg"][0],
        -starting["reachedMmDeg"][1],
        starting["reachedMmDeg"][2],
    ]
    if r["poseMmDeg"] != expected:
        raise ValueError("Mismatched feeder registration")
    result = {
        "id": r["id"],
        "side": side,
        "stepUs": micros,
        "declaredPiece": "coral-A",
        "firstInBandMs": r["firstBandMs"],
        "portState": r["lastState"],
        "fault": r["reason"],
        "pipeSha256": r["trajectorySha256"],
        "stateSha256": r["stateSha256"],
        "startTime": starting["stationWallS"],
        "hardStop": r["hardStop"],
        "loss": r["loss"],
        "fieldCameraSha256": starting["fieldCameraSha256"],
        "frames": [],
    }
    if r["lastState"] != "IN_BAND" or r["lastJavaMs"] != 5460:
        return dict(result, guardedStationHandoff=False, decision="station hold")
    if (
        digest(field_path)
        != read(ROOT / "full-match/installed-stock.json")["field_step_sha256"]
    ):
        raise ValueError("Field geometry changed")
    path = assets / "straight-guide-match/attained-feed" / r["id"] / (r["id"] + ".npy")
    if digest(path) != result["pipeSha256"]:
        raise ValueError("Feeder trajectory mismatch")
    tr = np.load(path, mmap_mode="r")
    fixed, roofs, _ = views(
        assets, "nominal", ROOT / "guide-setting", write_labels=False
    )
    sensor.scenes.fixed_pose(O)
    bg = scene(field_mesh(field_path), r["poseMmDeg"], fixed)
    empty = [g for g, z in bg]
    for stamp in (5372, 5389, 5406, 5423, 5440, 5457):
        row = tr[np.abs(tr[:, 0] - stamp).argmin()]
        if abs(row[0] - stamp) > 1e-5:
            raise ValueError("Missing actual capture")
        center = row[1:4]
        axis = row[4:7]
        truth = [center[0], center[1], center[2] - 557.15]
        sensor.scenes.fixed_pose(O)
        imgs = silhouette(bg, center, axis)
        support = read_pair(imgs, empty, 560, 0)
        fixed_ok = support["status"] == "position-witness"
        if fixed_ok and not all(
            support[k][0] <= x <= support[k][1]
            for k, x in zip(
                ("u_interval_mm", "y_interval_mm", "bottom_relative_to_shelf_mm"), truth
            )
        ):
            raise ValueError("Fixed camera bound missed pipe")
        sensor.optics.configure()
        actual, ref = roofs["guarded"]
        imgs = sensor.silhouette(actual, center, axis)
        f = measure(imgs, ref, peak_bound=peak)
        angle = f.get("independentYawIntervalRad")
        accepts = (
            f["status"] == "inside"
            and angle is not None
            and max(abs(v) for v in angle) <= math.radians(2)
        )
        if accepts and not all(
            f[k][0] <= x <= f[k][1]
            for k, x in zip(("x_mm", "y_mm", "bottom_mm"), truth)
        ):
            raise ValueError("Roof image missed pipe")
        if accepts and not angle[0] <= math.atan2(axis[0], axis[1]) <= angle[1]:
            raise ValueError("Wrong yaw")
        result["frames"].append(
            dict(
                javaMs=stamp,
                serviceMs=5 * math.ceil((stamp + 15) / 5),
                trueCenterMm=center.tolist(),
                trueAxis=axis.tolist(),
                fixed=fixed_ok,
                roof=accepts,
                roofStatus=f["status"],
                measuredBoxMm=[f.get(k) for k in ("x_mm", "y_mm", "bottom_mm")],
                angleRad=angle,
            )
        )
    eligible = [v for v in result["frames"] if v["serviceMs"] <= 5460]
    ok = len(eligible) == 5 and all(v["fixed"] and v["roof"] for v in eligible)
    return dict(
        result, guardedStationHandoff=ok, decision="carry" if ok else "station hold"
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("assets", type=Path)
    p.add_argument("field", type=Path)
    p.add_argument("side")
    p.add_argument("micros", type=float)
    a = p.parse_args()
    result = per_case(a.assets, a.side, a.micros, a.field)
    (HERE / f"station-{a.side}-{a.micros:g}.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf8", newline="\n"
    )
