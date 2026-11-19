"""Run the selected staged drive with the supplied installed station raster and detector."""

import numpy as np
from supply import n, staged, referenced, approach, join


def opening(
    assets,
    side,
    *,
    pace,
    wheel_accel_mm_s2,
    registered_depth,
    observation_policy,
    station_fault,
):
    cfg = n.read(n.ROOT / "station-landmark/routes.json")["cases"][side]
    scale = np.asarray(cfg["wheelScale"])
    width = cfg["effectiveTrackMm"]
    q, wall, f, leg, t, actual, _ = staged(
        assets / "relieved-match",
        scale,
        width,
        cfg["gyroBiasDegS"],
        cfg["gyroScaleError"],
        cfg["startHeadingErrorDeg"],
        stage_mm=400,
        side_mm=0,
        pace=pace,
    )
    _, _, bias, _ = referenced(assets / "selected-optics/issued/marked")
    image_path, expected_sha = registered_depth
    if n.sha(image_path) != expected_sha:
        raise ValueError("Station camera geometry changed")
    with np.load(image_path) as raw:
        bg = [raw[key + "_z"].copy() for key in ("left", "right")]
    r = approach(
        q,
        wall,
        scale,
        width,
        bg,
        bias,
        side_bias=cfg["assumedStationExtrinsicErrorMm"],
        deadline=21.1,
        wheel_accel_mm_s2=wheel_accel_mm_s2,
        observation_policy=observation_policy,
        fault=station_fault,
    )
    # The original source labels the integrated end of each 10 ms wheel step with the start time.
    # Put that pose at the end of its physical interval and retain the exact initial zero-speed state.
    rows = [[wall, *q, 0.0, 0.0]] + [[row[0] + 0.01, *row[1:]] for row in r["path"]]
    return (
        *join(f, leg, t, actual, rows),
        r["finalStationMmDeg"],
        r["status"],
        r["wallS"],
    )
