"""Drive the selected reef approach from the pose actually reached at the station."""

import argparse, json, math, sys
from pathlib import Path
import numpy as np
from plant import ROOT, HERE, read, digest

sys.path.insert(0, str(ROOT / "reef-inertial-observer"))
from observer import nav, Observer, coarse_observation
from heading import drive


def run(assets, anchored=False):
    out = []
    reg = read(ROOT / "mounted-guide-vision/setting-station.json")["set-nominal"][
        "register"
    ]
    ctx = nav.checked_scene(
        assets,
        use_guide=False,
        registered_background=(
            assets / "mounted-guide-vision/set-nominal-fixed.npz",
            reg["fixedBackgroundSha256"],
        ),
    )
    for side, opening in read(HERE / "revised-opening.json").items():
        if opening["status"] != "observed stationary":
            continue
        x, y, h = opening["reachedMmDeg"]
        config = read(ROOT / "station-landmark/routes.json")["cases"][side]
        target, before, _ = nav.placed_route("coral-A", [-x, -y, h])
        received = opening["poseFits"][-1]
        if abs(received[0] - opening["stationWallS"]) > 1e-6:
            raise ValueError("Stale opening pose")
        camera = received[2]
        planned_source = (
            [-camera[0], -camera[1], math.degrees(camera[2])] if anchored else [0, 0, 0]
        )
        _, planned, _ = nav.placed_route("coral-A", planned_source)
        if anchored:
            _, baseline, _ = nav.placed_route("coral-A", [0, 0, 0])
            constrained = nav.corrected().Route(planned.start)
            for leg in baseline.segments:
                if leg["kind"] == "drive":
                    constrained.go(leg["end"][:2], reverse=leg["demand"] < 0)
            constrained.face(baseline.end[2])
            planned = constrained
        t, p, w = drive(
            planned,
            before.start,
            np.array(config["wheelScale"]),
            config["effectiveTrackMm"],
            config["gyroBiasDegS"],
            config["gyroScaleError"],
            0.85,
        )
        marker = target.segments[-1]["start"]
        initial = [
            1000 * (p[-1, 0] - marker[0]),
            1000 * (p[-1, 1] - marker[1]),
            math.degrees(nav.corrected().short(p[-1, 2] - marker[2])),
        ]
        cache = (
            assets
            / "straight-guide-match"
            / ("anchored-route" if anchored else "attained-route")
            / side
        )
        cache.mkdir(parents=True, exist_ok=True)
        stage = cache / "transit.npz"
        np.savez_compressed(stage, times=t, poses=p, wheels_mm_s=w)
        for fault in ("nominal", "stale", "shared-survey-shift"):
            observer = Observer(ctx, config, fault)
            row = nav.sector.tracecase(
                "coral-A",
                initial,
                fault,
                ctx,
                cache,
                wheel_scale=config["wheelScale"],
                physical_track_mm=config["effectiveTrackMm"],
                observer=coarse_observation,
                features=observer,
                yaw_sensor=observer.gyro.sample,
            )
            row.pop("path")
            row.update(
                side=side,
                transitSha256=digest(stage),
                startPoseMmDeg=opening["reachedMmDeg"],
                driveSeedMmDeg=planned_source,
                stationMotionSha256=opening["pathSha256"],
                cameraSha256=reg["fixedBackgroundSha256"],
            )
            print(side, fault, row["accepted"], row["finalBodyErrorMmDeg"], flush=True)
            out.append(row)
    return out


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("assets", type=Path)
    p.add_argument("--anchored", action="store_true")
    a = p.parse_args()
    (HERE / ("anchored-route.json" if a.anchored else "route.json")).write_text(
        json.dumps(run(a.assets, a.anchored), indent=2) + "\n",
        encoding="utf8",
        newline="\n",
    )
