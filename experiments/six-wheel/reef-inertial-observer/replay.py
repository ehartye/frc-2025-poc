"""Exercise the unchanged drive and camera entry points with an onboard rate channel."""

import argparse
import json
from pathlib import Path
from observer import ROOT, Observer, nav, coarse_observation

HERE = Path(__file__).resolve().parent


def run(assets, cases):
    record = nav.read(ROOT / "mounted-guide-vision/station.json")["nominal"]["register"]
    context = nav.checked_scene(
        assets,
        use_guide=False,
        registered_background=(
            assets / "mounted-guide-vision/nominal-fixed.npz",
            record["fixedBackgroundSha256"],
        ),
    )
    initial = nav.read(ROOT / "conditioned-match/slow-navigation.json")
    calibration = nav.read(ROOT / "station-landmark/routes.json")["cases"]
    trials = []
    for side, start in initial.items():
        for fault in sorted(cases):
            config = calibration[side]
            observer = Observer(context, config, fault)
            cache = assets / "reef-inertial-observer" / "sampled" / side
            cache.mkdir(exist_ok=True, parents=True)
            result = nav.sector.tracecase(
                "coral-A",
                start["initial"],
                fault,
                context,
                cache,
                wheel_scale=config["wheelScale"],
                physical_track_mm=config["effectiveTrackMm"],
                observer=coarse_observation,
                features=observer,
                yaw_sensor=observer.gyro.sample,
            )
            result.pop("path")
            result.update(
                side=side,
                observations=observer.used,
                detectedRays=[
                    (
                        r["captureMs"],
                        r["ambiguous"],
                        [(p[0], p[1]) for p in r["features"]],
                    )
                    for r in observer.observations
                ],
                imageRegister=record["fixedBackgroundSha256"],
            )
            trials.append(result)
            print(
                side,
                fault,
                result["accepted"],
                result["reason"],
                result["finalBodyErrorMmDeg"],
                result["latestInterval"],
                result["truthOutsideDeclaredCalibration"],
                flush=True,
            )
    return sorted(trials, key=lambda r: (r["side"], r["mode"]))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("artifacts", type=Path)
    parser.add_argument(
        "--modes",
        nargs="+",
        default=[
            "nominal",
            "wrong-face",
            "dropout",
            "stale",
            "ambiguous",
            "gyro-dropout",
            "shared-survey-shift",
            "lateral-kick",
        ],
    )
    parser.add_argument("--output", type=Path, default=HERE / "results.json")
    args = parser.parse_args()
    args.output.write_text(
        json.dumps(run(args.artifacts, args.modes), indent=2) + "\n",
        encoding="utf8",
        newline="\n",
    )
