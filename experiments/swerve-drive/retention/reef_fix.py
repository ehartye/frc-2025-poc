"""Bound a terminal reef fix using a decoded approach tag, gyro increments and stationary bolt pixels."""

import argparse
import itertools
import json
import math
from pathlib import Path

import numpy as np

import mono
from screen import HERE, ROOT, read, sha

V = mono.V
PIXEL = 0.25
ROTATION_STEP = math.radians(0.025)
MOUNT_MM = 1.0
MOUNT_ANGLE_RAD = math.radians(0.05)
WHEEL_PROPAGATION_MM = 12.0


def rays(observed, height, eye, pixel_error=PIXEL):
    radius = np.array(list(itertools.product((-pixel_error, pixel_error), repeat=2)))
    return V.unproject(observed + radius, np.full(4, height), eye)


def rotated_box(point, local, angles, angular_error, linear_error):
    # Each polygon is the interval image of one pixel rectangle at a surveyed
    # height. Bound all angles between the sampled centers by their largest chord.
    x, y = local.T
    cs = np.cos(angles)[:, None]
    sn = np.sin(angles)[:, None]
    xx = point[0] - cs * x + sn * y
    yy = point[1] - sn * x - cs * y
    pad = linear_error + 2 * np.linalg.norm(local, axis=1).max() * math.sin(
        angular_error / 2
    )
    return np.stack((xx.min(1) - pad, yy.min(1) - pad), axis=1), np.stack(
        (xx.max(1) + pad, yy.max(1) + pad), axis=1
    )


def box_report(low, high, angles, calibration):
    good = np.all(low <= high, axis=1)
    if not good.any():
        return None
    return [
        np.r_[
            low[good].min(0) - calibration, angles[good].min() - ROTATION_STEP / 2
        ].tolist(),
        np.r_[
            high[good].max(0) + calibration, angles[good].max() + ROTATION_STEP / 2
        ].tolist(),
    ]


def fix(case, ctx, heads, mode, tag_error=PIXEL):
    # The starting prior is odometric heading with a deliberately broad +/- 5 deg.
    # Relative gyro drift and scale are independent of the AprilTag fit.
    odom = np.array(case["terminalOdom"])
    angles = odom[2] + np.arange(-200, 201) * ROTATION_STEP
    low = np.full((len(angles), 2), -np.inf)
    high = -low.copy()
    for row in case["views"]:
        age = case["endWallS"] - row["wallS"]
        if age > 3.0:
            continue
        estimate = np.array(row["estimated"])
        delta = odom - estimate
        turn = delta[2]
        untracked = math.radians(0.02) * age + 0.001 * abs(turn)
        alpha = ROTATION_STEP / 2 + untracked + MOUNT_ANGLE_RAD
        if mode == "wrong-tag":
            continue
        # A common translational drift never enlarges the tag's measured heading
        # spread. Constrain the gyro separately before propagating its position.
        yaw_low = (
            row["bounds"][0][2] + turn - untracked - MOUNT_ANGLE_RAD - ROTATION_STEP / 2
        )
        yaw_high = (
            row["bounds"][1][2] + turn + untracked + MOUNT_ANGLE_RAD + ROTATION_STEP / 2
        )
        excluded = (angles < yaw_low) | (angles > yaw_high)
        low[excluded] = np.inf
        high[excluded] = -np.inf
        local_delta = V.rotation(-estimate[2]) @ delta[:2]
        for corner, world in zip(row["corners"], ctx["points"]):
            polygon = rays(corner, world[2], V.EYES[row["eye"]], tag_error) - local_delta
            l, h = rotated_box(
                world[:2],
                polygon,
                angles - turn,
                alpha,
                WHEEL_PROPAGATION_MM + MOUNT_MM,
            )
            low = np.maximum(low, l)
            high = np.minimum(high, h)
    prior = box_report(low, high, angles, 0)
    if mode == "wrong-tag" or prior is None:
        return dict(prior=None, accepted=False)
    matches = []
    reference = ctx["bolts"][0]
    for index, features in enumerate(heads):
        eye = V.EYES[index]
        choices = []
        for centroid in features:
            a, b = rotated_box(
                reference[:2],
                rays(centroid, reference[2], eye),
                angles,
                ROTATION_STEP / 2 + MOUNT_ANGLE_RAD,
                MOUNT_MM,
            )
            if np.any(np.all(np.maximum(low, a) <= np.minimum(high, b), axis=1)):
                choices.append((centroid, a, b))
        if len(choices) != 1:
            return dict(
                prior=prior,
                accepted=False,
                matchingCounts=[len(choices)],
                reason="missing or ambiguous surveyed bolt",
            )
        matches.append(choices[0])
    for _, l, h in matches:
        low = np.maximum(low, l)
        high = np.minimum(high, h)
    result = box_report(low, high, angles, 0)
    return dict(
        prior=prior,
        accepted=result is not None,
        positionMmHeadingRad=result,
        matchedPixels=[np.asarray(q).tolist() for q, _, _ in matches],
    )


def run(artifacts, survey):
    ctx = mono.dock.create_scene(artifacts)
    out = {}
    for name, case in read(survey).items():
        final = np.array(case["final"])
        wheel = artifacts / "retained-coral" / name / "continued/path.npz"
        if sha(wheel) != case["wheel"]:
            raise ValueError("Different wheel history")
        with np.load(wheel) as path:
            actual = path["worldPose"]
            odom = path["estimatedPose"]
            times = path["relativeS"]
        wheel_error = []
        accepted = []
        for row in case["views"]:
            t = case["endWallS"] - row["wallS"]
            if t > 3:
                continue
            i = int(np.argmin(abs(times - (times[-1] - t))))
            d = V.rotation(actual[i, 2] - odom[i, 2]) @ (odom[-1, :2] - odom[i, :2])
            wheel_error.append(
                float(1000 * np.linalg.norm(d - (actual[-1, :2] - actual[i, :2])))
            )
            accepted.append(row)
        if not wheel_error or max(wheel_error) > WHEEL_PROPAGATION_MM:
            raise ValueError("Uncovered transport error")
        _, heads, _, _ = V.station_frame(final, ctx)
        target = V.project(ctx["bolts"][:1], final, V.EYES[0])[0][0]
        controls = {
            "nominal": heads,
            "missing-eye": [[], heads[1]],
            "ambiguous": [heads[0] + [target + np.array([1.0, 0.0])], heads[1]],
            "wrong-tag": heads,
        }
        rows = {mode: fix(case, ctx, h, mode) for mode, h in controls.items()}
        nominal = rows["nominal"]
        if nominal["accepted"]:
            l, h = np.array(nominal["positionMmHeadingRad"])
            if np.any(final < l) or np.any(final > h):
                raise ValueError("Optical bound excludes attained pose")
        out[name] = dict(
            wheel=case["wheel"],
            finalMmMmRad=case["final"],
            decodedViews=len(case["views"]),
            recentViews=len(accepted),
            lastTagAgeS=case["endWallS"] - accepted[-1]["wallS"],
            maxObservedTransportErrorMm=max(wheel_error),
            observation=rows,
        )
    return dict(
        surveySha256=sha(survey),
        nativeRetainer=read(HERE / "open-yokes.json")["stepSha256"],
        reefSource=sha(mono.dock.SECTOR / "scene.json"),
        pixelError=PIXEL,
        mountTranslationMm=MOUNT_MM,
        mountYawDeg=math.degrees(MOUNT_ANGLE_RAD),
        gyroDriftDegPerS=0.02,
        gyroScaleError=0.001,
        wheelDisplacementErrorMm=WHEEL_PROPAGATION_MM,
        cases=out,
        deliveryCredit=0,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for arg in ("artifacts", "survey", "output"):
        p.add_argument(arg, type=Path)
    args = p.parse_args()
    args.output.write_text(
        json.dumps(run(args.artifacts, args.survey), indent=2) + "\n", encoding="utf8"
    )
