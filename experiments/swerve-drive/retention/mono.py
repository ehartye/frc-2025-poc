"""Use a decoded one-eye tag only when enough independent corners, calibration and time survive."""

from pathlib import Path
import argparse
import json
import math
import cv2
import numpy as np
import dock_view as dock

V = dock.vision


def corners_from_frame(q, ctx, tag):
    # Use the complete native occlusion render, including both real fasteners.
    # Capture the per-eye detector results before its historical two-eye gate.
    captures = []
    detector = V.detect

    def capture(pixels):
        corners = detector(pixels)
        captures.append(corners)
        return corners

    try:
        V.detect = capture
        V.station_frame(q, ctx, tag)
    finally:
        V.detect = detector
    return captures


def solve(corners, world, eye):
    local = V.unproject(corners, world[:, 2], eye)
    x = local - local.mean(axis=0)
    y = world[:, :2] - world[:, :2].mean(axis=0)
    angle = math.atan2(sum(x[:, 0] * y[:, 1] - x[:, 1] * y[:, 0]), np.sum(x * y))
    point = world[:, :2].mean(axis=0) - V.rotation(angle) @ local.mean(axis=0)
    pose = np.r_[point, angle]
    for _ in range(4):
        project = lambda q: V.project(world, q, eye)[0].ravel()
        old = project(pose)
        offsets = np.diag([0.02, 0.02, 0.00003])
        jac = np.column_stack(
            [(project(pose + v) - old) / v[i] for i, v in enumerate(offsets)]
        )
        pose += np.linalg.lstsq(jac, corners.ravel() - old, rcond=None)[0]
    err = np.max(abs(V.project(world, pose, eye)[0] - corners))
    return pose, float(err)


def enclosure(corners, world, eye, target):
    prior = V.EYES
    try:
        V.EYES = (eye,)
        return V.interval([corners], world, target, epsilon=0.25)
    finally:
        V.EYES = prior


def run(artifacts):
    ctx = dock.create_scene(artifacts)
    gathered = {}
    for side in ("plus", "minus"):
        folder = artifacts / "retained-coral" / side / "continued"
        route = dock.read(folder / "route.json")
        with np.load(folder / "path.npz") as archive:
            poses = archive["worldPose"]
            odom = archive["estimatedPose"]
            clock = archive["relativeS"]
        origin = np.array(route["goals"][-1])
        local = lambda p: np.r_[1000 * (p[:2] - origin) + [-54, -180], p[2]]
        end = local(odom[-1])
        survey = []
        for i in range(0, len(clock), 10):
            pose = local(poses[i])
            estimate = local(odom[i])
            if abs(estimate[1] + 180) > 70 or estimate[0] < -1080 or abs(estimate[2]) > 0.05:
                continue
            for j, corners in enumerate(corners_from_frame(pose, ctx, 10)):
                if corners is None:
                    continue
                eye = V.EYES[j]
                fitted, residual = solve(corners, ctx["points"], eye)
                delta = end - estimate
                transported = (
                    fitted
                    + np.r_[V.rotation(fitted[2] - estimate[2]) @ delta[:2], delta[2]]
                )
                if residual > 0.35:
                    continue
                outer = enclosure(corners, ctx["points"], eye, pose[:2])
                if outer is None:
                    continue
                lo, hi = outer[:2]
                if np.any(pose < lo - 1e-5) or np.any(pose > hi + 1e-5):
                    raise ValueError("Accepted pixel interval excluded actual pose")
                survey.append(
                    dict(
                        wallS=round(route["startWallS"] + clock[i], 6),
                        eye=j,
                        corners=corners.tolist(),
                        estimated=estimate.tolist(),
                        fit=fitted.tolist(),
                        trueMmMmRad=pose.tolist(),
                        bounds=[lo.tolist(), hi.tolist()],
                        transported=transported.tolist(),
                        residualPx=residual,
                        remainingOdomDistanceMm=float(np.linalg.norm(delta[:2])),
                    )
                )
        gathered[side] = dict(
            wheel=route["trajectory"],
            final=local(poses[-1]).tolist(),
            terminalOdom=end.tolist(),
            endWallS=route["endWallS"],
            views=survey,
        )
    return gathered


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("artifacts", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    a.output.write_text(json.dumps(run(a.artifacts), indent=2) + "\n", encoding="utf-8")
