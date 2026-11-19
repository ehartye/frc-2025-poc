"""Evaluate an optional single-eye coarse ID on the archived field-camera images."""
import math
from pathlib import Path
from tempfile import TemporaryDirectory

import cv2
import numpy as np
import vision


def estimate_single_eye(quad, points, eye):
    body = vision.unproject(quad, points[:, 2], eye)
    world = points[:, :2]
    local = body - body.mean(axis=0)
    target = world - world.mean(axis=0)
    yaw = math.atan2(
        np.sum(local[:, 0] * target[:, 1] - local[:, 1] * target[:, 0]),
        np.sum(local * target),
    )
    center = world.mean(axis=0) - vision.rotation(yaw) @ body.mean(axis=0)
    pose = np.array([*center, yaw])
    increments = np.array([.1, .1, .0001])
    for _ in range(3):
        predicted = vision.project(points, pose, eye)[0].ravel()
        jacobian = np.stack([
            (vision.project(points, pose + delta, eye)[0].ravel() - predicted) / scale
            for delta, scale in zip(np.diag(increments), increments)
        ], axis=1)
        pose += np.linalg.lstsq(jacobian, quad.ravel() - predicted, rcond=None)[0]
    return pose


def coarse_observation(pose, ctx, tag, ambiguous, hide):
    # The archived observer exposes both complete camera images in one lossless PNG.
    # Only coarse tag acquisition changes. Terminal acceptance still requires both heads.
    with TemporaryDirectory() as directory:
        path = Path(directory) / "camera.png"
        result = vision.station_frame(pose, ctx, tag, ambiguous, hide, path)
        tagged, heads, projected, diagnostics = result
        if tagged is not None:
            return result
        full = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    hits = []
    for eye, frame in zip(vision.EYES, np.hsplit(full, 2)):
        quad = vision.detect(frame)
        if quad is not None:
            hits.append((eye, quad))
    if len(hits) != 1:
        return result
    eye, quad = hits[0]
    guess = estimate_single_eye(quad, ctx["points"], eye)
    residual = np.max(abs(vision.project(ctx["points"], guess, eye)[0] - quad))
    if residual > .35:
        return result
    return guess, heads, projected, diagnostics
