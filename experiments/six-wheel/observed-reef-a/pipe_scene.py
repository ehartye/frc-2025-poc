"""Render the finite carried pipe and fabricated guide against the fixed-camera shop depth."""

from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "conditioned-match"))
from views import load_navigation

nav = load_navigation()
from vision import C, S, F, EYES
from scene import context


def robot_scene(assets):
    source = nav.read(ROOT / "station-guide/optics-narrow.json")
    path = assets / "station-guide/selected-background/stowed-robot-fixed.npz"
    if nav.sha(path) != source["registeredReceiverDepthSha256"]:
        raise ValueError("Changed shop stowed robot")
    sys.path.insert(0, str(ROOT / "station-guide"))
    from optics import depth

    result = context(assets / "sector-alignment")
    with np.load(path) as full:
        for name, eye in zip(("left", "right"), EYES):
            guide = depth(eye, 0, 3.8, 40, 20)
            z = full[name + "_z"]
            result[name] = np.where(guide < z, 160, full[name]).astype(np.uint8)
            result[name + "_z"] = np.minimum(z, guide)
    return result


def finite_depth(center, axis, eye, radius=57.15, half=150.8125):
    """Nearest axial depth of the capped finite tube, in millimetres per pixel ray."""
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    offset = np.asarray([-315.0, eye, 970.0]) - center
    v, u = np.indices((480, 640), dtype=float)
    up = (240 - v) / F
    rays = np.array([C + up * S, (320 - u) / F, -S + up * C])
    axial = np.einsum("i,ijk->jk", axis, rays)
    origin = np.dot(offset, axis)
    k = np.einsum("ijk,ijk->jk", rays, rays) - axial**2
    b = np.einsum("i,ijk->jk", offset, rays) - origin * axial
    c = np.dot(offset, offset) - origin**2 - radius**2
    root = b * b - k * c
    with np.errstate(divide="ignore", invalid="ignore"):
        near = (-b - np.sqrt(np.maximum(root, 0))) / k
        far = (-b + np.sqrt(np.maximum(root, 0))) / k
        low_cap = (-half - origin) / axial
        high_cap = (half - origin) / axial
    entry = np.maximum(near, np.minimum(low_cap, high_cap))
    exit_depth = np.minimum(far, np.maximum(low_cap, high_cap))
    return np.where((root >= 0) & (entry > 0) & (exit_depth >= entry), entry, np.inf)


def carried(robot, center, axis, radius=57.15, half=150.8125):
    frame = dict(robot)
    for name, eye in zip(("left", "right"), EYES):
        tube = finite_depth(np.asarray(center), axis, eye, radius, half)
        z = robot[name + "_z"]
        frame[name] = np.where(tube < z, 170, robot[name]).astype(np.uint8)
        frame[name + "_z"] = np.minimum(z, tube)
    return frame


def bound_machined_shelf(scene):
    """Fill the whole machined shelf's two-millimeter deformation envelope."""
    register = nav.read(HERE / "machined-bounds.json")["boxesMm"]
    ids = [
        "receiver-R6-drilled-for-camera",
        "r6-receiver-insert",
        *(f"observer-moving-pad-{i}" for i in range(3)),
    ]
    boxes = np.array([register[key] for key in ids])
    low = boxes[:, :3].min(axis=0) - 2.0
    high = boxes[:, 3:].max(axis=0) + 2.0
    v, u = np.indices((480, 640), dtype=float)
    up = (240.0 - v) / F
    rays = [C + up * S, (320.0 - u) / F, -S + up * C]
    for eye, key in zip(EYES, ("left", "right")):
        start = np.full(v.shape, -np.inf)
        end = np.full(v.shape, np.inf)
        for i, (origin, ray) in enumerate(zip((-315.0, eye, 970.0), rays)):
            with np.errstate(divide="ignore", invalid="ignore"):
                a, b = (low[i] - origin) / ray, (high[i] - origin) / ray
            start = np.maximum(start, np.minimum(a, b))
            end = np.minimum(end, np.maximum(a, b))
        z = scene[key + "_z"]
        opaque = (start > 0) & (end >= start) & (start <= z)
        scene[key] = np.where(opaque, 0, scene[key]).astype(np.uint8)
        scene[key + "_z"] = np.where(opaque, start, z)
    return [*low.tolist(), *high.tolist()]
