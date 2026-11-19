"""Anchor fixed-camera heading to surveyed features and transport it with the chassis gyro."""

from itertools import combinations, product
import math
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "mounted-guide-vision"))
from sector import VisibleFasteners, nav, coarse_observation
import vision

PIXEL_ERROR = 0.1805
from rate import RATE_BIAS, GAIN_ERROR, RatePackets

OFFSETS = np.array(list(product((-PIXEL_ERROR, PIXEL_ERROR), repeat=2)))


def body_patch(points, eye, target, measured):
    return vision.unproject(
        measured + OFFSETS, np.full(4, points[target, 2]), vision.EYES[eye]
    )


def optical_yaw(rays, points):
    """Intersect bounds on the rotation between all pairs of different landmarks."""
    lower, upper = -math.inf, math.inf
    for (e, i, a), (f, j, b) in combinations(rays, 2):
        if i == j:
            continue
        pa, pb = body_patch(points, e, i, a), body_patch(points, f, j, b)
        world = points[j, :2] - points[i, :2]
        differences = (pb[:, None, :] - pa[None, :, :]).reshape(-1, 2)
        angles = np.arctan2(
            differences[:, 0] * world[1] - differences[:, 1] * world[0],
            differences @ world,
        )
        if np.ptp(angles) > math.pi:
            return None
        lower = max(lower, angles.min())
        upper = min(upper, angles.max())
    if not math.isfinite(lower) or lower > upper:
        return None
    return float(lower), float(upper)


def position(rays, points, heading, goal):
    """Enclose every robot center consistent with the visible corners and gyro interval."""
    lo, hi = heading
    count = max(1, math.ceil((hi - lo) / math.radians(0.03)))
    edges = np.linspace(lo, hi, count + 1)
    mid = (edges[1:] + edges[:-1]) / 2
    cs, sn = np.cos(mid), np.sin(mid)
    lows = np.full((count, 2), -np.inf)
    highs = -lows.copy()
    for eye, target, measured in rays:
        xy = body_patch(points, eye, target, measured)
        x, y = xy.T
        pad = np.max(np.linalg.norm(xy, axis=1)) * (hi - lo) / (2 * count) + 0.001
        px = points[target, 0] - cs[:, None] * x + sn[:, None] * y
        py = points[target, 1] - sn[:, None] * x - cs[:, None] * y
        lows = np.maximum(lows, np.column_stack((px.min(axis=1), py.min(axis=1))) - pad)
        highs = np.minimum(
            highs, np.column_stack((px.max(axis=1), py.max(axis=1))) + pad
        )
    valid = np.all(lows <= highs, axis=1)
    if not np.any(valid):
        return None
    corners = np.maximum(np.abs(lows[valid] - goal), np.abs(highs[valid] - goal))
    radius = np.linalg.norm(corners, axis=1).max()
    return (
        np.r_[lows[valid].min(axis=0), edges[:-1][valid].min()],
        np.r_[highs[valid].max(axis=0), edges[1:][valid].max()],
        float(radius),
    )


class RegisteredRays(list):
    pass


class Observer(VisibleFasteners):
    def __init__(self, scene, calibration, fault):
        super().__init__(scene)
        self.gyro = RatePackets(
            calibration["gyroBiasDegS"], calibration["gyroScaleError"], fault
        )
        self.anchors = []
        self.used = []
        self.stationary_pose = None

    def identify(self, detections, prediction, capture):
        super().identify(detections, prediction, capture)
        record = self.observations[-1]
        if record["ambiguous"]:
            return None
        # Stored centers are quantized to 0.001 pixel. OFFSETS includes the
        # additional half-quantum in the enclosure.
        rays = RegisteredRays(
            (e, j, np.array([u, v])) for e, j, u, v in record["features"]
        )
        if not rays:
            return None
        packet = self.gyro.at_capture(capture)
        if packet is None:
            return None
        angle, endpoint_error = packet
        visible_bound = optical_yaw(rays, self.points)
        if visible_bound is not None:
            self.anchors.append((capture, angle, endpoint_error, visible_bound))
        self.anchors = [a for a in self.anchors if capture - a[0] <= 1800]
        if not self.anchors:
            return None
        low, high = -math.inf, math.inf
        for stamp, initial_angle, initial_error, (
            anchor_low,
            anchor_high,
        ) in self.anchors:
            delta = angle - initial_angle
            drift = RATE_BIAS * (capture - stamp) / 1000
            uncertainty = (
                drift + abs(delta) * GAIN_ERROR + endpoint_error + initial_error
            ) / (1 - GAIN_ERROR)
            low = max(low, anchor_low + delta - uncertainty)
            high = min(high, anchor_high + delta + uncertainty)
        if low > high:
            return None
        bounds = position(rays, self.points, (low, high), np.zeros(2))
        if bounds is None:
            return None
        rays.heading = low, high
        self.stationary_pose = (bounds[0] + bounds[1]) / 2
        self.used.append(
            [int(capture), len(set(j for _, j, _ in rays)), math.degrees(high - low)]
        )
        return rays

    def pose(self, rays, prediction):
        return self.stationary_pose

    def bounds(self, rays, goal):
        if len({e for e, _, _ in rays}) < 2:
            return None
        return position(rays, self.points, rays.heading, goal)
