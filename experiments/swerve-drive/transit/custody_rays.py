"""Screen finite non-imaging through-beams against the actual hollow pipe and R4 body bounds."""

from itertools import product
import math
import numpy as np


def wall_intercepts(center, axis, x, z):
    """Open-bore tube with an annular end face. Finite transmitter/receiver segment in Y."""
    center = np.asarray(center)
    axis = np.asarray(axis)
    v = np.array([x, 0.0, z]) - center
    dot = v @ axis
    lo, hi = -185.0, 185.0
    if abs(axis[1]) < 1e-7:
        if abs(dot) > 150.8125:
            return False
    else:
        ends = sorted([(-150.8125 - dot) / axis[1], (150.8125 - dot) / axis[1]])
        lo, hi = max(lo, ends[0]), min(hi, ends[1])
    if lo > hi:
        return False

    def radial(y):
        d = v + np.array([0.0, y, 0.0])
        return d @ d - (d @ axis) ** 2

    k = 1 - axis[1] ** 2
    nearest = lo if k < 1e-12 else np.clip(-(v[1] - dot * axis[1]) / k, lo, hi)
    return bool(radial(nearest) <= 57.15**2 and max(radial(lo), radial(hi)) >= 50.8**2)


def ray_screen(cases, hard, solids):
    from premise import possible_overlaps

    def poses(c):
        return (np.asarray(c["centerMm"]), np.asarray(c["axis"]))

    sources = {c["case"]: poses(c) for c in cases}
    sources["attained-rear-stop"] = (
        np.array(hard["lastCenterMm"]),
        np.array(hard["lastAxis"]),
    )
    sources["24mm-forward-control"] = (
        sources["plus"][0] + [24, 0, 0],
        sources["plus"][1],
    )
    sources["24mm-transverse-control"] = (
        sources["plus"][0] + [0, 24, 0],
        sources["plus"][1],
    )
    sources["seven-degree-yaw-control"] = (
        sources["plus"][0],
        np.array([math.sin(math.radians(7)), math.cos(math.radians(7)), 0]),
    )
    sources["identical-pose-grip-failed"] = sources["plus"]
    result = []
    for name, x, z in (
        ("occupancy", 75, 611),
        ("rear-limit", 26, 594),
        ("front-limit", 145, 557),
    ):
        masks = {}
        for label, (center, axis) in sources.items():
            masks[label] = [
                wall_intercepts(center, axis, x + dx, z + dz)
                for dx, dz in product((-1, 0, 1), repeat=2)
            ]
        result.append(
            dict(
                name=name,
                centerXzMm=[x, z],
                yEndpointsMm=[-185, 185],
                rawBeamStates=masks,
                emptyBlocked=False,
                stowedBroadphaseCandidates=possible_overlaps(
                    [[x - 1, -185, z - 1, x + 1, 185, z + 1]], solids
                ),
            )
        )
    return result
