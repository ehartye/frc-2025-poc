"""Estimate a camera exposure change using only surveyed, fixed robot pixels."""

import numpy as np


def encode(image, gain, offset):
    return np.clip(np.rint(image.astype(float) * gain + offset), 0, 255).astype(
        np.uint8
    )


def samples(empty, eligible):
    yy, xx = np.indices(empty.shape)
    grid = eligible & (xx % 4 == 0) & (yy % 4 == 0)
    split = (xx // 32 + yy // 32) % 2
    groups = []
    for level in np.unique(empty[grid]):
        if not 35 <= level <= 180:
            continue
        same = grid & (empty == level)
        positions = [np.flatnonzero(same & (split == s)) for s in (0, 1)]
        if min(map(len, positions)) >= 12:
            groups.append(
                (int(level), *[p[:: max(1, len(p) // 200)] for p in positions])
            )
    return groups


def normalize(image, groups):
    """Use training clusters to solve an affine response and disjoint tiles to test it."""
    if len(groups) < 3 or groups[-1][0] - groups[0][0] < 35:
        return None, {"status": "insufficient-surface-range"}
    flat = image.ravel()
    levels = np.array([g[0] for g in groups])
    measured = np.array([np.median(flat[training]) for _, training, _ in groups])
    slopes = [
        (measured[j] - measured[i]) / (levels[j] - levels[i])
        for i in range(len(groups))
        for j in range(i + 1, len(groups))
        if levels[j] - levels[i] >= 24
    ]
    gain = float(np.median(slopes))
    offset = float(np.median(measured - gain * levels))
    errors = [
        float(np.quantile(abs(flat[held].astype(float) - (gain * v + offset)), 0.90))
        for v, _, held in groups
    ]
    receipt = {
        "status": "validated",
        "gain": gain,
        "offset": offset,
        "maxValidationError": max(errors),
        "surfaceLevels": len(levels),
        "span": int(np.ptp(levels)),
    }
    if not 0.55 <= gain <= 1.35 or not -20 <= offset <= 25 or max(errors) > 3.0:
        return None, {**receipt, "status": "outside-survey"}
    restored = np.clip(np.rint((image.astype(float) - offset) / gain), 0, 255).astype(
        np.uint8
    )
    return restored, receipt
