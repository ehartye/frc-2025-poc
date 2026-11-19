"""Bound a convex projected pipe end with every pair of independently visible edge samples."""

import numpy as np


def peak(ends, first, last, error):
    rows, values = np.array(ends, dtype=float).T
    gap = rows[None, :] - rows[:, None]
    valid = gap > 0
    slope_lo = np.full_like(gap, -np.inf)
    slope_hi = np.full_like(gap, np.inf)
    delta = values[None, :] - values[:, None]
    np.divide(delta - 2 * error, gap, out=slope_lo, where=valid)
    np.divide(delta + 2 * error, gap, out=slope_hi, where=valid)
    # A concave edge lies below each extrapolated secant on both OUTSIDE intervals.
    # Retain the tightest slope at the selected endpoint, with both endpoint errors.
    left = np.max(slope_lo, axis=1)
    right = np.min(slope_hi, axis=0)
    q = np.arange(first, last + 1)[:, None]
    dist = q - rows
    a = dist + 0.5 * np.sign(left)
    b = dist + 0.5 * np.sign(right)
    edges = np.broadcast_to(values + error, dist.shape)
    left_upper = np.full_like(dist, np.inf)
    right_upper = np.full_like(dist, np.inf)
    left_ok = (dist <= -0.5) & np.isfinite(left)
    right_ok = (dist >= 0.5) & np.isfinite(right)
    np.multiply(a, left, out=left_upper, where=left_ok)
    np.multiply(b, right, out=right_upper, where=right_ok)
    return float((edges + np.minimum(left_upper, right_upper)).min(axis=1).max())
