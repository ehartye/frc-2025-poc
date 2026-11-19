"""Read a signed outbound stroke and reconstruct one differentiable position law."""

import hashlib
import numpy as np


def read_motion(path, expected_sha256):
    raw = path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != expected_sha256:
        raise ValueError("Unregistered achieved-motion input")
    import io

    rows = np.genfromtxt(io.BytesIO(raw), delimiter=",", names=True, ndmin=1)
    names = ("time_s", "stroke_m", "velocity_mps", "acceleration_mps2")
    if rows.dtype.names != names or len(rows) < 2:
        raise ValueError("Expected at least two position, velocity and acceleration records")
    data = np.column_stack([rows[k] for k in names])
    t, x, v, a = data.T
    h = np.diff(t)
    if (
        not np.isfinite(data).all()
        or abs(t[0]) > 1e-12
        or abs(x[0]) > 1e-6
        or min(h) <= 0
        or min(x) < 0
        or max(x) > 0.370001
        or min(np.diff(x)) < 0
    ):
        raise ValueError("Expected finite outbound history from stow")
    # The cubic on each interval matches measured position and velocity at
    # both ends. It independently predicts two endpoint accelerations. A
    # contradictory velocity, or alternating accelerations whose mean cancels,
    # cannot be hidden by shrinking the spacing between input timestamps.
    mean = np.diff(x) / h
    c2 = (3 * mean - 2 * v[:-1] - v[1:]) / h
    c3 = (v[:-1] + v[1:] - 2 * mean) / h**2
    left = 2 * c2
    right = left + 6 * c3 * h
    if not (
        np.allclose(left, a[:-1], rtol=0.02, atol=0.1)
        and np.allclose(right, a[1:], rtol=0.02, atol=0.1)
    ):
        raise ValueError(
            "Recorded derivatives disagree with the position and velocity reconstruction"
        )
    # Check the only interior extremum of quadratic segment velocity.
    test_t = np.zeros_like(h)
    np.divide(-c2, 3 * c3, where=c3 != 0, out=test_t)
    test_t = np.clip(test_t, 0, h)
    min_v = np.minimum(v[:-1] + test_t * (2 * c2 + 3 * c3 * test_t), np.minimum(v[:-1], v[1:]))
    if min(min_v) < -1e-8:
        raise ValueError("Motion reverses within an outbound segment")
    return data, c2, c3


def sample_motion(motion, dt):
    data, c2, c3 = motion
    t, x, v, _ = data.T
    duration = float(t[-1])
    count = int(np.floor(np.nextafter(duration / dt, np.inf)))
    grid = np.minimum(np.arange(count + 1) * dt, duration)
    idx = np.clip(np.searchsorted(t, grid, side="right") - 1, 0, len(c2) - 1)
    delta = grid - t[idx]
    return (
        duration,
        grid,
        x[idx] + delta * (v[idx] + delta * (c2[idx] + delta * c3[idx])),
        v[idx] + delta * (2 * c2[idx] + 3 * delta * c3[idx]),
        2 * c2[idx] + 6 * delta * c3[idx],
    )
