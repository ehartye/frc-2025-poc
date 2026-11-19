"""Four-corner wheel loads for the stowed swerve project, SI units."""

from itertools import product
import numpy as np

G = 9.80665


class Support:
    """Fast load-transfer callback with registered static scale shares and rates."""

    def __init__(self, xy, mass, shares, z, rates):
        xy = np.asarray(xy, dtype=float)
        shares = np.asarray(shares, dtype=float)
        rates = np.asarray(rates, dtype=float)
        if (
            xy.shape != (4, 2)
            or rates.shape != (4,)
            or shares.shape != (4,)
            or mass <= 0
            or z <= 0
            or min(shares) <= 0
            or min(rates) <= 0
            or abs(shares.sum() - 1) > 1e-9
        ):
            raise ValueError("Invalid four-corner static registration")
        self.mass = mass
        self.z = z
        self.cg = shares @ xy
        self.static = mass * G * shares
        self.xy = xy
        self.k = rates
        A = np.vstack((np.ones(4), xy.T))
        self.B = (rates[:, None] * A.T) @ np.linalg.inv((A * rates) @ A.T)
        self.heights = self.static / rates

    def forces(self, ax, ay, omega, alpha):
        """Origin acceleration and yaw in robot axes, both accelerations inertial."""
        cx, cy = self.cg
        axg = ax - alpha * cy - omega**2 * cx
        ayg = ay + alpha * cx - omega**2 * cy
        w = -self.mass * self.z * np.array([axg, ayg])
        n = self.static + self.B[:, 1:] @ w
        if min(n) >= 0:
            return n
        return corner_loads(
            self.xy, self.mass, [cx, cy, self.z], [ax, ay], omega, alpha, self.k, self.heights
        )


def corner_loads(positions, mass, cg, origin_accel, omega, alpha, rates, heights):
    """Rigid deck, independent linear vertical tread springs on level floor.

    heights means the wheel protrudes down relative to the mean carriage plane.
    Enforce unilateral vertical contact, pitch and roll equilibrium. Centripetal
    and yaw acceleration at the offset CG use chassis axes.
    """
    xy = np.asarray(positions, dtype=float)
    k = np.asarray(rates, dtype=float)
    h = np.asarray(heights, dtype=float)
    if xy.shape != (4, 2) or k.shape != (4,) or h.shape != (4,) or mass <= 0 or min(k) <= 0:
        raise ValueError("Expected four positive springs and a positive mass")
    cx, cy, z = cg
    acc = np.asarray(origin_accel) + np.array(
        [-alpha * cy - omega**2 * cx, alpha * cx - omega**2 * cy]
    )
    projected = np.array([cx, cy]) - z * acc / G
    A = np.vstack((np.ones(4), xy.T))
    wrench = mass * G * np.r_[1, projected]
    for omitted in (None, 0, 1, 2, 3):
        ix = np.arange(4) if omitted is None else np.delete(np.arange(4), omitted)
        R = A[:, ix]
        stiffness = k[ix]
        try:
            plane = np.linalg.solve((R * stiffness) @ R.T, wrench - R @ (stiffness * h[ix]))
        except np.linalg.LinAlgError:
            continue
        penetration = h + A.T @ plane
        force = k * np.maximum(0.0, penetration)
        if np.allclose(A @ force, wrench, atol=1e-6, rtol=0):
            return force
    raise ValueError("No stable unilateral support polygon")


def load_bound(
    xy, mass, cg, acc, omega, alpha, rate_fraction=0.1, height_error=0.00025, rate=150000.0
):
    """Enclose the four-contact solution; zeros flag possible wheel unloading.

    The single redundant force mode permits analytic interval endpoints.
    height error is independent per corner. At four contacts f=f0+c*u,
    and compatibility gives c=([u.h]-[u.f0/k])/[sum(u*u/k)].
    This linear-fractional function achieves extrema at endpoints of 1/k.
    If any four-contact lower limit reaches zero, each positive-traction
    guarantee is conservatively dropped; tipping and tripod loads need replay.
    """
    A = np.vstack((np.ones(4), np.asarray(xy).T))
    if np.linalg.matrix_rank(A) != 3 or rate <= 0 or not 0 <= rate_fraction < 1 or height_error < 0:
        raise ValueError("Degenerate module geometry or compliance")
    u = np.linalg.svd(A, full_matrices=True)[2][-1]
    low = np.full(4, np.inf)
    high = -low
    for cx, cy, z in product(*cg):
        ax = acc[0] - alpha * cy - omega**2 * cx
        ay = acc[1] + alpha * cx - omega**2 * cy
        w = mass * G * np.array([1, cx - z * ax / G, cy - z * ay / G])
        f0 = np.linalg.lstsq(A, w, rcond=None)[0]
        for factors in product((1 - rate_fraction, 1 + rate_fraction), repeat=4):
            inv = 1 / (rate * np.array(factors))
            denom = (u * u) @ inv
            center = -(u * f0) @ inv / denom
            radius = sum(abs(u)) * height_error / denom
            lo = f0 + u * center - abs(u) * radius
            hi = f0 + u * center + abs(u) * radius
            low = np.minimum(low, lo)
            high = np.maximum(high, hi)
    return low, high


def preloaded_bounds(xy, mass, cg, acc, omega, alpha, min_share=0.22, rate_fraction=0.1):
    """Incremental force from a four-pad static acceptance; constant height errors cancel.

    With all corners still loaded the acceptance also bounds each static maximum
    at 1-3*min_share. Walk acceleration from zero and only credit positive minima.
    """
    xy = np.asarray(xy)
    A = np.vstack((np.ones(4), xy.T))
    if not 0 <= min_share <= 0.25:
        raise ValueError("Four corner static shares do not close")
    changes = []
    for factors in product((1 - rate_fraction, 1 + rate_fraction), repeat=4):
        k = np.array(factors)
        transfer = (k[:, None] * A.T) @ np.linalg.inv((A * k) @ A.T)
        for cx, cy, z in product(*cg):
            ax = acc[0] - alpha * cy - omega**2 * cx
            ay = acc[1] + alpha * cx - omega**2 * cy
            changes.append(transfer @ np.array([0.0, -mass * z * ax, -mass * z * ay]))
    return mass * G * min_share + np.min(changes, axis=0), mass * G * (1 - 3 * min_share) + np.max(
        changes, axis=0
    )
