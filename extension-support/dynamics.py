"""Vertical receiver fixture under a prescribed outbound displacement; SI, positive Z up."""

import hashlib
import json
import os
from functools import lru_cache
from pathlib import Path
import sys

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "compliant-carriage"))
import model as carriage

plate = carriage.plate
G = 9.80665


def read(path):
    return json.loads(path.read_bytes())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def profile(t, duration=3.0):
    """Issued zero-rate, zero-acceleration endpoint quintic. Duration scaling is a control."""
    u = np.clip(np.asarray(t) / duration, 0, 1)
    return (
        0.370 * u**3 * (10 - 15 * u + 6 * u * u),
        0.370 / duration * 30 * u**2 * (1 - u) ** 2,
        0.370 / duration**2 * 60 * u * (1 - u) * (1 - 2 * u),
    )


def floors(s, geometry):
    # Intersection of the two translating straight extrusion lands. The switch
    # between limiting front edges occurs BETWEEN the sampled native poses.
    rear, tip = geometry["contacts"]["L"]["0"]["innerOnMiddle"]["downwardXMm"]
    outer_back, outer_front = geometry["contacts"]["L"]["0"]["middleOnOuter"]["downwardXMm"]
    return (
        rear / 1000 + s,
        np.minimum(tip / 1000 + s, outer_front / 1000 + s / 2),
        outer_back / 1000 + s / 2,
        np.full_like(s, outer_front / 1000),
    )


@lru_cache(maxsize=1)
def shape_model():
    # Match the previous plate Ritz build exactly to preserve its physical
    # plate coordinates in a projected restart. New joint kinematics replace
    # the previous two fore/aft-independent guide-line displacements per layer.
    previous = carriage.build(1.0, 5e6, 1.0, 2)
    shapes, omega, impulses = plate.plate(True, 13, 20, 2.2, 0.0, True)
    keep = omega <= 2300 * 2 * np.pi
    if sum(keep) != previous["nm"]:
        raise ValueError("Different retained plate space")
    x, y, areas = carriage.axes(2.2)
    t, h = plate.sections(x * 1000, y * 1000, True, True)
    mass = (plate.RHO_PE * t + plate.RHO_AL * h) * areas
    return shapes[:, keep], omega[keep], impulses[:, keep], mass, x, previous


def plate_at(x, y, dx=0, dy=0):
    return plate.basis(np.atleast_1d(x), np.atleast_1d(y), 13, 20, dx, dy) @ shape_model()[0]


def surface(x, y, derivative=False):
    """Rows taking generalized coordinates to receiver Z or fore/aft slope."""
    x, y = np.broadcast_arrays(np.atleast_1d(x), np.atleast_1d(y))
    f = plate_at(x, y, int(derivative))
    rows = np.zeros((len(x), 6 + f.shape[1]))
    rows[:, 4] = 0 if derivative else 1
    rows[:, 5] = 1 if derivative else x
    rows[:, 6:] = f
    return rows


def contact_rows(s, geo, keeper):
    """Integer order per joint is rear floor, front floor, opposed upper pin."""
    s = np.asarray(s)
    ir, ifront, mr, mfront = floors(s, geo)
    out = np.zeros((len(s), 6, 6))
    deriv = np.zeros_like(out)
    inner_tip = geo["contacts"]["L"]["0"]["innerOnMiddle"]["downwardXMm"][1] / 1000
    front_limit = geo["contacts"]["L"]["0"]["middleOnOuter"]["downwardXMm"][1] / 1000
    for j, (upper, lower, x, ratio, sign) in enumerate(
        [
            (4, 2, ir, 1.0, 1),
            (4, 2, ifront, np.where(inner_tip + s < front_limit + s / 2, 1.0, 0.5), 1),
            (4, 2, keeper["innerCrosspinLocalXMm"] / 1000 + s, 1.0, -1),
            (2, 0, mr, 0.5, 1),
            (2, 0, mfront, 0.0, 1),
            (2, 0, keeper["outerDogFixedXMm"] / 1000 + 0 * s, 0.0, -1),
        ]
    ):
        for i, orientation in [(upper, sign), (lower, -sign)]:
            rate = i / 4
            out[:, j, i] = orientation
            out[:, j, i + 1] = orientation * (x - rate * s)
            deriv[:, j, i + 1] = orientation * (ratio - rate)
    return out, deriv


def matrices(state, dressed=True, extra_cg=0.04):
    _, w, impulse, mp, grid_x, previous = shape_model()
    geo = read(HERE / "geometry.json")
    keep = read(HERE / "keepers.json")
    moving = read(ROOT / "integrated-slide/mass.json")["moving"]
    payload = read(HERE / "balance.json")["pipeMassKg"]
    n = 6 + len(w)
    M, K, C = (np.zeros((n, n)) for _ in range(3))
    gravity = np.zeros(n)
    moment = np.zeros(n)

    def rigid(i, m, x, variance, above_drive=0.0):
        M[i : i + 2, i : i + 2] += m * np.array([[1, x], [x, x * x + variance]])
        gravity[i : i + 2] -= m * G * np.array([1, x])
        moment[i + 1] += m * above_drive * (i / 4)

    points = np.asarray(read(ROOT / "compliant-carriage/boundary.json")["positionsMm"])[:, 0] / 1000
    B = np.column_stack([np.ones(4), points])
    K[:2, :2] = B.T @ previous["Kbase"] @ B
    rigid(0, previous["frameKg"], points.mean(), points.var())
    exact_plate = sum(v["massKg"] for k, v in geo["movingStock"].items() if k.startswith("r6-"))
    # Plate quadrature has its own consistent modal mass and inertia. Scale all
    # plate inertial terms together to the reopened paired CAD masses.
    scale = exact_plate / mp.sum()
    cross = np.stack((impulse[0], impulse[2] - 0.087 * impulse[0])) * scale
    M[4:6, 6:] = cross
    M[6:, 4:6] = cross.T
    M[6:, 6:] = np.eye(len(w)) * scale
    gravity[6:] = -G * cross[0]
    K[6:, 6:] = np.diag(w * w)
    C[6:, 6:] = np.diag(2 * 0.02 * w)
    rigid(
        4,
        exact_plate,
        np.average(grid_x, weights=mp),
        np.average((grid_x - np.average(grid_x, weights=mp)) ** 2, weights=mp),
        0.035,
    )
    masses = {"inner": exact_plate + payload, "middle": 0.0}
    for name, part in keep["movingStock"].items():
        m = part["massKg"]
        stage = part["stage"]
        masses[stage] += m
        # Bounding pitch inertia: substitute a uniform 320 mm beam for each
        # rail, retain the actual centroid; narrow transverse clips/axles are
        # treated as points in the XZ projection.
        variance = 0.320**2 / 12 if "slide" in name else 0.0
        rigid(4 if stage == "inner" else 2, m, part["stowedCenterXMm"] / 1000, variance)
    if dressed:
        for stage, i, published in [
            ("inner", 4, moving["outerWithAllowanceKg"]),
            ("middle", 2, moving["middleMassKg"]),
        ]:
            old = sum(
                v["massKg"]
                for k, v in geo["movingStock"].items()
                if v["stage"] == stage and not k.startswith("r6-")
            )
            new = sum(v["massKg"] for v in keep["movingStock"].values() if v["stage"] == stage)
            total = published + new - old
            extra = total - masses[stage]
            if extra < 0:
                raise ValueError("Double counted translating hardware")
            # Position and height of the unlocated dressed equipment are fixture
            # inputs; source accounting supplies a mass but no single centroid.
            rigid(i, extra, extra_cg, 0.080**2, 0.050)
            masses[stage] = total
    t, weights = np.polynomial.legendre.leggauss(16)
    a = np.asarray(state["a"])
    p = np.asarray(state["p"])
    xyz = p + (0.1508125 * t[:, None]) * a
    b = surface(xyz[:, 0], xyz[:, 1])
    # Fixed in-plane center and distributed vertical mass. A loss of positive
    # normal demand invalidates the assumed follower; this cannot grant custody.
    m = payload * weights / 2
    M += (m[:, None] * b).T @ b
    gravity -= G * (m @ b)
    moment[5] += payload * (p[2] - 0.467)
    min_eig = np.linalg.eigvalsh(M)[0]
    if min_eig <= 0:
        raise ValueError("Nonpositive fixture mass")
    return M, K, C, gravity, moment, b, weights / 2, masses


def project_legacy(state):
    """Old amplitudes are down-positive disturbances about the original dead-load datum."""
    old = shape_model()[-1]
    nm = old["nm"]
    pts = np.array(read(ROOT / "compliant-carriage/boundary.json")["positionsMm"])[:, 0] / 1000
    fit = np.column_stack([np.ones(4), pts])

    def convert(key):
        physical = -(old["basis"] @ np.asarray(state[key]))
        lands = physical[nm : nm + 4]
        rel = physical[nm + 4 :].reshape(2, 2)
        outer = np.linalg.lstsq(fit, lands, rcond=None)[0]
        mid = outer.copy()
        mid[0] += rel[0].mean()
        inner = mid.copy()
        inner[0] += rel[1].mean()
        residual = max(
            abs(lands - fit @ outer).max(), np.abs(rel - rel.mean(axis=1)[:, None]).max()
        )
        return np.r_[outer, mid, inner, physical[:nm]], float(residual)

    q, qr = convert("z")
    v, vr = convert("zdot")
    return q, v, qr, vr


@lru_cache(maxsize=1)
def button_table():
    # Sample the finite button face on each run of the installed machining
    # section. The maximum stock depth is the first downward interference.
    # Native 3-D common-volume witnesses below independently check this grid.
    s = np.arange(0.0, 222.25, 0.25)
    r = np.linspace(-3.2, 3.2, 33)
    xx, yy = np.meshgrid(r, r)
    ix = xx * xx + yy * yy <= 3.2**2 + 1e-12
    result = []
    for cy in (-66.0, 42.0):
        gaps = []
        for chunks in np.array_split(s, 30):
            x = 118 - chunks[:, None] + xx[ix]
            y = np.broadcast_to(cy + yy[ix], x.shape)
            pe, al = plate.sections(x.ravel(), y.ravel(), True, True)
            bottom = 500 - 1000 * (pe + al).reshape(x.shape)
            bottom[(x < -100) | (x > 165)] = np.inf
            gaps.extend((bottom.min(axis=1) - 484.9) / 1000)
        result.append(np.array(gaps))
    return s / 1000, np.array(result)


def button_gaps(s):
    grid, table = button_table()
    idx = np.clip(np.rint(np.asarray(s) / 0.00025).astype(int), 0, len(grid) - 1)
    return np.where(np.asarray(s)[:, None] >= 0.222, np.inf, table[:, idx].T)
