"""Ritz plate replay of the registered pipe line loads; chassis axes, SI internally."""

import argparse
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
import numpy as np
from numpy.polynomial.legendre import Legendre
from tray import OUTLINE, LANDS, RIB_X, RIB_Y, RIB, RADIUS, FIT, SKIN

HERE = Path(__file__).resolve().parent
LENGTH, SPAN = 0.265, 0.360
X0, Y0 = -0.100, -0.180
E_PE, NU_PE, RHO_PE = 1.034214e9, 0.42, 940.0
E_AL, NU_AL, RHO_AL = 68.3e9, 0.33, 2700.0
# Section comparisons assume no shear coupling between the polymer and metal.
# Inside either homogeneous layer bending is continuous; screws retain the faces.
DT = 0.000015625


def inside(x, y, rect, rad=0.0):
    a, b, c, d = rect
    return (x - np.clip(x, a + rad, c - rad)) ** 2 + (
        y - np.clip(y, b + rad, d - rad)
    ) ** 2 <= rad**2 + 1e-12


def sections(x, y, hybrid, installed=False):
    t = np.full(x.shape, 15.0)
    metal = np.zeros_like(t)
    if hybrid:
        pocket = np.zeros_like(x, dtype=bool)
        footprint = pocket.copy()
        for a, b, c, d in OUTLINE:
            pocket |= inside(x, y, (a - FIT, b - FIT, c + FIT, d + FIT), RADIUS + FIT)
            footprint |= inside(x, y, (a, b, c, d), RADIUS)
        t[pocket] = 4.0
        metal[footprint] = 11.0
        xs = [-93, *RIB_X, 153]
        ys = [-168, *RIB_Y, 168]
        machined = np.zeros_like(footprint)
        for a, c in zip(xs, xs[1:]):
            for b, d in zip(ys, ys[1:]):
                if min(c - a - RIB, d - b - RIB) > 2 * RADIUS:
                    machined |= inside(
                        x,
                        y,
                        (a + RIB / 2, b + RIB / 2, c - RIB / 2, d - RIB / 2),
                        RADIUS,
                    )
        for r in LANDS:
            machined &= ~inside(x, y, r)
        metal[machined & footprint] = SKIN
        if installed:
            for yy in (-66.0, 42.0):
                metal[inside(x, y, (113, yy - 5, 123, yy + 5), 2.0)] = 11.0
                lane = (x < 115) & (abs(y - yy) < 5.0)
                cut = np.clip((115 - x) * (0.8 / 46), 0, 0.8) * lane
                t -= cut * (t > 10.0)
                metal -= cut * (metal > 10.0)
            for yy in (-70.0, 0.0, 70.0):
                altered = inside(x, y, (124.85, yy - 2.65, 155.15, yy + 2.65)) | (
                    (x - 155) ** 2 + (y - yy) ** 2 < 3.75**2
                )
                t[altered] -= 0.8
    for a, b, r in [(40, 0, 6), (100, 0, 6), (96, -127, 1.65), (112, -127, 1.65)] + [
        (xx, yy, 2.65) for xx in (-80, 140) for yy in (-180, 180)
    ]:
        hole = (x - a) ** 2 + (y - b) ** 2 < r * r
        t[hole] = 0
        metal[hole] = 0
    return t / 1000, metal / 1000


def basis(x, y, nx, ny, dx=0, dy=0):
    xx = 2 * (x - X0) / LENGTH - 1
    a = np.column_stack(
        [Legendre.basis(k).deriv(dx)(xx) * (2 / LENGTH) ** dx for k in range(nx)]
    )
    k = np.arange(1, ny + 1) * np.pi / SPAN
    b = np.sin(np.outer(y - Y0, k) + dy * np.pi / 2) * k**dy
    return (a[:, :, None] * b[:, None, :]).reshape((len(x), nx * ny))


def axis_cells(lo, hi, breaks, spacing):
    edges = np.unique(np.clip([lo, hi, *breaks], lo, hi))
    cells = []
    for a, b in zip(edges, edges[1:]):
        cells.extend(np.linspace(a, b, int(np.ceil((b - a) / spacing)) + 1)[:-1])
    nodes = np.r_[cells, hi]
    return (nodes[:-1] + nodes[1:]) / 2000, np.diff(nodes) / 1000


def plate(hybrid, nx, ny, spacing=2.5, seat_stiffness=0.0, installed=False):
    # Conform cells to stock boundaries and rib faces. A uniform mesh can alias
    # a 3 mm rib by an entire cell at just one edge and bias stiffness.
    ax = [z + f for z in [-93, *RIB_X, 153] for f in [-RIB / 2, RIB / 2]]
    ay = [z + f for z in [-168, *RIB_Y, 168] for f in [-RIB / 2, RIB / 2]]
    for a, b, c, d in OUTLINE + LANDS:
        ax.extend([a - FIT, a, c, c + FIT])
        ay.extend([b - FIT, b, d, d + FIT])
    if installed:
        ax.extend([69.0, 113.0, 115.0, 123.0, 124.85, 155.15])
        ay.extend([v + p for v in (-66.0, 42.0) for p in (-5.0, 5.0)])
    xx, wx = axis_cells(-100, 165, ax, spacing)
    yy, wy = axis_cells(-180, 180, ay, spacing)
    x, y = np.meshgrid(xx, yy)
    x, y = x.ravel(), y.ravel()
    t, h = sections(x * 1000, y * 1000, hybrid, installed)
    area = np.outer(wy, wx).ravel()
    mp = (RHO_PE * t + RHO_AL * h) * area
    dp = E_PE * t**3 / (12 * (1 - NU_PE**2)) * area
    da = E_AL * h**3 / (12 * (1 - NU_AL**2)) * area
    d = dp + da
    v = dp * NU_PE + da * NU_AL
    b = basis(x, y, nx, ny)
    bx = basis(x, y, nx, ny, 2, 0)
    by = basis(x, y, nx, ny, 0, 2)
    xy = basis(x, y, nx, ny, 1, 1)
    mass = b.T @ (mp[:, None] * b)
    stiff = bx.T @ (d[:, None] * bx + v[:, None] * by) + by.T @ (
        d[:, None] * by + v[:, None] * bx
    )
    stiff += xy.T @ ((2 * (d - v))[:, None] * xy)
    if seat_stiffness:
        if seat_stiffness > 0:
            xy = np.array([[0.132, -0.082], [0.132, 0.082]])
        else:
            # An optimistic grounded crossmember under the arrival line replaces
            # two individual seats in this design sensitivity.
            xy = np.array([[0.116, y] for y in (-0.135, -0.045, 0.045, 0.135)])
        seat = basis(xy[:, 0], xy[:, 1], nx, ny)
        stiff += abs(seat_stiffness) * (seat.T @ seat)
    ch = np.linalg.cholesky(mass)
    left = np.linalg.solve(ch, stiff)
    k = np.linalg.solve(ch, left.T).T
    ev, u = np.linalg.eigh((k + k.T) / 2)
    mode = np.linalg.solve(ch.T, u)
    # Integral of each mode's vertical mass, plus first moments about Y0 and rear lock.
    inertia = np.stack((mp, mp * (y - Y0) / SPAN, mp * (x + 0.087))) @ b @ mode
    return mode, np.sqrt(ev), inertia


def forced(q, omega, damping, dt):
    """Exact transition under a held force during each registered integration step."""
    w = omega
    wd = w * np.sqrt(1 - damping**2)
    exp = np.exp(-damping * w * dt)
    sn = exp * np.sin(wd * dt) / wd
    cs = exp * np.cos(wd * dt)
    aa = cs + damping * w * sn
    bb = cs - damping * w * sn
    disp = np.zeros_like(q)
    vel = np.zeros(q.shape[1])
    for i in range(len(q) - 1):
        p = q[i] / w**2
        u = disp[i] - p
        disp[i + 1] = aa * u + sn * vel + p
        vel = bb * vel - w**2 * sn * u
    # Differentiate the state analytically for each load interval.
    accel = np.zeros_like(q)
    vel = np.zeros_like(vel)
    for i in range(len(q) - 1):
        accel[i] = q[i] - 2 * damping * w * vel - w**2 * disp[i]
        vel = bb * vel - w**2 * sn * (disp[i] - q[i] / w**2)
    accel[-1] = q[-1] - 2 * damping * w * vel - w**2 * disp[-1]
    return disp, accel


def trace(csv_file, source):
    if hashlib.sha256(csv_file.read_bytes()).hexdigest() != source["stationsSha256"]:
        raise ValueError("Unregistered distributed load " + csv_file.name)
    data = np.loadtxt(csv_file, delimiter=",", skiprows=1)
    # Integrate this first pulse only, under the contact producer's 1%-of-peak convention.
    data = data[
        (data[:, 0] >= source["startFromLipS"] - 1e-10)
        & (data[:, 0] < source["endFromLipS"] - 1e-10)
    ]
    idx = np.rint((data[:, 0] - data[:, 0].min()) / source["dt"]).astype(int)
    return idx, data


def stresses(disp, static, mode, nx, ny, hybrid):
    # Gauge points on the rib, open pocket and edge rail. These are nominal
    # plate extreme-fibre stresses; no notch, bond or bolt allowable is implied.
    x = np.array([115, 121, 140, 115, 115, 92, 60, 25, 115, 140, 140], float) * 0.001
    y = np.array([0, 20, 55, 120, 164, 80, 40, -55, -164, -178, 178], float) * 0.001
    pe, al = sections(x * 1000, y * 1000, hybrid)
    fields = [basis(x, y, nx, ny, a, b) @ mode for a, b in ((2, 0), (0, 2), (1, 1))]

    def peak(state, e, nu, t):
        kx, ky, kxy = (state @ m.T for m in fields)
        scale = e * t / (2 * (1 - nu**2))
        sx = scale * (kx + nu * ky)
        sy = scale * (ky + nu * kx)
        tau = scale * (1 - nu) * kxy
        return np.sqrt(sx * sx - sx * sy + sy * sy + 3 * tau * tau).max(axis=0) * 1e-6

    return {
        "gaugeXYmm": np.column_stack((x * 1000, y * 1000)).tolist(),
        "staticPeMPa": peak(static[None, :], E_PE, NU_PE, pe).tolist(),
        "dynamicPeMPa": peak(disp, E_PE, NU_PE, pe).tolist(),
        "staticAlMPa": peak(static[None, :], E_AL, NU_AL, al).tolist(),
        "dynamicAlMPa": peak(disp, E_AL, NU_AL, al).tolist(),
    }


def evaluate(model, nx, ny, rows, dt, damping, hybrid):
    mode, w, inertia = model
    idx, d = rows
    n = len(w)
    nsteps = int(np.ceil(0.12 / dt)) + idx.max() + 1
    forces = np.zeros((nsteps, 3))
    # Column 7 is negative when loaded down. All other forces are chassis-frame.
    down = -d[:, 7]
    np.add.at(
        forces,
        idx,
        np.column_stack(
            (
                down,
                down * (d[:, 3] * 0.001 - Y0) / SPAN,
                down * (0.001 * d[:, 2] + 0.087),
            )
        ),
    )
    phi = basis(d[:, 2] / 1000, d[:, 3] / 1000, nx, ny) @ mode
    loads = np.zeros((nsteps, n))
    np.add.at(loads, idx, down[:, None] * phi)
    peak = forces[:, 0].argmax()
    spatial = loads[peak] / forces[peak, 0]
    static = loads[peak] / w**2
    static_travel = static @ spatial
    x = np.array([0.1164, 0.100, 0.040, 0.1164, 0.1164])
    y = np.array([0.0, 0.0, 0.0, -0.120, 0.120])
    probes = basis(x, y, nx, ny) @ mode
    disp, acc = forced(loads, w, damping, dt)
    displacement = disp @ probes.T
    reactions = forces - acc @ inertia.T
    left = reactions[:, 1]
    right = reactions[:, 0] - left
    center = reactions[:, 2]
    return {
        "firstHz": float(w[0] / (2 * np.pi)),
        "retainedModes": n,
        "pulseImpulseNs": float(forces[:, 0].sum() * dt),
        "peakForceN": float(forces[peak, 0]),
        "peakShapeStaticStiffnessNPerM": float(forces[peak, 0] / static_travel),
        "peakShapeStaticMeanMm": float(static_travel * 1000),
        "staticTouchdownCenterMm": float(probes[0] @ static * 1000),
        "probePeakDownMm": (displacement.max(axis=0) * 1000).tolist(),
        "probePeakUpMm": (-displacement.min(axis=0) * 1000).tolist(),
        "peakMeanTravelMm": float((disp @ spatial).max() * 1000),
        "zeroCrossingAfterFirstDownS": first_return(displacement[:, 0], dt),
        "railLeftDownUpN": [float(left.max()), float(-left.min())],
        "railRightDownUpN": [float(right.max()), float(-right.min())],
        "maxCombinedDownN": float(reactions[:, 0].max()),
        "gauges": stresses(disp, static, mode, nx, ny, hybrid),
        "rearVerticalLoadPitchMaxAbsNm": float(np.max(np.abs(center))),
        "staticPitchAtForcePeakNm": float(forces[peak, 2]),
    }


def first_return(series, dt):
    low = np.flatnonzero(series[series.argmax() :] < 0)
    return float((series.argmax() + low[0]) * dt) if len(low) else None


def run(load_path, record, soft_path, soft_record):
    base = json.loads(record.read_bytes())
    sources = [
        (r, load_path / (r["case"] + "-stations.csv"), "400k")
        for r in base
        if r["case"] in ("leftSmall", "rightSmall")
    ]
    softer = json.loads(soft_record.read_bytes())
    for r in softer:
        fp = next(
            (
                p
                for p in soft_path.glob("*-stations.csv")
                if hashlib.sha256(p.read_bytes()).hexdigest() == r["stationsSha256"]
            ),
            None,
        )
        if fp is None:
            raise ValueError("Missing signed soft-interface trace")
        sources.append((r, fp, fp.name.removesuffix("-stations.csv")))
    rows = []
    for hybrid in (False, True):
        for nx, ny, dx in [(7, 12, 1.8), (11, 18, 2.2), (13, 20, 2.2), (15, 22, 2.2)]:
            m = plate(hybrid, nx, ny, dx)
            for r, path, law in sources:
                if nx != 13 and (r["case"] != "leftSmall" or law != "400k"):
                    continue
                for zeta in [0.02, 0.08] if nx == 13 and law == "400k" else [0.02]:
                    out = evaluate(m, nx, ny, trace(path, r), r["dt"], zeta, hybrid)
                    rows.append(
                        {
                            "hardware": "insert" if hybrid else "stock",
                            "nx": nx,
                            "ny": ny,
                            "integrationCellMm": dx,
                            "dampingRatio": zeta,
                            "law": law,
                            "case": r["case"],
                            "loadSha256": r["stationsSha256"],
                            **out,
                        }
                    )
    return {
        "supportsYmm": [-180, 180],
        "probeXYmm": [[116.4, 0], [100, 0], [40, 0], [116.4, -120], [116.4, 120]],
        "idealUpperPlateModel": True,
        "physicalDeformationMm": None,
        "cases": rows,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for key in ("load_path", "record", "soft_path", "soft_record", "out"):
        p.add_argument(key, type=Path)
    a = p.parse_args()
    result = run(a.load_path, a.record, a.soft_path, a.soft_record)
    a.out.write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
