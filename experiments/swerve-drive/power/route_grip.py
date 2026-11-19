"""Challenge an achieved swerve route against the conditional four-pad load envelope."""

import argparse
import csv
from itertools import product
from pathlib import Path
import numpy as np
from balance import centers, parcel_bounds
from loads import G
from electrical import IDS, HERE
from mass import read
from screen import sha, write


def accepted_normals(xy, cg, mass, acc, omega, alpha, share=0.22, kspread=0.1):
    n = len(omega)
    low = np.full((n, 4), np.inf)
    A = np.vstack((np.ones(4), xy.T))
    for c in product(*cg):
        ax = acc[:, 0] - alpha * c[1] - omega**2 * c[0]
        ay = acc[:, 1] + alpha * c[0] - omega**2 * c[1]
        w = -mass * c[2] * np.stack((ax, ay), axis=1)
        for rates in product((1 - kspread, 1 + kspread), repeat=4):
            k = np.asarray(rates)
            transfer = (k[:, None] * A.T) @ np.linalg.inv((A * k) @ A.T)
            low = np.minimum(low, mass * G * share + w @ transfer[:, 1:].T)
    return low


def verify(path, registered):
    if sha(path) != registered:
        raise ValueError("Unregistered swerve motion")
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if len(rows) < 8 or len(rows) % 4:
        raise ValueError("Incomplete motion grid")
    wall = []
    normals = []
    forces = []
    speed = []
    xy = None
    for i in range(0, len(rows), 4):
        records = {r["id"]: r for r in rows[i : i + 4]}
        if set(records) != set(IDS):
            raise ValueError("Corner order missing")
        items = [records[k] for k in IDS]
        state = np.array(
            [
                [float(r[k]) for k in ("wall_s", "body_vx_mps", "body_vy_mps", "body_omega_radps")]
                for r in items
            ]
        )
        if not np.isfinite(state).all() or not np.allclose(state, state[0], atol=1e-10, rtol=0):
            raise ValueError("Asynchronous corners")
        here = np.array([[float(r["x_m"]), float(r["y_m"])] for r in items])
        if not np.allclose(here, centers(), atol=1e-6, rtol=0):
            raise ValueError("Different wheel stations")
        xy = here
        wall.append(state[0, 0])
        speed.append(state[0, 1:])
        forces.append(
            [np.hypot(float(r["drive_force_n"]), float(r["lateral_force_n"])) for r in items]
        )
        normals.append([float(r["normal_n"]) for r in items])
    wall = np.array(wall)
    speed = np.array(speed)
    forces = np.array(forces)
    normals = np.array(normals)
    if min(np.diff(wall)) <= 0 or max(np.diff(wall)) > 0.00501:
        raise ValueError("Nonmonotone or absent 5 ms interval")
    dto = np.diff(wall)[:, None]
    deriv = np.diff(speed, axis=0) / dto
    balance = read(HERE / "balance.json")
    # Use the entered plant mass reconstructed from total 4-corner normal load;
    # separate from a hypothetical arbitrary load distribution at that mass.
    m = normals.sum(axis=1) / G
    if not np.isfinite(m).all() or np.ptp(m) > 0.025 or min(m) <= 0:
        raise ValueError("Unequal applied robot mass or unsupported route load")
    derived = mass = float(np.mean(m))
    cg = parcel_bounds(mass, xy, balance["residualPositionsM"]["centeredRemainder"], loaded=False)
    low = np.full_like(forces, np.inf)
    for idx in (0, 1):
        dv = np.vstack((deriv[0], deriv)) if idx == 0 else np.vstack((deriv, deriv[-1]))
        ax = dv[:, 0] - speed[:, 1] * speed[:, 2]
        ay = dv[:, 1] + speed[:, 0] * speed[:, 2]
        trial = accepted_normals(xy, cg, mass, np.stack((ax, ay), axis=1), speed[:, 2], dv[:, 2])
        low = np.minimum(low, trial)
    failure = (low <= 0) | (forces > 0.8 * low + 1e-7)
    where = np.argwhere(failure)
    first = None if not len(where) else [float(wall[where[0, 0]]), IDS[where[0, 1]]]
    with np.errstate(divide="ignore", invalid="ignore"):
        nominal = np.where(forces > 0, forces / normals, 0.0)
    return dict(
        inputSha256=registered,
        wallStartS=wall[0],
        wallEndS=wall[-1],
        samples=len(wall),
        plantMassFromFourNormalsKg=derived,
        emptyCGBoundsM=cg,
        minimumPlantNormalN=normals.min(axis=0).tolist(),
        peakPlantMu=float(np.max(nominal)) if np.all(np.isfinite(nominal)) else None,
        assumedStaticShareFloor=0.22,
        rateFraction=0.10,
        checkedFriction=0.80,
        guaranteedCornerLowerN=low.min(axis=0).tolist(),
        firstUncovered=[*first] if first else None,
        lowestTractionMarginN=(0.80 * low - forces).min(axis=0).tolist(),
        fieldGripQualification=None,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("clock", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument("--sha256", required=True)
    a = p.parse_args()
    write(a.out, verify(a.clock, a.sha256))
