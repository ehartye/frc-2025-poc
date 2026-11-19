"""Compact source-associated adversarial and attained swerve power/traction receipts."""

import argparse
from pathlib import Path
import numpy as np
from electrical import HERE, IDS, PACKS
from screen import record, result, sha, write
from trace_power import assess
from route_grip import verify
from balance import centers
from loads import G


def witness(path, z=0.31):
    r = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf8").reshape((-1, 4))
    s = np.stack([r[k][:, 0] for k in ("body_vx_mps", "body_vy_mps", "body_omega_radps")], axis=1)
    wall = r["wall_s"][:, 0]
    deriv = np.diff(s, axis=0) / np.diff(wall)[:, None]
    xy = centers()
    A = np.vstack((np.ones(4), xy.T))
    k = np.array([0.9, 0.9, 1.1, 1.1])
    mass = float(r["normal_n"][0].sum() / G)
    proj = (k[:, None] * A.T) @ np.linalg.inv((A * k) @ A.T)
    tangential = np.hypot(r["drive_force_n"], r["lateral_force_n"])
    best = None
    for edge, dv in (
        ("arriving", np.vstack((deriv[0], deriv))),
        ("departing", np.vstack((deriv, deriv[-1]))),
    ):
        acc = np.stack([dv[:, 0] - s[:, 1] * s[:, 2], dv[:, 1] + s[:, 0] * s[:, 2]], axis=1)
        normal = mass * (G * 0.25 - z * (acc @ proj[:, 1:].T))
        margin = 0.8 * normal - tangential
        i, j = np.unravel_index(np.argmin(margin), margin.shape)
        if best is None or margin[i, j] < best["marginN"]:
            best = dict(
                derivative=edge,
                wallS=float(wall[i]),
                corner=IDS[j],
                massKg=mass,
                friction=0.8,
                staticShares=[0.25] * 4,
                cgM=[0.0, 0.0, z],
                rateRatios=k.tolist(),
                accelerationMps2=acc[i].tolist(),
                normalN=float(normal[i, j]),
                tireResultantN=float(tangential[i, j]),
                requiredMu=float(tangential[i, j] / normal[i, j]),
                marginN=float(margin[i, j]),
            )
    return best


def evaluate(path, pin):
    data, extra = record(path)
    support = verify(path, pin)
    trials = []
    for pack in PACKS:
        actual = assess(path, pin, "stowed", pack)
        low = min((row["minTerminalV"], k) for k, row in actual["perBranch"].items())
        constraints = []
        for adverse in (False, True):
            calc, _ = result(*data, pack=pack, adverse=adverse, **extra)
            phase = {
                axis: max(
                    (v["peakPhaseA"]["value"], k)
                    for k, v in calc["branches"].items()
                    if k.endswith(axis)
                )
                for axis in ("Drive", "Steer")
            }
            head = min(
                (v["motorVoltageHeadroomV"]["value"], k)
                for k, v in calc["branches"].items()
                if "motorVoltageHeadroomV" in v
            )
            constraints.append(
                dict(
                    adverse=adverse,
                    firstInhibitWallS=calc["firstInhibitWallS"],
                    minBusV=calc["minBusV"],
                    peakPackA=calc["peakPackA"],
                    peakPhaseA=phase,
                    minimumMotorVoltageHeadroomV=head,
                    couplingTorqueSupplied=calc["couplingTorqueSupplied"],
                )
            )
        trials.append(
            dict(
                pack=pack,
                actualBranchSupplyPeakA=max(
                    (v["maxBranchA"], k) for k, v in actual["perBranch"].items()
                ),
                actualPeakPackA=actual["peakPackA"],
                actualMinTerminalV=low,
                actualFirstBlockedWallS=actual["firstBlockedWallS"],
                inverseDemand=constraints,
            )
        )
    return dict(clockSha256=pin, support=support, electrical=trials)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("register", type=Path)
    p.add_argument("raw", type=Path)
    p.add_argument("out", type=Path)
    a = p.parse_args()
    import json

    spec = json.loads(a.register.read_bytes())
    trials = {}
    for key, v in spec["traces"].items():
        path = a.raw / v["file"]
        if sha(path) != v["sha256"]:
            raise ValueError("Wrong clock " + key)
        trials[key] = dict(registeredClock=key, **evaluate(path, v["sha256"]))
        if key == "abrupt":
            trials[key]["consistentAdverse"] = witness(path)
    write(
        a.out,
        dict(
            traces=trials,
            reproductionInputs={
                p.name: sha(p) for p in (HERE / "balance.json", HERE / "branches.json", a.register)
            },
            physicalMassKg=None,
            loadCellQualification=None,
            placementCredit=None,
        ),
    )
