"""Evaluate a registered four-corner demand clock against all eight protected branches."""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from electrical import (
    HERE,
    IDS,
    PACKS,
    RADIUS,
    DRIVE_PHASE_CAP,
    STEER_PHASE_CAP,
    branches,
    demand,
    other_watts,
    source,
)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def result(
    wall,
    force,
    lateral,
    normal,
    speed,
    acc,
    steer_rate,
    steer_acc,
    drive_rpm=None,
    steer_rpm=None,
    pack="screened",
    receiver="stowed",
    adverse=False,
    friction=0.8,
    coupling_torque=None,
    drive_rotor_accel=None,
):
    arrays = [
        np.asarray(v, dtype=float)
        for v in (force, lateral, normal, speed, acc, steer_rate, steer_acc)
    ]
    wall = np.asarray(wall, dtype=float)
    if len(wall) < 1 or not np.isfinite(wall).all() or np.any(np.diff(wall) <= 0):
        raise ValueError("Expected a finite strictly advancing clock")
    if any(a.shape != (len(wall), 4) or not np.isfinite(a).all() for a in arrays):
        raise ValueError("Expected four finite module states in FL FR RL RR order")
    force, lateral, normal, speed, acc, steer_rate, steer_acc = arrays
    if np.min(normal) < 0 or friction <= 0:
        raise ValueError("Negative wheel load or nonpositive friction")
    scrub = 0.20 * 0.012 * normal * np.sign(steer_rate)
    opt = (
        {
            "drive_eff": 0.75,
            "steer_eff": 0.60,
            "roll_inertia": 1.5 * 1.5 * RADIUS**2,
            "steer_inertia": 0.0255,
        }
        if adverse
        else {}
    )
    extra = 0.0 if coupling_torque is None else np.asarray(coupling_torque)
    if (
        not np.isfinite(extra).all()
        or (coupling_torque is not None and extra.shape != force.shape)
        or (
            drive_rotor_accel is not None
            and (drive_rotor_accel.shape != force.shape or not np.isfinite(drive_rotor_accel).all())
        )
    ):
        raise ValueError("Invalid coupling data")
    drive, steer = demand(
        force,
        speed,
        acc,
        steer_rate,
        steer_acc,
        scrub * (2 if adverse else 1) + extra,
        drive_rpm,
        steer_rpm,
        drive_rotor_accel=drive_rotor_accel,
        **opt
    )
    w = other_watts((len(wall),), receiver)
    need = {}
    phases = {}
    for axis, (current, volts, watts) in (("Drive", drive), ("Steer", steer)):
        for i, m in enumerate(IDS):
            k = m + axis
            w[k] = watts[:, i]
            need[k] = volts[:, i]
            phases[k] = abs(current[:, i])
    names, bus, term, amps = source(w, PACKS[pack])
    br = branches()
    badbus = ~np.isfinite(bus)

    def envelope(values, maximum=True):
        finite = np.flatnonzero(np.isfinite(values))
        if not finite.size:
            return None
        sub = np.asarray(values)[finite]
        j = finite[np.argmax(sub) if maximum else np.argmin(sub)]
        return {"value": float(values[j]), "wallS": float(wall[j])}

    branches_out = {}
    failed = {
        "sourceCollapse": badbus,
        "terminalFloor": np.any(term < 9.5, axis=1),
        "drivePhase": np.any(abs(drive[0]) > DRIVE_PHASE_CAP, axis=1),
        "steerPhase": np.any(abs(steer[0]) > STEER_PHASE_CAP, axis=1),
        "motorVoltage": np.zeros(len(wall), dtype=bool),
        "breakerRating": np.zeros(len(wall), dtype=bool),
    }
    for j, k in enumerate(names):
        margin = term[:, j] - need.get(k, 0.0)
        failed["motorVoltage"] |= margin < 0 if k in need else False
        failed["breakerRating"] |= amps[:, j] > br[k]["breakerA"]
        branches_out[k] = dict(
            minimumTerminalV=envelope(term[:, j], False),
            peakSupplyA=envelope(amps[:, j]),
            ratingA=br[k]["breakerA"],
        )
        if k in need:
            branches_out[k].update(
                peakPhaseA=envelope(phases[k]), motorVoltageHeadroomV=envelope(margin, False)
            )
    tangential = np.hypot(force, lateral)
    with np.errstate(divide="ignore", invalid="ignore"):
        mu = np.where(tangential == 0, 0, tangential / normal)
    failed["grip"] = (tangential > friction * normal + 1e-7).any(axis=1)

    def first(mask):
        ix = np.flatnonzero(mask)
        return float(wall[ix[0]]) if ix.size else None

    accepted = ~np.logical_or.reduce(list(failed.values()))
    return dict(
        pack=pack,
        otherLoads=receiver,
        adverseMechanism=adverse,
        samples=len(wall),
        wheelOrder=list(IDS),
        couplingTorqueSupplied=coupling_torque is not None,
        couplingRotorAccelerationSupplied=drive_rotor_accel is not None,
        qualification="conditional offline screen",
        firstInhibitWallS=first(~accepted),
        firstByGate={k: first(v) for k, v in failed.items()},
        floorV=9.5,
        gripCoefficient=friction,
        peakRequiredMu=None if not np.isfinite(mu).all() else float(mu.max()),
        minBusV=envelope(bus, False),
        peakPackA=envelope(np.sum(amps, axis=1)),
        branches=branches_out,
        recordedTrajectoryAccepted=bool(accepted.all()),
    ), (bus, amps, mu)


def record(csvpath):
    with csvpath.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows or len(rows) % 4:
        raise ValueError("Expected a complete four-row clock")
    values = {}
    extra = {k: [] for k in ("coupling_torque_nm", "drive_rotor_accel_radps2") if k in rows[0]}
    for i in range(0, len(rows), 4):
        chunk = rows[i : i + 4]
        times = {float(r["wall_s"]) for r in chunk}
        if len(times) != 1 or {r["id"] for r in chunk} != set(IDS):
            raise ValueError("Missing, duplicate or asynchronous module")
        by_id = {r["id"]: r for r in chunk}
        for k in (
            "wall_s",
            "drive_force_n",
            "lateral_force_n",
            "normal_n",
            "wheel_mps",
            "wheel_accel_mps2",
            "azimuth_rate_radps",
            "azimuth_accel_radps2",
            "drive_rotor_rpm",
            "steer_rotor_rpm",
        ):
            values.setdefault(k, []).append(
                next(iter(times)) if k == "wall_s" else [float(by_id[id][k]) for id in IDS]
            )
        for k, v in extra.items():
            v.append([float(by_id[id][k]) for id in IDS])
    keys = {
        "coupling_torque_nm": "coupling_torque",
        "drive_rotor_accel_radps2": "drive_rotor_accel",
    }
    return [np.asarray(v, dtype=float) for v in values.values()], {
        keys[k]: np.asarray(v, dtype=float) for k, v in extra.items()
    }


def write(path, d):
    path.write_text(json.dumps(d, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("clock", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument("--sha256", required=True)
    p.add_argument("--receiver", choices=["stowed", "loaded", "actuating"], default="stowed")
    a = p.parse_args()
    if sha(a.clock) != a.sha256:
        raise ValueError("Unregistered demand clock")
    data, extra = record(a.clock)
    jobs = [
        result(*data, pack=pk, receiver=a.receiver, adverse=adv, **extra)[0]
        for pk in PACKS
        for adv in (False, True)
    ]
    write(a.out, dict(trajectorySha256=a.sha256, trials=jobs, physicalVerdict=None))
