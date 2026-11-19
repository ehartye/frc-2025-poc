"""Four MAXSwerve pairs and the retained stowed receiver on one loaded battery."""

from pathlib import Path
import json
import numpy as np

HERE = Path(__file__).resolve().parent
PROJECT = next(
    folder
    for folder in HERE.parents
    if any(
        (folder / p).is_file()
        for p in (
            "design/electrical-inspection/circuits.json",
            "experiments/six-wheel/design/electrical-inspection/circuits.json",
        )
    )
)
ROOT = (
    PROJECT / "experiments/six-wheel"
    if (PROJECT / "experiments/six-wheel/design/electrical-inspection/circuits.json").exists()
    else PROJECT
)
PACKAGE = (
    HERE.parent / "packaging"
    if (HERE.parent / "packaging/module.json").exists()
    else PROJECT / "swerve-package"
)
IDS = ("FL", "FR", "RL", "RR")
AWG = {10: 0.003277, 12: 0.005211, 14: 0.008286, 16: 0.01317, 18: 0.02095}
DRIVE_RATIO = 5.50
STEER_RATIO = 9424 / 203
RADIUS = 0.0762 / 2
DRIVE_PHASE_CAP = 40.0
STEER_PHASE_CAP = 20.0
PACKS = {"nominal": (12.4, 0.025), "screened": (11.8, 0.027), "weak": (11.8, 0.032)}
STEER_PHASE_R = 0.003 + 2 * 1.25 * (0.30 * AWG[16] + 0.15 * AWG[12])
DRIVE_PHASE_R = 0.003 + 2 * 1.25 * 0.70 * AWG[12]
DRIVE_SUPPLY_R = 0.003 + 2 * 1.25 * (1.55 * AWG[10] + 0.25 * AWG[12])
STEER_SUPPLY_R = 0.003 + 2 * 1.25 * (1.65 * AWG[14] + 0.15 * AWG[12])


def branches():
    circuits = json.loads((ROOT / "design/electrical-inspection/circuits.json").read_bytes())[
        "branches"
    ]
    retain = {
        r["load"]: dict(
            channel=r["channel"],
            can=r["can"],
            breakerA=r["ratingA"],
            ohm=0.003 + 2 * 1.25 * r["leadM"] * AWG[r["awg"]],
        )
        for r in circuits
        if r["load"] not in ("DriveLeft", "DriveRight", "Observer")
    }
    for name, ch, length in (("Host", 6, 0.8), ("FixedCamera", 8, 0.95), ("MovingCamera", 9, 1.1)):
        retain[name] = dict(
            channel=ch, can=None, breakerA=10, ohm=0.003 + 2 * 1.25 * length * AWG[18]
        )
    for axis, channels, can, breaker, res in (
        ("Drive", (0, 1, 10, 11), (11, 12, 17, 18), 40, DRIVE_SUPPLY_R),
        ("Steer", (12, 13, 14, 15), (19, 20, 21, 22), 20, STEER_SUPPLY_R),
    ):
        for m, ch, addr in zip(IDS, channels, can):
            retain[m + axis] = dict(channel=ch, can=addr, breakerA=breaker, ohm=res)
    ch = [r["channel"] for r in retain.values()]
    can = [r["can"] for r in retain.values() if r["can"] is not None] + [1]
    if len(set(ch)) != len(ch) or len(set(can)) != len(can):
        raise ValueError("Overlapping protected branch or CAN identity")
    return retain


def motor(rpm, torque, stall_a, stall_nm, free_rpm, free_a, phase_lead):
    """Effective signed phase current and required controller voltage magnitude."""
    sign = np.sign(np.where(abs(rpm) > 1e-8, rpm, torque))
    current = torque / (stall_nm / stall_a) + sign * free_a
    resistance = 12 / stall_a
    terminal = rpm / free_rpm * (12 - resistance * free_a) + current * resistance
    volts = terminal + phase_lead * current
    # Positive source envelope also charges braking electrical power magnitude.
    watts = 0.7 + abs(volts * current) / 0.90
    return current, np.abs(volts), watts


def demand(
    force,
    wheel_mps,
    wheel_accel,
    steer_rate,
    steer_accel,
    steer_torque,
    drive_rpm=None,
    steer_rpm=None,
    drive_eff=0.83,
    steer_eff=0.72,
    roll_inertia=1.5 * RADIUS**2,
    steer_inertia=0.017,
    drive_rotor_accel=None,
):
    inputs = [
        np.asarray(v, dtype=float)
        for v in (force, wheel_mps, wheel_accel, steer_rate, steer_accel, steer_torque)
    ]
    force, wheel_mps, wheel_accel, steer_rate, steer_accel, steer_torque = inputs
    roll = wheel_mps / RADIUS
    acc = wheel_accel / RADIUS
    if drive_rpm is None:
        drive_rpm = roll * DRIVE_RATIO * 60 / (2 * np.pi)
    if steer_rpm is None:
        steer_rpm = steer_rate * STEER_RATIO * 60 / (2 * np.pi)
    # Upper motor torque demand assumes motoring loss in both directions.
    # Both equivalent inertias include the reflected rotor in output coordinates.
    drive_shaft = (force * RADIUS + roll_inertia * acc) / (DRIVE_RATIO * drive_eff)
    if drive_rotor_accel is not None:
        # The prior effective output J already includes pure-roll rotor acceleration.
        # Apply only the difference due to hypothesized mechanical coupling.
        drive_shaft += 0.00003 * (drive_rotor_accel - DRIVE_RATIO * acc)
    steer_shaft = (steer_torque + steer_inertia * steer_accel) / (STEER_RATIO * steer_eff)
    drive = motor(drive_rpm, drive_shaft, 105, 2.6, 5676, 1.8, DRIVE_PHASE_R)
    steer = motor(steer_rpm, steer_shaft, 100, 0.97, 11000, 1.4, STEER_PHASE_R)
    return drive, steer


def other_watts(shape, receiver="stowed"):
    w = {
        "Radio": 12.0,
        "RIO": 15.0,
        "PH": 1.5,
        "Host": 5.21 * 3.2 / 0.75,
        "FixedCamera": 5.21 * 1.2 / 0.75,
        "MovingCamera": 5.21 * 1.2 / 0.75,
        "Lift": 0.0,
        "Slide": 0.7,
        "LeftGrip": 0.7,
        "RightGrip": 0.7,
    }
    if receiver in ("loaded", "actuating"):
        # 8 A per grip under retained stationary custody; for concurrent actuators
        # reserve inherited full rack cap and running grip voltage separately.
        vgrip = 8 * (0.12 + 0.003 + 2 * 1.25 * 2.85 * AWG[16])
        if receiver == "actuating":
            vgrip += 5.0
        w["LeftGrip"] = w["RightGrip"] = 0.7 + 8 * vgrip / 0.90
        if receiver == "actuating":
            w["Slide"] = 0.7 + 20 * 8 / 0.90
    elif receiver != "stowed":
        raise ValueError("Unknown receiver state")
    return {k: np.full(shape, p, dtype=float) for k, p in w.items()}


def source(watts, pack):
    """Stable high-voltage solution of branch+source droop; NaN denotes collapse."""
    b = branches()
    names = list(watts)
    w = np.stack([watts[k] for k in names], axis=-1)
    if not np.isfinite(w).all() or np.min(w) < 0:
        raise ValueError("Expected finite electrical loads")
    voc, r = pack
    rs = np.array([b[k]["ohm"] for k in names])
    bus = np.full(w.shape[:-1], voc)
    for _ in range(160):
        with np.errstate(invalid="ignore", divide="ignore"):
            term = (bus[..., None] + np.sqrt(bus[..., None] ** 2 - 4 * rs * w)) / 2
            amps = w / term
            next_bus = voc - r * amps.sum(axis=-1)
        next_bus = np.where((next_bus > 0) & (bus > 0), next_bus, np.nan)
        valid = np.isfinite(next_bus) & np.isfinite(bus)
        if not valid.any() or (
            (abs(next_bus[valid] - bus[valid]) < 1e-9).all()
            and np.array_equal(np.isfinite(next_bus), np.isfinite(bus))
        ):
            finite = np.isfinite(next_bus[..., None])
            return names, next_bus, np.where(finite, term, np.nan), np.where(finite, amps, np.nan)
        bus = next_bus
    with np.errstate(invalid="ignore", divide="ignore"):
        term = (bus[..., None] + np.sqrt(bus[..., None] ** 2 - 4 * rs * w)) / 2
        amps = w / term
        valid = np.isfinite(bus) & (abs(bus + r * amps.sum(axis=-1) - voc) < 1e-7)
    return (
        names,
        np.where(valid, bus, np.nan),
        np.where(valid[..., None], term, np.nan),
        np.where(valid[..., None], amps, np.nan),
    )
