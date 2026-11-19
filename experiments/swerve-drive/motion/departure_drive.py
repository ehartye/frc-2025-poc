"""Advance the four loaded wheel pairs from the attained station under a finite departure lease."""

import csv
import math

import numpy as np

import opening as dock
import importlib.util

spec = importlib.util.spec_from_file_location(
    "swerve_departure_supply", dock.HERE.parent / "power/electrical.py"
)
electrical = importlib.util.module_from_spec(spec)
spec.loader.exec_module(electrical)


def run(case, clock_dir, lease, destination, service):
    if service["case"] != case or not 0 <= lease <= 0.6:
        raise ValueError("Unexpected signed state or lease duration")
    cargo_position = service["centerMm"]
    issued = dock.read(dock.HERE / "energy.json")["cases"][case]
    path = clock_dir / (case + ".csv")
    if dock.sha(path) != issued["csvSha256"] or issued["stationLockWallS"] is None:
        raise ValueError("Opening pose and wheels require the attained station clock")
    with path.open(newline="") as src:
        rows = list(csv.DictReader(src))[-4:]
    if [r["id"] for r in rows] != list(dock.ORDER) or any(
        float(r["wall_s"]) != 21.10 for r in rows
    ):
        raise ValueError("Incorrect wheel snapshot")

    def column(name):
        return np.array([float(r[name]) for r in rows])

    # The issued contact fixture immobilizes the chassis and wheel rates during feed.
    residual = max(
        abs(column("wheel_mps")).max(),
        abs(column("body_vx_mps")).max(),
        abs(column("body_vy_mps")).max(),
        abs(column("body_omega_radps")).max(),
    )
    if residual > 1e-5:
        raise ValueError("Incoming motion exceeds the stationary feed fixture")
    f, _, team, _ = dock.n.route.route.field.scenario(
        dock.read(dock.ROOT / "fixed-height-match/placements.json")["placements"],
        inboard=True,
    )
    xy = np.array(issued["centersMm"]) / 1000
    sign = 1 if case == "plus" else -1
    coupling = np.array([1.0, -1.0, 1.0, -1.0])
    mapping = dock.EncoderMap(
        0.0762,
        5.5,
        np.array([0, 1.5, -2.2, 2.8]),
        np.array([1, -1, -1, 1]),
        coupling,
        coupling,
    )
    q = issued["actualTerminalMmMmDeg"]
    start = dock.world([q[0], q[1], math.radians(q[2])], f)
    empty = issued["assumedGrossMassKg"]
    cargo = 0.816466266
    shares0 = np.array([0.26, 0.24, 0.26, 0.24])
    added = np.asarray(cargo_position) / 1000
    # Bilinear positive static reactions from the attained pipe's mass centroid.
    cargo_shares = (
        (1 + np.sign(xy[:, 0]) * added[0] / abs(xy[0, 0]))
        * (1 + np.sign(xy[:, 1]) * added[1] / abs(xy[0, 1]))
        / 4
    )
    if min(cargo_shares) <= 0:
        raise ValueError("Cargo lies outside the four-wheel support")
    mass = empty + cargo
    shares = (empty * shares0 + cargo * cargo_shares) / mass
    cg0 = shares0 @ xy
    cg = shares @ xy
    # Conservative tube yaw inertia adds m LÃ‚Â²/12 and parallel-axis shift.
    inertia = (
        issued["assumedYawInertiaKgM2"]
        + empty * np.sum((cg - cg0) ** 2)
        + cargo * (0.301625**2 / 12 + 0.05715**2 + np.sum((added[:2] - cg) ** 2))
    )
    support = dock.Support(
        xy,
        mass,
        shares,
        (empty * 0.38 + cargo * added[2]) / mass,
        150000 * np.array([0.9, 0.9, 1.1, 1.1]),
    )
    plant = dock.Plant(
        xy,
        start,
        mass=mass,
        inertia=inertia,
        loads=mass * 9.80665 * shares,
        support=support,
        radii=0.0381 * (1 + sign * np.array([0.001, -0.001, 0.0005, -0.0005])),
        coupling=coupling,
        encoder_map=mapping,
        initial_angles=column("azimuth_rad"),
    )
    gyro = start[2]
    phase_origin = service["wallS"]
    dt = 0.005
    pose = [start.copy()]
    velocity = [plant.velocity.copy()]
    for k in range(math.ceil((lease + 0.90) / dt)):
        elapsed = k * dt
        phase = min(1.0, elapsed / 0.25)
        # Back away from the feeder and bias the first leg laterally with the original gyro.
        aim = np.array([-0.15, 0.025, -2.0 * dock.wrapped(gyro - start[2])]) * phase
        authorized = elapsed < lease - 1e-10
        if not authorized:
            aim[:] = 0.0
        angle = mapping.angle(plant.sensors()[1], 1024)
        plant.advance(aim, angle, stopped=not authorized)
        gyro += dt * (
            (1 + sign * 0.001) * plant.velocity[2] + sign * math.radians(0.02)
        )
        pose.append(plant.pose.copy())
        velocity.append(plant.velocity.copy())
    pose = np.array(pose)
    velocity = np.array(velocity)
    times = np.arange(len(pose)) * dt
    angles = pose[:-1, 2]
    deriv = np.diff(velocity, axis=0) / dt
    sp = np.array(
        [
            [*dock.field_to_body(v[:2], a), 0.0, w, alpha]
            for v, a, w, alpha in zip(deriv, angles, velocity[:-1, 2], deriv[:, 2])
        ]
    )
    destination.mkdir(parents=True, exist_ok=True)
    out = destination / f"{case}-{round(lease * 1000)}.npz"
    np.savez(
        out,
        relativeS=times,
        worldPose=pose,
        worldVelocity=velocity,
        bodyAcceleration=sp,
        stationMmMmRad=np.array([dock.station(p, f) for p in pose]),
    )
    field = dock.screen(
        f,
        phase_origin + times,
        pose,
        dock.read(dock.ROOT / "full-match/installed-stock.json")["stock"],
        team,
        phase_origin + times[-1],
    )
    load = np.asarray(plant.power_samples)
    watts = electrical.other_watts(len(load), "loaded")
    requests = {}
    for axis, k, r in (
        ("Drive", 0, electrical.DRIVE_PHASE_R),
        ("Steer", 2, electrical.STEER_PHASE_R),
    ):
        phase = load[:, k + 1, :]
        volts = load[:, k, :] + r * phase
        for idx, name in enumerate(dock.ORDER):
            key = name + axis
            requests[key] = abs(volts[:, idx])
            watts[key] = 0.7 + abs(volts[:, idx] * phase[:, idx]) / 0.9
    packs = {}
    for name, pack in electrical.PACKS.items():
        keys, bus, term, _ = electrical.source(watts, pack)
        motors = np.stack([term[:, keys.index(k)] for k in requests], axis=1)
        head = motors - np.stack(list(requests.values()), axis=1)
        packs[name] = {
            "minimumControllerV": float(motors.min()),
            "headroomV": float(head.min()),
        }
    return dict(
        case=case,
        leaseS=lease,
        elapsedS=times[-1],
        startWallS=phase_origin,
        driveClock=dock.sha(path),
        serviceState=service["outgoing"],
        commandedTravelMm=float(
            np.linalg.norm(pose[round(lease / dt), :2] - pose[0, :2]) * 1000
        ),
        driftAfterBrakeMm=float(
            np.linalg.norm(pose[-1, :2] - pose[round(lease / dt), :2]) * 1000
        ),
        fieldDeltaXYmm=(1000 * (pose[-1, :2] - pose[0, :2])).tolist(),
        chassisDeltaXYmm=(
            1000 * dock.field_to_body(pose[-1, :2] - pose[0, :2], pose[0, 2])
        ).tolist(),
        trajectory=dock.sha(out),
        initialBodyYawRad=start[2],
        incomingMaxLinearOrAngularRate=residual,
        loadedMassKg=mass,
        pipeCenterMm=np.asarray(cargo_position).tolist(),
        yawInertiaKgM2=inertia,
        fieldMinimumMm={k: v for k, v in field.items() if k.startswith("minimum")},
        battery=packs,
        peaks=plant.peak,
        distanceMm=1000 * np.linalg.norm(pose[-1, :2] - pose[0, :2]),
        maxFieldSpeedMps=np.max(np.linalg.norm(velocity[:, :2], axis=1)),
        finalFieldSpeedMps=np.linalg.norm(velocity[-1, :2]),
        finalStationMmMmRad=dock.station(pose[-1], f).tolist(),
    )
