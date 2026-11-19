"""Propagate an observed, loaded R4 departure along the registered sector A field corridor."""

import argparse
import math
from pathlib import Path

import numpy as np

from register import MOTION, prior, read, sha, write
import opening as dock

DT = 0.005


def restore(state, route, side):
    wheel = route["wheel"]
    xy = (
        np.asarray(dock.read(MOTION / "energy.json")["cases"][side]["centersMm"]) / 1000
    )
    polarity = np.array([1, -1, 1, -1])
    enc = dock.EncoderMap(
        0.0762,
        5.5,
        np.array([0, 1.5, -2.2, 2.8]),
        np.array([1, -1, -1, 1]),
        polarity,
        polarity,
    )
    mass = wheel["loadedMassKg"]
    z = (
        (mass - 0.816466266) * 0.38 + 0.816466266 * wheel["pipeCenterMm"][2] / 1000
    ) / mass
    with np.load(state, allow_pickle=False) as st:
        support = dock.Support(
            xy, mass, st["staticShares"], z, 150000 * np.array([0.9, 0.9, 1.1, 1.1])
        )
        plant = dock.Plant(
            xy,
            st["pose"],
            mass=mass,
            inertia=wheel["yawInertiaKgM2"],
            radii=st["radii"],
            support=support,
            loads=st["load"].copy(),
            coupling=polarity,
            encoder_map=enc,
            initial_angles=st["angle"],
        )
        for key in (
            "velocity",
            "azimuth_rate",
            "wheel_speed",
            "wheel_turns",
            "target_speed",
        ):
            setattr(plant, key, st[key].copy())
        gyro = float(st["gyro"])
        odom = dock.Odometry(plant, st["estimatedPose"], enc, gyro)
        for key, register in (
            ("angles", "odomAngles"),
            ("rotor", "odomRotor"),
            ("absolute", "odomAbsolute"),
        ):
            np.testing.assert_array_equal(getattr(odom, key), st[register])
    return plant, enc, odom, gyro


def run(raw, out, case, step, cut=None, policy="south-offset"):
    row, reached, wheels, cargo = prior(raw, case, step)
    plant, enc, odom, gyro = restore(wheels, reached, case)
    layout = dock.ROOT / "fixed-height-match/placements.json"
    stock = dock.ROOT / "full-match/installed-stock.json"
    f, _, team, _ = dock.n.route.route.field.scenario(
        read(layout)["placements"], inboard=True
    )
    _, corridor, _ = f.routes(-0.18)
    goals = [s["end"][:2] for s in corridor.segments if s["kind"] == "drive"]
    if policy != "crossing":
        gx, gy = goals[-1]
        goals[-1] = (gx - 0.300, gy)
    if policy == "south-offset":
        gx, gy = goals[1]
        goals[1] = (gx, gy - 0.45)
    heading0 = odom.pose[2]
    start = reached["wheel"]["startWallS"] + reached["wheel"]["elapsedS"]
    sign = 1 if case == "plus" else -1
    velocity = [plant.velocity.copy()]
    poses = [plant.pose.copy()]
    estimates = [odom.pose.copy()]
    command = np.zeros(3)
    marks = []
    index = 0
    end = None
    for tick in range(round(40 / DT)):
        elapsed = tick * DT
        if cut is not None and elapsed + 1e-10 >= cut and end is None:
            end = elapsed
        if end is None:
            target = np.array(goals[index])
            error = target - odom.pose[:2]
            heading = heading0 if index < 2 else 0.0
            dyaw = dock.wrapped(heading - odom.pose[2])
            length = np.linalg.norm(error)
            observed_speed = (
                np.linalg.norm(
                    odom.pose[:2] - estimates[max(0, len(estimates) - 11)][:2]
                )
                / (min(10, len(estimates) - 1) * DT)
                if len(estimates) > 1
                else 0.0
            )
            settled = observed_speed < 0.018 if policy != "crossing" else True
            if length < 0.009 and abs(dyaw) < math.radians(0.4) and settled:
                marks.append(
                    dict(
                        index=index,
                        wallS=start + elapsed,
                        truePose=plant.pose.tolist(),
                        estimatedPose=odom.pose.tolist(),
                    )
                )
                index += 1
                if index == len(goals):
                    end = elapsed
            speed = min(0.68, math.sqrt(0.48 * length)) if policy != "crossing" else 0.8
            gain = 1.35 if policy != "crossing" else 2.0
            desired = np.r_[
                dock.velocity(error, speed, gain), np.clip(2 * dyaw, -0.65, 0.65)
            ]
            command[:2] += dock.velocity(desired[:2] - command[:2], 0.45 * DT, 1.0)
            command[2] += np.clip(desired[2] - command[2], -0.65 * DT, 0.65 * DT)
        if end is not None:
            command[:] = 0.0
        angle = enc.angle(plant.sensors()[1], 1024)
        plant.advance(
            np.r_[dock.field_to_body(command[:2], odom.pose[2]), command[2]],
            angle,
            stopped=end is not None,
        )
        gyro += DT * (
            (1 + sign * 0.001) * plant.velocity[2] + sign * math.radians(0.02)
        )
        odom.update(*plant.sensors(), gyro)
        poses.append(plant.pose.copy())
        estimates.append(odom.pose.copy())
        velocity.append(plant.velocity.copy())
        if end is not None and elapsed + DT >= end + 0.9 - 1e-10:
            break
    if end is None:
        raise ValueError("Route timed out before a monitored stop")
    pose, vel = np.array(poses), np.array(velocity)
    times = np.arange(len(pose)) * DT
    accel = np.diff(vel, axis=0) / DT
    body = np.array(
        [
            [*dock.field_to_body(a[:2], q[2]), 0, v[2], a[2]]
            for a, q, v in zip(accel, pose, vel)
        ]
    )
    out.mkdir(exist_ok=True, parents=True)
    file = out / "path.npz"
    np.savez_compressed(
        file,
        relativeS=times,
        worldPose=pose,
        worldVelocity=vel,
        estimatedPose=estimates,
        bodyAcceleration=body,
        stationMmMmRad=[dock.station(q, f) for q in pose],
    )
    gaps = dock.screen(
        f, start + times, pose, read(stock)["stock"], team, start + times[-1]
    )
    return dict(
        case=case,
        stepUs=step,
        policy=policy,
        piece=row["pipe"],
        sourceWheel=sha(wheels),
        sourcePipe=sha(cargo),
        sourceMatchRegister=sha(MOTION / "loaded-departure.json"),
        sourceFieldStock=sha(stock),
        startWallS=start,
        endWallS=start + times[-1],
        brakeWallS=start + end,
        startPose=poses[0].tolist(),
        endPose=poses[-1].tolist(),
        goals=goals,
        waypoints=marks,
        trajectory=sha(file),
        inputs="four shaft/azimuth channels and signed gyro",
        peaks=plant.peak,
        battery=dock.supply_review(plant.power_samples, receiver="loaded"),
        field=gaps,
        finalSpeedMps=float(np.linalg.norm(vel[-1, :2])),
        pathLengthM=float(np.linalg.norm(np.diff(pose[:, :2], axis=0), axis=1).sum()),
        maxOdometryErrorMm=float(
            1000
            * np.linalg.norm(pose[:, :2] - np.array(estimates)[:, :2], axis=1).max()
        ),
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("raw", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument("case", choices=("plus", "minus"))
    p.add_argument("step", type=float, choices=(31.25, 15.625))
    p.add_argument(
        "--policy",
        choices=("south-offset", "settled", "crossing"),
        default="south-offset",
    )
    p.add_argument("--cut", type=float, help="Elapsed seconds to initiate a fault stop")
    a = p.parse_args()
    write(a.out / "route.json", run(a.raw, a.out, a.case, a.step, a.cut, a.policy))
