"""A FRC field-relative swerve opening run. The controller sees odometry and delayed stereo."""

import argparse
import csv
import hashlib
import json
import math
import sys
from collections import deque
from pathlib import Path

import numpy as np

from kinematics import ORDER, EncoderMap, body_to_field, field_to_body, wrapped
from plant import Plant, Odometry, SETTINGS

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "conditioned-match"))
from supply import n

sys.path.insert(0, str(ROOT / "swerve-power"))
from loads import Support
from field import screen
import panel


def read(path):
    return json.loads(path.read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def world(q, field):
    xy = np.array(field.DOCK) + body_to_field(np.asarray(q[:2]) / 1000, field.NORMAL)
    return np.r_[xy, field.NORMAL + math.pi + q[2]]


def station(pose, field):
    xy = field_to_body(pose[:2] - field.DOCK, field.NORMAL)
    return np.r_[1000 * xy, wrapped(pose[2] - field.NORMAL - math.pi)]


def velocity(error, maximum, gain):
    length = np.linalg.norm(error)
    return error * min(gain, maximum / max(1e-12, length))


class Stereo:
    def __init__(self, assets):
        path = assets / "mounted-guide-vision/set-nominal-fixed.npz"
        expected = read(ROOT / "mounted-guide-vision/setting-station.json")[
            "set-nominal"
        ]["register"]["fixedBackgroundSha256"]
        if sha(path) != expected:
            raise ValueError("Station raster registration changed")
        with np.load(path) as raw:
            self.depth = [raw[k + "_z"] for k in ("left", "right")]
        _, _, self.bias, _ = panel.referenced(assets / "selected-optics/issued/marked")
        self.background = expected

    def capture(self, actual, prior, dark):
        # The simulation's rendering input is inaccessible to the estimator.
        frames = panel.render(actual, self.depth, contrast=dark)
        predicted = panel.pixels(prior) + self.bias
        marks = np.full((2, 4, 2), np.nan)
        for eye, image in enumerate(frames):
            found = np.asarray(panel.components(image)).reshape(-1, 2)
            for feature in (0, 1):
                near = found[
                    np.linalg.norm(found - predicted[eye, feature], axis=1) < 11
                ]
                if len(near) != 1:
                    return None
                marks[eye, feature] = near[0]
        # Fit XY with the separately integrated short-horizon gyro heading.
        q = prior.copy()
        for _ in range(6):
            px = panel.pixels(q)[:, :2]
            residual = (px + self.bias[:, :2] - marks[:, :2]).ravel()
            jac = np.stack(
                [
                    ((panel.pixels(q + e)[:, :2] - px) / 0.05).ravel()
                    for e in ([0.05, 0, 0], [0, 0.05, 0])
                ],
                axis=1,
            )
            shift = np.linalg.lstsq(jac, residual, rcond=None)[0]
            q[:2] -= shift
            if np.linalg.norm(shift) < 0.0001:
                break
        return (
            q
            if np.max(abs((panel.pixels(q) + self.bias - marks)[:, :2])) < 1.35
            else None
        )


def run(assets, out, case):
    f, _, team, _ = n.route.route.field.scenario(
        read(ROOT / "fixed-height-match/placements.json")["placements"], inboard=True
    )
    source = read(ROOT / "station-landmark/native.json")
    module = read(ROOT / "swerve-package/module.json")
    if tuple(module["order"]) != ORDER:
        raise ValueError("Unexpected native module order")
    centers = np.array([[module["modules"][i][k] for k in ("xM", "yM")] for i in ORDER])
    start = np.r_[
        np.array(source["startXYin"]) * 0.0254, math.radians(source["startHeadingDeg"])
    ]
    sign = -1 if case in ("minus", "upper-minus") else 1
    true_start = start + np.array(
        [
            sign * 0.002,
            -sign * 0.002,
            math.radians(0.8 if case == "misaligned" else sign * 0.12),
        ]
    )
    couplings = np.array([1.0, -1.0, 1.0, -1.0])
    enc = EncoderMap(
        0.0762,
        5.5,
        np.array([0, 1.5, -2.2, 2.8]),
        np.array([1, -1, -1, 1]),
        np.array([1, -1, 1, -1]),
        couplings,
    )
    upper = case in ("heavy", "upper-minus")
    mass = 69.5 if upper else 65.0
    support = Support(
        centers,
        mass,
        [0.26, 0.24, 0.26, 0.24],
        0.38,
        150000 * np.array([0.9, 0.9, 1.1, 1.1]),
    )
    plant = Plant(
        centers,
        true_start,
        mass=mass,
        radii=0.0381 * (1 + sign * np.array([0.001, -0.001, 0.0005, -0.0005])),
        loads=mass * 9.80665 * np.array([0.26, 0.24, 0.26, 0.24]),
        coupling=couplings,
        encoder_map=enc,
        support=support,
        inertia=8.0 if upper else 5.0,
    )
    odom = Odometry(plant, start, enc, start[2])
    sensor = Stereo(assets)
    stage = world([400.0, 0.0, 0.0], f)
    waypoints = [np.array([440.0, 248.0]) * 0.0254, stage[:2]]
    events = []
    path = [plant.pose.copy()]
    timeline = [0.0]
    sample = [
        [
            0.0,
            *np.round(plant.pose, 5),
            *np.round(plant.angle, 3),
            *np.round(plant.wheel_speed, 3),
        ]
    ]
    images = []
    pending = deque()
    step = 0.005
    wall_end = 21.10
    index = 0
    locked = False
    lock_time = None
    latest = -1.0
    prior_read = None
    accepted_frames = 0
    valid_streak = 0
    requested_world = np.zeros(2)
    last_angle = odom.angles.copy()
    stall_ticks = np.zeros(4, dtype=int)
    previous_goal = last_angle.copy()
    previous_rotor = odom.rotor.copy()
    # The gyro is independently sampled; use measured increments rather than reinitializing pose.
    gyro = start[2]
    gyro_queue = deque()
    out.mkdir(parents=True, exist_ok=True)
    csvpath = out / (case + ".csv")
    with csvpath.open("w", encoding="utf8", newline="") as csvfile:
        table = None
        for k in range(round(wall_end / step)):
            wall = k * step
            if k % 2 == 0:
                yaw = (
                    start[2]
                    + (plant.pose[2] - true_start[2]) * (1 + sign * 0.0005)
                    + math.radians(sign * 0.006 * wall)
                )
                gyro_queue.append((k + 2, yaw))
            while gyro_queue and gyro_queue[0][0] <= k:
                gyro = gyro_queue.popleft()[1]
            raw, absolute = plant.sensors()
            odom.update(raw, absolute, gyro)
            steer_change = wrapped(odom.angles - last_angle)
            measured_wheel = (
                enc.delta_distance(raw - previous_rotor, steer_change) / step
            )
            jam = (abs(wrapped(previous_goal - odom.angles)) > 0.45) & (
                abs(steer_change) < 1e-6
            )
            stall_ticks = np.where(jam, stall_ticks + 1, 0)
            if index != 3 and np.max(stall_ticks) >= 32:
                events.append([round(wall, 3), "azimuth response timeout"])
                index = 3
            previous_rotor = raw.copy()
            last_angle = odom.angles.copy()
            if index == 2 and k % 8 == 0:
                q = station(plant.pose, f)
                prior = station(odom.pose, f)
                measured = sensor.capture(q, prior, case != "blind")
                delay = 20 if case == "late" else 6
                pending.append((k + delay, k, measured, odom.pose.copy()))
                images.append([round(wall, 3), measured is not None])
            while pending and pending[0][0] <= k:
                _, capture, observed, then = pending.popleft()
                if observed is None or (k - capture) * step > 0.065:
                    valid_streak = 0
                    continue
                valid_streak += 1
                accepted_frames += 1
                observed_world = world(observed, f)
                odom.pose[:2] += observed_world[:2] - then[:2]
                latest = capture * step
                prior_read = observed
            target = np.zeros(3)
            disabled = 15 <= wall < 18
            safety_hold = 14 <= wall < 18 or index == 3
            if index < 2 and not safety_hold:
                error = waypoints[index] - odom.pose[:2]
                target[:2] = field_to_body(
                    velocity(
                        error, min(1.12, math.sqrt(0.48 * np.linalg.norm(error))), 0.95
                    ),
                    odom.pose[2],
                )
                target[2] = np.clip(2.1 * wrapped(stage[2] - odom.pose[2]), -0.8, 0.8)
                if np.linalg.norm(error) < (0.07 if index == 0 else 0.023):
                    events.append(
                        [
                            round(wall, 3),
                            "entered waypoint"
                            if index == 0
                            else "station acquisition range",
                        ]
                    )
                    index += 1
            elif index == 2 and not safety_hold:
                q = station(odom.pose, f)
                if wall - latest < 0.11:
                    # Separate chassis heading from station-normal and sideways translation.
                    speed = velocity(-q[:2] / 1000, 0.32, 2.5)
                    target[:2] = field_to_body(
                        body_to_field(speed, f.NORMAL), odom.pose[2]
                    )
                    target[2] = np.clip(-2.1 * q[2], -0.30, 0.30)
                    if (
                        prior_read is not None
                        and np.linalg.norm(q[:2]) < 2.3
                        and abs(q[2]) < math.radians(0.16)
                        and max(abs(measured_wheel)) < 0.015
                        and valid_streak >= 3
                    ):
                        if not locked:
                            events.append(
                                [
                                    round(wall, 3),
                                    "station observation and wheel-speed gate",
                                ]
                            )
                            lock_time = wall
                        locked = True
                elif images and wall - images[0][0] > 0.35:
                    index = 3
                    events.append([round(wall, 3), "station observation timeout"])
            request = body_to_field(target[:2], odom.pose[2])
            requested_world += velocity(request - requested_world, 0.62 * step, 1.0)
            target[:2] = field_to_body(requested_world, odom.pose[2])
            if locked or safety_hold or index == 3:
                target[:] = 0.0
                requested_world[:] = 0.0
            row = plant.advance(
                target,
                odom.angles,
                stopped=locked or safety_hold or index == 3,
                disabled=disabled,
                locked_steer=0 if case == "jam" else None,
            )
            previous_goal = row["target_azimuth_rad"].copy()
            end = (k + 1) * step
            answer = station(plant.pose, f)
            body = field_to_body(plant.velocity[:2], plant.pose[2])
            fields = [
                "wall_s",
                "id",
                "x_m",
                "y_m",
                "body_vx_mps",
                "body_vy_mps",
                "body_omega_radps",
                *row,
            ]
            if table is None:
                table = csv.writer(csvfile, lineterminator="\n")
                table.writerow(fields)
            for i, name in enumerate(ORDER):
                table.writerow(
                    [
                        f"{end:.3f}",
                        name,
                        *centers[i],
                        *body,
                        plant.velocity[2],
                        *(value[i] for value in row.values()),
                    ]
                )
            path.append(plant.pose.copy())
            timeline.append(end)
            if (k + 1) % 20 == 0 or abs(end - wall_end) < 1e-8:
                sample.append(
                    [
                        round(end, 3),
                        *np.round(plant.pose, 5),
                        *np.round(plant.angle, 3),
                        *np.round(plant.wheel_speed, 3),
                    ]
                )
    t = np.array(timeline)
    poses = np.array(path)
    gaps = screen(
        f,
        t,
        poses,
        read(ROOT / "full-match/installed-stock.json")["stock"],
        team,
        wall_end,
    )
    pause = poses[(t >= 15) & (t <= 18)]
    tape = (690.875 - 144 - 65.5 - 88) * f.IN
    extent = lambda p: 0.425 * (abs(math.sin(p[2])) + abs(math.cos(p[2])))
    hull_start = 1000 * (extent(poses[0]) - abs(poses[0, 0] - tape))
    hull_end = 1000 * (pause[0, 0] - extent(pause[0]) - tape)
    return dict(
        case=case,
        assumedGrossMassKg=mass,
        assumedYawInertiaKgM2=plant.yaw_inertia,
        footprintRadiusPastTapeMmAtStart=hull_start,
        tapeGapMmAt15s=hull_end,
        disabledYawDeg=float(abs(np.degrees(pause[-1, 2] - pause[0, 2]))),
        centersMm=(centers * 1000).tolist(),
        backgroundSha256=sensor.background,
        csvSha256=sha(csvpath),
        csvRows=(len(t) - 1) * 4,
        stationLockWallS=lock_time,
        actualTerminalMmMmDeg=[*answer[:2], math.degrees(answer[2])],
        estimatedTerminalMmMmDeg=[
            *station(odom.pose, f)[:2],
            math.degrees(station(odom.pose, f)[2]),
        ],
        renderedDetections=sum(v for _, v in images),
        acceptedCameraFrames=accepted_frames,
        imagesTaken=len(images),
        events=events,
        peaks=plant.peak,
        disabledTravelMm=1000 * float(np.linalg.norm(pause[-1, :2] - pause[0, :2])),
        geometry=gaps,
        samples=sample,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument(
        "--case",
        choices=(
            "plus",
            "minus",
            "blind",
            "late",
            "jam",
            "misaligned",
            "heavy",
            "upper-minus",
        ),
        required=True,
    )
    args = p.parse_args()
    result = run(args.assets, args.out, args.case)
    (args.out / (args.case + ".json")).write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf8"
    )
    print(
        {
            k: result[k]
            for k in (
                "case",
                "events",
                "actualTerminalMmMmDeg",
                "acceptedCameraFrames",
                "csvSha256",
            )
        }
    )
