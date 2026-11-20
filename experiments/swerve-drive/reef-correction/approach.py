"""Move from the measured, captive reef stops using tag history, both native bolt pixels and shaft/gyro feedback."""

from pathlib import Path
import argparse
import json
import math
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
RET = HERE.parent / "retention"
sys.path.insert(0, str(RET))
sys.path.insert(0, str(HERE.parent / "transit"))
import field_transit as F
from register import read, sha, write

sys.path.insert(0, str(RET))
from importlib.util import module_from_spec, spec_from_file_location

spec = spec_from_file_location(
    "reef_correction_optics", HERE.parents[1] / "six-wheel/sector-alignment/vision.py"
)
V = module_from_spec(spec)
spec.loader.exec_module(V)

DT = F.DT
TARGET = np.array([-54.0, -180.0, 0.0])
MAX_RESIDUAL = 0.18


def project_local(p, origin):
    return np.r_[1000 * (p[:2] - origin) + TARGET[:2], p[2]]


def sensed_center(heads, ctx, prediction):
    point = ctx["bolts"][:1]
    positions = []
    for j, eye in enumerate(V.EYES):
        expected = V.project(point, prediction, eye)[0][0]
        matches = [v for v in heads[j] if np.linalg.norm(expected - v) < 9.0]
        if len(matches) != 1:
            return None
        body = V.unproject(np.array(matches), point[:, 2], eye)[0]
        positions.append(point[0, :2] - V.rotation(prediction[2]) @ body)
    if np.linalg.norm(positions[0] - positions[1]) > 2.0:
        return None
    return np.r_[np.mean(positions, axis=0), prediction[2]]


def run(artifacts, side, mode="nominal", *, cam_run=None, forward_mm=0.0, cam_source=None):
    if cam_run is None:
        route_dir = artifacts / "retained-coral" / side / "continued"
        survey = artifacts / "captive-observer/mono.json"
        camera = artifacts / "reef-correction/camera.npz"
        pipe = artifacts / "captive-observer" / side / "state.npz"
        source = read(RET / "held.json")["cases"][side]["source"]
        folder = artifacts / "reef-correction" / side
    else:
        cam = artifacts / "cam-scoring" if cam_source is None else cam_source
        route_dir = cam / side / "route"
        survey = cam / "reef-survey.json"
        camera = cam / "camera.npz"
        pipe = cam / side / "route-piece.npz"
        custody = read(cam / side / "route-piece.json")
        if cam_source is None:
            published = read(HERE.parent / "cam-scoring/results.json")["developmentCases"][side]
            if custody["state"] != published["stages"]["route"]["state"]:
                raise ValueError("Different published development state")
        station=read(cam/side/"station-piece.json")
        if (not custody["allGranted"] or custody["fault"] or not station["allGranted"]
                or custody["piece"] != station["piece"] or custody["piece"]!="coral-A"
                or custody["before"]!=station["state"] or custody["wheel"] != read(route_dir/"route.json")["trajectory"]):
            raise ValueError("Cam capture or signed route changed")
        source = {"checkpoint": custody["state"], "endWallS": custody["endWallS"],
                  "qualifiedAtWheelEndpoint": {"retained": custody["allGranted"]}}
        folder = cam_run / side
    case = read(survey)[side]
    previous = read(route_dir / "route.json")
    scene = read(camera.with_suffix(".json"))
    if scene["bytes"] != sha(camera) or scene["sourceSurvey"] != sha(survey):
        raise ValueError("Optical source changed")
    with np.load(camera) as data:
        ctx = {k: data[k] for k in data.files}
    ctx["faces"] = [ctx.pop("face0"), ctx.pop("face1")]
    optic = scene["cases"][side]
    corner_review = optic["cornerReview"]
    wheel = route_dir / "end-wheels.npz"
    if (
        sha(wheel) != previous["terminalWheels"]
        or sha(pipe) != source["checkpoint"]
        or previous["endWallS"] != source["endWallS"]
    ):
        raise ValueError("Changed clock, pipe or wheels")
    if (
        not all(source["qualifiedAtWheelEndpoint"].values())
        or not previous["stopFeedback"]["confirmed"]
    ):
        raise ValueError("Need stopped wheel and identified, held pipe")
    wheel_record = F.prior(artifacts / "loaded-swerve", side, 31.25)[1]
    plant, enc, odom, gyro = F.restore(wheel, wheel_record, side)
    origin = np.asarray(previous["goals"][-1])
    goal = origin + np.array([forward_mm / 1000, 0.0])
    initial = project_local(plant.pose, origin)
    if (
        np.linalg.norm(initial - np.asarray(optic["pose"])) > 1e-6
        or case["wheel"] != optic["source"]
    ):
        raise ValueError("Wheel and optical service differ")
    registered = optic["optical"]
    if not registered["accepted"]:
        raise ValueError("No initial tag/bolt fix")
    lo, hi = np.array(registered["positionMmHeadingRad"])
    position = (lo + hi) / 2
    odom.pose = np.r_[origin + (position[:2] - TARGET[:2]) / 1000, position[2]]
    start = previous["endWallS"]
    command = np.zeros(3)
    complete = None
    last = None
    abort = None
    pose = [plant.pose.copy()]
    est = [odom.pose.copy()]
    motion = [plant.velocity.copy()]
    sensors = [np.r_[*plant.sensors(), gyro]]
    images = []
    queue = []
    # A rendered exposure is delivered at a later 5-ms service. Debounce the
    # fine position on delivered frames only. Mechanical stop uses real channels.
    for tick in range(1000):
        secs = tick * DT
        if tick % 10 == 0:
            _, heads, _, _ = V.station_frame(project_local(plant.pose, origin), ctx)
            if mode == "missing-eye" and tick >= 80:
                heads[0] = []
            q = sensed_center(heads, ctx, project_local(odom.pose, origin))
            if mode != "lost-packets" or tick < 80:
                queue.append(
                    (
                        tick + 3,
                        None if q is None else q[:2].copy(),
                        odom.pose[:2].copy(),
                    )
                )
        if queue and queue[0][0] == tick:
            _, xy, old = queue.pop(0)
            if xy is None and abort is None:
                abort = "missing or ambiguous bolt"
                complete = tick
            if xy is not None and abort is None:
                odom.pose[:2] = (
                    origin + (xy - TARGET[:2]) / 1000 + (odom.pose[:2] - old)
                )
                last = tick
                images.append(
                    dict(
                        wallS=start + secs,
                        measuredMm=project_local(odom.pose, origin).tolist(),
                    )
                )
        if last is not None and tick - last > 12 and abort is None and complete is None:
            abort = "expired optical lease"
            complete = tick
        err = goal - odom.pose[:2]
        heading = F.dock.wrapped(-odom.pose[2])
        if (
            complete is None
            and tick > 50
            and np.linalg.norm(err) < 0.0012
            and abs(heading) < math.radians(0.10)
        ):
            complete = tick
        if (last is not None and tick - last > 12) or complete is not None:
            command[:] = 0
        elif last is not None:
            desired = np.r_[
                F.dock.velocity(err, 0.020, 1.8), np.clip(1.5 * heading, -0.04, 0.04)
            ]
            command[:2] += F.dock.velocity(desired[:2] - command[:2], 0.08 * DT, 1.0)
            command[2] += np.clip(desired[2] - command[2], -0.12 * DT, 0.12 * DT)
        plant.advance(
            np.r_[F.dock.field_to_body(command[:2], odom.pose[2]), command[2]],
            enc.angle(plant.sensors()[1], 1024),
            stopped=complete is not None or last is None,
        )
        gyro += DT * (
            (1 + (1 if side == "plus" else -1) * 0.001) * plant.velocity[2]
            + (1 if side == "plus" else -1) * math.radians(0.02)
        )
        raw, absolute = plant.sensors()
        odom.update(raw, absolute, gyro)
        pose.append(plant.pose.copy())
        est.append(odom.pose.copy())
        motion.append(plant.velocity.copy())
        sensors.append(np.r_[raw, absolute, gyro])
        if complete is not None and tick >= complete + 100:
            break
    if complete is None:
        raise ValueError("Corrective command did not complete")
    poses = np.array(pose)
    vel = np.array(motion)
    times = np.arange(len(pose)) * DT
    accel = np.diff(vel, axis=0) / DT
    body = np.array(
        [
            [*F.dock.field_to_body(a[:2], q[2]), 0, v[2], a[2]]
            for a, q, v in zip(accel, poses, vel)
        ]
    )
    stop = F.stop_feedback(sensors, est, enc, len(est) - 1)
    if not stop["confirmed"]:
        raise ValueError("Correction did not stop")
    terminal = project_local(poses[-1], origin)
    if abort:
        result = dict(
            case=side,
            mode=mode,
            reason=abort,
            sourceWheels=sha(wheel),
            sourcePipe=sha(pipe),
            firstInhibitWallS=start + complete * DT,
            endWallS=start + times[-1],
            stopped=stop,
            travelDuringBrakeMm=float(
                1000
                * np.linalg.norm(np.diff(poses[complete:, :2], axis=0), axis=1).sum()
            ),
            terminalMmMmRad=terminal.tolist(),
            deliveryCredit=0,
        )
        folder.mkdir(exist_ok=True, parents=True)
        write(folder / (mode + ".json"), result)
        return result
    # Propagate the source gyro enclosure to the measured end heading, then
    # restrict position using the same uniquely-associated actual stationary bolt.
    net = gyro - float(np.load(wheel)["gyro"])
    drift = math.radians(0.02) * times[-1] + 0.001 * sum(
        abs(np.diff(np.asarray(sensors)[:, -1]))
    )
    angles = np.arange(
        lo[2] + net - drift,
        hi[2] + net + drift + math.radians(0.025),
        math.radians(0.025),
    )
    lows = np.full((len(angles), 2), -np.inf)
    highs = -lows.copy()
    _, heads, _, _ = V.station_frame(terminal, ctx)
    for eye, h in zip(V.EYES, heads):
        expected = V.project(ctx["bolts"][:1], project_local(odom.pose, origin), eye)[
            0
        ][0]
        match = [v for v in h if np.linalg.norm(v - expected) < 9.0]
        if len(match) != 1:
            raise ValueError("Missing final real field landmark")
        w = ctx["bolts"][0]
        rays = V.unproject(
            np.asarray(match[0])
            + [[-0.25, -0.25], [-0.25, 0.25], [0.25, -0.25], [0.25, 0.25]],
            np.full(4, w[2]),
            eye,
        )
        poly = np.array([w[:2] - rays @ V.rotation(yaw).T for yaw in angles])
        pad = 1.0 + 2 * np.linalg.norm(rays, axis=1).max() * math.sin(
            math.radians(0.0625) / 2
        )
        a = poly.min(1) - pad
        b = poly.max(1) + pad
        lows = np.maximum(lows, a)
        highs = np.minimum(highs, b)
    valid = np.all(lows <= highs, axis=1)
    enclosure = (
        None
        if not valid.any()
        else [
            np.r_[
                lows[valid].min(0), min(angles[valid]) - math.radians(0.0125)
            ].tolist(),
            np.r_[
                highs[valid].max(0), max(angles[valid]) + math.radians(0.0125)
            ].tolist(),
        ]
    )
    if (
        enclosure is None
        or np.any(terminal < enclosure[0])
        or np.any(terminal > enclosure[1])
    ):
        raise ValueError("Final measured enclosure missed physical state")
    folder.mkdir(exist_ok=True, parents=True)
    terminal_wheels = None
    if cam_run is not None:
        terminal_wheels = folder / "end-wheels.npz"
        F.save_endpoint(terminal_wheels, wheel, plant, odom, gyro)
    path = folder / "chassis.npz"
    np.savez_compressed(
        path,
        relativeS=times,
        worldPose=poses,
        estimatedPose=est,
        worldVelocity=vel,
        bodyAcceleration=body,
        rotorAbsoluteGyro=sensors,
    )
    layout, _, team, _ = F.dock.n.route.route.field.scenario(
        read(F.dock.ROOT / "fixed-height-match/placements.json")["placements"],
        inboard=True,
    )
    field = F.dock.screen(
        layout,
        start + times,
        poses,
        read(F.dock.ROOT / "full-match/installed-stock.json")["stock"],
        team,
        start + times[-1],
    )
    receipt = dict(
        case=side,
        piece="coral-A",
        sourcePipe=sha(pipe),
        sourceWheels=sha(wheel),
        sourcePose=initial.tolist(),
        prior=registered,
        skippedByResidual=sum(not r["accepted"] for r in corner_review),
        cornerReview=corner_review,
        detected=len(case["views"]),
        accepted=sum(r["accepted"] for r in corner_review),
        newFrames=images,
        startWallS=start,
        endWallS=start + times[-1],
        route=sha(path),
        stopped=stop,
        field=field,
        power=F.supply_review(plant.power_samples),
        terminalMmMmRad=terminal.tolist(),
        enclosure=enclosure,
        cargoContinued=False,
        deliveryCredit=0,
    )
    if terminal_wheels is not None:
        receipt.update(terminalWheels=sha(terminal_wheels), correctionForwardMm=forward_mm, camera=sha(camera))
    write(folder / "chassis.json", receipt)
    return receipt


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifacts", type=Path)
    parser.add_argument("case", choices=["plus", "minus"])
    parser.add_argument(
        "--mode", choices=["nominal", "missing-eye", "lost-packets"], default="nominal"
    )
    args = parser.parse_args()
    r = run(args.artifacts, args.case, args.mode)
    print(
        {
            k: r.get(k)
            for k in (
                "case",
                "endWallS",
                "stopped",
                "terminalMmMmRad",
                "enclosure",
                "reason",
            )
        }
    )
