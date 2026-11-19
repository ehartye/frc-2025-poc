"""Evaluate moving, field-backed R4 stereo at exposure time and execute a causal 5 ms permit."""

import math
import numpy as np

import receiver_witness as camera


from station_clock import first_hold


def run(
    field,
    feed,
    service,
    initial,
    chassis,
    contact,
    folder,
    finish="guarded",
    render_cache=None,
):
    if (
        camera.digest(field)
        != camera.read(camera.LEGACY / "full-match/installed-stock.json")[
            "field_step_sha256"
        ]
    ):
        raise ValueError("Different station geometry")
    if (
        initial["terminalStateSha256"] != service["incoming"]
        or initial["piece"] != contact["piece"]
        or service["outgoing"] != contact["serviceState"]
    ):
        raise ValueError("Different incoming custody")
    if chassis["trajectory"] != contact["chassis"] or not initial["gate"]["accepted"]:
        raise ValueError("Route has no observed authority")
    path = folder / contact["file"]
    route = folder / f"{feed['case']}-{round(chassis['leaseS'] * 1000)}.npz"
    if (
        camera.digest(path) != contact["sha256"]
        or camera.digest(route) != chassis["trajectory"]
    ):
        raise ValueError("Cargo or wheel trajectory has changed")
    pipe = np.load(path)
    with np.load(route) as data:
        times = data["relativeS"]
        station = data["stationMmMmRad"]
    if render_cache is None:
        parts = camera.source_parts()
        render_cache = (
            camera.field_mesh(field),
            camera.render(parts, "fixed"),
            camera.render(parts, "roof"),
        )
    mesh, fixed_robot, roof_robot = render_cache
    first = service["wallS"]
    epoch = feed["javaEpochWallS"]
    witness = []
    lost_eye = None
    stamp = 5474
    while True:
        wall = epoch + stamp * 0.001
        rel = wall - first
        if rel > pipe[-1, 0] + 1e-10:
            break
        physical = np.array([np.interp(rel, pipe[:, 0], v) for v in pipe[:, 1:7].T])
        axis = physical[3:] / np.linalg.norm(physical[3:])
        q = np.array([np.interp(rel, times, v) for v in station.T])
        geometry = [-q[0], -q[1], math.degrees(q[2])]
        camera.sensor.scenes.fixed_pose(camera.O)
        fixed = camera.scene(mesh, geometry, fixed_robot)
        camera.sensor.optics.configure()
        actual = camera.scene(mesh, geometry, roof_robot)
        ref = [
            (g, np.where(rz <= z + 0.01, rz, 3500.0))
            for (g, z), (_, rz) in zip(actual, roof_robot)
        ]
        roof = camera.sensor.shelf_guard(
            actual, ref, camera.sensor.optics.ORIGIN[2], finish == "matched", 0.05
        )
        outcome = camera.observe(fixed, roof, 1000 * physical[:3], axis)
        if lost_eye is None:
            bad = camera.observe(
                fixed, roof, 1000 * physical[:3], axis, "fixed-left-drop"
            )
            if bad["accepted"]:
                raise ValueError("Missing eye authorized a fresh moving capture")
            lost_eye = dict(captureWallS=wall, sourceJavaMs=stamp, **bad)
        witness.append(
            dict(
                captureWallS=wall,
                sourceJavaMs=stamp,
                stationMmMmDeg=[q[0], q[1], geometry[2]],
                **outcome,
            )
        )
        stamp += 17
    start = {**initial, "epoch": epoch}
    stop, reason = first_hold(start, witness, first, first + chassis["leaseS"])
    dropout_tick, dropout_reason = first_hold(
        start, [lost_eye] if lost_eye else [], first, first + chassis["leaseS"]
    )
    return dict(
        dropout={
            "frame": lost_eye,
            "stopWallS": dropout_tick,
            "reason": dropout_reason,
        },
        case=feed["case"],
        stepUs=feed["plantStepUs"],
        piece=feed["physicalPieceId"],
        finish=finish,
        chassis=chassis["trajectory"],
        contact=contact["sha256"],
        captureThroughS=pipe[-1, 0],
        stopWallS=stop,
        reason=reason,
        acceptedFrames=sum(v["accepted"] for v in witness),
        frames=witness,
    )
