"""Reproject the R4 custody cameras at attained field poses and audit the wheel-service clock."""

import argparse
import math
from pathlib import Path

import numpy as np

from register import prior, read, sha, write
import receiver_witness as camera


def backgrounds(field):
    if (
        sha(field)
        != read(camera.LEGACY / "full-match/installed-stock.json")["field_step_sha256"]
    ):
        raise ValueError("Field edition changed")
    parts = camera.source_parts()
    return (
        camera.field_mesh(field),
        camera.render(parts, "fixed"),
        camera.render(parts, "roof"),
    )


def observation(frames, masked, q, center, axis):
    mesh, fixed_blank, roof_blank = frames
    delta = [-q[0], -q[1], math.degrees(q[2])]
    camera.sensor.scenes.fixed_pose(camera.O)
    fixed = camera.scene(mesh, delta, fixed_blank)
    camera.sensor.optics.configure()
    roof = camera.scene(mesh, delta, roof_blank)
    ref = [
        (gray, np.where(rz <= depth + 0.01, rz, 3500.0))
        for (gray, depth), (_, rz) in zip(roof, roof_blank)
    ]
    roof = camera.sensor.shelf_guard(
        roof, ref, camera.sensor.optics.ORIGIN[2], False, 0.05
    )
    return camera.observe(fixed, roof, center, axis, wheel_mask=masked)


def probe(raw, run, field, out, case, step, interval):
    row, previous, _, terminal = prior(raw, case, step)
    route = read(run / "route.json")
    if route["trajectory"] != sha(run / "path.npz") or route["sourcePipe"] != sha(
        terminal
    ):
        raise ValueError("Unregistered route or reference cargo")
    with np.load(terminal) as st:
        center, axis = st["p"] * 1000, st["a"]
    with np.load(run / "path.npz") as st:
        times, poses = st["relativeS"], st["stationMmMmRad"]
    config = raw / "wheel-visibility.json"
    masks = camera.load_wheel_mask(config)
    view = backgrounds(field)
    sampled = []
    for k in range(0, len(times), round(interval / 0.005)):
        obs = observation(view, masks, poses[k], center, axis)
        sampled.append(
            dict(wallS=route["startWallS"] + times[k], pose=poses[k].tolist(), **obs)
        )
    write(
        out,
        dict(
            source="attained wheel poses, frozen incoming tube for optical coverage only",
            piece=row["pipe"],
            stepUs=step,
            case=case,
            route=route["trajectory"],
            pipe=sha(terminal),
            wheelMask=sha(config),
            samples=sampled,
        ),
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("raw", "run", "field", "out"):
        p.add_argument(name, type=Path)
    p.add_argument("case", choices=("plus", "minus"))
    p.add_argument("step", type=float)
    p.add_argument("--interval", type=float, default=0.5)
    a = p.parse_args()
    probe(a.raw, a.run, a.field, a.out, a.case, a.step, a.interval)
