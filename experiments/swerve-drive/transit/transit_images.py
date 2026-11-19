"""Reproject the R4 custody cameras at attained field poses and audit the wheel-service clock."""

import argparse
import hashlib
import inspect
import math
from pathlib import Path

import numpy as np

from register import prior, read, sha, write
import receiver_witness as camera


def backgrounds(field, cache=None):
    if (
        sha(field)
        != read(camera.LEGACY / "full-match/installed-stock.json")["field_step_sha256"]
    ):
        raise ValueError("Field edition changed")
    bound = {
        "field": sha(field),
        "r4": sha(camera.PROJECT / "cad/full-swerve-r4/full-robot.step"),
        "appearances": sha(camera.PROJECT / "cad/full-swerve-r4/appearances.json"),
        "renderFunctions": hashlib.sha256("\n".join(inspect.getsource(fn).rstrip() for fn in (camera.source_parts,camera.render)).encode("utf8")).hexdigest(),
        "station": sha(Path(camera.field_mesh.__code__.co_filename)),
        "pupils": sha(camera.LEGACY / "selected-optics/mount.json"),
    }
    if cache is not None:
        record = cache / "scene.json"
        raw = cache / "scene.npz"
        if record.is_file():
            registered = read(record)
            if registered["sources"] != bound or registered["bytes"] != sha(raw):
                raise ValueError(
                    "Camera or field cache differs from registered sources"
                )
            with np.load(raw, allow_pickle=False) as data:
                mesh = [
                    (key, data[f"v{i}"], data[f"f{i}"].tolist())
                    for i, key in enumerate(registered["keys"])
                ]
                heads = [
                    [(data[f"{head}{eye}g"], data[f"{head}{eye}z"]) for eye in range(2)]
                    for head in ("fixed", "roof")
                ]
            mount = read(camera.LEGACY / "selected-optics/mount.json")["registrations"][
                "roof"
            ]
            camera.sensor.optics.ORIGIN = np.asarray(mount["assumedRectifiedOriginMm"])
            camera.sensor.optics.configure()
            return mesh, *heads
    parts = camera.source_parts()
    scene = (
        camera.field_mesh(field),
        camera.render(parts, "fixed"),
        camera.render(parts, "roof"),
    )
    if cache is not None:
        cache.mkdir(parents=True, exist_ok=True)
        arrays = {}
        for i, (_, vs, fs) in enumerate(scene[0]):
            arrays[f"v{i}"], arrays[f"f{i}"] = vs, fs
        for head, images in zip(("fixed", "roof"), scene[1:]):
            for i, (g, z) in enumerate(images):
                arrays[f"{head}{i}g"], arrays[f"{head}{i}z"] = g, z
        np.savez_compressed(raw, **arrays)
        write(
            record, dict(sources=bound, bytes=sha(raw), keys=[x[0] for x in scene[0]])
        )
    return scene


def observation(frames, masked, q, center, axis, fault=None):
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
    return camera.observe(fixed, roof, center, axis, fault=fault, wheel_mask=masked)


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
