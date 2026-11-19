"""Test whether the paired observers can use only sightlines backed by registered robot material."""

import argparse
from functools import partial
import math
from pathlib import Path

import numpy as np

from register import prior, read, sha, write
import receiver_witness as camera


def setup(raw, cache):
    config = read(cache / "scene.json")
    if config["bytes"] != sha(cache / "scene.npz") or config["sources"]["r4"] != sha(
        camera.PROJECT / "cad/full-swerve-r4/full-robot.step"
    ):
        raise ValueError("Unregistered four-pupil image source")
    guard = camera.load_wheel_mask(raw / "wheel-visibility.json")
    with np.load(cache / "scene.npz") as src:
        heads = {
            h: [(src[f"{h}{i}g"], src[f"{h}{i}z"]) for i in (0, 1)]
            for h in ("fixed", "roof")
        }
    for h in heads:
        guard[h] = [
            mask | (depth >= 3499.0) for mask, (_, depth) in zip(guard[h], heads[h])
        ]
    reg = read(camera.LEGACY / "selected-optics/mount.json")["registrations"]["roof"]
    camera.sensor.optics.ORIGIN = np.asarray(reg["assumedRectifiedOriginMm"])
    camera.sensor.optics.configure()
    roof = camera.sensor.shelf_guard(
        heads["roof"], heads["roof"], camera.sensor.optics.ORIGIN[2], False, 0.05
    )
    return heads["fixed"], roof, guard


def run(raw, cache, out):
    from fixed_contour import read_pair

    fixed, roof, guard = setup(raw, cache)
    rows = []
    for case in ("plus", "minus"):
        for step in (31.25, 15.625):
            _, source, _, terminal = prior(raw, case, step)
            with np.load(terminal) as st:
                center, axis = 1000 * st["p"], st["a"]
            modes = {
                "historical": camera.observe(
                    fixed, roof, center, axis, wheel_mask=guard
                )
            }
            for cell in (1.0, 0.5, 0.25):
                reader = partial(read_pair, cell=cell)
                modes[str(cell)] = camera.observe(
                    fixed, roof, center, axis, wheel_mask=guard, fixed_reader=reader
                )
            tests = {}
            for k, pt, ax, fault in (
                ("empty", None, axis, None),
                ("one-eye", center, axis, "fixed-left-drop"),
                ("18mm-side", center + np.array([0, 18, 0]), axis, None),
                ("20mm-up", center + np.array([0, 0, 20]), axis, None),
                (
                    "7-degree-yaw",
                    center,
                    np.array([math.sin(math.radians(7)), math.cos(math.radians(7)), 0]),
                    None,
                ),
            ):
                tests[k] = camera.observe(
                    fixed, roof, pt, ax, fault, guard, fixed_reader=read_pair
                )
            rows.append(
                dict(
                    case=case,
                    stepUs=step,
                    pipe=source["pipe"]["piece"],
                    checkpoint=sha(terminal),
                    modes=modes,
                    adverse=tests,
                )
            )
    write(
        out,
        dict(
            scene=sha(cache / "scene.json"),
            source="robot-backed rays; externally backed rays are unknown",
            maskedPixels={
                k: [int(m.sum()) for m in guard[k]] for k in ("fixed", "roof")
            },
            cases=rows,
        ),
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("raw", "cache", "out"):
        p.add_argument(name, type=Path)
    a = p.parse_args()
    run(a.raw, a.cache, a.out)
