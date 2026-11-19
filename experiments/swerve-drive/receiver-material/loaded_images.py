"""Reproject the material-specific pipe and actual wheel trajectory after acquisition."""

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
from calibrators import off_volume, witness
from observations import run_view
from radiometry import samples
from station_clock import first_hold


def rendered(mesh, heads, facing, delta, wheels, *, robot_only):
    fixed, roof = heads
    witness.sensor.scenes.fixed_pose(witness.O)
    fixed_scene = witness.scene(mesh, delta, fixed)
    witness.sensor.optics.configure()
    roof_scene = witness.scene(mesh, delta, roof)
    ref = [
        (g, np.where(rz <= z + 0.01, rz, 3500.0))
        for (g, z), (_, rz) in zip(roof_scene, roof)
    ]
    actual, reference = witness.sensor.shelf_guard(
        roof_scene, ref, witness.sensor.optics.ORIGIN[2], True, 0.05
    )
    grades = []
    for name, body, scene in (("fixed", fixed, fixed_scene), ("roof", roof, actual)):
        if name == "fixed":
            witness.sensor.scenes.fixed_pose(witness.O)
        else:
            witness.sensor.optics.configure()
        for eye, ((_, rz), (gray, z)) in enumerate(zip(body, scene)):
            native = (rz < 3490) & (abs(rz - z) < 0.002)
            face = facing[name][eye] & native
            g = np.where(face, 80, gray).astype(np.uint8)
            if name == "fixed" and robot_only:
                # Rays behind or clear of the chassis receive no changing field reference.
                # Keep actual nearer opaque objects as blockers in the pipe raster.
                g = np.where(native, g, 32).astype(np.uint8)
                # The unknown region remains masked even if a pipe crosses in front of it.
            scene[eye] = (g, z)
            if name == "roof":
                reference[eye] = (g, reference[eye][1])
            grade = (
                native & ~wheels[name][eye] & off_volume(witness.sensor.camera, eye, z)
            )
            grades.append(samples(g, grade))
    return (fixed_scene, (actual, reference), None), grades


def run(args):
    sys.path.insert(0, str(args.transit.resolve()))
    from transit_images import backgrounds

    mesh, fixed, roof = backgrounds(args.field, args.scene)
    source = witness.read(args.case / "result.json")["feed"]
    receipt = witness.read(args.case / "loaded/result.json")
    station = witness.read(args.case / "photometry-rims-core.json")["nominal"]
    contact, drive = receipt["contact"], receipt["drive"]
    case = source["case"]
    folder = args.case / "loaded"
    pipe_file = folder / contact["file"]
    route_file = folder / (case + "-600.npz")
    if (
        contact["sha256"] != witness.digest(pipe_file)
        or drive["trajectory"] != witness.digest(route_file)
        or receipt["service"]["incoming"] != source["pipeStateSha256"]
        or not station["gate"]["accepted"]
    ):
        raise ValueError("Discontinuous source evidence")
    # One cropped identity pass per head is reused only for rays on the same robot
    # material as at the dock. Unmarked or newly visible stock gets no black-face credit.
    with np.load(args.surface / (case + ".npz")) as dock:
        facing = {
            name: [dock[f"{key}{i}"] for i in range(2)]
            for name, key in (("fixed", "fixedAll"), ("roof", "allFaces"))
        }
    for i in range(2):
        mask = facing["roof"][i]
        padded = np.pad(mask, 1, constant_values=False)
        facing["roof"][i] = np.logical_and.reduce(
            [
                padded[j : j + mask.shape[0], k : k + mask.shape[1]]
                for j in range(3)
                for k in range(3)
            ]
        )
    wheels = witness.load_wheel_mask(args.wheel)
    pipe = np.load(pipe_file)
    with np.load(route_file) as tr:
        time = tr["relativeS"]
        stations = tr["stationMmMmRad"]
    results = []
    first = receipt["service"]["wallS"]
    epoch = source["javaEpochWallS"]
    for stamp in range(5474, 7500, 17):
        wall = epoch + stamp / 1000
        rel = wall - first
        if rel > pipe[-1, 0] + 1e-10:
            break
        coords = np.array([np.interp(rel, time, v) for v in stations.T])
        pose = np.array([np.interp(rel, pipe[:, 0], v) for v in pipe[:, 1:7].T])
        axis = pose[3:] / np.linalg.norm(pose[3:])
        views, grades = rendered(
            mesh,
            (fixed, roof),
            facing,
            [-coords[0], -coords[1], math.degrees(coords[2])],
            wheels,
            robot_only=args.robot_only,
        )
        masks = dict(wheels)
        if args.robot_only:
            masks["fixed"] = [
                wheel | ((rz >= 3490) | (abs(rz - z) > 0.002))
                for wheel, (_, rz), (_, z) in zip(wheels["fixed"], fixed, views[0])
            ]
        obs = run_view(
            (1000 * pose[:3], axis), case, views, grades, masks, (0.72, 4), rims=True
        )
        results.append(
            {
                "sourceJavaMs": stamp,
                "captureWallS": wall,
                "accepted": obs["accepted"],
                "fixed": obs.get("fixed"),
                "roof": obs.get("rimCheck", {}).get("status"),
                "imageSha256": obs.get("imageSha256"),
            }
        )
    stop, why = first_hold(
        {**station, "epoch": epoch}, results, first, first + drive["leaseS"]
    )
    report = {
        "case": case,
        "robotOnly": args.robot_only,
        "pipeSha256": contact["sha256"],
        "sceneSha256": witness.digest(args.scene / "scene.json"),
        "wheelSha256": witness.digest(args.wheel),
        "dockSurfaceSha256": witness.digest(args.surface / (case + ".npz")),
        "firstWallS": first,
        "stopWallS": stop,
        "reason": why,
        "captures": results,
    }
    filename = (
        "images-" + ("robot-only" if args.robot_only else "surveyed-field") + ".json"
    )
    (folder / filename).write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
    print(
        case,
        args.robot_only,
        stop,
        why,
        len(results),
        sum(r["accepted"] for r in results),
        flush=True,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("case", "transit", "scene", "field", "surface", "wheel"):
        p.add_argument(name, type=Path)
    p.add_argument("--robot-only", action="store_true")
    run(p.parse_args())
